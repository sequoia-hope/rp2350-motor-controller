#!/usr/bin/env python3
"""settle stage 2: emit every checkpoint as a complete board and gate it.

Per checkpoint (each built in its own pcbnew process — swig crashes are
intermittent and Remove() corrupts live wrappers): the dynamic originals are
stripped once from the base board (cached), the deformed chains and vias are
added back, movable parts sit at origin + D, ghosts born (G >= 1) are placed
at their pose, zones refill, and the board is saved to data/<name>/steps/.

Gate, in order:
  1. kicad-cli DRC at severity error: no violation that is not in the
     checkpoint-0 roundtrip baseline (position-independent signature);
  2. unconnected items not above the baseline;
  3. every static copper object of the base board is present, byte-for-byte
     (signature set inclusion), and every fixed footprint is where it was;
  4. net length growth within the config caps (from settle_report.json);
  5. check_sync on the final board (schematic <-> board, once).

    python3 emit.py CONFIG.json [--all | --step K] [--no-drc] [--no-sync]
Writes data/<name>/gate.json.
"""
import json, math, os, shutil, subprocess, sys
from collections import Counter
from common import (NM, HERE, HW, load_config, load_json, save_json, data_path,
                    load_board, tsig, vsig, sig_key, quiet_stderr)

cfg = load_config()
M = load_json(cfg, 'model.json')
BASE = cfg['board']
PRO = BASE.replace('.kicad_pcb', '.kicad_pro')
STEPS = data_path(cfg, 'steps')
STRIPPED = data_path(cfg, 'base_stripped.kicad_pcb')
os.makedirs(STEPS, exist_ok=True)
dyn_sigs = {e['sig'] for e in M['edges']} | {n['sig'] for n in M['nodes'] if n['kind'] == 'via'}


def nm(x):
    return int(round(x / NM))


def board_sigs(board):
    return {sig_key(tsig(t) if t.GetClass() == 'PCB_TRACK' else vsig(t)) for t in board.GetTracks()}

# ---- strip once --------------------------------------------------------------
if '--strip' in sys.argv:
    import pcbnew
    board = load_board(BASE)
    doomed = [t for t in board.GetTracks()
              if sig_key(tsig(t) if t.GetClass() == 'PCB_TRACK' else vsig(t)) in dyn_sigs]
    for t in doomed:
        board.Remove(t)
    with quiet_stderr():
        pcbnew.SaveBoard(STRIPPED, board)
    print(f'stripped {len(doomed)} of {len(dyn_sigs)} dynamic originals -> {STRIPPED}')
    sys.exit(0 if len(doomed) == len(dyn_sigs) else 3)

# ---- emit one checkpoint -----------------------------------------------------


def rdp(pts, eps=0.002):
    if len(pts) < 3:
        return pts
    ax, ay = pts[0]; bx, by = pts[-1]
    L = math.hypot(bx - ax, by - ay)
    dmax, imax = 0.0, 0
    for i in range(1, len(pts) - 1):
        px, py = pts[i]
        d = math.hypot(px - ax, py - ay) if L < 1e-9 else abs((bx - ax) * (ay - py) - (ax - px) * (by - ay)) / L
        if d > dmax:
            dmax, imax = d, i
    if dmax > eps:
        return rdp(pts[:imax + 1], eps)[:-1] + rdp(pts[imax:], eps)
    return [pts[0], pts[-1]]


if '--emit-step' in sys.argv:
    import pcbnew
    k = int(sys.argv[sys.argv.index('--emit-step') + 1])
    TR = load_json(cfg, 'traj.json')
    ck = TR['checkpoints'][k]
    P, D, G, T = ck['P'], ck['D'], ck['G'], ck['T']
    ck_chains = ck.get('chains') or TR['chains']
    board = load_board(STRIPPED)
    nets = board.GetNetsByName()

    def find_net(name):
        n = nets.find(name)
        return None if n == nets.end() else n.value()[1]

    def to_v(x, y):
        return pcbnew.VECTOR2I(nm(x), nm(y))

    added = no_net = 0
    for e, ids in zip(M['edges'], ck_chains):
        net = find_net(e['net'])
        if net is None:
            no_net += 1
            continue
        pts = rdp([tuple(P[i]) for i in ids])
        for j in range(len(pts) - 1):
            (x1, y1), (x2, y2) = pts[j], pts[j + 1]
            if math.hypot(x2 - x1, y2 - y1) < 0.0005:
                continue
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(to_v(x1, y1)); t.SetEnd(to_v(x2, y2))
            t.SetWidth(nm(e['w'])); t.SetLayer(e['layer']); t.SetNet(net)
            board.Add(t)
            added += 1
    for nid, n in enumerate(M['nodes']):
        if n['kind'] != 'via':
            continue
        net = find_net(n['net'])
        if net is None:
            no_net += 1
            continue
        via = pcbnew.PCB_VIA(board)
        via.SetPosition(to_v(P[nid][0], P[nid][1]))
        via.SetDrill(nm(n['drill']))
        try:
            via.SetWidth(nm(n['dia']))
        except TypeError:
            via.SetWidth(pcbnew.PADSTACK.ALL_LAYERS, nm(n['dia']))
        via.SetViaType(pcbnew.VIATYPE_THROUGH)
        via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        via.SetNet(net)
        board.Add(via)
        added += 1

    fps = {f.GetReference(): f for f in board.GetFootprints()}
    n_moved = 0
    for r, (dx, dy) in D.items():
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            continue
        f = fps[r]
        p0 = M['parts'][r]
        f.SetPosition(pcbnew.VECTOR2I(nm(p0['x'] + dx), nm(p0['y'] + dy)))
        n_moved += 1
    n_born = 0
    for gi, gh in enumerate(M['ghosts']):
        if G[gi] < 1.0 - 1e-9:
            continue
        f = fps[gh['ref']]
        if (gh['side'] == 'B') != f.IsFlipped():
            f.Flip(f.GetPosition(), False)
        f.SetOrientationDegrees(gh['rot'])
        f.SetPosition(pcbnew.VECTOR2I(nm(gh['origin'][0] + T[gi][0]), nm(gh['origin'][1] + T[gi][1])))
        n_born += 1
    with quiet_stderr():
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        out = os.path.join(STEPS, f'step_{k:02d}.kicad_pcb')
        pcbnew.SaveBoard(out, board)
    shutil.copy(PRO, out.replace('.kicad_pcb', '.kicad_pro'))
    print(f'step {k}: s={ck["s"]:.3f} {n_moved} parts moved, {n_born} ghosts born, '
          f'{added} dynamic objects ({no_net} net misses) -> {out}')
    sys.exit(0)

# ---- driver ------------------------------------------------------------------
if not os.path.exists(STRIPPED) or os.path.getmtime(STRIPPED) < os.path.getmtime(data_path(cfg, 'model.json')):
    r = subprocess.run([sys.executable, os.path.abspath(__file__), cfg['_path'], '--strip'],
                       capture_output=True, text=True)
    tail = [l for l in r.stdout.splitlines() if 'stripped' in l]
    print(tail[-1] if tail else r.stdout[-300:], flush=True)
    if r.returncode != 0:
        sys.exit(f'strip failed (rc {r.returncode}):\n{r.stderr[-600:]}')

TR = load_json(cfg, 'traj.json')
N = len(TR['checkpoints'])
steps = range(N) if '--step' not in sys.argv else [int(sys.argv[sys.argv.index('--step') + 1])]
if '--step' in sys.argv:
    steps = [0] + [s for s in steps if s != 0]      # baseline always first


def drc(path, out):
    subprocess.run(['kicad-cli', 'pcb', 'drc', '--severity-error', '--format', 'json', '-o', out, path],
                   check=True, capture_output=True)
    d = json.load(open(out))
    return d.get('unconnected_items', []), d.get('violations', [])


def sig(v):
    """Position-independent for footprint pairs (a pre-existing courtyard
    overlap that rides along with a moved part is not new), position-keyed
    for copper."""
    keys = []
    for i in v['items']:
        if i['description'].startswith('Footprint '):
            keys.append(i['description'])
        elif 'pos' in i:
            keys.append((round(i['pos']['x'], 2), round(i['pos']['y'], 2)))
    return (v['type'], tuple(sorted(map(str, keys))))


# static copper + fixed footprints of the base, for gate 3
import pcbnew  # noqa: E402
base_board = load_board(BASE)
base_static = board_sigs(base_board) - dyn_sigs
movable = {r for r, p in M['parts'].items() if p['movable']}
ghost_refs = {gh['ref'] for gh in M['ghosts']}
fixed_pos = {f.GetReference(): (f.GetPosition().x, f.GetPosition().y, f.GetOrientationDegrees())
             for f in base_board.GetFootprints()
             if f.GetReference() not in movable and f.GetReference() not in ghost_refs}
del base_board

rep = load_json(cfg, 'settle_report.json') if os.path.exists(data_path(cfg, 'settle_report.json')) else {}
over_cap = [r['net'] for r in rep.get('lengths', []) if r.get('at_cap')]

base_sigs = None
report = []
for k in steps:
    for attempt in range(3):
        r = subprocess.run([sys.executable, os.path.abspath(__file__), cfg['_path'], '--emit-step', str(k)],
                           capture_output=True, text=True)
        if r.returncode == 0:
            break
        print(f'  emit-step {k} attempt {attempt + 1} failed (rc {r.returncode}), retrying', flush=True)
    else:
        sys.exit(f'emit-step {k} failed 3x:\n{r.stderr[-800:]}')
    out = [l for l in r.stdout.splitlines() if l.startswith('step ')]
    print(out[-1] if out else r.stdout.strip()[-200:], flush=True)
    path = os.path.join(STEPS, f'step_{k:02d}.kicad_pcb')
    entry = dict(step=k, s=round(TR['checkpoints'][k]['s'], 3), tag=TR['checkpoints'][k].get('tag', ''))
    if '--no-drc' not in sys.argv:
        u, viol = drc(path, os.path.join(STEPS, f'drc_{k:02d}.json'))
        if base_sigs is None:
            base_sigs = {sig(v) for v in viol}
            base_v, base_unc = len(viol), len(u)
            print(f'  baseline (s=0 roundtrip): {base_v} violations, {base_unc} unconnected', flush=True)
            new = []
        else:
            new = [v for v in viol if sig(v) not in base_sigs]
        entry.update(unconnected=len(u), new_violations=len(new),
                     new_desc=[f'{v["type"]}: {v["description"][:90]}' for v in new[:8]])
        entry['drc_ok'] = not new and len(u) <= base_unc
    # gate 3: static copper untouched, fixed parts unmoved
    b = load_board(path)
    missing = base_static - board_sigs(b)
    moved_fixed = [f.GetReference() for f in b.GetFootprints()
                   if f.GetReference() in fixed_pos and
                   (f.GetPosition().x, f.GetPosition().y, f.GetOrientationDegrees()) != fixed_pos[f.GetReference()]]
    del b
    entry.update(static_missing=len(missing), fixed_moved=moved_fixed[:10], static_ok=not missing and not moved_fixed)
    entry['ok'] = entry.get('drc_ok', True) and entry['static_ok']
    report.append(entry)
    print(f'  step {k:2d} s={entry["s"]:.3f}: ' +
          (f'{entry["unconnected"]} unconnected, {entry["new_violations"]} new violations, ' if 'unconnected' in entry else '') +
          f'{len(missing)} static objects missing, {len(moved_fixed)} fixed parts moved '
          f'{"OK" if entry["ok"] else "<-- GATE FAIL"}', flush=True)
    for d in entry.get('new_desc', []):
        print(f'      {d}', flush=True)

# gate 4: length caps
print(f'  length caps: {len(over_cap)} nets at cap' + (f' ({", ".join(over_cap[:8])})' if over_cap else ''))

# gate 5: check_sync on the final board
sync = None
if '--no-sync' not in sys.argv and steps and (N - 1) in list(steps):
    final = os.path.join(STEPS, f'step_{N - 1:02d}.kicad_pcb')
    cs = os.path.join(HW, 'tools', 'revb', 'check_sync.py')
    if os.path.exists(cs):
        r = subprocess.run([sys.executable, cs, '--pcb', final], capture_output=True, text=True)
        sync = r.returncode == 0
        tail = r.stdout.strip().splitlines()[-1] if r.stdout.strip() else r.stderr[-200:]
        print(f'  check_sync: {"IN SYNC" if sync else "DESYNC"} — {tail}')

good = sum(1 for e in report if e['ok'])
print(f'\nsettle trajectory: {good}/{len(report)} checkpoints pass the gate')
save_json(cfg, 'gate.json', dict(steps=report, baseline_violations=base_v if base_sigs is not None else None,
                                 over_cap=over_cap, check_sync=sync))
print(f'boards in {STEPS}/, gate.json written')
