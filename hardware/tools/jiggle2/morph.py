#!/usr/bin/env python3
"""jiggle2 morph: the rework without the rip.

Start from the SHIPPED board (board_prerip_2238aca — fully routed, zero
airwires) and morph it to the rev-B placement in small nudges. Each checkpoint
recorded by `solve.py --snapshots N` is emitted as a complete board: the
dragged copper is removed and re-added at its deformed positions (a drag in
file terms — never a rip), the 49 moved footprints sit at exactly the same
smoothstep progress `s` as the copper anchors, zones are refilled, and real
kicad DRC gates the state. The invariant per checkpoint: zero unconnected
items (the board never stops being fully routed) and no new violations vs the
checkpoint-0 roundtrip baseline.

The 8 bridge edges the solver drops (copper that rev-B's net splits will cut
at the termination switches) are still dragged here as straight segments —
under the shipped netlist they are legitimate single-net copper. New parts and
net splits are deliberately absent: they are the follow-up routing phase, the
only place airwires can honestly appear.

Usage:  solve.py --snapshots 12   (writes data/morph.json)
        morph.py                  (emits + gates every checkpoint)
        morph.py --emit-step K    (internal: build one checkpoint board)
"""
import json, math, os, shutil, subprocess, sys
from collections import Counter
from common import (NM, HW, BOARD_PRE, BOARD_CUR, DATA, load_json, save_json,
                    load_board, quiet_stderr, replay_drop)

MORPH_BOARD = os.path.join(HW, 'rp2350_driver_morph.kicad_pcb')
MORPH_PRO = os.path.join(HW, 'rp2350_driver_morph.kicad_pro')
ARCHIVE = os.path.join(DATA, 'morph')
STRIPPED = os.path.join(ARCHIVE, 'base_stripped.kicad_pcb')

G = load_json('graph.json')
O = load_json('obstacles.json')
M = load_json('morph.json')
nodes = G['nodes']
kept, drop = replay_drop(G, O)
assert len(M['chains']) == len(kept), 'morph.json chains misaligned with replay'
via_nids = [i for i, n in enumerate(nodes) if n['kind'] == 'via']


def nm(x):
    return int(round(x / NM))


def orig_sigs():
    """Signatures of every original dragged object on the shipped board."""
    sigs = set()
    for e in G['edges']:
        a = (nm(nodes[e['a']]['x']), nm(nodes[e['a']]['y']))
        b = (nm(nodes[e['b']]['x']), nm(nodes[e['b']]['y']))
        if b < a:
            a, b = b, a
        sigs.add(('T', e['layer'], a, b, nm(e['w'])))
    for nid in via_nids:
        n = nodes[nid]
        sigs.add(('V', nm(n['x']), nm(n['y']), nm(n['drill'])))
    return sigs


# ---- strip once: remove the dragged originals in a minimal process ----------
# (pcbnew Remove() corrupts live swig wrappers if the process does anything
# else afterwards — same lesson as graft.py's nonew variant. Load, remove,
# save, exit. The removal set is checkpoint-independent, so one stripped base
# serves the whole trajectory.)
if '--strip' in sys.argv:
    import pcbnew
    from common import tsig, vsig
    board = load_board(BOARD_PRE)
    sigs = orig_sigs()
    doomed = [t for t in board.GetTracks()
              if (tsig(t) if t.GetClass() == 'PCB_TRACK' else vsig(t)) in sigs]
    for t in doomed:
        board.Remove(t)
    with quiet_stderr():
        pcbnew.SaveBoard(STRIPPED, board)
    exp = len(G['edges']) + len(via_nids)
    print(f'stripped {len(doomed)} of {exp} dragged originals -> {STRIPPED}')
    sys.exit(0 if len(doomed) == exp else 3)

# ---- emit one checkpoint board (purely additive on the stripped base) -------
if '--emit-step' in sys.argv:
    import pcbnew
    k = int(sys.argv[sys.argv.index('--emit-step') + 1])
    ck = M['checkpoints'][k]
    s, P = ck['s'], ck['P']
    sref = ck.get('sref') or {}       # per-part transit clock (convoy schedule)
    ck_chains = ck.get('chains') or M['chains']   # per-checkpoint topology
    assert len(ck_chains) == len(kept), 'checkpoint chains misaligned'
    board = load_board(STRIPPED)
    cur = load_board(BOARD_CUR)
    removed = 'pre-stripped'

    nets = board.GetNetsByName()
    def find_net(name):
        n = nets.find(name)
        return None if n == nets.end() else n.value()[1]

    def to_v(x, y):
        return pcbnew.VECTOR2I(nm(x), nm(y))

    def add_seg(x1, y1, x2, y2, w, lay, net):
        if math.hypot(x2 - x1, y2 - y1) < 0.001:
            return 0
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(to_v(x1, y1)); t.SetEnd(to_v(x2, y2))
        t.SetWidth(nm(w)); t.SetLayer(lay); t.SetNet(net)
        board.Add(t)
        return 1

    added = no_net = 0
    for (ei, e), ids in zip(kept, ck_chains):
        net = find_net(G['edges'][ei]['net'])     # shipped netlist names
        if net is None:
            no_net += 1
            continue
        pts = [P[i] for i in ids]
        for j in range(len(pts) - 1):
            added += add_seg(pts[j][0], pts[j][1], pts[j+1][0], pts[j+1][1],
                             e['w'], e['layer'], net)
    for ei in sorted(drop):                        # bridge edges: still one net here
        e = G['edges'][ei]
        net = find_net(e['net'])
        if net is None:
            no_net += 1
            continue
        a, b = P[e['a']], P[e['b']]
        added += add_seg(a[0], a[1], b[0], b[1], e['w'], e['layer'], net)
    for nid in via_nids:
        n = nodes[nid]
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

    # moved parts ride the same smoothstep progress as the copper anchors
    pf = {f.GetReference(): f for f in board.GetFootprints()}
    n_moved = 0
    for f in cur.GetFootprints():
        r = f.GetReference()
        if r not in pf:
            continue                               # rev-B new part: not yet born
        dp, dc = pf[r].GetPosition(), f.GetPosition()
        if dp.x == dc.x and dp.y == dc.y:
            continue
        u = sref.get(r, s)
        pf[r].SetPosition(pcbnew.VECTOR2I(int(round(dp.x + (dc.x - dp.x) * u)),
                                          int(round(dp.y + (dc.y - dp.y) * u))))
        n_moved += 1

    with quiet_stderr():
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        pcbnew.SaveBoard(MORPH_BOARD, board)
    print(f'step {k}: s={s:.3f} moved {n_moved} parts, dragged {added} objects '
          f'({no_net} net misses)')
    sys.exit(0)

# ---- driver: emit + DRC-gate the whole trajectory ---------------------------
if not os.path.exists(MORPH_PRO):
    shutil.copy(os.path.join(HW, 'rp2350_driver.kicad_pro'), MORPH_PRO)
os.makedirs(ARCHIVE, exist_ok=True)
if not os.path.exists(STRIPPED):
    r = subprocess.run([sys.executable, os.path.abspath(__file__), '--strip'],
                       capture_output=True, text=True)
    tail = [l for l in r.stdout.splitlines() if 'stripped' in l]
    print(tail[-1] if tail else r.stdout[-200:], flush=True)
    if r.returncode != 0:
        sys.exit(f'strip failed (rc {r.returncode}):\n{r.stderr[-600:]}')


def drc(path, out):
    subprocess.run(['kicad-cli', 'pcb', 'drc', '--severity-error',
                    '--format', 'json', '-o', out, path],
                   check=True, capture_output=True)
    d = json.load(open(out))
    return d.get('unconnected_items', []), d.get('violations', [])


def sig(v):
    pts = tuple(sorted((round(i['pos']['x'], 2), round(i['pos']['y'], 2))
                       for i in v['items'] if 'pos' in i))
    return (v['type'], pts)


base_sigs = None
report = []
N = len(M['checkpoints'])
for k in range(N):
    for attempt in range(3):       # pcbnew swig crashes are intermittent
        r = subprocess.run([sys.executable, os.path.abspath(__file__),
                            '--emit-step', str(k)], capture_output=True, text=True)
        if r.returncode == 0:
            break
        print(f'  emit-step {k} attempt {attempt + 1} failed (rc {r.returncode}), '
              f'retrying', flush=True)
    else:
        sys.exit(f'emit-step {k} failed 3x:\n{r.stderr[-800:]}')
    out = [l for l in r.stdout.splitlines() if l.startswith('step ')]
    print(out[-1] if out else r.stdout.strip()[-200:], flush=True)
    u, viol = drc(MORPH_BOARD, f'{DATA}/drc_morph_{k}.json')
    shutil.copy(MORPH_BOARD, os.path.join(ARCHIVE, f'step_{k:02d}.kicad_pcb'))
    if base_sigs is None:                          # checkpoint 0 = roundtrip baseline
        base_sigs = {sig(v) for v in viol}
        base_v = len(viol)
        base_unc = len(u)
        print(f'  baseline (s=0 roundtrip): {base_v} violations, '
              f'{base_unc} unconnected', flush=True)
        new = []
    else:
        new = [v for v in viol if sig(v) not in base_sigs]
    s = M['checkpoints'][k]['s']
    ok = not new and len(u) <= base_unc
    report.append(dict(step=k, s=round(s, 3), unconnected=len(u),
                       new_violations=len(new), ok=ok))
    print(f'  step {k:2d} s={s:.3f}: {len(u)} unconnected, '
          f'{len(new)} new violations vs baseline '
          f'{"OK" if ok else "<-- GATE FAIL"}', flush=True)
    if new:
        for v in new[:5]:
            print(f'      {v["type"]}: {v["description"][:80]}', flush=True)

good = sum(1 for r in report if r['ok'])
print(f'\nmorph trajectory: {good}/{N} checkpoints fully routed and '
      f'DRC-clean vs the shipped baseline')
save_json('morph_report.json', dict(baseline_violations=base_v, steps=report))
print(f'boards archived in {ARCHIVE}/, final state {MORPH_BOARD}')
