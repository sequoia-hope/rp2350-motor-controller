#!/usr/bin/env python3
"""jiggle2 stage 7: incremental nudge. Start from the shipped board and add
recovered copper in small batches; a batch is accepted only if the work copy
still shows ZERO new DRC violations vs the shipped baseline and airwires did
not increase. Offenders inside a failing batch are withdrawn by violation
attribution and the remainder retried, so one bad chain cannot poison its
batchmates. Every accepted checkpoint is a valid board — the process can stop
after any batch and hand over rp2350_driver_jiggle2*.kicad_pcb as-is.

This inverts drcloop.py: additive and monotone instead of emit-everything
then withdraw, so no intermediate state is ever worse than the shipped board.
Candidates are solution entries not yet accepted/withdrawn (a later reroute.py
pass appends more; re-running nudge.py picks up exactly the new ones).
"""
import json, math, subprocess, sys, os
from common import DATA, BOARD_BASE, BOARD_OUT, VARIANT, load_json, save_json

K = int(sys.argv[sys.argv.index('--batch') + 1]) if '--batch' in sys.argv else 24
NEAR = 0.6      # mm: violation-to-copper attribution radius (as drcloop.py)
HERE = os.path.dirname(os.path.abspath(__file__))


def vsig(v):
    pts = tuple(sorted((round(i['pos']['x'], 2), round(i['pos']['y'], 2))
                       for i in v['items'] if 'pos' in i))
    return (v['type'], pts)


def pt_polyline_dist(x, y, pts):
    best = 1e9
    for k in range(len(pts) - 1):
        (ax, ay), (bx, by) = pts[k], pts[k + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        best = min(best, math.hypot(x - ax - t * dx, y - ay - t * dy))
    return best


def drc(path, out):
    subprocess.run(['kicad-cli', 'pcb', 'drc', '--severity-error',
                    '--format', 'json', '-o', out, path],
                   check=True, capture_output=True)
    with open(out) as f:
        d = json.load(f)
    return len(d.get('unconnected_items', [])), d.get('violations', [])


def emit_sel():
    r = subprocess.run([sys.executable, os.path.join(HERE, 'emit.py'),
                        '--sel', '--no-drc'], capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit('emit failed:\n' + r.stderr[-800:])


sol = load_json('solution.json')
E, V = sol['edges'], sol['vias']

base_u, base_viol = drc(BOARD_BASE, f'{DATA}/drc_nudge_baseline.json')
base_sigs = {vsig(v) for v in base_viol}
print(f'shipped baseline ({VARIANT or "full"} variant): '
      f'{len(base_viol)} violations, {base_u} airwires')

# candidates: not yet decided
cand = [('v', i) for i, o in enumerate(V) if not o.get('accepted') and not o.get('withdrawn')]
cand += [('e', i) for i, o in enumerate(E) if not o.get('accepted') and not o.get('withdrawn')]
obj = lambda c: V[c[1]] if c[0] == 'v' else E[c[1]]

# Batch whole connected components: a chain whose neighbours land in a LATER
# batch would be a floating island, read as an airwire regression, and get an
# innocent batch wholesale-rejected. Union-find on rounded endpoint keys.
def keys(c):
    o = obj(c)
    if c[0] == 'v':
        return [(round(o['x'], 2), round(o['y'], 2), o['net'])]
    return [(round(o['pts'][0][0], 2), round(o['pts'][0][1], 2), o['net']),
            (round(o['pts'][-1][0], 2), round(o['pts'][-1][1], 2), o['net'])]

parent = {}
def find(x):
    while parent.setdefault(x, x) != x:
        parent[x] = parent[parent[x]]
        x = parent[x]
    return x
bykey = {}
for c in cand:
    for k in keys(c):
        if k in bykey:
            parent[find(c)] = find(bykey[k])
        else:
            bykey[k] = c
comps = {}
for c in cand:
    comps.setdefault(find(c), []).append(c)
comps = sorted(comps.values(),
               key=lambda ms: (0 if all(obj(c).get('clean') for c in ms) else 1, len(ms)))

# greedy-pack whole components into batches of ~K (a big component stays whole)
batches = []
cur = []
for ms in comps:
    if cur and sum(len(m) for m in cur) + len(ms) > K:
        batches.append(cur)
        cur = []
    cur.append(ms)
if cur:
    batches.append(cur)
print(f'candidates: {len(cand)} '
      f'({sum(1 for c in cand if c[0] == "e")} chains, '
      f'{sum(1 for c in cand if c[0] == "v")} vias) in {len(comps)} components, '
      f'{len(batches)} batches (target size {K})')

RETRY_CAP = 5    # attribution retries per component before giving up on it
cur_u = base_u
n_acc = n_wd = n_drc = 0
traj = []
deferred = []    # components that regressed airwires: stub-awakening, retry at end
import re


def trial(members):
    global n_drc
    for o in E + V:
        o['sel'] = bool(o.get('accepted'))
    for c in members:
        obj(c)['sel'] = True
    save_json('solution.json', sol)
    emit_sel()
    u, viol = drc(BOARD_OUT, f'{DATA}/drc_nudge.json')
    n_drc += 1
    return [v for v in viol if vsig(v) not in base_sigs], u


def member_dist(c, x, y):
    o = obj(c)
    if c[0] == 'v':
        return math.hypot(x - o['x'], y - o['y']) - o['dia'] / 2
    return pt_polyline_dist(x, y, o['pts']) - o['w'] / 2


def attribute(new, members):
    """One most-implicated member per violation: proximity first, then the
    nearest member of an implicated net (a touched dormant stub re-nets and
    can violate at its far end, away from all members)."""
    hit = set()
    for v in new:
        best = None
        for it in v['items']:
            if 'pos' not in it:
                continue
            x, y = it['pos']['x'], it['pos']['y']
            for c in members:
                d = member_dist(c, x, y)
                if d < NEAR and (best is None or d < best[0]):
                    best = (d, c)
        if best is None:
            vnets = {m for it in v['items']
                     for m in re.findall(r'\[([^\]]+)\]', it.get('description', ''))}
            for it in v['items']:
                if 'pos' not in it:
                    continue
                x, y = it['pos']['x'], it['pos']['y']
                for c in members:
                    if obj(c)['net'] not in vnets:
                        continue
                    d = member_dist(c, x, y)
                    if best is None or d < best[0]:
                        best = (d, c)
        if best:
            hit.add(best[1])
    return hit


def accept(members, u):
    global n_acc, cur_u
    for c in members:
        obj(c)['accepted'] = True
    n_acc += len(members)
    cur_u = u


def withdraw(members):
    global n_wd
    for c in members:
        obj(c)['withdrawn'] = True
        obj(c)['sel'] = False
    n_wd += len(members)


# "Not breaking anything" = never worse than the SHIPPED board: zero new
# violations and airwires <= baseline. Airwires are not required to fall
# monotonically — connecting a dormant stub re-nets it and KiCad honestly
# starts counting its ratsnest, so good copper can raise the count until a
# later component closes that stub's route. Such components are deferred and
# retried once at the end instead of being withdrawn.
def run_component(ms, defer_ok=True):
    members = list(ms)
    for _ in range(RETRY_CAP):
        if not members:
            return
        new, u = trial(members)
        if not new and u <= base_u:
            accept(members, u)
            return
        if new:
            hit = attribute(new, members) or set(members)
            withdraw(hit)
            members = [c for c in members if c not in hit]
        elif defer_ok:                         # airwire regression: stub woke up
            print(f'  deferred component ({len(members)} members, '
                  f'airwires {u} > {base_u})', flush=True)
            deferred.append(members)
            return
        else:
            withdraw(members)
            return
    if members:
        print(f'  component gave up after {RETRY_CAP} retries '
              f'({len(members)} members left)', flush=True)
        withdraw(members)


for bi, comps_in_batch in enumerate(batches):
    flat = [c for ms in comps_in_batch for c in ms]
    new, u = trial(flat)
    if not new and u <= base_u:
        accept(flat, u)                        # fast path: 1 DRC per clean batch
    else:
        print(f'  batch contested ({len(new)} new viol, airwires {u}); '
              f'trying {len(comps_in_batch)} components separately', flush=True)
        for ms in comps_in_batch:
            run_component(ms)
    traj.append((n_acc, cur_u))
    print(f'batch {bi + 1}/{len(batches)}: accepted {n_acc}, withdrawn {n_wd}, '
          f'airwires {cur_u} (baseline {base_u})', flush=True)

if deferred:
    print(f'\nretrying {len(deferred)} deferred components '
          f'(their stub-mates may have landed since)', flush=True)
    pending, deferred = deferred, []
    for ms in pending:
        run_component(ms, defer_ok=False)
    print(f'after deferred pass: accepted {n_acc}, withdrawn {n_wd}, '
          f'airwires {cur_u}', flush=True)

# leave the board at the accepted state and record the final referee numbers
for o in E + V:
    o['sel'] = bool(o.get('accepted'))
save_json('solution.json', sol)
emit_sel()
u, viol = drc(BOARD_OUT, f'{DATA}/drc_nudge.json')
new = [v for v in viol if vsig(v) not in base_sigs]
print(f'\nfinal: {n_acc} accepted, {n_wd} withdrawn, {n_drc + 2} DRC runs')
print(f'  violations {len(viol)} ({len(new)} new vs baseline {len(base_viol)}) — '
      f'{"CLEAN" if not new else "NOT CLEAN"}')
print(f'  airwires {u} (baseline {base_u})')
save_json('nudge.json', dict(baseline_viol=len(base_viol), baseline_air=base_u,
                             accepted=n_acc, withdrawn=n_wd,
                             final_viol=len(viol), final_new=len(new),
                             final_air=u, trajectory=traj))
