#!/usr/bin/env python3
"""jiggle2 stage 5: homotopy invariant check (Cabello/Liu/Mantler/Snoeyink,
SoCG 2002). For every solved chain, build the closed loop

    old segment -> (old_b -> new_b drag) -> reversed new polyline -> (new_a -> old_a drag)

and compute its crossing word against a system of disjoint vertical rays, one
per puncture. Punctures are *pinned* obstacles only — other-net pads of parts
that did not move between the pre-rip and base boards, plus kept vias — since
homotopy rel a moving obstacle is undefined. Reduced word empty <=> the field
deformed the trace without ever pulling it across an obstacle: shape changed,
topology kept. Bound endpoints lerp on straight lines in solve.py, so the
straight connectors are the exact endpoint trajectories for anchored nodes.

Reads data/graph.json + data/obstacles.json + data/solution.json, writes
data/homotopy.json. Pure check; never touches boards or the solution.
"""
import math, sys
from collections import defaultdict
from common import NM, BOARD_PRE, BOARD_BASE, VARIANT, load_board, load_json, save_json

G = load_json('graph.json')
O = load_json('obstacles.json')
sol = load_json('solution.json')
nodes, edges = G['nodes'], G['edges']
F_CU, B_CU = G['F'], G['B']
COPPER = sorted({e['layer'] for e in edges} | {t['layer'] for t in O['tracks']}
                | {F_CU, B_CU})

# ---- replay solve.py's deterministic bridge-edge drop to align indices -------
adj = defaultdict(list)
for ei, e in enumerate(edges):
    adj[e['a']].append(ei); adj[e['b']].append(ei)

comp_of = {}
comps = []
for start in range(len(nodes)):
    if start in comp_of:
        continue
    stack, comp = [start], []
    comp_of[start] = len(comps)
    while stack:
        n = stack.pop()
        comp.append(n)
        for ei in adj[n]:
            for m in (edges[ei]['a'], edges[ei]['b']):
                if m not in comp_of:
                    comp_of[m] = len(comps)
                    stack.append(m)
    comps.append(comp)

cur_pad_net = {(p['ref'], p['num']): p['net'] for p in O['pads']}
drop = set()
for comp in comps:
    votes = defaultdict(int)
    for n in comp:
        b = nodes[n]['bind']
        if b and cur_pad_net.get(tuple(b)):
            votes[cur_pad_net[tuple(b)]] += 1
    if len(votes) <= 1:
        continue
    from heapq import heappush, heappop
    dist = {}
    pq = []
    for n in comp:
        b = nodes[n]['bind']
        if b and cur_pad_net.get(tuple(b)):
            dist[n] = (0.0, cur_pad_net[tuple(b)])
            heappush(pq, (0.0, n))
    while pq:
        dn, n = heappop(pq)
        if dist[n][0] < dn:
            continue
        for ei in adj[n]:
            m = edges[ei]['b'] if edges[ei]['a'] == n else edges[ei]['a']
            L = math.hypot(nodes[m]['x']-nodes[n]['x'], nodes[m]['y']-nodes[n]['y'])
            if m not in dist or dist[m][0] > dn + L:
                dist[m] = (dn + L, dist[n][1])
                heappush(pq, (dn + L, m))
    nnet = {n: dist.get(n, (0, ''))[1] for n in comp}
    for n in comp:
        for ei in adj[n]:
            e = edges[ei]
            na, nb = nnet[e['a']], nnet[e['b']]
            if na and nb and na != nb:
                drop.add(ei)

kept = [(ei, e) for ei, e in enumerate(edges) if ei not in drop]
if len(kept) != len(sol['edges']):
    sys.exit(f'alignment failure: replay kept {len(kept)} edges, '
             f'solution has {len(sol["edges"])} — drop replay diverged')
for (ei, e), se in zip(kept, sol['edges']):
    if e['layer'] != se['layer'] or abs(e['w'] - se['w']) > 1e-9:
        sys.exit(f'alignment failure at graph edge {ei}: '
                 f'({e["layer"]},{e["w"]}) vs ({se["layer"]},{se["w"]})')
print(f'alignment: {len(kept)} chains map 1:1 onto solution ({len(drop)} dropped bridges)')

# ---- pinned parts: zero delta between pre-rip and base boards ----------------
pre = load_board(BOARD_PRE)
base = load_board(BOARD_BASE)
LNAME = {lay: base.GetLayerName(lay) for lay in COPPER}
ppos = {f.GetReference(): f.GetPosition() for f in pre.GetFootprints()}
pinned, moved, appeared = set(), set(), set()
for f in base.GetFootprints():
    r = f.GetReference()
    if r not in ppos:
        appeared.add(r)
        continue
    p = f.GetPosition()
    d = math.hypot((p.x - ppos[r].x) * NM, (p.y - ppos[r].y) * NM)
    (pinned if d < 0.005 else moved).add(r)
print(f'parts: {len(pinned)} pinned, {len(moved)} moved, {len(appeared)} rev-B new '
      f'(moved+new pads excluded from punctures)')

# ---- puncture system: one vertical ray per pinned other-net obstacle ---------
# Jitter x per puncture so rays are pairwise disjoint and miss the nm grid.
# Drilled pads and vias block every copper layer; SMD pads only their own.
punct = []   # (x_jit, y, layers, net, label)
for p in O['pads']:
    if p['ref'] not in pinned:
        continue
    lays = COPPER if (p['drill'] or len(p['layers']) > 1) else p['layers']
    punct.append((p['x'], p['y'], frozenset(lays), p['net'],
                  f"{p['ref']}.{p['num']}"))
for v in O['vias']:
    punct.append((v['x'], v['y'], frozenset(COPPER), v['net'],
                  f"via@{v['x']:.2f},{v['y']:.2f}"))
punct = [(x + 2.19e-8 + i * 1.37e-7, y, lays, net, lab)
         for i, (x, y, lays, net, lab) in enumerate(punct)]
by_layer = {lay: [(i, p) for i, p in enumerate(punct) if lay in p[2]]
            for lay in COPPER}
print('punctures:', len(punct), 'total,',
      ', '.join(f'{LNAME[lay]} {len(by_layer[lay])}' for lay in COPPER))

def crossing_word(loop, cands):
    """Reduced word of the closed polyline against downward vertical rays."""
    letters = []
    n = len(loop)
    for k in range(n):
        x1, y1 = loop[k]
        x2, y2 = loop[(k + 1) % n]
        if x1 == x2:
            continue
        hits = []
        for pi, (px, py, _, _, _) in cands:
            if (x1 < px) != (x2 < px):
                t = (px - x1) / (x2 - x1)
                if y1 + (y2 - y1) * t > py:
                    hits.append((t, pi + 1 if x2 > x1 else -(pi + 1)))
        hits.sort()
        letters += [l for _, l in hits]
    st = []
    for l in letters:
        if st and st[-1] == -l:
            st.pop()
        else:
            st.append(l)
    return st

if '--selftest' in sys.argv:
    T = (5.0 + 2.19e-8, 5.0, frozenset(COPPER), 'X', 'T.1')
    C = [(0, T)]
    assert crossing_word([(4, 4), (6, 4), (6, 6), (4, 6)], C), \
        'loop encircling the puncture must give a nonempty word'
    assert not crossing_word([(7, 4), (9, 4), (9, 6), (7, 6)], C), \
        'disjoint loop must reduce to empty'
    same = [(0, 7), (10, 7)] + [(0, 7), (5, 9), (10, 7)][::-1]
    assert not crossing_word(same, C), 'same-side deformation must reduce to empty'
    flip = [(0, 7), (10, 7)] + [(0, 7), (4, 1), (10, 7)][::-1]
    assert crossing_word(flip, C) == [1], 'side switch must survive reduction'
    print('selftest: 4/4 crossing-word cases pass')

def pt_seg_dist(px, py, a, b):
    (ax, ay), (bx, by) = a, b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)

# ---- check every chain -------------------------------------------------------
all_mode = any(o.get('withdrawn') for o in sol['edges'] + sol['vias'])
results = []
defl = []
n_ok = n_bad = 0
for (ei, e), se in zip(kept, sol['edges']):
    a, b = nodes[e['a']], nodes[e['b']]
    old = [(a['x'], a['y']), (b['x'], b['y'])]
    new = [tuple(q) for q in se['pts']]
    defl.append(max(pt_seg_dist(q[0], q[1], old[0], old[1]) for q in new))
    loop = old + new[::-1]
    lay = e['layer']
    minx = min(q[0] for q in loop); maxx = max(q[0] for q in loop)
    maxy = max(q[1] for q in loop)
    net = se['net']
    cands = [(i, p) for i, p in by_layer[lay]
             if p[3] != net and minx <= p[0] <= maxx and p[1] <= maxy]
    word = crossing_word(loop, cands) if cands else []
    emitted = not se.get('withdrawn') and (se['clean'] or all_mode)
    if word:
        n_bad += 1
        results.append(dict(
            edge=ei, net=net, layer=LNAME.get(lay, str(lay)),
            old=[list(q) for q in old], word_len=len(word),
            crossed=sorted({punct[abs(l) - 1][4] for l in word}),
            clean=se['clean'], withdrawn=bool(se.get('withdrawn')),
            emitted=emitted))
    else:
        n_ok += 1

emitted_bad = [r for r in results if r['emitted']]
withheld_bad = [r for r in results if not r['emitted']]
ds = sorted(defl)
print(f'\nhomotopy check ({VARIANT or "full"} variant, emit mode '
      f'{"--all+referee" if all_mode else "clean-only"}):')
print(f'  deflection from pre-rip segment: max {ds[-1]:.2f}mm, '
      f'p95 {ds[int(len(ds) * 0.95)]:.2f}mm, moved >0.1mm: '
      f'{sum(1 for d in ds if d > 0.1)}/{len(ds)} chains')
print(f'  {n_ok}/{len(kept)} chains homotopic to their pre-rip original')
print(f'  {n_bad} changed topology: {len(emitted_bad)} on the emitted board, '
      f'{len(withheld_bad)} withheld/withdrawn anyway')
for r in emitted_bad:
    print(f"    EMITTED edge {r['edge']:3d} {r['net']:20s} {r['layer']}  "
          f"crossed {', '.join(r['crossed'])}")
for r in withheld_bad[:12]:
    print(f"    withheld edge {r['edge']:3d} {r['net']:20s} {r['layer']}  "
          f"crossed {', '.join(r['crossed'])}")
if len(withheld_bad) > 12:
    print(f'    ... {len(withheld_bad) - 12} more withheld')

save_json('homotopy.json', dict(
    checked=len(kept), ok=n_ok, changed=n_bad,
    emitted_changed=len(emitted_bad), punctures=len(punct),
    max_deflection=round(ds[-1], 4),
    pinned_parts=len(pinned), moved_parts=len(moved), new_parts=len(appeared),
    violations=results))
print('wrote homotopy.json')
