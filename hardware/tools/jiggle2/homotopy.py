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

Chains appended past the graph alignment (reroute.py's rebuilt copper) span
different endpoints than any single original edge, so they are class-checked
by their own stage and only counted here.

Reads data/graph.json + data/obstacles.json + data/solution.json, writes
data/homotopy.json. Pure check; never touches boards or the solution.
"""
import math, sys
from common import (NM, BOARD_PRE, BOARD_BASE, VARIANT, load_board, load_json,
                    save_json, replay_drop, path_word, pinned_parts,
                    build_punctures)

G = load_json('graph.json')
O = load_json('obstacles.json')
sol = load_json('solution.json')
nodes = G['nodes']
F_CU, B_CU = G['F'], G['B']
COPPER = sorted({e['layer'] for e in G['edges']} | {t['layer'] for t in O['tracks']}
                | {F_CU, B_CU})

kept, drop = replay_drop(G, O)
extra = len(sol['edges']) - len(kept)
if extra < 0:
    sys.exit(f'alignment failure: replay kept {len(kept)} edges, '
             f'solution has {len(sol["edges"])} — drop replay diverged')
for (ei, e), se in zip(kept, sol['edges']):
    if e['layer'] != se['layer'] or abs(e['w'] - se['w']) > 1e-9:
        sys.exit(f'alignment failure at graph edge {ei}: '
                 f'({e["layer"]},{e["w"]}) vs ({se["layer"]},{se["w"]})')
print(f'alignment: {len(kept)} chains map 1:1 onto solution '
      f'({len(drop)} dropped bridges, {extra} appended rerouted entries)')

pre = load_board(BOARD_PRE)
base = load_board(BOARD_BASE)
LNAME = {lay: base.GetLayerName(lay) for lay in COPPER}
pinned, moved, appeared = pinned_parts(pre, base)
print(f'parts: {len(pinned)} pinned, {len(moved)} moved, {len(appeared)} rev-B new '
      f'(moved+new pads excluded from punctures)')

punct = build_punctures(O, pinned, COPPER)
by_layer = {lay: [(i, p) for i, p in enumerate(punct) if lay in p[2]]
            for lay in COPPER}
print('punctures:', len(punct), 'total,',
      ', '.join(f'{LNAME[lay]} {len(by_layer[lay])}' for lay in COPPER))

if '--selftest' in sys.argv:
    T = (5.0 + 2.19e-8, 5.0, frozenset(COPPER), 'X', 'T.1')
    C = [(0, T)]
    assert path_word([(4, 4), (6, 4), (6, 6), (4, 6)], C, closed=True), \
        'loop encircling the puncture must give a nonempty word'
    assert not path_word([(7, 4), (9, 4), (9, 6), (7, 6)], C, closed=True), \
        'disjoint loop must reduce to empty'
    same = [(0, 7), (10, 7)] + [(0, 7), (5, 9), (10, 7)][::-1]
    assert not path_word(same, C, closed=True), 'same-side deformation must reduce to empty'
    flip = [(0, 7), (10, 7)] + [(0, 7), (4, 1), (10, 7)][::-1]
    assert path_word(flip, C, closed=True) == [1], 'side switch must survive reduction'
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
    word = path_word(loop, cands, closed=True) if cands else []
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

rerouted = [se for se in sol['edges'][len(kept):]]
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
if rerouted:
    print(f'  {len(rerouted)} rerouted chains skipped here '
          f'(class-checked by reroute.py; {sum(1 for r in rerouted if r.get("class_ok"))} '
          f'marked class-preserving)')
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
    max_deflection=round(ds[-1], 4), rerouted_skipped=len(rerouted),
    pinned_parts=len(pinned), moved_parts=len(moved), new_parts=len(appeared),
    violations=results))
print('wrote homotopy.json')
