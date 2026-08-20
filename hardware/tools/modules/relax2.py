#!/usr/bin/env python3
"""Module-level stretch, greedy/exact version.
Each movable leaf in turn tries 16 directions x 4 step sizes; a move is taken
if it is feasible (no cross-leaf seam drops below min(current, T); courtyards
stay >= min(current,0.3) inside the edge; |v| <= CAP) and lowers the leaf's
seam energy sum((T-g)^2 for g<T) + MU*|v|^2. Sweeps until no leaf improves.
Exact polygon gaps on the leaf's own pairs only (fast). Also --probe: per leaf
max feasible travel in 8 directions (moving that leaf alone)."""
import json, math, sys, os, collections
S = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, S)
from geom import poly_dist, bbox, bbox_dist, rect_poly, translate
B = json.load(open(S+"/board.json")); M = json.load(open(S+"/modules.json"))
fp = {f['ref']: f for f in B['fps']}; mods = M['modules']
EDGE = B['edge']; ECLR = 0.3; PADCLR = 0.18
T = float(os.environ.get('T', 0.4)); CAP = float(os.environ.get('CAP', 0.6)); MU = float(os.environ.get('MU', 0.3))
LCLR = {}  # pad local clearances (mounting holes 1.4mm) from board.json if present
def obstacles(f):
    out = []
    for side, polys in f['courtyard'].items():
        for poly in polys: out.append(('cy', side, [tuple(p) for p in poly], bbox(poly), 0.0))
    if not f['courtyard']:
        for p in f['pads']:
            b = p['bbox']; bb = (b[0]-0.2, b[1]-0.2, b[2]+0.2, b[3]+0.2); out.append(('cy', f['layer'], rect_poly(bb), bb, 0.0))
    for p in f['pads']:
        b = p['bbox']; poly = rect_poly(b); lc = p.get('lc', 0.0)
        for side, on in (('F', p['F']), ('B', p['B'])):
            if on: out.append(('pad', side, poly, b, lc))
    return out
LO = {m: [(r, o) for r in d['geo_refs'] for o in obstacles(fp[r])] for m, d in mods.items()}
names = list(mods); idx = {m: i for i, m in enumerate(names)}
movable = [m for m in names if not mods[m]['pinned']]
pairs = []; by_leaf = collections.defaultdict(list)
for a in range(len(names)):
    for b in range(a+1, len(names)):
        ma, mb = names[a], names[b]
        if mods[ma]['pinned'] and mods[mb]['pinned']: continue
        for ra, oa in LO[ma]:
            for rb, ob in LO[mb]:
                if oa[0] != ob[0] or oa[1] != ob[1]: continue
                if bbox_dist(oa[3], ob[3]) > 2.5: continue
                clr = max(oa[4], ob[4], PADCLR) if oa[0] == 'pad' else 0.0
                d = poly_dist(oa[2], ob[2], oa[3], ob[3]) - clr
                pairs.append((a, b, oa, ob, clr, d, ra, rb)); by_leaf[a].append(len(pairs)-1); by_leaf[b].append(len(pairs)-1)
edges = collections.defaultdict(list)
for m in movable:
    for r, o in LO[m]:
        if o[0] == 'cy': edges[idx[m]].append(o)
def gap(pi, V):
    a, b, oa, ob, clr, g0, ra, rb = pairs[pi]
    A = translate(oa[2], *V[a]); Bp = translate(ob[2], *V[b])
    bA = (oa[3][0]+V[a][0], oa[3][1]+V[a][1], oa[3][2]+V[a][0], oa[3][3]+V[a][1]); bB = (ob[3][0]+V[b][0], ob[3][1]+V[b][1], ob[3][2]+V[b][0], ob[3][3]+V[b][1])
    if bbox_dist(bA, bB) - clr > 3.0: return 3.0
    return poly_dist(A, Bp, bA, bB) - clr
def edge_ok(i, V):
    for o in edges[i]:
        b = o[3]
        for g0, g1 in ((b[0]-(EDGE[0]+ECLR), b[0]+V[i][0]-(EDGE[0]+ECLR)), (EDGE[2]-ECLR-b[2], EDGE[2]-ECLR-(b[2]+V[i][0])),
                       (b[1]-(EDGE[1]+ECLR), b[1]+V[i][1]-(EDGE[1]+ECLR)), (EDGE[3]-ECLR-b[3], EDGE[3]-ECLR-(b[3]+V[i][1]))):
            if g1 < min(g0, 0.3) - 1e-6: return False
    return True
def leaf_ok(i, V):
    if not edge_ok(i, V): return False
    for pi in by_leaf[i]:
        g0 = pairs[pi][5]
        if gap(pi, V) < min(g0, T) - 1e-6: return False
    return True
def energy(i, V):
    e = MU*(V[i][0]**2 + V[i][1]**2)
    for pi in by_leaf[i]:
        if pairs[pi][5] < 0: continue          # designed tuck-unders don't count
        g = gap(pi, V)
        if g < T: e += (T-g)**2
    return e
V = [[0.0, 0.0] for _ in names]
if '--probe' in sys.argv:
    print('max feasible travel per leaf (mm), moving it alone, stretch floors (no seam under T shrinks):')
    PROBE = {}
    for m in movable:
        i = idx[m]; row = []
        for name, (ux, uy) in (('+x', (1, 0)), ('-x', (-1, 0)), ('+y', (0, 1)), ('-y', (0, -1))):
            lo, hi = 0.0, 2.0
            Vt = [list(v) for v in V]; Vt[i] = [ux*hi, uy*hi]
            if leaf_ok(i, Vt): lo = hi
            else:
                for _ in range(14):
                    mid = (lo+hi)/2; Vt[i] = [ux*mid, uy*mid]
                    if leaf_ok(i, Vt): lo = mid
                    else: hi = mid
            row.append(f'{name}:{lo:5.2f}'); PROBE.setdefault(m, {})[name] = round(lo, 3)
        print(f'  {m:8s} ' + '  '.join(row))
    json.dump(PROBE, open(S+'/probe.json', 'w'), indent=1)
    sys.exit()
DIRS = [(math.cos(2*math.pi*k/16), math.sin(2*math.pi*k/16)) for k in range(16)]
STEPS = [0.2, 0.1, 0.05, 0.02]
for sweep in range(40):
    moved = 0
    for m in movable:
        i = idx[m]; e0 = energy(i, V)
        if e0 < 1e-9: continue
        best = None
        for ux, uy in DIRS:
            for s in STEPS:
                vx, vy = V[i][0]+ux*s, V[i][1]+uy*s
                if math.hypot(vx, vy) > CAP + 1e-9: continue
                Vt = [list(v) for v in V]; Vt[i] = [vx, vy]
                if not leaf_ok(i, Vt): continue
                e1 = energy(i, Vt)
                if e1 < e0 - 1e-9 and (best is None or e1 < best[0]): best = (e1, vx, vy)
                break   # largest feasible, improving step in this direction is enough
        if best:
            V[i] = [best[1], best[2]]; moved += 1
    print(f'sweep {sweep}: {moved} leaves moved, |v|max {max(math.hypot(*v) for v in V):.3f}')
    if moved == 0: break
vec = {m: [round(V[idx[m]][0], 4), round(V[idx[m]][1], 4)] for m in names}
seam0 = collections.defaultdict(lambda: 9.9); seam1 = collections.defaultdict(lambda: 9.9); seamw = {}
for pi, p in enumerate(pairs):
    k = (names[p[0]], names[p[1]])
    g0 = p[5]; g1 = gap(pi, V)
    if p[2][0] == 'cy':
        seam0[k] = min(seam0[k], g0); seam1[k] = min(seam1[k], g1)
    if g0 < seam0.get(k, 9.9) + 1e-9: seamw[k] = (p[6], p[7])
seams = sorted(((k, round(seam0[k], 3), round(seam1[k], 3)) for k in seam0), key=lambda t: t[1])
json.dump(dict(T=T, CAP=CAP, vectors=vec, seams=[[a, b, g0, g1, seamw.get((a,b), ('',''))] for (a, b), g0, g1 in seams]), open(S+'/vectors.json', 'w'), indent=1)
print('\nleaf vectors (mm):')
for m in movable:
    v = vec[m]
    if math.hypot(*v) > 0.005: print(f'  {m:8s} ({v[0]:+.3f}, {v[1]:+.3f})  |v|={math.hypot(*v):.3f}')
print('\nseams under T before (courtyard gap before -> after):')
for (a, b), g0, g1 in seams:
    if g0 < T: print(f'  {a:8s} ~ {b:8s}  {g0:6.3f} -> {g1:6.3f}  {"WORSE" if g1 < g0 - 1e-3 else ("+%.3f" % (g1-g0) if g1 > g0+1e-3 else "")}')
tight = [(a, b, g0, g1) for (a, b), g0, g1 in seams if 0 <= g0 < T]
print(f'seams 0<=g<T: {len(tight)}, opened: {sum(1 for t in tight if t[3] > t[2]+1e-3)}, mean gain {sum(t[3]-t[2] for t in tight)/max(1,len(tight)):.3f} mm, still under T after: {sum(1 for t in tight if t[3] < T)}')
