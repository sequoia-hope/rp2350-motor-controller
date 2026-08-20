#!/usr/bin/env python3
"""Module-level stretch: translate rigid leaf modules apart by small amounts.
Forces open seams narrower than T; hard constraints: no cross-leaf gap may
fall below min(current, 0) for courtyards (same side) or min(current, 0) for
rule-adjusted pad gaps (shared copper side); courtyards stay inside the edge
inset; |v| <= CAP. Pinned leaves never move. Writes vectors.json + seams."""
import json, math, sys, os, collections
S = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, S)
from geom import poly_dist, poly_closest, bbox, bbox_dist, rect_poly, translate
B = json.load(open(S+"/board.json")); M = json.load(open(S+"/modules.json"))
fp = {f['ref']: f for f in B['fps']}; mods = M['modules']
EDGE = B['edge']; ECLR = 0.3; PADCLR = 0.18
T = float(os.environ.get('T', 0.4)); CAP = float(os.environ.get('CAP', 0.6)); LAM = 0.15; ALPHA = 0.25
def obstacles(f):
    out = []
    for side, polys in f['courtyard'].items():
        for poly in polys: out.append(('cy', side, [tuple(p) for p in poly], bbox(poly)))
    if not f['courtyard']:
        for p in f['pads']:
            b = p['bbox']; bb = (b[0]-0.2, b[1]-0.2, b[2]+0.2, b[3]+0.2); out.append(('cy', f['layer'], rect_poly(bb), bb))
    for p in f['pads']:
        b = p['bbox']; poly = rect_poly(b)
        for side, on in (('F', p['F']), ('B', p['B'])):
            if on: out.append(('pad', side, poly, b))
    return out
LO = {m: [(r, o) for r in d['geo_refs'] for o in obstacles(fp[r])] for m, d in mods.items()}
names = list(mods); idx = {m: i for i, m in enumerate(names)}
movable = [m for m in names if not mods[m]['pinned']]
# ---- candidate pairs with closest points at v=0 --------------------------------
pairs = []   # (i, j, kind, g0, nx, ny, ri, rj, oi, oj)
for a in range(len(names)):
    for b in range(a+1, len(names)):
        ma, mb = names[a], names[b]
        if mods[ma]['pinned'] and mods[mb]['pinned']: continue
        for ra, oa in LO[ma]:
            for rb, ob in LO[mb]:
                if oa[0] != ob[0] or oa[1] != ob[1]: continue
                if bbox_dist(oa[3], ob[3]) > 2.5: continue
                d, pa, pb = poly_closest(oa[2], ob[2])
                if oa[0] == 'pad': d -= PADCLR
                nx, ny = pa[0]-pb[0], pa[1]-pb[1]; L = math.hypot(nx, ny)
                if L < 1e-9:  # overlapping: use bbox centre direction
                    nx = (oa[3][0]+oa[3][2]-ob[3][0]-ob[3][2])/2; ny = (oa[3][1]+oa[3][3]-ob[3][1]-ob[3][3])/2; L = math.hypot(nx, ny) or 1.0
                pairs.append([a, b, oa[0], d, nx/L, ny/L, ra, rb, oa, ob])
print(f'{len(pairs)} cross-leaf obstacle pairs within 2.5 mm ({sum(1 for p in pairs if p[3] < T)} under T={T})')
edges = []   # (i, side-string, o)
for m in movable:
    for r, o in LO[m]:
        if o[0] == 'cy': edges.append((idx[m], o))
def edge_gaps(o, vx, vy):
    b = o[3]; return (b[0]+vx-(EDGE[0]+ECLR), EDGE[2]-ECLR-(b[2]+vx), b[1]+vy-(EDGE[1]+ECLR), EDGE[3]-ECLR-(b[3]+vy))
V = [[0.0, 0.0] for _ in names]
def gap_exact(p, V):
    a, b, kind, g0, nx, ny, ra, rb, oa, ob = p
    A = translate(oa[2], V[a][0], V[a][1]); Bp = translate(ob[2], V[b][0], V[b][1])
    bA = (oa[3][0]+V[a][0], oa[3][1]+V[a][1], oa[3][2]+V[a][0], oa[3][3]+V[a][1]); bB = (ob[3][0]+V[b][0], ob[3][1]+V[b][1], ob[3][2]+V[b][0], ob[3][3]+V[b][1])
    d = poly_dist(A, Bp, bA, bB)
    return d - (PADCLR if kind == 'pad' else 0.0)
def gap_lin(p, V):
    a, b, kind, g0, nx, ny = p[:6]
    return g0 + (V[a][0]-V[b][0])*nx + (V[a][1]-V[b][1])*ny
def feasible_pairs(V, exact_thresh=0.35):
    bad = []
    for p in pairs:
        floor = min(p[3], T) - 1e-6          # stretch-only: no seam under T may shrink
        g = gap_lin(p, V)
        if g < exact_thresh + 0.5:
            g = gap_exact(p, V)
        if g < floor: bad.append(('pair', p[0], p[1], p[4], p[5]))
    for i, o in edges:
        e0 = edge_gaps(o, 0, 0); e1 = edge_gaps(o, V[i][0], V[i][1])
        for k, (g0, g1) in enumerate(zip(e0, e1)):
            if g1 < min(g0, 0.3) - 1e-6: bad.append(('x' if k < 2 else 'y', i, i, 0, 0))
    return bad
def feasible(V): return feasible_pairs(V)
for it in range(300):
    F = [[0.0, 0.0] for _ in names]
    for p in pairs:
        g = gap_lin(p, V)
        if p[3] < 0: continue               # designed tuck-unders are left alone
        if g < T: f = (T - g) + 0.05
        elif g < 1.5: f = 0.05*(1.5-g)/1.1
        else: continue
        if True:
            a, b = p[0], p[1]
            F[a][0] += f*p[4]; F[a][1] += f*p[5]; F[b][0] -= f*p[4]; F[b][1] -= f*p[5]
    for i, o in edges:
        e = edge_gaps(o, V[i][0], V[i][1])
        if e[0] < 0.3: F[i][0] += (0.3-e[0])
        if e[1] < 0.3: F[i][0] -= (0.3-e[1])
        if e[2] < 0.3: F[i][1] += (0.3-e[2])
        if e[3] < 0.3: F[i][1] -= (0.3-e[3])
    for m in movable:
        i = idx[m]; F[i][0] -= LAM*V[i][0]; F[i][1] -= LAM*V[i][1]
    for m in names:
        if mods[m]['pinned']: F[idx[m]] = [0.0, 0.0]
    # normalise force per leaf (count-independent) and step
    D = []
    for i, m in enumerate(names):
        fx, fy = F[i]; L = math.hypot(fx, fy)
        if L > 1.0: fx, fy = fx/L, fy/L
        D.append((ALPHA*fx, ALPHA*fy))
    alpha = 1.0; ok = False
    for _try in range(12):
        Dn = [list(d) for d in D]
        for _proj in range(25):
            Vn = []
            for i in range(len(names)):
                vx, vy = V[i][0]+alpha*Dn[i][0], V[i][1]+alpha*Dn[i][1]; L = math.hypot(vx, vy)
                if L > CAP: vx, vy = vx*CAP/L, vy*CAP/L
                Vn.append([vx, vy])
            badp = feasible_pairs(Vn)
            if not badp: ok = True; break
            # project the offending relative motion out (slide tangentially)
            for kind, a, b, nx, ny in badp:
                if kind == 'pair':
                    rel = (Dn[a][0]-Dn[b][0])*nx + (Dn[a][1]-Dn[b][1])*ny   # >0 opens, <0 closes
                    if rel >= 0: rel = 1e-3   # closing came from CAP clamp/nonlinearity: nudge open
                    c = rel - 1e-3
                    pa, pb = mods[names[a]]['pinned'], mods[names[b]]['pinned']
                    wa = 0.0 if pa else (1.0 if pb else 0.5); wb = 1.0 - wa if not pb else 0.0
                    Dn[a][0] -= wa*c*nx; Dn[a][1] -= wa*c*ny; Dn[b][0] += wb*c*nx; Dn[b][1] += wb*c*ny
                else:   # edge: kill the offending axis component
                    if kind == 'x': Dn[a][0] = 0.0
                    else: Dn[a][1] = 0.0
        if ok: break
        alpha *= 0.5
    if not ok: print('stalled at iteration', it); break
    dmax = max(math.hypot(Vn[i][0]-V[i][0], Vn[i][1]-V[i][1]) for i in range(len(names)))
    V = Vn
    if it % 20 == 0: print(f'iter {it:3d} max step {dmax:.4f} |v|max {max(math.hypot(*v) for v in V):.3f}')
    if dmax < 2e-4 and it > 10: print(f'converged at iteration {it}'); break
vec = {m: [round(V[idx[m]][0], 4), round(V[idx[m]][1], 4)] for m in names}
# seam report: min courtyard gap per leaf pair before/after
seam0 = collections.defaultdict(lambda: 9.9); seam1 = collections.defaultdict(lambda: 9.9)
for p in pairs:
    if p[2] != 'cy': continue
    k = (names[p[0]], names[p[1]]); seam0[k] = min(seam0[k], p[3]); seam1[k] = min(seam1[k], gap_exact(p, V))
seams = sorted(((k, round(seam0[k], 3), round(seam1[k], 3)) for k in seam0), key=lambda t: t[1])
json.dump(dict(T=T, CAP=CAP, vectors=vec, seams=[[a, b, g0, g1] for (a, b), g0, g1 in seams]), open(S+'/vectors.json', 'w'), indent=1)
print('\nleaf vectors (mm):')
for m in movable:
    v = vec[m]
    if math.hypot(*v) > 0.005: print(f'  {m:8s} ({v[0]:+.3f}, {v[1]:+.3f})  |v|={math.hypot(*v):.3f}')
print('\ntight seams (courtyard gap before -> after), under T before:')
for (a, b), g0, g1 in seams:
    if g0 < T: print(f'  {a:8s} ~ {b:8s}  {g0:6.3f} -> {g1:6.3f}  {"WORSE" if g1 < g0 - 1e-3 else ""}')
shr = [(a, b, g0, g1) for (a, b), g0, g1 in seams if g1 < min(g0, T) - 1e-3]
print(f'seams that shrank below min(before,T): {len(shr)}', shr[:10])
opened = sum(1 for (a, b), g0, g1 in seams if g0 < T and g1 > g0 + 1e-3)
print(f'seams under T: {sum(1 for (a,b),g0,g1 in seams if 0 <= g0 < T)}, opened: {opened}, mean gain {sum(max(0,g1-g0) for (a,b),g0,g1 in seams if 0<=g0<T)/max(1,sum(1 for (a,b),g0,g1 in seams if 0<=g0<T)):.3f} mm')
