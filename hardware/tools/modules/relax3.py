#!/usr/bin/env python3
"""Module-level stretch as a sequential LP (coupled rigid-body moves).
max  sum_k t_k - MU*|v|_1   s.t.  t_k <= g_k(v) (linearised), t_k <= T for tight seams,
     g_k(v) >= min(g0_k, T) for every cross-leaf pair (stretch-only floors),
     courtyards inside edge inset (no worse than min(current,0.3)), |v| <= CAP,
     trust region per iteration; exact polygon re-check + re-linearisation each step."""
import json, math, sys, os, collections
import numpy as np
from scipy.optimize import linprog
S = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, S)
from geom import poly_dist, poly_closest, bbox, bbox_dist, rect_poly, translate
B = json.load(open(S+"/board.json")); M = json.load(open(S+"/modules.json"))
fp = {f['ref']: f for f in B['fps']}; mods = M['modules']
EDGE = B['edge']; ECLR = 0.3; PADCLR = 0.18
T = float(os.environ.get('T', 0.4)); CAP = float(os.environ.get('CAP', 0.6)); MU = float(os.environ.get('MU', 0.05)); TR = 0.1
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
movable = [m for m in names if not mods[m]['pinned']]; mi = {m: k for k, m in enumerate(movable)}
pairs = []
for a in range(len(names)):
    for b in range(a+1, len(names)):
        ma, mb = names[a], names[b]
        if mods[ma]['pinned'] and mods[mb]['pinned']: continue
        for ra, oa in LO[ma]:
            for rb, ob in LO[mb]:
                if oa[0] != ob[0] or oa[1] != ob[1]: continue
                if bbox_dist(oa[3], ob[3]) > 2.5: continue
                clr = max(oa[4], ob[4], PADCLR) if oa[0] == 'pad' else 0.0
                d, pa, pb = poly_closest(oa[2], ob[2]); d -= clr
                nx, ny = pa[0]-pb[0], pa[1]-pb[1]; L = math.hypot(nx, ny)
                if L < 1e-9:
                    nx = (oa[3][0]+oa[3][2]-ob[3][0]-ob[3][2])/2; ny = (oa[3][1]+oa[3][3]-ob[3][1]-ob[3][3])/2; L = math.hypot(nx, ny) or 1.0
                pairs.append(dict(a=a, b=b, oa=oa, ob=ob, clr=clr, g0=d, g=d, n=(nx/L, ny/L), n0=(nx/L, ny/L), ra=ra, rb=rb))
print(f'{len(pairs)} pairs, {sum(1 for p in pairs if 0 <= p["g0"] < T)} tight (0<=g<T)')
def exact(p, V):
    oa, ob = p['oa'], p['ob']; a, b = p['a'], p['b']
    if p['g0'] < -1e-3:   # designed tuck-under: 'no deeper along the initial normal' (bbox depth is not a consistent metric)
        n = p['n0']; return p['g0'] + (V[a][0]-V[b][0])*n[0] + (V[a][1]-V[b][1])*n[1], n
    A = translate(oa[2], *V[a]); Bp = translate(ob[2], *V[b])
    bA = (oa[3][0]+V[a][0], oa[3][1]+V[a][1], oa[3][2]+V[a][0], oa[3][3]+V[a][1]); bB = (ob[3][0]+V[b][0], ob[3][1]+V[b][1], ob[3][2]+V[b][0], ob[3][3]+V[b][1])
    if bbox_dist(bA, bB) - p['clr'] > 3.0: return 3.0, None
    d, pa, pb = poly_closest(A, Bp); d -= p['clr']
    nx, ny = pa[0]-pb[0], pa[1]-pb[1]; L = math.hypot(nx, ny)
    n = (nx/L, ny/L) if L > 1e-9 else p['n']
    return d, n
edges = []
for m in movable:
    for r, o in LO[m]:
        if o[0] == 'cy': edges.append((idx[m], o[3]))
V = [[0.0, 0.0] for _ in names]
nv = len(movable)
def solve_lp(V):
    # refresh gaps/normals near floors (exact), keep linearisation elsewhere
    tight = [k for k, p in enumerate(pairs) if -1e-3 <= p['g0'] < T]
    nt = len(tight)
    # variables: dvx+ dvx- dvy+ dvy- per movable leaf (4nv), then t_k (nt)
    nvar = 4*nv + nt
    c = np.zeros(nvar)
    for k in range(nv): c[4*k:4*k+4] = MU
    for j in range(nt): c[4*nv+j] = -1.0
    A_ub = []; b_ub = []
    def dv_coeff(row, leaf, nx, ny, sign):
        if mods[names[leaf]]['pinned']: return
        k = mi[names[leaf]]
        row[4*k] += sign*nx; row[4*k+1] -= sign*nx; row[4*k+2] += sign*ny; row[4*k+3] -= sign*ny
    tk = {k: j for j, k in enumerate(tight)}
    for k, p in enumerate(pairs):
        floor = min(p['g0'], T)
        g, n = p['g'], p['n']
        # -( (dva - dvb).n ) <= g - floor
        row = np.zeros(nvar)
        dv_coeff(row, p['a'], n[0], n[1], -1.0); dv_coeff(row, p['b'], n[0], n[1], +1.0)
        A_ub.append(row); b_ub.append(g - floor + 1e-9)
        if k in tk:
            row2 = np.zeros(nvar); dv_coeff(row2, p['a'], n[0], n[1], -1.0); dv_coeff(row2, p['b'], n[0], n[1], +1.0)
            row2[4*nv+tk[k]] = 1.0   # t_k - (dva-dvb).n <= g
            A_ub.append(row2); b_ub.append(g)
    for i, bb in edges:
        k = mi[names[i]]
        for axis, g0, g in ((0, bb[0]-(EDGE[0]+ECLR), bb[0]+V[i][0]-(EDGE[0]+ECLR)), (0, -(EDGE[2]-ECLR-bb[2]), -(EDGE[2]-ECLR-(bb[2]+V[i][0]))),
                             (1, bb[1]-(EDGE[1]+ECLR), bb[1]+V[i][1]-(EDGE[1]+ECLR)), (1, -(EDGE[3]-ECLR-bb[3]), -(EDGE[3]-ECLR-(bb[3]+V[i][1])))):
            # left/top: dv >= floor - g ; right/bottom: dv <= g_right_room ... handled via sign trick: for right side we stored negatives
            pass
        # simpler explicit bounds per axis below
    bounds = []
    for m in movable:
        i = idx[m]
        lox = -TR; hix = TR; loy = -TR; hiy = TR
        for ii, bb in edges:
            if ii != i: continue
            roomL = (bb[0]+V[i][0]) - (EDGE[0]+ECLR); roomR = (EDGE[2]-ECLR) - (bb[2]+V[i][0])
            roomT = (bb[1]+V[i][1]) - (EDGE[1]+ECLR); roomB = (EDGE[3]-ECLR) - (bb[3]+V[i][1])
            fl = lambda room0, room: max(0.0, room - min(room0, 0.3))  # how much we may still eat
            lox = max(lox, -fl(roomL - V[i][0] + V[i][0], roomL)); hix = min(hix, fl(roomR, roomR))
            loy = max(loy, -fl(roomT, roomT)); hiy = min(hiy, fl(roomB, roomB))
        # CAP box
        lox = max(lox, -CAP - V[i][0]); hix = min(hix, CAP - V[i][0]); loy = max(loy, -CAP - V[i][1]); hiy = min(hiy, CAP - V[i][1])
        bounds += [(0, max(0, hix)), (0, max(0, -lox)), (0, max(0, hiy)), (0, max(0, -loy))]
    for j in range(nt): bounds.append((-5, T - 0))   # t_k <= T; (t_k may be negative if g<0; tight has g0>=0)
    res = linprog(c, A_ub=np.array(A_ub), b_ub=np.array(b_ub), bounds=bounds, method='highs')
    if not res.success: print('LP failed:', res.message); return None
    x = res.x
    dV = {}
    for m in movable:
        k = mi[m]; dV[idx[m]] = (x[4*k]-x[4*k+1], x[4*k+2]-x[4*k+3])
    return dV
def all_ok(V):
    bad = 0
    for p in pairs:
        if p['g'] > min(p['g0'], T) + 0.6: continue   # can't have moved enough to matter (TR <= 0.1/iter, checked each iter)
        g, n = exact(p, V)
        if g < min(p['g0'], T) - 1e-6: bad += 1
    return bad == 0
for it in range(30):
    dV = solve_lp(V)
    if dV is None: break
    step = max(math.hypot(*d) for d in dV.values())
    # try full step, backtrack by halves on exact infeasibility
    s = 1.0; accepted = False
    for _ in range(6):
        Vn = [list(v) for v in V]
        for i, d in dV.items():
            vx, vy = V[i][0]+s*d[0], V[i][1]+s*d[1]; L = math.hypot(vx, vy)
            if L > CAP: vx, vy = vx*CAP/L, vy*CAP/L
            Vn[i] = [vx, vy]
        # exact refresh of gaps/normals for pairs that could be near floor
        ok = True
        for p in pairs:
            if p['g'] - min(p['g0'], T) > 0.5: continue
            g, n = exact(p, Vn)
            if g < min(p['g0'], T) - 1e-6: ok = False; break
        if ok: accepted = True; break
        s *= 0.5
    if not accepted: print('no feasible step at iter', it); break
    V = Vn
    for p in pairs:   # re-linearise near-floor pairs at the accepted point
        if p['g'] - min(p['g0'], T) > 0.8: continue
        g, n = exact(p, V); p['g'] = g; p['n'] = n
    tight_now = sum(1 for p in pairs if 0 <= p['g0'] < T and p['g'] < T)
    gain = sum(min(p['g'], T) - p['g0'] for p in pairs if 0 <= p['g0'] < T)
    print(f'iter {it:2d}: step {s*step:.3f} |v|max {max(math.hypot(*v) for v in V):.3f} tight-still-under-T {tight_now} total gain {gain:.3f}')
    if s*step < 1e-3: break
vec = {m: [round(V[idx[m]][0], 4), round(V[idx[m]][1], 4)] for m in names}
seam0 = collections.defaultdict(lambda: 9.9); seam1 = collections.defaultdict(lambda: 9.9); who = {}
for p in pairs:
    if p['oa'][0] != 'cy': continue
    k = (names[p['a']], names[p['b']]); g1 = exact(p, V)[0]
    if p['g0'] < seam0[k]: seam0[k] = p['g0']; who[k] = (p['ra'], p['rb'])
    seam1[k] = min(seam1[k], g1)
seams = sorted(((k, round(seam0[k], 3), round(seam1[k], 3)) for k in seam0), key=lambda t: t[1])
json.dump(dict(T=T, CAP=CAP, vectors=vec, seams=[[a, b, g0, g1, who[(a, b)]] for (a, b), g0, g1 in seams]), open(S+'/vectors.json', 'w'), indent=1)
print('\nleaf vectors (mm):')
for m in movable:
    v = vec[m]
    if math.hypot(*v) > 0.005: print(f'  {m:8s} ({v[0]:+.3f}, {v[1]:+.3f})  |v|={math.hypot(*v):.3f}')
print('\nseams under T before (courtyard gap before -> after):')
for (a, b), g0, g1 in seams:
    if g0 < T: print(f'  {a:8s} ~ {b:8s}  {g0:6.3f} -> {g1:6.3f}  {"WORSE" if g1 < g0 - 1e-3 else ("+%.3f" % (g1-g0) if g1 > g0+1e-3 else "")}')
tight = [(a, b, g0, g1) for (a, b), g0, g1 in seams if 0 <= g0 < T]
print(f'seams 0<=g<T: {len(tight)}, opened: {sum(1 for t in tight if t[3] > t[2]+1e-3)}, mean gain {sum(t[3]-t[2] for t in tight)/max(1,len(tight)):.3f} mm, still under T after: {sum(1 for t in tight if t[3] < T)}')
