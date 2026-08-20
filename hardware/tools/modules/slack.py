#!/usr/bin/env python3
"""Per-module translational slack: how far can each module slide in +x/-x/+y/-y
before any of its parts' courtyard (same side) or pad (shared copper side)
touches another module's part, or the board edge. Exact polygons."""
import json, math, sys, os, collections
S = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, S)
from geom import poly_dist, bbox, bbox_dist, rect_poly, translate, pip
B = json.load(open(S+"/board.json")); M = json.load(open(S+"/modules.json"))
fp = {f['ref']: f for f in B['fps']}
assign = M['assign']; mods = M['modules']
EDGE = B['edge']; ECLR = 0.3
PADCLR = 0.18
def obstacles(f):
    """list of (kind, side, poly, bbox): courtyards per side, pads per copper side."""
    out = []
    for side, polys in f['courtyard'].items():
        for poly in polys:
            out.append(('cy', side, [tuple(p) for p in poly], bbox(poly)))
    if not f['courtyard']:
        # courtyard-less (test points etc.): pad bbox + 0.2 as a stand-in
        for p in f['pads']:
            b = p['bbox']; bb = (b[0]-0.2, b[1]-0.2, b[2]+0.2, b[3]+0.2)
            out.append(('cy', f['layer'], rect_poly(bb), bb))
    for p in f['pads']:
        b = p['bbox']; poly = rect_poly(b)
        for side, on in (('F', p['F']), ('B', p['B'])):
            if on: out.append(('pad', side, poly, b))
    return out
OBS = {r: obstacles(f) for r, f in fp.items()}
def gap(o1, o2):
    """rule-aware gap between two obstacles; None if they don't interact."""
    k1, s1, P1, b1 = o1; k2, s2, P2, b2 = o2
    if s1 != s2: return None
    if k1 == 'cy' and k2 == 'cy': return poly_dist(P1, P2, b1, b2)
    if k1 == 'pad' and k2 == 'pad': return poly_dist(P1, P2, b1, b2) - PADCLR
    return None
REPLACE = set('U30 U31 U32 U33 C110 C111 C112 C113 R97 R98 R99 R100 R101 R103 R104 R105 R108 Q6'.split())
def module_slack(m, maxd=3.0, others=None, ignore=REPLACE):
    refs = [r for r in mods[m]['refs'] if r not in ignore]
    mine = [(r, o) for r in refs for o in OBS[r]]
    res = {}
    for name, (ux, uy) in (('+x', (1, 0)), ('-x', (-1, 0)), ('+y', (0, 1)), ('-y', (0, -1))):
        lim = maxd; who = 'free'
        # board edge
        for r, o in mine:
            if o[0] != 'cy': continue
            b = o[3]
            d = {'+x': EDGE[2]-ECLR-b[2], '-x': b[0]-(EDGE[0]+ECLR), '+y': EDGE[3]-ECLR-b[3], '-y': b[1]-(EDGE[1]+ECLR)}[name]
            if d < lim: lim, who = d, f'edge({r})'
        for r, o in mine:
            for q, mq in assign.items():
                if mq == m or q in ignore: continue
                for o2 in OBS[q]:
                    if o2[1] != o[1] or (o[0] != o2[0]): continue
                    # only obstacles ahead in the move direction matter; binary search the contact distance
                    b1, b2 = o[3], o2[3]
                    # quick reject: not overlapping in the perpendicular extent, or behind
                    if ux:
                        if b2[3] < b1[1] - 0.5 or b2[1] > b1[3] + 0.5: continue
                        if ux > 0 and b2[2] < b1[0]: continue
                        if ux < 0 and b2[0] > b1[2]: continue
                    else:
                        if b2[2] < b1[0] - 0.5 or b2[0] > b1[2] + 0.5: continue
                        if uy > 0 and b2[3] < b1[1]: continue
                        if uy < 0 and b2[1] > b1[3]: continue
                    g0 = gap(o, o2)
                    if g0 is None: continue
                    if bbox_dist(b1, b2) > lim + 0.01: continue
                    # find largest t in [0,lim] s.t. gap(translate(o,t)) >= min(g0, 0) (may not worsen an existing overlap)
                    floor = min(g0, 0.0)   # never deepen an overlap, never create one
                    lo, hi = 0.0, lim
                    def gt(t): return gap((o[0], o[1], translate(o[2], ux*t, uy*t), (b1[0]+ux*t, b1[1]+uy*t, b1[2]+ux*t, b1[3]+uy*t)), o2)
                    if gt(hi) >= floor - 1e-9: continue
                    for _ in range(18):
                        mid = (lo+hi)/2
                        if gt(mid) >= floor - 1e-9: lo = mid
                        else: hi = mid
                    if lo < lim: lim, who = lo, f'{q}[{mq}]' + ('' if o[0]=='cy' else ' pad')
        res[name] = (round(lim, 3), who)
    return res
if __name__ == '__main__':
    which = sys.argv[1:] or [m for m in mods if not mods[m]['pinned']]
    for m in which:
        r = module_slack(m)
        print(f"{m:9s} " + '  '.join(f"{k}:{v[0]:5.2f} {v[1]:>16s}" for k, v in r.items()))
