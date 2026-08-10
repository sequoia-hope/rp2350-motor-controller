#!/usr/bin/env python3
"""jiggle2 creep: feasibility-projected morph stepper.

The motion law IS the rule system. Movable units — footprint assemblies
(pads + bound copper endpoints + courtyard), free trace nodes, vias — advance
toward their rev-B targets only as far as the DRC clearance geometry admits:
each proposed move gets a line search for the largest fraction whose margins
stay above per-pair floors, and a unit with no room does not move. Floors:
rule + guard for pairs the shipped board held apart; the shipped margin
itself for pairs shipped at/below rule + 0.02 (proof by shipping — the model
carries per-pair bias, so the shipped distance is the only trustworthy
'legal like it shipped' proxy; same lesson as solve.py's grandfathering).
Pad geometry comes from padgeom.json (kicad's own effective polygons plus
local clearance overrides like the mounting holes' 1.4mm) so the bias is
small and position-independent — graft's bounding rects are NOT good enough:
a chain sliding along a rect at held model-margin walks off the real shape.

Connectivity is a constraint too: same-net pairs that touch on the shipped
board (T-junctions, stubs under pads — contacts graft's endpoint drag test
can't see) must KEEP touching, aggregated per pair as 'at least one contact
point survives'. GND is exempt (the pour reconnects it). A part that would
slide off its stub blocks honestly instead of silently going unconnected.

Blocked units deposit pressure on the dynamic copper that clamps them
(attachment clamps pull the left-behind copper along instead); pressured
copper yields in the alternating copper passes as far as ITS floors allow,
so packed corridors creep like a traffic jam instead of shearing. Pressure
absorbed by static copper or a wedged cohort is the stall set — the computed
mid-flight reroute worklist — written to creep_report.json.

Steps are capped at --step mm, far below any rule corridor width, so
endpoint feasibility is sufficient (no tunneling) and DRC only ever judges
the emitted static checkpoints anyway.

Usage:  courtyards.py && padgeom.py       (once, after board prep)
        creep.py [--snapshots 12] [--step 0.05] [--cycles 600]
                 [--guard 0.015] [--copper-sweeps 6] [--measure-only]
        morph.py                          (emit + gate every checkpoint)
"""
import math, sys, time
from collections import defaultdict
from common import load_json, save_json, seg_seg_dist, replay_drop


def _arg(flag, default, cast=float):
    return cast(sys.argv[sys.argv.index(flag) + 1]) if flag in sys.argv else default

G = load_json('graph.json')
O = load_json('obstacles.json')
try:
    CY = load_json('courtyards.json')
except FileNotFoundError:
    CY = {}
    print('WARNING: courtyards.json missing (run courtyards.py) — '
          'courtyard rule unchecked')
try:
    PG = load_json('padgeom.json')
except FileNotFoundError:
    PG = {}
    print('WARNING: padgeom.json missing (run padgeom.py) — '
          'bounding-rect pads, no local clearances')

nodes = G['nodes']
CLR = O['clearance']
HOLE_CLR = O['hole_clearance']
EDGE = O.get('board_edge')
ECLR = O.get('edge_clearance', 0.3)
MOVED = O.get('moved') or {}
if not MOVED:
    sys.exit('creep.py requires the morph variant (JIGGLE2_VARIANT=morph)')

SUB = 0.8                       # chain subdivision pitch, mm
STEP = _arg('--step', 0.05)     # per-unit per-cycle motion cap, mm
GUARD = _arg('--guard', 0.015)  # floor above rule for non-grandfathered pairs
CRT_GUARD = 0.003               # courtyard-courtyard floor (rule is overlap=0)
GF_CUT = 0.02                   # measure pass records pairs with margin below
OV = 0.02                       # attachment: required copper overlap depth
SNAPSHOTS = _arg('--snapshots', 12, int)
CYCLES = _arg('--cycles', 600, int)
NCOPPER = _arg('--copper-sweeps', 6, int)
STALL_WIN = 40                  # cycles without part advance -> stall
EPS = 1e-9
CELL = 1.2
QR = 0.9                        # grid query/insert radius

# ---- graph -> solver state ---------------------------------------------------
kept, drop = replay_drop(G, O)
E = [e for _, e in kept]                       # shipped nets, original order
bridges = [G['edges'][ei] for ei in sorted(drop)]

P = [[n['x'], n['y']] for n in nodes]
is_via = [n['kind'] == 'via' for n in nodes]
via_dia = [n.get('dia') or 0.0 for n in nodes]
via_drill = [n.get('drill') or 0.0 for n in nodes]
node_net = [n.get('net') or '' for n in nodes]

fixed = set()                   # never move: pins + anchors of pinned parts
part_of = {}                    # nid -> moved ref (rides the part)
part_anchors = defaultdict(list)
anchor0 = {}
for i, n in enumerate(nodes):
    if n['bind'] and n['target']:
        r = n['bind'][0]
        if r in MOVED:
            part_of[i] = r
            part_anchors[r].append(i)
            anchor0[i] = (n['x'], n['y'])
        else:
            fixed.add(i)
    elif n.get('pin'):
        fixed.add(i)

segs = []                       # dict(a,b,layer,w,net,c[,bridge])
chains = []                     # per kept edge: ordered node ids (morph.json)
inc = defaultdict(list)         # nid -> [seg index]
chain_segs = defaultdict(list)  # chain id -> [seg index] (AT entrainment)


def _new_node(x, y, net):
    nid = len(P)
    P.append([x, y])
    is_via.append(False)
    via_dia.append(0.0)
    via_drill.append(0.0)
    node_net.append(net)
    return nid

for ci, e in enumerate(E):
    a, b = e['a'], e['b']
    (ax, ay), (bx, by) = P[a], P[b]
    nsub = max(1, int(math.ceil(math.hypot(bx - ax, by - ay) / SUB)))
    ids = [a]
    for k in range(1, nsub):
        t = k / nsub
        ids.append(_new_node(ax + (bx - ax) * t, ay + (by - ay) * t, e['net']))
    ids.append(b)
    chains.append(ids)
    for k in range(len(ids) - 1):
        si = len(segs)
        segs.append(dict(a=ids[k], b=ids[k + 1], layer=e['layer'], w=e['w'],
                         net=e['net'], c=ci))
        inc[ids[k]].append(si)
        inc[ids[k + 1]].append(si)
        chain_segs[ci].append(si)
NB = len(E)
for bi, e in enumerate(bridges):
    si = len(segs)
    segs.append(dict(a=e['a'], b=e['b'], layer=e['layer'], w=e['w'],
                     net=e['net'], c=NB + bi, bridge=True))
    inc[e['a']].append(si)
    inc[e['b']].append(si)
    chain_segs[NB + bi].append(si)
via_nids = [i for i in range(len(nodes)) if is_via[i]]
print(f'{len(P)} nodes, {len(segs)} sub-segments ({len(bridges)} bridges), '
      f'{len(MOVED)} moved parts, {len(part_anchors)} with bound copper')

# ---- pads (true shapes + local clearances) + courtyards ---------------------
# kicad's effective polygons are INSCRIBED (chords inside the true rounded
# shape), so raw they overestimate distance by the chord sagitta — up to the
# board's 5um polygonization error, position-dependent along the perimeter,
# which leaks through held-margin slides (measured: R92 pad 1.2um under rule
# at the gate). Inflate outward by that error so the model only ever errs
# conservative; the measure pass uses the same inflated shapes, so shipped
# margins stay consistent and no legal room is lost at rest.
POLY_ERR = 0.005


def inflate_poly(pts, r):
    n = len(pts)
    if n < 3:
        return [q[:] for q in pts]
    cx = sum(q[0] for q in pts) / n
    cy_ = sum(q[1] for q in pts) / n

    def nrm(a, b):
        ex, ey = b[0] - a[0], b[1] - a[1]
        L = math.hypot(ex, ey) or 1e-12
        nx, ny = ey / L, -ex / L
        mx, my = (a[0] + b[0]) / 2 - cx, (a[1] + b[1]) / 2 - cy_
        return (nx, ny) if nx * mx + ny * my > 0 else (-nx, -ny)
    out = []
    for i in range(n):
        p0, p1, p2 = pts[i - 1], pts[i], pts[(i + 1) % n]
        n1, n2 = nrm(p0, p1), nrm(p1, p2)
        mx, my = n1[0] + n2[0], n1[1] + n2[1]
        L = math.hypot(mx, my)
        if L < 1e-9:
            out.append([p1[0] + n1[0] * r, p1[1] + n1[1] * r])
        else:
            s = min(2 * r, 2 * r / L)          # miter: r / cos(half-angle)
            out.append([p1[0] + mx / L * s, p1[1] + my / L * s])
    return out

pads = O['pads']
pads_of = defaultdict(list)
n_true = 0
for pi, p in enumerate(pads):
    pg = PG.get(f"{p['ref']}|{p['num']}|{p['x']:.3f}|{p['y']:.3f}")
    if pg and pg['pts']:
        p['pts'] = inflate_poly(pg['pts'], POLY_ERR)
        p['lc'] = pg['lc']
        p['npth'] = pg['npth']
        n_true += 1
    else:
        p['lc'] = 0.0
        p['npth'] = False
    pads_of[p['ref']].append(pi)
    p['cx0'], p['cy0'] = p['x'], p['y']
    p['pts0'] = [q[:] for q in p['pts']]
    p['layset'] = frozenset(p['layers'])
if PG:
    print(f'true pad shapes for {n_true}/{len(pads)} pads, '
          f'{sum(1 for p in pads if p["lc"])} local clearances')

cy0, cy = {}, {}
for r, entry in CY.items():
    for lay, polys in entry.items():
        cy0[(r, lay)] = polys
        cy[(r, lay)] = [[q[:] for q in poly] for poly in polys]

u = {r: 0.0 for r in MOVED}


def set_part_geom(r):
    dx, dy = MOVED[r][0] * u[r], MOVED[r][1] * u[r]
    for pi in pads_of[r]:
        p = pads[pi]
        p['x'], p['y'] = p['cx0'] + dx, p['cy0'] + dy
        p['pts'] = [[q[0] + dx, q[1] + dy] for q in p['pts0']]
    for lay in ('F', 'B'):
        key = (r, lay)
        if key in cy0:
            cy[key] = [[[q[0] + dx, q[1] + dy] for q in poly]
                       for poly in cy0[key]]
    for nid in part_anchors[r]:
        P[nid][0] = anchor0[nid][0] + dx
        P[nid][1] = anchor0[nid][1] + dy


def _polys_bbox(polys, d):
    xs = [q[0] for poly in polys for q in poly]
    ys = [q[1] for poly in polys for q in poly]
    return (min(xs) + min(d[0], 0), min(ys) + min(d[1], 0),
            max(xs) + max(d[0], 0), max(ys) + max(d[1], 0))

cy_pairs = defaultdict(list)     # moved ref -> [(other_ref, lay)]
for r in MOVED:
    for lay in ('F', 'B'):
        if (r, lay) not in cy0:
            continue
        b1 = _polys_bbox(cy0[(r, lay)], MOVED[r])
        for r2 in CY:
            if r2 == r or (r2, lay) not in cy0:
                continue
            b2 = _polys_bbox(cy0[(r2, lay)], MOVED.get(r2, (0.0, 0.0)))
            if b1[0] - 1.0 < b2[2] and b2[0] - 1.0 < b1[2] and \
               b1[1] - 1.0 < b2[3] and b2[1] - 1.0 < b1[3]:
                cy_pairs[r].append((r2, lay))

# ---- grids -------------------------------------------------------------------
static_grid = defaultdict(list)
seg_grid = defaultdict(list)
vgrid = defaultdict(list)


def cells_for_box(x0, y0, x1, y1, r):
    for cx in range(int((x0 - r) // CELL), int((x1 + r) // CELL) + 1):
        for cy_ in range(int((y0 - r) // CELL), int((y1 + r) // CELL) + 1):
            yield (cx, cy_)


def build_static_grid():
    static_grid.clear()
    for ti, t in enumerate(O['tracks']):
        (ax, ay), (bx, by) = t['a'], t['b']
        for c in cells_for_box(min(ax, bx), min(ay, by),
                               max(ax, bx), max(ay, by), QR):
            static_grid[c].append(('T', ti))
    for vi, v in enumerate(O['vias']):
        for c in cells_for_box(v['x'], v['y'], v['x'], v['y'], QR):
            static_grid[c].append(('V', vi))
    for pi, p in enumerate(pads):
        xs = [q[0] for q in p['pts']]
        ys = [q[1] for q in p['pts']]
        for c in cells_for_box(min(xs), min(ys), max(xs), max(ys),
                               QR + p['drill'] / 2 + p['lc']):
            static_grid[c].append(('P', pi))


def build_dyn_grids():
    seg_grid.clear()
    vgrid.clear()
    for si, s in enumerate(segs):
        (ax, ay), (bx, by) = P[s['a']], P[s['b']]
        for c in cells_for_box(min(ax, bx), min(ay, by),
                               max(ax, bx), max(ay, by), QR):
            seg_grid[c].append(si)
    for nid in via_nids:
        x, y = P[nid]
        for c in cells_for_box(x, y, x, y, QR):
            vgrid[c].append(nid)

# ---- geometry primitives -----------------------------------------------------


def point_in_poly(x, y, pts):
    inside = False
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi:
            inside = not inside
        j = i
    return inside


def poly_seg_dist(pts, p1, p2):
    n = len(pts)
    d = min(seg_seg_dist(p1, p2, tuple(pts[i]), tuple(pts[(i + 1) % n]))
            for i in range(n))
    if d > 0 and point_in_poly((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2, pts):
        return 0.0
    return d


def poly_pt_dist(pts, c):
    n = len(pts)
    d = min(seg_seg_dist(c, c, tuple(pts[i]), tuple(pts[(i + 1) % n]))
            for i in range(n))
    if d > 0 and point_in_poly(c[0], c[1], pts):
        return 0.0
    return d


def poly_poly_dist(A, B):
    na, nb = len(A), len(B)
    d = min(seg_seg_dist(tuple(A[i]), tuple(A[(i + 1) % na]),
                         tuple(B[j]), tuple(B[(j + 1) % nb]))
            for i in range(na) for j in range(nb))
    if d > 0 and (point_in_poly(A[0][0], A[0][1], B) or
                  point_in_poly(B[0][0], B[0][1], A)):
        return 0.0
    return d

# ---- uniform copper-object margins -------------------------------------------
# descriptor: (kind, g1, g2, r_cu, layers, hole, lc)
#   kind 'seg':  g1,g2 endpoints, r_cu = w/2, layers = int
#   kind 'disc': g1 center,       r_cu = dia/2, layers = ALL (None)
#   kind 'poly': g1 pts, g2 center, r_cu = 0, layers = frozenset | ALL
#                (empty frozenset = no copper at all, e.g. NPTH)
#   hole = ((x,y), r) or None;  lc = local clearance override (max rules)
ALL = None
_KORD = {'seg': 0, 'disc': 1, 'poly': 2}


def _no_cu(l):
    return isinstance(l, frozenset) and not l


def _lay_overlap(a, b):
    if _no_cu(a) or _no_cu(b):
        return False
    if a is ALL or b is ALL:
        return True
    if isinstance(a, int):
        return a == b if isinstance(b, int) else a in b
    if isinstance(b, int):
        return b in a
    return bool(a & b)


def cu_dist(A, B):
    if _KORD[A[0]] > _KORD[B[0]]:
        A, B = B, A
    ka, kb = A[0], B[0]
    if ka == 'seg':
        if kb == 'seg':
            return seg_seg_dist(A[1], A[2], B[1], B[2]) - A[3] - B[3]
        if kb == 'disc':
            return seg_seg_dist(A[1], A[2], B[1], B[1]) - A[3] - B[3]
        return poly_seg_dist(B[1], A[1], A[2]) - A[3]
    if ka == 'disc':
        if kb == 'disc':
            return math.hypot(A[1][0] - B[1][0], A[1][1] - B[1][1]) - A[3] - B[3]
        return poly_pt_dist(B[1], A[1]) - A[3]
    return poly_poly_dist(A[1], B[1])


def _bbox_of(desc):
    if desc[0] == 'seg':
        (x1, y1), (x2, y2) = desc[1], desc[2]
        return min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)
    if desc[0] == 'disc':
        x, y = desc[1]
        return x, y, x, y
    xs = [q[0] for q in desc[1]]
    ys = [q[1] for q in desc[1]]
    return min(xs), min(ys), max(xs), max(ys)


def _c_margins(A, B, eff_c):
    """Copper-copper margins. For pairs involving a polygon, PER-EDGE tags:
    within one straight edge the model is exact line-to-line distance, and
    the polygonization bias only jumps between edges — one aggregate floor
    lets a sliding contact walk the perimeter through the bias jumps (the
    R92 leak), per-edge floors re-guard every new edge it reaches."""
    if _KORD[A[0]] > _KORD[B[0]]:
        A, B = B, A
    if B[0] != 'poly':
        return [('c', cu_dist(A, B) - eff_c)]
    H = eff_c + GF_CUT + 0.05                  # yield horizon
    out = []
    if A[0] != 'poly':
        pts = B[1]
        n = len(pts)
        inside = None
        if A[0] == 'seg':
            r = A[3]
            mid = ((A[1][0] + A[2][0]) / 2, (A[1][1] + A[2][1]) / 2)
            dj = [seg_seg_dist(A[1], A[2], tuple(pts[j]),
                               tuple(pts[(j + 1) % n])) for j in range(n)]
        else:
            r = A[3]
            mid = A[1]
            dj = [seg_seg_dist(A[1], A[1], tuple(pts[j]),
                               tuple(pts[(j + 1) % n])) for j in range(n)]
        if min(dj) > 0 and point_in_poly(mid[0], mid[1], pts):
            return [(('c', 'in'), -eff_c - r)]
        for j in range(n):
            m = dj[j] - r - eff_c
            if m < H - eff_c:
                out.append((('c', j), m))
        return out
    pa, pb = A[1], B[1]
    na, nb = len(pa), len(pb)
    if point_in_poly(pa[0][0], pa[0][1], pb) or \
            point_in_poly(pb[0][0], pb[0][1], pa):
        return [(('c', 'in'), -eff_c)]
    for i in range(na):
        e1a, e1b = tuple(pa[i]), tuple(pa[(i + 1) % na])
        for j in range(nb):
            m = seg_seg_dist(e1a, e1b, tuple(pb[j]),
                             tuple(pb[(j + 1) % nb])) - eff_c
            if m < H - eff_c:
                out.append((('c', i, j), m))
    return out


def pair_margins(A, B, hole_only=False):
    """[(tag, margin)] for canonical pair (A first). Tags: c copper-copper
    (per-edge for polygons, see _c_margins), h1/h2 hole-of-one vs
    copper-of-other, hh hole-hole (solve.py's rules). Local clearance
    overrides raise both rules (max semantics, e.g. H4)."""
    eff_c = max(CLR, A[6], B[6])
    eff_h = max(HOLE_CLR, A[6], B[6])
    ha, hb = A[5], B[5]
    if A[0] == 'poly' or B[0] == 'poly':        # fast bbox reject
        ax0, ay0, ax1, ay1 = _bbox_of(A)
        bx0, by0, bx1, by1 = _bbox_of(B)
        gap = math.hypot(max(bx0 - ax1, ax0 - bx1, 0),
                         max(by0 - ay1, ay0 - by1, 0))
        slack = A[3] + B[3] + (ha[1] if ha else 0) + (hb[1] if hb else 0)
        if gap - slack > max(eff_c, eff_h) + GF_CUT + 0.06:
            return []
    out = []
    if not hole_only and _lay_overlap(A[4], B[4]):
        out.extend(_c_margins(A, B, eff_c))
    if ha and not hole_only and not _no_cu(B[4]):
        out.append(('h1', cu_dist(('disc', ha[0], None, ha[1], ALL, None, 0),
                                  B) - eff_h))
    if hb and not hole_only and not _no_cu(A[4]):
        out.append(('h2', cu_dist(('disc', hb[0], None, hb[1], ALL, None, 0),
                                  A) - eff_h))
    if ha and hb:
        out.append(('hh', math.hypot(ha[0][0] - hb[0][0], ha[0][1] - hb[0][1])
                    - ha[1] - hb[1] - eff_h))
    return out

# ---- movable-element descriptors (mv-aware) ---------------------------------


def npos(nid, mvn, a):
    x, y = P[nid]
    d = mvn.get(nid)
    return (x + d[0] * a, y + d[1] * a) if d else (x, y)


def seg_desc(si, mvn, a):
    s = segs[si]
    return ('seg', npos(s['a'], mvn, a), npos(s['b'], mvn, a),
            s['w'] / 2, s['layer'], None, 0.0)


def via_desc(nid, mvn, a):
    c = npos(nid, mvn, a)
    return ('disc', c, None, via_dia[nid] / 2, ALL,
            (c, via_drill[nid] / 2) if via_drill[nid] else None, 0.0)


def pad_desc(pi, mvr, a):
    p = pads[pi]
    d = mvr.get(p['ref'])
    if d:
        dx, dy = d[0] * a, d[1] * a
        pts = [[q[0] + dx, q[1] + dy] for q in p['pts']]
        c = (p['x'] + dx, p['y'] + dy)
    else:
        pts, c = p['pts'], (p['x'], p['y'])
    if p['npth']:
        lays = frozenset()                     # no copper: hole rules only
    else:
        lays = ALL if p['drill'] else p['layset']
    return ('poly', pts, c, 0.0, lays,
            (c, p['drill'] / 2) if p['drill'] else None, p['lc'])

track_desc = [('seg', tuple(t['a']), tuple(t['b']), t['w'] / 2, t['layer'],
               None, 0.0) for t in O['tracks']]
svia_desc = [('disc', (v['x'], v['y']), None, v['dia'] / 2, ALL,
              ((v['x'], v['y']), v['drill'] / 2), 0.0) for v in O['vias']]


def edge_margin(xys, hw):
    ex0, ey0, ex1, ey1 = EDGE
    return min(min(x - ex0, ex1 - x, y - ey0, ey1 - y) for x, y in xys) \
        - ECLR - hw

# ---- attachment (same-net contact persistence) -------------------------------
# att holds keys ('AT', <base pair key>) recorded touching at s=0; the pair
# must keep at least one contact (max-aggregated in feasible()). GND is
# exempt: the pour reconnects whatever slides apart.


def at_margin(A, B):
    return -cu_dist(A, B) - OV


def _at(base_key, A, B, blk):
    """Yield an attachment margin if this pair is a recorded contact."""
    key = ('AT',) + base_key
    if key in gf:
        m = at_margin(A, B)
        if m > -0.1:                           # participation: near contact
            return [(key, m, blk)]
    return []

# ---- pair enumeration per moving element ------------------------------------


def gen_seg(si, mvn, mvr, a):
    s = segs[si]
    ci, net, lay, w = s['c'], s['net'], s['layer'], s['w']
    A = seg_desc(si, mvn, a)
    bx0, by0, bx1, by1 = _bbox_of(A)
    seen = set()
    for cell in cells_for_box(bx0, by0, bx1, by1, QR):
        for sj in seg_grid.get(cell, ()):
            if sj == si or ('S', sj) in seen:
                continue
            seen.add(('S', sj))
            t = segs[sj]
            if t['c'] == ci or t['layer'] != lay:
                continue
            if t['net'] == net:
                if net != 'GND':
                    yield from _at(('SS', min(ci, t['c']), max(ci, t['c'])),
                                   A, seg_desc(sj, mvn, a), ('S', sj))
                continue
            m = seg_seg_dist(A[1], A[2], npos(t['a'], mvn, a),
                             npos(t['b'], mvn, a)) - CLR - (w + t['w']) / 2
            yield ('SS', min(ci, t['c']), max(ci, t['c']), 'c'), m, ('S', sj)
        for nj in vgrid.get(cell, ()):
            if ('N', nj) in seen:
                continue
            seen.add(('N', nj))
            if node_net[nj] == net:
                if net != 'GND':
                    yield from _at(('SN', ci, nj), A, via_desc(nj, mvn, a),
                                   ('N', nj))
                continue
            for tag, m in pair_margins(A, via_desc(nj, mvn, a)):
                yield ('SN', ci, nj, tag), m, ('N', nj)
        for kind, idx in static_grid.get(cell, ()):
            if (kind, idx) in seen:
                continue
            seen.add((kind, idx))
            if kind == 'T':
                t = O['tracks'][idx]
                if t['layer'] != lay:
                    continue
                if t['net'] == net:
                    if net != 'GND':
                        yield from _at(('ST', ci, idx), A, track_desc[idx],
                                       ('T', idx))
                    continue
                m = seg_seg_dist(A[1], A[2], tuple(t['a']), tuple(t['b'])) \
                    - CLR - (w + t['w']) / 2
                yield ('ST', ci, idx, 'c'), m, ('T', idx)
            elif kind == 'V':
                if O['vias'][idx]['net'] == net:
                    if net != 'GND':
                        yield from _at(('SV', ci, idx), A, svia_desc[idx],
                                       ('V', idx))
                    continue
                for tag, m in pair_margins(A, svia_desc[idx]):
                    yield ('SV', ci, idx, tag), m, ('V', idx)
            else:
                p = pads[idx]
                if p['net'] and p['net'] == net:
                    if net != 'GND':
                        yield from _at(('SP', ci, idx), A,
                                       pad_desc(idx, mvr, a), ('P', idx))
                    continue
                for tag, m in pair_margins(A, pad_desc(idx, mvr, a)):
                    yield ('SP', ci, idx, tag), m, ('P', idx)
    if EDGE:
        yield ('SE', ci), edge_margin((A[1], A[2]), w / 2), ('E',)


def gen_via(nid, mvn, mvr, a):
    net = node_net[nid]
    A = via_desc(nid, mvn, a)
    x, y = A[1]
    seen = set()
    for cell in cells_for_box(x, y, x, y, QR):
        for sj in seg_grid.get(cell, ()):
            if ('S', sj) in seen:
                continue
            seen.add(('S', sj))
            t = segs[sj]
            if t['net'] == net:
                if net != 'GND':
                    yield from _at(('SN', t['c'], nid),
                                   seg_desc(sj, mvn, a), A, ('S', sj))
                continue
            for tag, m in pair_margins(seg_desc(sj, mvn, a), A):
                yield ('SN', t['c'], nid, tag), m, ('S', sj)
        for nj in vgrid.get(cell, ()):
            if nj == nid or ('N', nj) in seen:
                continue
            seen.add(('N', nj))
            lo, hi = min(nid, nj), max(nid, nj)
            same = node_net[nj] == net
            if same and net != 'GND':
                yield from _at(('NN', lo, hi), A, via_desc(nj, mvn, a),
                               ('N', nj))
            for tag, m in pair_margins(via_desc(lo, mvn, a),
                                       via_desc(hi, mvn, a), hole_only=same):
                yield ('NN', lo, hi, tag), m, ('N', nj)
        for kind, idx in static_grid.get(cell, ()):
            if (kind, idx) in seen:
                continue
            seen.add((kind, idx))
            if kind == 'T':
                t = O['tracks'][idx]
                if t['net'] == net:
                    if net != 'GND':
                        yield from _at(('NT', nid, idx), A, track_desc[idx],
                                       ('T', idx))
                    continue
                for tag, m in pair_margins(A, track_desc[idx]):
                    yield ('NT', nid, idx, tag), m, ('T', idx)
            elif kind == 'V':
                v = O['vias'][idx]
                same = v['net'] == net
                if same and net != 'GND':
                    yield from _at(('NV', nid, idx), A, svia_desc[idx],
                                   ('V', idx))
                for tag, m in pair_margins(A, svia_desc[idx], hole_only=same):
                    yield ('NV', nid, idx, tag), m, ('V', idx)
            else:
                p = pads[idx]
                if p['net'] and p['net'] == net:
                    if net != 'GND':
                        yield from _at(('NP', nid, idx), A,
                                       pad_desc(idx, mvr, a), ('P', idx))
                    continue
                for tag, m in pair_margins(A, pad_desc(idx, mvr, a)):
                    yield ('NP', nid, idx, tag), m, ('P', idx)
    if EDGE:
        yield ('VE', nid), edge_margin((A[1],), via_dia[nid] / 2), ('E',)


def gen_pad(pi, mvn, mvr, a):
    p = pads[pi]
    net, ref = p['net'], p['ref']
    A = pad_desc(pi, mvr, a)
    bx0, by0, bx1, by1 = _bbox_of(A)
    seen = set()
    for cell in cells_for_box(bx0, by0, bx1, by1, QR + p['drill'] / 2 + p['lc']):
        for sj in seg_grid.get(cell, ()):
            if ('S', sj) in seen:
                continue
            seen.add(('S', sj))
            t = segs[sj]
            if net and t['net'] == net:
                if net != 'GND':
                    yield from _at(('SP', t['c'], pi),
                                   seg_desc(sj, mvn, a), A, ('S', sj))
                continue
            for tag, m in pair_margins(seg_desc(sj, mvn, a), A):
                yield ('SP', t['c'], pi, tag), m, ('S', sj)
        for nj in vgrid.get(cell, ()):
            if ('N', nj) in seen:
                continue
            seen.add(('N', nj))
            if net and node_net[nj] == net:
                if net != 'GND':
                    yield from _at(('NP', nj, pi), via_desc(nj, mvn, a), A,
                                   ('N', nj))
                continue
            for tag, m in pair_margins(via_desc(nj, mvn, a), A):
                yield ('NP', nj, pi, tag), m, ('N', nj)
        for kind, idx in static_grid.get(cell, ()):
            if (kind, idx) in seen:
                continue
            seen.add((kind, idx))
            if kind == 'T':
                t = O['tracks'][idx]
                if net and t['net'] == net:
                    if net != 'GND':
                        yield from _at(('PT', pi, idx), A, track_desc[idx],
                                       ('T', idx))
                    continue
                for tag, m in pair_margins(A, track_desc[idx]):
                    yield ('PT', pi, idx, tag), m, ('T', idx)
            elif kind == 'V':
                if net and O['vias'][idx]['net'] == net:
                    if net != 'GND':
                        yield from _at(('PV', pi, idx), A, svia_desc[idx],
                                       ('V', idx))
                    continue
                for tag, m in pair_margins(A, svia_desc[idx]):
                    yield ('PV', pi, idx, tag), m, ('V', idx)
            else:
                if idx == pi:
                    continue
                q = pads[idx]
                if q['ref'] == ref:
                    continue
                if net and q['net'] == net:
                    if net != 'GND':
                        yield from _at(('PP', min(pi, idx), max(pi, idx)),
                                       A, pad_desc(idx, mvr, a), ('P', idx))
                    continue
                lo, hi = min(pi, idx), max(pi, idx)
                for tag, m in pair_margins(pad_desc(lo, mvr, a),
                                           pad_desc(hi, mvr, a)):
                    yield ('PP', lo, hi, tag), m, ('P', idx)
    if EDGE and not p['npth']:
        yield ('PE', pi), edge_margin(A[1], 0.0), ('E',)


def crt_margin(A, B):
    d = poly_poly_dist(A, B)
    return d if d > 0 else -0.05


def gen_crt(ref, mvr, a):
    d = mvr.get(ref)
    for r2, lay in cy_pairs[ref]:
        for pa in cy[(ref, lay)]:
            if d:
                pa = [[q[0] + d[0] * a, q[1] + d[1] * a] for q in pa]
            for pb in cy[(r2, lay)]:
                lo, hi = min(ref, r2), max(ref, r2)
                yield ('CY', lo, hi, lay), crt_margin(pa, pb), ('C', r2)


def gen_part(ref, mvn, mvr, a):
    for pi in pads_of[ref]:
        yield from gen_pad(pi, mvn, mvr, a)
    seen = set()
    for nid in part_anchors[ref]:
        for si in inc[nid]:
            if si not in seen:
                seen.add(si)
                yield from gen_seg(si, mvn, mvr, a)
        if is_via[nid]:
            yield from gen_via(nid, mvn, mvr, a)
    yield from gen_crt(ref, mvr, a)


def gen_node(nid, mvn, a):
    for si in inc[nid]:
        yield from gen_seg(si, mvn, {}, a)
    if is_via[nid]:
        yield from gen_via(nid, mvn, {}, a)

# ---- floors: grandfathered shipped margins + guard band ---------------------
gf = {}


def floor_of(key):
    if key[0] == 'AT':
        f = gf.get(key)
        return -1e9 if f is None else min(f, 0.0)
    g = CRT_GUARD if key[0] == 'CY' else GUARD
    f = gf.get(key)
    return g if f is None else min(f, g)


def measure():
    """Record shipped margins (min per clearance pair) and shipped same-net
    contacts (max per attachment pair) at s=0. Contacts are discovered by a
    dedicated sweep: generators only *check* attachment keys already in gf."""
    build_static_grid()
    build_dyn_grids()

    # contacts first: same-net touching pairs -> AT keys seeded into gf so
    # the recording sweep (and later moves) will evaluate them
    def seed(base_key, A, B):
        m = at_margin(A, B)
        if m >= -OV - 0.0005:                  # copper overlaps (touches)
            key = ('AT',) + base_key
            if m > gf.get(key, -1e9):
                gf[key] = m
    for si, s in enumerate(segs):
        net, ci, lay = s['net'], s['c'], s['layer']
        if net == 'GND':
            continue
        A = seg_desc(si, {}, 0)
        bx0, by0, bx1, by1 = _bbox_of(A)
        for cell in cells_for_box(bx0, by0, bx1, by1, 0.1):
            for sj in seg_grid.get(cell, ()):
                t = segs[sj]
                if sj != si and t['c'] != ci and t['net'] == net \
                        and t['layer'] == lay:
                    seed(('SS', min(ci, t['c']), max(ci, t['c'])),
                         A, seg_desc(sj, {}, 0))
            for nj in vgrid.get(cell, ()):
                if node_net[nj] == net:
                    seed(('SN', ci, nj), A, via_desc(nj, {}, 0))
            for kind, idx in static_grid.get(cell, ()):
                if kind == 'T' and O['tracks'][idx]['net'] == net \
                        and O['tracks'][idx]['layer'] == lay:
                    seed(('ST', ci, idx), A, track_desc[idx])
                elif kind == 'V' and O['vias'][idx]['net'] == net:
                    seed(('SV', ci, idx), A, svia_desc[idx])
                elif kind == 'P' and pads[idx]['net'] == net:
                    seed(('SP', ci, idx), A, pad_desc(idx, {}, 0))
    for r in MOVED:
        for pi in pads_of[r]:
            p = pads[pi]
            net = p['net']
            if not net or net == 'GND':
                continue
            A = pad_desc(pi, {}, 0)
            bx0, by0, bx1, by1 = _bbox_of(A)
            for cell in cells_for_box(bx0, by0, bx1, by1, 0.1):
                for nj in vgrid.get(cell, ()):
                    if node_net[nj] == net:
                        seed(('NP', nj, pi), via_desc(nj, {}, 0), A)
                for kind, idx in static_grid.get(cell, ()):
                    if kind == 'T' and O['tracks'][idx]['net'] == net:
                        seed(('PT', pi, idx), A, track_desc[idx])
                    elif kind == 'V' and O['vias'][idx]['net'] == net:
                        seed(('PV', pi, idx), A, svia_desc[idx])
    for nid in via_nids:
        net = node_net[nid]
        if net == 'GND':
            continue
        A = via_desc(nid, {}, 0)
        x, y = A[1]
        for cell in cells_for_box(x, y, x, y, 0.1):
            for nj in vgrid.get(cell, ()):
                if nj != nid and node_net[nj] == net:
                    seed(('NN', min(nid, nj), max(nid, nj)),
                         via_desc(min(nid, nj), {}, 0),
                         via_desc(max(nid, nj), {}, 0))
            for kind, idx in static_grid.get(cell, ()):
                if kind == 'T' and O['tracks'][idx]['net'] == net:
                    seed(('NT', nid, idx), A, track_desc[idx])
                elif kind == 'V' and O['vias'][idx]['net'] == net:
                    seed(('NV', nid, idx), A, svia_desc[idx])
                elif kind == 'P' and pads[idx]['net'] == net:
                    seed(('NP', nid, idx), A, pad_desc(idx, {}, 0))
    n_at = sum(1 for k in gf if k[0] == 'AT')

    # clearance margins (AT keys re-yielded by generators are max-recorded)
    def rec(gen):
        for key, m, _ in gen:
            if key[0] == 'AT':
                continue                       # already seeded above
            if m < GF_CUT and m < gf.get(key, 1e9):
                gf[key] = m
    for si in range(len(segs)):
        rec(gen_seg(si, {}, {}, 0.0))
    for nid in via_nids:
        rec(gen_via(nid, {}, {}, 0.0))
    for r in MOVED:
        for pi in pads_of[r]:
            rec(gen_pad(pi, {}, {}, 0.0))
        rec(gen_crt(r, {}, 0.0))
    ncl = len(gf) - n_at
    neg = sum(1 for k, v in gf.items() if k[0] != 'AT' and v < 0)
    print(f'grandfathered {ncl} shipped at/below-guard pairs '
          f'({neg} below model rule — bias absorbed), '
          f'{n_at} same-net contacts held as attachments')

# ---- feasibility line search -------------------------------------------------


def feasible(gen, collect):
    ok = True
    at_best = {}
    for key, m, blk in gen:
        if key[0] == 'AT':
            if key not in at_best or m > at_best[key][0]:
                at_best[key] = (m, blk)
            continue
        if m < floor_of(key) - EPS:
            if collect is None:
                return False
            ok = False
            collect.append((key, m, blk))
    for key, (m, blk) in at_best.items():
        if m < floor_of(key) - EPS:
            if collect is None:
                return False
            ok = False
            collect.append((key, m, blk))
    return ok


def line_search(gen_at):
    clamps = []
    if feasible(gen_at(1.0), clamps):
        return 1.0, []
    lo, hi = 0.0, 1.0
    for _ in range(6):
        mid = (lo + hi) / 2
        if feasible(gen_at(mid), None):
            lo = mid
        else:
            hi = mid
    return lo, clamps

# ---- pressure ----------------------------------------------------------------
press = defaultdict(lambda: [0.0, 0.0])
absorb = defaultdict(int)       # (mover, blocker descr) -> count, stall evidence


def blk_info(blk):
    k = blk[0]
    if k == 'S':
        s = segs[blk[1]]
        return dict(kind='dyn_seg', net=s['net'], layer=s['layer'],
                    pos=[round((P[s['a']][0] + P[s['b']][0]) / 2, 3),
                         round((P[s['a']][1] + P[s['b']][1]) / 2, 3)])
    if k == 'N':
        return dict(kind='dyn_via', net=node_net[blk[1]],
                    pos=[round(P[blk[1]][0], 3), round(P[blk[1]][1], 3)])
    if k == 'T':
        t = O['tracks'][blk[1]]
        return dict(kind='static_track', net=t['net'], layer=t['layer'],
                    pos=[round((t['a'][0] + t['b'][0]) / 2, 3),
                         round((t['a'][1] + t['b'][1]) / 2, 3)])
    if k == 'V':
        v = O['vias'][blk[1]]
        return dict(kind='static_via', net=v['net'],
                    pos=[round(v['x'], 3), round(v['y'], 3)])
    if k == 'P':
        p = pads[blk[1]]
        return dict(kind='pad', ref=f"{p['ref']}.{p['num']}", net=p['net'],
                    pos=[round(p['x'], 3), round(p['y'], 3)])
    if k == 'C':
        return dict(kind='courtyard', ref=blk[1])
    return dict(kind='board_edge')


def _push(nid, bx, by, mag, toward=True):
    dx, dy = bx - P[nid][0], by - P[nid][1]
    L = math.hypot(dx, dy) or 1.0
    if not toward:
        dx, dy = -dx, -dy
    pr = press[nid]
    pr[0] += dx / L * mag
    pr[1] += dy / L * mag


def deposit(clamps, mover_mid, fx, fy, mover_name):
    for key, m, blk in clamps:
        need = floor_of(key) - m + 0.001
        if need <= 0:
            continue
        mag = min(STEP, need)
        k = blk[0]
        is_at = key[0] == 'AT'
        info = blk_info(blk)
        if k == 'S' or k == 'N':
            # dynamic blocker: push it away (clearance) / pull it along (AT)
            tgt = [blk[1]] if k == 'N' else [segs[blk[1]]['a'],
                                             segs[blk[1]]['b']]
            bx, by = info['pos']
            dx, dy = bx - mover_mid[0], by - mover_mid[1]
            L = math.hypot(dx, dy)
            if L < 1e-6:
                dx, dy, L = fx, fy, 1.0
            if is_at:
                dx, dy = -dx, -dy
            for nid in tgt:
                if nid not in fixed and nid not in part_of:
                    pr = press[nid]
                    pr[0] += dx / L * mag
                    pr[1] += dy / L * mag
            continue
        if is_at and key[1] in ('SP', 'ST', 'SV'):
            # unmovable contact, but the contact-keeper is the mover's own
            # chain: glue its free nodes toward the contact — the trace
            # stays soldered and stretches while the part tows it
            ci = key[2]
            bx, by = info['pos']
            glued = False
            for si in chain_segs.get(ci, ()):
                s = segs[si]
                mx = (P[s['a']][0] + P[s['b']][0]) / 2
                my = (P[s['a']][1] + P[s['b']][1]) / 2
                if math.hypot(mx - bx, my - by) < 1.0:
                    for nid in (s['a'], s['b']):
                        if nid not in fixed and nid not in part_of:
                            _push(nid, bx, by, mag)
                            glued = True
            if glued:
                continue
        elif is_at and key[1] in ('NT', 'NV', 'NP'):
            nid = key[2]
            if nid not in fixed and nid not in part_of:
                _push(nid, info['pos'][0], info['pos'][1], mag)
                continue
        absorb[(mover_name, info['kind'],
                info.get('net') or info.get('ref', ''))] += 1

# ---- passes ------------------------------------------------------------------
_mdx = sum(d[0] for d in MOVED.values()) / len(MOVED)
_mdy = sum(d[1] for d in MOVED.values()) / len(MOVED)
_L = math.hypot(_mdx, _mdy) or 1.0
UX, UY = _mdx / _L, _mdy / _L


def part_mid(r):
    xs = [pads[pi]['x'] for pi in pads_of[r]]
    ys = [pads[pi]['y'] for pi in pads_of[r]]
    return sum(xs) / len(xs), sum(ys) / len(ys)

order = sorted(MOVED, key=lambda r: -(part_mid(r)[0] * UX + part_mid(r)[1] * UY))
print(f'convoy order along ({UX:+.2f},{UY:+.2f}): '
      f'{order[0]} first ... {order[-1]} last')
blocked_log = {}


def part_pass():
    adv, nblk = 0.0, 0
    for ref in order:
        if u[ref] >= 1.0:
            continue
        d = MOVED[ref]
        L = math.hypot(*d)
        du = min(1.0 - u[ref], STEP / L)
        delta = (d[0] * du, d[1] * du)
        mvn = {nid: delta for nid in part_anchors[ref]}
        mvr = {ref: delta}
        al, clamps = line_search(lambda a: gen_part(ref, mvn, mvr, a))
        if al == 1.0:                       # full step (or the arrival snap)
            u[ref] = min(1.0, u[ref] + du)
            set_part_geom(ref)
            adv += du * L
        elif al * du * L > 5e-5:
            u[ref] = min(1.0, u[ref] + al * du)
            set_part_geom(ref)
            adv += al * du * L
        else:
            al = 0.0
        if al < 1.0:
            nblk += 1
            blocked_log[ref] = clamps
            deposit(clamps, part_mid(ref), delta[0] / (du * L),
                    delta[1] / (du * L), ref)
        else:
            blocked_log.pop(ref, None)
    return adv, nblk


def stretch_vec(nid):
    vx = vy = 0.0
    cap = SUB * 1.6
    for si in inc[nid]:
        s = segs[si]
        if s.get('bridge'):
            continue
        o = s['b'] if s['a'] == nid else s['a']
        dx, dy = P[o][0] - P[nid][0], P[o][1] - P[nid][1]
        L = math.hypot(dx, dy)
        if L > cap:
            pull = (L - cap) / 2
            vx += dx / L * pull
            vy += dy / L * pull
    return vx, vy


def copper_pass():
    moved = 0.0
    cap = SUB * 1.6
    for _ in range(NCOPPER):
        build_dyn_grids()
        todo = sorted(press, key=lambda n: -(press[n][0] ** 2 + press[n][1] ** 2))
        in_todo = set(todo)
        for s in segs:
            if s.get('bridge'):
                continue
            (ax, ay), (bx, by) = P[s['a']], P[s['b']]
            if math.hypot(bx - ax, by - ay) > cap:
                for nid in (s['a'], s['b']):
                    if nid not in in_todo:
                        in_todo.add(nid)
                        todo.append(nid)
        for nid in todo:
            if nid in fixed or nid in part_of:
                press.pop(nid, None)
                continue
            pv = press.pop(nid, None)
            if pv is not None and (pv[0] or pv[1]):
                vx, vy = pv                    # pressure wins: no stretch tug-of-war
            else:
                vx, vy = stretch_vec(nid)
            Lv = math.hypot(vx, vy)
            if Lv < 2e-4:
                continue
            if Lv > STEP:
                vx, vy = vx / Lv * STEP, vy / Lv * STEP
                Lv = STEP
            mvn = {nid: (vx, vy)}
            al, clamps = line_search(lambda a: gen_node(nid, mvn, a))
            if al > 0:
                P[nid][0] += vx * al
                P[nid][1] += vy * al
                moved += al * Lv
            if al < 1.0 and clamps:
                deposit(clamps, (P[nid][0], P[nid][1]), vx / Lv, vy / Lv,
                        f'node{nid}')
    return moved

# ---- trajectory --------------------------------------------------------------
cps = []


def capture():
    s = sum(u.values()) / len(u)
    cps.append(dict(s=s, P=[p[:] for p in P], sref={r: u[r] for r in u}))
    print(f'  checkpoint {len(cps)}: s={s:.3f}', flush=True)

t0 = time.time()
measure()
if '--measure-only' in sys.argv:
    sys.exit(0)
capture()                                   # s=0: pristine roundtrip baseline

hist = []
idle = 0
cyc = 0
while cyc < CYCLES:
    cyc += 1
    build_static_grid()
    build_dyn_grids()
    adv, nblk = part_pass()
    cu = copper_pass()
    mean_u = sum(u.values()) / len(u)
    arrived = sum(1 for r in u if u[r] >= 1.0)
    hist.append(dict(cycle=cyc, mean_u=round(mean_u, 4), arrived=arrived,
                     blocked=nblk, adv=round(adv, 4), copper=round(cu, 4)))
    while len(cps) <= SNAPSHOTS and mean_u >= len(cps) / SNAPSHOTS - 1e-9:
        capture()
    if cyc % 5 == 0 or cyc == 1:
        print(f'cycle {cyc}: mean_u={mean_u:.3f} arrived={arrived}/{len(u)} '
              f'blocked={nblk} adv={adv:.3f}mm copper={cu:.3f}mm '
              f'[{time.time() - t0:.0f}s]', flush=True)
    if arrived == len(u):
        print(f'all {len(u)} parts arrived after {cyc} cycles')
        break
    idle = idle + 1 if adv < 5e-4 else 0
    if idle >= STALL_WIN:
        print(f'STALL after {cyc} cycles ({STALL_WIN} without part advance): '
              f'mean_u={mean_u:.3f}, {len(u) - arrived} parts short of target')
        break

if not cps or cps[-1]['s'] < sum(u.values()) / len(u) - 1e-9:
    capture()                               # final state (stall or arrival)

save_json('morph.json', dict(checkpoints=cps, chains=chains))

stalled = [dict(ref=r, u=round(u[r], 4),
                rem_mm=round((1 - u[r]) * math.hypot(*MOVED[r]), 3),
                blockers=[dict(key=str(k), margin=round(m, 4),
                               floor=round(floor_of(k), 4), **blk_info(b))
                          for k, m, b in blocked_log.get(r, [])])
           for r in order if u[r] < 1.0]
save_json('creep_report.json', dict(
    cycles=cyc, mean_u=round(sum(u.values()) / len(u), 4),
    arrived=sum(1 for r in u if u[r] >= 1.0), parts=len(u),
    u={r: round(u[r], 4) for r in order},
    stalled=stalled,
    absorb=[dict(mover=a, blocker=b, what=c, hits=n)
            for (a, b, c), n in sorted(absorb.items(), key=lambda kv: -kv[1])],
    history=hist))
print(f'wrote morph.json ({len(cps)} checkpoints) + creep_report.json '
      f'({len(stalled)} stalled parts) in {time.time() - t0:.0f}s — run morph.py')
