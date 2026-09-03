#!/usr/bin/env python3
"""settle: feasibility-projected settling of a board region.

The bag-of-chips engine. Movable units — footprint assemblies (pads + bound
copper endpoints + courtyard), rigid groups, free trace nodes, vias, and
GHOSTS (parts to be born, inflating from a speck to full size at their
target pose) — move only as far as the DRC clearance geometry admits: every
proposed move gets a line search for the largest fraction whose margins stay
above per-pair floors, and a unit with no room does not move.

Drive (what wants to move):
  * ghost inflation: a ghost's pads and courtyard scale up about its centre;
    what it clamps against gets pushed radially — parts included;
  * targets / shoves: per-part displacement vectors from the config;
  * pressure: a blocked mover deposits displacement demand on its blockers
    (dynamic copper AND movable parts), which yield as far as THEIR floors
    allow — packed corridors creep like a traffic jam;
  * springs: per-net copper length is tracked; nets grown past their
    allowance pull their nodes and end parts back (stiffness k), and a hard
    cap per net enters the line search so no step can exceed it.

Admissibility (what DRC allows) is the creep.py oracle: true pad polygons
with per-edge floors, netclass clearances with max semantics, hole rules,
courtyards, board edge, and same-net contacts held as attachments. Pairs
below floor at the start (or made so by a ghost speck sitting on copper)
obey a NON-WORSENING rule: a move is admissible if such a pair's margin does
not decrease. Ghost pairs are never grandfathered, so a ghost only grows
where the rule holds.

Stalls consume their own worklist (mid-flight reroutes of clamped chains);
what remains is reported: blockers, board-edge pressure (the stretch
signal), nets at their length cap.

    python3 settle.py CONFIG.json [--cycles N] [--step MM] [--snapshots N]
                      [--guard MM] [--copper-sweeps N] [--measure-only]
                      [--no-reroute]
Writes data/<name>/traj.json + settle_report.json — run emit.py to gate.
"""
import heapq, math, sys, time
from collections import defaultdict
from common import load_config, load_json, save_json, seg_seg_dist

cfg = load_config()
M = load_json(cfg, 'model.json')


def _arg(flag, default, cast=float):
    return cast(sys.argv[sys.argv.index(flag) + 1]) if flag in sys.argv else default

R = M['rules']
CLR = R['clearance']
HOLE_CLR = R['hole_clearance']
EDGE = R['board_edge']
ECLR = R['edge_clearance']
REGION = M['region']
NETS = M['nets']

SUB = 0.8                                   # chain subdivision pitch, mm
STEP = _arg('--step', cfg.get('step', 0.05))
GUARD = _arg('--guard', R.get('guard', 0.015))
CRT_GUARD = 0.003
GF_CUT = 0.02
OV = 0.02
SNAPSHOTS = _arg('--snapshots', cfg.get('snapshots', 10), int)
CYCLES = _arg('--cycles', cfg.get('cycles', 600), int)
NCOPPER = _arg('--copper-sweeps', 6, int)
RELAX = _arg('--relax', 25, int)            # settle cycles after arrival
STALL_WIN = 40
IDLE_ADV = 0.1 * STEP           # unit advance per cycle below this = idle (springs jitter forever)
EPS = 1e-9
CELL = 1.2
QR = 0.9
G_MIN = 0.05                                # ghost birth scale
G_ANCHOR = cfg.get('ghost_anchor', 0.3)     # ghost pull-back toward its site
NO_RR = '--no-reroute' in sys.argv


def net_clr(net):
    return NETS.get(net, NETS['']).get('clr', CLR)


def net_el(net):
    return NETS.get(net, NETS[''])

# ---- graph -> solver state ---------------------------------------------------
nodes = M['nodes']
E = M['edges']
parts = M['parts']
movable = {r for r, p in parts.items() if p['movable']}

P = [[n['x'], n['y']] for n in nodes]
is_via = [n['kind'] == 'via' for n in nodes]
via_dia = [n.get('dia') or 0.0 for n in nodes]
via_drill = [n.get('drill') or 0.0 for n in nodes]
node_net = [n.get('net') or '' for n in nodes]

fixed = set()
part_of = {}
part_anchors = defaultdict(list)
anchor0 = {}
for i, n in enumerate(nodes):
    if n.get('bind'):
        r = n['bind'][0]
        if r in movable:
            part_of[i] = r
            part_anchors[r].append(i)
            anchor0[i] = (n['x'], n['y'])
        else:
            fixed.add(i)
    elif n.get('pin'):
        fixed.add(i)

segs = []
chains = []
inc = defaultdict(list)
chain_segs = defaultdict(list)


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
via_nids = [i for i in range(len(nodes)) if is_via[i]]
D = {r: [0.0, 0.0] for r in movable}
print(f'{len(P)} nodes, {len(segs)} sub-segments, {len(movable)} movable parts '
      f'({len(part_anchors)} with bound copper), {len(fixed)} fixed nodes')

# ---- pads (true shapes, inflated by the polygonization error) + ghosts ------
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
            s = min(2 * r, 2 * r / L)
            out.append([p1[0] + mx / L * s, p1[1] + my / L * s])
    return out

pads = M['pads']
ghosts = M['ghosts']
g = [G_MIN] * len(ghosts)
T = [[0.0, 0.0] for _ in ghosts]
gpress = defaultdict(lambda: [0.0, 0.0])
for gi, gh in enumerate(ghosts):
    for pe in gh['pads']:
        pe['ghost'] = gi
        pads.append(pe)
pads_of = defaultdict(list)
for pi, p in enumerate(pads):
    p['pts'] = inflate_poly(p['pts'], POLY_ERR)
    p['lc'] = max(p.get('lc', 0.0), net_clr(p.get('real_net', p['net'])))
    pads_of[p['ref']].append(pi)
    p['cx0'], p['cy0'] = p['x'], p['y']
    p['pts0'] = [q[:] for q in p['pts']]
    p['layset'] = frozenset(p['layers'])

cy0, cy = {}, {}
for r, entry in M['courtyards'].items():
    for lay, polys in entry.items():
        cy0[(r, lay)] = polys
        cy[(r, lay)] = [[q[:] for q in poly] for poly in polys]
for gi, gh in enumerate(ghosts):
    for lay, polys in gh['courtyards'].items():
        cy0[('GHOST:' + gh['ref'], lay)] = polys
        cy[('GHOST:' + gh['ref'], lay)] = [[q[:] for q in poly] for poly in polys]
cy_refs = sorted({r for r, _ in cy0})


def set_part_geom(r):
    dx, dy = D[r]
    for pi in pads_of[r]:
        p = pads[pi]
        p['x'], p['y'] = p['cx0'] + dx, p['cy0'] + dy
        p['pts'] = [[q[0] + dx, q[1] + dy] for q in p['pts0']]
    for lay in ('F', 'B'):
        key = (r, lay)
        if key in cy0:
            cy[key] = [[[q[0] + dx, q[1] + dy] for q in poly] for poly in cy0[key]]
    for nid in part_anchors[r]:
        P[nid][0] = anchor0[nid][0] + dx
        P[nid][1] = anchor0[nid][1] + dy


def set_ghost_geom(gi):
    gh = ghosts[gi]
    cx, cy_ = gh['at']
    tx, ty = T[gi]
    s = g[gi]
    ref = 'GHOST:' + gh['ref']
    for pi in pads_of[ref]:
        p = pads[pi]
        p['x'] = cx + tx + (p['cx0'] - cx) * s
        p['y'] = cy_ + ty + (p['cy0'] - cy_) * s
        p['pts'] = [[cx + tx + (q[0] - cx) * s, cy_ + ty + (q[1] - cy_) * s] for q in p['pts0']]
    for lay in ('F', 'B'):
        key = (ref, lay)
        if key in cy0:
            cy[key] = [[[cx + tx + (q[0] - cx) * s, cy_ + ty + (q[1] - cy_) * s] for q in poly]
                       for poly in cy0[key]]

for gi in range(len(ghosts)):
    set_ghost_geom(gi)

# ---- transforms (proposed moves) ---------------------------------------------
# mvr[ref] = ('t', dx, dy)                       translate
#          = ('s', cx, cy, f, dx, dy)            scale about (cx,cy) by 1+f*a, then translate


def xf_pt(q, mv, a):
    if mv[0] == 't':
        return (q[0] + mv[1] * a, q[1] + mv[2] * a)
    _, cx, cy_, f, dx, dy = mv
    s = 1 + f * a
    return (cx + (q[0] - cx) * s + dx * a, cy_ + (q[1] - cy_) * s + dy * a)


def xf_pts(pts, mv, a):
    return [list(xf_pt(q, mv, a)) for q in pts]


def _polys_bbox(polys):
    xs = [q[0] for poly in polys for q in poly]
    ys = [q[1] for poly in polys for q in poly]
    return (min(xs), min(ys), max(xs), max(ys))

cy_pairs = defaultdict(list)


def build_cy_pairs():
    """Candidate courtyard pairs from current bboxes (movers + ghosts vs all)."""
    cy_pairs.clear()
    boxes = {}
    for (r, lay), polys in cy.items():
        boxes[(r, lay)] = _polys_bbox(polys)
    movers = [r for r in cy_refs if r in movable or r.startswith('GHOST:')]
    for r in movers:
        for lay in ('F', 'B'):
            if (r, lay) not in boxes:
                continue
            b1 = boxes[(r, lay)]
            for (r2, lay2), b2 in boxes.items():
                if lay2 != lay or r2 == r:
                    continue
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
    for ti, t in enumerate(M['tracks']):
        (ax, ay), (bx, by) = t['a'], t['b']
        for c in cells_for_box(min(ax, bx), min(ay, by), max(ax, bx), max(ay, by), QR):
            static_grid[c].append(('T', ti))
    for vi, v in enumerate(M['vias']):
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
        if s.get('dead'):
            continue
        (ax, ay), (bx, by) = P[s['a']], P[s['b']]
        for c in cells_for_box(min(ax, bx), min(ay, by), max(ax, bx), max(ay, by), QR):
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
    d = min(seg_seg_dist(p1, p2, tuple(pts[i]), tuple(pts[(i + 1) % n])) for i in range(n))
    if d > 0 and point_in_poly((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2, pts):
        return 0.0
    return d


def poly_pt_dist(pts, c):
    n = len(pts)
    d = min(seg_seg_dist(c, c, tuple(pts[i]), tuple(pts[(i + 1) % n])) for i in range(n))
    if d > 0 and point_in_poly(c[0], c[1], pts):
        return 0.0
    return d


def poly_poly_dist(A, B):
    na, nb = len(A), len(B)
    d = min(seg_seg_dist(tuple(A[i]), tuple(A[(i + 1) % na]), tuple(B[j]), tuple(B[(j + 1) % nb]))
            for i in range(na) for j in range(nb))
    if d > 0 and (point_in_poly(A[0][0], A[0][1], B) or point_in_poly(B[0][0], B[0][1], A)):
        return 0.0
    return d

# ---- uniform copper-object margins -------------------------------------------
# descriptor: (kind, g1, g2, r_cu, layers, hole, lc)
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
    if _KORD[A[0]] > _KORD[B[0]]:
        A, B = B, A
    if B[0] != 'poly':
        return [('c', cu_dist(A, B) - eff_c)]
    H = eff_c + GF_CUT + 0.05
    out = []
    if A[0] != 'poly':
        pts = B[1]
        n = len(pts)
        r = A[3]
        if A[0] == 'seg':
            mid = ((A[1][0] + A[2][0]) / 2, (A[1][1] + A[2][1]) / 2)
            dj = [seg_seg_dist(A[1], A[2], tuple(pts[j]), tuple(pts[(j + 1) % n])) for j in range(n)]
        else:
            mid = A[1]
            dj = [seg_seg_dist(A[1], A[1], tuple(pts[j]), tuple(pts[(j + 1) % n])) for j in range(n)]
        if min(dj) > 0 and point_in_poly(mid[0], mid[1], pts):
            return [(('c', 'in'), -eff_c - r)]
        for j in range(n):
            m = dj[j] - r - eff_c
            if m < H - eff_c:
                out.append((('c', j), m))
        return out
    pa, pb = A[1], B[1]
    na, nb = len(pa), len(pb)
    if point_in_poly(pa[0][0], pa[0][1], pb) or point_in_poly(pb[0][0], pb[0][1], pa):
        return [(('c', 'in'), -eff_c)]
    for i in range(na):
        e1a, e1b = tuple(pa[i]), tuple(pa[(i + 1) % na])
        for j in range(nb):
            m = seg_seg_dist(e1a, e1b, tuple(pb[j]), tuple(pb[(j + 1) % nb])) - eff_c
            if m < H - eff_c:
                out.append((('c', i, j), m))
    return out


def pair_margins(A, B, hole_only=False):
    eff_c = max(CLR, A[6], B[6])
    eff_h = max(HOLE_CLR, A[6], B[6])
    ha, hb = A[5], B[5]
    if A[0] == 'poly' or B[0] == 'poly':
        ax0, ay0, ax1, ay1 = _bbox_of(A)
        bx0, by0, bx1, by1 = _bbox_of(B)
        gap = math.hypot(max(bx0 - ax1, ax0 - bx1, 0), max(by0 - ay1, ay0 - by1, 0))
        slack = A[3] + B[3] + (ha[1] if ha else 0) + (hb[1] if hb else 0)
        if gap - slack > max(eff_c, eff_h) + GF_CUT + 0.06:
            return []
    out = []
    if not hole_only and _lay_overlap(A[4], B[4]):
        out.extend(_c_margins(A, B, eff_c))
    if ha and not hole_only and not _no_cu(B[4]):
        out.append(('h1', cu_dist(('disc', ha[0], None, ha[1], ALL, None, 0), B) - eff_h))
    if hb and not hole_only and not _no_cu(A[4]):
        out.append(('h2', cu_dist(('disc', hb[0], None, hb[1], ALL, None, 0), A) - eff_h))
    if ha and hb:
        out.append(('hh', math.hypot(ha[0][0] - hb[0][0], ha[0][1] - hb[0][1]) - ha[1] - hb[1] - eff_h))
    return out

# ---- movable-element descriptors (mv-aware) ---------------------------------


def npos(nid, mvn, a):
    x, y = P[nid]
    d = mvn.get(nid)
    return (x + d[0] * a, y + d[1] * a) if d else (x, y)


def seg_desc(si, mvn, a):
    s = segs[si]
    return ('seg', npos(s['a'], mvn, a), npos(s['b'], mvn, a), s['w'] / 2, s['layer'], None,
            net_clr(s['net']))


def via_desc(nid, mvn, a):
    c = npos(nid, mvn, a)
    return ('disc', c, None, via_dia[nid] / 2, ALL,
            (c, via_drill[nid] / 2) if via_drill[nid] else None, net_clr(node_net[nid]))


def pad_desc(pi, mvr, a):
    p = pads[pi]
    mv = mvr.get(p['ref'])
    if mv:
        pts = xf_pts(p['pts'], mv, a)
        c = xf_pt((p['x'], p['y']), mv, a)
    else:
        pts, c = p['pts'], (p['x'], p['y'])
    if p['npth']:
        lays = frozenset()
    else:
        lays = ALL if p['drill'] else p['layset']
    return ('poly', pts, c, 0.0, lays, (c, p['drill'] / 2) if p['drill'] else None, p['lc'])

track_desc = [('seg', tuple(t['a']), tuple(t['b']), t['w'] / 2, t['layer'], None, net_clr(t['net']))
              for t in M['tracks']]
svia_desc = [('disc', (v['x'], v['y']), None, v['dia'] / 2, ALL, ((v['x'], v['y']), v['drill'] / 2),
              net_clr(v['net'])) for v in M['vias']]


OUTLINE = R.get('outline') or dict(rect=EDGE, corner_r=0.0)
OX0, OY0, OX1, OY1 = OUTLINE['rect']
OCR = OUTLINE['corner_r']


def edge_dist(x, y):
    """Distance from (x,y) inside the board to the outline: a rectangle
    with rounded corners of radius OCR (negative outside)."""
    d = min(x - OX0, OX1 - x, y - OY0, OY1 - y)
    if OCR <= 0:
        return d
    cx = OX0 + OCR if x < OX0 + OCR else (OX1 - OCR if x > OX1 - OCR else None)
    cy = OY0 + OCR if y < OY0 + OCR else (OY1 - OCR if y > OY1 - OCR else None)
    if cx is not None and cy is not None:
        return OCR - math.hypot(x - cx, y - cy)
    return d


def edge_margin(xys, hw):
    return min(edge_dist(x, y) for x, y in xys) - ECLR - hw

# ---- attachment (same-net contact persistence) -------------------------------


def at_margin(A, B):
    return -cu_dist(A, B) - OV


def _at(base_key, A, B, blk):
    key = ('AT',) + base_key
    if key in gf:
        m = at_margin(A, B)
        if m > -0.1:
            return [(key, m, blk)]
    return []

# ---- net length (springs + hard cap) -----------------------------------------
net_len = defaultdict(float)
net_len0 = {}
net_static = defaultdict(float)
for t in M['tracks']:
    net_static[t['net']] += math.hypot(t['b'][0] - t['a'][0], t['b'][1] - t['a'][1])


def recompute_net_len():
    net_len.clear()
    for s in segs:
        if s.get('dead'):
            continue
        (ax, ay), (bx, by) = P[s['a']], P[s['b']]
        net_len[s['net']] += math.hypot(bx - ax, by - ay)


def net_tot(net):
    return net_len0.get(net, 0.0) + net_static.get(net, 0.0)


def net_allow_mm(net):
    el = net_el(net)
    return max(el['allow'] * net_tot(net), el.get('allow_mm', 0.0))


def net_cap_mm(net):
    el = net_el(net)
    if el['cap'] > 1e6:
        return 1e9
    return max(el['cap'] * net_tot(net), el.get('cap_mm', 0.0))


def net_excess(net):
    """Growth beyond the allowance, as a fraction of the net's region copper
    (0 while within allowance). Drives the springs."""
    L0 = net_len0.get(net)
    if not L0:
        return 0.0
    over = net_len[net] - L0 - net_allow_mm(net)
    return max(0.0, over / net_tot(net))


def net_growth(net):
    L0 = net_len0.get(net)
    return (net_len[net] - L0) / net_tot(net) if L0 else 0.0


def len_delta(mvn, a):
    """Per-net copper length change if nodes in mvn move by a*mv."""
    d = defaultdict(float)
    seen = set()
    for nid in mvn:
        for si in inc.get(nid, ()):
            if si in seen:
                continue
            seen.add(si)
            s = segs[si]
            if s.get('dead'):
                continue
            (ax, ay), (bx, by) = P[s['a']], P[s['b']]
            (nx1, ny1), (nx2, ny2) = npos(s['a'], mvn, a), npos(s['b'], mvn, a)
            d[s['net']] += math.hypot(nx2 - nx1, ny2 - ny1) - math.hypot(bx - ax, by - ay)
    return d


def gen_len(mvn, a):
    """Hard-cap margins: L_max - L(a) per touched net."""
    for net, dl in len_delta(mvn, a).items():
        L0 = net_len0.get(net)
        cap = net_cap_mm(net)
        if not L0 or cap > 1e8:
            continue
        yield ('LEN', net), L0 + cap - (net_len[net] + dl), ('L', net)

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
                    yield from _at(('SS', min(ci, t['c']), max(ci, t['c'])), A, seg_desc(sj, mvn, a), ('S', sj))
                continue
            m = seg_seg_dist(A[1], A[2], npos(t['a'], mvn, a), npos(t['b'], mvn, a)) \
                - max(CLR, A[6], net_clr(t['net'])) - (w + t['w']) / 2
            yield ('SS', min(ci, t['c']), max(ci, t['c']), 'c'), m, ('S', sj)
        for nj in vgrid.get(cell, ()):
            if ('N', nj) in seen:
                continue
            seen.add(('N', nj))
            if node_net[nj] == net:
                if net != 'GND':
                    yield from _at(('SN', ci, nj), A, via_desc(nj, mvn, a), ('N', nj))
                continue
            for tag, m in pair_margins(A, via_desc(nj, mvn, a)):
                yield ('SN', ci, nj, tag), m, ('N', nj)
        for kind, idx in static_grid.get(cell, ()):
            if (kind, idx) in seen:
                continue
            seen.add((kind, idx))
            if kind == 'T':
                t = M['tracks'][idx]
                if t['layer'] != lay:
                    continue
                if t['net'] == net:
                    if net != 'GND':
                        yield from _at(('ST', ci, idx), A, track_desc[idx], ('T', idx))
                    continue
                m = seg_seg_dist(A[1], A[2], tuple(t['a']), tuple(t['b'])) \
                    - max(CLR, A[6], track_desc[idx][6]) - (w + t['w']) / 2
                yield ('ST', ci, idx, 'c'), m, ('T', idx)
            elif kind == 'V':
                if M['vias'][idx]['net'] == net:
                    if net != 'GND':
                        yield from _at(('SV', ci, idx), A, svia_desc[idx], ('V', idx))
                    continue
                for tag, m in pair_margins(A, svia_desc[idx]):
                    yield ('SV', ci, idx, tag), m, ('V', idx)
            else:
                p = pads[idx]
                if p['net'] and p['net'] == net:
                    if net != 'GND':
                        yield from _at(('SP', ci, idx), A, pad_desc(idx, mvr, a), ('P', idx))
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
                    yield from _at(('SN', t['c'], nid), seg_desc(sj, mvn, a), A, ('S', sj))
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
                yield from _at(('NN', lo, hi), A, via_desc(nj, mvn, a), ('N', nj))
            for tag, m in pair_margins(via_desc(lo, mvn, a), via_desc(hi, mvn, a), hole_only=same):
                yield ('NN', lo, hi, tag), m, ('N', nj)
        for kind, idx in static_grid.get(cell, ()):
            if (kind, idx) in seen:
                continue
            seen.add((kind, idx))
            if kind == 'T':
                t = M['tracks'][idx]
                if t['net'] == net:
                    if net != 'GND':
                        yield from _at(('NT', nid, idx), A, track_desc[idx], ('T', idx))
                    continue
                for tag, m in pair_margins(A, track_desc[idx]):
                    yield ('NT', nid, idx, tag), m, ('T', idx)
            elif kind == 'V':
                v = M['vias'][idx]
                same = v['net'] == net
                if same and net != 'GND':
                    yield from _at(('NV', nid, idx), A, svia_desc[idx], ('V', idx))
                for tag, m in pair_margins(A, svia_desc[idx], hole_only=same):
                    yield ('NV', nid, idx, tag), m, ('V', idx)
            else:
                p = pads[idx]
                if p['net'] and p['net'] == net:
                    if net != 'GND':
                        yield from _at(('NP', nid, idx), A, pad_desc(idx, mvr, a), ('P', idx))
                    continue
                for tag, m in pair_margins(A, pad_desc(idx, mvr, a)):
                    yield ('NP', nid, idx, tag), m, ('P', idx)
    if EDGE:
        yield ('VE', nid), edge_margin((A[1],), via_dia[nid] / 2), ('E',)


def gen_pad(pi, mvn, mvr, a, skip_refs=frozenset()):
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
                    yield from _at(('SP', t['c'], pi), seg_desc(sj, mvn, a), A, ('S', sj))
                continue
            for tag, m in pair_margins(seg_desc(sj, mvn, a), A):
                yield ('SP', t['c'], pi, tag), m, ('S', sj)
        for nj in vgrid.get(cell, ()):
            if ('N', nj) in seen:
                continue
            seen.add(('N', nj))
            if net and node_net[nj] == net:
                if net != 'GND':
                    yield from _at(('NP', nj, pi), via_desc(nj, mvn, a), A, ('N', nj))
                continue
            for tag, m in pair_margins(via_desc(nj, mvn, a), A):
                yield ('NP', nj, pi, tag), m, ('N', nj)
        for kind, idx in static_grid.get(cell, ()):
            if (kind, idx) in seen:
                continue
            seen.add((kind, idx))
            if kind == 'T':
                t = M['tracks'][idx]
                if net and t['net'] == net:
                    if net != 'GND':
                        yield from _at(('PT', pi, idx), A, track_desc[idx], ('T', idx))
                    continue
                for tag, m in pair_margins(A, track_desc[idx]):
                    yield ('PT', pi, idx, tag), m, ('T', idx)
            elif kind == 'V':
                if net and M['vias'][idx]['net'] == net:
                    if net != 'GND':
                        yield from _at(('PV', pi, idx), A, svia_desc[idx], ('V', idx))
                    continue
                for tag, m in pair_margins(A, svia_desc[idx]):
                    yield ('PV', pi, idx, tag), m, ('V', idx)
            else:
                if idx == pi:
                    continue
                q = pads[idx]
                if q['ref'] == ref or q['ref'] in skip_refs:
                    continue
                if net and q['net'] == net:
                    if net != 'GND':
                        yield from _at(('PP', min(pi, idx), max(pi, idx)), A, pad_desc(idx, mvr, a), ('P', idx))
                    continue
                lo, hi = min(pi, idx), max(pi, idx)
                for tag, m in pair_margins(pad_desc(lo, mvr, a), pad_desc(hi, mvr, a)):
                    yield ('PP', lo, hi, tag), m, ('P', idx)
    if EDGE and not p['npth']:
        yield ('PE', pi), edge_margin(A[1], 0.0), ('E',)


def crt_margin(A, B):
    d = poly_poly_dist(A, B)
    return d if d > 0 else -0.05


def gen_crt(ref, mvr, a, skip_refs=frozenset()):
    mv = mvr.get(ref)
    for r2, lay in cy_pairs[ref]:
        if r2 in skip_refs:
            continue
        for pa in cy[(ref, lay)]:
            if mv:
                pa = xf_pts(pa, mv, a)
            for pb in cy[(r2, lay)]:
                mv2 = mvr.get(r2)
                if mv2:
                    pb = xf_pts(pb, mv2, a)
                lo, hi = min(ref, r2), max(ref, r2)
                yield ('CY', lo, hi, lay), crt_margin(pa, pb), ('C', r2)


def gen_part(ref, mvn, mvr, a, skip_refs=frozenset()):
    for pi in pads_of[ref]:
        yield from gen_pad(pi, mvn, mvr, a, skip_refs)
    seen = set()
    for nid in part_anchors[ref]:
        for si in inc[nid]:
            if si not in seen and not segs[si].get('dead'):
                seen.add(si)
                yield from gen_seg(si, mvn, mvr, a)
        if is_via[nid]:
            yield from gen_via(nid, mvn, mvr, a)
    yield from gen_crt(ref, mvr, a, skip_refs)
    yield from gen_len(mvn, a)


def gen_ghost(gi, mvr, a):
    ref = 'GHOST:' + ghosts[gi]['ref']
    for pi in pads_of[ref]:
        yield from gen_pad(pi, {}, mvr, a)
    yield from gen_crt(ref, mvr, a)


def gen_node(nid, mvn, a):
    for si in inc[nid]:
        if not segs[si].get('dead'):
            yield from gen_seg(si, mvn, {}, a)
    if is_via[nid]:
        yield from gen_via(nid, mvn, {}, a)
    yield from gen_len(mvn, a)

# ---- floors: grandfathered shipped margins + guard band ---------------------
gf = {}


def floor_of(key):
    if key[0] == 'AT':
        f = gf.get(key)
        return -1e9 if f is None else min(f, 0.0)
    if key[0] == 'LEN':
        return 0.0
    gg = CRT_GUARD if key[0] == 'CY' else GUARD
    f = gf.get(key)
    return gg if f is None else min(f, gg)


def _ghost_blk(blk):
    return blk[0] == 'P' and pads[blk[1]].get('ghost') is not None or \
        blk[0] == 'C' and blk[1].startswith('GHOST:')


def measure():
    """Record shipped margins (min per clearance pair) and same-net contacts
    (max per attachment pair) at s=0. Ghost pairs are never recorded."""
    build_static_grid()
    build_dyn_grids()
    build_cy_pairs()

    def seed(base_key, A, B):
        m = at_margin(A, B)
        if m >= -OV - 0.0005:
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
                if sj != si and t['c'] != ci and t['net'] == net and t['layer'] == lay:
                    seed(('SS', min(ci, t['c']), max(ci, t['c'])), A, seg_desc(sj, {}, 0))
            for nj in vgrid.get(cell, ()):
                if node_net[nj] == net:
                    seed(('SN', ci, nj), A, via_desc(nj, {}, 0))
            for kind, idx in static_grid.get(cell, ()):
                if kind == 'T' and M['tracks'][idx]['net'] == net and M['tracks'][idx]['layer'] == lay:
                    seed(('ST', ci, idx), A, track_desc[idx])
                elif kind == 'V' and M['vias'][idx]['net'] == net:
                    seed(('SV', ci, idx), A, svia_desc[idx])
                elif kind == 'P' and pads[idx]['net'] == net:
                    seed(('SP', ci, idx), A, pad_desc(idx, {}, 0))
    for r in movable:
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
                    if kind == 'T' and M['tracks'][idx]['net'] == net:
                        seed(('PT', pi, idx), A, track_desc[idx])
                    elif kind == 'V' and M['vias'][idx]['net'] == net:
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
                    seed(('NN', min(nid, nj), max(nid, nj)), via_desc(min(nid, nj), {}, 0),
                         via_desc(max(nid, nj), {}, 0))
            for kind, idx in static_grid.get(cell, ()):
                if kind == 'T' and M['tracks'][idx]['net'] == net:
                    seed(('NT', nid, idx), A, track_desc[idx])
                elif kind == 'V' and M['vias'][idx]['net'] == net:
                    seed(('NV', nid, idx), A, svia_desc[idx])
                elif kind == 'P' and pads[idx]['net'] == net:
                    seed(('NP', nid, idx), A, pad_desc(idx, {}, 0))
    n_at = sum(1 for k in gf if k[0] == 'AT')

    hist = defaultdict(int)

    def rec(gen):
        for key, m, blk in gen:
            if key[0] in ('AT', 'LEN') or _ghost_blk(blk):
                continue
            hist[min(9, max(-1, int(m / 0.02)))] += 1
            if m < GF_CUT and m < gf.get(key, 1e9):
                gf[key] = m
    for si in range(len(segs)):
        rec(gen_seg(si, {}, {}, 0.0))
    for nid in via_nids:
        rec(gen_via(nid, {}, {}, 0.0))
    for r in movable:
        for pi in pads_of[r]:
            rec(gen_pad(pi, {}, {}, 0.0))
        rec(gen_crt(r, {}, 0.0))
    ncl = len(gf) - n_at
    neg = sum(1 for k, v in gf.items() if k[0] != 'AT' and v < 0)
    print(f'grandfathered {ncl} shipped at/below-guard pairs ({neg} below model rule), '
          f'{n_at} same-net contacts held as attachments')
    print('margin census over rule (20um bins, <0 first): ' +
          ' '.join(f'{k * 20:+d}:{hist[k]}' for k in sorted(hist)))
    recompute_net_len()
    for net, L in net_len.items():
        net_len0[net] = L

# ---- feasibility line search (with the non-worsening rule) -------------------


def _eval(gen):
    """Aggregate a generator into {key: (margin, blk)} (AT keys max)."""
    out = {}
    for key, m, blk in gen:
        if key[0] == 'AT':
            if key not in out or m > out[key][0]:
                out[key] = (m, blk)
        else:
            if key not in out or m < out[key][0]:
                out[key] = (m, blk)
    return out


def _violations(ev, base):
    bad = []
    for key, (m, blk) in ev.items():
        if m < floor_of(key) - EPS:
            if base is not None and m >= base.get(key, 1e9) - EPS:
                continue                       # already below floor, not worsened
            bad.append((key, m, blk))
    return bad


def line_search(gen_at):
    ev1 = _eval(gen_at(1.0))
    bad = _violations(ev1, None)
    if not bad:
        return 1.0, []
    base = {k: m for k, (m, _) in _eval(gen_at(0.0)).items()}
    bad = _violations(ev1, base)
    if not bad:
        return 1.0, []
    lo, hi = 0.0, 1.0
    for _ in range(6):
        mid = (lo + hi) / 2
        if not _violations(_eval(gen_at(mid)), base):
            lo = mid
        else:
            hi = mid
    return lo, bad

# ---- pressure ----------------------------------------------------------------
press = defaultdict(lambda: [0.0, 0.0])
ppress = defaultdict(lambda: [0.0, 0.0])
absorb = defaultdict(int)


def blk_info(blk):
    k = blk[0]
    if k == 'S':
        s = segs[blk[1]]
        return dict(kind='dyn_seg', net=s['net'], layer=s['layer'], chain=s['c'],
                    pos=[round((P[s['a']][0] + P[s['b']][0]) / 2, 3),
                         round((P[s['a']][1] + P[s['b']][1]) / 2, 3)])
    if k == 'N':
        return dict(kind='dyn_via', net=node_net[blk[1]],
                    pos=[round(P[blk[1]][0], 3), round(P[blk[1]][1], 3)])
    if k == 'T':
        t = M['tracks'][blk[1]]
        return dict(kind='static_track', net=t['net'], layer=t['layer'],
                    pos=[round((t['a'][0] + t['b'][0]) / 2, 3), round((t['a'][1] + t['b'][1]) / 2, 3)])
    if k == 'V':
        v = M['vias'][blk[1]]
        return dict(kind='static_via', net=v['net'], pos=[round(v['x'], 3), round(v['y'], 3)])
    if k == 'P':
        p = pads[blk[1]]
        return dict(kind='ghost_pad' if p.get('ghost') is not None else 'pad',
                    ref=f"{p['ref']}.{p['num']}", net=p.get('real_net', p['net']),
                    pos=[round(p['x'], 3), round(p['y'], 3)])
    if k == 'C':
        return dict(kind='courtyard', ref=blk[1])
    if k == 'L':
        return dict(kind='length_cap', net=blk[1])
    return dict(kind='board_edge')


def part_mid(r):
    xs = [pads[pi]['x'] for pi in pads_of[r]]
    ys = [pads[pi]['y'] for pi in pads_of[r]]
    return sum(xs) / len(xs), sum(ys) / len(ys)


def ghost_mid(gi):
    return ghosts[gi]['at'][0] + T[gi][0], ghosts[gi]['at'][1] + T[gi][1]


def _push_part(ref, dx, dy, mag):
    pr = ppress[ref]
    pr[0] += dx * mag
    pr[1] += dy * mag


def _push_ghost(gi, dx, dy, mag):
    pr = gpress[gi]
    pr[0] += dx * mag
    pr[1] += dy * mag


def _push_node(nid, dx, dy, mag, own=None):
    if nid in fixed:
        return False
    r = part_of.get(nid)
    if r is not None:
        if r == own:
            return False
        _push_part(r, dx, dy, mag)
        return True
    pr = press[nid]
    pr[0] += dx * mag
    pr[1] += dy * mag
    return True


def _absorb(mover, info):
    what = info.get('net', '') or ''
    if info.get('ref'):
        what = f"{info['ref']} [{what}]" if what else info['ref']
    absorb[(mover, info['kind'], what)] += 1


def deposit(clamps, mover_mid, fx, fy, mover_name, own=None, reaction=None):
    """Distribute displacement demand onto blockers. `own` = the mover's own
    ref (its anchors are not pushed). `reaction`, if given, receives a
    push-back on the mover from unmovable blockers (ghosts slide off walls)."""
    for key, m, blk in clamps:
        need = floor_of(key) - m + 0.001
        if need <= 0:
            continue
        mag = min(STEP, need)
        k = blk[0]
        is_at = key[0] == 'AT'
        info = blk_info(blk)
        if k in ('S', 'N'):
            tgt = [blk[1]] if k == 'N' else [segs[blk[1]]['a'], segs[blk[1]]['b']]
            bx, by = info['pos']
            dx, dy = bx - mover_mid[0], by - mover_mid[1]
            L = math.hypot(dx, dy)
            if L < 1e-6:
                dx, dy, L = fx, fy, 1.0
            if is_at:
                dx, dy = -dx, -dy
            moved = False
            for nid in tgt:
                moved |= _push_node(nid, dx / L, dy / L, mag, own)
            if not moved:
                _absorb(mover_name, info)
                if reaction is not None:
                    reaction[0] -= dx / L * mag; reaction[1] -= dy / L * mag
            continue
        if k in ('P', 'C'):
            ref = pads[blk[1]]['ref'] if k == 'P' else blk[1]
            if is_at and key[1] in ('SP', 'ST', 'SV'):
                ci = key[2]
                bx, by = info.get('pos', mover_mid)
                glued = False
                for si in chain_segs.get(ci, ()):
                    s = segs[si]
                    mx = (P[s['a']][0] + P[s['b']][0]) / 2
                    my = (P[s['a']][1] + P[s['b']][1]) / 2
                    if math.hypot(mx - bx, my - by) < 1.0:
                        for nid in (s['a'], s['b']):
                            if nid not in fixed and nid not in part_of:
                                ddx, ddy = bx - P[nid][0], by - P[nid][1]
                                LL = math.hypot(ddx, ddy) or 1.0
                                _push_node(nid, ddx / LL, ddy / LL, mag)
                                glued = True
                if glued:
                    continue
            if ref.startswith('GHOST:'):
                gi = next(i for i, gh in enumerate(ghosts) if 'GHOST:' + gh['ref'] == ref)
                bx, by = ghost_mid(gi)
                dx, dy = bx - mover_mid[0], by - mover_mid[1]
                L = math.hypot(dx, dy) or 1.0
                _push_ghost(gi, dx / L, dy / L, mag)
                continue
            if ref in movable and ref != own:
                bx, by = part_mid(ref)
                dx, dy = bx - mover_mid[0], by - mover_mid[1]
                L = math.hypot(dx, dy)
                if L < 1e-6:
                    dx, dy, L = fx, fy, 1.0
                if is_at:
                    dx, dy = -dx, -dy
                _push_part(ref, dx / L, dy / L, mag)
                continue
            _absorb(mover_name, info)
            if reaction is not None:
                bx, by = info.get('pos') or part_mid(ref) if ref in pads_of else mover_mid
                dx, dy = bx - mover_mid[0], by - mover_mid[1]
                L = math.hypot(dx, dy) or 1.0
                reaction[0] -= dx / L * mag; reaction[1] -= dy / L * mag
            continue
        if is_at and key[1] in ('NT', 'NV', 'NP'):
            nid = key[2]
            bx, by = info['pos']
            ddx, ddy = bx - P[nid][0], by - P[nid][1]
            LL = math.hypot(ddx, ddy) or 1.0
            if _push_node(nid, ddx / LL, ddy / LL, mag):
                continue
        _absorb(mover_name, info)
        if reaction is not None and k == 'E':
            ex0, ey0, ex1, ey1 = EDGE
            x, y = mover_mid
            d = min((x - ex0, (1, 0)), (ex1 - x, (-1, 0)), (y - ey0, (0, 1)), (ey1 - y, (0, -1)))
            reaction[0] += d[1][0] * mag; reaction[1] += d[1][1] * mag

# ---- drives ------------------------------------------------------------------


def cap_vec(vx, vy, cap):
    L = math.hypot(vx, vy)
    if L > cap:
        return vx / L * cap, vy / L * cap
    return vx, vy


def spring_part(ref):
    vx = vy = 0.0
    for nid in part_anchors[ref]:
        for si in inc[nid]:
            s = segs[si]
            if s.get('dead'):
                continue
            el = net_el(s['net'])
            ex = net_excess(s['net'])
            if ex <= 0 or el['k'] <= 0:
                continue
            o = s['b'] if s['a'] == nid else s['a']
            dx, dy = P[o][0] - P[nid][0], P[o][1] - P[nid][1]
            L = math.hypot(dx, dy)
            if L < 1e-6:
                continue
            pull = min(STEP, el['k'] * ex * STEP)
            vx += dx / L * pull
            vy += dy / L * pull
    return cap_vec(vx, vy, STEP)


def part_drive(ref):
    p = parts[ref]
    vx = vy = 0.0
    t = p.get('target')
    if t:
        vx, vy = cap_vec(t[0] - D[ref][0], t[1] - D[ref][1], STEP)
    pr = ppress.get(ref)
    if pr:
        vx += pr[0]; vy += pr[1]
    sx, sy = spring_part(ref)
    vx += sx; vy += sy
    return cap_vec(vx, vy, STEP * p['mobility'])

# units: rigid groups move as one
group_members = defaultdict(list)
for r in movable:
    gname = parts[r].get('group')
    if gname and cfg.get('groups', {}).get(gname, {}).get('rigid', True):
        group_members[gname].append(r)
unit_of = {}
for gname, refs in group_members.items():
    for r in refs:
        unit_of[r] = gname
units = [(gname, refs) for gname, refs in group_members.items()] + \
        [(r, [r]) for r in sorted(movable) if r not in unit_of]
if group_members:
    print(f'{len(group_members)} rigid groups: ' +
          ', '.join(f'{gname}({len(refs)})' for gname, refs in group_members.items()))

blocked_log = {}


def move_unit(name, refs, vx, vy, mover_name=None):
    """Joint line-searched translation of a set of parts. Returns mm advanced."""
    L = math.hypot(vx, vy)
    if L < 2e-4:
        return 0.0
    mvn, mvr = {}, {}
    for r in refs:
        mvr[r] = ('t', vx, vy)
        for nid in part_anchors[r]:
            mvn[nid] = (vx, vy)
    skip = frozenset(refs) if len(refs) > 1 else frozenset()

    def gen(a):
        for r in refs:
            yield from gen_part(r, mvn, mvr, a, skip)
    al, clamps = line_search(gen)
    adv = 0.0
    if al * L > 5e-5:
        for r in refs:
            D[r][0] += vx * al
            D[r][1] += vy * al
            set_part_geom(r)
        adv = al * L
        build_dyn_grids()
    if al < 1.0:
        blocked_log[name] = clamps
        mids = [part_mid(r) for r in refs]
        mid = (sum(x for x, _ in mids) / len(mids), sum(y for _, y in mids) / len(mids))
        deposit(clamps, mid, vx / L, vy / L, mover_name or name,
                own=refs[0] if len(refs) == 1 else None)
    else:
        blocked_log.pop(name, None)
    return adv


def part_pass():
    drives = []
    for name, refs in units:
        vx = vy = 0.0
        for r in refs:
            dx, dy = part_drive(r)
            vx += dx; vy += dy
        vx, vy = cap_vec(vx / len(refs), vy / len(refs), STEP)
        if math.hypot(vx, vy) >= 2e-4:
            drives.append((math.hypot(vx, vy), name, refs, vx, vy))
    drives.sort(key=lambda d: -d[0])
    adv, nblk = 0.0, 0
    for _, name, refs, vx, vy in drives:
        a = move_unit(name, refs, vx, vy)
        adv += a
        if name in blocked_log:
            nblk += 1
        for r in refs:
            ppress.pop(r, None)
    return adv, nblk


def ghost_pass():
    adv = 0.0
    for gi, gh in enumerate(ghosts):
        ref = 'GHOST:' + gh['ref']
        cx, cy_ = gh['at']
        mid = ghost_mid(gi)
        # 1. drift: pressure + anchor spring, line-searched translation
        pr = gpress.pop(gi, None)
        vx = vy = 0.0
        if pr:
            vx, vy = pr
        vx += G_ANCHOR * (cx - mid[0]); vy += G_ANCHOR * (cy_ - mid[1])
        vx, vy = cap_vec(vx, vy, STEP)
        if math.hypot(vx, vy) >= 2e-4:
            mvr = {ref: ('t', vx, vy)}
            al, clamps = line_search(lambda a: gen_ghost(gi, mvr, a))
            if al > 0:
                T[gi][0] += vx * al; T[gi][1] += vy * al
                set_ghost_geom(gi)
                adv += al * math.hypot(vx, vy)
            if al < 1.0:
                react = [0.0, 0.0]
                deposit(clamps, ghost_mid(gi), vx / math.hypot(vx, vy), vy / math.hypot(vx, vy),
                        ref, reaction=react)
        # 2. growth
        if g[gi] >= 1.0:
            blocked_log.pop(ref, None)
            continue
        dg = min(1.0 - g[gi], STEP / gh['r'])
        mid = ghost_mid(gi)
        mvr = {ref: ('s', mid[0], mid[1], dg / g[gi], 0.0, 0.0)}
        al, clamps = line_search(lambda a: gen_ghost(gi, mvr, a))
        if al * dg * gh['r'] > 5e-5:
            g[gi] = min(1.0, g[gi] + al * dg)
            set_ghost_geom(gi)
            adv += al * dg * gh['r']
        if al < 1.0:
            blocked_log[ref] = clamps
            react = [0.0, 0.0]
            deposit(clamps, mid, 0.0, 0.0, ref, reaction=react)
            if react[0] or react[1]:
                rx, ry = cap_vec(react[0], react[1], STEP)
                _push_ghost(gi, rx, ry, 1.0)
        else:
            blocked_log.pop(ref, None)
    return adv


def hard_walled(name):
    for key, m, blk in blocked_log.get(name, ()):
        if key[0] == 'AT':
            continue
        if blk[0] == 'E' or (blk[0] == 'P' and pads[blk[1]]['ref'] not in movable
                             and pads[blk[1]].get('ghost') is None):
            return True
    return False


def group_pass():
    """Joint step for mutually-wedged units (dynamic convoys)."""
    names = [n for n, _ in units if n in blocked_log and not hard_walled(n)]
    if len(names) < 2:
        return 0.0
    nset = set(names)
    unit_by_ref = {r: n for n, refs in units for r in refs}
    parent = {n: n for n in names}

    def find(n):
        while parent[n] != n:
            parent[n] = parent[parent[n]]
            n = parent[n]
        return n

    def union(a, b):
        if a in nset and b in nset:
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[ra] = rb
    for n in names:
        for key, m, blk in blocked_log[n]:
            if blk[0] == 'S':
                ci = segs[blk[1]]['c']
                for nid in (chains[ci][0], chains[ci][-1]):
                    r2 = part_of.get(nid)
                    if r2:
                        union(n, unit_by_ref.get(r2, r2))
            elif blk[0] == 'P':
                union(n, unit_by_ref.get(pads[blk[1]]['ref'], ''))
            elif blk[0] == 'C':
                union(n, unit_by_ref.get(blk[1], ''))
    comps = defaultdict(list)
    for n in names:
        comps[find(n)].append(n)
    adv = 0.0
    umap = dict(units)
    for comp in comps.values():
        if len(comp) < 2:
            continue
        refs = [r for n in comp for r in umap[n]]
        vx = vy = 0.0
        for r in refs:
            dx, dy = part_drive(r)
            vx += dx; vy += dy
        vx, vy = cap_vec(vx / len(refs), vy / len(refs), STEP)
        adv += move_unit('grp:' + comp[0], refs, vx, vy)
    return adv


def stretch_vec(nid):
    vx = vy = 0.0
    cap = SUB * 1.6
    for si in inc[nid]:
        s = segs[si]
        if s.get('dead'):
            continue
        o = s['b'] if s['a'] == nid else s['a']
        dx, dy = P[o][0] - P[nid][0], P[o][1] - P[nid][1]
        L = math.hypot(dx, dy)
        if L > cap:
            pull = (L - cap) / 2
            vx += dx / L * pull
            vy += dy / L * pull
    return vx, vy


def spring_node(nid):
    net = node_net[nid]
    el = net_el(net)
    ex = net_excess(net)
    if ex <= 0 or el['k'] <= 0:
        return 0.0, 0.0
    xs, ys, n = 0.0, 0.0, 0
    for si in inc[nid]:
        s = segs[si]
        if s.get('dead'):
            continue
        o = s['b'] if s['a'] == nid else s['a']
        xs += P[o][0]; ys += P[o][1]; n += 1
    if n < 2:
        return 0.0, 0.0
    k = min(1.0, el['k'] * ex)
    return (xs / n - P[nid][0]) * k, (ys / n - P[nid][1]) * k


def copper_pass():
    moved = 0.0
    cap = SUB * 1.6
    for sweep in range(NCOPPER):
        build_dyn_grids()
        todo = sorted(press, key=lambda n: -(press[n][0] ** 2 + press[n][1] ** 2))
        in_todo = set(todo)
        for s in segs:
            if s.get('dead'):
                continue
            (ax, ay), (bx, by) = P[s['a']], P[s['b']]
            over = math.hypot(bx - ax, by - ay) > cap
            spring = sweep == 0 and net_excess(s['net']) > 0 and net_el(s['net'])['k'] > 0
            if over or spring:
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
                vx, vy = pv
            else:
                vx, vy = stretch_vec(nid)
                sx, sy = spring_node(nid)
                vx += sx; vy += sy
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
                deposit(clamps, (P[nid][0], P[nid][1]), vx / Lv, vy / Lv, f'node{nid}')
        if sweep == 0:
            recompute_net_len()
    return moved

# ---- mid-flight atomic reroute (the stall worklist, consumed) ----------------
RR_GRID = 0.10
RR_M = GUARD + 0.008
RR_MAXPOP = 200000
RR_PER_ROUND = _arg('--rr-per-round', 16, int)
RR_ROUNDS = 0 if NO_RR else _arg('--rr-rounds', 12, int)
RR_WIN = 25
RX0, RY0 = REGION[0], REGION[1]
RW = int((REGION[2] - RX0) / RR_GRID) + 1
RH = int((REGION[3] - RY0) / RR_GRID) + 1


def _rxy(idx):
    return (RX0 + (idx % RW) * RR_GRID, RY0 + (idx // RW) * RR_GRID)


def _rinb(i, j):
    return 0 <= i < RW and 0 <= j < RH


def _pt_seg(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def _stamp_disc(m, x, y, r):
    i0 = max(0, int((x - r - RX0) / RR_GRID)); i1 = min(RW - 1, int((x + r - RX0) / RR_GRID) + 1)
    j0 = max(0, int((y - r - RY0) / RR_GRID)); j1 = min(RH - 1, int((y + r - RY0) / RR_GRID) + 1)
    r2 = r * r
    for j in range(j0, j1 + 1):
        cy_ = RY0 + j * RR_GRID
        base = j * RW
        for i in range(i0, i1 + 1):
            if (RX0 + i * RR_GRID - x) ** 2 + (cy_ - y) ** 2 <= r2:
                m[base + i] = 1


def _stamp_seg(m, ax, ay, bx, by, r):
    i0 = max(0, int((min(ax, bx) - r - RX0) / RR_GRID)); i1 = min(RW - 1, int((max(ax, bx) + r - RX0) / RR_GRID) + 1)
    j0 = max(0, int((min(ay, by) - r - RY0) / RR_GRID)); j1 = min(RH - 1, int((max(ay, by) + r - RY0) / RR_GRID) + 1)
    for j in range(j0, j1 + 1):
        cy_ = RY0 + j * RR_GRID
        base = j * RW
        for i in range(i0, i1 + 1):
            if _pt_seg(RX0 + i * RR_GRID, cy_, ax, ay, bx, by) <= r:
                m[base + i] = 1


def _stamp_poly(m, pts, r):
    xs = [q[0] for q in pts]; ys = [q[1] for q in pts]
    i0 = max(0, int((min(xs) - r - RX0) / RR_GRID)); i1 = min(RW - 1, int((max(xs) + r - RX0) / RR_GRID) + 1)
    j0 = max(0, int((min(ys) - r - RY0) / RR_GRID)); j1 = min(RH - 1, int((max(ys) + r - RY0) / RR_GRID) + 1)
    n = len(pts)
    for j in range(j0, j1 + 1):
        cy_ = RY0 + j * RR_GRID
        base = j * RW
        for i in range(i0, i1 + 1):
            cx = RX0 + i * RR_GRID
            if point_in_poly(cx, cy_, pts):
                m[base + i] = 1
                continue
            for k in range(n):
                a, b = pts[k], pts[(k + 1) % n]
                if _pt_seg(cx, cy_, a[0], a[1], b[0], b[1]) <= r:
                    m[base + i] = 1
                    break

_rr_masks = {}


def rr_mask(lay, net, hw, mm):
    key = (lay, net, round(hw, 4), round(mm, 4))
    if key in _rr_masks:
        return _rr_masks[key]
    m = bytearray(RW * RH)
    nc = net_clr(net)
    if EDGE:
        em = ECLR + hw + mm
        for j in range(RH):
            cy_ = RY0 + j * RR_GRID
            base = j * RW
            near_y = cy_ < OY0 + em + OCR or cy_ > OY1 - em - OCR
            for i in range(RW):
                cx = RX0 + i * RR_GRID
                if near_y or cx < OX0 + em + OCR or cx > OX1 - em - OCR:
                    if edge_dist(cx, cy_) < em:
                        m[base + i] = 1
    for t in M['tracks']:
        if t['layer'] != lay or t['net'] == net:
            continue
        _stamp_seg(m, t['a'][0], t['a'][1], t['b'][0], t['b'][1],
                   max(nc, net_clr(t['net'])) + mm + hw + t['w'] / 2)
    for v in M['vias']:
        if v['net'] == net:
            continue
        _stamp_disc(m, v['x'], v['y'], max(nc, net_clr(v['net'])) + mm + hw + v['dia'] / 2)
        _stamp_disc(m, v['x'], v['y'], HOLE_CLR + mm + hw + v['drill'] / 2)
    for p in pads:
        if p['net'] and p['net'] == net:
            continue
        pc = max(nc, p['lc']) + mm + hw
        ph = max(HOLE_CLR, p['lc']) + mm + hw
        if p.get('ghost') is not None:
            gi = p['ghost']
            cx, cy_ = ghosts[gi]['at']
            tx, ty = T[gi]
            pts = [[q[0] + tx, q[1] + ty] for q in p['pts0']]      # full size
            if not p['npth'] and (p['drill'] or lay in p['layset']):
                _stamp_poly(m, pts, pc)
            continue
        if not p['npth'] and (p['drill'] or lay in p['layset']):
            _stamp_poly(m, p['pts'], pc)
        if p['drill']:
            _stamp_disc(m, p['x'], p['y'], ph + p['drill'] / 2)
    for s in segs:
        if s.get('dead') or s['layer'] != lay or s['net'] == net:
            continue
        (ax, ay), (bx, by) = P[s['a']], P[s['b']]
        _stamp_seg(m, ax, ay, bx, by, max(nc, net_clr(s['net'])) + mm + hw + s['w'] / 2)
    for nid in via_nids:
        if node_net[nid] == net:
            continue
        x, y = P[nid]
        _stamp_disc(m, x, y, max(nc, net_clr(node_net[nid])) + mm + hw + via_dia[nid] / 2)
        _stamp_disc(m, x, y, HOLE_CLR + mm + hw + via_drill[nid] / 2)
    _rr_masks[key] = m
    return m


def _clear_disc(m, x, y, r):
    i0 = max(0, int((x - r - RX0) / RR_GRID)); i1 = min(RW - 1, int((x + r - RX0) / RR_GRID) + 1)
    j0 = max(0, int((y - r - RY0) / RR_GRID)); j1 = min(RH - 1, int((y + r - RY0) / RR_GRID) + 1)
    r2 = r * r
    for j in range(j0, j1 + 1):
        cy_ = RY0 + j * RR_GRID
        base = j * RW
        for i in range(i0, i1 + 1):
            if (RX0 + i * RR_GRID - x) ** 2 + (cy_ - y) ** 2 <= r2:
                m[base + i] = 0

_DIRS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
         (1, 1, 1.41421356), (1, -1, 1.41421356), (-1, 1, 1.41421356), (-1, -1, 1.41421356)]


def _rr_snap(m, x, y, rad=10):
    ci0 = int(round((x - RX0) / RR_GRID)); cj0 = int(round((y - RY0) / RR_GRID))
    for r in range(rad + 1):
        best = None
        for dj in range(-r, r + 1):
            for di in range(-r, r + 1):
                if max(abs(di), abs(dj)) != r:
                    continue
                i, j = ci0 + di, cj0 + dj
                if _rinb(i, j) and not m[j * RW + i]:
                    d = di * di + dj * dj
                    if best is None or d < best[0]:
                        best = (d, i, j)
        if best:
            return best[1], best[2]
    return None


def rr_astar(m, a, b):
    sa = _rr_snap(m, a[0], a[1]); sb = _rr_snap(m, b[0], b[1])
    if not sa or not sb:
        return None
    start = sa[1] * RW + sa[0]; goal = sb[1] * RW + sb[0]
    gx, gy = _rxy(goal)

    def h(idx):
        x, y = _rxy(idx)
        dx, dy = abs(x - gx), abs(y - gy)
        return max(dx, dy) + 0.41421356 * min(dx, dy)
    openq = [(h(start), 0.0, start)]
    gbest = {start: 0.0}
    par = {}
    pops = 0
    while openq and pops < RR_MAXPOP:
        f, gg, idx = heapq.heappop(openq)
        pops += 1
        if gg > gbest.get(idx, 1e18) + 1e-12:
            continue
        if idx == goal:
            path = [idx]
            while path[-1] in par:
                path.append(par[path[-1]])
            return [_rxy(c) for c in reversed(path)]
        i, j = idx % RW, idx // RW
        for di, dj, c in _DIRS:
            ni, nj = i + di, j + dj
            if not _rinb(ni, nj):
                continue
            nidx = nj * RW + ni
            if m[nidx]:
                continue
            if di and dj and (m[j * RW + ni] or m[nj * RW + i]):
                continue
            ng = gg + c * RR_GRID
            if ng < gbest.get(nidx, 1e18) - 1e-12:
                gbest[nidx] = ng
                par[nidx] = idx
                heapq.heappush(openq, (ng + h(nidx), ng, nidx))
    return None


def rr_smooth(pts, m):
    def ok(a, b):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(2, int(L / (RR_GRID * 0.4)))
        for k in range(n + 1):
            t = k / n
            x = a[0] + (b[0] - a[0]) * t; y = a[1] + (b[1] - a[1]) * t
            i = int(round((x - RX0) / RR_GRID)); j = int(round((y - RY0) / RR_GRID))
            if not _rinb(i, j) or m[j * RW + i]:
                return False
        return True
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not ok(pts[i], pts[j]):
            j -= 1
        out.append(pts[j])
        i = j
    return out


def rr_apply(ci, mid_pts):
    e = E[ci]
    a_id, b_id = chains[ci][0], chains[ci][-1]
    poly = [tuple(P[a_id])] + [tuple(q) for q in mid_pts] + [tuple(P[b_id])]
    dense = [poly[0]]
    for k in range(len(poly) - 1):
        (x1, y1), (x2, y2) = poly[k], poly[k + 1]
        L = math.hypot(x2 - x1, y2 - y1)
        if L < 1e-6:
            continue
        n = max(1, int(math.ceil(L / SUB)))
        for t in range(1, n + 1):
            dense.append((x1 + (x2 - x1) * t / n, y1 + (y2 - y1) * t / n))
    tx = dict(ci=ci, old_ids=chains[ci], old_sis=chain_segs[ci][:], np0=len(P), ns0=len(segs))
    ids = [a_id]
    for x, y in dense[1:-1]:
        ids.append(_new_node(x, y, e['net']))
    ids.append(b_id)
    for si in tx['old_sis']:
        segs[si]['dead'] = True
    new_sis = []
    for k in range(len(ids) - 1):
        si = len(segs)
        segs.append(dict(a=ids[k], b=ids[k + 1], layer=e['layer'], w=e['w'], net=e['net'], c=ci))
        inc[ids[k]].append(si); inc[ids[k + 1]].append(si)
        new_sis.append(si)
    chains[ci] = ids
    chain_segs[ci] = new_sis
    for nid in tx['old_ids'][1:-1]:
        press.pop(nid, None)
    recompute_net_len()
    return tx


def rr_rollback(tx):
    ci = tx['ci']
    for si in tx['old_sis']:
        segs[si].pop('dead', None)
    del segs[tx['ns0']:]
    for arr in (P, is_via, via_dia, via_drill, node_net):
        del arr[tx['np0']:]
    for nid in (chains[ci][0], chains[ci][-1]):
        inc[nid] = [si for si in inc[nid] if si < tx['ns0']]
    for nid in chains[ci][1:-1]:
        inc.pop(nid, None); press.pop(nid, None)
    chains[ci] = tx['old_ids']
    chain_segs[ci] = tx['old_sis']
    recompute_net_len()


def _at_pair_margin(key):
    base = key[1:]
    t = base[0]
    best = -1e9
    if t == 'SS':
        for si in chain_segs[base[1]]:
            if segs[si].get('dead'):
                continue
            A = seg_desc(si, {}, 0)
            for sj in chain_segs[base[2]]:
                if segs[sj].get('dead') or segs[si]['layer'] != segs[sj]['layer']:
                    continue
                best = max(best, at_margin(A, seg_desc(sj, {}, 0)))
        return best
    if t == 'SN':
        B = via_desc(base[2], {}, 0)
    elif t == 'ST':
        B = track_desc[base[2]]
    elif t == 'SV':
        B = svia_desc[base[2]]
    else:
        B = pad_desc(base[2], {}, 0)
    for si in chain_segs[base[1]]:
        if segs[si].get('dead'):
            continue
        if t == 'ST' and segs[si]['layer'] != M['tracks'][base[2]]['layer']:
            continue
        best = max(best, at_margin(seg_desc(si, {}, 0), B))
    return best


def rr_verify(ci):
    build_dyn_grids()

    def gen():
        for si in chain_segs[ci]:
            for key, mm, blk in gen_seg(si, {}, {}, 0.0):
                if key[0] != 'AT':
                    yield key, mm, blk
    bad = _violations(_eval(gen()), None)
    if bad:
        return False, bad
    net = E[ci]['net']
    if net_len[net] - net_len0.get(net, 0.0) > net_cap_mm(net):
        return False, [(('LEN', net), 0.0, ('L', net))]
    for k in gf:
        if k[0] != 'AT':
            continue
        t = k[1]
        if not ((t == 'SS' and ci in (k[2], k[3])) or (t in ('SN', 'ST', 'SV', 'SP') and k[2] == ci)):
            continue
        mm = _at_pair_margin(k)
        if mm < floor_of(k) - EPS:
            return False, [(k, mm, ('AT',))]
    return True, []

rr_events = []
rr_ok_count = defaultdict(int)
_absorb_seen = {}
RR_OK_CAP = 3


def rr_candidates():
    score = defaultdict(float)
    for name, clamps in blocked_log.items():
        own = None
        for key, mm, blk in clamps:
            if blk[0] == 'S':
                score[segs[blk[1]]['c']] += 1000
            elif blk[0] == 'N':
                for si in inc[blk[1]]:
                    s = segs[si]
                    if not s.get('dead'):
                        score[s['c']] += 500
            elif blk[0] in ('P', 'T', 'V'):
                if own is None:
                    own = set()
                    refs = dict(units).get(name, [name] if name in movable else [])
                    for r in refs:
                        for nid in part_anchors.get(r, ()):
                            for si in inc[nid]:
                                s = segs[si]
                                if not s.get('dead'):
                                    own.add(s['c'])
                for ci in own:
                    score[ci] += 800
    for k, n in absorb.items():
        dn = n - _absorb_seen.get(k, 0)
        mover = k[0]
        if dn < 30 or not mover.startswith('node'):
            continue
        for si in inc.get(int(mover[4:]), ()):
            s = segs[si]
            if not s.get('dead'):
                score[s['c']] += dn / 10
    _absorb_seen.update(absorb)
    return [ci for ci, _ in sorted(score.items(), key=lambda kv: -kv[1]) if rr_ok_count[ci] < RR_OK_CAP]


def rr_route(ci, avoid=None):
    e = E[ci]
    hw = e['w'] / 2
    a, b = P[chains[ci][0]], P[chains[ci][-1]]
    for mm in (RR_M, 0.004):
        m = bytearray(rr_mask(e['layer'], e['net'], hw, mm))
        _clear_disc(m, a[0], a[1], 0.35)
        _clear_disc(m, b[0], b[1], 0.35)
        for (kind, idx), bumps in (avoid or {}).items():
            extra = CLR + hw + GUARD + 0.05 * bumps
            if kind == 'P':
                p = pads[idx]
                _stamp_poly(m, p['pts'], extra)
                if p['drill']:
                    _stamp_disc(m, p['x'], p['y'], extra + p['drill'] / 2)
            elif kind == 'T':
                t = M['tracks'][idx]
                _stamp_seg(m, t['a'][0], t['a'][1], t['b'][0], t['b'][1], extra + t['w'] / 2)
            elif kind == 'V':
                v = M['vias'][idx]
                _stamp_disc(m, v['x'], v['y'], extra + v['dia'] / 2)
            elif kind == 'N':
                _stamp_disc(m, P[idx][0], P[idx][1], extra + via_dia[idx] / 2)
            elif kind == 'S':
                for si in chain_segs[idx]:
                    s = segs[si]
                    if s.get('dead'):
                        continue
                    (ax, ay), (bx, by) = P[s['a']], P[s['b']]
                    _stamp_seg(m, ax, ay, bx, by, extra + s['w'] / 2)
        path = rr_astar(m, a, b)
        if path is not None:
            return rr_smooth(path, m)
    return None


def _rr_noop(tx, tol=0.05):
    """A verified reroute that retraces the old polyline changes nothing —
    retrying it forever is the treadmill. True if every new node lies within
    tol of the old path."""
    old = [tuple(P[i]) for i in tx['old_ids']]
    for nid in chains[tx['ci']]:
        x, y = P[nid]
        if min(_pt_seg(x, y, old[k][0], old[k][1], old[k + 1][0], old[k + 1][1])
               for k in range(len(old) - 1)) > tol:
            return False
    return True


def reroute_round(rn):
    _rr_masks.clear()
    build_static_grid()
    build_dyn_grids()
    cands = rr_candidates()
    nok = tried = 0
    for ci in cands:
        if tried >= RR_PER_ROUND:
            break
        tried += 1
        e = E[ci]
        ev = dict(round=rn, chain=ci, net=e['net'], layer=e['layer'])
        avoid = {}
        for attempt in range(3):
            path = rr_route(ci, avoid)
            if path is None:
                ev['result'] = 'no_path'
                break
            tx = rr_apply(ci, path)
            ok, bad = rr_verify(ci)
            if ok and _rr_noop(tx):
                rr_rollback(tx)
                build_dyn_grids()
                rr_ok_count[ci] = RR_OK_CAP
                ev['result'] = 'noop'
                break
            if ok:
                nok += 1
                rr_ok_count[ci] += 1
                _rr_masks.clear()
                ev['result'] = 'ok'
                ev['nodes'] = len(chains[ci])
                ev.pop('why', None)
                break
            rr_rollback(tx)
            build_dyn_grids()
            ev['result'] = 'reject'
            ev['why'] = str(bad[0][0])
            fatal = False
            for key, mv, blk in bad:
                if key[0] in ('AT', 'LEN') or blk[0] == 'E':
                    fatal = True
                    break
                tgt = (blk[0], segs[blk[1]]['c']) if blk[0] == 'S' else (blk[0], blk[1])
                avoid[tgt] = avoid.get(tgt, 0) + 1
            if fatal:
                break
        rr_events.append(ev)
    print(f'  reroute round {rn}: {nok}/{tried} chains rerouted ({len(cands)} candidates)', flush=True)
    return nok

# ---- trajectory --------------------------------------------------------------
cps = []
targets = {r for r in movable if parts[r].get('target')}


def progress():
    vals = [gv for gv in g]
    for r in targets:
        t = parts[r]['target']
        L = math.hypot(*t) or 1.0
        vals.append(min(1.0, (D[r][0] * t[0] + D[r][1] * t[1]) / (L * L)))
    return sum(vals) / len(vals) if vals else 1.0


def capture(tag=''):
    s = progress()
    cps.append(dict(s=s, P=[p[:] for p in P], D={r: D[r][:] for r in D},
                    G=g[:], T=[t[:] for t in T], chains=[ids[:] for ids in chains], tag=tag))
    print(f'  checkpoint {len(cps)}: s={s:.3f} {tag}', flush=True)


def length_report():
    rows = []
    for net, L0 in sorted(net_len0.items()):
        if not L0:
            continue
        el = net_el(net)
        gr = net_growth(net)
        cap = net_cap_mm(net)
        rows.append(dict(net=net, L0=round(L0, 3), L=round(net_len[net], 3),
                         static=round(net_static.get(net, 0.0), 3), growth=round(gr, 4),
                         grown_mm=round(net_len[net] - L0, 3),
                         allow_mm=round(net_allow_mm(net), 3) if el['allow'] < 1e6 else None,
                         cap_mm=round(cap, 3) if cap < 1e8 else None,
                         allow=el['allow'] if el['allow'] < 1e6 else None,
                         cap=el['cap'] if el['cap'] < 1e6 else None,
                         at_cap=bool(cap < 1e8 and net_len[net] - L0 >= cap - 1e-4)))
    rows.sort(key=lambda r: -r['growth'])
    return rows

t0 = time.time()
measure()
if '--measure-only' in sys.argv:
    sys.exit(0)
capture('start')

hist = []
idle = 0
best_s = 0.0
cyc = 0
scount = 1
win = STALL_WIN
rounds = 0
relaxing = 0
while cyc < CYCLES:
    cyc += 1
    build_static_grid()
    build_dyn_grids()
    build_cy_pairs()
    adv = ghost_pass()
    padv, nblk = part_pass()
    adv += padv
    adv += group_pass()
    cu = copper_pass()
    s = progress()
    arrived = sum(1 for gv in g if gv >= 1.0)
    hist.append(dict(cycle=cyc, s=round(s, 4), ghosts=arrived, blocked=nblk,
                     adv=round(adv, 4), copper=round(cu, 4)))
    while scount <= SNAPSHOTS and s >= scount / SNAPSHOTS - 1e-9:
        capture()
        scount += 1
    if cyc % 5 == 0 or cyc == 1:
        print(f'cycle {cyc}: s={s:.3f} ghosts={arrived}/{len(g)} blocked={nblk} '
              f'adv={adv:.3f}mm copper={cu:.3f}mm [{time.time() - t0:.0f}s]', flush=True)
    if s >= 1.0 - 1e-9:
        relaxing += 1
        if relaxing == 1:
            print(f'all ghosts born / targets reached after {cyc} cycles; relaxing {RELAX} cycles')
        if relaxing >= RELAX or (adv + cu) < IDLE_ADV:
            break
        continue
    if s > best_s + 1e-3:
        best_s, idle = s, 0
    else:
        idle += 1
    if idle >= win:
        if rounds < RR_ROUNDS:
            print(f'stall at cycle {cyc} (s={s:.3f}): reroute round {rounds + 1}', flush=True)
            if reroute_round(rounds + 1):
                rounds += 1
                idle = 0
                best_s = s
                win = RR_WIN
                capture('reroute')
                continue
        print(f'STALL after {cyc} cycles ({win} without advance, {rounds} reroute rounds): '
              f's={s:.3f}, {len(g) - arrived} ghosts unborn')
        break

if not cps or cps[-1]['s'] < progress() - 1e-9 or cps[-1]['tag'] != 'final':
    capture('final')

save_json(cfg, 'traj.json', dict(checkpoints=cps, chains=chains))

stalled = []
for name, clamps in blocked_log.items():
    stalled.append(dict(unit=name, hard_walled=hard_walled(name),
                        blockers=[dict(key=str(k), margin=round(m, 4), floor=round(floor_of(k), 4),
                                       **blk_info(b)) for k, m, b in clamps][:12]))
report = dict(
    cycles=cyc, s=round(progress(), 4), ghosts={gh['ref']: dict(g=round(g[i], 4), T=[round(t, 3) for t in T[i]])
                                               for i, gh in enumerate(ghosts)},
    parts_moved={r: [round(v, 3) for v in D[r]] for r in D if math.hypot(*D[r]) > 1e-3},
    reroutes=rr_events, reroute_rounds=rounds, stalled=stalled,
    absorb=[dict(mover=a, blocker=b, what=c, hits=n)
            for (a, b, c), n in sorted(absorb.items(), key=lambda kv: -kv[1])][:60],
    lengths=length_report(), history=hist, seconds=round(time.time() - t0, 1))
save_json(cfg, 'settle_report.json', report)
edge_hits = sum(n for (a, b, c), n in absorb.items() if b == 'board_edge')
cap_hits = sum(n for (a, b, c), n in absorb.items() if b == 'length_cap')
over = [r for r in report['lengths'] if r['allow_mm'] is not None and r['grown_mm'] > r['allow_mm']]
print(f'wrote traj.json ({len(cps)} checkpoints) + settle_report.json in {time.time() - t0:.0f}s: '
      f'{len(report["parts_moved"])} parts moved, {len(over)} nets over allowance, '
      f'{cap_hits} length-cap hits, {edge_hits} board-edge hits — run emit.py')
