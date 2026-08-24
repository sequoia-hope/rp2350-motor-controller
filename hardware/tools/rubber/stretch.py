#!/usr/bin/env python3
"""Rubber stretch: dilate the board about its centre by (KX, KY) with
footprints as rigid inclusions.

Every footprint moves rigidly by the dilation of its origin. Copper is owned
by connectivity (same graph as modules/warp2.py): a track end on a pad is
pinned to that pad's footprint vector, so nothing detaches; every other
track end / via node takes  ambient(x,y) + H,  where ambient is the pure
dilation field ((KX-1)(x-cx), (KY-1)(y-cy)) and H is the graph-harmonic
interpolation of the pinned residuals (footprint vector minus ambient) —
the rigid-inclusion lag decays smoothly along each net's own copper instead
of stepping at the last segment. Pad-less clusters (stitch farms) take pure
ambient per node: dilation only grows their spacings. Zone outlines follow
the footprint field inside a courtyard, ambient outside; zones refill.
Edge.Cuts lines stretch, corner arcs translate rigidly (radius kept) and the
lines re-snap to their endpoints. Silk text and free graphics ride ambient.

env: SRC, OUT, KX, KY, CX, CY   (defaults: main board, _rubber copy, 1.0, centre)
"""
import json, math, os, shutil, sys, collections
S = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, S)
HW = os.path.dirname(os.path.dirname(S))
sys.path.insert(0, os.path.join(HW, 'tools', 'jiggle2'))
from common import load_board, NM, quiet_stderr
import pcbnew

SRC = os.environ.get('SRC', os.path.join(HW, 'rp2350_driver.kicad_pcb'))
OUT = os.environ.get('OUT', os.path.join(HW, 'rp2350_driver_rubber.kicad_pcb'))
KX = float(os.environ.get('KX', '1.0')); KY = float(os.environ.get('KY', '1.0'))

bd = load_board(SRC)
bb = bd.GetBoardEdgesBoundingBox()
CX = float(os.environ.get('CX', (bb.GetLeft() + bb.GetRight()) / 2 * NM))
CY = float(os.environ.get('CY', (bb.GetTop() + bb.GetBottom()) / 2 * NM))

def ambient(x, y):
    return ((KX - 1.0) * (x - CX), (KY - 1.0) * (y - CY))

def to_v(x, y):
    return pcbnew.VECTOR2I(int(round(x / NM)), int(round(y / NM)))

# ---- footprints: rigid, vector = dilation of the origin ---------------------
V = {}          # ref -> (vx, vy)
terr = []       # (poly, bbox, ref) courtyard territories, OLD coordinates
for f in bd.GetFootprints():
    r = f.GetReference(); p = f.GetPosition()
    V[r] = ambient(p.x * NM, p.y * NM)
    try: f.BuildCourtyardCaches()
    except AttributeError: pass
    for lay in (pcbnew.F_CrtYd, pcbnew.B_CrtYd):
        ps = f.GetCourtyard(lay)
        for i in range(ps.OutlineCount()):
            o = ps.Outline(i)
            pts = [(o.CPoint(j).x * NM, o.CPoint(j).y * NM) for j in range(o.PointCount())]
            if len(pts) >= 3:
                xs = [q[0] for q in pts]; ys = [q[1] for q in pts]
                terr.append((pts, (min(xs), min(ys), max(xs), max(ys)), r,
                             'F' if lay == pcbnew.F_CrtYd else 'B'))

def pip(x, y, pts):
    inside = False; j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi = pts[i]; xj, yj = pts[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-300) + xi:
            inside = not inside
        j = i
    return inside

def fp_field(x, y):
    """footprint vector if (x,y) is inside a courtyard (old coords), else ambient."""
    for pts, tb, r, _side in terr:
        if tb[0] <= x <= tb[2] and tb[1] <= y <= tb[3] and pip(x, y, pts):
            return V[r]
    return ambient(x, y)

# ---- courtyard relax: rigid-lag must not close any courtyard pair -----------
def _ss_close(a1, a2, b1, b2):
    best = None
    for (p, q1, q2, flip) in ((a1, b1, b2, 0), (a2, b1, b2, 0), (b1, a1, a2, 1), (b2, a1, a2, 1)):
        ax, ay = q1; bx, by = q2; px, py = p
        dx, dy = bx - ax, by - ay; L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
        c = (ax + t * dx, ay + t * dy)
        d = math.hypot(px - c[0], py - c[1])
        pa, pb = (p, c) if not flip else (c, p)
        if best is None or d < best[0]: best = (d, pa, pb)
    return best

def _orient(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])

def poly_gap(A, B, va=(0, 0), vb=(0, 0)):
    """(dist, pa, pb, crossing) of A+va vs B+vb boundaries; crossing includes containment."""
    At = [(x + va[0], y + va[1]) for x, y in A]; Bt = [(x + vb[0], y + vb[1]) for x, y in B]
    best = None; cross = False
    for i in range(len(At)):
        a1, a2 = At[i], At[(i + 1) % len(At)]
        for j in range(len(Bt)):
            b1, b2 = Bt[j], Bt[(j + 1) % len(Bt)]
            d1, d2 = _orient(b1, b2, a1), _orient(b1, b2, a2)
            d3, d4 = _orient(a1, a2, b1), _orient(a1, a2, b2)
            if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)): cross = True
            r = _ss_close(a1, a2, b1, b2)
            if best is None or r[0] < best[0]: best = r
    if not cross and (pip(At[0][0], At[0][1], Bt) or pip(Bt[0][0], Bt[0][1], At)): cross = True
    return best[0], best[1], best[2], cross

def _pen(A, B, va=(0, 0), vb=(0, 0)):
    """max penetration of A+va / B+vb vertices into the other poly ->
    (depth, ux, uy): unit push direction that separates B from A."""
    At = [(x + va[0], y + va[1]) for x, y in A]; Bt = [(x + vb[0], y + vb[1]) for x, y in B]
    best = (0.0, 1.0, 0.0)
    for P, Q, sgn in ((At, Bt, 1.0), (Bt, At, -1.0)):
        # vertices of P inside Q: depth = min distance to Q's boundary
        for (px, py) in P:
            if not pip(px, py, Q): continue
            bd_ = None
            for j in range(len(Q)):
                r = _ss_close((px, py), (px, py), Q[j], Q[(j + 1) % len(Q)])
                if bd_ is None or r[0] < bd_[0]: bd_ = r
            if bd_ and bd_[0] > best[0]:
                # push direction: from the vertex toward Q's boundary = out of Q
                ux, uy = bd_[2][0] - px, bd_[2][1] - py
                Lu = math.hypot(ux, uy) or 1.0
                # sgn=+1: A-vertex inside B -> B must move away along -(out of B)... 
                # convention: return direction to ADD to V[rb] (B's motion)
                best = (bd_[0], sgn * -ux / Lu, sgn * -uy / Lu)
    return best

cy_by = collections.defaultdict(list)
for pts, tb, r, side in terr: cy_by[(r, side)].append((pts, tb))
npads = {f.GetReference(): max(1, len(list(f.Pads()))) for f in bd.GetFootprints()}
prs = []
refs = list(cy_by)
for i in range(len(refs)):
    for j in range(i + 1, len(refs)):
        (ra, sa), (rb, sb) = refs[i], refs[j]
        if ra == rb or sa != sb: continue
        for pa, ba in cy_by[refs[i]]:
            for pb, bb2 in cy_by[refs[j]]:
                if max(ba[0] - bb2[2], bb2[0] - ba[2], ba[1] - bb2[3], bb2[1] - ba[3], 0) > 1.2: continue
                g0, _, _, cross0 = poly_gap(pa, pb)
                if g0 < 0.6 or cross0: prs.append((ra, rb, pa, pb, g0, cross0))
n_corr = 0; corr_max = 0.0
for it in range(80):
    worst = 0.0
    for ra, rb, pa, pb, g0, cross0 in prs:
        axc = sum(x for x, y in pa) / len(pa); ayc = sum(y for x, y in pa) / len(pa)
        bxc = sum(x for x, y in pb) / len(pb); byc = sum(y for x, y in pb) / len(pb)
        if cross0:
            # already touching in pcbnew geometry: penetration depth must not grow.
            pen0 = _pen(pa, pb)
            pen = _pen(pa, pb, V[ra], V[rb])
            if pen[0] <= pen0[0] + 1e-6: continue
            dlt = pen[0] - pen0[0]
            ux, uy = pen[1], pen[2]
        else:
            req = min(g0, 0.1)
            d, cpa, cpb, cross = poly_gap(pa, pb, V[ra], V[rb])
            if not cross and d >= req - 1e-6: continue
            if cross:
                dlt = 0.02; ux, uy = bxc - axc, byc - ayc
            else:
                dlt = req - d; ux, uy = cpb[0] - cpa[0], cpb[1] - cpa[1]
        Lu = math.hypot(ux, uy) or 1.0
        ux, uy = ux / Lu, uy / Lu
        ma, mb = 1.0 / npads[ra], 1.0 / npads[rb]
        wa, wb = ma / (ma + mb), mb / (ma + mb)
        V[ra] = (V[ra][0] - ux * dlt * wa, V[ra][1] - uy * dlt * wa)
        V[rb] = (V[rb][0] + ux * dlt * wb, V[rb][1] + uy * dlt * wb)
        worst = max(worst, dlt); n_corr += 1; corr_max = max(corr_max, dlt)
    if worst < 1e-5: break
n_bad = 0
for ra, rb, pa, pb, g0, cross0 in prs:
    if cross0: continue
    d, _, _, cross = poly_gap(pa, pb, V[ra], V[rb])
    if cross or d < min(g0, 0.1) - 1e-4: n_bad += 1
print(f'  courtyard relax: {len(prs)} close pairs, {n_corr} corrections '
      f'(max {corr_max*1000:.1f} um), {n_bad} unresolved')

n_fp = 0
for f in bd.GetFootprints():
    v = V[f.GetReference()]
    if abs(v[0]) > 1e-9 or abs(v[1]) > 1e-9:
        p = f.GetPosition()
        f.SetPosition(pcbnew.VECTOR2I(p.x + int(round(v[0] / NM)), p.y + int(round(v[1] / NM))))
        n_fp += 1

# pads are at NEW positions; hit-test old points against pad translated back
pads = []
for f in bd.GetFootprints():
    v = V[f.GetReference()]
    for p in f.Pads():
        pb = p.GetBoundingBox()
        pads.append(((pb.GetLeft() * NM - v[0], pb.GetTop() * NM - v[1],
                      pb.GetRight() * NM - v[0], pb.GetBottom() * NM - v[1]), p, v))

def pad_hit(x, y, layer, netcode):
    for pb, p, v in pads:
        if not (pb[0] - 1e-4 <= x <= pb[2] + 1e-4 and pb[1] - 1e-4 <= y <= pb[3] + 1e-4):
            continue
        if layer is not None and not p.IsOnLayer(layer):
            continue
        if p.GetNetCode() != netcode:
            continue
        if p.HitTest(to_v(x + v[0], y + v[1])):
            return v
    return None

# ---- copper graph (residuals; adapted from modules/warp2.py) ----------------
tracks = [t for t in bd.GetTracks() if t.GetClass() == 'PCB_TRACK']
vias = [t for t in bd.GetTracks() if t.GetClass() == 'PCB_VIA']
node_of = {}
nodes = []      # dict(x, y, res:(rx,ry)|None, net)   res = pinned residual
adj = collections.defaultdict(list)
via_nodes = []; tnodes = []

def new_node(x, y, net):
    nodes.append(dict(x=x, y=y, res=None, net=net)); return len(nodes) - 1

def residual(vec, x, y):
    a = ambient(x, y); return (vec[0] - a[0], vec[1] - a[1])

conn = bd.GetConnectivity()
pad_vec = {}
for f in bd.GetFootprints():
    v = V[f.GetReference()]
    for p in f.Pads():
        pad_vec[p.m_Uuid.AsString()] = v

via_index = collections.defaultdict(list)
for vo in vias:
    p = vo.GetPosition(); x, y = p.x * NM, p.y * NM; nc = vo.GetNetCode()
    nid = new_node(x, y, nc); via_nodes.append((vo, nid))
    try: r = vo.GetWidth(pcbnew.PADSTACK.ALL_LAYERS) * NM / 2
    except Exception: r = 0.3
    via_index[(int(x // 1.0), int(y // 1.0))].append((x, y, r, nc, nid))
    fv = pad_hit(x, y, None, nc)
    if fv is None:
        try:
            for cp in conn.GetConnectedPads(vo):
                pv = pad_vec.get(cp.m_Uuid.AsString())
                if pv is not None: fv = pv; break
        except Exception: pass
    if fv is not None:
        nodes[nid]['res'] = residual(fv, x, y)

def find_via(x, y, nc):
    for cx in (int(x // 1.0) - 1, int(x // 1.0), int(x // 1.0) + 1):
        for cy in (int(y // 1.0) - 1, int(y // 1.0), int(y // 1.0) + 1):
            for vx, vy, r, vnc, nid in via_index.get((cx, cy), ()):
                if vnc == nc and math.hypot(vx - x, vy - y) <= r + 1e-6:
                    return nid
    return None

n_pin = n_overlap_pin = 0
for t in tracks:
    s, e = t.GetStart(), t.GetEnd(); lay = t.GetLayer(); nc = t.GetNetCode()
    ends = []
    for P in (s, e):
        x, y = P.x * NM, P.y * NM
        nid = find_via(x, y, nc)
        if nid is None:
            key = (P.x, P.y, int(lay), nc)
            nid = node_of.get(key)
            if nid is None:
                nid = new_node(x, y, nc); node_of[key] = nid
                fv = pad_hit(x, y, lay, nc)
                if fv is not None:
                    nodes[nid]['res'] = residual(fv, x, y); n_pin += 1
        ends.append(nid)
    na, nb = ends
    try: cpads = list(conn.GetConnectedPads(t))
    except Exception: cpads = []
    for cp in cpads:
        pv = pad_vec.get(cp.m_Uuid.AsString())
        if pv is None: continue
        pc = cp.GetPosition(); pcx, pcy = pc.x * NM, pc.y * NM
        da = math.hypot(s.x * NM - pcx, s.y * NM - pcy)
        db = math.hypot(e.x * NM - pcx, e.y * NM - pcy)
        nid = na if da <= db else nb
        if nodes[nid]['res'] is None:
            nodes[nid]['res'] = residual(pv, nodes[nid]['x'], nodes[nid]['y']); n_overlap_pin += 1
    L = math.hypot((e.x - s.x) * NM, (e.y - s.y) * NM)
    w = 1.0 / max(L, 0.02)
    adj[na].append((nb, w)); adj[nb].append((na, w))
    tnodes.append((t, na, nb))

# components + harmonic residual solve
seen = [False] * len(nodes); comps = []
for i in range(len(nodes)):
    if seen[i]: continue
    stack = [i]; seen[i] = True; comp = []
    while stack:
        n = stack.pop(); comp.append(n)
        for m, w in adj[n]:
            if not seen[m]: seen[m] = True; stack.append(m)
    comps.append(comp)

H = [None] * len(nodes)
n_free_cl = n_pin_cl = 0
for comp in comps:
    pinned = [n for n in comp if nodes[n]['res'] is not None]
    if not pinned:
        for n in comp: H[n] = (0.0, 0.0)     # pure ambient: stitch farms etc.
        n_free_cl += 1
        continue
    n_pin_cl += 1
    vals = {tuple(nodes[n]['res']) for n in pinned}
    if len(vals) == 1:
        v = next(iter(vals))
        for n in comp: H[n] = v
        continue
    cur = {n: (nodes[n]['res'] if nodes[n]['res'] is not None else None) for n in comp}
    free = [n for n in comp if cur[n] is None]
    mx = sum(v[0] for v in vals) / len(vals); my = sum(v[1] for v in vals) / len(vals)
    for n in free: cur[n] = (mx, my)
    for it in range(3000):
        dmax = 0.0
        for n in free:
            sw = sx = sy = 0.0
            for m, w in adj[n]:
                if m in cur: sw += w; sx += w * cur[m][0]; sy += w * cur[m][1]
            if sw == 0: continue
            nv = (sx / sw, sy / sw)
            dmax = max(dmax, abs(nv[0] - cur[n][0]), abs(nv[1] - cur[n][1]))
            cur[n] = nv
        if dmax < 1e-7: break
    for n in comp: H[n] = cur[n]

def node_disp(n):
    a = ambient(nodes[n]['x'], nodes[n]['y']); h = H[n]
    return (a[0] + h[0], a[1] + h[1])

n_t = n_v = 0
for t, na, nb in tnodes:
    da, db = node_disp(na), node_disp(nb)
    s, e = t.GetStart(), t.GetEnd()
    t.SetStart(to_v(s.x * NM + da[0], s.y * NM + da[1]))
    t.SetEnd(to_v(e.x * NM + db[0], e.y * NM + db[1])); n_t += 1
for vo, nid in via_nodes:
    d = node_disp(nid)
    p = vo.GetPosition()
    vo.SetPosition(to_v(p.x * NM + d[0], p.y * NM + d[1])); n_v += 1

# ---- zones ------------------------------------------------------------------
n_zv = 0
for z in bd.Zones():
    o = z.Outline()
    for i in range(o.TotalVertices()):
        p = o.CVertex(i); x, y = p.x * NM, p.y * NM
        v = fp_field(x, y)
        o.SetVertex(i, to_v(x + v[0], y + v[1])); n_zv += 1

# ---- edge cuts: map every defining point through ambient ---------------------
# (a circle maps to a circle under isotropic scale; corner radii grow by k)
for d in [d for d in bd.GetDrawings() if d.GetLayer() == pcbnew.Edge_Cuts]:
    if d.GetShape() == pcbnew.SHAPE_T_ARC:
        pts = []
        for P in (d.GetStart(), d.GetArcMid(), d.GetEnd()):
            v = ambient(P.x * NM, P.y * NM)
            pts.append(to_v(P.x * NM + v[0], P.y * NM + v[1]))
        d.SetArcGeometry(*pts)
    else:
        s, e = d.GetStart(), d.GetEnd()
        vs = ambient(s.x * NM, s.y * NM); ve = ambient(e.x * NM, e.y * NM)
        d.SetStart(to_v(s.x * NM + vs[0], s.y * NM + vs[1]))
        d.SetEnd(to_v(e.x * NM + ve[0], e.y * NM + ve[1]))

# ---- free text / graphics on other layers -----------------------------------
n_txt = 0
for d in bd.GetDrawings():
    if d.GetLayer() == pcbnew.Edge_Cuts: continue
    p = d.GetPosition(); v = ambient(p.x * NM, p.y * NM)
    d.Move(to_v(v[0], v[1]) - to_v(0, 0)); n_txt += 1

with quiet_stderr():
    pcbnew.ZONE_FILLER(bd).Fill(bd.Zones())
    pcbnew.SaveBoard(OUT, bd)
pro = SRC.replace('.kicad_pcb', '.kicad_pro'); opro = OUT.replace('.kicad_pcb', '.kicad_pro')
if os.path.exists(pro) and os.path.abspath(pro) != os.path.abspath(opro):
    shutil.copy(pro, opro)

res_mags = [math.hypot(*nodes[n]['res']) for n in range(len(nodes)) if nodes[n]['res'] is not None]
print(f'stretch KX={KX} KY={KY} centre=({CX:.2f},{CY:.2f})')
print(f'  {n_fp} footprints moved; graph {len(nodes)} nodes / {len(comps)} clusters '
      f'({n_pin_cl} pinned, {n_free_cl} pad-less); {n_t} tracks, {n_v} vias, {n_zv} zone vertices, {n_txt} drawings')
if res_mags:
    print(f'  rigid-lag residuals: max {max(res_mags)*1000:.1f} um, mean {sum(res_mags)/len(res_mags)*1000:.1f} um')
print('saved', OUT)
