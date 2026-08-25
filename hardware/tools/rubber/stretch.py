#!/usr/bin/env python3
"""Rubber stretch: dilate the board about its centre by (KX, KY) with
footprints as rigid inclusions.

Algorithm A (2026-08-25).  Each footprint translates rigidly, and the resulting
rigid-inclusion lag is interpolated through SPACE by field.WarpField -- one
harmonic displacement field per COPPER LAYER, evaluated by position for every
track, via, pad, zone vertex and silk item on the board.

What that fixes.  The previous engine interpolated the same residual along each
net's own copper graph, so two traces 0.2 mm apart on different nets got
uncorrelated displacements -- one pinned to a lagging footprint, its neighbour
riding near-pure ambient -- and the gap between them changed by the difference
of two unrelated residuals.  That was the entire F-46 violation census.  A field
that is a function of position cannot shear neighbouring copper apart, whatever
nets it belongs to.

What is rigid, and why each piece:
  * a footprint's PADS (not its courtyard -- copper merely passing under a
    courtyard is free to deform, and abutting courtyards fight over the seam),
    plus what those pads enclose (CLOSE/ENCLOSE), so a trace threading a pin
    field is carried by the pins on both sides;
  * every DRILL, on all six layers, whether or not it carries an annulus.  A
    hole pierces the whole stack and has its own DRC clearance, so an NPTH
    mounting hole with no net is as rigid as any pad;
  * every VIA barrel, coupled across the layers it pierces in two passes: it is
    one rigid body and can only be in one place, so it takes the mean of what
    the layers want and that value is handed back to each of them.

The rigid translation itself is ambient(PAD CENTROID), not ambient(origin): the
footprint origin is an arbitrary CAD anchor, and the choice that least disturbs
surrounding copper is the least-squares one over the part's own pads.  On this
board the phase connectors carry their origin 5.59 mm off centroid, which was
dragging their mounting holes through 56 um of needless lag at +1%.

Two geometric passes keep the discretisation honest:
  * adaptive splitting -- a straight segment cannot follow a curved field, so
    each track is subdivided until its chord is within SPLIT_TOL of the field.
    This is also what keeps tee junctions (364 track ends resting on a passing
    segment's body) and pads sitting mid-body on their host.
  * sigma_min census -- the map F(x) = x + u(x) closes a gap iff the smallest
    singular value of I + grad u drops below 1, so the field reports where it is
    contracting before DRC ever runs.  fieldmap.py draws it; sigma_probe.py bins
    it by distance from the nearest rigid territory.

Edge.Cuts stays on pure ambient (the outline is exactly the dilated outline);
the field's far-field equals ambient, so copper near the edge is consistent.
Free silk art is stretched point-by-point through the field; silk text moves
through it and scales by k.  Footprint silk is rigid with its part.

env: SRC, OUT, KX, KY, CX, CY, H (grid mm), MARGIN, CLOSE, ENCLOSE, ANCHOR,
     SPLIT_TOL, VIA_PASSES, STATS, FIELD_DUMP
"""
import json, math, os, shutil, sys, collections, time
S = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, S)
HW = os.path.dirname(os.path.dirname(S))
sys.path.insert(0, os.path.join(HW, 'tools', 'jiggle2'))
from common import load_board, NM, quiet_stderr
from field import WarpField, dilation
import numpy as np
from scipy import ndimage
import pcbnew

SRC = os.environ.get('SRC', os.path.join(HW, 'rp2350_driver.kicad_pcb'))
OUT = os.environ.get('OUT', os.path.join(HW, 'rp2350_driver_rubber.kicad_pcb'))
KX = float(os.environ.get('KX', '1.0')); KY = float(os.environ.get('KY', '1.0'))
H = float(os.environ.get('H', '0.12'))
MARGIN = float(os.environ.get('MARGIN', '8.0'))
CLOSE = float(os.environ.get('CLOSE', '1.0'))
ENCLOSE = os.environ.get('ENCLOSE', 'close')
SPLIT_TOL = float(os.environ.get('SPLIT_TOL', '0.015'))
STATS = os.environ.get('STATS', '')
FIELD_DUMP = os.environ.get('FIELD_DUMP', '')

t_start = time.time()
bd = load_board(SRC)
bb = bd.GetBoardEdgesBoundingBox()
CX = float(os.environ.get('CX', (bb.GetLeft() + bb.GetRight()) / 2 * NM))
CY = float(os.environ.get('CY', (bb.GetTop() + bb.GetBottom()) / 2 * NM))

def ambient(x, y):
    return ((KX - 1.0) * (x - CX), (KY - 1.0) * (y - CY))

def to_v(x, y):
    return pcbnew.VECTOR2I(int(round(x / NM)), int(round(y / NM)))

# ---- footprints: rigid, vector = dilation of the PAD CENTROID ---------------
# A rigid part has one translation to choose, and the choice that least disturbs
# the copper around it is the one minimising sum |v - ambient(pad_i)|^2 over its
# own pads -- which, ambient being affine, is exactly ambient(pad centroid).
# The footprint origin is an arbitrary CAD anchor and is a poor stand-in: on
# this board the phase connectors J1/J2/J9 carry their origin 5.59 mm off their
# pad centroid, so anchoring there dragged every one of their pads -- mounting
# holes included -- through an extra 56 um of lag at +1% for no reason at all.
ANCHOR = os.environ.get('ANCHOR', 'centroid')
V = {}          # ref -> (vx, vy)
anchor_off = []
terr = []       # (poly, bbox, ref, side) courtyard territories, OLD coordinates
for f in bd.GetFootprints():
    r = f.GetReference(); p = f.GetPosition()
    q = [(pd.GetPosition().x * NM, pd.GetPosition().y * NM) for pd in f.Pads()]
    if ANCHOR == 'centroid' and q:
        ax = sum(t[0] for t in q) / len(q); ay = sum(t[1] for t in q) / len(q)
    else:
        ax, ay = p.x * NM, p.y * NM
    anchor_off.append(math.hypot(ax - p.x * NM, ay - p.y * NM))
    V[r] = ambient(ax, ay)
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

# ---- courtyard relax: rigid-lag must not close any courtyard pair -----------
# (dilation moves origins apart, but two territories can still slide past each
# other into a closer approach; this pre-corrects V before the field is built.)
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
        for (px, py) in P:
            if not pip(px, py, Q): continue
            bd_ = None
            for j in range(len(Q)):
                r = _ss_close((px, py), (px, py), Q[j], Q[(j + 1) % len(Q)])
                if bd_ is None or r[0] < bd_[0]: bd_ = r
            if bd_ and bd_[0] > best[0]:
                ux, uy = bd_[2][0] - px, bd_[2][1] - py
                Lu = math.hypot(ux, uy) or 1.0
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
if ANCHOR == 'centroid':
    print(f'  anchor: pad centroid ({sum(1 for d in anchor_off if d > 0.05)} footprints '
          f'off their origin, worst {max(anchor_off):.2f} mm = '
          f'{max(anchor_off)*abs(KX-1)*1000:.0f} um of lag avoided)')


# ---- the spatial fields, one per copper layer -------------------------------
# Territory of a footprint on a layer = its PADS present on that layer.  Two
# reasons it is pads and not courtyards: what physically has to stay rigid is a
# part's pad geometry (copper merely passing under a courtyard is free to
# deform, and a trace under a QFN is exactly that), and abutting courtyards
# fight over the seam cells between them.  All pads of one footprint share one
# vector, so the part stays rigid, and the space between its own pads is bounded
# by that same value on every side so it comes out rigid too, for free.
#
# One field per layer, because the board is six copper layers and clearance
# rules live inside a layer, never between them.  A single 2-D field forces
# parts on opposite sides that overlap in projection to fight over the same
# cells -- measured here as Y1's pad being owned by U16 and U7's by CL1, worth
# up to 57 um of bogus shear.  Layers couple only where copper physically
# pierces them: through-hole pads (rigid on every layer, so Dirichlet in every
# field) and vias (one rigid barrel, so one displacement -- the mean of what
# the layers it spans ask for).
CU = list(bd.GetEnabledLayers().CuStack())
BOUNDS = (bb.GetLeft() * NM, bb.GetTop() * NM, bb.GetRight() * NM, bb.GetBottom() * NM)

def _oct(cx_, cy_, rx, ry, n=12):
    return [(cx_ + rx * math.cos(2 * math.pi * i / n),
             cy_ + ry * math.sin(2 * math.pi * i / n)) for i in range(n)]

pad_polys = {L: collections.defaultdict(list) for L in CU}
n_hole = 0
for f in bd.GetFootprints():
    r = f.GetReference()
    for p in f.Pads():
        polys = []
        try:
            ps = p.GetEffectivePolygon(pcbnew.PADSTACK.ALL_LAYERS)
            for i in range(ps.OutlineCount()):
                o = ps.Outline(i)
                polys.append([(o.CPoint(j).x * NM, o.CPoint(j).y * NM)
                              for j in range(o.PointCount())])
        except Exception:
            pb = p.GetBoundingBox()
            polys.append([(pb.GetLeft() * NM, pb.GetTop() * NM),
                          (pb.GetRight() * NM, pb.GetTop() * NM),
                          (pb.GetRight() * NM, pb.GetBottom() * NM),
                          (pb.GetLeft() * NM, pb.GetBottom() * NM)])
        for L in CU:
            if p.IsOnLayer(L):
                pad_polys[L][r].extend(polys)
        # A DRILL pierces every layer whether or not there is copper on it, and
        # it carries its own DRC clearance, so it is rigid on all six -- an NPTH
        # mounting hole has no net and may have no annulus at all.  Adding it
        # explicitly means we never depend on the copper polygon happening to
        # cover the hole (it does on this board; that is not guaranteed).
        dx, dy = p.GetDrillSizeX() * NM, p.GetDrillSizeY() * NM
        if dx > 0 and dy > 0:
            pos = p.GetPosition()
            hole = _oct(pos.x * NM, pos.y * NM, dx / 2, dy / 2)
            for L in CU:
                pad_polys[L][r].append(hole)
            n_hole += 1
print(f'  {n_hole} drilled holes pinned on all {len(CU)} copper layers '
      f'({sum(1 for f in bd.GetFootprints() for p in f.Pads() if p.GetAttribute() == 3)} NPTH)')

def build_fields(extra=()):
    """One harmonic field per copper layer.  `extra` is a list of
    (polys, vec) rigid inclusions added to EVERY layer -- how a via barrel,
    which is one rigid body piercing the whole stack, is made to mean the same
    thing to each layer it passes through."""
    out = {}
    for L in CU:
        fl = WarpField(BOUNDS, h=H, margin=MARGIN, close=CLOSE, enclose=ENCLOSE,
                       ambient=dilation(CX, CY, KX, KY))
        for r, pl in pad_polys[L].items():
            fl.add_inclusion(pl, V[r], priority=npads.get(r, 1), tag=r)
        for polys, vec in extra:
            fl.add_inclusion(polys, vec, priority=0.0, tag='via')  # pads outrank vias
        fl.solve(verbose=False)
        out[L] = fl
    return out

t0 = time.time()
fields = build_fields()
print(f'  {len(CU)} layer fields ({" ".join(bd.GetLayerName(L) for L in CU)}) '
      f'on a {fields[CU[0]].nx}x{fields[CU[0]].ny} grid @ {H} mm, {time.time()-t0:.1f}s')

def fld1(x, y, L):
    ux, uy = fields[L](np.array([x]), np.array([y]))
    return float(ux[0]), float(uy[0])

def fldn(x, y, lays):
    """mean of the layer fields over `lays` -- a via barrel is one rigid body."""
    sx = sy = 0.0
    for L in lays:
        u = fld1(x, y, L); sx += u[0]; sy += u[1]
    n = max(len(lays), 1)
    return sx / n, sy / n

# ---- footprint moves --------------------------------------------------------
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

# ---- copper nodes -----------------------------------------------------------
# The graph is kept only for node IDENTITY (endpoints shared by several tracks,
# and via barrels, must move as one) and for pad pinning.  There is no solve on
# it any more: a free node's displacement is just the field at its position.
tracks = [t for t in bd.GetTracks() if t.GetClass() == 'PCB_TRACK']
vias = [t for t in bd.GetTracks() if t.GetClass() == 'PCB_VIA']
node_of = {}
nodes = []      # dict(x, y, pin:(vx,vy)|None, net, lays:set)
via_nodes = []; tnodes = []; via_rad = {}

def new_node(x, y, net):
    nodes.append(dict(x=x, y=y, pin=None, net=net, lays=set())); return len(nodes) - 1

via_index = collections.defaultdict(list)
for vo in vias:
    p = vo.GetPosition(); x, y = p.x * NM, p.y * NM; nc = vo.GetNetCode()
    nid = new_node(x, y, nc); via_nodes.append((vo, nid))
    try: r = vo.GetWidth(pcbnew.PADSTACK.ALL_LAYERS) * NM / 2
    except Exception: r = 0.3
    via_rad[nid] = r
    via_index[(int(x // 1.0), int(y // 1.0))].append((x, y, r, nc, nid))
    # CuStack is in physical order, so a via spans the slice it pierces
    i0, i1 = CU.index(vo.TopLayer()), CU.index(vo.BottomLayer())
    nodes[nid]['lays'].update(CU[min(i0, i1):max(i0, i1) + 1])
    fv = pad_hit(x, y, None, nc)
    if fv is not None:
        nodes[nid]['pin'] = fv

def find_via(x, y, nc):
    for cx in (int(x // 1.0) - 1, int(x // 1.0), int(x // 1.0) + 1):
        for cy in (int(y // 1.0) - 1, int(y // 1.0), int(y // 1.0) + 1):
            for vx, vy, r, vnc, nid in via_index.get((cx, cy), ()):
                if vnc == nc and math.hypot(vx - x, vy - y) <= r + 1e-6:
                    return nid
    return None

n_pin = 0
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
                    nodes[nid]['pin'] = fv; n_pin += 1
        nodes[nid]['lays'].add(lay)
        ends.append(nid)
    tnodes.append((t, ends[0], ends[1]))

# node displacement: its layer field (mean over layers for a via barrel),
# except on a pad where the footprint vector is authoritative.  Inside a
# territory the field already equals that vector, so the two agree; what is
# left of the gap measures how well the territories cover the pads.
NX = np.array([n['x'] for n in nodes]); NY = np.array([n['y'] for n in nodes])
LMASK = {L: np.array([L in n['lays'] for n in nodes]) for L in CU}

def node_disps(flds):
    UX = np.zeros(len(nodes)); UY = np.zeros(len(nodes)); W = np.zeros(len(nodes))
    for L in CU:
        m = LMASK[L]
        if not m.any(): continue
        ux, uy = flds[L](NX[m], NY[m])
        UX[m] += ux; UY[m] += uy; W[m] += 1.0
    W[W == 0] = 1.0
    d = list(zip(UX / W, UY / W)); err = 0.0
    for i, n in enumerate(nodes):
        if n['pin'] is not None:
            err = max(err, math.hypot(n['pin'][0] - d[i][0], n['pin'][1] - d[i][1]))
            d[i] = n['pin']
    return d, err

# Pass 1 fixes what each layer wants at every via; the barrel can only be in one
# place, so it takes the mean.  Pass 2 hands that one displacement back to every
# layer as a rigid inclusion, so each layer's field now agrees with the barrel
# where it pierces -- without it a via shears against the copper beside it by
# however much the layers disagreed, which was most of the residual census.
disp, _ = node_disps(fields)
n_via_incl = 0
VIA_PASSES = int(os.environ.get('VIA_PASSES', '2'))
oct8 = [(math.cos(i * math.pi / 4), math.sin(i * math.pi / 4)) for i in range(8)]
for _ in range(VIA_PASSES - 1):
    extra = []
    for vo, nid in via_nodes:
        r = via_rad.get(nid, 0.3)
        extra.append(([[(nodes[nid]['x'] + r * c, nodes[nid]['y'] + r * s) for c, s in oct8]],
                      disp[nid]))
    fields = build_fields(extra)
    disp, _ = node_disps(fields)
    n_via_incl = len(extra)
disp, pin_err = node_disps(fields)

# The solved fields, exactly as applied below, for fieldmap.py to draw.  Dumping
# them rather than rebuilding them in the renderer is the only way the picture
# is guaranteed to be the field the board actually got.
if FIELD_DUMP:
    f0 = fields[CU[0]]
    dump = dict(kx=KX, ky=KY, cx=CX, cy=CY, h=f0.h, x0=f0.x0, y0=f0.y0,
                anchor=ANCHOR, enclose=ENCLOSE, close=CLOSE,
                layers=np.array([bd.GetLayerName(L) for L in CU]))
    for L in CU:
        n = bd.GetLayerName(L)
        dump[f'RX_{n}'] = fields[L].RX.astype(np.float32)
        dump[f'RY_{n}'] = fields[L].RY.astype(np.float32)
        dump[f'OWN_{n}'] = fields[L].owner >= 0
    np.savez_compressed(FIELD_DUMP, **dump)
    print(f'  field dump -> {FIELD_DUMP}')

# ---- adaptive splitting: a chord cannot follow a curved field ---------------
# Subdivide each track until its straight chord is within SPLIT_TOL of the
# field everywhere along it.  This is also what holds tee ends (a track end
# resting on a passing segment's body -- KiCad connects by overlap, not
# topology) and same-net pads crossed mid-body onto their host: both follow the
# field, and the host is never further than SPLIT_TOL from it.
def refine(P0, P1, t0_, t1_, d0, d1, L, out, depth):
    if depth > 5: return
    tm = 0.5 * (t0_ + t1_)
    mx = P0[0] + (P1[0] - P0[0]) * tm; my = P0[1] + (P1[1] - P0[1]) * tm
    dm = fld1(mx, my, L)
    cx_ = 0.5 * (d0[0] + d1[0]); cy_ = 0.5 * (d0[1] + d1[1])
    if math.hypot(dm[0] - cx_, dm[1] - cy_) <= SPLIT_TOL:
        return
    refine(P0, P1, t0_, tm, d0, dm, L, out, depth + 1)
    out.append((tm, (mx, my), dm))
    refine(P0, P1, tm, t1_, dm, d1, L, out, depth + 1)

n_t = n_split = n_newseg = 0; sag_max = 0.0
for t, na, nb in tnodes:
    s, e = t.GetStart(), t.GetEnd(); L = t.GetLayer()
    P0 = (s.x * NM, s.y * NM); P1 = (e.x * NM, e.y * NM)
    d0, d1 = disp[na], disp[nb]
    mx, my = 0.5 * (P0[0] + P1[0]), 0.5 * (P0[1] + P1[1])
    dm = fld1(mx, my, L)
    sag_max = max(sag_max, math.hypot(dm[0] - 0.5 * (d0[0] + d1[0]),
                                      dm[1] - 0.5 * (d0[1] + d1[1])))
    cuts = []
    refine(P0, P1, 0.0, 1.0, d0, d1, L, cuts, 0)
    if cuts:
        pts = [(P0, d0)] + [(p, d) for _, p, d in cuts] + [(P1, d1)]
        t.SetStart(to_v(pts[0][0][0] + pts[0][1][0], pts[0][0][1] + pts[0][1][1]))
        t.SetEnd(to_v(pts[1][0][0] + pts[1][1][0], pts[1][0][1] + pts[1][1][1]))
        for k in range(1, len(pts) - 1):
            nt = pcbnew.PCB_TRACK(bd)
            nt.SetLayer(L); nt.SetWidth(t.GetWidth()); nt.SetNetCode(t.GetNetCode())
            nt.SetStart(to_v(pts[k][0][0] + pts[k][1][0], pts[k][0][1] + pts[k][1][1]))
            nt.SetEnd(to_v(pts[k+1][0][0] + pts[k+1][1][0], pts[k+1][0][1] + pts[k+1][1][1]))
            bd.Add(nt); n_newseg += 1
        n_split += 1
    else:
        t.SetStart(to_v(P0[0] + d0[0], P0[1] + d0[1]))
        t.SetEnd(to_v(P1[0] + d1[0], P1[1] + d1[1]))
    n_t += 1

n_v = 0
for vo, nid in via_nodes:
    d = disp[nid]
    p = vo.GetPosition()
    vo.SetPosition(to_v(p.x * NM + d[0], p.y * NM + d[1])); n_v += 1

# ---- zones ------------------------------------------------------------------
n_zv = 0
for z in bd.Zones():
    lays = [L for L in CU if z.IsOnLayer(L)] or [CU[0]]
    o = z.Outline()
    for i in range(o.TotalVertices()):
        p = o.CVertex(i); x, y = p.x * NM, p.y * NM
        vx, vy = fldn(x, y, lays)
        o.SetVertex(i, to_v(x + vx, y + vy)); n_zv += 1

# ---- edge cuts: pure ambient (the fields' far field agrees with it) ---------
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

# ---- free silk art and text: STRETCH, don't just translate ------------------
# Every defining point of a graphic maps through the field, so a logo, a long
# rule or a box actually deforms with the sheet instead of sliding rigidly.
# Text cannot deform, so it is the one thing that gets the honest compromise:
# its anchor moves through the field and its glyph size and stroke scale by k,
# which is what "stretched" means for something that must stay legible.
# (Footprint silk is NOT touched here -- it belongs to a rigid part and already
# moved with it.)
KAVG = 0.5 * (KX + KY)

def stretch_pt(P, L):
    vx, vy = fld1(P.x * NM, P.y * NM, L)
    return to_v(P.x * NM + vx, P.y * NM + vy)

n_txt = n_art = 0
for d in bd.GetDrawings():
    if d.GetLayer() == pcbnew.Edge_Cuts: continue
    L = CU[-1] if bd.GetLayerName(d.GetLayer()).startswith('B.') else CU[0]
    cls = d.GetClass()
    if cls in ('PCB_TEXT', 'PCB_TEXTBOX'):
        p = d.GetPosition()
        d.SetPosition(stretch_pt(p, L))
        try:
            sz = d.GetTextSize()
            d.SetTextSize(pcbnew.VECTOR2I(int(round(sz.x * KX)), int(round(sz.y * KY))))
            d.SetTextThickness(int(round(d.GetTextThickness() * KAVG)))
        except Exception:
            pass
        n_txt += 1
        continue
    shape = d.GetShape() if hasattr(d, 'GetShape') else None
    try:
        if shape == pcbnew.SHAPE_T_ARC:
            d.SetArcGeometry(stretch_pt(d.GetStart(), L), stretch_pt(d.GetArcMid(), L),
                             stretch_pt(d.GetEnd(), L))
        elif shape == pcbnew.SHAPE_T_CIRCLE:
            c = d.GetPosition()
            d.SetPosition(stretch_pt(c, L))
            d.SetEnd(stretch_pt(d.GetEnd(), L))       # radius point
        elif shape == pcbnew.SHAPE_T_POLY:
            ps = d.GetPolyShape()
            for i in range(ps.OutlineCount()):
                o = ps.Outline(i)
                for j in range(o.PointCount()):
                    q = o.CPoint(j)
                    vx, vy = fld1(q.x * NM, q.y * NM, L)
                    o.SetPoint(j, to_v(q.x * NM + vx, q.y * NM + vy))
        elif shape == pcbnew.SHAPE_T_BEZIER:
            d.SetStart(stretch_pt(d.GetStart(), L)); d.SetEnd(stretch_pt(d.GetEnd(), L))
            d.SetBezierC1(stretch_pt(d.GetBezierC1(), L))
            d.SetBezierC2(stretch_pt(d.GetBezierC2(), L))
        else:                                          # segment, rect
            d.SetStart(stretch_pt(d.GetStart(), L)); d.SetEnd(stretch_pt(d.GetEnd(), L))
        if hasattr(d, 'SetWidth'):
            d.SetWidth(int(round(d.GetWidth() * KAVG)))
        n_art += 1
    except Exception:
        p = d.GetPosition()                            # last resort: translate
        vx, vy = fld1(p.x * NM, p.y * NM, L)
        d.Move(to_v(vx, vy) - to_v(0, 0)); n_art += 1

with quiet_stderr():
    pcbnew.ZONE_FILLER(bd).Fill(bd.Zones())
    pcbnew.SaveBoard(OUT, bd)
pro = SRC.replace('.kicad_pcb', '.kicad_pro'); opro = OUT.replace('.kicad_pcb', '.kicad_pro')
if os.path.exists(pro) and os.path.abspath(pro) != os.path.abspath(opro):
    shutil.copy(pro, opro)

# ---- admissibility census ---------------------------------------------------
# The map F(x) = x + u(x) closes a gap iff the smallest singular value of
# I + grad u drops below 1.  Reported per layer over the board, skipping rigid
# interiors (trivially 1) -- the cells that matter are free space and the ramp
# along each pad's rim, which is exactly where the F-46 pairs sit.  Note the
# global minimum is a corner effect: harmonic gradients blow up at the sharp
# corners of a polygonal pad, so read the percentile, not the min.
f0 = fields[CU[0]]
gx = f0.x0 + f0.h * np.arange(f0.nx)[None, :]
gy = f0.y0 + f0.h * np.arange(f0.ny)[:, None]
inb = ((gx >= BOUNDS[0]) & (gx <= BOUNDS[2]) & (gy >= BOUNDS[1]) & (gy <= BOUNDS[3]))
inb = np.broadcast_to(inb, (f0.ny, f0.nx)).copy()
reps = {}
for L in CU:
    fl = fields[L]
    m = inb & ~ndimage.binary_erosion(fl.owner >= 0, iterations=1)
    s = fl.sigma_min()
    reps[L] = dict(name=bd.GetLayerName(L), sigma_min=float(s[m].min()),
                   p01=float(np.percentile(s[m], 1)), p50=float(np.percentile(s[m], 50)),
                   contracting=int((s[m] < 1.0 - 1e-4).sum()), cells=int(m.sum()))

print(f'stretch KX={KX} KY={KY} centre=({CX:.2f},{CY:.2f})  [algorithm A: spatial field]')
print(f'  {n_fp} footprints moved; {len(nodes)} copper nodes, {n_pin} pad-pinned '
      f'(field/pin agreement {pin_err*1000:.2f} um)')
print(f'  rigid inclusions per layer field: {sum(len(v) for v in pad_polys[CU[0]].values())} pad/hole '
      f'shapes on {bd.GetLayerName(CU[0])}, {n_hole} drills on all {len(CU)}, '
      f'{n_via_incl} via barrels (of {len(vias)} vias, all coupled across the stack)')
print(f'  {n_t} tracks, {n_v} vias, {n_zv} zone vertices, {n_art} silk graphics, {n_txt} texts; '
      f'{n_split} tracks split (+{n_newseg} segments), max chord sag {sag_max*1000:.1f} um')
print('  admissibility (sigma_min of I+grad u, per layer):')
for L in CU:
    r_ = reps[L]
    print(f'    {r_["name"]:7s} min {r_["sigma_min"]:.3f}  p01 {r_["p01"]:.4f}  '
          f'p50 {r_["p50"]:.4f}  contracting {100.0*r_["contracting"]/max(r_["cells"],1):5.2f}%')
print(f'saved {OUT}  [{time.time()-t_start:.1f}s]')
if STATS:
    json.dump(dict(kx=KX, ky=KY, h=H, split_tol=SPLIT_TOL, n_fp=n_fp,
                   nodes=len(nodes), pinned=n_pin, pin_err_um=pin_err * 1000,
                   tracks=n_t, vias=n_v, zone_verts=n_zv, split=n_split,
                   new_segments=n_newseg, sag_max_um=sag_max * 1000,
                   layers={bd.GetLayerName(L): reps[L] for L in CU}),
              open(STATS, 'w'), indent=1)
