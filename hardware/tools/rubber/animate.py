#!/usr/bin/env python3
"""Animate the top copper under both stretch algorithms, 0 -> +KMAX -> 0.

Left panel: the graph-harmonic engine (residual interpolated along each net's
own copper).  Right panel: algorithm A (residual interpolated through space).
Copper is coloured by its LAG -- how far it trails the pure ambient dilation --
so the failure mode is visible directly: in the left panel neighbouring traces
on different nets take unrelated colours and visibly slide past each other,
which is the F-46 shear; in the right panel colour varies smoothly with
position, so neighbours move together and gaps survive.

Both engines are exactly linear in (k-1): every pinned residual is
v_i - ambient(x) = (k-1)(origin_i - x), and both solves are linear.  So each is
solved once at (k-1) = 1 and every frame is a scalar multiple -- the animation
is exact, not interpolated, and costs two solves in total.  (The courtyard
relax, which is not linear, is skipped here for that reason; it contributes no
correction at all below +3% on this board.)

    OUT=... KMAX=0.10 FRAMES=240 FPS=30 H=0.12 python3 animate.py
"""
import collections, math, os, sys, time
S = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, S)
HW = os.path.dirname(os.path.dirname(S))
sys.path.insert(0, os.path.join(HW, 'tools', 'jiggle2'))
from common import load_board, NM
from field import WarpField, dilation
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.animation import FFMpegWriter
import pcbnew

SRC = os.environ.get('SRC', os.path.join(HW, 'rp2350_driver.kicad_pcb'))
OUT = os.environ.get('OUT', os.path.join(HW, '..', 'review', 'img', 'rubber_stretch.mp4'))
KMAX = float(os.environ.get('KMAX', '0.10'))
FRAMES = int(os.environ.get('FRAMES', '240'))
FPS = int(os.environ.get('FPS', '30'))
H = float(os.environ.get('H', '0.12'))
LAYER_NAME = os.environ.get('LAYER', 'F.Cu')

t_start = time.time()
bd = load_board(SRC)
bb = bd.GetBoardEdgesBoundingBox()
CX = (bb.GetLeft() + bb.GetRight()) / 2 * NM
CY = (bb.GetTop() + bb.GetBottom()) / 2 * NM
CU = list(bd.GetEnabledLayers().CuStack())
VIEW = next(L for L in CU if bd.GetLayerName(L) == LAYER_NAME)
BOUNDS = (bb.GetLeft() * NM, bb.GetTop() * NM, bb.GetRight() * NM, bb.GetBottom() * NM)

def amb1(x, y):                      # ambient at (k-1) = 1
    return (x - CX, y - CY)

# ---- unit footprint vectors -------------------------------------------------
V1 = {}; npads = {}
for f in bd.GetFootprints():
    r = f.GetReference(); p = f.GetPosition()
    V1[r] = amb1(p.x * NM, p.y * NM)
    npads[r] = max(1, len(list(f.Pads())))

# ---- copper graph (shared by both engines) ----------------------------------
tracks = [t for t in bd.GetTracks() if t.GetClass() == 'PCB_TRACK']
vias = [t for t in bd.GetTracks() if t.GetClass() == 'PCB_VIA']
nodes = []; node_of = {}; adj = collections.defaultdict(list)
via_nodes = []; tnodes = []; via_rad = {}

def new_node(x, y, net):
    nodes.append(dict(x=x, y=y, pin=None, net=net, lays=set())); return len(nodes) - 1

pads = []
for f in bd.GetFootprints():
    for p in f.Pads():
        pb = p.GetBoundingBox()
        pads.append(((pb.GetLeft() * NM, pb.GetTop() * NM,
                      pb.GetRight() * NM, pb.GetBottom() * NM), p, V1[f.GetReference()]))

def pad_hit(x, y, layer, nc):
    for pb, p, v in pads:
        if not (pb[0] - 1e-4 <= x <= pb[2] + 1e-4 and pb[1] - 1e-4 <= y <= pb[3] + 1e-4): continue
        if layer is not None and not p.IsOnLayer(layer): continue
        if p.GetNetCode() != nc: continue
        if p.HitTest(pcbnew.VECTOR2I(int(round(x / NM)), int(round(y / NM)))): return v
    return None

via_index = collections.defaultdict(list)
for vo in vias:
    p = vo.GetPosition(); x, y = p.x * NM, p.y * NM; nc = vo.GetNetCode()
    nid = new_node(x, y, nc); via_nodes.append((vo, nid))
    try: r = vo.GetWidth(pcbnew.PADSTACK.ALL_LAYERS) * NM / 2
    except Exception: r = 0.3
    via_rad[nid] = r
    via_index[(int(x // 1.0), int(y // 1.0))].append((x, y, r, nc, nid))
    i0, i1 = CU.index(vo.TopLayer()), CU.index(vo.BottomLayer())
    nodes[nid]['lays'].update(CU[min(i0, i1):max(i0, i1) + 1])
    fv = pad_hit(x, y, None, nc)
    if fv is not None: nodes[nid]['pin'] = fv

def find_via(x, y, nc):
    for cx in (int(x // 1.0) - 1, int(x // 1.0), int(x // 1.0) + 1):
        for cy in (int(y // 1.0) - 1, int(y // 1.0), int(y // 1.0) + 1):
            for vx, vy, r, vnc, nid in via_index.get((cx, cy), ()):
                if vnc == nc and math.hypot(vx - x, vy - y) <= r + 1e-6: return nid
    return None

conn = bd.GetConnectivity()
pad_vec = {}
for f in bd.GetFootprints():
    for p in f.Pads(): pad_vec[p.m_Uuid.AsString()] = V1[f.GetReference()]

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
                if fv is not None: nodes[nid]['pin'] = fv
        nodes[nid]['lays'].add(lay)
        ends.append(nid)
    na, nb = ends
    L = math.hypot((e.x - s.x) * NM, (e.y - s.y) * NM)
    w = 1.0 / max(L, 0.02)
    adj[na].append((nb, w)); adj[nb].append((na, w))
    tnodes.append((t, na, nb))
print(f'graph: {len(nodes)} nodes, {len(tracks)} tracks, {len(vias)} vias')

# ---- engine 1: graph-harmonic (the old one, faithful incl. overlap pinning) --
old_pin = [n['pin'] for n in nodes]
for (t, na, nb) in tnodes:
    s, e = t.GetStart(), t.GetEnd()
    try: cpads = list(conn.GetConnectedPads(t))
    except Exception: cpads = []
    for cp in cpads:
        pv = pad_vec.get(cp.m_Uuid.AsString())
        if pv is None: continue
        pc = cp.GetPosition(); pcx, pcy = pc.x * NM, pc.y * NM
        da = math.hypot(s.x * NM - pcx, s.y * NM - pcy)
        db = math.hypot(e.x * NM - pcx, e.y * NM - pcy)
        nid = na if da <= db else nb
        if old_pin[nid] is None: old_pin[nid] = pv

def residual(vec, x, y):
    a = amb1(x, y); return (vec[0] - a[0], vec[1] - a[1])

seen = [False] * len(nodes); comps = []
for i in range(len(nodes)):
    if seen[i]: continue
    st = [i]; seen[i] = True; comp = []
    while st:
        n = st.pop(); comp.append(n)
        for m, w in adj[n]:
            if not seen[m]: seen[m] = True; st.append(m)
    comps.append(comp)

Hh = [None] * len(nodes)
for comp in comps:
    pinned = [n for n in comp if old_pin[n] is not None]
    if not pinned:
        for n in comp: Hh[n] = (0.0, 0.0)
        continue
    vals = {residual(old_pin[n], nodes[n]['x'], nodes[n]['y']) for n in pinned}
    if len(vals) == 1:
        v = next(iter(vals))
        for n in comp: Hh[n] = v
        continue
    cur = {n: (residual(old_pin[n], nodes[n]['x'], nodes[n]['y'])
               if old_pin[n] is not None else None) for n in comp}
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
            dmax = max(dmax, abs(nv[0] - cur[n][0]), abs(nv[1] - cur[n][1])); cur[n] = nv
        if dmax < 1e-7: break
    for n in comp: Hh[n] = cur[n]
DOLD = np.array([[amb1(nodes[n]['x'], nodes[n]['y'])[0] + Hh[n][0],
                  amb1(nodes[n]['x'], nodes[n]['y'])[1] + Hh[n][1]] for n in range(len(nodes))])
print(f'old engine solved ({len(comps)} clusters)')

# ---- engine 2: spatial fields, one per layer, via-coupled -------------------
pad_polys = {L: collections.defaultdict(list) for L in CU}
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
            polys.append([(pb.GetLeft() * NM, pb.GetTop() * NM), (pb.GetRight() * NM, pb.GetTop() * NM),
                          (pb.GetRight() * NM, pb.GetBottom() * NM), (pb.GetLeft() * NM, pb.GetBottom() * NM)])
        for L in CU:
            if p.IsOnLayer(L): pad_polys[L][r].extend(polys)

def build_fields(extra=()):
    out = {}
    for L in CU:
        fl = WarpField(BOUNDS, h=H, margin=8.0, close=1.0, ambient=dilation(CX, CY, 2.0, 2.0))
        for r, pl in pad_polys[L].items():
            fl.add_inclusion(pl, V1[r], priority=npads.get(r, 1), tag=r)
        for polys, vec in extra:
            fl.add_inclusion(polys, vec, priority=0.0, tag='via')
        fl.solve(verbose=False)
        out[L] = fl
    return out

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
    d = np.column_stack([UX / W, UY / W])
    for i, n in enumerate(nodes):
        if n['pin'] is not None: d[i] = n['pin']
    return d

fields = build_fields()
DNEW = node_disps(fields)
oct8 = [(math.cos(i * math.pi / 4), math.sin(i * math.pi / 4)) for i in range(8)]
extra = [([[(nodes[nid]['x'] + via_rad.get(nid, .3) * c,
             nodes[nid]['y'] + via_rad.get(nid, .3) * s) for c, s in oct8]], DNEW[nid])
         for vo, nid in via_nodes]
fields = build_fields(extra)
DNEW = node_disps(fields)
print(f'new engine solved ({len(CU)} layer fields) [{time.time()-t_start:.0f}s]')

# ---- what to draw: the viewed layer only ------------------------------------
seg_idx = [(na, nb) for (t, na, nb) in tnodes if t.GetLayer() == VIEW]
seg_w = np.array([t.GetWidth() * NM for (t, na, nb) in tnodes if t.GetLayer() == VIEW])
seg_a = np.array([a for a, b in seg_idx]); seg_b = np.array([b for a, b in seg_idx])
via_id = np.array([nid for vo, nid in via_nodes])
via_r = np.array([via_rad.get(nid, 0.3) for vo, nid in via_nodes])

pad_polys_view = []; pad_ref = []
for f in bd.GetFootprints():
    r = f.GetReference()
    for p in f.Pads():
        if not p.IsOnLayer(VIEW): continue
        try:
            ps = p.GetEffectivePolygon(pcbnew.PADSTACK.ALL_LAYERS)
            o = ps.Outline(0)
            pad_polys_view.append(np.array([[o.CPoint(j).x * NM, o.CPoint(j).y * NM]
                                            for j in range(o.PointCount())]))
            pad_ref.append(r)
        except Exception:
            pass
PADV = np.array([V1[r] for r in pad_ref]) if pad_ref else np.zeros((0, 2))

edges = []
for d in bd.GetDrawings():
    if d.GetLayer() != pcbnew.Edge_Cuts: continue
    if d.GetShape() == pcbnew.SHAPE_T_ARC:
        c = d.GetCenter(); rr = d.GetRadius() * NM
        a0 = math.atan2(d.GetStart().y - c.y, d.GetStart().x - c.x)
        a1 = math.atan2(d.GetEnd().y - c.y, d.GetEnd().x - c.x)
        if a1 < a0: a1 += 2 * math.pi
        ts = np.linspace(a0, a1, 16)
        edges.append(np.column_stack([c.x * NM + rr * np.cos(ts), c.y * NM + rr * np.sin(ts)]))
    else:
        s, e = d.GetStart(), d.GetEnd()
        edges.append(np.array([[s.x * NM, s.y * NM], [e.x * NM, e.y * NM]]))

# lag = displacement minus pure ambient, per node, at (k-1) = 1
AMB = np.column_stack([NX - CX, NY - CY])
LAG = {'old': np.hypot(*(DOLD - AMB).T), 'new': np.hypot(*(DNEW - AMB).T)}
# 97th percentile, not the max: a handful of connector far-pads is 10x the rest
# and would crush every other trace into the black end of the ramp.  GAMMA then
# spends most of the colour range on the small lags, which is where the shear
# between neighbouring traces actually lives.
VMAX = KMAX * 1000 * max(np.percentile(LAG['old'], 97), np.percentile(LAG['new'], 97))
GAMMA = 0.55
print(f'lag colour scale 0..{VMAX:.0f} um at +{KMAX*100:.0f}%')

# ---- figure -----------------------------------------------------------------
W_MM = (bb.GetRight() - bb.GetLeft()) * NM; H_MM = (bb.GetBottom() - bb.GetTop()) * NM
pad_mm = 2.0
X0 = CX - (W_MM / 2 + pad_mm) * (1 + KMAX); X1 = CX + (W_MM / 2 + pad_mm) * (1 + KMAX)
Y0 = CY - (H_MM / 2 + pad_mm) * (1 + KMAX); Y1 = CY + (H_MM / 2 + pad_mm) * (1 + KMAX)
ZOOM = os.environ.get('ZOOM', '')       # "cx,cy,width_mm" to inspect one region
if ZOOM:
    zx, zy, zw = (float(v) for v in ZOOM.split(','))
    zh = zw * (Y1 - Y0) / (X1 - X0)
    X0, X1 = zx - zw / 2, zx + zw / 2
    Y0, Y1 = zy - zh / 2, zy + zh / 2
DPI = 100
BAR = 0.85                                   # header strip, inches
PANEL_W = 12.0; PANEL_H = PANEL_W * (Y1 - Y0) / (X1 - X0)
TOT_H = round((PANEL_H + BAR) * DPI / 2) * 2 / DPI   # h264 needs even pixel dims
PANEL_H = TOT_H - BAR
fig = plt.figure(figsize=(PANEL_W * 2, TOT_H), dpi=DPI, facecolor='#0d1117')
cmap = matplotlib.colormaps['inferno']
axes = {}
for i, key in enumerate(('old', 'new')):
    ax = fig.add_axes([i * 0.5, 0.0, 0.5, PANEL_H / TOT_H])
    ax.set_xlim(X0, X1); ax.set_ylim(Y1, Y0); ax.set_aspect('equal')
    ax.axis('off'); ax.set_facecolor('#0d1117')
    axes[key] = ax
pts_per_mm = (PANEL_W * DPI / (X1 - X0)) * 72.0 / DPI

art = {}
for key, ax in axes.items():
    ec = LineCollection([e for e in edges], colors='#3d4756', linewidths=1.4, zorder=1)
    ax.add_collection(ec)
    pc = PolyCollection(pad_polys_view, facecolors='#59626f', edgecolors='none', zorder=2)
    ax.add_collection(pc)
    lc = LineCollection([], linewidths=seg_w * pts_per_mm, capstyle='round', zorder=3)
    ax.add_collection(lc)
    vc = ax.scatter(NX[via_id], NY[via_id], s=(via_r * 2 * pts_per_mm) ** 2,
                    marker='o', linewidths=0, zorder=4)
    art[key] = dict(ec=ec, pc=pc, lc=lc, vc=vc)

fig.text(0.25, 1 - 0.30 / TOT_H, 'graph-harmonic  —  residual follows each net\'s copper',
         ha='center', va='top', color='#e6edf3', fontsize=15, family='DejaVu Sans')
fig.text(0.75, 1 - 0.30 / TOT_H, 'algorithm A  —  residual follows space',
         ha='center', va='top', color='#7ee787', fontsize=15, family='DejaVu Sans')
sub = fig.text(0.5, 1 - 0.66 / TOT_H, '', ha='center', va='top',
               color='#8b949e', fontsize=12, family='DejaVu Sans Mono')

D = {'old': DOLD, 'new': DNEW}

def draw(eps):
    for key, ax in axes.items():
        d = D[key]
        px = NX + eps * d[:, 0]; py = NY + eps * d[:, 1]
        segs = np.stack([np.column_stack([px[seg_a], py[seg_a]]),
                         np.column_stack([px[seg_b], py[seg_b]])], axis=1)
        art[key]['lc'].set_segments(segs)
        lag = eps * 1000 * LAG[key]
        def ramp(v):
            return cmap(0.08 + 0.92 * np.clip(v / max(VMAX, 1e-9), 0, 1) ** GAMMA)
        art[key]['lc'].set_color(ramp((lag[seg_a] + lag[seg_b]) / 2))
        art[key]['vc'].set_offsets(np.column_stack([px[via_id], py[via_id]]))
        art[key]['vc'].set_color(ramp(lag[via_id]))
        art[key]['pc'].set_verts([pp + eps * PADV[i] for i, pp in enumerate(pad_polys_view)])
        art[key]['ec'].set_segments([np.column_stack([CX + (e[:, 0] - CX) * (1 + eps),
                                                      CY + (e[:, 1] - CY) * (1 + eps)]) for e in edges])
    sub.set_text(f'{LAYER_NAME}   stretch +{eps*100:5.2f}%   '
                 f'board {W_MM*(1+eps):.1f} x {H_MM*(1+eps):.1f} mm   '
                 f'colour = lag behind the rubber, 0 - {VMAX:.0f} um')

os.makedirs(os.path.dirname(os.path.abspath(OUT)), exist_ok=True)
writer = FFMpegWriter(fps=FPS, codec='libx264', bitrate=-1,
                      extra_args=['-pix_fmt', 'yuv420p', '-preset', 'slow', '-crf', '20'])
# a raised cosine out and back, so the loop has no visible turn or seam
eps_seq = [KMAX * 0.5 * (1 - math.cos(2 * math.pi * i / FRAMES)) for i in range(FRAMES)]
with writer.saving(fig, os.path.abspath(OUT), DPI):
    for i, eps in enumerate(eps_seq):
        draw(eps)
        writer.grab_frame(facecolor='#0d1117')
        if i % 30 == 0: print(f'  frame {i}/{FRAMES}  +{eps*100:.2f}%')
print(f'wrote {os.path.abspath(OUT)}  [{time.time()-t_start:.0f}s]')
