#!/usr/bin/env python3
"""Static, board-referenced map of the warp field that stretch.py applied.

Renders, for one copper layer, over the real copper of that layer:

  left   the LAG field -- how far each point trails the pure ambient dilation --
         as a heat map plus arrows, with the rigid territories outlined.  A part
         reads as a plateau; the ramp around it is where the strain goes.  Any
         place the plateau does not cover what it should (a mounting hole
         stranded from its connector, an unpinned via) shows up here as a hole
         in the plateau or a knot in the arrows.
  right  sigma_min of I + grad u.  Below 1 the map contracts and a gap in the
         worst direction shrinks; the scale is centred on 1 so contraction and
         expansion read as opposite colours.

It draws the field DUMPED BY stretch.py (FIELD_DUMP=...), not a rebuilt one, so
the picture is guaranteed to be the field the board actually got.

    FIELD=dump.npz LAYER=In3.Cu OUT=map.png [ZOOM=cx,cy,w] python3 fieldmap.py
"""
import math, os, sys
S = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, S)
HW = os.path.dirname(os.path.dirname(S))
sys.path.insert(0, os.path.join(HW, 'tools', 'jiggle2'))
from common import load_board, NM
from field import sigma_min_2x2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection, PolyCollection
import pcbnew

SRC = os.environ.get('SRC', os.path.join(HW, 'rp2350_driver.kicad_pcb'))
FIELD = os.environ['FIELD']
LAYER = os.environ.get('LAYER', 'F.Cu')
OUT = os.environ.get('OUT', os.path.join(HW, '..', 'review', 'img', 'fieldmap.png'))
ZOOM = os.environ.get('ZOOM', '')
LABELS = os.environ.get('LABELS', '1') == '1'

Z = np.load(FIELD, allow_pickle=False)
KX, KY = float(Z['kx']), float(Z['ky'])
CXf, CYf = float(Z['cx']), float(Z['cy'])
h, x0, y0 = float(Z['h']), float(Z['x0']), float(Z['y0'])
RX, RY = Z[f'RX_{LAYER}'].astype(float), Z[f'RY_{LAYER}'].astype(float)
OWN = Z[f'OWN_{LAYER}']
ny, nx = RX.shape
gx = x0 + h * np.arange(nx)[None, :]
gy = y0 + h * np.arange(ny)[:, None]
AX = (KX - 1.0) * (gx - CXf) * np.ones((ny, nx))
AY = (KY - 1.0) * (gy - CYf) * np.ones((ny, nx))
UX, UY = RX + AX, RY + AY                       # total displacement
LAGX, LAGY = RX, RY                             # residual == lag behind ambient
LAG = np.hypot(LAGX, LAGY) * 1000.0             # um

dUXdy, dUXdx = np.gradient(UX, h, h)
dUYdy, dUYdx = np.gradient(UY, h, h)
SIG = sigma_min_2x2(1.0 + dUXdx, dUXdy, dUYdx, 1.0 + dUYdy)

bd = load_board(SRC)
bb = bd.GetBoardEdgesBoundingBox()
CU = list(bd.GetEnabledLayers().CuStack())
VIEW = next(L for L in CU if bd.GetLayerName(L) == LAYER)
BX0, BY0 = bb.GetLeft() * NM, bb.GetTop() * NM
BX1, BY1 = bb.GetRight() * NM, bb.GetBottom() * NM

X0, X1, Y0, Y1 = BX0 - 1, BX1 + 1, BY0 - 1, BY1 + 1
if ZOOM:
    zx, zy, zw = (float(v) for v in ZOOM.split(','))
    zh = zw * (Y1 - Y0) / (X1 - X0)
    X0, X1, Y0, Y1 = zx - zw / 2, zx + zw / 2, zy - zh / 2, zy + zh / 2

# ---- board features on this layer -------------------------------------------
segs, widths = [], []
for t in bd.GetTracks():
    if t.GetClass() != 'PCB_TRACK' or t.GetLayer() != VIEW:
        continue
    s, e = t.GetStart(), t.GetEnd()
    segs.append([(s.x * NM, s.y * NM), (e.x * NM, e.y * NM)])
    widths.append(t.GetWidth() * NM)
vias = [(t.GetPosition().x * NM, t.GetPosition().y * NM,
         t.GetWidth(pcbnew.PADSTACK.ALL_LAYERS) * NM / 2)
        for t in bd.GetTracks() if t.GetClass() == 'PCB_VIA']
padp, holes, labels = [], [], []
for f in bd.GetFootprints():
    r = f.GetReference(); pos = f.GetPosition()
    q = [(p.GetPosition().x * NM, p.GetPosition().y * NM) for p in f.Pads()]
    if q and X0 <= sum(a[0] for a in q) / len(q) <= X1 and Y0 <= sum(a[1] for a in q) / len(q) <= Y1:
        labels.append((sum(a[0] for a in q) / len(q), sum(a[1] for a in q) / len(q), r, len(q)))
    for p in f.Pads():
        if p.IsOnLayer(VIEW):
            try:
                ps = p.GetEffectivePolygon(pcbnew.PADSTACK.ALL_LAYERS)
                o = ps.Outline(0)
                padp.append(np.array([[o.CPoint(j).x * NM, o.CPoint(j).y * NM]
                                      for j in range(o.PointCount())]))
            except Exception:
                pass
        dx, dy = p.GetDrillSizeX() * NM, p.GetDrillSizeY() * NM
        if dx > 0:
            c = p.GetPosition()
            holes.append((c.x * NM, c.y * NM, dx / 2, dy / 2,
                          p.GetAttribute() == 3))          # NPTH?
edges = []
for d in bd.GetDrawings():
    if d.GetLayer() != pcbnew.Edge_Cuts:
        continue
    if d.GetShape() == pcbnew.SHAPE_T_ARC:
        c = d.GetCenter(); rr = d.GetRadius() * NM
        a0 = math.atan2(d.GetStart().y - c.y, d.GetStart().x - c.x)
        a1 = math.atan2(d.GetEnd().y - c.y, d.GetEnd().x - c.x)
        if a1 < a0: a1 += 2 * math.pi
        ts = np.linspace(a0, a1, 24)
        edges.append(np.column_stack([c.x * NM + rr * np.cos(ts), c.y * NM + rr * np.sin(ts)]))
    else:
        s, e = d.GetStart(), d.GetEnd()
        edges.append(np.array([[s.x * NM, s.y * NM], [e.x * NM, e.y * NM]]))

# ---- figure ------------------------------------------------------------------
DPI = 110
PW = 13.0
PH = PW * (Y1 - Y0) / (X1 - X0)
fig, axes = plt.subplots(1, 2, figsize=(PW * 2, PH + 1.15), dpi=DPI, facecolor='white')
fig.subplots_adjust(left=0.035, right=0.995, bottom=0.055, top=0.90, wspace=0.10)
ext = [x0 - h / 2, x0 + h * (nx - 0.5), y0 + h * (ny - 0.5), y0 - h / 2]
pts_per_mm = (PW * DPI / (X1 - X0)) * 72.0 / DPI

def decorate(ax, title):
    ax.add_collection(LineCollection(edges, colors='#111', linewidths=1.3, zorder=6))
    ax.add_collection(LineCollection(segs, linewidths=np.array(widths) * pts_per_mm,
                                     colors='#00000022', capstyle='round', zorder=4))
    if padp:
        ax.add_collection(PolyCollection(padp, facecolors='#00000033',
                                         edgecolors='#00000055', linewidths=0.4, zorder=5))
    for vx, vy, vr in vias:
        ax.add_patch(plt.Circle((vx, vy), vr, fc='none', ec='#00000055', lw=0.4, zorder=5))
    for hx, hy, rx, ry, npth in holes:
        ax.add_patch(plt.Circle((hx, hy), max(rx, ry), fc='none',
                                ec='#d62728' if npth else '#1f77b4', lw=1.3, zorder=7))
    # rigid territories: the plateau boundary
    ax.contour(gx.ravel(), gy.ravel(), OWN.astype(float), levels=[0.5],
               colors='#2ca02c', linewidths=0.9, zorder=8)
    if LABELS:
        for lx, ly, r, n in labels:
            if n >= 3 or r.startswith(('J', 'H', 'U')):
                ax.text(lx, ly, r, fontsize=6.5, ha='center', va='center', zorder=9,
                        color='#000', bbox=dict(fc='#ffffffcc', ec='none', pad=0.6))
    ax.set_xlim(X0, X1); ax.set_ylim(Y1, Y0); ax.set_aspect('equal')
    ax.set_title(title, fontsize=13, pad=8)
    ax.set_xlabel('x (mm)', fontsize=9); ax.set_ylabel('y (mm)', fontsize=9)
    ax.grid(True, color='#0000001a', lw=0.5, ls=':')
    ax.tick_params(labelsize=8)

# left: lag magnitude + arrows
im = axes[0].imshow(LAG, extent=ext, origin='upper', cmap='magma',
                    vmin=0, vmax=float(np.percentile(LAG, 99.5)), zorder=1)
st = max(1, int(round(0.9 / h)))
axes[0].quiver(gx.ravel()[::st], gy.ravel()[::st], LAGX[::st, ::st], LAGY[::st, ::st],
               color='#7fffd4', angles='xy', scale_units='xy',
               scale=max(np.hypot(LAGX, LAGY).max(), 1e-9) / (2.5 * h * st),
               width=0.0016, zorder=3)
decorate(axes[0], f'lag behind the ambient dilation — {LAYER}')
cb = fig.colorbar(im, ax=axes[0], fraction=0.030, pad=0.012)
cb.set_label('µm', fontsize=9); cb.ax.tick_params(labelsize=8)

# right: sigma_min, diverging about 1
d = float(max(1.0 - np.percentile(SIG, 0.5), np.percentile(SIG, 99.5) - 1.0, 1e-4))
im2 = axes[1].imshow(SIG, extent=ext, origin='upper', cmap='RdBu',
                     vmin=1 - d, vmax=1 + d, zorder=1)
decorate(axes[1], f'σ$_{{min}}$ of I + ∇u  —  red contracts, blue expands')
cb2 = fig.colorbar(im2, ax=axes[1], fraction=0.030, pad=0.012)
cb2.set_label('σ$_{min}$ (1.0 = gaps preserved)', fontsize=9); cb2.ax.tick_params(labelsize=8)

fig.suptitle(f'Warp field, {LAYER}, stretch +{(KX-1)*100:g}%   ·   '
             f'green = rigid territory   ·   blue circles = plated holes, red = NPTH   ·   '
             f'anchor: {str(Z["anchor"])}, enclose: {str(Z["enclose"])}',
             fontsize=12, y=0.975)
os.makedirs(os.path.dirname(os.path.abspath(OUT)), exist_ok=True)
fig.savefig(os.path.abspath(OUT), dpi=DPI, facecolor='white')
print(f'wrote {os.path.abspath(OUT)}  ({LAYER}, lag max {LAG.max():.0f} um, '
      f'sigma_min {SIG.min():.3f})')
