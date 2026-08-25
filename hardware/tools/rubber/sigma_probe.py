#!/usr/bin/env python3
"""Admissibility probe: where, and how deeply, does a warp field contract?

The map F(x) = x + u(x) closes a gap between two nearby points iff the smallest
singular value of J = I + grad u drops below 1.  `stretch.py` prints that as one
number per layer, which is not enough to act on: the global minimum is always a
corner artifact (harmonic gradients blow up at the sharp corners of a polygonal
pad, and get worse as the grid is refined), so a bare sigma_min says "0.63" on a
field whose bulk is perfectly healthy.

This bins the contraction by distance from the nearest rigid territory, which
separates the two populations: the pad-rim ramp, where a shallow dip is expected
and harmless because the only things there are the pads themselves, and open
routing space, where a dip would actually eat a trace-to-trace clearance.  Read
the depth column -- a sigma of 0.998 across a 0.2 mm gap is 0.4 um.

This solves the PAD-ONLY field, without the via inclusions stretch.py adds in
its second pass, so it isolates the contribution of the rigid parts.  Its
sigma_min therefore reads higher than stretch.py's on layers carrying many vias
(F.Cu here: 0.94 vs 0.63); the difference is the corners of the via octagons.

    KX=1.01 KY=1.01 LAYER=F.Cu python3 sigma_probe.py
"""
import collections, json, math, os, sys
S = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, S)
HW = os.path.dirname(os.path.dirname(S))
sys.path.insert(0, os.path.join(HW, 'tools', 'jiggle2'))
from common import load_board, NM
from field import WarpField, dilation
import numpy as np
from scipy import ndimage
import pcbnew

SRC = os.environ.get('SRC', os.path.join(HW, 'rp2350_driver.kicad_pcb'))
KX = float(os.environ.get('KX', '1.01')); KY = float(os.environ.get('KY', '1.01'))
H = float(os.environ.get('H', '0.12')); CLOSE = float(os.environ.get('CLOSE', '1.0'))
LAYERS = os.environ.get('LAYER', '')
GAP = float(os.environ.get('GAP', '0.2'))     # reference clearance, mm
OUTJ = os.environ.get('OUTJ', '')

bd = load_board(SRC)
bb = bd.GetBoardEdgesBoundingBox()
CX = (bb.GetLeft() + bb.GetRight()) / 2 * NM
CY = (bb.GetTop() + bb.GetBottom()) / 2 * NM
CU = list(bd.GetEnabledLayers().CuStack())
want = [L for L in CU if not LAYERS or bd.GetLayerName(L) in LAYERS.split(',')]
BOUNDS = (bb.GetLeft() * NM, bb.GetTop() * NM, bb.GetRight() * NM, bb.GetBottom() * NM)

V = {}; npads = {}; pads_by = {L: collections.defaultdict(list) for L in CU}
for f in bd.GetFootprints():
    r = f.GetReference(); p = f.GetPosition()
    V[r] = ((KX - 1) * (p.x * NM - CX), (KY - 1) * (p.y * NM - CY))
    npads[r] = max(1, len(list(f.Pads())))
    for pd in f.Pads():
        polys = []
        try:
            ps = pd.GetEffectivePolygon(pcbnew.PADSTACK.ALL_LAYERS)
            for i in range(ps.OutlineCount()):
                o = ps.Outline(i)
                polys.append([(o.CPoint(j).x * NM, o.CPoint(j).y * NM)
                              for j in range(o.PointCount())])
        except Exception:
            pb = pd.GetBoundingBox()
            polys.append([(pb.GetLeft() * NM, pb.GetTop() * NM), (pb.GetRight() * NM, pb.GetTop() * NM),
                          (pb.GetRight() * NM, pb.GetBottom() * NM), (pb.GetLeft() * NM, pb.GetBottom() * NM)])
        for L in CU:
            if pd.IsOnLayer(L): pads_by[L][r].extend(polys)

BINS = [0.0, 0.15, 0.3, 0.6, 1.0, 2.0]
out = {}
print(f'sigma probe  KX={KX} KY={KY}  h={H}  reference gap {GAP} mm')
for L in want:
    fl = WarpField(BOUNDS, h=H, margin=8.0, close=CLOSE, ambient=dilation(CX, CY, KX, KY))
    for r, pl in pads_by[L].items():
        fl.add_inclusion(pl, V[r], priority=npads[r], tag=r)
    fl.solve(verbose=False)
    s = fl.sigma_min()
    gx = fl.x0 + fl.h * np.arange(fl.nx)[None, :]
    gy = fl.y0 + fl.h * np.arange(fl.ny)[:, None]
    inb = np.broadcast_to((gx >= BOUNDS[0]) & (gx <= BOUNDS[2]) &
                          (gy >= BOUNDS[1]) & (gy <= BOUNDS[3]), (fl.ny, fl.nx)).copy()
    own = fl.owner >= 0
    m = inb & ~ndimage.binary_erosion(own, iterations=1)
    bad = m & (s < 1.0 - 1e-4)
    dist = ndimage.distance_transform_edt(~own) * fl.h
    name = bd.GetLayerName(L)
    print(f'\n{name}: {bad.sum()} of {m.sum()} cells contracting '
          f'({100.0*bad.sum()/max(m.sum(),1):.1f}%), global min {s[m].min():.3f}')
    print('  distance from    share of      share of      worst    median     worst loss')
    print('  nearest pad      contracting   all cells     sigma    sigma      on a %.2f mm gap' % GAP)
    rows = []
    for thr in BINS:
        sel = bad & (dist > thr)
        allsel = m & (dist > thr)
        w = float(s[sel].min()) if sel.any() else 1.0
        md = float(np.median(s[sel])) if sel.any() else 1.0
        rows.append(dict(thr=thr, share_bad=100.0 * sel.sum() / max(bad.sum(), 1),
                         share_all=100.0 * allsel.sum() / max(m.sum(), 1),
                         worst=w, median=md, loss_um=(1.0 - w) * GAP * 1000))
        print(f'  > {thr:4.2f} mm       {rows[-1]["share_bad"]:6.1f}%      '
              f'{rows[-1]["share_all"]:6.1f}%      {w:.4f}   {md:.4f}     '
              f'{rows[-1]["loss_um"]:6.1f} um')
    out[name] = dict(contracting=int(bad.sum()), cells=int(m.sum()),
                     sigma_min=float(s[m].min()), bins=rows)
if OUTJ:
    json.dump(out, open(OUTJ, 'w'), indent=1)
    print('\nwrote', OUTJ)
