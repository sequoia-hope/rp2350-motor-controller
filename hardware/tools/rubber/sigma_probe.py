#!/usr/bin/env python3
"""Admissibility probe: where, and how deeply, does a warp field contract?

The map F(x) = x + u(x) closes a gap between two nearby points iff the smallest
singular value of J = I + grad u drops below 1.  `stretch.py` prints that as one
number per layer, which is not enough to act on: the global minimum is always a
corner artifact (harmonic gradients blow up at the sharp corners of a polygonal
pad, and get worse as the grid is refined), so a bare sigma_min says "0.75" on a
field whose bulk is perfectly healthy.

This bins the contraction by distance from the nearest rigid territory, which
separates the two populations: the pad-rim ramp, where a shallow dip is expected
and harmless because the only things there are the pads themselves, and open
routing space, where a dip would actually eat a trace-to-trace clearance.  Read
the depth column -- a sigma of 0.998 across a 0.2 mm gap is 0.4 um.

Like fieldmap.py it reads the field stretch.py DUMPED, so all three tools are
describing the same field rather than three separate rebuilds of it.

    FIELD=dump.npz [LAYER=F.Cu,B.Cu] [GAP=0.2] [OUTJ=out.json] python3 sigma_probe.py
"""
import json, os, sys
S = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, S)
from field import sigma_min_2x2
import numpy as np
from scipy import ndimage

FIELD = os.environ['FIELD']
GAP = float(os.environ.get('GAP', '0.2'))          # reference clearance, mm
OUTJ = os.environ.get('OUTJ', '')
Z = np.load(FIELD, allow_pickle=False)
KX, KY = float(Z['kx']), float(Z['ky'])
CX, CY = float(Z['cx']), float(Z['cy'])
h, x0, y0 = float(Z['h']), float(Z['x0']), float(Z['y0'])
all_layers = [str(s) for s in Z['layers']]
want = os.environ.get('LAYER', '').split(',') if os.environ.get('LAYER') else all_layers

BINS = [0.0, 0.15, 0.3, 0.6, 1.0, 2.0]
out = {}
print(f'sigma probe  KX={KX} KY={KY}  h={h}  reference gap {GAP} mm  '
      f'(anchor {str(Z["anchor"])}, enclose {str(Z["enclose"])})')
for name in want:
    name = name.strip()
    if name not in all_layers:
        continue
    RX, RY = Z[f'RX_{name}'].astype(float), Z[f'RY_{name}'].astype(float)
    own = Z[f'OWN_{name}']
    ny, nx = RX.shape
    gx = x0 + h * np.arange(nx)[None, :]
    gy = y0 + h * np.arange(ny)[:, None]
    UX = RX + (KX - 1.0) * (gx - CX)
    UY = RY + (KY - 1.0) * (gy - CY)
    dUXdy, dUXdx = np.gradient(UX, h, h)
    dUYdy, dUYdx = np.gradient(UY, h, h)
    s = sigma_min_2x2(1.0 + dUXdx, dUXdy, dUYdx, 1.0 + dUYdy)
    m = np.ones_like(s, bool)
    m[0, :] = m[-1, :] = m[:, 0] = m[:, -1] = False
    m &= ~ndimage.binary_erosion(own, iterations=1)
    bad = m & (s < 1.0 - 1e-4)
    dist = ndimage.distance_transform_edt(~own) * h
    print(f'\n{name}: {bad.sum()} of {m.sum()} cells contracting '
          f'({100.0*bad.sum()/max(m.sum(),1):.1f}%), global min {s[m].min():.3f}')
    print('  distance from    share of      share of      worst    median     worst loss')
    print(f'  nearest rigid    contracting   all cells     sigma    sigma      on a {GAP:.2f} mm gap')
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
