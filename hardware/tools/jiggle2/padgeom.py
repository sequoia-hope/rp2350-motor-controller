#!/usr/bin/env python3
"""jiggle2: dump true pad geometry from the shipped board.

graft.py's obstacle pads are 4-pt bounding rects — up to ~0.4mm larger than
the real shape, and the bias is position-dependent (rect side vs round pad),
which silently breaks margin grandfathering when copper slides along a pad.
This dumps kicad's own effective polygons plus the per-pad local clearance
overrides (e.g. the mounting holes' 1.4mm) and the NPTH flag (no copper).
Writes data/padgeom.json keyed "ref|num|x|y" to survive duplicate NPTH nums.
"""
import pcbnew
from common import NM, BOARD_PRE, load_board, save_json, in_region

board = load_board(BOARD_PRE)
out = {}
for f in board.GetFootprints():
    fx, fy = f.GetPosition().x * NM, f.GetPosition().y * NM
    if not in_region(fx, fy, 8.0):
        continue
    try:
        flc = f.GetLocalClearance() or 0
    except TypeError:
        flc = 0
    for pad in f.Pads():
        px, py = pad.GetPosition().x * NM, pad.GetPosition().y * NM
        try:
            lc = pad.GetLocalClearance() or 0
        except TypeError:
            lc = 0
        try:
            ps = pad.GetEffectivePolygon()
        except TypeError:
            ps = pad.GetEffectivePolygon(pcbnew.ERROR_INSIDE)
        pts = []
        if ps.OutlineCount():
            o = ps.Outline(0)
            pts = [[o.CPoint(j).x * NM, o.CPoint(j).y * NM]
                   for j in range(o.PointCount())]
        key = f'{f.GetReference()}|{pad.GetNumber()}|{px:.3f}|{py:.3f}'
        out[key] = dict(
            pts=pts,
            lc=max(lc, flc) * NM if max(lc, flc) > 1 else 0.0,
            npth=pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH)
n_lc = sum(1 for v in out.values() if v['lc'])
n_np = sum(1 for v in out.values() if v['npth'])
save_json('padgeom.json', out)
print(f'padgeom: {len(out)} pads, {n_lc} with local clearance, '
      f'{n_np} NPTH -> padgeom.json')
