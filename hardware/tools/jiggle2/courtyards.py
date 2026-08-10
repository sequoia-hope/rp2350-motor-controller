#!/usr/bin/env python3
"""jiggle2: dump per-footprint courtyard polygons from the shipped board.

These are the same cached polys kicad DRC tests (BuildCourtyardCaches):
footprints with unbuildable courtyard outlines yield no poly and are skipped,
exactly like the real gate skips them. Writes data/courtyards.json:
ref -> {'F'|'B': [[[x,y],...], ...]} in mm, at shipped positions.
"""
import pcbnew
from common import NM, BOARD_PRE, load_board, save_json, in_region

board = load_board(BOARD_PRE)
out = {}
n_poly = 0
for f in board.GetFootprints():
    x, y = f.GetPosition().x * NM, f.GetPosition().y * NM
    if not in_region(x, y, 8.0):
        continue
    try:
        f.BuildCourtyardCaches()
    except AttributeError:
        pass
    entry = {}
    for lay, name in ((pcbnew.F_CrtYd, 'F'), (pcbnew.B_CrtYd, 'B')):
        ps = f.GetCourtyard(lay)
        polys = []
        for i in range(ps.OutlineCount()):
            o = ps.Outline(i)
            pts = [[o.CPoint(j).x * NM, o.CPoint(j).y * NM]
                   for j in range(o.PointCount())]
            if len(pts) >= 3:
                polys.append(pts)
        if polys:
            entry[name] = polys
            n_poly += len(polys)
    if entry:
        out[f.GetReference()] = entry
save_json('courtyards.json', out)
print(f'courtyards: {len(out)} footprints, {n_poly} polys -> courtyards.json')
