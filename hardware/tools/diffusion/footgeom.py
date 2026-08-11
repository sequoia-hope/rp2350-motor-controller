#!/usr/bin/env python3
"""diffusion experiment: extract real footprint geometry for the movers.

Dumps footprints.json — per mover: pads (shape/size) and body outline
(B.Fab shapes, falling back to B.Silkscreen for footprints without a fab
outline), all in mm relative to the shipped courtyard-bbox min corner so
render_page.py can draw the part at any placement. Runs under system
python (needs pcbnew), like extract.py.
"""
import json, os, sys

import pcbnew

os.environ.setdefault('JIGGLE2_VARIANT', 'morph')
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'jiggle2'))
from common import NM, BOARD_BASE, load_board

OUT = os.path.join(HERE, 'footprints.json')

with open(os.path.join(HERE, 'rp2350_netlist.json')) as f:
    NL = json.load(f)
movers = {o['name']: (o['ship_x0'], o['ship_y0'])
          for o in NL['objects'] if o['kind'] == 'mov'}

board = load_board(BOARD_BASE)
foots = {f.GetReference(): f for f in board.GetFootprints()}

SHAPE = {}
for nm, code in (('CIRCLE', 'c'), ('OVAL', 'o'), ('ROUNDRECT', 'rr'),
                 ('CHAMFERED_RECT', 'rr'), ('RECT', 'r'), ('RECTANGLE', 'r'),
                 ('TRAPEZOID', 'r'), ('CUSTOM', 'r')):
    v = getattr(pcbnew, f'PAD_SHAPE_{nm}', None)
    if v is not None:
        SHAPE[v] = code

L_FAB = board.GetLayerID('B.Fab')
L_SILK = board.GetLayerID('B.Silkscreen')
R = lambda v: round(v, 4)


def body_shapes(f):
    items = [g for g in f.GraphicalItems() if isinstance(g, pcbnew.PCB_SHAPE)]
    for lay in (L_FAB, L_SILK):
        got = [g for g in items if g.GetLayer() == lay]
        if got:
            return got
    return []


geom = {}
for ref, (sx, sy) in sorted(movers.items()):
    f = foots[ref]
    pads, lines, circles = [], [], []
    for p in f.Pads():
        pos = p.GetPosition()
        w, h = p.GetSize().x * NM, p.GetSize().y * NM
        rot = p.GetOrientation().AsDegrees() % 360
        if rot % 180 == 90:
            w, h = h, w
        elif rot % 90:
            print(f'warn: {ref} pad {p.GetPadName()} rot {rot}', file=sys.stderr)
        pads.append(dict(s=SHAPE.get(p.GetShape(), 'r'),
                         x=R(pos.x * NM - sx), y=R(pos.y * NM - sy),
                         w=R(w), h=R(h)))
    for g in body_shapes(f):
        kind = g.GetShapeStr()
        if kind in ('Line', 'Segment'):
            s, e = g.GetStart(), g.GetEnd()
            lines.append([R(s.x * NM - sx), R(s.y * NM - sy),
                          R(e.x * NM - sx), R(e.y * NM - sy)])
        elif kind == 'Circle':
            c = g.GetCenter()
            circles.append([R(c.x * NM - sx), R(c.y * NM - sy),
                            R(g.GetRadius() * NM)])
        elif kind == 'Arc':
            s, m, e = g.GetStart(), g.GetArcMid(), g.GetEnd()
            lines.append([R(s.x * NM - sx), R(s.y * NM - sy),
                          R(m.x * NM - sx), R(m.y * NM - sy)])
            lines.append([R(m.x * NM - sx), R(m.y * NM - sy),
                          R(e.x * NM - sx), R(e.y * NM - sy)])
        else:                      # Rect / Polygon / anything else: bbox ring
            bb = g.GetBoundingBox()
            x0, y0 = bb.GetLeft() * NM - sx, bb.GetTop() * NM - sy
            x1, y1 = bb.GetRight() * NM - sx, bb.GetBottom() * NM - sy
            for a, b, c, d in ((x0, y0, x1, y0), (x1, y0, x1, y1),
                               (x1, y1, x0, y1), (x0, y1, x0, y0)):
                lines.append([R(a), R(b), R(c), R(d)])
    geom[ref] = dict(value=f.GetValue(),
                     lib=str(f.GetFPID().GetLibItemName()),
                     pads=pads, lines=lines, circles=circles)

with open(OUT, 'w') as f:
    json.dump(geom, f)
n_pads = sum(len(g['pads']) for g in geom.values())
n_lines = sum(len(g['lines']) for g in geom.values())
print(f'{len(geom)} movers, {n_pads} pads, {n_lines} body lines -> {OUT}')
