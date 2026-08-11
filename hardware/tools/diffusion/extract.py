#!/usr/bin/env python3
"""diffusion experiment: extract the morph region as a placement netlist.

Counterpoint to jiggle2's morph stepper (arXiv 2407.12282, "Chip Placement
with Diffusion Models"): instead of morphing rev-A copper toward rev-B
targets, ask a generative placer for a FRESH placement of the same 49 parts
in the same region and compare it against the hand-designed rev-B targets.

Objects:
  movable  the 49 MOVED parts (all back-side); size = courtyard bbox,
           pins = pad centers
  fixed    non-moved footprints whose back courtyard intersects the canvas,
           clipped to the canvas so the boundary term of the legality
           potential stays satisfiable. Front-side parts with drilled pads
           (THT connectors, power diodes, mounting holes) block only at the
           pads: the rev-B design itself places movers under the J2/J12
           bodies, so the body outline must not repel
  port     one virtual terminal per net for all pads outside the graph,
           at their centroid clamped to the canvas edge (partitioning-style)

Nets: every net touching a moved part except GND (plane-reconnected, same
exemption creep.py uses). Star topology from the port terminal if the net
leaves the graph, else from its first terminal — the paper's driving-pin
convention.

Runs under system python (needs pcbnew). Writes rp2350_netlist.json here;
build_dataset.py (venv python, needs torch) turns it into chipdiffusion's
PyG pickles.
"""
import json, math, os, sys

import pcbnew

os.environ.setdefault('JIGGLE2_VARIANT', 'morph')
J2 = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'jiggle2')
sys.path.insert(0, J2)
from common import NM, BOARD_BASE, load_board, load_json

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   'rp2350_netlist.json')
MARGIN = 0.25          # canvas breathing room beyond the shipped∪target hull
EDGE_CLR = 0.3         # board edge clearance (matches obstacles.json)
PORT_SIZE = 0.2        # virtual terminal square, mm

O = load_json('obstacles.json')
CY = load_json('courtyards.json')
MOVED = O['moved']
ex0, ey0, ex1, ey1 = O['board_edge']

MSIDE = 'B'            # the movers all live on the back copper
board = load_board(BOARD_BASE)
foots = {f.GetReference(): f for f in board.GetFootprints()}
missing = [r for r in MOVED if r not in CY or MSIDE not in CY[r]]
assert not missing, f'moved parts without {MSIDE} courtyard: {missing}'
front = [r for r in MOVED if not foots[r].IsFlipped()]
assert not front, f'moved parts not on back side: {front}'


def cy_bbox(r, side=MSIDE):
    pts = [p for poly in CY[r].get(side, []) for p in poly]
    if not pts:
        return None
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)

# ---- canvas: shipped ∪ target courtyard hull of the movers -------------------
xs, ys = [], []
for r, (dx, dy) in MOVED.items():
    x0, y0, x1, y1 = cy_bbox(r)
    xs += [x0, x1, x0 + dx, x1 + dx]
    ys += [y0, y1, y0 + dy, y1 + dy]
canvas = [max(min(xs) - MARGIN, ex0 + EDGE_CLR),
          max(min(ys) - MARGIN, ey0 + EDGE_CLR),
          min(max(xs) + MARGIN, ex1 - EDGE_CLR),
          min(max(ys) + MARGIN, ey1 - EDGE_CLR)]
cx0, cy0, cx1, cy1 = canvas

objects = []           # {name, kind, x0, y0, w, h} bottom-left ref placement
obj_of = {}            # ref -> object index (movers + fixed)

for r in sorted(MOVED):
    x0, y0, x1, y1 = cy_bbox(r)
    dx, dy = MOVED[r]
    obj_of[r] = len(objects)
    objects.append(dict(name=r, kind='mov', x0=x0 + dx, y0=y0 + dy,
                        w=x1 - x0, h=y1 - y0,
                        ship_x0=x0, ship_y0=y0))

# ---- fixed blockers ----------------------------------------------------------
PAD_HALO = 0.15        # clearance halo around drilled pads, mm


pad_obj = {}           # (ref, pad name) -> object index of per-pad blocker


def add_fixed(name, x0, y0, x1, y1, ref=None, pad=None):
    x0, y0 = max(x0, cx0), max(y0, cy0)
    x1, y1 = min(x1, cx1), min(y1, cy1)
    if x1 - x0 < 0.05 or y1 - y0 < 0.05:
        return
    if ref is not None:
        obj_of[ref] = len(objects)
    if pad is not None:
        pad_obj[pad] = len(objects)
    objects.append(dict(name=name, kind='fix', x0=x0, y0=y0,
                        w=x1 - x0, h=y1 - y0))

for r in sorted(CY):
    if r in MOVED or MSIDE not in CY[r]:
        continue
    x0, y0, x1, y1 = cy_bbox(r, MSIDE)
    add_fixed(r, x0, y0, x1, y1, ref=r)

for r in sorted(foots):              # holed front parts: pads block, body not
    f = foots[r]
    if r in MOVED or r in obj_of:
        continue
    for p in f.Pads():
        if p.GetDrillSize().x <= 0:
            continue
        if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
            pos = p.GetPosition()
            hx, hy = p.GetDrillSize().x * NM / 2, p.GetDrillSize().y * NM / 2
            x0, y0 = pos.x * NM - hx, pos.y * NM - hy
            x1, y1 = pos.x * NM + hx, pos.y * NM + hy
        else:
            bb = p.GetBoundingBox()
            x0, y0 = bb.GetLeft() * NM, bb.GetTop() * NM
            x1, y1 = bb.GetRight() * NM, bb.GetBottom() * NM
        add_fixed(f'{r}.{p.GetPadName()}',
                  x0 - PAD_HALO, y0 - PAD_HALO, x1 + PAD_HALO, y1 + PAD_HALO,
                  pad=(r, str(p.GetPadName())))

# ---- grandfather by shipping: rev-B target placement must be legal -----------
# KiCad courtyard DRC never tests courtyard-vs-pad; where the shipped rev-B
# puts a mover inside a pad blocker's halo, the designer judged the real
# (round-hole / pin-protrusion) geometry fine. Shrink those blockers to the
# largest boxes that keep the reference placement clean.
EPS = 0.005
for o in [o for o in objects if o['kind'] == 'fix' and '.' in o['name']]:
    for r, (dx, dy) in MOVED.items():
        mx0, my0, mx1, my1 = cy_bbox(r)
        mx0, my0, mx1, my1 = mx0 + dx, my0 + dy, mx1 + dx, my1 + dy
        x0, y0 = o['x0'], o['y0']
        x1, y1 = x0 + o['w'], y0 + o['h']
        if min(x1, mx1) - max(x0, mx0) <= 0 or min(y1, my1) - max(y0, my0) <= 0:
            continue
        # cheapest single-edge cut that fully clears the mover's rect
        cuts = [(x1 - mx0, 'x1'), (mx1 - x0, 'x0'),
                (y1 - my0, 'y1'), (my1 - y0, 'y0')]
        cost, edge = min(cuts)
        cut = cost + EPS
        if edge == 'x1':
            o['w'] -= cut
        elif edge == 'x0':
            o['x0'] += cut
            o['w'] -= cut
        elif edge == 'y1':
            o['h'] -= cut
        else:
            o['y0'] += cut
            o['h'] -= cut
objects[:] = [o for o in objects
              if o['kind'] != 'fix' or (o['w'] > 0.05 and o['h'] > 0.05)]
name_idx = {o['name']: i for i, o in enumerate(objects)}
obj_of = {r: name_idx[r] for r in obj_of if r in name_idx}
pad_obj = {k: name_idx[f'{k[0]}.{k[1]}'] for k in pad_obj
           if f'{k[0]}.{k[1]}' in name_idx}

# ---- nets and terminals ------------------------------------------------------
pads = []              # whole board, from pcbnew (obstacles.json is region-cut)
for r, f in foots.items():
    for p in f.Pads():
        n = p.GetNetname()
        if n:
            pos = p.GetPosition()
            pads.append((r, str(p.GetPadName()), n, pos.x * NM, pos.y * NM))

nets_of_movers = {n for r, _, n, _, _ in pads if r in MOVED and n != 'GND'}
nets = []
n_ports = 0
for net in sorted(nets_of_movers):
    terms = []         # (obj_index, pin_x, pin_y) pin offsets from bbox min
    ext = []
    for r, num, n, px, py in pads:
        if n != net:
            continue
        oi = obj_of.get(r)
        if oi is None:
            oi = pad_obj.get((r, num))
        if oi is None:
            ext.append((px, py))
        else:
            o = objects[oi]
            if o['kind'] == 'mov':
                sx, sy = o['ship_x0'], o['ship_y0']   # pads sit at shipped pos
            else:
                sx, sy = o['x0'], o['y0']
            terms.append((oi, px - sx, py - sy))
    port = None
    if ext:
        mx = sum(x for x, _ in ext) / len(ext)
        my = sum(y for _, y in ext) / len(ext)
        px = min(max(mx, cx0 + PORT_SIZE / 2), cx1 - PORT_SIZE / 2)
        py = min(max(my, cy0 + PORT_SIZE / 2), cy1 - PORT_SIZE / 2)
        port = len(objects)
        objects.append(dict(name=f'port:{net}', kind='port',
                            x0=px - PORT_SIZE / 2, y0=py - PORT_SIZE / 2,
                            w=PORT_SIZE, h=PORT_SIZE))
        terms.insert(0, (port, PORT_SIZE / 2, PORT_SIZE / 2))
        n_ports += 1
    if len(terms) >= 2:
        src = terms[0]
        nets.append(dict(net=net,
                         edges=[[src[0], src[1], src[2], t[0], t[1], t[2]]
                                for t in terms[1:]]))

n_mov = sum(o['kind'] == 'mov' for o in objects)
n_fix = sum(o['kind'] == 'fix' for o in objects)
n_edges = sum(len(n['edges']) for n in nets)
with open(OUT, 'w') as f:
    json.dump(dict(canvas=canvas, objects=objects, nets=nets,
                   moved=MOVED), f, indent=1)
print(f'canvas {cx1-cx0:.1f} x {cy1-cy0:.1f} mm  '
      f'[{cx0:.2f},{cy0:.2f} .. {cx1:.2f},{cy1:.2f}]')
print(f'{n_mov} movers, {n_fix} fixed, {n_ports} ports; '
      f'{len(nets)} nets, {n_edges} star edges -> {OUT}')
