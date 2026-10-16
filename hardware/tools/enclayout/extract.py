#!/usr/bin/env python3
"""enclayout stage 0: board -> data/corner.json, the abstract model of the encoder corner.

Movable parts (bottom side, the encoder / hall / termination subsystem) are stored with their
rot-0 local geometry (courtyard bbox, pads with nets); everything else in the window is an
obstacle: fixed B-side courtyards, THT pads and vias of nets that touch no movable part,
mounting-hole keepouts, fixed B.Cu tracks (sampled to points).  Chains list the pad sequence
each differential path has to visit from its J12 pin to its transceiver.

    python3 extract.py [BOARD]      default: ../settle/candidates/plan_of_record.kicad_pcb
"""
import json, math, os, re, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'settle'))
from common import quiet_stderr
import pcbnew
HERE = os.path.dirname(os.path.abspath(__file__))
BOARD = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, '..', 'settle', 'candidates', 'plan_of_record.kicad_pcb')
NM = 1e-6
WIN = (111.0, 92.0, 138.0, 129.5)          # extraction window (mm)
REGION = (112.475, 96.0, 131.0, 128.425)   # where movable courtyards may live
MOVABLE = ('U18 U19 U20 U21 U22 U23 U24 U25 U26 U27 U30 U31 U32 U33 D23 D24 D25 D26 D27 '
           'FL1 FL2 FL3 FL4 R82 R83 R84 R85 R86 R87 R88 R89 R90 R91 R92 R93 R94 R95 R96 R97 R98 R99 R100 '
           'C60 C61 C62 C63 C64 C65 C66 C67 C68 C69 C70 C71 C72 C73 C74 C75 C110 C111 C112 C113').split()
SLIDERS = ['J1', 'J2']                      # phase connectors: may move up (−y) together
PLANES = {'GND', '+3V3', 'Vdrive'}
CHAIN = {  # per channel: J12 pins (P, N) and the ordered chain parts
    'A': dict(pins=('11', '9'), tvs='D23', sw=('U22', 'U26'), fl='FL2', trx='U18', term=('U30', 'R82')),
    'B': dict(pins=('7', '5'),  tvs='D24', sw=('U23', 'U27'), fl='FL1', trx='U19', term=('U31', 'R83')),
    'C': dict(pins=('3', '1'),  tvs='D25', sw=('U24', 'U25'), fl='FL3', trx='U20', term=('U32', 'R84')),
    'D': dict(pins=('6', '8'),  tvs='D26', sw=None,           fl='FL4', trx='U21', term=('U33', 'R85')),
}

with quiet_stderr():
    b = pcbnew.LoadBoard(BOARD)
fps = {f.GetReference(): f for f in b.GetFootprints()}

def unrot(dx, dy, deg):
    a = math.radians(deg)
    return (dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a))

def inwin(x, y):
    return WIN[0] <= x <= WIN[2] and WIN[1] <= y <= WIN[3]

mov_nets = set()
parts = {}
for r in MOVABLE:
    f = fps[r]; o = f.GetPosition(); ox, oy = o.x * NM, o.y * NM; rot = f.GetOrientationDegrees()
    cy = f.GetCourtyard(pcbnew.B_CrtYd).BBox()
    corners = [unrot((cy.GetLeft() * NM) - ox, (cy.GetTop() * NM) - oy, rot), unrot((cy.GetRight() * NM) - ox, (cy.GetBottom() * NM) - oy, rot)]
    xs = [c[0] for c in corners]; ys = [c[1] for c in corners]
    pads = []
    for p in f.Pads():
        q = p.GetPosition(); lx, ly = unrot(q.x * NM - ox, q.y * NM - oy, rot)
        sz = p.GetSize(); sw, sh = sz.x * NM, sz.y * NM
        if round(rot) % 180 == 90: sw, sh = sh, sw   # size is in board frame; local frame swaps at 90
        net = str(p.GetNetname())
        if net and not net.startswith('unconnected'): mov_nets.add(net)
        pads.append(dict(n=str(p.GetNumber()), net=net, x=round(lx, 4), y=round(ly, 4), w=round(sw, 3), h=round(sh, 3)))
    parts[r] = dict(ref=r, value=str(f.GetValue()), fp=str(f.GetFPID().GetLibItemName()),
                    crt=[round(min(xs), 3), round(min(ys), 3), round(max(xs), 3), round(max(ys), 3)],
                    pads=pads, pose=[round(ox, 4), round(oy, 4), round(rot) % 360])

# --- obstacles -----------------------------------------------------------------
rects, circles, holes = [], [], []      # rects: fixed B courtyards; circles: copper on B (pads/vias); holes: drill-ish
for r, f in fps.items():
    if r in MOVABLE: continue
    o = f.GetPosition()
    if f.IsFlipped():
        cy = f.GetCourtyard(pcbnew.B_CrtYd).BBox()
        if cy.GetWidth() > 0 and inwin(o.x * NM, o.y * NM):
            rects.append(dict(ref=r, box=[cy.GetLeft() * NM, cy.GetTop() * NM, cy.GetRight() * NM, cy.GetBottom() * NM]))
    for p in f.Pads():
        q = p.GetPosition(); x, y = q.x * NM, q.y * NM
        if not inwin(x, y): continue
        att = p.GetAttribute()
        if att in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH):
            sx, sy = p.GetSize().x * NM, p.GetSize().y * NM
            rad = max(sx, sy) / 2 if p.GetShape() in (pcbnew.PAD_SHAPE_CIRCLE, pcbnew.PAD_SHAPE_OVAL) else math.hypot(sx, sy) / 2  # square pads: corner radius
            drill = max(p.GetDrillSize().x, p.GetDrillSize().y) * NM / 2
            circles.append(dict(ref=f'{r}.{p.GetNumber()}', x=x, y=y, r=rad, slider=r in SLIDERS, net=str(p.GetNetname())))
            # a through-hole pad is a solder joint on the bottom side: bodies must clear the whole annulus, not just the drill
            holes.append(dict(ref=f'{r}.{p.GetNumber()}', x=x, y=y, r=rad, slider=r in SLIDERS))
        elif p.IsOnLayer(pcbnew.B_Cu) and not f.IsFlipped():
            pass
        elif f.IsFlipped() and p.IsOnLayer(pcbnew.B_Cu) and not any(rr['ref'] == r for rr in rects):
            sz = p.GetSize(); rects.append(dict(ref=f'{r}.{p.GetNumber()}', box=[x - sz.x * NM / 2, y - sz.y * NM / 2, x + sz.x * NM / 2, y + sz.y * NM / 2]))
# vias: obstacles unless their net touches a movable part (those get re-routed)
fixed_pads = [(str(p.GetNetname()), p.GetPosition().x * NM, p.GetPosition().y * NM) for r, f in fps.items() if r not in MOVABLE for p in f.Pads()]
vias_kept, vias_dropped = [], []
for t in b.GetTracks():
    if t.GetClass() != 'PCB_VIA': continue
    q = t.GetPosition(); x, y = q.x * NM, q.y * NM
    if not inwin(x, y): continue
    net = str(t.GetNetname()); rad = t.GetWidth() * NM / 2; drill = t.GetDrillValue() * NM / 2
    keep = net not in mov_nets
    if net in PLANES:   # a plane via next to a fixed part's pad belongs to that part
        keep = any(n == net and math.hypot(px - x, py - y) < 1.2 for n, px, py in fixed_pads)
    (vias_kept if keep else vias_dropped).append(dict(x=x, y=y, r=rad, drill=drill, net=net))
for v in vias_kept:
    circles.append(dict(ref='via', x=v['x'], y=v['y'], r=v['r'], slider=False, net=v['net']))
    holes.append(dict(ref='via', x=v['x'], y=v['y'], r=v['drill'], slider=False))
# keepouts (mounting holes) — footprints not allowed: courtyard must stay outside
def all_zones():
    for z in b.Zones(): yield z
    for f in b.GetFootprints():
        for z in f.Zones(): yield z
for z in all_zones():
    if not z.GetIsRuleArea(): continue
    bb = z.GetBoundingBox(); cx, cy_ = bb.GetCenter().x * NM, bb.GetCenter().y * NM
    if inwin(cx, cy_) or inwin(bb.GetLeft() * NM, bb.GetBottom() * NM):
        rects.append(dict(ref='keepout', box=[bb.GetLeft() * NM, bb.GetTop() * NM, bb.GetRight() * NM, bb.GetBottom() * NM],
                          circle=[cx, cy_, bb.GetWidth() * NM / 2]))
# fixed B.Cu tracks of nets that touch no movable part: sampled points
pts = []
for t in b.GetTracks():
    if t.GetClass() != 'PCB_TRACK' or t.GetLayer() != pcbnew.B_Cu: continue
    net = str(t.GetNetname())
    if net in mov_nets: continue
    s, e = t.GetStart(), t.GetEnd(); x1, y1, x2, y2 = s.x * NM, s.y * NM, e.x * NM, e.y * NM
    if not (inwin(x1, y1) or inwin(x2, y2)): continue
    n = max(1, int(math.hypot(x2 - x1, y2 - y1) / 0.25)); w = t.GetWidth() * NM
    for i in range(n + 1):
        f_ = i / n; pts.append([round(x1 + (x2 - x1) * f_, 3), round(y1 + (y2 - y1) * f_, 3), round(w / 2, 3)])
# copper on ANY layer near the slider pads: what the connector pins would run into when the stack moves up
slider_pads = [(c['x'], c['y'], c['r'], c['net']) for c in circles if c['slider']]
spts = []
for t in b.GetTracks():
    net = str(t.GetNetname())
    if net in mov_nets: continue      # re-routed with the corner anyway
    if t.GetClass() == 'PCB_TRACK':
        s_, e_ = t.GetStart(), t.GetEnd(); x1, y1, x2, y2 = s_.x * NM, s_.y * NM, e_.x * NM, e_.y * NM; w = t.GetWidth() * NM / 2
        n = max(1, int(math.hypot(x2 - x1, y2 - y1) / 0.2))
        cand = [(x1 + (x2 - x1) * i / n, y1 + (y2 - y1) * i / n) for i in range(n + 1)]
    else:
        q = t.GetPosition(); cand = [(q.x * NM, q.y * NM)]; w = t.GetWidth() * NM / 2
    for x, y in cand:
        if any(math.hypot(x - px, y - py) < pr + 3.0 and net != pn for px, py, pr, pn in slider_pads):
            spts.append([round(x, 3), round(y, 3), round(w, 3), net])
# board outline
bb = b.GetBoardEdgesBoundingBox()
outline = dict(box=[bb.GetLeft() * NM, bb.GetTop() * NM, bb.GetRight() * NM, bb.GetBottom() * NM], corner_r=1.5)
# J12 pins
j12 = {str(p.GetNumber()): dict(x=p.GetPosition().x * NM, y=p.GetPosition().y * NM, net=str(p.GetNetname())) for p in fps['J12'].Pads()}
# net anchors: for nets that leave the corner, the nearest fixed pad (excluding J12, which is explicit)
anchors = {}
for net in sorted(mov_nets):
    if net in PLANES: continue
    cands = [(math.hypot(px - 122, py - 118), px, py) for n, px, py in fixed_pads if n == net]
    if cands:
        d, px, py = min(cands)
        if px > REGION[2] + 4:   # leaves the corner: what matters is reaching the right edge
            px, py = REGION[2] + 1.0, min(max(py, REGION[1] + 2), REGION[3] - 2)
        anchors[net] = [round(px, 3), round(py, 3)]
sliders = {r: dict(pose=[fps[r].GetPosition().x * NM, fps[r].GetPosition().y * NM],
                   crt=[fps[r].GetCourtyard(pcbnew.F_CrtYd).BBox().GetLeft() * NM, fps[r].GetCourtyard(pcbnew.F_CrtYd).BBox().GetTop() * NM,
                        fps[r].GetCourtyard(pcbnew.F_CrtYd).BBox().GetRight() * NM, fps[r].GetCourtyard(pcbnew.F_CrtYd).BBox().GetBottom() * NM])
           for r in SLIDERS}
out = dict(board=os.path.relpath(BOARD, HERE), window=WIN, region=REGION, outline=outline, parts=parts, rects=rects,
           circles=circles, holes=holes, track_pts=pts, slider_pts=spts, j12=j12, chain=CHAIN, anchors=anchors, sliders=sliders,
           vias_dropped=vias_dropped, planes=sorted(PLANES))
json.dump(out, open(os.path.join(HERE, 'data', 'corner.json'), 'w'), indent=0)
print(f'{len(parts)} movable parts, {len(rects)} fixed rects, {len(circles)} circles ({len(vias_kept)} vias kept, {len(vias_dropped)} dropped), '
      f'{len(pts)} track points, {len(spts)} slider-conflict points, {len(anchors)} anchored nets')
from collections import Counter
print('kept via nets:', Counter(v['net'] for v in vias_kept).most_common())
print('fixed rects:', ' '.join(r['ref'] for r in rects))
