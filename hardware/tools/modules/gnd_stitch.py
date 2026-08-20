#!/usr/bin/env python3
"""Fan out still-unconnected GND (or any plane-net) pads to the inner planes:
for each SMD pad of NET that the ratsnest reports unconnected, search a 0.1 mm
grid within R mm for a via site clear of other-net copper (pads/tracks/vias
with rule clearance, holes with hole-to-hole 0.254, edge inset), add the via and
a track-width stub from the pad centre. Real DRC referees the result."""
import sys, os, math, json, collections
S = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, S)
sys.path.insert(0, '/home/sequoia/pcb/rp2350-motor-controller/hardware/tools/jiggle2')
from common import load_board, NM, quiet_stderr
import pcbnew
from geom import seg_seg
SRC, OUT = sys.argv[1], sys.argv[2]; NET = sys.argv[3] if len(sys.argv) > 3 else 'GND'
R = 1.6; VIA_D, VIA_DR = 0.5, 0.25; CLR = 0.18; HOLE = 0.254; EDGE = 0.3; W = 0.2
import shutil; shutil.copy(SRC.replace('.kicad_pcb', '.kicad_pro'), OUT.replace('.kicad_pcb', '.kicad_pro'))
bd = load_board(SRC); conn = bd.GetConnectivity(); conn.RecalculateRatsnest()
net = bd.FindNet(NET); nc = net.GetNetCode()
bb = bd.GetBoardEdgesBoundingBox(); X0, Y0, X1, Y1 = bb.GetLeft()*NM, bb.GetTop()*NM, bb.GetRight()*NM, bb.GetBottom()*NM
# obstacles
pads = []; segs = collections.defaultdict(list); vias = []; holes = []
for f in bd.GetFootprints():
    for p in f.Pads():
        b = p.GetBoundingBox(); box = (b.GetLeft()*NM, b.GetTop()*NM, b.GetRight()*NM, b.GetBottom()*NM)
        try: lc = max(p.GetLocalClearance() or 0, f.GetLocalClearance() or 0)*NM
        except TypeError: lc = 0
        pads.append((box, p.GetNetCode(), p.IsOnLayer(pcbnew.F_Cu), p.IsOnLayer(pcbnew.B_Cu), max(lc, CLR), bool(p.GetDrillSize().x)))
        if p.GetDrillSize().x: holes.append((p.GetPosition().x*NM, p.GetPosition().y*NM, max(p.GetDrillSize().x, p.GetDrillSize().y)*NM/2))
for t in bd.GetTracks():
    if t.GetClass() == 'PCB_VIA':
        p = t.GetPosition(); vias.append((p.x*NM, p.y*NM, t.GetWidth(pcbnew.PADSTACK.ALL_LAYERS)*NM/2, t.GetNetCode())); holes.append((p.x*NM, p.y*NM, t.GetDrill()*NM/2))
    else:
        s, e = t.GetStart(), t.GetEnd(); segs[int(t.GetLayer())].append(((s.x*NM, s.y*NM), (e.x*NM, e.y*NM), t.GetWidth()*NM/2, t.GetNetCode()))
COPPER = [int(l) for l in bd.GetEnabledLayers().CuStack()]
ZONES = [z for z in bd.Zones() if not z.GetIsRuleArea() and z.GetNetCode() == nc]
def in_zone(x, y):
    if not ZONES: return True
    v = pcbnew.VECTOR2I(int(round(x/NM)), int(round(y/NM)))
    for z in ZONES:
        for l in z.GetLayerSet().Seq():
            if z.HitTestFilledArea(l, v, 0): return True
    return False
def site_ok(x, y):
    if not in_zone(x, y): return False
    if x < X0+EDGE+VIA_D/2 or x > X1-EDGE-VIA_D/2 or y < Y0+EDGE+VIA_D/2 or y > Y1-EDGE-VIA_D/2: return False
    for hx, hy, hr in holes:
        if math.hypot(hx-x, hy-y) < hr + VIA_DR/2 + HOLE - 1e-6: return False
    for box, pnc, onf, onb, clr, drill in pads:
        if pnc == nc: continue
        if box[0]-clr-VIA_D/2 < x < box[2]+clr+VIA_D/2 and box[1]-clr-VIA_D/2 < y < box[3]+clr+VIA_D/2: return False
    for vx, vy, vr, vnc in vias:
        if vnc == nc: continue
        if math.hypot(vx-x, vy-y) < vr + VIA_D/2 + CLR: return False
    for lay in COPPER:
        for a, b, hw, snc in segs[lay]:
            if snc == nc: continue
            if seg_seg(a, b, (x, y), (x, y)) < hw + VIA_D/2 + CLR: return False
    return True
def stub_ok(ax, ay, bx, by, layer):
    for box, pnc, onf, onb, clr, drill in pads:
        if pnc == nc or not ((layer == pcbnew.F_Cu and onf) or (layer == pcbnew.B_Cu and onb)): continue
        # segment vs box distance
        cx = min(max(ax, box[0]), box[2]); cy = min(max(ay, box[1]), box[3])
        if seg_seg((ax, ay), (bx, by), (box[0], box[1]), (box[2], box[1])) < W/2+clr or seg_seg((ax, ay), (bx, by), (box[2], box[1]), (box[2], box[3])) < W/2+clr or \
           seg_seg((ax, ay), (bx, by), (box[2], box[3]), (box[0], box[3])) < W/2+clr or seg_seg((ax, ay), (bx, by), (box[0], box[3]), (box[0], box[1])) < W/2+clr: return False
    for a, b, hw, snc in segs[int(layer)]:
        if snc == nc: continue
        if seg_seg(a, b, (ax, ay), (bx, by)) < hw + W/2 + CLR: return False
    for vx, vy, vr, vnc in vias:
        if vnc == nc: continue
        if seg_seg((ax, ay), (bx, by), (vx, vy), (vx, vy)) < vr + W/2 + CLR: return False
    return True
# unconnected pads of NET: take them from a fresh DRC ratsnest (pads named in unconnected_items of NET)
import re, subprocess
from drcdiff import run_drc
dj = run_drc(SRC, S+'/drc_stitch_pre.json'); want = set()
for it in dj['unconnected_items']:
    for i in it['items']:
        m = re.match(r'Pad (\S+) \[([^\]]+)\] of (\S+) ', i['description'])
        if m and m.group(2) == NET: want.add((m.group(3), m.group(1)))
print(f'{len(want)} {NET} pads named in the ratsnest')
targets = []
for f in bd.GetFootprints():
    for p in f.Pads():
        if p.GetNetCode() != nc or p.GetDrillSize().x: continue
        if (f.GetReference(), p.GetNumber()) not in want: continue
        # connected to a via (through-plane) or a THT pad already?
        ok = False
        for x in conn.GetConnectedTracks(p):
            if x.GetClass() == 'PCB_VIA': ok = True; break
        if ok: continue
        for q in conn.GetConnectedPads(p):
            if q.GetDrillSize().x: ok = True; break
        if ok: continue
        # a zone of NET on this pad's layer covering the pad also counts
        for z in bd.Zones():
            if z.GetIsRuleArea() or z.GetNetCode() != nc: continue
            for l in z.GetLayerSet().Seq():
                if p.IsOnLayer(l) and z.HitTestFilledArea(l, p.GetPosition(), 0): ok = True; break
            if ok: break
        if ok: continue
        targets.append(p)
print(f'{len(targets)} {NET} SMD pads without via/plane reach')
added = 0; failed = []
for p in targets:
    pp = p.GetPosition(); px, py = pp.x*NM, pp.y*NM; layer = pcbnew.F_Cu if p.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
    best = None
    n = int(R/0.1)
    for i in range(-n, n+1):
        for j in range(-n, n+1):
            x, y = px+i*0.1, py+j*0.1; d = math.hypot(x-px, y-py)
            if d > R or (best and d >= best[0]): continue
            if not site_ok(x, y): continue
            if not stub_ok(px, py, x, y, layer): continue
            best = (d, x, y)
    if not best: failed.append(p.GetParentFootprint().GetReference()+'.'+p.GetNumber()); continue
    d, x, y = best
    v = pcbnew.PCB_VIA(bd); v.SetPosition(pcbnew.VECTOR2I(int(round(x/NM)), int(round(y/NM)))); v.SetDrill(int(VIA_DR/NM))
    try: v.SetWidth(int(VIA_D/NM))
    except TypeError: v.SetWidth(pcbnew.PADSTACK.ALL_LAYERS, int(VIA_D/NM))
    v.SetViaType(pcbnew.VIATYPE_THROUGH); v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu); v.SetNet(net); bd.Add(v)
    if d > 1e-3:
        t = pcbnew.PCB_TRACK(bd); t.SetStart(pp); t.SetEnd(v.GetPosition()); t.SetWidth(int(W/NM)); t.SetLayer(layer); t.SetNet(net); bd.Add(t)
    vias.append((x, y, VIA_D/2, nc)); holes.append((x, y, VIA_DR/2)); segs[int(layer)].append(((px, py), (x, y), W/2, nc)); added += 1
print(f'added {added} stitch vias; failed {len(failed)}: {failed[:20]}')
with quiet_stderr():
    pcbnew.ZONE_FILLER(bd).Fill(bd.Zones()); pcbnew.SaveBoard(OUT, bd)
bd2 = load_board(OUT); print('unconnected now', bd2.GetConnectivity().GetUnconnectedCount(True))
