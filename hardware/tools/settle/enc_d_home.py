#!/usr/bin/env python3
"""settle: bring encoder channel D home — its cluster re-placed beside its own
J12 pins instead of 20 mm away at the top of the section.

Base: the corner-squeezed board (enc_squeeze_final). Channel D's copper inside
the region is stripped (its pair, transceiver and termination nets entirely;
the GND / +3V3 / DATA / SW stubs hanging off the cluster's pads back to the
first via or foreign pad), and the cluster's ten parts become ghosts:
D26 / FL4 / R90 / R91 / R100 site-searched around J12 pins 6+8 (their only
non-ghost partners), U21 / R85 / U33 / C62 / C63 placed relative to FL4 / U21.

    python3 enc_d_home.py            writes data/boards/enc_d_base.kicad_pcb + configs/enc_d_home.json
"""
import json, os, shutil, sys
from common import HERE, load_board, quiet_stderr, in_rect
import pcbnew
NM = 1e6
CFG = os.path.join(HERE, 'configs'); BOARDS = os.path.join(CFG, 'data', 'boards')
SRC = os.path.join(BOARDS, 'enc_squeeze_final.kicad_pcb')
DST = os.path.join(BOARDS, 'enc_d_base.kicad_pcb')
REGION = [112.4, 96.0, 130.5, 128.5]
GHOSTS = ['D26', 'FL4', 'R90', 'R91', 'R100', 'U21', 'R85', 'U33', 'C62', 'C63']
LOCAL_NETS = {'/ENC_D_N', '/ENC_D_P', '/ENC_D_TRX_A', '/ENC_D_TRX_B', '/TERM_D_R'}

with quiet_stderr():
    board = pcbnew.LoadBoard(SRC)
fps = {f.GetReference(): f for f in board.GetFootprints()}
ghost_pads = [p for r in GHOSTS for p in fps[r].Pads()]
other_pads = [p for r, f in fps.items() if r not in GHOSTS for p in f.Pads()]
tracks = [t for t in board.GetTracks() if t.GetClass() == 'PCB_TRACK']
vias = [t for t in board.GetTracks() if t.GetClass() == 'PCB_VIA']
def inreg(v): return in_rect(v.x / NM, v.y / NM, REGION)
def same(a, b): return abs(a.x - b.x) < 1000 and abs(a.y - b.y) < 1000
doomed = set()
def key(t): return t.m_Uuid.AsString()
# (a) the local nets, entirely, inside the region
for t in tracks:
    if t.GetNetname() in LOCAL_NETS and (inreg(t.GetStart()) or inreg(t.GetEnd())):
        doomed.add(key(t))
for v in vias:
    if v.GetNetname() in LOCAL_NETS and inreg(v.GetPosition()):
        doomed.add(key(v))
# (b) stubs from ghost pads on shared nets, flooded to the first via / foreign pad
via_pts = [v.GetPosition() for v in vias]
def stop_point(pt, net):
    return any(same(pt, vp) for vp in via_pts) or any(p.GetNetname() == net and p.HitTest(pt) for p in other_pads)
frontier = []
for t in tracks:
    if key(t) in doomed: continue
    for pad in ghost_pads:
        if pad.GetNetname() == t.GetNetname() and (pad.HitTest(t.GetStart()) or pad.HitTest(t.GetEnd())):
            doomed.add(key(t)); frontier.append(t); break
while frontier:
    t = frontier.pop()
    for end in (t.GetStart(), t.GetEnd()):
        if stop_point(end, t.GetNetname()): continue
        for u in tracks:
            if key(u) in doomed or u.GetNetname() != t.GetNetname(): continue
            if same(u.GetStart(), end) or same(u.GetEnd(), end):
                doomed.add(key(u)); frontier.append(u)
items = [t for t in tracks + vias if key(t) in doomed]
from collections import Counter
print('stripping', len(items), 'items:', Counter(t.GetNetname() for t in items).most_common())
for t in items:
    board.Delete(t)
with quiet_stderr():
    pcbnew.SaveBoard(DST, board)
shutil.copy(SRC.replace('.kicad_pcb', '.kicad_pro'), DST.replace('.kicad_pcb', '.kicad_pro'))
print('wrote', DST)

c = json.load(open(os.path.join(CFG, 'enc_squeeze.json')))
c['name'] = 'enc_d_home'; c['board'] = 'data/boards/enc_d_base.kicad_pcb'
c['drive'] = {'shove': [], 'inflate': [
    {'ref': 'D26', 'at': 'search', 'radius': 4.0},
    {'ref': 'FL4', 'at': 'search', 'radius': 4.0},
    {'ref': 'R90', 'at': 'search', 'radius': 4.0},
    {'ref': 'R91', 'at': 'search', 'radius': 4.0},
    {'ref': 'R100', 'at': 'search', 'radius': 4.0},
    {'ref': 'U21', 'at': {'near': 'FL4', 'offset': [3.2, -2.5], 'search': 3.0}},
    {'ref': 'R85', 'at': {'near': 'U21', 'offset': [-2.4, 0.0], 'search': 2.0}},
    {'ref': 'U33', 'at': {'near': 'U21', 'offset': [0.0, 3.4], 'search': 2.5}},
    {'ref': 'C62', 'at': {'near': 'U21', 'offset': [2.6, -0.6], 'search': 2.0}},
    {'ref': 'C63', 'at': {'near': 'U21', 'offset': [2.6, 0.8], 'search': 2.0}},
]}
c['cycles'] = 600
json.dump(c, open(os.path.join(CFG, 'enc_d_home.json'), 'w'), indent=1)
print('wrote configs/enc_d_home.json')
