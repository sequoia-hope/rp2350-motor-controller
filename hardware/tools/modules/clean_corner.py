#!/usr/bin/env python3
"""Corner cleanup before routing: remove copper that collides with pads of the
relaxed/new parts (stray vias under pads, track segments inside pad clearance),
and nudge the two parts that sit in J2's peg/pin clearance. Everything removed
becomes an airwire for the router; nothing is added. Loops DRC until no
copper-vs-pad violation remains (max 4 rounds)."""
import os, sys, json, re, math, collections
S = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, S)
sys.path.insert(0, '/home/sequoia/pcb/rp2350-motor-controller/hardware/tools/jiggle2')
from common import load_board, NM, quiet_stderr
import pcbnew
from drcdiff import run_drc, summary
SRC = sys.argv[1]; OUT = sys.argv[2]
import shutil; shutil.copy(SRC, OUT); shutil.copy(SRC.replace('.kicad_pcb', '.kicad_pro'), OUT.replace('.kicad_pcb', '.kicad_pro'))
log = []
for rnd in range(4):
    d = run_drc(OUT, f'{S}/drc_clean_{rnd}.json')
    print(f'round {rnd}: {summary(d)}')
    bd = load_board(OUT)
    kill_vias = {}; kill_segs = []
    for v in d['violations']:
        if v['type'] not in ('shorting_items', 'clearance', 'hole_clearance'): continue
        items = v['items']
        descs = [i['description'] for i in items]
        has_pad = any(d_.startswith('Pad') or 'PTH pad' in d_ for d_ in descs)
        if not has_pad: 
            # track-track (C40/C_SENSE case): rip the narrower-net track? skip; router will not fix it. handle below via pads only
            continue
        for it in items:
            de = it['description']
            if de.startswith('Via'):
                kill_vias[(round(it['pos']['x'], 3), round(it['pos']['y'], 3))] = de
            elif de.startswith('Track'):
                kill_segs.append(((it['pos']['x'], it['pos']['y']), de, v['type']))
    # also the two track-vs-track pairs at C40 (C_SENSE vs +3V3 after the 0402 shrink): rip the C_SENSE segments there
    for v in d['violations']:
        if v['type'] == 'clearance' and all(i['description'].startswith('Track') for i in v['items']):
            for it in v['items']:
                if '[C_SENSE]' in it['description']: kill_segs.append(((it['pos']['x'], it['pos']['y']), it['description'], 'track-track'))
    nv = ns = 0
    for t in list(bd.GetTracks()):
        if t.GetClass() == 'PCB_VIA':
            p = t.GetPosition(); key = (round(p.x*NM, 3), round(p.y*NM, 3))
            hit = any(abs(key[0]-k[0]) < 0.02 and abs(key[1]-k[1]) < 0.02 for k in kill_vias)
            if hit: bd.Delete(t); nv += 1
    # segments: match by net name in description + violation position near the segment
    segs_by_net = collections.defaultdict(list)
    for t in bd.GetTracks():
        if t.GetClass() == 'PCB_TRACK': segs_by_net[t.GetNetname()].append(t)
    def pt_seg(px, py, t):
        s, e = t.GetStart(), t.GetEnd(); ax, ay, bx, by = s.x*NM, s.y*NM, e.x*NM, e.y*NM
        dx, dy = bx-ax, by-ay; L2 = dx*dx+dy*dy; u = 0 if L2 < 1e-18 else max(0, min(1, ((px-ax)*dx+(py-ay)*dy)/L2))
        return math.hypot(px-ax-u*dx, py-ay-u*dy)
    doomed = set()
    for (px, py), de, typ in kill_segs:
        m = re.search(r'\[([^\]]+)\]', de); net = m.group(1) if m else None
        lm = re.search(r' on ([A-Za-z0-9.]+)', de); lay = lm.group(1) if lm else None
        best = None
        for t in segs_by_net.get(net, []):
            if lay and bd.GetLayerName(t.GetLayer()) != lay: continue
            dd = pt_seg(px, py, t) - t.GetWidth()*NM/2
            if dd < 0.25 and (best is None or dd < best[0]): best = (dd, t)
        if best: doomed.add(best[1].m_Uuid.AsString())
    for t in list(bd.GetTracks()):
        if t.GetClass() == 'PCB_TRACK' and t.m_Uuid.AsString() in doomed: bd.Delete(t); ns += 1
    print(f'   removed {nv} vias, {ns} segments')
    log.append(dict(round=rnd, before=summary(d), vias=nv, segs=ns))
    with quiet_stderr():
        pcbnew.ZONE_FILLER(bd).Fill(bd.Zones()); pcbnew.SaveBoard(OUT, bd)
    if nv == 0 and ns == 0: break
d = run_drc(OUT, f'{S}/drc_clean_final.json'); print('final:', summary(d))
bd = load_board(OUT); print('unconnected (ratsnest):', bd.GetConnectivity().GetUnconnectedCount(True))
json.dump(log, open(S+'/clean_log.json', 'w'), indent=1)
