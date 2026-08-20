#!/usr/bin/env python3
"""Post-route analysis: compare routed board with the pre-route board, prune
dangling new copper (tracks ending nowhere / vias on one layer), report per-net
remaining airwires and DRC delta, write route_result.json (+html snippet)."""
import sys, os, json, collections, re, html
S = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, S)
sys.path.insert(0, '/home/sequoia/pcb/rp2350-motor-controller/hardware/tools/jiggle2')
from common import load_board, NM, quiet_stderr
import pcbnew
from drcdiff import run_drc, diff, summary, norm
PRE = sys.argv[1]; ROUTED = sys.argv[2]; OUT = sys.argv[3]; PRE_DRC = sys.argv[4]
pre = load_board(PRE); bd = load_board(ROUTED)
import shutil; shutil.copy(ROUTED.replace('.kicad_pcb', '.kicad_pro'), OUT.replace('.kicad_pcb', '.kicad_pro'))
def sig_t(t):
    s, e = t.GetStart(), t.GetEnd(); a, b = (s.x, s.y), (e.x, e.y)
    if b < a: a, b = b, a
    return ('T', int(t.GetLayer()), a, b, t.GetWidth())
def sig_v(v): p = v.GetPosition(); return ('V', p.x, p.y)
pre_sigs = {sig_t(t) if t.GetClass() == 'PCB_TRACK' else sig_v(t) for t in pre.GetTracks()}
new_items = [t for t in bd.GetTracks() if (sig_t(t) if t.GetClass() == 'PCB_TRACK' else sig_v(t)) not in pre_sigs]
by_layer = collections.Counter(); nvias = 0; length = 0.0; nets = collections.Counter()
for t in new_items:
    if t.GetClass() == 'PCB_VIA': nvias += 1
    else: by_layer[bd.GetLayerName(t.GetLayer())] += 1; length += t.GetLength()*NM
    nets[t.GetNetname()] += 1
print(f'new copper: {len(new_items)-nvias} segments ({length:.0f} mm), {nvias} vias; by layer {dict(by_layer)}')
# prune: repeatedly drop new segments with a free end (not on pad/via/other track/zone) and new vias with <2 connected layers
conn = bd.GetConnectivity()
pruned_t = pruned_v = 0
for it in range(0 if os.environ.get("NOPRUNE") else 6):
    conn = bd.GetConnectivity(); conn.RecalculateRatsnest()
    drop = []
    for t in list(bd.GetTracks()):
        if (sig_t(t) if t.GetClass() == 'PCB_TRACK' else sig_v(t)) in pre_sigs: continue
        if t.GetClass() == 'PCB_VIA':
            # dangling if tracks/pads/zones connect on fewer than 2 layers
            items = [i for i in conn.GetConnectedItems(t, [pcbnew.PCB_TRACE_T, pcbnew.PCB_PAD_T, pcbnew.PCB_ZONE_T])] if False else None
            try:
                tr = conn.GetConnectedTracks(t); pads = conn.GetConnectedPads(t)
                lays = set()
                for x in tr: lays.add(int(x.GetLayer()))
                for p in pads: lays |= {int(l) for l in p.GetLayerSet().Seq() if pcbnew.IsCopperLayer(l)}
                # zones: via into plane counts (GND/+3V3 planes) -> check zones of same net covering the via
                for z in bd.Zones():
                    if z.GetIsRuleArea() or z.GetNetCode() != t.GetNetCode(): continue
                    if z.HitTestFilledArea(z.GetFirstLayer(), t.GetPosition(), 0): lays.add(int(z.GetFirstLayer()))
                if len(lays) < 2: drop.append(t)
            except Exception as e: pass
        else:
            try:
                tr = conn.GetConnectedTracks(t); pads = conn.GetConnectedPads(t)
                s, e = t.GetStart(), t.GetEnd()
                def anchored(P):
                    for x in tr:
                        if x.GetClass() == 'PCB_VIA':
                            if x.HitTest(P): return True
                        else:
                            if x.GetStart() == P or x.GetEnd() == P or x.HitTest(P): return True
                    for p in pads:
                        if p.HitTest(P): return True
                    for z in bd.Zones():
                        if z.GetIsRuleArea() or z.GetNetCode() != t.GetNetCode(): continue
                        if t.GetLayer() in [l for l in z.GetLayerSet().Seq()] and z.HitTestFilledArea(t.GetLayer(), P, 0): return True
                    return False
                if not anchored(s) or not anchored(e): drop.append(t)
            except Exception as e: pass
    if not drop: break
    for t in drop:
        if t.GetClass() == 'PCB_VIA': pruned_v += 1
        else: pruned_t += 1
        bd.Delete(t)
print(f'pruned dangling new copper: {pruned_t} segments, {pruned_v} vias')
with quiet_stderr():
    pcbnew.ZONE_FILLER(bd).Fill(bd.Zones()); pcbnew.SaveBoard(OUT, bd)
bd2 = load_board(OUT); unc = bd2.GetConnectivity().GetUnconnectedCount(True)
pre_d = json.load(open(PRE_DRC)); cur = run_drc(OUT, S+'/drc_routed.json'); new, gone = diff(pre_d, cur)
c = collections.Counter(v['type'] for v in cur['violations'])
print('pre-route :', summary(pre_d)); print('routed    :', summary(cur)); print(f'new {len(new)} gone {len(gone)}; unconnected {unc}')
rem = collections.Counter()
for it in cur['unconnected_items']:
    m = re.search(r'\[([^\]]+)\]', it['items'][0]['description']); rem[m.group(1) if m else '?'] += 1
newc = collections.Counter(v['type'] for v in new)
print('remaining airwires by net:', rem.most_common(40)); print('new violations by type:', dict(newc))
for v in new[:25]:
    p = v['items'][0].get('pos', {}); print(f"  NEW {v['type']:20s} ({p.get('x',0):.2f},{p.get('y',0):.2f}) " + ' | '.join(norm(i['description'])[:55] for i in v['items']))
res = dict(drc=dict(v=len(cur['violations']), u=len(cur['unconnected_items']), by=dict(c)), new_segments=len(new_items)-nvias, new_vias=nvias, new_length_mm=round(length), by_layer=dict(by_layer),
           pruned_t=pruned_t, pruned_v=pruned_v, new_viol=len(new), new_viol_by=dict(newc), gone=len(gone), unconnected=unc, remaining=rem.most_common(), nets_touched=len(nets))
res['html'] = ''  # filled by hand in render step
json.dump(res, open(S+'/route_result.json', 'w'), indent=1)
