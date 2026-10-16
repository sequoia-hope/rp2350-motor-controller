#!/usr/bin/env python3
"""enclayout stage 3: write one candidate into a board copy and measure it.

    python3 apply.py RUN NAME [--out BOARD.kicad_pcb] [--drc]

Moves the 62 corner parts to the candidate's poses, slides J1/J2 up by dy, strips the corner copper that the model
assumed re-routed (B.Cu tracks of nets touching a movable part inside the window, plus the vias the model dropped),
refills zones, saves beside the .kicad_pro, and (with --drc) runs kicad-cli DRC and counts courtyard / clearance /
keepout violations and airwires.  The result is a placement to route, not a routed board.
"""
import json, math, os, re, subprocess, sys, shutil
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'settle'))
from common import quiet_stderr
import pcbnew
HERE = os.path.dirname(os.path.abspath(__file__)); NM = 1e-6
run, name = sys.argv[1], sys.argv[2]
rec = next(r for r in json.load(open(os.path.join(HERE, 'data', 'runs', run, 'results.json'))) if r['name'] == name)
D = json.load(open(os.path.join(HERE, 'data', 'corner.json')))
src = os.path.normpath(os.path.join(HERE, D['board']))
out = sys.argv[sys.argv.index('--out') + 1] if '--out' in sys.argv else os.path.join(HERE, 'data', 'boards', f'{run}_{name}.kicad_pcb')
os.makedirs(os.path.dirname(out), exist_ok=True)
with quiet_stderr():
    b = pcbnew.LoadBoard(src)
fps = {f.GetReference(): f for f in b.GetFootprints()}
mov = set(D['parts']); mov_nets = {p['net'] for r in mov for p in D['parts'][r]['pads'] if p['net'] and not p['net'].startswith('unconnected')}
W = D['window']
def inwin(v): return W[0] <= v.x * NM <= W[2] and W[1] <= v.y * NM <= W[3]
# 1. strip copper the model assumed gone
doomed = []
for t in b.GetTracks():
    if t.GetClass() == 'PCB_TRACK':
        if t.GetLayer() == pcbnew.B_Cu and str(t.GetNetname()) in mov_nets and (inwin(t.GetStart()) or inwin(t.GetEnd())): doomed.append(t)
    elif t.GetClass() == 'PCB_VIA':
        p = t.GetPosition()
        if any(abs(v['x'] - p.x * NM) < 0.002 and abs(v['y'] - p.y * NM) < 0.002 for v in D['vias_dropped']): doomed.append(t)
# also inner-layer tracks that only connected those vias (encoder nets on In3 etc.): strip encoder-net tracks in the window on any layer
for t in b.GetTracks():
    if t.GetClass() == 'PCB_TRACK' and t.GetLayer() != pcbnew.B_Cu and re.search(r'ENC|TERM|HALL|EXT_ANALOG|_SW$', str(t.GetNetname())) \
            and (inwin(t.GetStart()) or inwin(t.GetEnd())) and t not in doomed: doomed.append(t)
for t in doomed: b.Delete(t)
# 2. place
for r, pose in rec['pose'].items():
    f = fps[r]; f.SetPosition(pcbnew.VECTOR2I(int(round(pose[0] / NM)), int(round(pose[1] / NM)))); f.SetOrientationDegrees(pose[2])
dy = rec['dy']
for r in D['sliders']:
    f = fps[r]; p = f.GetPosition(); f.SetPosition(pcbnew.VECTOR2I(p.x, int(round(p.y - dy / NM))))
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
with quiet_stderr():
    pcbnew.SaveBoard(out, b)
pro = src.replace('.kicad_pcb', '.kicad_pro')
if os.path.exists(pro): shutil.copy(pro, out.replace('.kicad_pcb', '.kicad_pro'))
print(f'wrote {out}: {len(doomed)} copper items stripped, {len(rec["pose"])} parts placed, connectors up {dy:.2f} mm')
if '--drc' in sys.argv:
    rep = out.replace('.kicad_pcb', '_drc.json')
    subprocess.run(['kicad-cli', 'pcb', 'drc', '--severity-all', '--format', 'json', '-o', rep, out], capture_output=True)
    j = json.load(open(rep)); from collections import Counter
    c = Counter(v['type'] for v in j['violations'])
    corner = [v for v in j['violations'] if any(W[0] <= it['pos']['x'] <= W[2] and W[1] <= it['pos']['y'] <= W[3] for it in v['items'])]
    cc = Counter(v['type'] for v in corner)
    print('DRC all:', dict(c)); print('DRC in corner window:', dict(cc)); print('unconnected:', len(j['unconnected_items']))
    for v in corner:
        if v['type'] in ('courtyards_overlap', 'clearance', 'hole_clearance', 'keepout', 'copper_edge_clearance', 'shorting_items'):
            print('  ', v['type'], v['description'][:110])
