#!/usr/bin/env python3
"""Manual-repair loop check for a rubber-stretched board.

Runs DRC, diffs position-independently against the pre-stretch baseline
(auto-built from hardware/rp2350_driver.kicad_pcb on first use), clusters the
remaining new violations into physical sites, prints them worst-first, checks
airwires and schematic sync.  Run it after every hand-editing session:

    python3 hardware/tools/rubber/status.py [--board PATH]

exit 0 = no stretch-caused violations remain (padstack artifact ignored)."""
import argparse, collections, json, math, os, re, shutil, subprocess, sys
S = os.path.dirname(os.path.abspath(__file__)); HW = os.path.dirname(os.path.dirname(S))
sys.path.insert(0, os.path.join(HW, 'tools', 'jiggle2'))
sys.path.insert(0, os.path.join(HW, 'tools', 'modules'))
from common import load_board, quiet_stderr
from drcdiff import run_drc, diff, norm

ap = argparse.ArgumentParser()
ap.add_argument('--board', default=os.path.join(HW, 'rp2350_driver_rubber.kicad_pcb'))
ap.add_argument('--json', default=os.path.join(HW, '..', 'review', 'rubber_status.json'))
args = ap.parse_args()
DATA = os.path.join(S, 'data'); os.makedirs(DATA, exist_ok=True)
BASE = os.path.join(DATA, 'base_rt_drc.json')
if not os.path.exists(BASE):
    print('building pre-stretch DRC baseline...')
    import pcbnew
    bd = load_board(os.path.join(HW, 'rp2350_driver.kicad_pcb'))
    rt = os.path.join(DATA, 'base_rt.kicad_pcb')
    with quiet_stderr():
        pcbnew.SaveBoard(rt, bd)
    shutil.copy(os.path.join(HW, 'rp2350_driver.kicad_pro'), rt.replace('.kicad_pcb', '.kicad_pro'))
    run_drc(rt, BASE)
base = json.load(open(BASE))
cur = run_drc(args.board, os.path.join(DATA, 'status_drc.json'))
new, gone = diff(base, cur)
art = [v for v in new if v['type'] == 'padstack']
new = [v for v in new if v['type'] != 'padstack']

DEFICIT = re.compile(r'clearance ([\d.]+) mm; actual ([\d.]+) mm')
def pair_sig(v):
    return (v['type'], tuple(sorted(norm(i['description']) for i in v['items'])))
insts = collections.defaultdict(list)
for v in new:
    p = v['items'][0].get('pos', {})
    insts[pair_sig(v)].append((p.get('x', 0), p.get('y', 0), v))
sites = []
for sig, pts in insts.items():
    used = [False]*len(pts)
    for i in range(len(pts)):
        if used[i]: continue
        cl = [pts[i]]; used[i] = True
        for j in range(i+1, len(pts)):
            if used[j]: continue
            if any(math.hypot(pts[j][0]-c[0], pts[j][1]-c[1]) < 0.4 for c in cl):
                cl.append(pts[j]); used[j] = True
        d = 0.0
        for c in cl:
            m = DEFICIT.search(c[2]['description'])
            if m: d = max(d, float(m.group(1)) - float(m.group(2)))
        sites.append(dict(x=sum(c[0] for c in cl)/len(cl), y=sum(c[1] for c in cl)/len(cl),
                          type=cl[0][2]['type'], deficit=d,
                          items=[i['description'] for i in cl[0][2]['items']]))
sites.sort(key=lambda s: -s['deficit'])
nb = len(base['unconnected_items']); nc = len(cur['unconnected_items'])
print(f'board: {args.board}')
print(f'stretch-caused violations: {len(new)} in {len(sites)} sites '
      f'({len(art)} ignored U8 padstack artifacts)')
print(f'airwires: {nc} (baseline {nb})' + ('  << REGRESSION' if nc > nb else ''))
for s in sites[:40]:
    print(f"  {s['deficit']*1000:4.0f}um  ({s['x']:7.2f},{s['y']:7.2f})  {s['type']:22s} "
          + ' | '.join(i[:52] for i in s['items']))
if len(sites) > 40: print(f'  ... and {len(sites)-40} more (see JSON)')
r = subprocess.run([sys.executable, os.path.join(HW, 'tools', 'revb', 'check_sync.py'),
                    '--pcb', args.board], capture_output=True, text=True)
sync = r.stdout.strip().splitlines()[-1] if r.stdout.strip() else r.stderr.strip().splitlines()[-1]
print('check_sync:', sync)
try:
    json.dump(dict(sites=sites, n_new=len(new), airwires=nc, airwires_base=nb, sync=sync),
              open(os.path.abspath(args.json), 'w'), indent=1)
except OSError: pass
sys.exit(0 if not new and nc <= nb else 1)
