#!/usr/bin/env python3
"""Freerouting feasibility probe: lock all existing copper, export DSN, route
headless, import SES into a copy, report DRC delta + unconnected count."""
import sys, os, json, subprocess, time, shutil
S = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, S)
sys.path.insert(0, '/home/sequoia/pcb/rp2350-motor-controller/hardware/tools/jiggle2')
from common import load_board, NM, quiet_stderr
import pcbnew
from drcdiff import run_drc, diff, summary
SRC = sys.argv[1]; TAG = sys.argv[2]; PASSES = sys.argv[3] if len(sys.argv) > 3 else '12'
FR = os.path.dirname(S) + '/fr'
JAR = '/home/sequoia/Software/magnet/route/freerouting-2.2.4.jar'
work = f'{FR}/{TAG}.kicad_pcb'; shutil.copy(SRC, work); shutil.copy(SRC.replace('.kicad_pcb', '.kicad_pro'), work.replace('.kicad_pcb', '.kicad_pro'))
bd = load_board(work)
n = 0
for t in bd.GetTracks(): t.SetLocked(True); n += 1
print(f'locked {n} tracks/vias')
dsn = f'{FR}/{TAG}.dsn'; ses = f'{FR}/{TAG}.ses'
ok = False
for call in ((bd, dsn), (dsn,)):
    try:
        with quiet_stderr(): r = pcbnew.ExportSpecctraDSN(*call)
        if os.path.exists(dsn) and os.path.getsize(dsn) > 1000: ok = True; print('DSN exported via', len(call), 'arg form, size', os.path.getsize(dsn)); break
    except Exception as e: print('export form', len(call), 'failed:', e)
if not ok: sys.exit('DSN export failed')
t0 = time.time()
cmd = ['java', '-Xss512m', '-jar', JAR, '-de', dsn, '-do', ses, '-mp', PASSES, '-dr', '0']
print(' '.join(cmd), flush=True)
env = dict(os.environ, FREEROUTING__GUI__ENABLED='false', FREEROUTING__ROUTER__MAX_PASSES=PASSES)
try:
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=int(os.environ.get('FR_TIMEOUT', '2400')), env=env)
    print('freerouting rc', r.returncode, f'{time.time()-t0:.0f}s'); print(r.stdout[-1500:]); print(r.stderr[-800:])
except subprocess.TimeoutExpired:
    print('freerouting TIMEOUT after', time.time()-t0)
if not os.path.exists(ses): sys.exit('no SES produced')
print('SES size', os.path.getsize(ses))
bd = load_board(work)
for call in ((bd, ses), (ses,)):
    try:
        with quiet_stderr(): r = pcbnew.ImportSpecctraSES(*call)
        print('SES import via', len(call), 'arg form ->', r); break
    except Exception as e: print('import form', len(call), 'failed:', e)
with quiet_stderr():
    pcbnew.ZONE_FILLER(bd).Fill(bd.Zones()); pcbnew.SaveBoard(work, bd)
bd2 = load_board(work)
print('unconnected after import:', bd2.GetConnectivity().GetUnconnectedCount(True))
base = json.load(open(os.path.dirname(S) + '/drc/baseline.json'))
cur = run_drc(work, f'{FR}/{TAG}_drc.json')
new, gone = diff(base, cur)
print('baseline:', summary(base)); print('routed:  ', summary(cur)); print(f'new {len(new)} gone {len(gone)}')
import collections
print(collections.Counter(v['type'] for v in new))
