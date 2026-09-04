#!/usr/bin/env python3
"""settle: inflate the seven encoder-corner parts on the result of another run.

    python3 chain_enc.py SRC_NAME DST_NAME [--radius 14]

Copies SRC's last gate-passing checkpoint to data/boards/<SRC>_final.kicad_pcb
and writes configs/<DST>.json = enc_after_wide with that board, SRC's region
and locks, and the ghosts' site search radius.
"""
import json, os, shutil, sys
from common import HERE
CFG = os.path.join(HERE, 'configs'); DATA = os.path.join(CFG, 'data')
src, dst = sys.argv[1], sys.argv[2]
radius = float(sys.argv[sys.argv.index('--radius') + 1]) if '--radius' in sys.argv else 14.0
gate = json.load(open(os.path.join(DATA, src, 'gate.json')))
ok = [e for e in gate['steps'] if e['ok']]
if not ok:
    sys.exit(f'{src} has no gate-passing checkpoint')
last = ok[-1]['step']
board = os.path.join(DATA, src, 'steps', f'step_{last:02d}.kicad_pcb')
out = os.path.join(DATA, 'boards', f'{src}_final.kicad_pcb')
shutil.copy(board, out); shutil.copy(board.replace('.kicad_pcb', '.kicad_pro'), out.replace('.kicad_pcb', '.kicad_pro'))
print(f'base: {src} step {last} (s={ok[-1]["s"]}) -> {out}')
srccfg = json.load(open(os.path.join(CFG, src + '.json')))
c = json.load(open(os.path.join(CFG, 'enc_after_wide.json')))
c['name'] = dst; c['board'] = f'data/boards/{src}_final.kicad_pcb'
c['region'] = srccfg['region']; c['locked'] = srccfg['locked']
for g in c['drive']['inflate']:
    if g['at'] == 'search': g['radius'] = radius
    elif isinstance(g['at'], dict): g['at']['search'] = radius / 2
json.dump(c, open(os.path.join(CFG, dst + '.json'), 'w'), indent=1)
print(f'wrote configs/{dst}.json (radius {radius} mm)')
