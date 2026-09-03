#!/usr/bin/env python3
"""settle: chain the encoder-corner inflation onto the squeezed power stage.

Takes the last gate-passing checkpoint board of `power_squeeze`, stores it as
data/boards/power_up.kicad_pcb (+ .kicad_pro), and writes two configs that
inflate the seven unplaced encoder parts on THAT board with the power stage
locked where the squeeze left it:

    enc_after_narrow.json   site search radius 8 mm   (the run that got 3/7 on the unsqueezed board)
    enc_after_wide.json     site search radius 14 mm  (7/7, but U30 12 mm and R100 13.5 mm from their partners)

The comparison to read afterwards: ghosts born, and each ghost's distance
from its partner centroid (prep.log "site search ... mm from partners").

    python3 after_squeeze.py          then  pipeline.py configs/enc_after_*.json
"""
import json, os, shutil, sys
from common import HERE

CFG = os.path.join(HERE, 'configs')
DATA = os.path.join(CFG, 'data')
gate = json.load(open(os.path.join(DATA, 'power_squeeze', 'gate.json')))
ok = [e for e in gate['steps'] if e['ok']]
if not ok:
    sys.exit('power_squeeze has no gate-passing checkpoint')
last = ok[-1]['step']
src = os.path.join(DATA, 'power_squeeze', 'steps', f'step_{last:02d}.kicad_pcb')
os.makedirs(os.path.join(DATA, 'boards'), exist_ok=True)
dst = os.path.join(DATA, 'boards', 'power_up.kicad_pcb')
shutil.copy(src, dst)
shutil.copy(src.replace('.kicad_pcb', '.kicad_pro'), dst.replace('.kicad_pcb', '.kicad_pro'))
print(f'base board: power_squeeze step {last} (s={ok[-1]["s"]}) -> {dst}')

base = json.load(open(os.path.join(CFG, 'enc_corner_wide.json')))
for name, radius in (('enc_after_narrow', 8), ('enc_after_wide', 14)):
    c = json.loads(json.dumps(base))
    c['name'] = name
    c['board'] = 'data/boards/power_up.kicad_pcb'
    c['region'] = [112.4, 81.9, 149.5, 128.5]
    c['locked']['rects'] = [[132.5, 81.9, 149.6, 128.6]]      # the squeezed power stage stays put
    for g in c['drive']['inflate']:
        if g['at'] == 'search':
            g['radius'] = radius
        elif isinstance(g['at'], dict):
            g['at']['search'] = radius / 2
    json.dump(c, open(os.path.join(CFG, name + '.json'), 'w'), indent=1)
    print(f'wrote configs/{name}.json (radius {radius} mm)')
