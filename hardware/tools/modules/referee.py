#!/usr/bin/env python3
"""DRC referee loop for the module stretch: warp -> DRC -> diff vs baseline ->
blame new violations on moved leaves -> zero the worst leaf -> repeat until the
warp is DRC-neutral. Writes vectors_clean.json + referee_log.json."""
import json, math, os, sys, subprocess, collections, copy
S = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, S)
from geom import bbox, pip
from drcdiff import run_drc, diff, norm
B = json.load(open(S+'/board.json')); M = json.load(open(S+'/modules.json')); mods = M['modules']
fpj = {f['ref']: f for f in B['fps']}
base = json.load(open(os.environ.get('BASE', '/tmp/claude-1000/-home-sequoia-pcb-rp2350-motor-controller/1cab53f8-9d3c-4719-b885-b225fee59d3d/scratchpad/drc/baseline.json')))
vec = json.load(open(os.environ.get('VEC', S+'/vectors.json')))
OUT = os.environ.get('OUT', '/home/sequoia/pcb/rp2350-motor-controller/hardware/rp2350_driver_modules.kicad_pcb')
SCALE = os.environ.get('SCALE', '1.0')
terr = {m: [] for m in mods}
for m, d in mods.items():
    for r in d['geo_refs']:
        f = fpj[r]
        for side, polys in f['courtyard'].items():
            for poly in polys: terr[m].append(([tuple(p) for p in poly], bbox(poly)))
        if not f['courtyard']:
            for p in f['pads']:
                b = p['bbox']; bb = (b[0]-0.2, b[1]-0.2, b[2]+0.2, b[3]+0.2); terr[m].append(([(bb[0], bb[1]), (bb[2], bb[1]), (bb[2], bb[3]), (bb[0], bb[3])], bb))
def rect_dist(b, x, y): return math.hypot(max(b[0]-x, 0, x-b[2]), max(b[1]-y, 0, y-b[3]))
ref_leaf = M['assign']
import re
def blame(v, moved):
    """set of moved leaves implicated by a violation (by item refs, else by proximity)."""
    out = set()
    for it in v['items']:
        m = re.search(r' of ([A-Z]+\d+) ', it['description'] + ' ') or re.search(r'Footprint ([A-Z]+\d+)', it['description'])
        if m and m.group(1) in ref_leaf and ref_leaf[m.group(1)] in moved: out.add(ref_leaf[m.group(1)])
    if out: return out
    for it in v['items']:
        if 'pos' not in it: continue
        x, y = it['pos']['x'], it['pos']['y']
        near = sorted((min((rect_dist(bb, x, y) for poly, bb in terr[m]), default=9), m) for m in moved)
        if near and near[0][0] < 2.0: out.add(near[0][1])
    return out
log = []
for rnd in range(40):
    json.dump(vec, open(S+'/vectors_cur.json', 'w'))
    env = dict(os.environ, VEC=S+'/vectors_cur.json', OUT=OUT, SCALE=SCALE)
    r = subprocess.run([sys.executable, S+'/warp2.py'], env=env, capture_output=True, text=True)
    if r.returncode != 0: print(r.stderr[-800:]); break
    cur = run_drc(OUT, S+f'/drc_ref_{rnd}.json')
    new, gone = diff(base, cur)
    moved = [m for m, v in vec['vectors'].items() if abs(v[0]) > 1e-6 or abs(v[1]) > 1e-6]
    cnt = collections.Counter(); unattributed = 0
    for v in new:
        b = blame(v, moved)
        if not b: unattributed += 1
        for m in b: cnt[m] += 1
    entry = dict(round=rnd, moved=len(moved), new=len(new), gone=len(gone), unconnected=len(cur['unconnected_items']), blame=dict(cnt), unattributed=unattributed)
    log.append(entry)
    print(f"round {rnd}: {len(moved)} leaves moving, {len(new)} new violations ({unattributed} unattributed), {len(gone)} gone, unconnected {len(cur['unconnected_items'])}; blame {cnt.most_common(6)}")
    if not new or not cnt: break
    worst = cnt.most_common(1)[0][0]
    print(f'   -> revert {worst} {vec["vectors"][worst]}')
    vec['vectors'][worst] = [0.0, 0.0]
json.dump(vec, open(S+'/vectors_clean.json', 'w'), indent=1)
json.dump(log, open(S+'/referee_log.json', 'w'), indent=1)
print('surviving moves:', {m: v for m, v in vec['vectors'].items() if abs(v[0]) > 1e-6 or abs(v[1]) > 1e-6})
