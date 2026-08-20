#!/usr/bin/env python3
"""Figures for review/modules.html: (1) module map per side, leaves coloured by
parent, pinned interfaces grey; (2) stretch vectors (attempted vs DRC-neutral
survivors, arrows x20); (3) placement before/after zooms."""
import json, math, os, colorsys, sys
S = os.path.dirname(os.path.abspath(__file__))
IMG = '/home/sequoia/pcb/rp2350-motor-controller/review/img/modules'
B = json.load(open(S+'/board.json')); M = json.load(open(S+'/modules.json'))
V_try = json.load(open(S+'/vectors.json'))['vectors']; V_ok = json.load(open(S+'/vectors_clean.json'))['vectors']
fp = {f['ref']: f for f in B['fps']}; mods = M['modules']; assign = M['assign']
x0, y0, x1, y1 = B['edge']
parents = []
for m, d in mods.items():
    if d['parent'] not in parents and d['parent'] != 'PIN': parents.append(d['parent'])
PCOL = {}
for i, p in enumerate(parents):
    h = (i*0.618034) % 1.0; r, g, b = colorsys.hls_to_rgb(h, 0.55, 0.65); PCOL[p] = '#%02x%02x%02x' % (int(r*255), int(g*255), int(b*255))
def svg_header(w, h, bg='#ffffff'):
    return [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w:.0f} {h:.0f}" width="{w:.0f}" height="{h:.0f}" font-family="ui-monospace,monospace">', f'<rect width="{w:.0f}" height="{h:.0f}" fill="{bg}"/>']
def side_map(side, out, SC=14, arrows=True):
    PAD = 10; W = (x1-x0)*SC+2*PAD; H = (y1-y0)*SC+2*PAD+26
    X = lambda x: PAD+(x-x0)*SC; Y = lambda y: PAD+(y-y0)*SC
    o = svg_header(W, H)
    o.append(f'<rect x="{X(x0):.1f}" y="{Y(y0):.1f}" width="{(x1-x0)*SC:.1f}" height="{(y1-y0)*SC:.1f}" fill="#f4f2ee" stroke="#333" stroke-width="1.5"/>')
    for f in B['fps']:
        if f['layer'] != side: continue
        m = assign[f['ref']]; d = mods[m]
        col = '#9aa0a8' if d['pinned'] else PCOL[d['parent']]
        polys = f['courtyard'].get(side) or [[(f['bbox'][0], f['bbox'][1]), (f['bbox'][2], f['bbox'][1]), (f['bbox'][2], f['bbox'][3]), (f['bbox'][0], f['bbox'][3])]]
        for poly in polys:
            dd = 'M'+' L'.join(f'{X(px):.1f},{Y(py):.1f}' for px, py in poly)+' Z'
            o.append(f'<path d="{dd}" fill="{col}" fill-opacity="0.55" stroke="{col}" stroke-width="0.8"><title>{f["ref"]} · {m} ({d["parent"]})</title></path>')
    # leaf labels at leaf centroid (only leaves that have parts on this side)
    for m, d in mods.items():
        refs = [r for r in d['refs'] if fp[r]['layer'] == side]
        if not refs: continue
        cx = sum(fp[r]['x'] for r in refs)/len(refs); cy = sum(fp[r]['y'] for r in refs)/len(refs)
        if len(d['refs']) >= 4 or d['pinned']:
            o.append(f'<text x="{X(cx):.1f}" y="{Y(cy)+3:.1f}" text-anchor="middle" font-size="{9 if len(d["refs"])>=4 else 7}" font-weight="bold" fill="#111" stroke="#fff" stroke-width="2.5" paint-order="stroke">{m}</text>')
        if arrows:
            vt = V_try.get(m, [0, 0]); vo = V_ok.get(m, [0, 0])
            for v, col, dash in ((vt, '#e0a44c', '4,3'), (vo, '#2c7a4b', '')):
                if abs(v[0]) + abs(v[1]) < 1e-6: continue
                ex, ey = X(cx)+v[0]*SC*20, Y(cy)+v[1]*SC*20
                o.append(f'<line x1="{X(cx):.1f}" y1="{Y(cy):.1f}" x2="{ex:.1f}" y2="{ey:.1f}" stroke="{col}" stroke-width="2.2" stroke-dasharray="{dash}" marker-end="url(#{"arrT" if dash else "arrO"})"/>')
    # legend
    ly = H-12; lx = PAD
    for p in parents:
        o.append(f'<rect x="{lx}" y="{ly-9}" width="10" height="10" fill="{PCOL[p]}"/><text x="{lx+13}" y="{ly}" font-size="9" fill="#222">{p}</text>'); lx += 13+len(p)*6.5+10
    o.append(f'<rect x="{lx}" y="{ly-9}" width="10" height="10" fill="#9aa0a8"/><text x="{lx+13}" y="{ly}" font-size="9" fill="#222">pinned interface</text>')
    if arrows:
        lx += 130
        o.append(f'<line x1="{lx}" y1="{ly-4}" x2="{lx+22}" y2="{ly-4}" stroke="#e0a44c" stroke-width="2.2" stroke-dasharray="4,3"/><text x="{lx+26}" y="{ly}" font-size="9" fill="#222">stretch proposed (x20)</text>')
        lx += 160
        o.append(f'<line x1="{lx}" y1="{ly-4}" x2="{lx+22}" y2="{ly-4}" stroke="#2c7a4b" stroke-width="2.2"/><text x="{lx+26}" y="{ly}" font-size="9" fill="#222">survived DRC referee (x20)</text>')
    o.append('<defs><marker id="arrT" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto"><path d="M0,0 L6,3 L0,6 Z" fill="#e0a44c"/></marker><marker id="arrO" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto"><path d="M0,0 L6,3 L0,6 Z" fill="#2c7a4b"/></marker></defs></svg>')
    open(out, 'w').write('\n'.join(o))
side_map('F', IMG+'/modules_front.svg'); side_map('B', IMG+'/modules_back.svg')
# ---- placement before/after zooms (placed board vs original poses) ------------------
BP = json.load(open(os.environ.get('PLACED', S+'/board_placed.json'))); fpp = {f['ref']: f for f in BP['fps']}
res = json.load(open(S+'/jobs10_result.json'))
def zoom(side, wx0, wy0, wx1, wy1, out, SC=30, title=''):
    PAD = 10; W = (wx1-wx0)*SC+2*PAD; H = (wy1-wy0)*SC+2*PAD+18
    X = lambda x: PAD+(x-wx0)*SC; Y = lambda y: PAD+(y-wy0)*SC
    o = svg_header(W, H)
    o.append(f'<rect x="{X(max(wx0,x0)):.1f}" y="{Y(max(wy0,y0)):.1f}" width="{(min(wx1,x1)-max(wx0,x0))*SC:.1f}" height="{(min(wy1,y1)-max(wy0,y0))*SC:.1f}" fill="#f4f2ee" stroke="#333"/>')
    moved = {r for r, v in res.items() if v}
    for f in BP['fps']:
        if f['x'] < wx0-4 or f['x'] > wx1+4 or f['y'] < wy0-4 or f['y'] > wy1+4: continue
        polys = f['courtyard'].get(side) or []
        tht = any(p['drill'] for p in f['pads'])
        if f['layer'] != side and not tht: continue
        m = assign.get(f['ref'], 'PIN_'+f['ref']); d = mods.get(m)
        col = '#7c5cbf' if f['ref'] in moved else ('#9aa0a8' if (d is None or d['pinned']) else PCOL[d['parent']])
        if f['layer'] == side:
            for poly in polys:
                dd = 'M'+' L'.join(f'{X(px):.1f},{Y(py):.1f}' for px, py in poly)+' Z'
                o.append(f'<path d="{dd}" fill="{col}" fill-opacity="{0.6 if f["ref"] in moved else 0.35}" stroke="{col}" stroke-width="{1.3 if f["ref"] in moved else 0.8}"/>')
        for p in f['pads']:
            if not (p['F'] if side == 'F' else p['B']): continue
            bb = p['bbox']; o.append(f'<rect x="{X(bb[0]):.1f}" y="{Y(bb[1]):.1f}" width="{(bb[2]-bb[0])*SC:.1f}" height="{(bb[3]-bb[1])*SC:.1f}" fill="#c9a227" fill-opacity="0.7"/>')
        if f['layer'] == side or tht:
            fs = 7.5 if f['ref'] in moved else 6.5
            o.append(f'<text x="{X(f["x"]):.1f}" y="{Y(f["y"])+2.5:.1f}" text-anchor="middle" font-size="{fs}" font-weight="{"bold" if f["ref"] in moved else "normal"}" fill="{"#3b2a66" if f["ref"] in moved else "#222"}">{f["ref"]}</text>')
    # ghosts: original poses of re-placed parts (dashed red) + arrows
    for r in moved:
        f0 = fp.get(r); f1 = fpp.get(r)
        if not f0 or not f1 or f1['layer'] != side: continue
        if not (wx0-2 <= f1['x'] <= wx1+2 and wy0-2 <= f1['y'] <= wy1+2): continue
        if f0['layer'] == side and f0['courtyard'].get(side) and math.hypot(f0['x']-f1['x'], f0['y']-f1['y']) > 0.05 and (wx0-3 <= f0['x'] <= wx1+3 and wy0-3 <= f0['y'] <= wy1+3):
            for poly in f0['courtyard'][side]:
                dd = 'M'+' L'.join(f'{X(px):.1f},{Y(py):.1f}' for px, py in poly)+' Z'
                o.append(f'<path d="{dd}" fill="none" stroke="#b03434" stroke-width="1" stroke-dasharray="3,2"/>')
            o.append(f'<line x1="{X(f0["x"]):.1f}" y1="{Y(f0["y"]):.1f}" x2="{X(f1["x"]):.1f}" y2="{Y(f1["y"]):.1f}" stroke="#b03434" stroke-width="1" marker-end="url(#arr2)"/>')
    o.append(f'<text x="{PAD}" y="{H-5}" font-size="9" fill="#333">{title}</text>')
    o.append('<defs><marker id="arr2" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto"><path d="M0,0 L6,3 L0,6 Z" fill="#b03434"/></marker></defs></svg>')
    open(out, 'w').write('\n'.join(o))
zoom('B', 111.5, 89, 136, 128.4, IMG+'/place_encoder_back.svg', 28, 'Encoder corner, back side: re-placed rev-B parts in purple (red dashed = provisional pose they came from)')
zoom('B', 160, 104, 188.5, 128.4, IMG+'/place_br_back.svg', 28, 'Bottom-right, back side: F-19 R114/JP2/R115, Q6 (now SOT-23), R108')
zoom('F', 160, 104, 188.5, 128.4, IMG+'/place_br_front.svg', 28, 'Bottom-right, front side: C115 between J11 and J6')
print('wrote figures')
