#!/usr/bin/env python3
"""Spatial analysis of rp2350_driver.kicad_pcb: courtyard clearances, density, stats."""
import re, math, json, sys
from collections import Counter, defaultdict

PCB = '/home/sequoia/pcb/rp2350-motor-controller/hardware/rp2350_driver.kicad_pcb'
s = open(PCB).read()

# ---------- helpers ----------
def find_blocks(text, name):
    """Yield top-of-file-level s-expr blocks '(name ...)' with balanced parens."""
    out = []
    i = 0
    pat = '(' + name
    while True:
        i = text.find(pat, i)
        if i < 0:
            break
        # ensure token boundary
        after = text[i + len(pat)]
        if after not in ' \n\t(':
            i += 1
            continue
        depth = 0
        j = i
        while j < len(text):
            if text[j] == '(':
                depth += 1
            elif text[j] == ')':
                depth -= 1
                if depth == 0:
                    break
            j += 1
        out.append(text[i:j + 1])
        i = j
    return out

# ---------- board outline ----------
edge_pts = []
for b in find_blocks(s, 'gr_line') + find_blocks(s, 'gr_arc') + find_blocks(s, 'gr_rect'):
    if '(layer "Edge.Cuts")' in b:
        for m in re.finditer(r'\((?:start|end|mid) ([\d.-]+) ([\d.-]+)\)', b):
            edge_pts.append((float(m.group(1)), float(m.group(2))))
xs = [p[0] for p in edge_pts]; ys = [p[1] for p in edge_pts]
BX0, BX1, BY0, BY1 = min(xs), max(xs), min(ys), max(ys)
print(f'BOARD bbox: x {BX0:.2f}..{BX1:.2f} ({BX1-BX0:.2f} mm)  y {BY0:.2f}..{BY1:.2f} ({BY1-BY0:.2f} mm)')

# ---------- stackup ----------
m = re.search(r'\(layers\n(.*?)\n\t\)', s, re.S)
cu = re.findall(r'\(\d+ "([^"]+\.Cu)" (?:signal|power|mixed)', m.group(1)) if m else []
print('copper layers:', cu)

# ---------- footprints ----------
fps = find_blocks(s, 'footprint')
print('footprints:', len(fps))

def rot(px, py, ang):
    a = math.radians(ang)
    return (px * math.cos(a) - py * math.sin(a), px * math.sin(a) + py * math.cos(a))

parts = {}
for f in fps:
    mref = re.search(r'\(property "Reference" "([^"]+)"', f)
    mat = re.search(r'\(at ([\d.-]+) ([\d.-]+)(?: ([\d.-]+))?\)', f)
    mlay = re.search(r'\(layer "([FB])\.Cu"\)', f)
    if not (mref and mat):
        continue
    ref = mref.group(1)
    fx, fy = float(mat.group(1)), float(mat.group(2))
    fr = float(mat.group(3) or 0)
    side = mlay.group(1) if mlay else 'F'
    # collect courtyard pts (local coords)
    pts = []
    layname = f'{side}.CrtYd'
    for seg in re.finditer(r'\(fp_(?:line|rect|arc|circle|poly)[^)]*?\(start ([\d.-]+) ([\d.-]+)\)(?:[^)]*?\(mid ([\d.-]+) ([\d.-]+)\))?[^)]*?\(end ([\d.-]+) ([\d.-]+)\)\s*(?:\(stroke[^)]*\)\))?\s*\(layer "' + re.escape(layname) + '"\)', f):
        g = seg.groups()
        pts.append((float(g[0]), float(g[1])))
        pts.append((float(g[4]), float(g[5])))
        if g[2]:
            pts.append((float(g[2]), float(g[3])))
    # also grab fp_poly pts blocks on courtyard
    for pol in find_blocks(f, 'fp_poly'):
        if f'(layer "{layname}")' in pol:
            for m2 in re.finditer(r'\(xy ([\d.-]+) ([\d.-]+)\)', pol):
                pts.append((float(m2.group(1)), float(m2.group(2))))
    if not pts:
        # fallback: pad extents
        for pad in find_blocks(f, 'pad'):
            mp = re.search(r'\(at ([\d.-]+) ([\d.-]+)(?: [\d.-]+)?\)', pad)
            ms = re.search(r'\(size ([\d.-]+) ([\d.-]+)\)', pad)
            if mp and ms:
                px, py = float(mp.group(1)), float(mp.group(2))
                w, h = float(ms.group(1)) / 2, float(ms.group(2)) / 2
                for dx in (-w, w):
                    for dy in (-h, h):
                        pts.append((px + dx, py + dy))
    if not pts:
        continue
    wpts = []
    for (px, py) in pts:
        rx, ry = rot(px, py, fr)
        wpts.append((fx + rx, fy + ry))
    x0 = min(p[0] for p in wpts); x1 = max(p[0] for p in wpts)
    y0 = min(p[1] for p in wpts); y1 = max(p[1] for p in wpts)
    parts[ref] = dict(x=fx, y=fy, rot=fr, side=side, bbox=(x0, y0, x1, y1))

fcount = Counter(p['side'] for p in parts.values())
print('per side:', dict(fcount))

# ---------- courtyard-gap nearest pairs (same side) ----------
def gap(b1, b2):
    gx = max(0.0, max(b2[0] - b1[2], b1[0] - b2[2]))
    gy = max(0.0, max(b2[1] - b1[3], b1[1] - b2[3]))
    if gx == 0 and gy == 0:
        # overlap penetration (negative)
        ox = min(b1[2], b2[2]) - max(b1[0], b2[0])
        oy = min(b1[3], b2[3]) - max(b1[1], b2[1])
        return -min(ox, oy)
    return math.hypot(gx, gy)

refs = sorted(parts)
pairs = []
for i in range(len(refs)):
    for j in range(i + 1, len(refs)):
        a, b = refs[i], refs[j]
        if parts[a]['side'] != parts[b]['side']:
            continue
        g = gap(parts[a]['bbox'], parts[b]['bbox'])
        if g < 0.5:
            pairs.append((g, a, b))
pairs.sort()
print('\nTightest courtyard pairs (gap < 0.5 mm, same side; negative = courtyards overlap):')
for g, a, b in pairs[:40]:
    pa, pb = parts[a], parts[b]
    print(f'  {g:7.3f} mm  {a:6s} {b:6s}  @({(pa["x"]+pb["x"])/2:6.1f},{(pa["y"]+pb["y"])/2:6.1f}) {pa["side"]}')
print(f'total pairs <0.5mm: {len(pairs)}, of which overlapping: {sum(1 for g,_,_ in pairs if g<0)}')

# ---------- nearest neighbor per part: median breathing room ----------
nn = {}
for i, a in enumerate(refs):
    best = 99
    for b in refs:
        if a == b or parts[a]['side'] != parts[b]['side']:
            continue
        g = gap(parts[a]['bbox'], parts[b]['bbox'])
        best = min(best, g)
    nn[a] = best
vals = sorted(nn.values())
print(f'\nnearest-neighbor gap: min {vals[0]:.3f}  p25 {vals[len(vals)//4]:.3f}  median {vals[len(vals)//2]:.3f}  p75 {vals[3*len(vals)//4]:.3f} mm')

# ---------- tracks & vias ----------
tracks = find_blocks(s, 'segment')
widths = Counter()
tlen = 0.0
seg_by_layer = Counter()
for t in tracks:
    mw = re.search(r'\(width ([\d.]+)\)', t)
    ml = re.search(r'\(layer "([^"]+)"\)', t)
    ms_ = re.search(r'\(start ([\d.-]+) ([\d.-]+)\)', t)
    me = re.search(r'\(end ([\d.-]+) ([\d.-]+)\)', t)
    if mw:
        widths[float(mw.group(1))] += 1
    if ml:
        seg_by_layer[ml.group(1)] += 1
    if ms_ and me:
        tlen += math.hypot(float(me.group(1)) - float(ms_.group(1)), float(me.group(2)) - float(ms_.group(2)))
vias = find_blocks(s, 'via')
via_sizes = Counter()
for v in vias:
    mv = re.search(r'\(size ([\d.]+)\).*?\(drill ([\d.]+)\)', v, re.S)
    if mv:
        via_sizes[(float(mv.group(1)), float(mv.group(2)))] += 1
print(f'\ntracks: {len(tracks)} segments, {tlen/1000:.2f} m total')
print('by layer:', dict(seg_by_layer))
print('widths mm:', {k: v for k, v in sorted(widths.items())})
print(f'vias: {len(vias)}', dict(via_sizes))
zones = find_blocks(s, 'zone')
print('zones:', len(zones))

# ---------- key part positions (for congestion crops / fixed constraints) ----------
keys = ['U8','U18','U19','U20','U21','U22','U23','U24','U25','U26','U27','R82','R83','R84','R85',
        'J1','J2','J3','J4','J5','J6','J7','J8','J9','J10','J11','J12','H1','H2','H3','H4',
        'U1','U6','U7','U12','AH1','AL1','BH1','BL1','CH1','CL1','CH2','CL2','U28','U16','U101',
        'R86','R87','R88','R89','R90','R91','R92','R93','C39','C42','C80','U5','U9','U10']
print('\nKey positions:')
for k in keys:
    if k in parts:
        p = parts[k]
        print(f'  {k:5s} ({p["x"]:7.2f},{p["y"]:7.2f}) rot {p["rot"]:5.1f} side {p["side"]} bbox {p["bbox"][0]:.1f},{p["bbox"][1]:.1f} .. {p["bbox"][2]:.1f},{p["bbox"][3]:.1f}')

# ---------- density grid (courtyard coverage %, 5mm cells) ----------
CELL = 5.0
nx = int((BX1 - BX0) / CELL) + 1
ny = int((BY1 - BY0) / CELL) + 1
grid = defaultdict(float)
for ref, p in parts.items():
    x0, y0, x1, y1 = p['bbox']
    for ci in range(int((x0 - BX0) / CELL), int((x1 - BX0) / CELL) + 1):
        for cj in range(int((y0 - BY0) / CELL), int((y1 - BY0) / CELL) + 1):
            cx0, cy0 = BX0 + ci * CELL, BY0 + cj * CELL
            ox = max(0, min(x1, cx0 + CELL) - max(x0, cx0))
            oy = max(0, min(y1, cy0 + CELL) - max(y0, cy0))
            grid[(ci, cj)] += ox * oy / (CELL * CELL)
dense = sorted(grid.items(), key=lambda kv: -kv[1])[:12]
print('\nDensest 5mm cells (courtyard coverage, both sides summed):')
for (ci, cj), cov in dense:
    cx, cy = BX0 + ci * CELL + CELL / 2, BY0 + cj * CELL + CELL / 2
    near = [r for r, p in parts.items() if abs(p['x'] - cx) < 4 and abs(p['y'] - cy) < 4][:6]
    print(f'  ({cx:6.1f},{cy:6.1f}) {cov*100:5.0f}%  near: {" ".join(near)}')

json.dump({r: p for r, p in parts.items()}, open('parts_pos.json', 'w'), default=list)
print('\nwrote parts_pos.json')
