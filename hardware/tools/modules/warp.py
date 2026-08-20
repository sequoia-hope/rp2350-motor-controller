#!/usr/bin/env python3
"""Apply per-leaf stretch vectors to a COPY of the board.
Footprints move rigidly with their leaf. Copper/zone/silk points move by a
blended displacement field: exact leaf vector where a point sits on a pad or
inside a leaf's courtyard territory (layer-aware; moving leaves win over pinned
connectors whose XY footprint overlaps them), inverse-distance-squared blend of
the leaf vectors in the gaps. Tracks that cross a gradient are subdivided into
polylines (0.1 mm steps, re-simplified), so rigid regions keep byte-identical
copper and only the seams stretch. Then zones are refilled and kicad DRC
referees the copy against the pre-warp baseline (position-independent diff)."""
import json, math, sys, os, shutil, collections
S = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, S)
sys.path.insert(0, '/home/sequoia/pcb/rp2350-motor-controller/hardware/tools/jiggle2')
from common import load_board, NM, quiet_stderr
import pcbnew
from geom import bbox, pip
HW = '/home/sequoia/pcb/rp2350-motor-controller/hardware'
SRC = os.environ.get('SRC', HW + '/rp2350_driver.kicad_pcb')
OUT = os.environ.get('OUT', HW + '/rp2350_driver_modules.kicad_pcb')
VEC = json.load(open(os.environ.get('VEC', S + '/vectors.json')))
M = json.load(open(S + '/modules.json')); B = json.load(open(S + '/board.json'))
mods = M['modules']; assign = M['assign']; fpj = {f['ref']: f for f in B['fps']}
V = {m: tuple(VEC['vectors'].get(m, [0, 0])) for m in mods}
SCALE = float(os.environ.get('SCALE', 1.0))
V = {m: (v[0]*SCALE, v[1]*SCALE) for m, v in V.items()}
STEP = 0.1; SIMP = 0.0015
# ---- territories: per leaf, courtyard polys per side (geo_refs only) ----------
terr = {m: {'F': [], 'B': []} for m in mods}
for m, d in mods.items():
    for r in d['geo_refs']:
        f = fpj[r]
        if f['courtyard']:
            for side, polys in f['courtyard'].items():
                for poly in polys: terr[m][side].append(([tuple(p) for p in poly], bbox(poly)))
        else:
            for p in f['pads']:
                b = p['bbox']; bb = (b[0]-0.2, b[1]-0.2, b[2]+0.2, b[3]+0.2)
                terr[m][f['layer']].append(([(bb[0], bb[1]), (bb[2], bb[1]), (bb[2], bb[3]), (bb[0], bb[3])], bb))
def rect_dist(b, x, y):
    dx = max(b[0]-x, 0, x-b[2]); dy = max(b[1]-y, 0, y-b[3]); return math.hypot(dx, dy)
def leaf_dist(m, x, y):
    best = 1e9
    for side in ('F', 'B'):
        for poly, bb in terr[m][side]:
            d = rect_dist(bb, x, y)
            if d < best: best = d
    return best
def containing(x, y, sides):
    """leaves whose courtyard territory on one of `sides` contains (x,y)."""
    out = []
    for m in mods:
        for side in sides:
            hit = False
            for poly, bb in terr[m][side]:
                if bb[0] <= x <= bb[2] and bb[1] <= y <= bb[3] and pip(x, y, poly): hit = True; break
            if hit: out.append(m); break
    return out
_cache = {}
def field(x, y, sides=('F', 'B')):
    key = (round(x, 5), round(y, 5), sides)
    if key in _cache: return _cache[key]
    c = containing(x, y, sides)
    if c:
        mv = [m for m in c if not mods[m]['pinned']] or c
        vx = sum(V[m][0] for m in mv)/len(mv); vy = sum(V[m][1] for m in mv)/len(mv)
        if len(mv) > 1: stats['multi'] += 1
        res = (vx, vy)
    else:
        ws = 0.0; vx = vy = 0.0
        ds = sorted((leaf_dist(m, x, y), m) for m in mods)
        for d, m in ds[:12]:
            if d > 8.0 and ws > 0: break
            w = 1.0/max(d, 1e-3)**2; ws += w; vx += w*V[m][0]; vy += w*V[m][1]
        res = (vx/ws, vy/ws)
    _cache[key] = res
    return res
stats = collections.Counter()
bd = load_board(SRC)
LAYSIDE = {pcbnew.F_Cu: ('F',), pcbnew.B_Cu: ('B',)}
def sides_of(layer): return LAYSIDE.get(layer, ('F', 'B'))
# ---- pad index for endpoint snapping -----------------------------------------
pads = []   # (bbox, pad, ref)
for f in bd.GetFootprints():
    for p in f.Pads():
        bb = p.GetBoundingBox(); pads.append(((bb.GetLeft()*NM, bb.GetTop()*NM, bb.GetRight()*NM, bb.GetBottom()*NM), p, f.GetReference()))
def pad_vec(x, y, layer, netcode):
    v = pcbnew.VECTOR2I(int(round(x/NM)), int(round(y/NM)))
    best = None
    for bb, p, ref in pads:
        if not (bb[0]-0.001 <= x <= bb[2]+0.001 and bb[1]-0.001 <= y <= bb[3]+0.001): continue
        if layer is not None and not p.IsOnLayer(layer): continue
        if not p.HitTest(v): continue
        same = (p.GetNetCode() == netcode)
        if best is None or (same and not best[0]): best = (same, ref)
    if best is None: return None
    return V[assign[best[1]]]
def disp(x, y, layer, netcode):
    pv = pad_vec(x, y, layer, netcode)
    if pv is not None: stats['pad'] += 1; return pv
    return field(x, y, sides_of(layer) if layer is not None else ('F', 'B'))
# ---- footprints ----------------------------------------------------------------
n_fp = 0
for f in bd.GetFootprints():
    v = V[assign[f.GetReference()]]
    if abs(v[0]) < 1e-9 and abs(v[1]) < 1e-9: continue
    p = f.GetPosition(); f.SetPosition(pcbnew.VECTOR2I(p.x + int(round(v[0]/NM)), p.y + int(round(v[1]/NM)))); n_fp += 1
# ---- tracks & vias -------------------------------------------------------------
def to_v(x, y): return pcbnew.VECTOR2I(int(round(x/NM)), int(round(y/NM)))
def simplify(pts):
    out = [pts[0]]
    for i in range(1, len(pts)-1):
        a, b, c = out[-1], pts[i], pts[i+1]
        # keep b unless it is within SIMP of segment a-c
        ax, ay = a; bx, by = b; cx, cy = c
        dx, dy = cx-ax, cy-ay; L2 = dx*dx+dy*dy
        if L2 < 1e-18: continue
        t = ((bx-ax)*dx+(by-ay)*dy)/L2
        if 0 <= t <= 1 and math.hypot(bx-ax-t*dx, by-ay-t*dy) < SIMP: continue
        out.append(b)
    out.append(pts[-1]); return out
n_rigid = n_split = n_new = n_via = 0
for t in list(bd.GetTracks()):
    if t.GetClass() == 'PCB_VIA':
        p = t.GetPosition(); x, y = p.x*NM, p.y*NM
        v = disp(x, y, None, t.GetNetCode())
        if abs(v[0]) > 1e-9 or abs(v[1]) > 1e-9:
            t.SetPosition(to_v(x+v[0], y+v[1])); n_via += 1
        continue
    s, e = t.GetStart(), t.GetEnd(); lay = t.GetLayer(); nc = t.GetNetCode()
    ax, ay, bx, by = s.x*NM, s.y*NM, e.x*NM, e.y*NM
    va = disp(ax, ay, lay, nc); vb = disp(bx, by, lay, nc)
    L = math.hypot(bx-ax, by-ay)
    if math.hypot(va[0]-vb[0], va[1]-vb[1]) < 2e-4:
        # uniform translation? check midpoints too (a track may pass through a gradient with equal ends)
        vm = field((ax+bx)/2, (ay+by)/2, sides_of(lay))
        if math.hypot(vm[0]-va[0], vm[1]-va[1]) < 2e-4 or L < 0.05:
            if abs(va[0]) > 1e-9 or abs(va[1]) > 1e-9:
                t.SetStart(to_v(ax+va[0], ay+va[1])); t.SetEnd(to_v(bx+vb[0], by+vb[1]))
            n_rigid += 1; continue
    n = max(2, int(math.ceil(L/STEP)))
    pts = []
    for k in range(n+1):
        u = k/n; x = ax+(bx-ax)*u; y = ay+(by-ay)*u
        v = va if k == 0 else (vb if k == n else field(x, y, sides_of(lay)))
        pts.append((x+v[0], y+v[1]))
    pts = simplify(pts)
    w = t.GetWidth(); net = t.GetNet(); locked = t.IsLocked()
    bd.Delete(t); n_split += 1
    for k in range(len(pts)-1):
        if math.hypot(pts[k+1][0]-pts[k][0], pts[k+1][1]-pts[k][1]) < 1e-4: continue
        nt = pcbnew.PCB_TRACK(bd); nt.SetStart(to_v(*pts[k])); nt.SetEnd(to_v(*pts[k+1])); nt.SetWidth(w); nt.SetLayer(lay); nt.SetNet(net); nt.SetLocked(locked)
        bd.Add(nt); n_new += 1
# ---- zones, silk text ------------------------------------------------------------
n_zv = 0
for z in bd.Zones():
    lays = z.GetLayerSet().Seq()
    sides = tuple(sorted({s for l in lays for s in sides_of(l)})) or ('F', 'B')
    o = z.Outline()
    for i in range(o.TotalVertices()):
        p = o.CVertex(i); x, y = p.x*NM, p.y*NM
        v = field(x, y, sides)
        if abs(v[0]) > 1e-9 or abs(v[1]) > 1e-9:
            o.SetVertex(i, to_v(x+v[0], y+v[1])); n_zv += 1
n_txt = 0
for d in bd.GetDrawings():
    if d.GetClass() == 'PCB_TEXT' and d.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS):
        p = d.GetPosition(); x, y = p.x*NM, p.y*NM
        v = field(x, y, ('F',) if d.GetLayer() == pcbnew.F_SilkS else ('B',))
        if abs(v[0]) > 1e-9 or abs(v[1]) > 1e-9: d.SetPosition(to_v(x+v[0], y+v[1])); n_txt += 1
with quiet_stderr():
    pcbnew.ZONE_FILLER(bd).Fill(bd.Zones())
    pcbnew.SaveBoard(OUT, bd)
pro = SRC.replace('.kicad_pcb', '.kicad_pro'); opro = OUT.replace('.kicad_pcb', '.kicad_pro')
if os.path.exists(pro) and os.path.abspath(pro) != os.path.abspath(opro): shutil.copy(pro, opro)
print(f'warp: {n_fp} footprints moved, tracks: {n_rigid} rigid, {n_split} split into {n_new} segments, {n_via} vias moved, '
      f'{n_zv} zone vertices, {n_txt} silk texts; endpoints snapped to pads {stats["pad"]}, multi-territory points {stats["multi"]}')
print('saved', OUT)
