#!/usr/bin/env python3
"""Module stretch, connectivity-owned copper.
Footprints move rigidly with their leaf. Copper is owned by connectivity:
track ends on a pad are pinned to that pad's leaf vector; every other
track end / via is a free node of the copper graph and takes the harmonic
(neighbour-average, length-weighted) interpolation of the pinned values —
so a trace from module A to module B stretches along itself, a bundle of
parallel A-B traces stays parallel, and copper wholly inside one module is
byte-identical. Pad-less clusters (stitching via farms, stubs into pours)
move rigidly by the spatial field at their centroid. Zones/silk follow the
spatial field. No segment is split; topology is untouched."""
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
SCALE = float(os.environ.get('SCALE', 1.0))
V = {m: (VEC['vectors'].get(m, [0, 0])[0]*SCALE, VEC['vectors'].get(m, [0, 0])[1]*SCALE) for m in mods}
# ---- spatial field (zones, silk, pad-less clusters) ------------------------------
terr = {m: [] for m in mods}
for m, d in mods.items():
    for r in d['geo_refs']:
        f = fpj[r]
        if f['courtyard']:
            for side, polys in f['courtyard'].items():
                for poly in polys: terr[m].append(([tuple(p) for p in poly], bbox(poly)))
        else:
            for p in f['pads']:
                b = p['bbox']; bb = (b[0]-0.2, b[1]-0.2, b[2]+0.2, b[3]+0.2)
                terr[m].append(([(bb[0], bb[1]), (bb[2], bb[1]), (bb[2], bb[3]), (bb[0], bb[3])], bb))
def rect_dist(b, x, y): return math.hypot(max(b[0]-x, 0, x-b[2]), max(b[1]-y, 0, y-b[3]))
def field(x, y):
    inside = [m for m in mods if any(bb[0] <= x <= bb[2] and bb[1] <= y <= bb[3] and pip(x, y, poly) for poly, bb in terr[m])]
    if inside:
        mv = [m for m in inside if not mods[m]['pinned']] or inside
        return (sum(V[m][0] for m in mv)/len(mv), sum(V[m][1] for m in mv)/len(mv))
    ds = sorted((min((rect_dist(bb, x, y) for poly, bb in terr[m]), default=1e9), m) for m in mods)
    ws = vx = vy = 0.0
    for d, m in ds[:12]:
        w = 1.0/max(d, 1e-3)**2; ws += w; vx += w*V[m][0]; vy += w*V[m][1]
    return (vx/ws, vy/ws)
bd = load_board(SRC)
def to_v(x, y): return pcbnew.VECTOR2I(int(round(x/NM)), int(round(y/NM)))
# ---- footprints ----------------------------------------------------------------
n_fp = 0
for f in bd.GetFootprints():
    v = V[assign[f.GetReference()]]
    if abs(v[0]) > 1e-9 or abs(v[1]) > 1e-9:
        p = f.GetPosition(); f.SetPosition(pcbnew.VECTOR2I(p.x + int(round(v[0]/NM)), p.y + int(round(v[1]/NM)))); n_fp += 1
# pads are now at NEW positions; build the copper graph on OLD coordinates (pad hit-test must use old pad shapes):
# simplest: test hit against pad shape translated back -> test (x - v) against the moved pad.
pads = []
for f in bd.GetFootprints():
    v = V[assign[f.GetReference()]]
    for p in f.Pads():
        bb = p.GetBoundingBox()
        pads.append(((bb.GetLeft()*NM - v[0], bb.GetTop()*NM - v[1], bb.GetRight()*NM - v[0], bb.GetBottom()*NM - v[1]), p, v))
def pad_hit(x, y, layer, netcode):
    best = None
    for bb, p, v in pads:
        if not (bb[0]-1e-4 <= x <= bb[2]+1e-4 and bb[1]-1e-4 <= y <= bb[3]+1e-4): continue
        if layer is not None and not p.IsOnLayer(layer): continue
        if p.GetNetCode() != netcode: continue
        if p.HitTest(to_v(x + v[0], y + v[1])): return v
    return None
# ---- copper graph ------------------------------------------------------------------
tracks = [t for t in bd.GetTracks() if t.GetClass() == 'PCB_TRACK']
vias = [t for t in bd.GetTracks() if t.GetClass() == 'PCB_VIA']
node_of = {}      # key -> node id
nodes = []        # dict(x, y, fixed:(vx,vy)|None, net)
adj = collections.defaultdict(list)   # node -> [(node, w)]
via_nodes = []
def new_node(x, y, net):
    nodes.append(dict(x=x, y=y, fixed=None, net=net)); return len(nodes)-1
# vias first: one node each, keyed by position (per net)
conn = bd.GetConnectivity()
pad_leafvec = {}
for f in bd.GetFootprints():
    v = V[assign[f.GetReference()]]
    for p in f.Pads(): pad_leafvec[p.m_Uuid.AsString()] = v
via_index = collections.defaultdict(list)
for vobj in vias:
    p = vobj.GetPosition(); x, y = p.x*NM, p.y*NM; nc = vobj.GetNetCode()
    nid = new_node(x, y, nc); via_nodes.append((vobj, nid))
    r = vobj.GetWidth(pcbnew.PADSTACK.ALL_LAYERS)*NM/2 if hasattr(pcbnew, 'PADSTACK') else 0.3
    via_index[(int(x//1.0), int(y//1.0))].append((x, y, r, nc, nid))
    fv = pad_hit(x, y, None, nc)
    if fv is not None: nodes[nid]['fixed'] = fv
    else:
        try:
            for cp in conn.GetConnectedPads(vobj):
                pv = pad_leafvec.get(cp.m_Uuid.AsString())
                if pv is not None: nodes[nid]['fixed'] = pv; break
        except Exception: pass
def find_via(x, y, nc):
    for cx in (int(x//1.0)-1, int(x//1.0), int(x//1.0)+1):
        for cy in (int(y//1.0)-1, int(y//1.0), int(y//1.0)+1):
            for vx, vy, r, vnc, nid in via_index.get((cx, cy), ()):
                if vnc == nc and math.hypot(vx-x, vy-y) <= r + 1e-6: return nid
    return None
conn = bd.GetConnectivity()
pad_leafvec = {}
for f in bd.GetFootprints():
    v = V[assign[f.GetReference()]]
    for p in f.Pads(): pad_leafvec[p.m_Uuid.AsString()] = v
n_overlap_pin = 0
tnodes = []   # (track, na, nb)
for t in tracks:
    s, e = t.GetStart(), t.GetEnd(); lay = t.GetLayer(); nc = t.GetNetCode()
    ends = []
    for P in (s, e):
        x, y = P.x*NM, P.y*NM
        nid = find_via(x, y, nc)
        if nid is None:
            key = (P.x, P.y, int(lay), nc)
            nid = node_of.get(key)
            if nid is None:
                nid = new_node(x, y, nc); node_of[key] = nid
                fv = pad_hit(x, y, lay, nc)
                if fv is not None: nodes[nid]['fixed'] = fv
        ends.append(nid)
    na, nb = ends
    # overlap-connected pads (end not inside the pad but the track body touches it): pin the nearer end
    try: cpads = list(conn.GetConnectedPads(t))
    except Exception: cpads = []
    for cp in cpads:
        pv = pad_leafvec.get(cp.m_Uuid.AsString())
        if pv is None: continue
        pc = cp.GetPosition(); pcx, pcy = pc.x*NM, pc.y*NM
        da = math.hypot(s.x*NM-pcx, s.y*NM-pcy); db = math.hypot(e.x*NM-pcx, e.y*NM-pcy)
        nid = na if da <= db else nb
        if nodes[nid]['fixed'] is None: nodes[nid]['fixed'] = pv; n_overlap_pin += 1
    L = math.hypot((e.x-s.x)*NM, (e.y-s.y)*NM)
    w = 1.0/max(L, 0.02)
    adj[na].append((nb, w)); adj[nb].append((na, w)); tnodes.append((t, na, nb))
# ---- components + harmonic solve -----------------------------------------------------
seen = [False]*len(nodes); comps = []
for i in range(len(nodes)):
    if seen[i]: continue
    stack = [i]; seen[i] = True; comp = []
    while stack:
        n = stack.pop(); comp.append(n)
        for m, w in adj[n]:
            if not seen[m]: seen[m] = True; stack.append(m)
    comps.append(comp)
disp = [None]*len(nodes)
n_rigid_cl = n_seam_cl = n_float_cl = 0
def crisp(x, y):
    """vector of the leaf whose territory contains (x,y), else the nearest leaf's (no blending)."""
    inside = [m for m in mods if any(bb[0] <= x <= bb[2] and bb[1] <= y <= bb[3] and pip(x, y, poly) for poly, bb in terr[m])]
    if inside:
        mv = [m for m in inside if not mods[m]['pinned']] or inside
        return V[mv[0]]
    d, m = min((min((rect_dist(bb, x, y) for poly, bb in terr[m]), default=1e9), m) for m in mods)
    return V[m]
# group pad-less clusters into farms: same net, any node within 1.2 mm of a node of the other cluster
floating = [c for c in comps if not any(nodes[n]['fixed'] is not None for n in c)]
cell = collections.defaultdict(list)
for ci, c in enumerate(floating):
    for n in c: cell[(int(nodes[n]['x']//1.2), int(nodes[n]['y']//1.2), nodes[n]['net'])].append(ci)
par = list(range(len(floating)))
def fnd(i):
    while par[i] != i: par[i] = par[par[i]]; i = par[i]
    return i
for ci, c in enumerate(floating):
    for n in c:
        x, y, nc = nodes[n]['x'], nodes[n]['y'], nodes[n]['net']
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for cj in cell.get((int(x//1.2)+dx, int(y//1.2)+dy, nc), ()):
                    if cj == ci or fnd(cj) == fnd(ci): continue
                    if any(math.hypot(nodes[m]['x']-x, nodes[m]['y']-y) <= 1.2 for m in floating[cj]): par[fnd(cj)] = fnd(ci)
farms = collections.defaultdict(list)
for ci in range(len(floating)): farms[fnd(ci)].extend(floating[ci])
for comp in farms.values():
    cx = sum(nodes[n]['x'] for n in comp)/len(comp); cy = sum(nodes[n]['y'] for n in comp)/len(comp)
    v = crisp(cx, cy)
    for n in comp: disp[n] = v
    n_float_cl += 1
for comp in comps:
    fixed = [n for n in comp if nodes[n]['fixed'] is not None]
    if not fixed: continue
    vals = {tuple(nodes[n]['fixed']) for n in fixed}
    if len(vals) == 1:
        v = next(iter(vals))
        for n in comp: disp[n] = v
        n_rigid_cl += 1; continue
    n_seam_cl += 1
    cur = {n: (nodes[n]['fixed'] if nodes[n]['fixed'] is not None else None) for n in comp}
    free = [n for n in comp if cur[n] is None]
    mx = sum(v[0] for v in vals)/len(vals); my = sum(v[1] for v in vals)/len(vals)
    for n in free: cur[n] = (mx, my)
    for it in range(3000):
        dmax = 0.0
        for n in free:
            sw = sx = sy = 0.0
            for m, w in adj[n]:
                if m in cur: sw += w; sx += w*cur[m][0]; sy += w*cur[m][1]
            if sw == 0: continue
            nv = (sx/sw, sy/sw); dmax = max(dmax, abs(nv[0]-cur[n][0]), abs(nv[1]-cur[n][1])); cur[n] = nv
        if dmax < 1e-7: break
    for n in comp: disp[n] = cur[n]
# ---- apply -------------------------------------------------------------------------
n_t = n_v = 0
for t, na, nb in tnodes:
    da, db = disp[na], disp[nb]
    if abs(da[0]) > 1e-9 or abs(da[1]) > 1e-9 or abs(db[0]) > 1e-9 or abs(db[1]) > 1e-9:
        s, e = t.GetStart(), t.GetEnd()
        t.SetStart(to_v(s.x*NM + da[0], s.y*NM + da[1])); t.SetEnd(to_v(e.x*NM + db[0], e.y*NM + db[1])); n_t += 1
for vobj, nid in via_nodes:
    d = disp[nid]
    if abs(d[0]) > 1e-9 or abs(d[1]) > 1e-9:
        p = vobj.GetPosition(); vobj.SetPosition(to_v(p.x*NM + d[0], p.y*NM + d[1])); n_v += 1
n_zv = 0
for z in bd.Zones():
    o = z.Outline()
    for i in range(o.TotalVertices()):
        p = o.CVertex(i); x, y = p.x*NM, p.y*NM; v = field(x, y)
        if abs(v[0]) > 1e-9 or abs(v[1]) > 1e-9: o.SetVertex(i, to_v(x+v[0], y+v[1])); n_zv += 1
n_txt = 0
for d in bd.GetDrawings():
    if d.GetClass() == 'PCB_TEXT' and d.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS):
        p = d.GetPosition(); x, y = p.x*NM, p.y*NM; v = field(x, y)
        if abs(v[0]) > 1e-9 or abs(v[1]) > 1e-9: d.SetPosition(to_v(x+v[0], y+v[1])); n_txt += 1
with quiet_stderr():
    pcbnew.ZONE_FILLER(bd).Fill(bd.Zones())
    pcbnew.SaveBoard(OUT, bd)
pro = SRC.replace('.kicad_pcb', '.kicad_pro'); opro = OUT.replace('.kicad_pcb', '.kicad_pro')
if os.path.exists(pro) and os.path.abspath(pro) != os.path.abspath(opro): shutil.copy(pro, opro)
print(f'warp2: {n_fp} footprints moved; copper graph {len(nodes)} nodes / {len(comps)} clusters '
      f'({n_rigid_cl} rigid, {n_seam_cl} seam, {n_float_cl} pad-less farms); {n_t} tracks, {n_v} vias moved; {n_overlap_pin} overlap-pinned ends; {n_zv} zone vertices, {n_txt} silk texts')
json.dump(dict(n_fp=n_fp, nodes=len(nodes), clusters=len(comps), rigid=n_rigid_cl, seam=n_seam_cl, floating=n_float_cl, tracks=n_t, vias=n_v), open(S+'/warp_stats.json', 'w'))
print('saved', OUT)
