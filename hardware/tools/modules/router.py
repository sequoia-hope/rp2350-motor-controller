#!/usr/bin/env python3
"""In-house multi-layer grid router for the remaining airwires.
Grid 0.1 mm on F.Cu/In2/In3/B.Cu (In1/In4 are the GND planes and stay
untouched). Per net: passable cells = distance to other-net copper (EDT on a
per-layer copper raster) >= w/2 + clearance (+ sampling margin), own pads are
always enterable; via sites need that on every copper layer plus hole-to-hole.
Each DRC-unconnected pair is routed cluster-to-cluster by A* (octile moves,
via cost, penalty inside other-net pours, EDT-to-target heuristic); routed
copper joins the rasters immediately. Plane nets (zone on a non-routed layer)
finish at the first legal via site. Then kicad DRC referees; offenders are
ripped. Rounds repeat until no progress."""
import sys, os, math, json, heapq, collections, time, re
import numpy as np
from scipy import ndimage
S = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, S)
sys.path.insert(0, '/home/sequoia/pcb/rp2350-motor-controller/hardware/tools/jiggle2')
from common import load_board, NM, quiet_stderr
import pcbnew
from drcdiff import run_drc, diff, summary, norm
SRC, OUT = sys.argv[1], sys.argv[2]; ROUNDS = int(sys.argv[3]) if len(sys.argv) > 3 else 3
import shutil; shutil.copy(SRC, OUT); shutil.copy(SRC.replace('.kicad_pcb', '.kicad_pro'), OUT.replace('.kicad_pcb', '.kicad_pro'))
G = 0.1; MARG = 0.02; DIL = 0.0; SAMP = 0.075; CLR0 = 0.18; EDGE = 0.3; H2H = 0.254; VIA_D, VIA_DR = 0.5, 0.25; VIA_COST = 1.6; ZONE_PEN = 6.0
HARD_ZONE_NETS = {'VMOT','/A_P','/B_P','/C_P','/A_N','/B_N','/C_N','/D','/VREG_LX','Net-(D4-K)'}
WIDE = {'VBUS': 0.8, 'Net-(JP1-A)': 0.8, 'Vdrive': 0.3, '+3V3': 0.25}
bd = load_board(OUT)
ROUTE_L = [pcbnew.F_Cu, pcbnew.In2_Cu, pcbnew.In3_Cu, pcbnew.B_Cu]
ALL_CU = [int(l) for l in bd.GetEnabledLayers().CuStack()]
LN = {int(l): bd.GetLayerName(l) for l in ALL_CU}
bb = bd.GetBoardEdgesBoundingBox(); X0, Y0 = bb.GetLeft()*NM-1.0, bb.GetTop()*NM-1.0; X1, Y1 = bb.GetRight()*NM+1.0, bb.GetBottom()*NM+1.0
W = int((X1-X0)/G)+1; H = int((Y1-Y0)/G)+1
def cx(i): return X0 + i*G
def cy(j): return Y0 + j*G
def ci(x): return int(round((x-X0)/G))
def cj(y): return int(round((y-Y0)/G))
ns = bd.GetDesignSettings().m_NetSettings
def net_rule(name):
    try: c = ns.GetEffectiveNetClass(name); return max(c.GetClearance()*NM, CLR0), c.GetTrackWidth()*NM
    except Exception: return CLR0, 0.2
_ncl = {}
def net_clr(nc):
    if nc not in _ncl:
        n = bd.FindNet(nc); _ncl[nc] = net_rule(n.GetNetname())[0] if n else CLR0
    return _ncl[nc]
def extra(nc, lc=0.0):
    """extra inflation for an obstacle of net nc: its class clearance above the default, or its local clearance above the default."""
    return max(net_clr(nc) - CLR0, lc - CLR0, 0.0)
# ---------------- rasters ----------------------------------------------------------
copper = {L: np.zeros((H, W), bool) for L in ALL_CU}     # all copper, by layer
ownr = {}                                                # (net, L) -> raster of that net's copper on L (built lazily from items)
items = collections.defaultdict(list)                    # L -> [(net, kind, geom)]
holes = np.zeros((H, W), bool); hole_r = 0.0
def stamp_disc(arr, x, y, r, val=True):
    i0, i1 = max(0, ci(x-r)-1), min(W-1, ci(x+r)+1); j0, j1 = max(0, cj(y-r)-1), min(H-1, cj(y+r)+1)
    if i1 < i0 or j1 < j0: return
    xs = X0 + np.arange(i0, i1+1)*G; ys = Y0 + np.arange(j0, j1+1)*G
    m = (xs[None, :]-x)**2 + (ys[:, None]-y)**2 <= r*r
    arr[j0:j1+1, i0:i1+1] |= m if val else False
def stamp_seg(arr, ax, ay, bx, by, r):
    i0, i1 = max(0, ci(min(ax, bx)-r)-1), min(W-1, ci(max(ax, bx)+r)+1); j0, j1 = max(0, cj(min(ay, by)-r)-1), min(H-1, cj(max(ay, by)+r)+1)
    if i1 < i0 or j1 < j0: return
    xs = X0 + np.arange(i0, i1+1)*G; ys = Y0 + np.arange(j0, j1+1)*G
    px, py = np.meshgrid(xs, ys); dx, dy = bx-ax, by-ay; L2 = dx*dx+dy*dy
    t = np.clip(((px-ax)*dx+(py-ay)*dy)/L2, 0, 1) if L2 > 1e-18 else np.zeros_like(px)
    d2 = (px-ax-t*dx)**2 + (py-ay-t*dy)**2
    arr[j0:j1+1, i0:i1+1] |= d2 <= r*r
def stamp_poly(arr, pts, r=0.0):
    xs_ = [p[0] for p in pts]; ys_ = [p[1] for p in pts]
    i0, i1 = max(0, ci(min(xs_)-r)-1), min(W-1, ci(max(xs_)+r)+1); j0, j1 = max(0, cj(min(ys_)-r)-1), min(H-1, cj(max(ys_)+r)+1)
    if i1 < i0 or j1 < j0: return
    xs = X0 + np.arange(i0, i1+1)*G; ys = Y0 + np.arange(j0, j1+1)*G
    px, py = np.meshgrid(xs, ys); inside = np.zeros(px.shape, bool)
    n = len(pts); j = n-1
    for i in range(n):
        xi, yi = pts[i]; xj, yj = pts[j]
        cond = ((yi > py) != (yj > py)) & (px < (xj-xi)*(py-yi)/((yj-yi) if yj != yi else 1e-12) + xi)
        inside ^= cond; j = i
    if r > 0:
        for i in range(n):
            a, b = pts[i], pts[(i+1) % n]
            dx, dy = b[0]-a[0], b[1]-a[1]; L2 = dx*dx+dy*dy
            t = np.clip(((px-a[0])*dx+(py-a[1])*dy)/L2, 0, 1) if L2 > 1e-18 else np.zeros_like(px)
            inside |= (px-a[0]-t*dx)**2 + (py-a[1]-t*dy)**2 <= r*r
    arr[j0:j1+1, i0:i1+1] |= inside
def pad_poly(p, layer=None):
    try:
        ps = pcbnew.SHAPE_POLY_SET()
        L = layer if layer is not None else (pcbnew.B_Cu if p.IsOnLayer(pcbnew.B_Cu) else pcbnew.F_Cu)
        p.TransformShapeToPolygon(ps, L, 0, 5000, pcbnew.ERROR_OUTSIDE)
        if ps.OutlineCount():
            o = ps.Outline(0); return [(o.CPoint(k).x*NM, o.CPoint(k).y*NM) for k in range(o.PointCount())]
    except Exception: pass
    b = p.GetBoundingBox(); return [(b.GetLeft()*NM, b.GetTop()*NM), (b.GetRight()*NM, b.GetTop()*NM), (b.GetRight()*NM, b.GetBottom()*NM), (b.GetLeft()*NM, b.GetBottom()*NM)]
def build_rasters():
    global copper, ownr, items, holes, keep, zone_fill, inside, edge_dist, pad_list, _cache
    copper = {L: np.zeros((H, W), bool) for L in ALL_CU}; ownr = {}; items = collections.defaultdict(list); holes = np.zeros((H, W), bool); _cache = {}
    print('building rasters...', flush=True); t0 = time.time()
    pad_list = []   # (netcode, ref, num, layers, poly, lc)
    for f in bd.GetFootprints():
        try: flc = (f.GetLocalClearance() or 0)*NM
        except TypeError: flc = 0.0
        for p in f.Pads():
            poly = pad_poly(p); lays = [L for L in ALL_CU if p.IsOnLayer(L)]
            try: lc = (p.GetLocalClearance() or 0)*NM
            except TypeError: lc = 0.0
            lc = max(lc, flc)
            pad_list.append((p.GetNetCode(), f.GetReference(), p.GetNumber(), lays, poly, lc, p))
            for L in lays:
                stamp_poly(copper[L], poly, extra(p.GetNetCode(), lc))   # class/local clearance above default folded in as extra copper
                items[L].append((p.GetNetCode(), 'poly', poly, lc))
            if p.GetDrillSize().x:
                r = max(p.GetDrillSize().x, p.GetDrillSize().y)*NM/2; stamp_disc(holes, p.GetPosition().x*NM, p.GetPosition().y*NM, r)
    for t in bd.GetTracks():
        if t.GetClass() == 'PCB_VIA':
            p = t.GetPosition(); x, y = p.x*NM, p.y*NM; r = t.GetWidth(pcbnew.PADSTACK.ALL_LAYERS)*NM/2
            for L in ALL_CU: stamp_disc(copper[L], x, y, r+extra(t.GetNetCode())); items[L].append((t.GetNetCode(), 'disc', (x, y, r), 0.0))
            stamp_disc(holes, x, y, t.GetDrill()*NM/2)
        else:
            s, e = t.GetStart(), t.GetEnd(); L = int(t.GetLayer()); r = t.GetWidth()*NM/2
            stamp_seg(copper[L], s.x*NM, s.y*NM, e.x*NM, e.y*NM, r+extra(t.GetNetCode())); items[L].append((t.GetNetCode(), 'seg', (s.x*NM, s.y*NM, e.x*NM, e.y*NM, r), 0.0))
    # rule areas (keepouts): hard obstacles for tracks/vias on their layers
    keep = {L: np.zeros((H, W), bool) for L in ALL_CU}
    zone_fill = {}   # (netcode, L) -> raster of filled pour (other-net: penalty; same-net: target)
    for z in bd.Zones():
        lays = [int(l) for l in z.GetLayerSet().Seq() if int(l) in ALL_CU]
        if z.GetIsRuleArea():
            if not (z.GetDoNotAllowTracks() or z.GetDoNotAllowVias()): continue
            o = z.Outline()
            for k in range(o.OutlineCount()):
                ol = o.Outline(k); pts = [(ol.CPoint(m).x*NM, ol.CPoint(m).y*NM) for m in range(ol.PointCount())]
                for L in lays: stamp_poly(keep[L], pts, 0.0)
            continue
        for L in lays:
            try: fp_ = z.GetFilledPolysList(L)
            except TypeError: continue
            r = zone_fill.setdefault((z.GetNetCode(), L), np.zeros((H, W), bool))
            for k in range(fp_.OutlineCount()):
                ol = fp_.Outline(k); pts = [(ol.CPoint(m).x*NM, ol.CPoint(m).y*NM) for m in range(ol.PointCount())]
                if len(pts) >= 3: stamp_poly(r, pts, 0.0)
    # board edge: cells whose centre is within EDGE of the outline (outline polygon)
    outline = pcbnew.SHAPE_POLY_SET(); bd.GetBoardPolygonOutlines(outline)
    inside = np.zeros((H, W), bool)
    for k in range(outline.OutlineCount()):
        ol = outline.Outline(k); pts = [(ol.CPoint(m).x*NM, ol.CPoint(m).y*NM) for m in range(ol.PointCount())]
        stamp_poly(inside, pts, 0.0)
    edge_dist = ndimage.distance_transform_edt(inside)*G - SAMP   # distance to outside (conservative)
    print(f'rasters built in {time.time()-t0:.1f}s; grid {W}x{H}', flush=True)
build_rasters()
# --------------- per-net passability ----------------------------------------------------
_cache = {}
def own_raster(nc, L):
    key = (nc, L)
    if key in ownr: return ownr[key]
    r = np.zeros((H, W), bool)
    for n_, kind, g, lc in items[L]:
        if n_ != nc: continue
        if kind == 'poly': stamp_poly(r, g, extra(nc, lc))
        elif kind == 'disc': stamp_disc(r, g[0], g[1], g[2]+extra(nc))
        else: stamp_seg(r, g[0], g[1], g[2], g[3], g[4]+extra(nc))
    ownr[key] = r; return r
def passable(nc, L, w, clr):
    key = (nc, L, round(w, 3), round(clr, 3))
    if key in _cache: return _cache[key]
    other = copper[L] & ~own_raster(nc, L)
    d = ndimage.distance_transform_edt(~other)*G
    ok = (d >= w/2 + clr + SAMP + MARG) & (edge_dist >= EDGE + w/2 + MARG) & ~keep[L]
    # own pads are always enterable (track inside own pad copper)
    core = np.zeros((H, W), bool)
    for n_, kind, g, lc in items[L]:
        if n_ == nc and kind == 'poly': stamp_poly(core, g, 0.0)
    core &= ndimage.distance_transform_edt(core)*G >= w/2 + 0.03
    okc = ok | (core & ~keep[L] & (edge_dist >= EDGE + w/2 + MARG))
    _cache[key] = okc; return okc
def via_ok(nc, clr):
    key = ('via', nc, round(clr, 3))
    if key in _cache: return _cache[key]
    ok = (edge_dist >= EDGE + VIA_D/2 + MARG)
    for L in ALL_CU:
        other = copper[L] & ~own_raster(nc, L)
        d = ndimage.distance_transform_edt(~other)*G
        ok &= (d >= VIA_D/2 + clr + SAMP + MARG) & ~keep[L]
    dh = ndimage.distance_transform_edt(~holes)*G
    ok &= dh >= VIA_DR/2 + H2H + SAMP + MARG
    _cache[key] = ok; return ok
def invalidate(L=None):
    for k in list(_cache):
        if L is None or k[0] == 'via' or (len(k) == 4 and k[1] == L): del _cache[k]
    for k in list(ownr):
        if L is None or k[1] == L: del ownr[k]
# --------------- clusters ---------------------------------------------------------------------
conn = bd.GetConnectivity()
def cluster_cells(item, nc):
    """cells of the connected cluster of `item`: (layer -> bool raster) restricted to routable layers; plus plane flag."""
    seen = set(); stack = [item]; res = {L: np.zeros((H, W), bool) for L in ROUTE_L}; plane = False
    if isinstance(item, tuple):   # zone: its filled cells on routable layers (planes on In1/In4 handled by plane_goal)
        for (znc, L), r in zone_fill.items():
            if znc == nc and L in res: res[L] |= r
        return res, 1
    while stack:
        it = stack.pop(); u = it.m_Uuid.AsString()
        if u in seen: continue
        seen.add(u)
        if it.GetClass() == 'PAD':
            poly = pad_poly(it)
            for L in ROUTE_L:
                if it.IsOnLayer(L): stamp_poly(res[L], poly, 0.0)
            for x in conn.GetConnectedTracks(it): stack.append(x)
        elif it.GetClass() == 'PCB_VIA':
            p = it.GetPosition()
            try: r = it.GetWidth(pcbnew.PADSTACK.ALL_LAYERS)*NM/2
            except TypeError:
                try: r = it.GetWidth()*NM/2
                except Exception: r = 0.25
            for L in ROUTE_L: stamp_disc(res[L], p.x*NM, p.y*NM, r)
            for x in conn.GetConnectedTracks(it): stack.append(x)
            for x in conn.GetConnectedPads(it): stack.append(x)
        else:
            s, e = it.GetStart(), it.GetEnd(); L = int(it.GetLayer())
            if L in res: stamp_seg(res[L], s.x*NM, s.y*NM, e.x*NM, e.y*NM, it.GetWidth()*NM/2)
            for x in conn.GetConnectedTracks(it): stack.append(x)
            for x in conn.GetConnectedPads(it): stack.append(x)
        if len(seen) > 4000: break
    return res, len(seen)
def find_item(desc, pos):
    """locate the board item named by a DRC unconnected entry."""
    x, y = pos['x'], pos['y']; v = pcbnew.VECTOR2I(int(round(x/NM)), int(round(y/NM)))
    m = re.match(r'(?:PTH |NPTH )?[Pp]ad (\S+) \[.*\] of (\S+)', desc)
    if m:
        for nc_, ref, num, lays, poly, lc, p in pad_list:
            if ref == m.group(2) and num == m.group(1): return p
    if desc.startswith('Zone'):
        lm = re.search(r' on ([A-Za-z0-9.]+)', desc); mm = re.search(r'\[([^\]]+)\]', desc)
        n = bd.FindNet(mm.group(1)) if mm else None
        return ('zone', n.GetNetCode() if n else None, lm.group(1) if lm else None)
    if desc.startswith('Via'):
        best = None
        for t in bd.GetTracks():
            if t.GetClass() != 'PCB_VIA': continue
            p = t.GetPosition(); d = math.hypot(p.x*NM-x, p.y*NM-y)
            if best is None or d < best[0]: best = (d, t)
        return best[1] if best and best[0] < 0.3 else None
    if desc.startswith('Track'):
        lm = re.search(r' on ([A-Za-z0-9.]+)', desc); lay = lm.group(1) if lm else None
        best = None
        for t in bd.GetTracks():
            if t.GetClass() == 'PCB_VIA': continue
            if lay and bd.GetLayerName(t.GetLayer()) != lay: continue
            s, e = t.GetStart(), t.GetEnd(); ax, ay, bx_, by_ = s.x*NM, s.y*NM, e.x*NM, e.y*NM
            dx, dy = bx_-ax, by_-ay; L2 = dx*dx+dy*dy; u = 0 if L2 < 1e-18 else max(0, min(1, ((x-ax)*dx+(y-ay)*dy)/L2))
            d = math.hypot(x-ax-u*dx, y-ay-u*dy)
            if best is None or d < best[0]: best = (d, t)
        return best[1] if best and best[0] < 0.3 else None
    return None
# --------------- A* -------------------------------------------------------------------------------
LI = {L: k for k, L in enumerate(ROUTE_L)}
DIRS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]
def astar(src, dst, passL, viaok, zpen, plane_goal, maxpop=600000):
    """src/dst: dict L->bool raster. Returns list of (L,i,j) or None."""
    NL = len(ROUTE_L); SZ = W*H
    goal = np.zeros(NL*SZ, bool); start = []
    for L in ROUTE_L:
        k = LI[L]
        goal[k*SZ:(k+1)*SZ] = dst[L].ravel()          # own copper: reachable even where not 'passable'
        for idx in np.flatnonzero(src[L].ravel()): start.append(k*SZ+int(idx))
    if plane_goal is not None:
        goal |= np.tile(plane_goal.ravel(), NL)
    if not start or not goal.any(): return None
    # heuristic: EDT (in mm) to goal cells, min over layers
    gany = goal.reshape(NL, H, W).any(axis=0)
    hmap = (ndimage.distance_transform_edt(~gany)*G).ravel()
    passflat = [passL[L].ravel() for L in ROUTE_L]; viaflat = viaok.ravel(); zflat = [zpen[L].ravel() for L in ROUTE_L]
    gbest = {}; parent = {}; pq = []
    for s in start: gbest[s] = 0.0; heapq.heappush(pq, (hmap[s % SZ], 0.0, s))
    pops = 0
    while pq and pops < maxpop:
        f, g, st = heapq.heappop(pq); pops += 1
        if gbest.get(st, 1e18) < g - 1e-12: continue
        if goal[st]:
            path = [st]
            while path[-1] in parent: path.append(parent[path[-1]])
            path.reverse(); return [(ROUTE_L[p // SZ], (p % SZ) % W, (p % SZ) // W) for p in path]
        k = st // SZ; idx = st % SZ; i = idx % W; j = idx // W
        for di, dj, c in DIRS:
            ni, nj = i+di, j+dj
            if ni < 0 or nj < 0 or ni >= W or nj >= H: continue
            nidx = nj*W+ni; nst = k*SZ+nidx
            if not passflat[k][nidx] and not goal[nst]: continue
            if di and dj and not (passflat[k][j*W+ni] or goal[k*SZ+j*W+ni]) and not (passflat[k][nj*W+i] or goal[k*SZ+nj*W+i]): continue
            ng = g + c*G*(ZONE_PEN if zflat[k][nidx] else 1.0)
            if ng < gbest.get(nst, 1e18) - 1e-12:
                gbest[nst] = ng; parent[nst] = st; heapq.heappush(pq, (ng + hmap[nidx], ng, nst))
        if viaflat[idx]:
            for k2 in range(NL):
                if k2 == k or not passflat[k2][idx]: continue
                ng = g + VIA_COST; nst = k2*SZ+idx
                if ng < gbest.get(nst, 1e18) - 1e-12:
                    gbest[nst] = ng; parent[nst] = st; heapq.heappush(pq, (ng + hmap[idx], ng, nst))
    return None
def simplify(pts):
    """merge collinear runs (cells) into segments."""
    out = [pts[0]]
    for k in range(1, len(pts)-1):
        (L0, i0, j0), (L1, i1, j1), (L2, i2, j2) = out[-1], pts[k], pts[k+1]
        if L0 == L1 == L2 and (i1-i0)*(j2-j1) == (j1-j0)*(i2-i1) and (i1-i0)*(i2-i1)+(j1-j0)*(j2-j1) > 0: continue
        out.append(pts[k])
    out.append(pts[-1]); return out
def commit(path, net, w):
    pts = simplify(path); added = []
    for a, b in zip(pts, pts[1:]):
        if a[0] != b[0]:   # via
            v = pcbnew.PCB_VIA(bd); x, y = cx(a[1]), cy(a[2]); v.SetPosition(pcbnew.VECTOR2I(int(round(x/NM)), int(round(y/NM))))
            v.SetDrill(int(VIA_DR/NM))
            try: v.SetWidth(int(VIA_D/NM))
            except TypeError: v.SetWidth(pcbnew.PADSTACK.ALL_LAYERS, int(VIA_D/NM))
            v.SetViaType(pcbnew.VIATYPE_THROUGH); v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu); v.SetNet(net); bd.Add(v); added.append(v)
            for L in ALL_CU: stamp_disc(copper[L], x, y, VIA_D/2+extra(net.GetNetCode())); items[L].append((net.GetNetCode(), 'disc', (x, y, VIA_D/2), 0.0))
            stamp_disc(holes, x, y, VIA_DR/2)
        else:
            t = pcbnew.PCB_TRACK(bd); ax, ay, bx_, by_ = cx(a[1]), cy(a[2]), cx(b[1]), cy(b[2])
            t.SetStart(pcbnew.VECTOR2I(int(round(ax/NM)), int(round(ay/NM)))); t.SetEnd(pcbnew.VECTOR2I(int(round(bx_/NM)), int(round(by_/NM))))
            t.SetWidth(int(round(w/NM))); t.SetLayer(a[0]); t.SetNet(net); bd.Add(t); added.append(t)
            stamp_seg(copper[a[0]], ax, ay, bx_, by_, w/2+extra(net.GetNetCode())); items[a[0]].append((net.GetNetCode(), 'seg', (ax, ay, bx_, by_, w/2), 0.0))
    invalidate(); return added
# --------------- main loop ----------------------------------------------------------------------
def sig_of(t):
    if t.GetClass() == 'PCB_VIA': p = t.GetPosition(); return ('V', p.x, p.y)
    s_, e_ = t.GetStart(), t.GetEnd(); a, b = (s_.x, s_.y), (e_.x, e_.y)
    if b < a: a, b = b, a
    return ('T', int(t.GetLayer()), a, b, t.GetWidth())
pre_sigs = {sig_of(t) for t in bd.GetTracks()}
PRE_DRC = run_drc(OUT, f'{S}/drc_route_pre.json')
log = []; all_added = []
for rnd in range(ROUNDS):
    with quiet_stderr(): pcbnew.SaveBoard(OUT, bd)
    d = run_drc(OUT, f'{S}/drc_route_r{rnd}.json'); unc = d['unconnected_items']
    print(f'round {rnd}: {summary(d)}', flush=True)
    if not unc: break
    bd = load_board(OUT); conn = bd.GetConnectivity()
    if rnd > 0:
        # rip this run's copper that is involved in NEW violations (vs the pre-route baseline), then rebuild rasters
        newv, _ = diff(PRE_DRC, d); ripped = 0
        pos = [(i['pos']['x'], i['pos']['y']) for v in newv if v['type'] != 'solder_mask_bridge' for i in v['items'] if 'pos' in i]
        if pos:
            for t in list(bd.GetTracks()):
                if (sig_of(t)) in pre_sigs: continue
                if t.GetClass() == 'PCB_VIA': p_ = t.GetPosition(); near = any(math.hypot(p_.x*NM-x, p_.y*NM-y) < 0.6 for x, y in pos)
                else:
                    s_, e_ = t.GetStart(), t.GetEnd(); near = False
                    for x, y in pos:
                        ax, ay, bx_, by_ = s_.x*NM, s_.y*NM, e_.x*NM, e_.y*NM; dx, dy = bx_-ax, by_-ay; L2 = dx*dx+dy*dy
                        u = 0 if L2 < 1e-18 else max(0, min(1, ((x-ax)*dx+(y-ay)*dy)/L2))
                        if math.hypot(x-ax-u*dx, y-ay-u*dy) < 0.4: near = True; break
                if near: bd.Delete(t); ripped += 1
            print(f'   ripped {ripped} new copper items involved in {len(newv)} new violations', flush=True)
            with quiet_stderr(): pcbnew.SaveBoard(OUT, bd)
            bd = load_board(OUT); conn = bd.GetConnectivity()
        build_rasters()
    pad_list = [(p.GetNetCode(), f.GetReference(), p.GetNumber(), [L for L in ALL_CU if p.IsOnLayer(L)], None, 0, p) for f in bd.GetFootprints() for p in f.Pads()]
    pairs = []
    for it in unc:
        a, b = it['items'][0], it['items'][1]
        m = re.search(r'\[([^\]]+)\]', a['description']); netname = m.group(1) if m else None
        pairs.append((netname, a, b))
    # shortest first
    pairs.sort(key=lambda p: math.hypot(p[1]['pos']['x']-p[2]['pos']['x'], p[1]['pos']['y']-p[2]['pos']['y']))
    done = 0; fail = []; t_round = time.time()
    for k, (netname, a, b) in enumerate(pairs):
        net = bd.FindNet(netname)
        if net is None: fail.append((netname, 'no net')); continue
        nc = net.GetNetCode(); clr, w = net_rule(netname); w = WIDE.get(netname, w)
        ia = find_item(a['description'], a['pos']); ib = find_item(b['description'], b['pos'])
        if ia is None or ib is None: fail.append((netname, 'item not found')); continue
        srcA, na = cluster_cells(ia, nc); srcB, nb_ = cluster_cells(ib, nc)
        if na > nb_: srcA, srcB = srcB, srcA
        passL = {L: passable(nc, L, w, clr) for L in ROUTE_L}; vok = via_ok(nc, clr)
        zpen = {L: np.zeros((H, W), bool) for L in ROUTE_L}
        for (znc, L), r in zone_fill.items():
            if L in zpen and znc != nc:
                zn = bd.FindNet(znc); zname = zn.GetNetname() if zn else ''
                if zname in HARD_ZONE_NETS: passL[L] = passL[L] & ~r
                else: zpen[L] |= r
        # same-net pours: target cells; plane nets (pour on a non-routed layer): any legal via site is a goal
        plane_goal = None
        for (znc, L), r in zone_fill.items():
            if znc != nc: continue
            if L in srcB: srcB[L] |= r
            else: plane_goal = vok if plane_goal is None else (plane_goal | vok)
        t1 = time.time()
        path = astar(srcA, srcB, passL, vok, zpen, plane_goal)
        if path is None and w > 0.26:   # wide power net: retry narrower rather than fail
            for w2 in (0.4, 0.25):
                if w2 >= w: continue
                passL = {L: passable(nc, L, w2, clr) for L in ROUTE_L}
                path = astar(srcA, srcB, passL, vok, zpen, plane_goal)
                if path is not None: w = w2; break
        for _retry in range(3):
            if path is None: break
            vs = [(cx(p[1]), cy(p[2])) for p, q in zip(path, path[1:]) if p[0] != q[0]]
            bad = [(a, b) for i_, a in enumerate(vs) for b in vs[i_+1:] if math.hypot(a[0]-b[0], a[1]-b[1]) < 0.6]
            if not bad: break
            for a, b in bad:
                for x_, y_ in (a, b): vok = vok.copy(); vok[max(0, cj(y_)-2):cj(y_)+3, max(0, ci(x_)-2):ci(x_)+3] = False
            path = astar(srcA, srcB, passL, vok, zpen, plane_goal)
        if path is None:
            ns_ = sum(int(srcA[L].sum()) for L in ROUTE_L); ng_ = sum(int(srcB[L].sum()) for L in ROUTE_L); ps_ = sum(int((srcA[L] & passL[L]).sum()) for L in ROUTE_L)
            fail.append((netname, f'{a["description"][:30]} ~ {b["description"][:30]}')); print(f'  [{k+1}/{len(pairs)}] {netname}: FAILED ({time.time()-t1:.1f}s) src cells {ns_} (passable {ps_}) dst cells {ng_} clusters {na}/{nb_} {a["description"][:28]} ~ {b["description"][:28]}', flush=True); continue
        added = commit(path, net, w); all_added += added; done += 1
        print(f'  [{k+1}/{len(pairs)}] {netname}: routed {len(path)} cells, {sum(1 for x in added if x.GetClass()=="PCB_VIA")} vias ({time.time()-t1:.1f}s)', flush=True)
        conn = bd.GetConnectivity()
    print(f'round {rnd}: routed {done}/{len(pairs)} in {time.time()-t_round:.0f}s; failed {len(fail)}', flush=True)
    log.append(dict(round=rnd, pairs=len(pairs), routed=done, failed=fail))
    with quiet_stderr():
        pcbnew.ZONE_FILLER(bd).Fill(bd.Zones()); pcbnew.SaveBoard(OUT, bd)
    if done == 0: break
d = run_drc(OUT, f'{S}/drc_route_last.json'); bd = load_board(OUT)
newv, _ = diff(PRE_DRC, d); ripped = 0
pos = [(i['pos']['x'], i['pos']['y']) for v in newv if v['type'] != 'solder_mask_bridge' for i in v['items'] if 'pos' in i]
for t in list(bd.GetTracks()):
    if sig_of(t) in pre_sigs: continue
    if t.GetClass() == 'PCB_VIA': p_ = t.GetPosition(); near = any(math.hypot(p_.x*NM-x, p_.y*NM-y) < 0.6 for x, y in pos)
    else:
        s_, e_ = t.GetStart(), t.GetEnd(); near = False
        for x, y in pos:
            ax, ay, bx_, by_ = s_.x*NM, s_.y*NM, e_.x*NM, e_.y*NM; dx, dy = bx_-ax, by_-ay; L2 = dx*dx+dy*dy
            u = 0 if L2 < 1e-18 else max(0, min(1, ((x-ax)*dx+(y-ay)*dy)/L2))
            if math.hypot(x-ax-u*dx, y-ay-u*dy) < 0.4: near = True; break
    if near: bd.Delete(t); ripped += 1
print(f'final rip: {ripped} items for {len(newv)} new violations')
with quiet_stderr():
    pcbnew.ZONE_FILLER(bd).Fill(bd.Zones()); pcbnew.SaveBoard(OUT, bd)
d = run_drc(OUT, f'{S}/drc_route_final.json'); print('final:', summary(d))
json.dump(log, open(S+'/router_log.json', 'w'), indent=1)
