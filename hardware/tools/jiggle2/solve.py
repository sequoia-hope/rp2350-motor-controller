#!/usr/bin/env python3
"""jiggle2 stage 2: PBD relaxation of the grafted copper graph.

Bound endpoints lerp from pre-rip to current pad positions; everything else
follows under constraints: chain stretch limit, rubber-band smoothing, and
clearance projection against static obstacles + other dynamic copper.
Pure python, no pcbnew. Reads data/graph.json + data/obstacles.json,
writes data/solution.json.
"""
import math, json, sys
from collections import defaultdict
from common import load_json, save_json, seg_seg_dist

G = load_json('graph.json')
O = load_json('obstacles.json')
nodes, edges = G['nodes'], G['edges']
F_CU, B_CU = G['F'], G['B']

CLR = O['clearance']            # copper-copper, mm
HOLE_CLR = O['hole_clearance']  # hole-copper, mm
SUB = 0.8                       # max sub-segment length
STEPS = int(sys.argv[sys.argv.index('--steps')+1]) if '--steps' in sys.argv else 60
SWEEPS = 3
RELAX = 0.6                     # projection under-relaxation
ETA = 0.15                      # rubber-band smoothing weight (drops in polish)
eta_now = ETA
EXTRA = 0.005                   # margin over exact clearance, mm
POLISH = 120
NODE_BUDGET = 4000              # cap for adaptive re-subdivision
true_viol = 0                   # violations of the actual rule (no margin)

# ---- resolve nets via bound pads (fixes renamed pre-rev-B nets) -------------
adj = defaultdict(list)
for ei, e in enumerate(edges):
    adj[e['a']].append(ei); adj[e['b']].append(ei)

comp_of = {}
comps = []
for start in range(len(nodes)):
    if start in comp_of:
        continue
    stack, comp = [start], []
    comp_of[start] = len(comps)
    while stack:
        n = stack.pop()
        comp.append(n)
        for ei in adj[n]:
            for m in (edges[ei]['a'], edges[ei]['b']):
                if m not in comp_of:
                    comp_of[m] = len(comps)
                    stack.append(m)
    comps.append(comp)

net_fix = 0
cur_pad_net = {(p['ref'], p['num']): p['net'] for p in O['pads']}
multi_net_comps = []
for comp in comps:
    votes = defaultdict(int)
    for n in comp:
        b = nodes[n]['bind']
        if b and tuple(b) in cur_pad_net:
            net = cur_pad_net[tuple(b)]
            if net:
                votes[net] += 1
    enets = {edges[ei]['net'] for n in comp for ei in adj[n]}
    if len(votes) == 1:
        net = next(iter(votes))
        for n in comp:
            for ei in adj[n]:
                if edges[ei]['net'] != net:
                    edges[ei]['net'] = net; net_fix += 1
            nodes[n]['net'] = net
    elif len(votes) > 1:
        # Pre-rip net was split by a rev-B change (e.g. termination switch
        # inserted). Assign each node the net of its graph-nearest bound pad;
        # edges bridging two nets are obsolete copper — drop them.
        multi_net_comps.append((sorted(votes.items(), key=lambda kv: -kv[1]),
                                sorted(enets)))
        from heapq import heappush, heappop
        dist = {}
        pq = []
        for n in comp:
            b = nodes[n]['bind']
            if b and cur_pad_net.get(tuple(b)):
                dist[n] = (0.0, cur_pad_net[tuple(b)])
                heappush(pq, (0.0, n))
        while pq:
            dn, n = heappop(pq)
            if dist[n][0] < dn:
                continue
            for ei in adj[n]:
                m = edges[ei]['b'] if edges[ei]['a'] == n else edges[ei]['a']
                L = math.hypot(nodes[m]['x']-nodes[n]['x'], nodes[m]['y']-nodes[n]['y'])
                if m not in dist or dist[m][0] > dn + L:
                    dist[m] = (dn + L, dist[n][1])
                    heappush(pq, (dn + L, m))
        for n in comp:
            nodes[n]['net'] = dist.get(n, (0, ''))[1]
        for n in comp:
            for ei in adj[n]:
                e = edges[ei]
                na, nb = nodes[e['a']]['net'], nodes[e['b']]['net']
                if na and nb and na != nb:
                    e['drop'] = True
                elif e['net'] != (na or nb):
                    e['net'] = na or nb; net_fix += 1
    else:
        for n in comp:
            nodes[n]['net'] = nodes[n].get('net') or (next(iter(enets)) if enets else '')

dropped = [e for e in edges if e.get('drop')]
edges = [e for e in edges if not e.get('drop')]
adj = defaultdict(list)
for ei, e in enumerate(edges):
    adj[e['a']].append(ei); adj[e['b']].append(ei)
print(f'net resolution: {len(comps)} components, {net_fix} edge nets rewritten, '
      f'{len(multi_net_comps)} split components, {len(dropped)} bridge edges dropped')
for votes, enets in multi_net_comps[:6]:
    print('  split:', votes, 'edges were', enets)

# ---- subdivide edges into chains --------------------------------------------
P = [[n['x'], n['y']] for n in nodes]         # positions (mutable)
anchor = {}                                    # nid -> (ox,oy,tx,ty)
for i, n in enumerate(nodes):
    if n['bind'] and n['target']:
        anchor[i] = (n['x'], n['y'], n['target'][0], n['target'][1])
is_via = [n['kind'] == 'via' for n in nodes]
via_dia = [n['dia'] or 0 for n in nodes]
via_drill = [n['drill'] or 0 for n in nodes]
node_net = [n.get('net') or '' for n in nodes]

segs = []   # dyn sub-segments: dict(a,b,layer,w,net,edge)
def add_seg(a, b, layer, w, net, ei):
    segs.append(dict(a=a, b=b, layer=layer, w=w, net=net, edge=ei))

for ei, e in enumerate(edges):
    a, b = e['a'], e['b']
    ax, ay = P[a]; bx, by = P[b]
    L = math.hypot(bx-ax, by-ay)
    nsub = max(1, int(math.ceil(L / SUB)))
    prev = a
    for k in range(1, nsub):
        t = k / nsub
        nid = len(P)
        P.append([ax + (bx-ax)*t, ay + (by-ay)*t])
        is_via.append(False); via_dia.append(0); via_drill.append(0)
        node_net.append(e['net'])
        add_seg(prev, nid, e['layer'], e['w'], e['net'], ei)
        prev = nid
    add_seg(prev, b, e['layer'], e['w'], e['net'], ei)

# chain neighbor map for smoothing (only degree-2 free nodes get smoothed)
nbr = defaultdict(list)
for s in segs:
    nbr[s['a']].append(s['b']); nbr[s['b']].append(s['a'])
print(f'{len(P)} solver nodes, {len(segs)} dyn sub-segments')

def resubdivide():
    """Rubber bands need length to wrap obstacles: split overstretched segs."""
    global nbr
    added = 0
    for si in range(len(segs)):
        if len(P) >= NODE_BUDGET:
            break
        s = segs[si]
        (x1, y1), (x2, y2) = seg_pts(s)
        if math.hypot(x2-x1, y2-y1) <= SUB * 1.4:
            continue
        nid = len(P)
        P.append([(x1+x2)/2, (y1+y2)/2])
        is_via.append(False); via_dia.append(0); via_drill.append(0)
        node_net.append(s['net'])
        old_b = s['b']
        s['b'] = nid
        segs.append(dict(a=nid, b=old_b, layer=s['layer'], w=s['w'],
                         net=s['net'], edge=s['edge']))
        added += 1
    if added:
        nbr = defaultdict(list)
        for s in segs:
            nbr[s['a']].append(s['b']); nbr[s['b']].append(s['a'])
    return added

# ---- static obstacle grids ---------------------------------------------------
CELL = 1.2
def cells_for_seg(ax, ay, bx, by, r):
    x0, x1 = sorted((ax, bx)); y0, y1 = sorted((ay, by))
    for cx in range(int((x0-r)//CELL), int((x1+r)//CELL)+1):
        for cy in range(int((y0-r)//CELL), int((y1+r)//CELL)+1):
            yield (cx, cy)

static_grid = defaultdict(list)   # cell -> list of (kind, idx)
pad_edges = []                    # flattened pad outline segments
for pi, p in enumerate(O['pads']):
    pts = p['pts']
    for i in range(len(pts)):
        a, b = pts[i], pts[(i+1) % len(pts)]
        pad_edges.append((a[0], a[1], b[0], b[1], pi))
for i, (ax, ay, bx, by, pi) in enumerate(pad_edges):
    for c in cells_for_seg(ax, ay, bx, by, 1.0):
        static_grid[c].append(('PE', i))
for ti, t in enumerate(O['tracks']):
    for c in cells_for_seg(t['a'][0], t['a'][1], t['b'][0], t['b'][1], 1.0):
        static_grid[c].append(('T', ti))
for vi, v in enumerate(O['vias']):
    for c in cells_for_seg(v['x'], v['y'], v['x'], v['y'], 1.0):
        static_grid[c].append(('V', vi))

def pad_covers_layer(p, layer):
    return layer in p['layers']

def point_in_poly(x, y, pts):
    inside = False
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi = pts[i]; xj, yj = pts[j]
        if (yi > y) != (yj > y) and x < (xj-xi)*(y-yi)/(yj-yi+1e-12)+xi:
            inside = not inside
        j = i
    return inside

# ---- projection helpers -------------------------------------------------------
def push_nodes(sidx, ux, uy, amt):
    """Push both nodes of dyn sub-seg sidx along (ux,uy) by amt, skipping anchors."""
    s = segs[sidx]
    for nid in (s['a'], s['b']):
        if nid in anchor:
            continue
        P[nid][0] += ux * amt
        P[nid][1] += uy * amt

def seg_pts(s):
    return (P[s['a']][0], P[s['a']][1]), (P[s['b']][0], P[s['b']][1])

def closest_dir(p1, p2, q1, q2):
    """Unit vector pushing seg p away from seg q, from closest approach."""
    # midpoint fallback direction
    mx = (p1[0]+p2[0])/2 - (q1[0]+q2[0])/2
    my = (p1[1]+p2[1])/2 - (q1[1]+q2[1])/2
    L = math.hypot(mx, my)
    if L < 1e-9:
        return 1.0, 0.0
    return mx/L, my/L

violations = 0
pen_pairs = {}                  # physical metric: (edge/via id, obstacle) -> worst penetration
def note_pen(key, depth):
    if depth > 0:
        pen_pairs[key] = max(pen_pairs.get(key, 0.0), depth)

def bump(sidx, ux, uy, need, rule_gap):
    """Apply a push and book-keep: `need` is deficit vs padded rule,
    rule_gap is distance minus the *actual* rule (negative = true violation)."""
    global violations, true_viol
    push_nodes(sidx, ux, uy, need * RELAX)
    violations += 1
    if rule_gap < 0:
        true_viol += 1

def project_pair_dyn_static(sidx, kind, idx):
    s = segs[sidx]
    p1, p2 = seg_pts(s)
    if kind == 'T':
        t = O['tracks'][idx]
        if t['layer'] != s['layer'] or t['net'] == s['net']:
            return
        d = seg_seg_dist(p1, p2, tuple(t['a']), tuple(t['b']))
        rule = CLR + (t['w'] + s['w'])/2
        if d < rule + EXTRA:
            ux, uy = closest_dir(p1, p2, tuple(t['a']), tuple(t['b']))
            bump(sidx, ux, uy, rule + EXTRA - d, d - rule)
    elif kind == 'V':
        v = O['vias'][idx]
        c = (v['x'], v['y'])
        if v['net'] == s['net']:
            return
        d = seg_seg_dist(p1, p2, c, c)
        rule = max(CLR + v['dia']/2 + s['w']/2,
                   HOLE_CLR + v['drill']/2 + s['w']/2)
        if d < rule + EXTRA:
            ux, uy = closest_dir(p1, p2, c, c)
            bump(sidx, ux, uy, rule + EXTRA - d, d - rule)
    elif kind == 'PE':
        ax, ay, bx, by, pi = pad_edges[idx]
        p = O['pads'][pi]
        if p['net'] == s['net']:
            return
        on_layer = s['layer'] in p['layers']
        if on_layer:
            d = seg_seg_dist(p1, p2, (ax, ay), (bx, by))
            if point_in_poly((p1[0]+p2[0])/2, (p1[1]+p2[1])/2, p['pts']):
                d = 0.0
            rule = CLR + s['w']/2
            if d < rule + EXTRA:
                ux, uy = closest_dir(p1, p2, (p['x'], p['y']), (p['x'], p['y']))
                bump(sidx, ux, uy, rule + EXTRA - d, d - rule)
        if p['drill']:
            # hole edge to track edge, any layer
            dh = seg_seg_dist(p1, p2, (p['x'], p['y']), (p['x'], p['y']))
            rule = HOLE_CLR + p['drill']/2 + s['w']/2
            if dh < rule + EXTRA:
                ux, uy = closest_dir(p1, p2, (p['x'], p['y']), (p['x'], p['y']))
                bump(sidx, ux, uy, rule + EXTRA - dh, dh - rule)

def bump_node(nid, ux, uy, need, rule_gap):
    global violations, true_viol
    if nid not in anchor:
        P[nid][0] += ux * need * RELAX
        P[nid][1] += uy * need * RELAX
    violations += 1
    if rule_gap < 0:
        true_viol += 1

def project_via_node(nid):
    """Dynamic via: keep clearance + hole spacing vs static copper and holes."""
    x, y = P[nid]
    r_cu = via_dia[nid]/2
    r_hole = via_drill[nid]/2
    net = node_net[nid]
    seen = set()
    for c in cells_for_seg(x, y, x, y, 1.0):
        for kind, idx in static_grid.get(c, ()):
            if (kind, idx) in seen:
                continue
            seen.add((kind, idx))
            if kind == 'T':
                t = O['tracks'][idx]
                d = seg_seg_dist((x, y), (x, y), tuple(t['a']), tuple(t['b']))
                rule = HOLE_CLR + r_hole + t['w']/2      # hole applies same-net too
                if t['net'] != net and t['layer'] in (F_CU, B_CU):
                    rule = max(rule, CLR + r_cu + t['w']/2)
                if t['net'] == net:
                    continue  # same-net track may touch via copper; hole ok by construction
                if d < rule + EXTRA:
                    ux, uy = closest_dir((x, y), (x, y), tuple(t['a']), tuple(t['b']))
                    bump_node(nid, ux, uy, rule + EXTRA - d, d - rule)
            elif kind == 'V':
                v = O['vias'][idx]
                d = math.hypot(x-v['x'], y-v['y'])
                rule = HOLE_CLR + r_hole + v['drill']/2   # hole-hole regardless of net
                if v['net'] != net:
                    rule = max(rule, CLR + r_cu + v['dia']/2)
                if d < rule + EXTRA:
                    ux, uy = closest_dir((x, y), (x, y), (v['x'], v['y']), (v['x'], v['y']))
                    bump_node(nid, ux, uy, rule + EXTRA - d, d - rule)
            elif kind == 'PE':
                ax, ay, bx, by, pi = pad_edges[idx]
                p = O['pads'][pi]
                if p['net'] == net:
                    continue
                d = seg_seg_dist((x, y), (x, y), (ax, ay), (bx, by))
                if point_in_poly(x, y, p['pts']):
                    d = 0.0
                rule = max(CLR + r_cu, HOLE_CLR + r_hole)
                if p['drill']:
                    dh = math.hypot(x - p['x'], y - p['y'])
                    hr = HOLE_CLR + r_hole + p['drill']/2
                    if dh < hr + EXTRA:
                        ux, uy = closest_dir((x, y), (x, y), (p['x'], p['y']), (p['x'], p['y']))
                        bump_node(nid, ux, uy, hr + EXTRA - dh, dh - hr)
                        continue
                if d < rule + EXTRA:
                    ux, uy = closest_dir((x, y), (x, y), (p['x'], p['y']), (p['x'], p['y']))
                    bump_node(nid, ux, uy, rule + EXTRA - d, d - rule)

# ---- wrap pass: exact homotopic detours around penetrated pads ------------------
def convex_hull_pts(pts):
    pts = sorted(set((round(x, 4), round(y, 4)) for x, y in pts))
    if len(pts) <= 2:
        return [list(p) for p in pts]
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0: lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0: hi.pop()
        hi.append(p)
    return [list(p) for p in lo[:-1] + hi[:-1]]   # CCW in y-up; consistent either way

def offset_convex(pts, r):
    """Offset a convex polygon outward by r (miter joins)."""
    hull = convex_hull_pts(pts)
    n = len(hull)
    if n < 3:
        return hull
    # ensure consistent winding via signed area
    area = sum(hull[i][0]*hull[(i+1) % n][1] - hull[(i+1) % n][0]*hull[i][1]
               for i in range(n))
    if area < 0:
        hull = hull[::-1]
    lines = []
    for i in range(n):
        ax, ay = hull[i]; bx, by = hull[(i+1) % n]
        ex, ey = bx-ax, by-ay
        L = math.hypot(ex, ey) or 1e-9
        nx, ny = ey/L, -ex/L     # outward for CCW area>0 in screen coords... sign fixed below
        # outward = away from centroid
        cx = sum(p[0] for p in hull)/n; cy = sum(p[1] for p in hull)/n
        mx, my = (ax+bx)/2 - cx, (ay+by)/2 - cy
        if nx*mx + ny*my < 0:
            nx, ny = -nx, -ny
        lines.append((ax + nx*r, ay + ny*r, ex, ey))
    out = []
    for i in range(n):
        x1, y1, e1x, e1y = lines[i-1]
        x2, y2, e2x, e2y = lines[i]
        den = e1x*e2y - e1y*e2x
        if abs(den) < 1e-12:
            out.append([x2, y2])
            continue
        t = ((x2-x1)*e2y - (y2-y1)*e2x) / den
        out.append([x1 + e1x*t, y1 + e1y*t])
    return out

def seg_poly_hits(p1, p2, poly):
    """Return sorted entry/exit params t of segment through convex poly."""
    inside1 = point_in_poly(p1[0], p1[1], poly)
    inside2 = point_in_poly(p2[0], p2[1], poly)
    ts = []
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i+1) % n]
        d1x, d1y = p2[0]-p1[0], p2[1]-p1[1]
        d2x, d2y = b[0]-a[0], b[1]-a[1]
        den = d1x*d2y - d1y*d2x
        if abs(den) < 1e-12:
            continue
        t = ((a[0]-p1[0])*d2y - (a[1]-p1[1])*d2x) / den
        u = ((a[0]-p1[0])*d1y - (a[1]-p1[1])*d1x) / den
        if -1e-9 <= t <= 1+1e-9 and -1e-9 <= u <= 1+1e-9:
            ts.append(max(0.0, min(1.0, t)))
    return inside1, inside2, sorted(ts)

def boundary_paths(poly, pa, pb):
    """Two corner paths along poly boundary from near-pa to near-pb."""
    n = len(poly)
    def edge_of(p):
        best = (1e9, 0)
        for i in range(n):
            a, b = poly[i], poly[(i+1) % n]
            d = seg_seg_dist(p, p, tuple(a), tuple(b))
            if d < best[0]:
                best = (d, i)
        return best[1]
    ea, eb = edge_of(pa), edge_of(pb)
    fwd = []
    i = (ea + 1) % n
    while True:
        fwd.append(poly[i])
        if i == eb:
            break
        i = (i + 1) % n
        if len(fwd) > n:
            break
    bwd = []
    i = ea
    while True:
        bwd.append(poly[i])
        if i == (eb + 1) % n:
            break
        i = (i - 1) % n
        if len(bwd) > n:
            break
    def plen(path):
        pts = [pa] + [tuple(p) for p in path] + [pb]
        return sum(math.hypot(pts[k+1][0]-pts[k][0], pts[k+1][1]-pts[k][1])
                   for k in range(len(pts)-1))
    return min((plen(fwd), fwd), (plen(bwd), bwd))[1]

wrap_fail = set()
wrapped = set()      # (edge_idx, pad_idx) pairs already detoured
no_smooth = set()    # detour corner nodes: smoothing would cut back into the pad
WRAP_M = 0.025       # route margin beyond the detection offset
def wrap_pass():
    """For each chain crossing an other-net pad, splice in the shorter
    boundary detour around the pad inflated by rule clearance."""
    global nbr
    fixed = 0
    by_edge2 = defaultdict(list)
    for s in segs:
        by_edge2[s['edge']].append(s)
    for ei in list(by_edge2):
        e = edges[ei]
        # order the chain
        ss = by_edge2[ei]
        nxt = defaultdict(list)
        for s in ss:
            nxt[s['a']].append(s); nxt[s['b']].append(s)
        cur = e['a']
        order = [cur]
        used = set()
        while len(used) < len(ss):
            cand = [s for s in nxt[cur] if id(s) not in used]
            if not cand:
                break
            s = cand[0]
            used.add(id(s))
            cur = s['b'] if s['a'] == cur else s['a']
            order.append(cur)
        pts = [tuple(P[nid]) for nid in order]
        # find pads this chain penetrates on its layer
        for pi, p in enumerate(O['pads']):
            if (ei, pi) in wrapped:
                continue
            if p['net'] == e['net'] or e['layer'] not in p['layers']:
                continue
            if not any(abs(p['x']-q[0]) < 6 and abs(p['y']-q[1]) < 6 for q in pts):
                continue
            rule = CLR + e['w']/2
            poly = offset_convex(p['pts'], rule)
            if len(poly) < 3:
                continue
            ins = [point_in_poly(q[0], q[1], poly) for q in pts]
            if not any(ins):
                # strict interior crossing: midpoint of the clipped window inside
                crossed = False
                for k in range(len(pts)-1):
                    i1, i2, ts = seg_poly_hits(pts[k], pts[k+1], poly)
                    if len(ts) >= 2 and ts[-1]-ts[0] > 1e-6:
                        tm = (ts[0] + ts[-1]) / 2
                        mx = pts[k][0] + (pts[k+1][0]-pts[k][0])*tm
                        my = pts[k][1] + (pts[k+1][1]-pts[k][1])*tm
                        if point_in_poly(mx, my, offset_convex(p['pts'], rule - 0.01)):
                            crossed = True
                            break
                if not crossed:
                    continue
            # anchors trapped inside are unsolvable by deformation
            trapped = [order[k] for k in range(len(pts)) if ins[k] and order[k] in anchor]
            if trapped:
                wrap_fail.add((e['net'], f"{p['ref']}.{p['num']}", 'anchor inside'))
                wrapped.add((ei, pi))
                continue
            poly = offset_convex(p['pts'], rule + WRAP_M)   # route strictly outside
            # entry/exit points on the chain
            first_k = last_k = None
            entry = exit_ = None
            for k in range(len(pts)-1):
                i1, i2, ts = seg_poly_hits(pts[k], pts[k+1], poly)
                hit = i1 or i2 or (len(ts) >= 2 and ts[1]-ts[0] > 1e-6)
                if hit and first_k is None:
                    first_k = k
                    t0 = ts[0] if ts and not i1 else 0.0
                    entry = (pts[k][0] + (pts[k+1][0]-pts[k][0])*t0,
                             pts[k][1] + (pts[k+1][1]-pts[k][1])*t0)
                if hit:
                    last_k = k
                    t1 = ts[-1] if ts and not i2 else 1.0
                    exit_ = (pts[k][0] + (pts[k+1][0]-pts[k][0])*t1,
                             pts[k][1] + (pts[k+1][1]-pts[k][1])*t1)
            if first_k is None or entry is None or exit_ is None:
                continue
            detour = boundary_paths(poly, entry, exit_)
            new_pts = pts[:first_k+1] + [list(entry)] + detour + [list(exit_)] + pts[last_k+1:]
            # rebuild this edge's chain: reuse first/last node ids, new interior
            keep_a, keep_b = order[0], order[-1]
            for s in ss:
                segs.remove(s)
            interior = []
            for q in new_pts[1:-1]:
                nid = len(P)
                P.append([q[0], q[1]])
                is_via.append(False); via_dia.append(0); via_drill.append(0)
                node_net.append(e['net'])
                no_smooth.add(nid)
                interior.append(nid)
            chain_ids = [keep_a] + interior + [keep_b]
            for k in range(len(chain_ids)-1):
                segs.append(dict(a=chain_ids[k], b=chain_ids[k+1], layer=e['layer'],
                                 w=e['w'], net=e['net'], edge=ei))
            wrapped.add((ei, pi))
            fixed += 1
            break   # re-wrap next pass if more pads involved
    if fixed:
        nbr = defaultdict(list)
        for s in segs:
            nbr[s['a']].append(s['b']); nbr[s['b']].append(s['a'])
    return fixed

# ---- diagnostics ---------------------------------------------------------------
if '--diag' in sys.argv:
    from collections import Counter
    cnt = Counter(); samples = {}
    def diag_seg(si, s):
        (x1, y1), (x2, y2) = seg_pts(s)
        seen = set()
        for c in cells_for_seg(x1, y1, x2, y2, 0.6):
            for kind, idx in static_grid.get(c, ()):
                if (kind, idx) in seen: continue
                seen.add((kind, idx))
                p1, p2 = (x1, y1), (x2, y2)
                if kind == 'T':
                    t = O['tracks'][idx]
                    if t['layer'] != s['layer'] or t['net'] == s['net']: continue
                    d = seg_seg_dist(p1, p2, tuple(t['a']), tuple(t['b']))
                    rule = CLR + (t['w'] + s['w'])/2
                    if d < rule:
                        k = ('T', s['net'], t['net'])
                        cnt[k] += 1
                        samples.setdefault(k, (round(d,3), round(rule,3), round(x1,2), round(y1,2)))
                elif kind == 'V':
                    v = O['vias'][idx]
                    if v['net'] == s['net']: continue
                    d = seg_seg_dist(p1, p2, (v['x'], v['y']), (v['x'], v['y']))
                    rule = max(CLR + v['dia']/2 + s['w']/2, HOLE_CLR + v['drill']/2 + s['w']/2)
                    if d < rule:
                        k = ('V', s['net'], v['net'])
                        cnt[k] += 1
                        samples.setdefault(k, (round(d,3), round(rule,3), round(x1,2), round(y1,2)))
                elif kind == 'PE':
                    ax, ay, bx, by, pi = pad_edges[idx]
                    p = O['pads'][pi]
                    if p['net'] == s['net']: continue
                    if s['layer'] in p['layers']:
                        d = seg_seg_dist(p1, p2, (ax, ay), (bx, by))
                        if point_in_poly((x1+x2)/2, (y1+y2)/2, p['pts']): d = 0.0
                        rule = CLR + s['w']/2
                        if d < rule:
                            k = ('P', s['net'], f"{p['ref']}.{p['num']}:{p['net']}")
                            cnt[k] += 1
                            samples.setdefault(k, (round(d,3), round(rule,3), round(x1,2), round(y1,2)))
    for si, s in enumerate(segs):
        diag_seg(si, s)
    print(f'--diag @alpha=0: {sum(cnt.values())} true seg violations, {len(cnt)} distinct pairs')
    for k, n in cnt.most_common(25):
        print(f'  {n:4d}  {k}  sample(d,rule,x,y)={samples[k]}')
    sys.exit(0)

# ---- main loop ----------------------------------------------------------------
def smoothstep(t):
    return t*t*(3 - 2*t)

def sweep():
    global violations, true_viol
    violations = 0
    true_viol = 0
    # 1. rubber-band smoothing on free degree-2 nodes
    for nid, ns in nbr.items():
        if nid in anchor or is_via[nid] or len(ns) != 2 or nid in no_smooth:
            continue
        mx = (P[ns[0]][0] + P[ns[1]][0]) / 2
        my = (P[ns[0]][1] + P[ns[1]][1]) / 2
        P[nid][0] += (mx - P[nid][0]) * eta_now
        P[nid][1] += (my - P[nid][1]) * eta_now
    # 2. stretch cap
    for s in segs:
        (x1, y1), (x2, y2) = seg_pts(s)
        L = math.hypot(x2-x1, y2-y1)
        cap = SUB * 1.6
        if L > cap:
            ex = (L - cap) / 2
            ux, uy = (x2-x1)/L, (y2-y1)/L
            if s['a'] not in anchor:
                P[s['a']][0] += ux*ex; P[s['a']][1] += uy*ex
            if s['b'] not in anchor:
                P[s['b']][0] -= ux*ex; P[s['b']][1] -= uy*ex
    # 3. dyn grid + dyn-dyn clearance
    dyn_grid = defaultdict(list)
    for si, s in enumerate(segs):
        (x1, y1), (x2, y2) = seg_pts(s)
        for c in cells_for_seg(x1, y1, x2, y2, 0.6):
            dyn_grid[c].append(si)
    done = set()
    for c, slist in dyn_grid.items():
        for i in slist:
            si, sj_pts = segs[i], seg_pts(segs[i])
            for j in slist:
                if j <= i or (i, j) in done:
                    continue
                done.add((i, j))
                sj = segs[j]
                if si['layer'] != sj['layer'] or si['net'] == sj['net']:
                    continue
                if si['edge'] == sj['edge']:
                    continue
                p1, p2 = sj_pts
                q1, q2 = seg_pts(sj)
                d = seg_seg_dist(p1, p2, q1, q2)
                rule = CLR + (si['w'] + sj['w'])/2
                if d < rule + EXTRA:
                    ux, uy = closest_dir(p1, p2, q1, q2)
                    push_nodes(i, ux, uy, (rule + EXTRA - d) * RELAX / 2)
                    push_nodes(j, -ux, -uy, (rule + EXTRA - d) * RELAX / 2)
                    violations += 1
                    if d < rule:
                        globals()['true_viol'] += 1
    # 4. dyn vs static
    for si, s in enumerate(segs):
        (x1, y1), (x2, y2) = seg_pts(s)
        seen = set()
        for c in cells_for_seg(x1, y1, x2, y2, 0.6):
            for kind, idx in static_grid.get(c, ()):
                if (kind, idx) in seen:
                    continue
                seen.add((kind, idx))
                project_pair_dyn_static(si, kind, idx)
    # 5. dynamic vias
    for nid in range(len(nodes)):
        if is_via[nid]:
            project_via_node(nid)
    return violations

# Wraps are opt-in: in a dense field a single-obstacle detour tends to plow
# into the neighboring pads. Default mode is gentle PBD + selective emission.
WRAP = '--wrap' in sys.argv
if WRAP:
    print(f'initial wrap pass: {wrap_pass()} chains detoured around pads')

for step in range(STEPS):
    a = smoothstep((step + 1) / STEPS)
    for nid, (ox, oy, tx, ty) in anchor.items():
        P[nid][0] = ox + (tx - ox) * a
        P[nid][1] = oy + (ty - oy) * a
    for _ in range(SWEEPS):
        v = sweep()
    if WRAP and step % 10 == 9:
        wrap_pass()
        resubdivide()
    if step % 10 == 0 or step == STEPS - 1:
        print(f'step {step+1}/{STEPS} alpha={a:.2f} pushes={v} '
              f'true_violations={true_viol} nodes={len(P)}')

print('polish...')
eta_now = 0.06
for k in range(POLISH):
    v = sweep()
    if k % 10 == 0 or true_viol == 0:
        print(f'polish {k}: pushes={v} true_violations={true_viol} nodes={len(P)}')
    if true_viol == 0:
        break

# ---- final classification: which chains/vias are actually clean? ---------------
TOL = 1e-4
bad_edges = set()
bad_vias = set()

def classify():
    dyn_grid = defaultdict(list)
    for si, s in enumerate(segs):
        (x1, y1), (x2, y2) = seg_pts(s)
        for c in cells_for_seg(x1, y1, x2, y2, 0.6):
            dyn_grid[c].append(si)
    done = set()
    for c, slist in dyn_grid.items():
        for i in slist:
            p1, p2 = seg_pts(segs[i])
            for j in slist:
                if j <= i or (i, j) in done:
                    continue
                done.add((i, j))
                si_, sj = segs[i], segs[j]
                if si_['layer'] != sj['layer'] or si_['net'] == sj['net']:
                    continue
                q1, q2 = seg_pts(sj)
                if seg_seg_dist(p1, p2, q1, q2) < CLR + (si_['w']+sj['w'])/2 - TOL:
                    bad_edges.add(si_['edge']); bad_edges.add(sj['edge'])
    for si, s in enumerate(segs):
        (x1, y1), (x2, y2) = seg_pts(s)
        p1, p2 = (x1, y1), (x2, y2)
        seen = set()
        for c in cells_for_seg(x1, y1, x2, y2, 0.6):
            for kind, idx in static_grid.get(c, ()):
                if (kind, idx) in seen:
                    continue
                seen.add((kind, idx))
                if kind == 'T':
                    t = O['tracks'][idx]
                    if t['layer'] != s['layer'] or t['net'] == s['net']:
                        continue
                    if seg_seg_dist(p1, p2, tuple(t['a']), tuple(t['b'])) < \
                            CLR + (t['w']+s['w'])/2 - TOL:
                        bad_edges.add(s['edge'])
                elif kind == 'V':
                    v = O['vias'][idx]
                    if v['net'] == s['net']:
                        continue
                    d = seg_seg_dist(p1, p2, (v['x'], v['y']), (v['x'], v['y']))
                    if d < max(CLR + v['dia']/2 + s['w']/2,
                               HOLE_CLR + v['drill']/2 + s['w']/2) - TOL:
                        bad_edges.add(s['edge'])
                elif kind == 'PE':
                    ax, ay, bx, by, pi = pad_edges[idx]
                    p = O['pads'][pi]
                    if p['net'] == s['net']:
                        continue
                    if s['layer'] in p['layers']:
                        d = seg_seg_dist(p1, p2, (ax, ay), (bx, by))
                        if point_in_poly((x1+x2)/2, (y1+y2)/2, p['pts']):
                            d = 0.0
                        if d < CLR + s['w']/2 - TOL:
                            bad_edges.add(s['edge'])
                    if p['drill']:
                        if seg_seg_dist(p1, p2, (p['x'], p['y']), (p['x'], p['y'])) < \
                                HOLE_CLR + p['drill']/2 + s['w']/2 - TOL:
                            bad_edges.add(s['edge'])
    for nid in range(len(nodes)):
        if not is_via[nid]:
            continue
        x, y = P[nid]
        r_cu, r_hole, net = via_dia[nid]/2, via_drill[nid]/2, node_net[nid]
        seen = set()
        for c in cells_for_seg(x, y, x, y, 1.0):
            for kind, idx in static_grid.get(c, ()):
                if (kind, idx) in seen:
                    continue
                seen.add((kind, idx))
                if kind == 'T':
                    t = O['tracks'][idx]
                    if t['net'] == net:
                        continue
                    d = seg_seg_dist((x, y), (x, y), tuple(t['a']), tuple(t['b']))
                    if d < max(CLR + r_cu + t['w']/2, HOLE_CLR + r_hole + t['w']/2) - TOL:
                        bad_vias.add(nid)
                elif kind == 'V':
                    v = O['vias'][idx]
                    d = math.hypot(x-v['x'], y-v['y'])
                    rule = HOLE_CLR + r_hole + v['drill']/2
                    if v['net'] != net:
                        rule = max(rule, CLR + r_cu + v['dia']/2)
                    if d < rule - TOL:
                        bad_vias.add(nid)
                elif kind == 'PE':
                    ax, ay, bx, by, pi = pad_edges[idx]
                    p = O['pads'][pi]
                    if p['net'] == net:
                        continue
                    d = seg_seg_dist((x, y), (x, y), (ax, ay), (bx, by))
                    if point_in_poly(x, y, p['pts']):
                        d = 0.0
                    if d < max(CLR + r_cu, HOLE_CLR + r_hole) - TOL:
                        bad_vias.add(nid)
                    if p['drill'] and math.hypot(x-p['x'], y-p['y']) < \
                            HOLE_CLR + r_hole + p['drill']/2 - TOL:
                        bad_vias.add(nid)
        # dynamic via vs dynamic segs
        for c in cells_for_seg(x, y, x, y, 0.8):
            for si in dyn_grid.get(c, ()):
                s = segs[si]
                if s['net'] == net:
                    continue
                d = seg_seg_dist((x, y), (x, y), *seg_pts(s))
                if d < max(CLR + r_cu + s['w']/2, HOLE_CLR + r_hole + s['w']/2) - TOL:
                    bad_vias.add(nid)
                    bad_edges.add(s['edge'])

classify()
print(f'classification: {len(bad_edges)}/{len(edges)} chains dirty, '
      f'{len(bad_vias)}/{sum(is_via)} vias dirty')

# ---- emit solution -------------------------------------------------------------
# walk each edge's sub-chain by connectivity (splits appended out of order)
by_edge = defaultdict(list)
for s in segs:
    by_edge[s['edge']].append(s)
out_edges = []
for ei, e in enumerate(edges):
    ss = by_edge[ei]
    nxt = defaultdict(list)
    for s in ss:
        nxt[s['a']].append(s); nxt[s['b']].append(s)
    cur = e['a']
    pl = [P[cur][:]]
    used = set()
    while len(used) < len(ss):
        cand = [s for s in nxt[cur] if id(s) not in used]
        if not cand:
            break
        s = cand[0]
        used.add(id(s))
        cur = s['b'] if s['a'] == cur else s['a']
        pl.append(P[cur][:])
    out_edges.append(dict(layer=e['layer'], w=e['w'], net=e['net'], pts=pl,
                          clean=ei not in bad_edges))
out_vias = [dict(x=P[i][0], y=P[i][1], dia=via_dia[i], drill=via_drill[i],
                 net=node_net[i], clean=i not in bad_vias)
            for i in range(len(nodes)) if is_via[i]]
n_clean = sum(1 for e in out_edges if e['clean'])
save_json('solution.json', dict(edges=out_edges, vias=out_vias,
                                residual=v, F=F_CU, B=B_CU))
print(f'wrote data/solution.json: {n_clean}/{len(out_edges)} chains clean, '
      f'{sum(1 for w_ in out_vias if w_["clean"])}/{len(out_vias)} vias clean')
