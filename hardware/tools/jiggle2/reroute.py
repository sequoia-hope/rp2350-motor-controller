#!/usr/bin/env python3
"""jiggle2 stage 6: homotopic reroute of referee-withdrawn copper.

Relaxation moves copper it already has; it cannot re-plan geometry around a
pad field that landed on top of an old route. This stage does exactly that
re-plan, and nothing else: the withdrawn chains are grouped into single-net
clusters, each cluster's terminals (pad-bound anchors, junctions shared with
kept copper, and its withdrawn vias after relocation) are reconnected by
grid A* on a clearance model built from everything already on the board.
Where the cluster's original path between two terminals is known and stays
on one layer, the search runs in the covering space — the A* state carries
the reduced crossing word rel pinned punctures, and only a path in the SAME
homotopy class as the pre-rip route is accepted (Maley's regime: fixed
topology, new geometry). Attach paths and fallbacks route free-class; every
emitted path records whether its class matched (class_ok).

No new objects are created: vias may relocate, chains get new polylines
appended to solution.json (originals stay withdrawn), and kicad DRC remains
the referee via emit.py/drcloop.py --all afterwards.
"""
import math, sys, heapq
from collections import defaultdict
from common import (NM, REGION, VARIANT, load_json, save_json, replay_drop,
                    path_word, reduce_word)

GRID = 0.10          # mm
M = 0.03             # model margin over exact rule
VIA_R = 2.0          # via relocation search radius, mm
MAX_POP = 250000     # A* expansion cap per route
WORD_SLACK = 2       # allowed extra letters over the target word during search

G = load_json('graph.json')
O = load_json('obstacles.json')
sol = load_json('solution.json')
nodes = G['nodes']
kept, drop = replay_drop(G, O)
assert len(sol['edges']) >= len(kept), 'run on a refereed solution.json'
CLR, HOLE_CLR = O['clearance'], O['hole_clearance']
EDGE, ECLR = O.get('board_edge'), O.get('edge_clearance', 0.3)
COPPER = sorted({e['layer'] for e in G['edges']} | {t['layer'] for t in O['tracks']})

X0, Y0, X1, Y1 = REGION['x0'], REGION['y0'], REGION['x1'], REGION['y1']
W = int((X1 - X0) / GRID) + 1
H = int((Y1 - Y0) / GRID) + 1
def xy(idx):  return (X0 + (idx % W) * GRID, Y0 + (idx // W) * GRID)
def at(x, y): return int(round((y - Y0) / GRID)) * W + int(round((x - X0) / GRID))
def inb(i, j): return 0 <= i < W and 0 <= j < H

def pt_seg(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)

def point_in_poly(x, y, pts):
    inside = False
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi = pts[i]; xj, yj = pts[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi:
            inside = not inside
        j = i
    return inside

# ---- static + emitted copper items per layer ---------------------------------
# kind: poly (pads), seg (tracks / emitted chains), disc (vias). Holes are
# separate discs with the hole rule folded into an effective halfwidth.
items = {lay: [] for lay in COPPER}
holes = []           # (net, x, y, drill/2) — block every layer
for p in O['pads']:
    lays = COPPER if p['drill'] else p['layers']   # drilled pads ring all layers
    for lay in lays:
        items[lay].append(('poly', p['net'], p['pts']))
    if p['drill']:
        holes.append((p['net'], p['x'], p['y'], p['drill'] / 2))
for t in O['tracks']:
    items[t['layer']].append(('seg', t['net'], (t['a'][0], t['a'][1], t['b'][0], t['b'][1], t['w'] / 2)))
for v in O['vias']:
    for lay in COPPER:
        items[lay].append(('disc', v['net'], (v['x'], v['y'], v['dia'] / 2)))
    holes.append((v['net'], v['x'], v['y'], v['drill'] / 2))

all_mode = any(o.get('withdrawn') for o in sol['edges'] + sol['vias'])
for se in sol['edges']:
    if se.get('withdrawn') or not (se['clean'] or all_mode):
        continue
    p = se['pts']
    for k in range(len(p) - 1):
        items[se['layer']].append(('seg', se['net'],
                                   (p[k][0], p[k][1], p[k+1][0], p[k+1][1], se['w'] / 2)))
for sv in sol['vias']:
    if sv.get('withdrawn') or not (sv['clean'] or all_mode):
        continue
    for lay in COPPER:
        items[lay].append(('disc', sv['net'], (sv['x'], sv['y'], sv['dia'] / 2)))
    holes.append((sv['net'], sv['x'], sv['y'], sv['drill'] / 2))

# ---- per-(layer, net, w) blocked masks, cached -------------------------------
_masks = {}
def stamp_disc(m, x, y, r):
    i0 = max(0, int((x - r - X0) / GRID)); i1 = min(W - 1, int((x + r - X0) / GRID) + 1)
    j0 = max(0, int((y - r - Y0) / GRID)); j1 = min(H - 1, int((y + r - Y0) / GRID) + 1)
    r2 = r * r
    for j in range(j0, j1 + 1):
        cy = Y0 + j * GRID
        base = j * W
        for i in range(i0, i1 + 1):
            cx = X0 + i * GRID
            if (cx - x) ** 2 + (cy - y) ** 2 <= r2:
                m[base + i] = 1

def stamp_seg(m, ax, ay, bx, by, r):
    x0 = min(ax, bx) - r; x1 = max(ax, bx) + r
    y0 = min(ay, by) - r; y1 = max(ay, by) + r
    i0 = max(0, int((x0 - X0) / GRID)); i1 = min(W - 1, int((x1 - X0) / GRID) + 1)
    j0 = max(0, int((y0 - Y0) / GRID)); j1 = min(H - 1, int((y1 - Y0) / GRID) + 1)
    for j in range(j0, j1 + 1):
        cy = Y0 + j * GRID
        base = j * W
        for i in range(i0, i1 + 1):
            if pt_seg(X0 + i * GRID, cy, ax, ay, bx, by) <= r:
                m[base + i] = 1

def stamp_poly(m, pts, r):
    xs = [q[0] for q in pts]; ys = [q[1] for q in pts]
    i0 = max(0, int((min(xs) - r - X0) / GRID)); i1 = min(W - 1, int((max(xs) + r - X0) / GRID) + 1)
    j0 = max(0, int((min(ys) - r - Y0) / GRID)); j1 = min(H - 1, int((max(ys) + r - Y0) / GRID) + 1)
    n = len(pts)
    for j in range(j0, j1 + 1):
        cy = Y0 + j * GRID
        base = j * W
        for i in range(i0, i1 + 1):
            cx = X0 + i * GRID
            if point_in_poly(cx, cy, pts):
                m[base + i] = 1
                continue
            for k in range(n):
                a, b = pts[k], pts[(k + 1) % n]
                if pt_seg(cx, cy, a[0], a[1], b[0], b[1]) <= r:
                    m[base + i] = 1
                    break

def mask(layer, net, w):
    key = (layer, net, round(w, 4))
    if key in _masks:
        return _masks[key]
    m = bytearray(W * H)
    if EDGE:
        em = ECLR + w / 2 + M
        for j in range(H):
            cy = Y0 + j * GRID
            if cy < EDGE[1] + em or cy > EDGE[3] - em:
                for i in range(W): m[j * W + i] = 1
        for i in range(W):
            cx = X0 + i * GRID
            if cx < EDGE[0] + em or cx > EDGE[2] - em:
                for j in range(H): m[j * W + i] = 1
    r_tr = CLR + w / 2 + M
    for kind, inet, g in items[layer]:
        if inet == net:
            continue
        if kind == 'poly':
            stamp_poly(m, g, r_tr)
        elif kind == 'seg':
            stamp_seg(m, g[0], g[1], g[2], g[3], r_tr + g[4])
        else:
            stamp_disc(m, g[0], g[1], r_tr + g[2])
    for hnet, hx, hy, hr in holes:
        if hnet == net:
            continue
        stamp_disc(m, hx, hy, HOLE_CLR + hr + w / 2 + M)
    _masks[key] = m
    return m

# ---- dynamic routed copper (grows as clusters route) -------------------------
routed = {lay: [] for lay in COPPER}     # (net, ax, ay, bx, by, hw_eff)
rhash = {lay: defaultdict(list) for lay in COPPER}
RCELL = 1.0
def routed_add(lay, net, ax, ay, bx, by, hw):
    idx = len(routed[lay])
    routed[lay].append((net, ax, ay, bx, by, hw))
    for cx in range(int(min(ax, bx) // RCELL) - 1, int(max(ax, bx) // RCELL) + 2):
        for cy in range(int(min(ay, by) // RCELL) - 1, int(max(ay, by) // RCELL) + 2):
            rhash[lay][(cx, cy)].append(idx)

def routed_clear(lay, net, x, y, w):
    seen = set()
    for i in rhash[lay].get((int(x // RCELL), int(y // RCELL)), ()):
        if i in seen: continue
        seen.add(i)
        rnet, ax, ay, bx, by, hw = routed[lay][i]
        if rnet != net and pt_seg(x, y, ax, ay, bx, by) < CLR + w / 2 + hw + M:
            return False
    return True

def passable(idx, lay, net, w, m):
    if m[idx]:
        return False
    x, y = xy(idx)
    return routed_clear(lay, net, x, y, w)

# ---- A*, optionally in the covering space (word state) -----------------------
DIRS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, 1.41421356), (1, -1, 1.41421356), (-1, 1, 1.41421356), (-1, -1, 1.41421356)]

def move_letters(x1, x2, y1, y2, cands):
    out = []
    for pi, p in cands:
        px, py = p[0], p[1]
        if (x1 < px) != (x2 < px):
            t = (px - x1) / (x2 - x1)
            if y1 + (y2 - y1) * t > py:
                out.append((t, pi + 1 if x2 > x1 else -(pi + 1)))
    out.sort()
    return [l for _, l in out]

def astar(lay, net, w, start_xy, goals, cands=None, target=None):
    """goals: set of cell idx. target: word tuple to require (with cands), or
    None for free-class. Returns list of cell idx or None."""
    m = mask(lay, net, w)
    si = sj = None
    for rad in range(0, 6):
        best = None
        for dj in range(-rad, rad + 1):
            for di in range(-rad, rad + 1):
                if max(abs(di), abs(dj)) != rad:
                    continue
                i = int(round((start_xy[0] - X0) / GRID)) + di
                j = int(round((start_xy[1] - Y0) / GRID)) + dj
                if inb(i, j) and passable(j * W + i, lay, net, w, m):
                    d = math.hypot(di, dj)
                    if best is None or d < best[0]:
                        best = (d, i, j)
        if best:
            si, sj = best[1], best[2]
            break
    if si is None:
        return None
    start = sj * W + si
    gx = [xy(g) for g in goals]
    def h(idx):
        x, y = xy(idx)
        return min(max(abs(x - a), abs(y - b)) for a, b in gx) if len(gx) <= 24 else 0.0
    cap = (len(target) + WORD_SLACK) if target is not None else 0
    w0 = ()
    openq = [(h(start), 0.0, start, w0)]
    gbest = {(start, w0): 0.0}
    parent = {}
    pops = 0
    while openq and pops < MAX_POP:
        f, g, idx, word = heapq.heappop(openq)
        pops += 1
        if gbest.get((idx, word), 1e18) < g - 1e-12:
            continue
        if idx in goals and (target is None or word == target):
            path = [(idx, word)]
            while path[-1] in parent:
                path.append(parent[path[-1]])
            return [p[0] for p in reversed(path)]
        x, y = xy(idx)
        i, j = idx % W, idx // W
        for di, dj, c in DIRS:
            ni, nj = i + di, j + dj
            if not inb(ni, nj):
                continue
            nidx = nj * W + ni
            if not passable(nidx, lay, net, w, m):
                continue
            if di and dj:   # no corner cutting
                if m[j * W + ni] or m[nj * W + i]:
                    continue
            nword = word
            if target is not None:
                nx, ny = xy(nidx)
                ls = move_letters(x, nx, y, ny, cands)
                if ls:
                    nword = tuple(reduce_word(list(word) + ls))
                    if len(nword) > cap:
                        continue
            ng = g + c * GRID
            if ng < gbest.get((nidx, nword), 1e18) - 1e-12:
                gbest[(nidx, nword)] = ng
                parent[(nidx, nword)] = (idx, word)
                heapq.heappush(openq, (ng + h(nidx), ng, nidx, nword))
    return None

def smooth(pts, lay, net, w, cands):
    """Greedy shortcutting; each shortcut must stay passable and keep the
    crossing word of the replaced sub-path (class-preserving by construction)."""
    m = mask(lay, net, w)
    def seg_ok(a, b):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(2, int(L / (GRID * 0.4)))
        for k in range(n + 1):
            t = k / n
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            i, j = int(round((x - X0) / GRID)), int(round((y - Y0) / GRID))
            if not inb(i, j) or m[j * W + i] or not routed_clear(lay, net, x, y, w):
                return False
        return True
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1:
            if seg_ok(pts[i], pts[j]) and \
               path_word(pts[i:j + 1], cands) == path_word([pts[i], pts[j]], cands):
                break
            j -= 1
        out.append(pts[j])
        i = j
    return out

# ---- pinned punctures (same system as homotopy.py) ---------------------------
from common import load_board, BOARD_PRE, BOARD_BASE, pinned_parts, build_punctures
pre = load_board(BOARD_PRE)
base = load_board(BOARD_BASE)
pinned, moved, appeared = pinned_parts(pre, base)
punct = build_punctures(O, pinned, COPPER)
p_by_layer = {lay: [(i, p) for i, p in enumerate(punct) if lay in p[2]]
              for lay in COPPER}
def local_cands(lay, net, pts_groups, margin=2.0):
    xs = [q[0] for g in pts_groups for q in g]
    ys = [q[1] for g in pts_groups for q in g]
    x0, x1 = min(xs) - margin, max(xs) + margin
    y0, y1 = min(ys) - margin, max(ys) + margin
    return [(i, p) for i, p in p_by_layer[lay]
            if p[3] != net and x0 <= p[0] <= x1 and y0 <= p[1] <= y1]

# ---- withdrawn sets, final node positions ------------------------------------
via_nids = [i for i, n in enumerate(nodes) if n['kind'] == 'via']
node_final = {}
for (ei, e), se in zip(kept, sol['edges']):
    node_final[e['a']] = tuple(se['pts'][0])
    node_final[e['b']] = tuple(se['pts'][-1])
for nid, sv in zip(via_nids, sol['vias']):
    node_final[nid] = (sv['x'], sv['y'])
for i, n in enumerate(nodes):
    if n['bind'] and n['target']:
        node_final[i] = tuple(n['target'])

wd = [(si, ei, e, se) for si, ((ei, e), se) in enumerate(zip(kept, sol['edges']))
      if se.get('withdrawn')]
wd_vias = [(vi, nid, sv) for vi, (nid, sv) in enumerate(zip(via_nids, sol['vias']))
           if sv.get('withdrawn')]
print(f'withdrawn: {len(wd)} chains, {len(wd_vias)} vias '
      f'({VARIANT or "full"} variant, grid {GRID}mm, {W}x{H} cells)')

# ---- 1. relocate withdrawn vias ----------------------------------------------
relocated = failed_vias = 0
via_ok = {}
for vi, nid, sv in wd_vias:
    net, dia, drill = sv['net'], sv['dia'], sv['drill']
    ms = [mask(lay, net, dia) for lay in COPPER]
    found = None
    steps = int(VIA_R / GRID)
    cand = sorted((di * di + dj * dj, di, dj)
                  for di in range(-steps, steps + 1) for dj in range(-steps, steps + 1))
    ci = int(round((sv['x'] - X0) / GRID)); cj = int(round((sv['y'] - Y0) / GRID))
    for d2, di, dj in cand:
        i, j = ci + di, cj + dj
        if not inb(i, j):
            continue
        idx = j * W + i
        if all(not m[idx] for m in ms) and \
           all(routed_clear(lay, net, X0 + i * GRID, Y0 + j * GRID, dia) for lay in COPPER):
            found = (X0 + i * GRID, Y0 + j * GRID)
            break
    if found:
        sv['x'], sv['y'] = found
        sv['withdrawn'] = False
        sv['clean'] = True
        sv['rerouted'] = True
        node_final[nid] = found
        via_ok[nid] = found
        # annular disc covers the hole rule too: dia/2 >= drill/2 + (HOLE_CLR-CLR)
        for lay in COPPER:
            routed_add(lay, net, found[0], found[1], found[0], found[1],
                       max(dia / 2, drill / 2 + (HOLE_CLR - CLR)))
        relocated += 1
    else:
        failed_vias += 1
print(f'via relocation: {relocated} placed, {failed_vias} stuck')

# ---- 2. cluster withdrawn chains ---------------------------------------------
parent = {}
def find(x):
    while parent.setdefault(x, x) != x:
        parent[x] = parent[parent[x]]
        x = parent[x]
    return x
bynode = defaultdict(list)
for si, ei, e, se in wd:
    parent[si] = si
    bynode[e['a']].append(si); bynode[e['b']].append(si)
for n, sis in bynode.items():
    for s2 in sis[1:]:
        parent[find(s2)] = find(sis[0])
clusters = defaultdict(list)
for si, ei, e, se in wd:
    clusters[find(si)].append((si, ei, e, se))

kept_ok_nodes = defaultdict(set)      # node -> layers of adjacent kept copper
for (ei, e), se in zip(kept, sol['edges']):
    if not se.get('withdrawn'):
        kept_ok_nodes[e['a']].add(e['layer'])
        kept_ok_nodes[e['b']].add(e['layer'])
pad_layers = {}
for p in O['pads']:
    pad_layers[(p['ref'], p['num'])] = COPPER if p['drill'] else p['layers']

new_entries = []
stats = dict(clusters=0, skipped=0, paths=0, class_kept=0, class_free=0,
             class_changed=0, fails=0)
order = sorted(clusters.values(), key=lambda ms: sum(
    math.hypot(nodes[e['b']]['x'] - nodes[e['a']]['x'],
               nodes[e['b']]['y'] - nodes[e['a']]['y']) for _, _, e, _ in ms))

for members in order:
    net = members[0][3]['net']
    cn = sorted({n for _, _, e, _ in members for n in (e['a'], e['b'])})
    wbylay = defaultdict(lambda: 0.2)
    for _, _, e, se in members:
        wbylay[e['layer']] = max(wbylay[e['layer']], e['w'])
    # terminals: (node, layers, exact position)
    terms = []
    for n in cn:
        nd = nodes[n]
        lays = set(kept_ok_nodes.get(n, ()))
        if nd['bind'] and nd['target']:
            lays |= set(pad_layers.get(tuple(nd['bind']), COPPER))
        if nd['kind'] == 'via' and (n in via_ok or not
                                    (n in {nid for _, nid, _ in wd_vias} and n not in via_ok)):
            lays |= set(COPPER)
        if lays and n in node_final:
            terms.append((n, sorted(lays), node_final[n]))
    stats['clusters'] += 1
    if len(terms) < 2:
        stats['skipped'] += 1
        continue
    # old-geometry adjacency for class targets
    oadj = defaultdict(list)
    for _, _, e, _ in members:
        oadj[e['a']].append((e['b'], e['layer']))
        oadj[e['b']].append((e['a'], e['layer']))
    def old_path(a, b):
        """(node id path, single layer or None) via BFS through the cluster."""
        prev = {a: None}
        q = [a]
        while q:
            u = q.pop(0)
            if u == b:
                break
            for v, lay in oadj[u]:
                if v not in prev:
                    prev[v] = u
                    q.append(v)
        if b not in prev:
            return None, None
        pth = [b]
        while prev[pth[-1]] is not None:
            pth.append(prev[pth[-1]])
        pth.reverse()
        lays = {lay for u, v0 in zip(pth, pth[1:])
                for v, lay in oadj[u] if v == v0}
        return pth, (lays.pop() if len(lays) == 1 else None)

    # seed: terminal with the most layers (via / drilled pad first)
    terms.sort(key=lambda t: (-len(t[1]),))
    seed = terms[0]
    tree_cells = defaultdict(set)
    for lay in seed[1]:
        tree_cells[lay].add(at(*seed[2]))
    tree_pts = {seed[0]: seed[2]}
    for tn, tlays, tpos in sorted(terms[1:],
                                  key=lambda t: math.hypot(t[2][0] - seed[2][0],
                                                           t[2][1] - seed[2][1])):
        done = False
        for lay in tlays:
            goals = tree_cells.get(lay)
            if not goals:
                continue
            w = wbylay[lay]
            # class target only for the first, terminal-to-seed connection
            target = cands = None
            if len(tree_pts) == 1:
                pth, plady = old_path(tn, seed[0])
                if pth and plady == lay:
                    opts = [(nodes[u]['x'], nodes[u]['y']) for u in pth]
                    cands = local_cands(lay, net, [opts, [tpos, seed[2]]])
                    if cands:
                        tw = path_word([tpos, opts[0]] + opts + [opts[-1], seed[2]], cands)
                        if len(tw) <= 5:
                            target = tuple(tw)
                        else:
                            cands = None
                    else:
                        target = ()
            cpath = astar(lay, net, w, tpos, goals, cands, target)
            klass = 'kept' if target is not None else 'free'
            if cpath is None and target is not None:
                cpath = astar(lay, net, w, tpos, goals)     # free-class fallback
                klass = 'free'
            if cpath is None:
                continue
            pts = [list(tpos)] + [list(xy(c)) for c in cpath[1:]]
            sm = smooth([tuple(q) for q in pts], lay, net, w,
                        cands if cands is not None else
                        local_cands(lay, net, [[tuple(q) for q in pts]]))
            class_ok = None
            if target is not None:
                class_ok = (path_word(sm, cands) == list(target))
            new_entries.append(dict(layer=lay, w=w, net=net,
                                    pts=[list(q) for q in sm], clean=True,
                                    rerouted=True, class_ok=class_ok))
            for k in range(len(sm) - 1):
                routed_add(lay, net, sm[k][0], sm[k][1], sm[k+1][0], sm[k+1][1], w / 2)
            for c in cpath:
                tree_cells[lay].add(c)
            tree_pts[tn] = tpos
            stats['paths'] += 1
            if class_ok is True:
                stats['class_kept'] += 1
            elif class_ok is False:
                stats['class_changed'] += 1
            else:
                stats['class_free'] += 1
            done = True
            break
        if not done:
            stats['fails'] += 1

print(f"clusters {stats['clusters']} (skipped {stats['skipped']} with <2 terminals), "
      f"paths routed {stats['paths']} (class kept {stats['class_kept']}, "
      f"free {stats['class_free']}, changed {stats['class_changed']}), "
      f"unroutable terminals {stats['fails']}")

sol['edges'] = sol['edges'][:len(kept)] + new_entries   # idempotent re-runs
save_json('solution.json', sol)
print(f'wrote solution.json: +{len(new_entries)} rerouted chains, '
      f'{relocated} relocated vias — run emit.py --all && drcloop.py --all')
