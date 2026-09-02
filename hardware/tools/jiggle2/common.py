#!/usr/bin/env python3
"""jiggle2 shared bits: paths, region, geometry helpers."""
import os, sys, math, json, contextlib

HW = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BOARD_CUR = os.path.join(HW, 'rp2350_driver.kicad_pcb')
BOARD_PRE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'board_prerip_2238aca.kicad_pcb')

# Variants (JIGGLE2_VARIANT env): '' = full board; 'nonew' = the 28 rev-B
# footprints removed everywhere (obstacles, base board, referee baseline).
VARIANT = os.environ.get('JIGGLE2_VARIANT', '')
_sfx = f'_{VARIANT}' if VARIANT else ''
BOARD_OUT = os.path.join(HW, f'rp2350_driver_jiggle2{_sfx}.kicad_pcb')
DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'data{_sfx}')
# The board the work copy is built from and DRC-baselined against.
BOARD_BASE = os.path.join(DATA, 'board_base.kicad_pcb') if VARIANT else BOARD_CUR
os.makedirs(DATA, exist_ok=True)

NM = 1e-6  # pcbnew nm -> mm

# Region containing all ripped copper, with working margin (mm).
REGION = dict(x0=110.0, y0=90.0, x1=142.0, y1=130.0)

def in_region(x, y, m=0.0):
    return REGION['x0']-m <= x <= REGION['x1']+m and REGION['y0']-m <= y <= REGION['y1']+m

@contextlib.contextmanager
def quiet_stderr():
    """Silence wx/pcbnew C-level chatter on fd 2."""
    fd = os.dup(2)
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, 2)
    try:
        yield
    finally:
        os.dup2(fd, 2)
        os.close(fd); os.close(devnull)

def load_board(path):
    import pcbnew
    with quiet_stderr():
        return pcbnew.LoadBoard(path)

def tsig(t):
    s, e = t.GetStart(), t.GetEnd()
    a, b = (s.x, s.y), (e.x, e.y)
    if b < a: a, b = b, a
    return ('T', t.GetLayer(), a, b, t.GetWidth())

def vsig(v):
    p = v.GetPosition()
    return ('V', p.x, p.y, v.GetDrill())

def seg_seg_dist(p1, p2, q1, q2):
    """Min distance between segments p1-p2 and q1-q2 (2D tuples, mm)."""
    def pt_seg(p, a, b):
        ax, ay = a; bx, by = b; px, py = p
        dx, dy = bx-ax, by-ay
        L2 = dx*dx + dy*dy
        if L2 < 1e-18: return math.hypot(px-ax, py-ay), a
        t = max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy) / L2))
        cx, cy = ax + t*dx, ay + t*dy
        return math.hypot(px-cx, py-cy), (cx, cy)
    def orient(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
    # proper intersection => distance 0
    d1, d2 = orient(q1, q2, p1), orient(q1, q2, p2)
    d3, d4 = orient(p1, p2, q1), orient(p1, p2, q2)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return 0.0
    return min(pt_seg(p1, q1, q2)[0], pt_seg(p2, q1, q2)[0],
               pt_seg(q1, p1, p2)[0], pt_seg(q2, p1, p2)[0])

def save_json(name, obj):
    path = os.path.join(DATA, name)
    with open(path, 'w') as f:
        json.dump(obj, f)
    return path

def load_json(name):
    with open(os.path.join(DATA, name)) as f:
        return json.load(f)

# ---- shared by homotopy.py / reroute.py ------------------------------------

def replay_drop(G, O):
    """Replay solve.py's deterministic bridge-edge drop so solution.json's
    first len(kept) edges map 1:1 onto graph edges. Returns (kept, drop):
    kept = [(graph_edge_index, edge_dict), ...] in solution order."""
    from collections import defaultdict
    from heapq import heappush, heappop
    nodes, edges = G['nodes'], G['edges']
    adj = defaultdict(list)
    for ei, e in enumerate(edges):
        adj[e['a']].append(ei); adj[e['b']].append(ei)
    comp_of, comps = {}, []
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
    cur_pad_net = {(p['ref'], p['num']): p['net'] for p in O['pads']}
    drop = set()
    for comp in comps:
        votes = defaultdict(int)
        for n in comp:
            b = nodes[n]['bind']
            if b and cur_pad_net.get(tuple(b)):
                votes[cur_pad_net[tuple(b)]] += 1
        if len(votes) <= 1:
            continue
        dist, pq = {}, []
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
                L = math.hypot(nodes[m]['x']-nodes[n]['x'],
                               nodes[m]['y']-nodes[n]['y'])
                if m not in dist or dist[m][0] > dn + L:
                    dist[m] = (dn + L, dist[n][1])
                    heappush(pq, (dn + L, m))
        nnet = {n: dist.get(n, (0, ''))[1] for n in comp}
        for n in comp:
            for ei in adj[n]:
                e = edges[ei]
                na, nb = nnet[e['a']], nnet[e['b']]
                if na and nb and na != nb:
                    drop.add(ei)
    return [(ei, e) for ei, e in enumerate(edges) if ei not in drop], drop

def reduce_word(letters):
    """Free-group reduction: cancel adjacent inverse pairs."""
    st = []
    for l in letters:
        if st and st[-1] == -l:
            st.pop()
        else:
            st.append(l)
    return st

def path_word(pts, cands, closed=False):
    """Reduced crossing word of a polyline against downward vertical rays.
    cands: [(puncture_index, (x_jittered, y, ...)), ...]. Letter +-(i+1)."""
    letters = []
    n = len(pts)
    for k in range(n if closed else n - 1):
        x1, y1 = pts[k][0], pts[k][1]
        x2, y2 = pts[(k + 1) % n][0], pts[(k + 1) % n][1]
        if x1 == x2:
            continue
        hits = []
        for pi, p in cands:
            px, py = p[0], p[1]
            if (x1 < px) != (x2 < px):
                t = (px - x1) / (x2 - x1)
                if y1 + (y2 - y1) * t > py:
                    hits.append((t, pi + 1 if x2 > x1 else -(pi + 1)))
        hits.sort()
        letters += [l for _, l in hits]
    return reduce_word(letters)

def pinned_parts(pre_board, base_board):
    """(pinned, moved, appeared) reference sets between two boards."""
    ppos = {f.GetReference(): f.GetPosition() for f in pre_board.GetFootprints()}
    pinned, moved, appeared = set(), set(), set()
    for f in base_board.GetFootprints():
        r = f.GetReference()
        if r not in ppos:
            appeared.add(r)
            continue
        p = f.GetPosition()
        d = math.hypot((p.x - ppos[r].x) * NM, (p.y - ppos[r].y) * NM)
        (pinned if d < 0.005 else moved).add(r)
    return pinned, moved, appeared

def build_punctures(O, pinned, copper_layers):
    """Vertical-ray puncture system from pinned other-net obstacles.
    Returns [(x_jit, y, frozenset(layers), net, label), ...]; drilled pads
    and vias puncture every copper layer, SMD pads only their own."""
    punct = []
    for p in O['pads']:
        if p['ref'] not in pinned:
            continue
        lays = copper_layers if (p['drill'] or len(p['layers']) > 1) else p['layers']
        punct.append((p['x'], p['y'], frozenset(lays), p['net'],
                      f"{p['ref']}.{p['num']}"))
    for v in O['vias']:
        punct.append((v['x'], v['y'], frozenset(copper_layers), v['net'],
                      f"via@{v['x']:.2f},{v['y']:.2f}"))
    return [(x + 2.19e-8 + i * 1.37e-7, y, lays, net, lab)
            for i, (x, y, lays, net, lab) in enumerate(punct)]
