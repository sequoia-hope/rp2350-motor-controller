#!/usr/bin/env python3
"""settle stage 0: board + config -> data/<name>/model.json

Everything the engine needs, in mm, region-scoped:
  - movable parts (in region, not locked) with their pads at true shape,
    courtyards, mobility, group, optional target vector;
  - static obstacles (pads of fixed parts, tracks and vias that stay put);
  - the dynamic copper graph: every track/via touching the region whose net
    is not locked, as nodes (endpoints, vias) + edges (one per segment),
    endpoints bound to the pad that covers them, junctions into static
    copper pinned;
  - ghosts: the parts to inflate, placed at their target pose, with pads and
    courtyard polygons about their centre;
  - per-net rule clearance (netclass, max semantics) and spring parameters.

    python3 prep.py CONFIG.json
"""
import math, sys, json, os
from collections import defaultdict
import pcbnew
from common import (NM, COPPER_LAYERS, F_CU, B_CU, load_config, save_json,
                    load_board, tsig, vsig, sig_key, in_rect, match_any,
                    resolve_elastic, quiet_stderr)

cfg = load_config()
board = load_board(cfg['board'])
pro_path = cfg['board'].replace('.kicad_pcb', '.kicad_pro')
pro = json.load(open(pro_path))

# ---- rules & netclasses ------------------------------------------------------
ds = pro['board']['design_settings']['rules']
classes = {c['name']: c for c in pro['net_settings']['classes']}
assign = {n: cls[0] for n, cls in pro['net_settings'].get('netclass_assignments', {}).items()}
rules = dict(cfg.get('rules', {}))
rules.setdefault('clearance', ds.get('min_clearance', 0.1))
rules.setdefault('hole_clearance', ds.get('min_hole_clearance', 0.254))
rules.setdefault('edge_clearance', ds.get('min_copper_edge_clearance', 0.3))
rules.setdefault('guard', 0.015)
bb = board.GetBoardEdgesBoundingBox()
rules['board_edge'] = [bb.GetLeft() * NM, bb.GetTop() * NM, bb.GetRight() * NM, bb.GetBottom() * NM]
region = cfg.get('region') or rules['board_edge']
MARGIN = cfg.get('margin', 2.0)

netclass_of = {}
for name in board.GetNetsByName().keys():
    n = str(name)
    netclass_of[n] = assign.get(n, 'Default')
nets = {}
for n, cls in netclass_of.items():
    c = classes.get(cls, classes['Default'])
    e = resolve_elastic(cfg, n, cls)
    nets[n] = dict(cls=cls, clr=float(c.get('clearance', rules['clearance'])), **e)
nets[''] = dict(cls='none', clr=rules['clearance'], k=0.0, allow=1e9, cap=1e9)
locked_nets = cfg.get('locked', {}).get('nets', [])
for n in nets:
    if n and match_any(n, locked_nets):
        nets[n]['locked'] = True

# ---- footprints --------------------------------------------------------------
lk = cfg.get('locked', {})
lk_refs = set(lk.get('refs', []))
lk_rects = lk.get('rects', [])
lk_sides = set(lk.get('sides', []))
mob = cfg.get('mobility', {})
ghost_specs = cfg.get('drive', {}).get('inflate', [])
ghost_refs = {g['ref'] for g in ghost_specs}
groups = cfg.get('groups', {})

fps = {f.GetReference(): f for f in board.GetFootprints()}
parts = {}


def pad_centroid(f):
    xs = [p.GetPosition().x * NM for p in f.Pads()]
    ys = [p.GetPosition().y * NM for p in f.Pads()]
    return (sum(xs) / len(xs), sum(ys) / len(ys)) if xs else \
        (f.GetPosition().x * NM, f.GetPosition().y * NM)


def mobility_of(ref, cx, cy):
    m = mob.get('default', 1.0)
    for r in mob.get('rects', []):
        if in_rect(cx, cy, r['rect']):
            m = r['m']
    return mob.get('refs', {}).get(ref, m)


def group_of(ref, cx, cy):
    for gname, g in groups.items():
        if ref in g.get('refs', []) or ('rect' in g and in_rect(cx, cy, g['rect'])):
            return gname
    return None

for ref, f in fps.items():
    if ref in ghost_refs:
        continue
    cx, cy = pad_centroid(f)
    side = 'B' if f.IsFlipped() else 'F'
    inreg = in_rect(cx, cy, region)
    locked = (ref in lk_refs or side in lk_sides or
              any(in_rect(cx, cy, r) for r in lk_rects))
    m = mobility_of(ref, cx, cy) if inreg and not locked else 0.0
    parts[ref] = dict(x=f.GetPosition().x * NM, y=f.GetPosition().y * NM,
                      cx=cx, cy=cy, side=side, rot=f.GetOrientationDegrees(),
                      movable=bool(inreg and not locked and m > 0), mobility=m,
                      group=group_of(ref, cx, cy) if inreg else None,
                      target=None, in_region=inreg)

# targets / shoves
drv = cfg.get('drive', {})
for ref, v in drv.get('targets', {}).items():
    if ref in parts:
        parts[ref]['target'] = list(v)
for sh in drv.get('shove', []):
    sel = set(sh.get('refs', []))
    if 'rect' in sh:
        sel |= {r for r, p in parts.items() if in_rect(p['cx'], p['cy'], sh['rect'])}
    for r in sel:
        p = parts.get(r)
        if not p or not p['movable']:
            continue
        if 'vector' in sh:
            p['target'] = list(sh['vector'])
        elif 'toward' in sh:
            tx, ty = sh['toward']
            dx, dy = tx - p['cx'], ty - p['cy']
            L = math.hypot(dx, dy) or 1.0
            d = min(sh.get('mm', L), L)
            p['target'] = [dx / L * d, dy / L * d]

# ---- pads (true shapes) and courtyards ---------------------------------------


def pad_entry(f, pad):
    px, py = pad.GetPosition().x * NM, pad.GetPosition().y * NM
    try:
        flc = f.GetLocalClearance() or 0
    except TypeError:
        flc = 0
    try:
        lc = pad.GetLocalClearance() or 0
    except TypeError:
        lc = 0
    try:
        ps = pad.GetEffectivePolygon()
    except TypeError:
        ps = pad.GetEffectivePolygon(pcbnew.ERROR_INSIDE)
    pts = []
    if ps.OutlineCount():
        o = ps.Outline(0)
        pts = [[o.CPoint(j).x * NM, o.CPoint(j).y * NM] for j in range(o.PointCount())]
    if not pts:
        b = pad.GetBoundingBox()
        pts = [[b.GetLeft()*NM, b.GetTop()*NM], [b.GetRight()*NM, b.GetTop()*NM],
               [b.GetRight()*NM, b.GetBottom()*NM], [b.GetLeft()*NM, b.GetBottom()*NM]]
    drill = pad.GetDrillSize().x * NM if pad.GetDrillSize().x else 0.0
    layers = [l for l in COPPER_LAYERS if pad.IsOnLayer(l)]
    return dict(ref=f.GetReference(), num=pad.GetNumber(), net=pad.GetNetname(),
                x=px, y=py, pts=pts, layers=layers, drill=drill,
                lc=max(lc, flc) * NM if max(lc, flc) > 1 else 0.0,
                npth=pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH)


def courtyard_entry(f):
    try:
        f.BuildCourtyardCaches()
    except AttributeError:
        pass
    entry = {}
    for lay, name in ((pcbnew.F_CrtYd, 'F'), (pcbnew.B_CrtYd, 'B')):
        ps = f.GetCourtyard(lay)
        polys = []
        for i in range(ps.OutlineCount()):
            o = ps.Outline(i)
            pts = [[o.CPoint(j).x * NM, o.CPoint(j).y * NM] for j in range(o.PointCount())]
            if len(pts) >= 3:
                polys.append(pts)
        if polys:
            entry[name] = polys
    return entry

pads = []
pad_objs = []            # parallel: pcbnew pad for binding
courtyards = {}
for ref, f in fps.items():
    if ref in ghost_refs:
        continue
    cx, cy = parts[ref]['cx'], parts[ref]['cy']
    if in_rect(cx, cy, region, 8.0):
        cy_e = courtyard_entry(f)
        if cy_e:
            courtyards[ref] = cy_e
    for pad in f.Pads():
        px, py = pad.GetPosition().x * NM, pad.GetPosition().y * NM
        if in_rect(px, py, region, MARGIN) or parts[ref]['movable']:
            pads.append(pad_entry(f, pad))
            pad_objs.append((pad, f.GetReference()))

# ---- copper: dynamic graph vs static obstacles --------------------------------
tracks_static, vias_static = [], []
dyn_t, dyn_v = [], []
for t in board.GetTracks():
    if t.GetClass() == 'PCB_TRACK':
        s, e = t.GetStart(), t.GetEnd()
        a = (s.x * NM, s.y * NM); b = (e.x * NM, e.y * NM)
        touch = in_rect(*a, region) or in_rect(*b, region)
        near = in_rect(*a, region, MARGIN) or in_rect(*b, region, MARGIN)
        net = t.GetNetname()
        locked = nets.get(net, {}).get('locked') or \
            any(in_rect(*a, r) and in_rect(*b, r) for r in lk_rects)
        if touch and not locked:
            dyn_t.append(t)
        elif near:
            tracks_static.append(dict(net=net, layer=int(t.GetLayer()),
                                      w=t.GetWidth() * NM, a=list(a), b=list(b)))
    elif t.GetClass() == 'PCB_VIA':
        p = t.GetPosition()
        x, y = p.x * NM, p.y * NM
        net = t.GetNetname()
        locked = nets.get(net, {}).get('locked') or any(in_rect(x, y, r) for r in lk_rects)
        dia = t.GetWidth(pcbnew.PADSTACK.ALL_LAYERS) * NM
        if in_rect(x, y, region) and not locked:
            dyn_v.append(t)
        elif in_rect(x, y, region, MARGIN):
            vias_static.append(dict(net=net, x=x, y=y, dia=dia, drill=t.GetDrill() * NM))

nodes, node_id, via_at = [], {}, {}
for v in dyn_v:
    p = v.GetPosition()
    nid = len(nodes)
    nodes.append(dict(x=p.x * NM, y=p.y * NM, kind='via',
                      dia=v.GetWidth(pcbnew.PADSTACK.ALL_LAYERS) * NM,
                      drill=v.GetDrill() * NM, net=v.GetNetname(), bind=None,
                      sig=sig_key(vsig(v))))
    via_at[(p.x, p.y)] = nid
edges = []
for t in dyn_t:
    lay = int(t.GetLayer())
    ends = []
    for P_ in (t.GetStart(), t.GetEnd()):
        nid = via_at.get((P_.x, P_.y))
        if nid is None:
            nid = node_id.get((P_.x, P_.y, lay))
        if nid is None:
            nid = len(nodes)
            nodes.append(dict(x=P_.x * NM, y=P_.y * NM, kind='end', dia=None, drill=None,
                              net=t.GetNetname(), bind=None))
            node_id[(P_.x, P_.y, lay)] = nid
        ends.append(nid)
    edges.append(dict(a=ends[0], b=ends[1], layer=lay, w=t.GetWidth() * NM,
                      net=t.GetNetname(), sig=sig_key(tsig(t))))

node_layers = defaultdict(set)
for e in edges:
    node_layers[e['a']].add(e['layer']); node_layers[e['b']].add(e['layer'])

# bind endpoints to the pad that covers them
pad_cells = defaultdict(list)
for i, (pad, ref) in enumerate(pad_objs):
    p = pad.GetPosition()
    pad_cells[(int(p.x * NM // 3), int(p.y * NM // 3))].append(i)


def find_pad(x, y, layers):
    v = pcbnew.VECTOR2I(int(round(x / NM)), int(round(y / NM)))
    best = None
    cx, cy = int(x // 3), int(y // 3)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for i in pad_cells.get((cx + dx, cy + dy), ()):
                pad, ref = pad_objs[i]
                pp = pad.GetPosition()
                if abs(pp.x * NM - x) > 3 or abs(pp.y * NM - y) > 3:
                    continue
                if layers and not any(pad.IsOnLayer(l) for l in layers):
                    continue
                if pad.HitTest(v):
                    d = math.hypot(pp.x * NM - x, pp.y * NM - y)
                    if best is None or d < best[0]:
                        best = (d, i)
    return best and best[1]

static_ends = set()
static_by_cell = defaultdict(list)
for ts in tracks_static:
    static_ends.add((round(ts['a'][0], 4), round(ts['a'][1], 4)))
    static_ends.add((round(ts['b'][0], 4), round(ts['b'][1], 4)))
    (ax, ay), (bx, by) = ts['a'], ts['b']
    for cx in range(int(min(ax, bx) // 2) - 1, int(max(ax, bx) // 2) + 2):
        for cy in range(int(min(ay, by) // 2) - 1, int(max(ay, by) // 2) + 2):
            static_by_cell[(cx, cy)].append(ts)
for vs in vias_static:
    static_ends.add((round(vs['x'], 4), round(vs['y'], 4)))


def pt_seg_d(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    u = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - u * dx, py - ay - u * dy)

n_bound = n_anchor = n_pin = n_out = 0
for nid, n in enumerate(nodes):
    lays = list(node_layers[nid]) if n['kind'] == 'end' else []
    i = find_pad(n['x'], n['y'], lays)
    if i is not None:
        pad, ref = pad_objs[i]
        n['bind'] = [ref, pad.GetNumber()]
        n_bound += 1
        if parts[ref]['movable']:
            n_anchor += 1
        continue
    pin = (round(n['x'], 4), round(n['y'], 4)) in static_ends
    if not pin and not in_rect(n['x'], n['y'], region):
        pin = True
        n_out += 1
    if not pin:
        for ts in static_by_cell.get((int(n['x'] // 2), int(n['y'] // 2)), ()):
            if (n['kind'] == 'via' or ts['layer'] in node_layers[nid]) and \
                    pt_seg_d(n['x'], n['y'], *ts['a'], *ts['b']) < ts['w'] / 2:
                pin = True
                break
    if pin:
        n['pin'] = True
        n_pin += 1

# a movable part whose pads carry locked (static) copper cannot move
static_touch = defaultdict(int)
v_all = pcbnew.VECTOR2I
for ts in tracks_static:
    for (x, y) in (ts['a'], ts['b']):
        i = find_pad(x, y, [ts['layer']])
        if i is not None:
            static_touch[pad_objs[i][1]] += 1
for ref, cnt in static_touch.items():
    if parts[ref]['movable']:
        parts[ref]['movable'] = False
        parts[ref]['mobility'] = 0.0
        parts[ref]['why_fixed'] = f'{cnt} static copper ends on its pads'
        print(f'  {ref}: fixed — {cnt} static (locked-net or out-of-region) copper ends on its pads')

# ---- ghosts ------------------------------------------------------------------
ghosts = []
power_like = ('GND', 'GND_ISO', '+3V3', '+1V1', 'VBUS', 'VMOT', 'Vdrive', '+5V_ISO')

# free-space raster per side for `at: "search"`: how much of a full-size
# footprint's courtyard box would land on pads / courtyards (weight 1) or on
# copper of that side's outer layer (weight 0.3, it can be pushed or rerouted)
_SR = 0.1
_free = {}


def free_raster(side):
    if side in _free:
        return _free[side]
    import numpy as np
    x0, y0, x1, y1 = region
    W = int((x1 - x0) / _SR) + 1
    H = int((y1 - y0) / _SR) + 1
    m = np.zeros((H, W), dtype=float)
    lay = F_CU if side == 'F' else B_CU

    def stamp_box(bx0, by0, bx1, by1, w):
        i0 = max(0, int((bx0 - x0) / _SR)); i1 = min(W - 1, int((bx1 - x0) / _SR) + 1)
        j0 = max(0, int((by0 - y0) / _SR)); j1 = min(H - 1, int((by1 - y0) / _SR) + 1)
        if i1 >= i0 and j1 >= j0:
            m[j0:j1 + 1, i0:i1 + 1] = np.maximum(m[j0:j1 + 1, i0:i1 + 1], w)
    for pe in pads:
        if pe['drill'] or lay in pe['layers']:
            xs = [q[0] for q in pe['pts']]; ys = [q[1] for q in pe['pts']]
            stamp_box(min(xs) - 0.1, min(ys) - 0.1, max(xs) + 0.1, max(ys) + 0.1, 1.0)
    for ref, entry in courtyards.items():
        for poly in entry.get(side, []):
            xs = [q[0] for q in poly]; ys = [q[1] for q in poly]
            stamp_box(min(xs), min(ys), max(xs), max(ys), 1.0)
    for t in tracks_static:
        if t['layer'] == lay:
            stamp_box(min(t['a'][0], t['b'][0]) - t['w'], min(t['a'][1], t['b'][1]) - t['w'],
                      max(t['a'][0], t['b'][0]) + t['w'], max(t['a'][1], t['b'][1]) + t['w'], 1.0)
    for t in dyn_t:
        if int(t.GetLayer()) != lay:
            continue
        s_, e_ = t.GetStart(), t.GetEnd()
        w = t.GetWidth() * NM
        ax, ay, bx, by = s_.x * NM, s_.y * NM, e_.x * NM, e_.y * NM
        # thin diagonal tracks: stamp along the segment in short boxes
        L = math.hypot(bx - ax, by - ay)
        n = max(1, int(L / 0.3))
        for k in range(n + 1):
            px, py = ax + (bx - ax) * k / n, ay + (by - ay) * k / n
            stamp_box(px - w / 2 - 0.05, py - w / 2 - 0.05, px + w / 2 + 0.05, py + w / 2 + 0.05, 0.3)
    for v in dyn_v:
        p = v.GetPosition()
        r = v.GetWidth(pcbnew.PADSTACK.ALL_LAYERS) * NM / 2
        stamp_box(p.x * NM - r, p.y * NM - r, p.x * NM + r, p.y * NM + r, 0.6)
    for v in vias_static:
        stamp_box(v['x'] - v['dia'] / 2, v['y'] - v['dia'] / 2, v['x'] + v['dia'] / 2, v['y'] + v['dia'] / 2, 1.0)
    # 2-D prefix sums for box queries
    S = np.zeros((H + 1, W + 1))
    S[1:, 1:] = m.cumsum(0).cumsum(1)
    _free[side] = (S, W, H)
    return _free[side]


def reserve_site(side, gpads, gcourt):
    """Stamp a placed ghost into the free raster so later searches avoid it."""
    import numpy as np
    S, W, H = free_raster(side)
    # rebuild the prefix sums after stamping: cheap for a few ghosts
    m = np.zeros((H, W))
    m[:, :] = np.diff(np.diff(S, axis=0), axis=1)
    x0, y0 = region[0], region[1]
    boxes = [(min(q[0] for q in pe['pts']), min(q[1] for q in pe['pts']),
              max(q[0] for q in pe['pts']), max(q[1] for q in pe['pts'])) for pe in gpads]
    for polys in gcourt.values():
        for poly in polys:
            boxes.append((min(q[0] for q in poly), min(q[1] for q in poly),
                          max(q[0] for q in poly), max(q[1] for q in poly)))
    for bx0, by0, bx1, by1 in boxes:
        i0 = max(0, int((bx0 - 0.1 - x0) / _SR)); i1 = min(W - 1, int((bx1 + 0.1 - x0) / _SR) + 1)
        j0 = max(0, int((by0 - 0.1 - y0) / _SR)); j1 = min(H - 1, int((by1 + 0.1 - y0) / _SR) + 1)
        if i1 >= i0 and j1 >= j0:
            m[j0:j1 + 1, i0:i1 + 1] = 1.0
    S2 = np.zeros((H + 1, W + 1))
    S2[1:, 1:] = m.cumsum(0).cumsum(1)
    _free[side] = (S2, W, H)


def site_search(f, side, centre, radius, lam=0.03):
    """Best centre for footprint f within `radius` of `centre`: least
    obstructed courtyard box, tie-broken toward the centre."""
    import numpy as np
    S, W, H = free_raster(side)
    x0, y0 = region[0], region[1]
    ce = courtyard_entry(f)
    polys = ce.get(side) or [[[q.x * NM, q.y * NM] for q in [f.GetBoundingBox(False, False).GetOrigin(),
                                                             f.GetBoundingBox(False, False).GetEnd()]]]
    xs = [q[0] for poly in polys for q in poly]; ys = [q[1] for poly in polys for q in poly]
    cx0, cy0 = pad_centroid(f)
    hw = (max(xs) - min(xs)) / 2 + 0.15
    hh = (max(ys) - min(ys)) / 2 + 0.15
    ox, oy = (max(xs) + min(xs)) / 2 - cx0, (max(ys) + min(ys)) / 2 - cy0   # box centre vs pad centroid
    best = None
    steps = int(radius / 0.25)
    for di in range(-steps, steps + 1):
        for dj in range(-steps, steps + 1):
            px, py = centre[0] + di * 0.25, centre[1] + dj * 0.25
            d = math.hypot(px - centre[0], py - centre[1])
            if d > radius:
                continue
            bx0, by0, bx1, by1 = px + ox - hw, py + oy - hh, px + ox + hw, py + oy + hh
            em = rules['edge_clearance'] + 0.25
            ex0, ey0, ex1, ey1 = rules['board_edge']
            if bx0 < max(region[0], ex0 + em) or by0 < max(region[1], ey0 + em) or \
               bx1 > min(region[2], ex1 - em) or by1 > min(region[3], ey1 - em):
                continue
            i0 = int((bx0 - x0) / _SR); i1 = int((bx1 - x0) / _SR) + 1
            j0 = int((by0 - y0) / _SR); j1 = int((by1 - y0) / _SR) + 1
            i1 = min(i1, W); j1 = min(j1, H)
            tot = S[j1, i1] - S[j0, i1] - S[j1, i0] + S[j0, i0]
            frac = tot / max(1, (i1 - i0) * (j1 - j0))
            score = frac + lam * d
            if best is None or score < best[0]:
                best = (score, px, py, frac, d)
    return best
for g in ghost_specs:
    ref = g['ref']
    f = fps.get(ref)
    if f is None:
        sys.exit(f'ghost {ref}: no such footprint')
    at = g.get('at', 'auto')
    partner = []
    near_search = None
    if isinstance(at, dict) and 'near' in at:
        host = next((gh for gh in ghosts if gh['ref'] == at['near']), None)
        if host is None:
            sys.exit(f"ghost {ref}: near={at['near']} must be listed before it")
        off = at.get('offset', [0.0, 0.0])
        near_search = at.get('search')
        at = [host['at'][0] + off[0], host['at'][1] + off[1]]
    search = at == 'search' or near_search is not None
    if near_search is not None:
        g = dict(g, radius=near_search)
    if at in ('auto', 'search'):
        gnets = {p.GetNetname() for p in f.Pads()} - set(power_like) - {''}
        for pe in pads:
            if pe['net'] in gnets and not pe['net'].startswith('unconnected'):
                partner.append((pe['x'], pe['y']))
        if not partner:
            sys.exit(f'ghost {ref}: at={at} but no partner pads in region')
        at = [sum(x for x, _ in partner) / len(partner), sum(y for _, y in partner) / len(partner)]
        at = [min(max(at[0], region[0] + 1.5), region[2] - 1.5),
              min(max(at[1], region[1] + 1.5), region[3] - 1.5)]
    if search:
        side_ = g.get('side') or ('B' if f.IsFlipped() else 'F')
        best = site_search(f, side_, at, g.get('radius', 6.0))
        if best:
            print(f'  ghost {ref}: site search {at[0]:.2f},{at[1]:.2f} -> {best[1]:.2f},{best[2]:.2f} '
                  f'(obstruction {best[3]:.2f}, {best[4]:.1f} mm from partners)')
            at = [best[1], best[2]]
    pos0, rot0, flip0 = f.GetPosition(), f.GetOrientationDegrees(), f.IsFlipped()
    want_side = g.get('side')
    if want_side and ((want_side == 'B') != flip0):
        f.Flip(f.GetPosition(), False)
    if 'rot' in g:
        f.SetOrientationDegrees(g['rot'])
    # move so the PAD CENTROID lands on `at`
    cx0, cy0 = pad_centroid(f)
    f.SetPosition(pcbnew.VECTOR2I(int(round((f.GetPosition().x * NM + at[0] - cx0) / NM)),
                                  int(round((f.GetPosition().y * NM + at[1] - cy0) / NM))))
    gp = [pad_entry(f, pad) for pad in f.Pads()]
    for pe in gp:
        pe['real_net'] = pe['net']
        pe['net'] = ''                 # foreign to everything while inflating
        pe['ref'] = 'GHOST:' + ref
    gc = courtyard_entry(f)
    for pe in gp:
        pe['ghost'] = len(ghosts)
    cx, cy = pad_centroid(f)
    r = 0.0
    for pe in gp:
        for q in pe['pts']:
            r = max(r, math.hypot(q[0] - cx, q[1] - cy))
    for polys in gc.values():
        for poly in polys:
            for q in poly:
                r = max(r, math.hypot(q[0] - cx, q[1] - cy))
    ghosts.append(dict(ref=ref, at=[cx, cy], origin=[f.GetPosition().x * NM, f.GetPosition().y * NM],
                       rot=f.GetOrientationDegrees(), side='B' if f.IsFlipped() else 'F',
                       pads=gp, courtyards=gc, r=r, partners=len(partner)))
    reserve_site('B' if f.IsFlipped() else 'F', gp, gc)
    # restore (nothing is saved, but keep the live object honest)
    f.SetPosition(pos0); f.SetOrientationDegrees(rot0)
    if want_side and ((want_side == 'B') != flip0):
        f.Flip(f.GetPosition(), False)
    print(f'  ghost {ref}: at ({cx:.2f},{cy:.2f}) r={r:.2f} '
          f'{len(gp)} pads, {len(partner)} partner pads')

# ---- summary + save ----------------------------------------------------------
n_mov = sum(1 for p in parts.values() if p['movable'])
n_in = sum(1 for p in parts.values() if p['in_region'])
print(f'region {region}: {n_in} parts in region, {n_mov} movable; '
      f'{len(pads)} pads modelled, {len(courtyards)} courtyards')
print(f'dynamic copper: {len(nodes)} nodes ({len(dyn_v)} vias), {len(edges)} segments; '
      f'static: {len(tracks_static)} segments, {len(vias_static)} vias')
print(f'bindings: {n_bound} on pads ({n_anchor} on movable parts), '
      f'{n_pin} pinned ({n_out} outside the region)')
locked_movers = [r for r, p in parts.items() if p['in_region'] and not p['movable']]
print(f'fixed in region: {len(locked_movers)}: {" ".join(sorted(locked_movers))[:400]}')
model = dict(name=cfg['name'], board=cfg['board'], region=region, rules=rules,
             layers=COPPER_LAYERS, nodes=nodes, edges=edges, pads=pads,
             tracks=tracks_static, vias=vias_static, courtyards=courtyards,
             parts=parts, ghosts=ghosts, nets=nets)
p = save_json(cfg, 'model.json', model)
print(f'wrote {p}')
