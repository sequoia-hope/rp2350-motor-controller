#!/usr/bin/env python3
"""jiggle2 stage 0/1: diff pre-rip vs current copper, build the dynamic
copper graph (nodes/edges/pad-bindings with old->new targets) and the
static obstacle set. Outputs data/graph.json + data/obstacles.json."""
import math, json
from collections import defaultdict
import pcbnew
from common import (NM, BOARD_CUR, BOARD_PRE, BOARD_BASE, VARIANT, REGION,
                    in_region, load_board, tsig, vsig, save_json, quiet_stderr)

pre = load_board(BOARD_PRE)
cur = load_board(BOARD_CUR)

# ---- per-part translation deltas (pre-rip -> current) ----------------------
pf = {f.GetReference(): f for f in pre.GetFootprints()}
cf = {f.GetReference(): f for f in cur.GetFootprints()}
delta = {}
for r in set(pf) & set(cf):
    dp, dc = pf[r].GetPosition(), cf[r].GetPosition()
    delta[r] = ((dc.x - dp.x) * NM, (dc.y - dp.y) * NM)

# ---- copper missing from current (the ripped set) ---------------------------
cur_sigs = set()
for t in cur.GetTracks():
    cur_sigs.add(tsig(t) if t.GetClass() == 'PCB_TRACK' else vsig(t))
miss_t = [t for t in pre.GetTracks() if t.GetClass() == 'PCB_TRACK' and tsig(t) not in cur_sigs]
miss_v = [v for v in pre.GetTracks() if v.GetClass() == 'PCB_VIA' and vsig(v) not in cur_sigs]
print(f'missing copper: {len(miss_t)} segments, {len(miss_v)} vias')

# ---- morph variant: trajectory semantics ------------------------------------
# The rip set is chosen for END-state compatibility; a continuously valid
# trajectory must also drag the KEPT copper attached to moving pads (mid-
# flight, a pad slides off its kept trace otherwise), pin drag endpoints that
# junction into static copper, and take obstacles from the SHIPPED board with
# per-ref motion deltas so the field is where the parts actually are.
MORPH = VARIANT == 'morph'
if MORPH:
    moved_refs = {r for r, (dx, dy) in delta.items() if math.hypot(dx, dy) > 0.005}
    moved_pads = [pad for fp in pre.GetFootprints()
                  if fp.GetReference() in moved_refs for pad in fp.Pads()]
    drag_sigs = {tsig(t) for t in miss_t} | {vsig(v) for v in miss_v}

    def on_moved_pad(x, y, layer=None):
        for pad in moved_pads:
            pp = pad.GetPosition()
            if abs(pp.x - x) > int(3 / NM) or abs(pp.y - y) > int(3 / NM):
                continue
            if layer is not None and not pad.IsOnLayer(layer):
                continue
            if pad.HitTest(pcbnew.VECTOR2I(x, y)):
                return True
        return False

    ext_t = ext_v = 0
    for t in pre.GetTracks():
        if t.GetClass() == 'PCB_TRACK':
            if tsig(t) in drag_sigs:
                continue
            s_, e_ = t.GetStart(), t.GetEnd()
            if on_moved_pad(s_.x, s_.y, t.GetLayer()) or \
               on_moved_pad(e_.x, e_.y, t.GetLayer()):
                miss_t.append(t); drag_sigs.add(tsig(t)); ext_t += 1
        else:
            if vsig(t) in drag_sigs:
                continue
            p = t.GetPosition()
            if on_moved_pad(p.x, p.y):
                miss_v.append(t); drag_sigs.add(vsig(t)); ext_v += 1
    print(f'morph: drag set extended by {ext_t} kept segments, {ext_v} kept vias '
          f'attached to {len(moved_refs)} moving parts')

    # everything a moving pad sweeps past must be free to flow out of its way:
    # copper inside any pad's swept corridor becomes dynamic too
    from common import seg_seg_dist
    corridors = []                   # (a, b, radius) swept pad path, inflated
    for pad in moved_pads:
        ref = pad.GetParentFootprint().GetReference()
        dx, dy = delta[ref]
        p = pad.GetPosition()
        bb = pad.GetBoundingBox()
        pr = max(bb.GetWidth(), bb.GetHeight()) * NM / 2
        corridors.append(((p.x * NM, p.y * NM),
                          (p.x * NM + dx, p.y * NM + dy), pr + 0.45))
    cor_t = cor_v = 0
    for t in pre.GetTracks():
        if t.GetClass() == 'PCB_TRACK':
            if tsig(t) in drag_sigs:
                continue
            s_, e_ = t.GetStart(), t.GetEnd()
            a = (s_.x * NM, s_.y * NM); b = (e_.x * NM, e_.y * NM)
            hw = t.GetWidth() * NM / 2
            if any(seg_seg_dist(a, b, ca, cb) < r + hw for ca, cb, r in corridors):
                miss_t.append(t); drag_sigs.add(tsig(t)); cor_t += 1
        else:
            if vsig(t) in drag_sigs:
                continue
            p = t.GetPosition()
            c = (p.x * NM, p.y * NM)
            hw = t.GetWidth(pcbnew.PADSTACK.ALL_LAYERS) * NM / 2
            if any(seg_seg_dist(c, c, ca, cb) < r + hw for ca, cb, r in corridors):
                miss_v.append(t); drag_sigs.add(vsig(t)); cor_v += 1
    print(f'morph: corridor capture made {cor_t} more segments, {cor_v} more vias '
          f'dynamic ({len(corridors)} swept pad corridors)')

# ---- pad lookup in the PRE board (who owned each endpoint) ------------------
# and the same pad's position in the CURRENT board (the anchor target).
def pad_index(board):
    idx = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            p = pad.GetPosition()
            idx.append((fp.GetReference(), pad.GetNumber(), p.x * NM, p.y * NM,
                        pad.GetNetname(), pad))
    return idx

pre_pads = [e for e in pad_index(pre) if in_region(e[2], e[3], m=3)]
cur_pad_pos = {}
cur_pad_net = {}
for fp in cur.GetFootprints():
    for pad in fp.Pads():
        key = (fp.GetReference(), pad.GetNumber())
        p = pad.GetPosition()
        cur_pad_pos[key] = (p.x * NM, p.y * NM)
        cur_pad_net[key] = pad.GetNetname()

def find_pad(x, y, layer):
    """Pad in PRE board whose copper covers point (x,y) on `layer`."""
    best = None
    v = pcbnew.VECTOR2I(int(x / NM), int(y / NM))
    for ref, num, px, py, net, pad in pre_pads:
        if abs(px - x) > 3 or abs(py - y) > 3:
            continue
        if not pad.IsOnLayer(layer):
            continue
        if pad.HitTest(v):
            d = math.hypot(px - x, py - y)
            if best is None or d < best[0]:
                best = (d, ref, num)
    return best and (best[1], best[2])

# ---- build graph ------------------------------------------------------------
# node key: (round nm x, round nm y, layerkey). Vias get layerkey 'VIA' and
# unify endpoints of tracks on any layer at that position.
nodes = []           # dicts: x,y, kind, layers, dia, drill, bind, target
node_id = {}
via_at = {}          # (x_nm, y_nm) -> node id

def key_of(x_nm, y_nm, layer):
    if (x_nm, y_nm) in via_at:
        return via_at[(x_nm, y_nm)]
    return node_id.get((x_nm, y_nm, layer))

# vias first so track endpoints snap onto them
for v in miss_v:
    p = v.GetPosition()
    nid = len(nodes)
    dia = v.GetWidth(pcbnew.PADSTACK.ALL_LAYERS) * NM
    nodes.append(dict(x=p.x * NM, y=p.y * NM, kind='via', dia=dia,
                      drill=v.GetDrill() * NM, net=v.GetNetname(),
                      bind=None, target=None))
    via_at[(p.x, p.y)] = nid

edges = []           # dicts: a, b, layer, w, net
for t in miss_t:
    lay = t.GetLayer()
    ends = []
    for P in (t.GetStart(), t.GetEnd()):
        nid = key_of(P.x, P.y, lay)
        if nid is None:
            nid = len(nodes)
            nodes.append(dict(x=P.x * NM, y=P.y * NM, kind='end', dia=None,
                              drill=None, net=t.GetNetname(), bind=None, target=None))
            node_id[(P.x, P.y, lay)] = nid
        ends.append(nid)
    edges.append(dict(a=ends[0], b=ends[1], layer=int(lay),
                      w=t.GetWidth() * NM, net=t.GetNetname()))

# ---- pad bindings + targets --------------------------------------------------
# For every node, if a PRE pad covers it, bind it; target = endpoint + that
# part's delta. Via nodes can bind too (via-in-pad / under-pad stitching).
bound = unbound_moved = 0
deg = defaultdict(int)
for e in edges:
    deg[e['a']] += 1; deg[e['b']] += 1
node_layers = defaultdict(set)
for e in edges:
    node_layers[e['a']].add(e['layer']); node_layers[e['b']].add(e['layer'])

for nid, n in enumerate(nodes):
    layers = node_layers[nid] or {pcbnew.F_Cu, pcbnew.B_Cu}
    hit = None
    for lay in layers:
        hit = find_pad(n['x'], n['y'], lay)
        if hit: break
    if hit:
        ref, num = hit
        if (ref, num) not in cur_pad_pos:
            continue  # pad vanished (shouldn't happen; report stays honest)
        dx, dy = delta.get(ref, (0.0, 0.0))
        n['bind'] = [ref, num]
        n['target'] = [n['x'] + dx, n['y'] + dy]
        # sanity: target should coincide with the pad's current copper
        cpx, cpy = cur_pad_pos[(ref, num)]
        ppx = next((px, py) for r, u, px, py, _, _ in pre_pads if r == ref and u == num) \
              if any(r == ref and u == num for r, u, *_ in pre_pads) else (cpx - dx, cpy - dy)
        n['pad_off'] = [n['x'] - ppx[0], n['y'] - ppx[1]] if isinstance(ppx, tuple) else [0, 0]
        bound += 1

moved_bound = sum(1 for n in nodes if n['bind'] and
                  math.hypot(n['target'][0]-n['x'], n['target'][1]-n['y']) > 0.01)
print(f'nodes {len(nodes)} (vias {len(miss_v)}), edges {len(edges)}, '
      f'pad-bound {bound} (of which actually moving {moved_bound})')

if MORPH:
    # pin unbound drag endpoints that junction into static copper: they must
    # stay put or the junction opens mid-flight
    static_ends = set()
    static_segs = defaultdict(list)          # coarse cell -> (ax,ay,bx,by,w,layer)
    SC = 2.0
    for t in pre.GetTracks():
        if t.GetClass() == 'PCB_TRACK':
            if tsig(t) in drag_sigs:
                continue
            s_, e_ = t.GetStart(), t.GetEnd()
            static_ends.add((s_.x, s_.y)); static_ends.add((e_.x, e_.y))
            ax, ay, bx, by = s_.x*NM, s_.y*NM, e_.x*NM, e_.y*NM
            for cx in range(int(min(ax, bx)//SC)-1, int(max(ax, bx)//SC)+2):
                for cy in range(int(min(ay, by)//SC)-1, int(max(ay, by)//SC)+2):
                    static_segs[(cx, cy)].append((ax, ay, bx, by,
                                                  t.GetWidth()*NM, int(t.GetLayer())))
        elif vsig(t) not in drag_sigs:
            p = t.GetPosition()
            static_ends.add((p.x, p.y))

    def pt_seg_d(px, py, ax, ay, bx, by):
        dx, dy = bx-ax, by-ay
        L2 = dx*dx + dy*dy
        u = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy)/L2))
        return math.hypot(px-ax-u*dx, py-ay-u*dy)

    pinned_n = 0
    for nid, n in enumerate(nodes):
        if n['bind']:
            continue
        key = (int(round(n['x']/NM)), int(round(n['y']/NM)))
        pin = key in static_ends
        if not pin:
            lays = node_layers[nid]
            for ax, ay, bx, by, w_, lay in static_segs.get(
                    (int(n['x']//SC), int(n['y']//SC)), ()):
                if (n['kind'] == 'via' or lay in lays) and \
                        pt_seg_d(n['x'], n['y'], ax, ay, bx, by) < w_/2:
                    pin = True
                    break
        if pin:
            n['pin'] = True
            pinned_n += 1
    print(f'morph: {pinned_n} junction endpoints pinned to static copper')

# net availability in current board
cur_nets = set(cur.GetNetsByName().keys()) if hasattr(cur, 'GetNetsByName') else set()
cur_nets = {str(k) for k in cur_nets}
missing_nets = sorted({e['net'] for e in edges} - cur_nets)
print('nets not present in current board:', missing_nets or 'none')

# ---- variant: rebuild the base board without the rev-B footprints -----------
# (all reads from the original board are done; a fresh load avoids touching
# live swig wrappers with Remove())
if VARIANT == 'nonew':
    import subprocess, sys as _sys
    new_refs = sorted(set(cf) - set(pf))
    strip = (
        "import pcbnew, sys\n"
        "b = pcbnew.LoadBoard(sys.argv[1])\n"
        "refs = set(sys.argv[3].split(','))\n"
        "for f in list(b.GetFootprints()):\n"
        "    if f.GetReference() in refs:\n"
        "        b.Remove(f)\n"
        "pcbnew.SaveBoard(sys.argv[2], b)\n"
    )
    subprocess.run([_sys.executable, '-c', strip, BOARD_CUR, BOARD_BASE,
                    ','.join(new_refs)], check=True, capture_output=True)
    cur = load_board(BOARD_BASE)   # obstacles come from the variant board
    print(f'variant nonew: removed {len(new_refs)} rev-B footprints, '
          f'base board saved to data_nonew/')

if MORPH:
    # the shipped board IS the base; obstacles are where parts actually are,
    # animated by solve.py via per-ref deltas. No rev-B ghosts.
    import shutil as _sh
    _sh.copy(BOARD_PRE, BOARD_BASE)
    _sh.copy(BOARD_CUR.replace('.kicad_pcb', '.kicad_pro'),
             BOARD_BASE.replace('.kicad_pcb', '.kicad_pro'))
    cur = pre
    print('variant morph: obstacles from the shipped board, '
          f'{len(moved_refs)} refs carry motion deltas')

# ---- static obstacles in region ---------------------------------------------
F, B = int(pcbnew.F_Cu), int(pcbnew.B_Cu)
bb = cur.GetBoardEdgesBoundingBox()
obst = dict(pads=[], tracks=[], vias=[], clearance=None, hole_clearance=0.254,
            board_edge=[bb.GetLeft()*NM, bb.GetTop()*NM, bb.GetRight()*NM, bb.GetBottom()*NM],
            edge_clearance=0.3)
obst['clearance'] = 0.18  # netclass 'Default' clearance per DRC reports
try:
    nc = cur.GetDesignSettings().m_NetSettings.GetDefaultNetclass()
    obst['clearance'] = nc.GetClearance() * NM
except Exception:
    pass

for fp in cur.GetFootprints():
    for pad in fp.Pads():
        p = pad.GetPosition()
        x, y = p.x * NM, p.y * NM
        if not in_region(x, y, m=2):
            continue
        try:
            poly = pad.GetEffectivePolygon()
            pts = [[poly.CVertex(i).x * NM, poly.CVertex(i).y * NM]
                   for i in range(poly.FullPointCount())]
        except Exception:
            bb = pad.GetBoundingBox()
            pts = [[bb.GetLeft()*NM, bb.GetTop()*NM], [bb.GetRight()*NM, bb.GetTop()*NM],
                   [bb.GetRight()*NM, bb.GetBottom()*NM], [bb.GetLeft()*NM, bb.GetBottom()*NM]]
        drill = pad.GetDrillSize().x * NM if pad.GetDrillSize().x else 0
        obst['pads'].append(dict(ref=fp.GetReference(), num=pad.GetNumber(),
                                 net=pad.GetNetname(), x=x, y=y, pts=pts,
                                 layers=[l for l in (F, B) if pad.IsOnLayer(l)],
                                 drill=drill))

for t in cur.GetTracks():
    if MORPH and (tsig(t) if t.GetClass() == 'PCB_TRACK' else vsig(t)) in drag_sigs:
        continue                     # dragged copper is dynamic, not an obstacle
    if t.GetClass() == 'PCB_TRACK':
        s, e = t.GetStart(), t.GetEnd()
        if not (in_region(s.x*NM, s.y*NM, m=2) or in_region(e.x*NM, e.y*NM, m=2)):
            continue
        obst['tracks'].append(dict(net=t.GetNetname(), layer=int(t.GetLayer()),
                                   w=t.GetWidth()*NM,
                                   a=[s.x*NM, s.y*NM], b=[e.x*NM, e.y*NM]))
    else:
        p = t.GetPosition()
        if not in_region(p.x*NM, p.y*NM, m=2):
            continue
        dia = t.GetWidth(pcbnew.PADSTACK.ALL_LAYERS) * NM
        obst['vias'].append(dict(net=t.GetNetname(), x=p.x*NM, y=p.y*NM,
                                 dia=dia, drill=t.GetDrill()*NM))

if MORPH:
    obst['moved'] = {r: list(delta[r]) for r in moved_refs}

print(f"obstacles: {len(obst['pads'])} pads, {len(obst['tracks'])} kept segs, "
      f"{len(obst['vias'])} kept vias, clearance {obst['clearance']:.3f}mm")

save_json('graph.json', dict(nodes=nodes, edges=edges, F=F, B=B))
save_json('obstacles.json', obst)
print('wrote data/graph.json, data/obstacles.json')
