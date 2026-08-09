#!/usr/bin/env python3
"""jiggle2 stage 0/1: diff pre-rip vs current copper, build the dynamic
copper graph (nodes/edges/pad-bindings with old->new targets) and the
static obstacle set. Outputs data/graph.json + data/obstacles.json."""
import math, json
from collections import defaultdict
import pcbnew
from common import (NM, BOARD_CUR, BOARD_PRE, REGION, in_region,
                    load_board, tsig, vsig, save_json)

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

# net availability in current board
cur_nets = set(cur.GetNetsByName().keys()) if hasattr(cur, 'GetNetsByName') else set()
cur_nets = {str(k) for k in cur_nets}
missing_nets = sorted({e['net'] for e in edges} - cur_nets)
print('nets not present in current board:', missing_nets or 'none')

# ---- static obstacles in region ---------------------------------------------
F, B = int(pcbnew.F_Cu), int(pcbnew.B_Cu)
obst = dict(pads=[], tracks=[], vias=[], clearance=None, hole_clearance=0.254,
            edge=[REGION['x0'], REGION['y0'], REGION['x1'], REGION['y1']])
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

print(f"obstacles: {len(obst['pads'])} pads, {len(obst['tracks'])} kept segs, "
      f"{len(obst['vias'])} kept vias, clearance {obst['clearance']:.3f}mm")

save_json('graph.json', dict(nodes=nodes, edges=edges, F=F, B=B))
save_json('obstacles.json', obst)
print('wrote data/graph.json, data/obstacles.json')
