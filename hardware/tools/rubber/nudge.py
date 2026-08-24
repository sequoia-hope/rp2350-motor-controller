#!/usr/bin/env python3
"""Violation-driven local track nudger for the rubber stretch (v2).

Round: DRC -> position-independent diff vs baseline -> cluster instances into
physical sites (same normalised item pair within 0.4 mm) -> one push decision
per site: displace the offending track's chain by (deficit + margin) with a
linear along-chain falloff.  Pad-glued ends never move; tee endpoints (a track
end resting on a passing same-net segment's body) and vias under a same-net
segment RIDE their bar, so overlap-made junctions cannot be pulled apart.
Per-node moves are capped per round and cumulatively.  A site that survives
ATTEMPT_CAP pushes is left for hand repair.  Zones refill before every DRC.

usage: nudge.py BOARD BASELINE_DRC.json LOG.json [ROUNDS]
"""
import json, math, os, re, sys, collections, heapq
S = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, S)
HW = os.path.dirname(os.path.dirname(S))
sys.path.insert(0, os.path.join(HW, 'tools', 'jiggle2'))
sys.path.insert(0, os.path.join(HW, 'tools', 'modules'))
from common import load_board, NM, quiet_stderr
from drcdiff import run_drc, diff, norm
import pcbnew

BOARD = sys.argv[1]; BASE = sys.argv[2]; LOG = sys.argv[3]
ROUNDS = int(sys.argv[4]) if len(sys.argv) > 4 else 20
MARGIN = 0.015; FALL = 2.0; CAP = 0.35; RCAP = 0.12; ATTEMPT_CAP = 8
PROTECT = {'VMOT', '/A_P', '/B_P', '/C_P', '/A_N', '/B_N', '/C_N', '/D',
           '/VREG_LX', 'Net-(D4-K)', '/USB_DP', '/USB_DN', '/USB_D+', '/USB_D-',
           'Net-(U8-USB_DM)', 'Net-(U8-USB_DP)', '/CAN0_N', '/CAN0_P'}
NUDGE_TYPES = {'clearance', 'hole_clearance', 'copper_edge_clearance', 'courtyards_overlap'}
IGNORE_TYPES = {'padstack'}

bd = load_board(BOARD)
base = json.load(open(BASE))

# ---- copper graph -----------------------------------------------------------
tracks = [t for t in bd.GetTracks() if t.GetClass() == 'PCB_TRACK']
vias = [t for t in bd.GetTracks() if t.GetClass() == 'PCB_VIA']
nodes = []; node_of = {}; adj = collections.defaultdict(list); tnodes = []
pads = []
for f in bd.GetFootprints():
    for p in f.Pads():
        pb = p.GetBoundingBox()
        pads.append(((pb.GetLeft()*NM, pb.GetTop()*NM, pb.GetRight()*NM, pb.GetBottom()*NM), p))
def pad_pin(x, y, layer, nc):
    for pb, p in pads:
        if not (pb[0]-1e-4 <= x <= pb[2]+1e-4 and pb[1]-1e-4 <= y <= pb[3]+1e-4): continue
        if layer is not None and not p.IsOnLayer(layer): continue
        if p.GetNetCode() != nc: continue
        if p.HitTest(pcbnew.VECTOR2I(int(round(x/NM)), int(round(y/NM)))):
            return p.GetParentFootprint().GetReference()
    return None
def new_node(x, y, net, via=None):
    nodes.append(dict(x=x, y=y, net=net, pinned=False, via=via, mv=[0.0, 0.0], fpref=None))
    return len(nodes)-1
via_index = collections.defaultdict(list)
for vo in vias:
    p = vo.GetPosition(); x, y = p.x*NM, p.y*NM; nc = vo.GetNetCode()
    nid = new_node(x, y, nc, via=vo)
    try: r = vo.GetWidth(pcbnew.PADSTACK.ALL_LAYERS)*NM/2
    except Exception: r = 0.3
    via_index[(int(x), int(y))].append((x, y, r, nc, nid))
    pr = pad_pin(x, y, None, nc)
    if pr: nodes[nid]['pinned'] = True; nodes[nid]['fpref'] = pr
def find_via(x, y, nc):
    for cx in (int(x)-1, int(x), int(x)+1):
        for cy in (int(y)-1, int(y), int(y)+1):
            for vx, vy, r, vnc, nid in via_index.get((cx, cy), ()):
                if vnc == nc and math.hypot(vx-x, vy-y) <= r+1e-6:
                    return nid, math.hypot(vx-x, vy-y)
    return None, None
rides = {}          # nid -> ('seg', j, t, ox, oy) | ('via', vnid, ox, oy)
for t in tracks:
    ends = []; lay = t.GetLayer(); nc = t.GetNetCode()
    for P in (t.GetStart(), t.GetEnd()):
        x, y = P.x*NM, P.y*NM
        vnid, vd = find_via(x, y, nc)
        if vnid is not None and vd <= 1e-4:
            nid = vnid                              # end exactly on via centre
        else:
            key = (P.x, P.y, int(lay), nc)
            nid = node_of.get(key)
            if nid is None:
                nid = new_node(x, y, nc); node_of[key] = nid
                pr = pad_pin(x, y, lay, nc)
                if pr: nodes[nid]['pinned'] = True; nodes[nid]['fpref'] = pr
                if vnid is not None and not nodes[nid]['pinned']:
                    # end on the via rim: keep own coords, ride the via
                    rides[nid] = ('via', vnid, x - nodes[vnid]['x'], y - nodes[vnid]['y'])
                    adj[nid].append((vnid, vd)); adj[vnid].append((nid, vd))
        ends.append(nid)
    na, nb = ends
    L = math.hypot((t.GetEnd().x-t.GetStart().x)*NM, (t.GetEnd().y-t.GetStart().y)*NM)
    adj[na].append((nb, L)); adj[nb].append((na, L))
    tnodes.append((t, na, nb))

# segments whose body crosses a same-net pad connect by overlap: pin their ends
padcell = collections.defaultdict(list)
for pb, p in pads:
    for gx in range(int(pb[0]//2), int(pb[2]//2)+1):
        for gy in range(int(pb[1]//2), int(pb[3]//2)+1):
            padcell[(gx, gy)].append((pb, p))
n_bodypin = 0
for t, na, nb in tnodes:
    s, e = t.GetStart(), t.GetEnd(); nc = t.GetNetCode(); lay = t.GetLayer()
    x0, y0, x1, y1 = s.x*NM, s.y*NM, e.x*NM, e.y*NM
    seen_p = set()
    for gx in range(int(min(x0,x1)//2), int(max(x0,x1)//2)+1):
        for gy in range(int(min(y0,y1)//2), int(max(y0,y1)//2)+1):
            for pb, p in padcell.get((gx, gy), ()):
                if id(p) in seen_p: continue
                seen_p.add(id(p))
                if p.GetNetCode() != nc or not p.IsOnLayer(lay): continue
                if p.HitTest(s) or p.HitTest(e): continue
                pp = p.GetPosition()
                dx, dy = x1-x0, y1-y0; L2 = dx*dx+dy*dy
                tt = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((pp.x*NM-x0)*dx+(pp.y*NM-y0)*dy)/L2))
                if not (0.02 < tt < 0.98): continue
                cxp = x0+dx*tt; cyp = y0+dy*tt
                if p.HitTest(pcbnew.VECTOR2I(int(round(cxp/NM)), int(round(cyp/NM)))):
                    pr = p.GetParentFootprint().GetReference()
                    for nn in (na, nb):
                        nodes[nn]['pinned'] = True
                        if nodes[nn]['fpref'] is None: nodes[nn]['fpref'] = pr
                    n_bodypin += 1
                    break
print(f'pad-under-bar: {n_bodypin} segments end-pinned', flush=True)

def pt_seg(px, py, ax, ay, bx, by):
    dx, dy = bx-ax, by-ay; L2 = dx*dx+dy*dy
    tt = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((px-ax)*dx+(py-ay)*dy)/L2))
    return math.hypot(px-ax-tt*dx, py-ay-tt*dy), tt

# ---- rides: tee endpoints and vias under same-net bars ----------------------
seg_nl = collections.defaultdict(list)
for i, (t, na, nb) in enumerate(tnodes):
    seg_nl[(t.GetNetCode(), t.GetLayer())].append(i)
node_segs = collections.defaultdict(set)
for i, (t, na, nb) in enumerate(tnodes):
    node_segs[na].add(i); node_segs[nb].add(i)
node_layers = collections.defaultdict(set)
for i, (t, na, nb) in enumerate(tnodes):
    node_layers[na].add(t.GetLayer()); node_layers[nb].add(t.GetLayer())
for nid, nd in enumerate(nodes):
    if nd['via'] is not None:
        node_layers[nid] = {lay for (nc2, lay) in seg_nl if nc2 == nd['net']}
for nid, nd in enumerate(nodes):
    if nd['pinned'] or nid in rides: continue
    best = None
    for lay in node_layers.get(nid, ()):
        for j in seg_nl.get((nd['net'], lay), ()):
            if j in node_segs[nid]: continue
            t, na, nb = tnodes[j]
            a, b = nodes[na], nodes[nb]
            d, tt = pt_seg(nd['x'], nd['y'], a['x'], a['y'], b['x'], b['y'])
            if d < tnodes[j][0].GetWidth()*NM/2*0.95 and 0.01 < tt < 0.99:
                if best is None or d < best[0]: best = (d, j, tt)
    if best is not None:
        d, j, tt = best
        t, na, nb = tnodes[j]; a, b = nodes[na], nodes[nb]
        ox = nd['x'] - (a['x'] + (b['x']-a['x'])*tt)
        oy = nd['y'] - (a['y'] + (b['y']-a['y'])*tt)
        rides[nid] = ('seg', j, tt, ox, oy)
RIDE = set(rides)
print(f'graph: {len(nodes)} nodes, {sum(1 for n in nodes if n["pinned"])} pad-pinned, '
      f'{len(rides)} riding (tee/via-under-bar)', flush=True)

fp_base = {}; npads_fp = {}
for f in bd.GetFootprints():
    r0 = f.GetReference(); p0 = f.GetPosition()
    fp_base[r0] = (p0.x*NM, p0.y*NM, f); npads_fp[r0] = len(list(f.Pads()))
fpmv = collections.defaultdict(lambda: [0.0, 0.0])
fpref_nodes = collections.defaultdict(list)
for nid, nd in enumerate(nodes):
    if nd.get('fpref'): fpref_nodes[nd['fpref']].append(nid)
def cur_fp_pos(ref):
    bx, by, f = fp_base[ref]; mv = fpmv[ref]
    return bx + mv[0], by + mv[1]
def move_fp(ref, dx, dy):
    mv = fpmv[ref]
    if math.hypot(mv[0]+dx, mv[1]+dy) > 0.15: return False
    mv[0] += dx; mv[1] += dy
    for nid in fpref_nodes[ref]:
        nodes[nid]['x'] += dx; nodes[nid]['y'] += dy
    return True

def settle_rides():
    for _ in range(3):
        for nid, rd in rides.items():
            if rd[0] == 'seg':
                _, j, tt, ox, oy = rd
                t, na, nb = tnodes[j]; a, b = nodes[na], nodes[nb]
                nodes[nid]['x'] = a['x'] + (b['x']-a['x'])*tt + ox
                nodes[nid]['y'] = a['y'] + (b['y']-a['y'])*tt + oy
            else:
                _, vnid, ox, oy = rd
                nodes[nid]['x'] = nodes[vnid]['x'] + ox
                nodes[nid]['y'] = nodes[vnid]['y'] + oy

def flush():
    settle_rides()
    for t, na, nb in tnodes:
        a, b = nodes[na], nodes[nb]
        t.SetStart(pcbnew.VECTOR2I(int(round(a['x']/NM)), int(round(a['y']/NM))))
        t.SetEnd(pcbnew.VECTOR2I(int(round(b['x']/NM)), int(round(b['y']/NM))))
    for n in nodes:
        if n['via'] is not None:
            n['via'].SetPosition(pcbnew.VECTOR2I(int(round(n['x']/NM)), int(round(n['y']/NM))))
    for ref, mv in fpmv.items():
        if mv[0] or mv[1]:
            bx, by, f = fp_base[ref]
            f.SetPosition(pcbnew.VECTOR2I(int(round((bx+mv[0])/NM)), int(round((by+mv[1])/NM))))

def save_and_drc(out_json):
    flush()
    with quiet_stderr():
        pcbnew.ZONE_FILLER(bd).Fill(bd.Zones())
        pcbnew.SaveBoard(BOARD, bd)
    return run_drc(BOARD, out_json)

# ---- item parsing / geometry lookup -----------------------------------------
seg_by_nl = collections.defaultdict(list)
for i, (t, na, nb) in enumerate(tnodes):
    seg_by_nl[(t.GetNetname(), bd.GetLayerName(t.GetLayer()))].append(i)
via_by_net = collections.defaultdict(list)
for vo in vias: via_by_net[vo.GetNetname()].append(vo)
via_node_by_net = collections.defaultdict(list)
for nid, nd in enumerate(nodes):
    if nd['via'] is not None:
        via_node_by_net[nd['via'].GetNetname()].append(nid)
pad_by_ref = collections.defaultdict(list)
for f in bd.GetFootprints():
    for p in f.Pads(): pad_by_ref[f.GetReference()].append(p)
IT_TRACK = re.compile(r'^Track \[([^\]]+)\] on ([A-Za-z0-9.]+)')
IT_VIA = re.compile(r'^Via \[([^\]]+)\]')
IT_PAD = re.compile(r'pad (\S+) \[([^\]]+)\] of (\S+)')
IT_NPTH = re.compile(r'^NPTH pad of (\S+)')
DEFICIT = re.compile(r'clearance ([\d.]+) mm; actual ([\d.]+) mm')
def parse_item(it):
    d = it['description']; pos = it.get('pos', {})
    x, y = pos.get('x'), pos.get('y')
    m = IT_TRACK.match(d)
    if m: return dict(kind='track', net=m.group(1), lay=m.group(2), x=x, y=y)
    m = IT_VIA.match(d)
    if m: return dict(kind='via', net=m.group(1), x=x, y=y)
    m = IT_PAD.search(d)
    if m: return dict(kind='pad', num=m.group(1), net=m.group(2), ref=m.group(3), x=x, y=y)
    m = IT_NPTH.match(d)
    if m: return dict(kind='pad', num=None, net=None, ref=m.group(1), x=x, y=y)
    m = re.match(r'^Footprint (\S+)', d)
    if m: return dict(kind='fp', ref=m.group(1), x=x, y=y)
    if d.startswith('Zone'): return dict(kind='zone', x=x, y=y)
    if 'Edge.Cuts' in d: return dict(kind='edge', x=x, y=y)
    return dict(kind='other', x=x, y=y)
def nearest_seg(net, layname, x, y):
    best = None
    for i in seg_by_nl.get((net, layname), ()):
        t, na, nb = tnodes[i]
        a, b = nodes[na], nodes[nb]
        d, tt = pt_seg(x, y, a['x'], a['y'], b['x'], b['y'])
        cx_ = a['x'] + (b['x']-a['x'])*tt; cy_ = a['y'] + (b['y']-a['y'])*tt
        if best is None or d < best[0]: best = (d, i, (cx_, cy_), tt)
    return best
def other_point(o, x, y):
    if o is None: return None
    if o['kind'] == 'track':
        b = nearest_seg(o['net'], o['lay'], x, y)
        if b: return b[2]
    if o['kind'] == 'via':
        best = None
        for vo in via_by_net.get(o['net'], ()):
            p = vo.GetPosition(); d = math.hypot(p.x*NM-x, p.y*NM-y)
            if best is None or d < best[0]: best = (d, (p.x*NM, p.y*NM))
        if best: return best[1]
    if o['kind'] == 'pad':
        best = None
        for p in pad_by_ref.get(o['ref'], ()):
            if o['num'] and p.GetNumber() != o['num']: continue
            pp = p.GetPosition(); d = math.hypot(pp.x*NM-x, pp.y*NM-y)
            if best is None or d < best[0]: best = (d, (pp.x*NM, pp.y*NM))
        if best: return best[1]
    return (o['x'], o['y']) if o.get('x') is not None else None
def chain_weights(seeds):
    dist = dict(seeds)
    heap = [(v, k) for k, v in dist.items()]; heapq.heapify(heap)
    while heap:
        dcur, n = heapq.heappop(heap)
        if dcur > FALL or dist.get(n, 1e9) < dcur: continue
        for m, L2 in adj[n]:
            nd2 = dcur + L2
            if nd2 < dist.get(m, 1e9) and nd2 <= FALL:
                dist[m] = nd2; heapq.heappush(heap, (nd2, m))
    return {n: max(0.0, 1.0 - d/FALL) for n, d in dist.items()
            if not nodes[n]['pinned'] and n not in RIDE and 1.0 - d/FALL > 0.02}

# ---- site registry ----------------------------------------------------------
def pair_sig(v):
    return (v['type'], tuple(sorted(norm(i['description']) for i in v['items'])))
registry = []          # dict(sig, x, y, attempts, manual, v, deficit)
def cluster_sites(new):
    insts = collections.defaultdict(list)
    for v in new:
        if v['type'] not in NUDGE_TYPES: continue
        p = v['items'][0].get('pos', {})
        insts[pair_sig(v)].append((p.get('x', 0), p.get('y', 0), v))
    sites = []
    for sig, pts in insts.items():
        used = [False]*len(pts)
        for i in range(len(pts)):
            if used[i]: continue
            cl = [pts[i]]; used[i] = True
            for j in range(i+1, len(pts)):
                if used[j]: continue
                if any(math.hypot(pts[j][0]-c[0], pts[j][1]-c[1]) < 0.4 for c in cl):
                    cl.append(pts[j]); used[j] = True
            cx_ = sum(c[0] for c in cl)/len(cl); cy_ = sum(c[1] for c in cl)/len(cl)
            deficit = 0.0
            for c in cl:
                m = DEFICIT.search(c[2]['description'])
                if m: deficit = max(deficit, float(m.group(1)) - float(m.group(2)))
            sites.append(dict(sig=sig, x=cx_, y=cy_, v=cl[0][2], deficit=deficit or 0.05))
    return sites
def match_registry(s):
    for r in registry:
        if r['sig'] == s['sig'] and math.hypot(r['x']-s['x'], r['y']-s['y']) < 0.8:
            r['improving'] = s['deficit'] < r['deficit'] - 0.003
            r.update(x=s['x'], y=s['y'], v=s['v'], deficit=s['deficit']); return r
    r = dict(sig=s['sig'], x=s['x'], y=s['y'], attempts=0, manual=False,
             v=s['v'], deficit=s['deficit'])
    registry.append(r); return r

log = dict(rounds=[])
hist = []; best_sites = 10**9; best_round = 0
import shutil
for rnd in range(1, ROUNDS+1):
    cur = save_and_drc(os.path.join(os.path.dirname(LOG), f'drc_r{rnd}.json'))
    new, gone = diff(base, cur)
    new = [v for v in new if v['type'] not in IGNORE_TYPES]
    sites = cluster_sites(new)
    live = []
    for s in sites:
        r = match_registry(s)
        if r['manual']: continue
        if r['attempts'] >= ATTEMPT_CAP: r['manual'] = True; continue
        live.append(r)
    desired = collections.defaultdict(lambda: [0.0, 0.0])
    n_push = 0
    for r in live:
        if not r.pop('improving', False):
            r['attempts'] += 1
        v = r['v']; items = [parse_item(i) for i in v['items']]
        if v['type'] == 'courtyards_overlap':
            fps = [o for o in items if o['kind'] == 'fp']
            if len(fps) == 2 and all(o['ref'] in fp_base for o in fps):
                a, b = fps
                if npads_fp.get(a['ref'], 99) > npads_fp.get(b['ref'], 99): a, b = b, a
                ax, ay = cur_fp_pos(a['ref']); bx2, by2 = cur_fp_pos(b['ref'])
                ux, uy = ax - bx2, ay - by2; Lu = math.hypot(ux, uy) or 1.0
                amt = 0.02 + 0.01 * r['attempts']
                if move_fp(a['ref'], ux/Lu*amt, uy/Lu*amt): n_push += 1
                else: r['manual'] = True
            else:
                r['manual'] = True
            continue
        cand = [i for i, o in enumerate(items)
                if o['kind'] in ('track', 'via') and o.get('net') not in PROTECT]
        if not cand: r['manual'] = True; continue
        moved_any = False
        for ci in cand:
            o = items[ci]; other = items[1-ci] if len(items) > 1 else None
            if o['kind'] == 'track':
                b = nearest_seg(o['net'], o['lay'], r['x'], r['y'])
                if not b: continue
                d, seg_i, cpt, tfrac = b
                t, na2, nb2 = tnodes[seg_i]
                a2, b2 = nodes[na2], nodes[nb2]
                L = math.hypot(b2['x']-a2['x'], b2['y']-a2['y'])
                seeds = {na2: tfrac*L, nb2: (1.0-tfrac)*L}
            else:
                best = None
                for nid in via_node_by_net.get(o['net'], ()):
                    nd2 = nodes[nid]
                    dd = math.hypot(nd2['x']-r['x'], nd2['y']-r['y'])
                    if best is None or dd < best[0]: best = (dd, nid)
                if best is None or best[0] > 1.5: continue
                nid = best[1]
                if nodes[nid]['pinned'] or nid in RIDE: continue
                cpt = (nodes[nid]['x'], nodes[nid]['y'])
                seeds = {nid: 0.0}
            op = other_point(other, cpt[0], cpt[1])
            if op is None: continue
            ux, uy = cpt[0]-op[0], cpt[1]-op[1]; Lu = math.hypot(ux, uy)
            if Lu < 1e-6:
                ux, uy = 1.0, 0.0; Lu = 1.0
            ux, uy = ux/Lu, uy/Lu
            wts = chain_weights(seeds)
            if not wts: continue
            # damp repeat pushes that are not improving their site: oscillating
            # pairs otherwise dig deeper holes than the stretch ever made
            amount = (r['deficit'] + MARGIN) * (0.6 ** max(0, r['attempts'] - 1)) / len(cand)
            for n, w in wts.items():
                desired[n][0] += ux*amount*w; desired[n][1] += uy*amount*w
            moved_any = True
        if moved_any: n_push += 1
        else: r['manual'] = True
    for n, (dx, dy) in desired.items():
        mag = math.hypot(dx, dy)
        if mag < 1e-9: continue
        f = min(1.0, RCAP/mag)
        nd = nodes[n]
        rem = CAP - math.hypot(nd['mv'][0], nd['mv'][1])
        if rem <= 0: continue
        f = min(f, rem/mag)
        nd['x'] += dx*f; nd['y'] += dy*f; nd['mv'][0] += dx*f; nd['mv'][1] += dy*f
    n_manual = sum(1 for r in registry if r['manual'])
    hist.append(len(sites))
    log['rounds'].append(dict(rnd=rnd, new=len(new), sites=len(sites),
                              pushed=n_push, manual=n_manual))
    print(f'round {rnd}: {len(new)} new ({len(sites)} sites), pushed {n_push}, '
          f'manual {n_manual}, airwires {len(cur["unconnected_items"])}', flush=True)
    if len(sites) < best_sites:
        best_sites = len(sites); best_round = rnd
        shutil.copy(BOARD, BOARD + '.best')
    if n_push == 0: break
    if rnd - best_round >= 4:
        print('no new best for 4 rounds — stopping'); break

cur = save_and_drc(os.path.join(os.path.dirname(LOG), 'drc_final.json'))
new, gone = diff(base, cur)
_s = cluster_sites([v for v in new if v['type'] not in IGNORE_TYPES])
if len(_s) > best_sites and os.path.exists(BOARD + '.best'):
    print(f'final ({len(_s)} sites) worse than round {best_round} ({best_sites}) — restoring best')
    shutil.copy(BOARD + '.best', BOARD)
    cur = run_drc(BOARD, os.path.join(os.path.dirname(LOG), 'drc_final.json'))
    new, gone = diff(base, cur)
new = [v for v in new if v['type'] not in IGNORE_TYPES]
sites = cluster_sites(new)
extra = [v for v in new if v['type'] not in NUDGE_TYPES]
log['remaining'] = [dict(type=s['v']['type'], x=round(s['x'], 3), y=round(s['y'], 3),
                         deficit=round(s['deficit'], 4), desc=s['v']['description'],
                         items=[i['description'] for i in s['v']['items']])
                    for s in sorted(sites, key=lambda s: -s['deficit'])]
log['other'] = [dict(type=v['type'], desc=v['description'],
                     items=[i['description'] for i in v['items']]) for v in extra]
log['final_new'] = len(new); log['final_sites'] = len(sites)
log['unconnected'] = len(cur['unconnected_items'])
log['rides'] = len(rides)
json.dump(log, open(LOG, 'w'), indent=1)
print(f'final: {len(new)} new violations in {len(sites)} sites '
      f'(+{len(extra)} non-nudgeable); airwires {len(cur["unconnected_items"])}')
