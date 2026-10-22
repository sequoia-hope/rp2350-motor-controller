"""Stage 2: Hybrid Harmonic-Field + Feasibility-Projected Settler.

Combines two complementary morphing paradigms:
  1. Feasibility-projected line search (`settle`):
     - Exact polygon/segment/via/hole/courtyard/attachment/net-length constraint graph
     - Monotone DRC floor: `req_f(g) = min(g.rule, g.d0 - 2e-4)` — never creates a
       new violation and never worsens a pre-existing tight spot
     - Stagnation detection, dynamic node splits, and mid-flight multi-layer A* reroute
  2. Per-layer coupled harmonic warp field (`LayerFieldStack` from `field.py`):
     - Pre-factored per-copper-layer Laplace solver (`splu`) with all-layer drill
       pinning and 2-pass dynamic via barrel coupling
     - Evaluates each free copper node's smooth spatial harmonic equilibrium
       `P_harm(nid) = P_0(nid) + U_L(P_0(nid))` as footprints move (with optional
       lookahead `beta` so copper corridors part smoothly ahead of advancing pads)
     - Replaces stationary waiting + point-collision kinking with coherent multi-net
       laminar bundle flow, while every node move remains strictly projected onto the
       DRC-admissible polytope via `line_search()`.
"""
import sys
import math
import time
import heapq
from collections import defaultdict
import numpy as np

from .common import (
    COPPER_LAYERS, F_CU, B_CU,
    load_config, data_path, load_json, save_json,
    pt_seg_dist, seg_seg_dist, point_in_poly,
    poly_seg_dist, poly_pt_dist, poly_poly_dist,
    inside_dist, inside_seg_dist, inside_poly_dist, inflate_poly, rdp,
    resolve_elastic,
)
from .field import LayerFieldStack
from .accel import CudaSettlerBackend, KIND_TO_ID, _load_cuda_lib


def pair_clearance(rules, net_a, net_b):
    if net_a == net_b and net_a != '':
        return 0.0
    ca = rules['net_to_class'].get(net_a, 'Default')
    cb = rules['net_to_class'].get(net_b, 'Default')
    cla = rules['netclasses'].get(ca, {}).get('clearance', rules['default_clearance'])
    clb = rules['netclasses'].get(cb, {}).get('clearance', rules['default_clearance'])
    base = max(cla, clb, rules['default_clearance'])
    if ca != cb:
        k = '|'.join(sorted([ca, cb]))
        base = max(base, rules['class_matrix'].get(k, 0.0))
    return base


class GEdge:
    __slots__ = ('kind', 'a', 'b', 'rule', 'd0', 'cur', 'layer')

    def __init__(self, kind, a, b, rule, d0, layer=-1):
        self.kind = kind
        self.a = a
        self.b = b
        self.rule = rule
        self.d0 = d0
        self.cur = d0
        self.layer = layer


def run_settle(cfg, use_field=None, use_gpu=None, verbose=True):
    """Run the settler (classic or hybrid harmonic-field, with optional CUDA GPU acceleration) on `model.json`."""
    t_start = time.time()
    resolve_elastic(cfg)
    m = load_json(data_path(cfg, 'model.json'))
    rules = m['rules']
    CY_GAP = float(cfg.get('courtyard_gap', 0.12))
    SUB = float(m.get('subdivide_mm', 0.8))

    fcfg = cfg.get('field') or {}
    if use_field is None:
        use_field = bool(fcfg.get('enabled', bool(fcfg)))
    if use_gpu is None:
        use_gpu = bool(cfg.get('use_gpu', True))

    parts_meta = m['parts']
    part_refs = sorted(parts_meta.keys())
    D = {r: [0.0, 0.0] for r in part_refs}
    TX = {r: parts_meta[r]['tx'] for r in part_refs}
    TY = {r: parts_meta[r]['ty'] for r in part_refs}
    CY0 = {r: [(p[0], p[1]) for p in parts_meta[r]['courtyard']] for r in part_refs}
    SIDE = {r: parts_meta[r]['side'] for r in part_refs}

    static_cys = [(sc['ref'], sc['side'], [(p[0], p[1]) for p in sc['courtyard']])
                  for sc in m['static_courtyards']]

    pads = m['pads']
    P_POLY0 = [[(p[0], p[1]) for p in inflate_poly(pad['poly'], 0.005)] for pad in pads]
    P_XY0 = [(pad['x'], pad['y']) for pad in pads]
    P_REF = [pad['ref'] for pad in pads]
    P_NET = [pad['net'] for pad in pads]
    P_LAYERS = [set(pad['layers']) for pad in pads]
    P_DRILL = [pad['drill'] for pad in pads]
    P_MOV = [pad['ref'] in D for pad in pads]
    pad_lookup = {(pad['ref'], pad['num']): i for i, pad in enumerate(pads)}

    nodes = m['nodes']
    P = [[n['x'], n['y']] for n in nodes]
    P0 = [(n['x0'], n['y0']) for n in nodes]
    N_NET = [n['net'] for n in nodes]
    is_via = {n['id']: (n['size'] / 2.0, n['drill'] / 2.0)
              for n in nodes if n['kind'] == 'via'}
    part_of = {n['id']: n['part'] for n in nodes if n.get('part')}
    fixed = {n['id'] for n in nodes if n.get('fixed')}
    part_nodes = defaultdict(list)
    for nid, r in part_of.items():
        part_nodes[r].append(nid)

    segs = [[s['a'], s['b'], s['w'], s['layer'], s['net']] for s in m['segs']]
    st_tracks = [(t['ax'], t['ay'], t['bx'], t['by'], t['w'] / 2.0, t['layer'], t['net'])
                 for t in m['static_tracks']]
    st_vias = [(v['x'], v['y'], v['size'] / 2.0, v['drill'] / 2.0, v['net'])
               for v in m['static_vias']]
    zones = m['zones']
    outline0 = [(p[0], p[1]) for p in m['outline']]
    target_outline = [(p[0], p[1]) for p in m.get('target_outline', m['outline'])]
    outline = list(outline0)
    has_shape = (
        len(outline0) > 0 and
        len(outline0) == len(target_outline) and
        any(math.hypot(a[0] - b[0], a[1] - b[1]) > 1e-4 for a, b in zip(outline0, target_outline))
    )
    s_edge = 0.0
    shape_blocker = None

    def set_outline_s(s):
        nonlocal s_edge
        s_edge = s
        for k_ol in range(len(outline0)):
            p0 = outline0[k_ol]
            p1 = target_outline[k_ol]
            outline[k_ol] = (p0[0] + s * (p1[0] - p0[0]), p0[1] + s * (p1[1] - p0[1]))

    # Optional harmonic field stack
    field_stack = None
    field_solve_ms = 0.0
    if use_field:
        t_f0 = time.time()
        field_stack = LayerFieldStack(
            m,
            h=float(fcfg.get('h', 0.25)),
            pad_inflate=float(fcfg.get('pad_inflate', 0.15)),
            bridge_mm=float(fcfg.get('bridge_mm', 1.0)),
            ambient_spec=fcfg.get('ambient'),
        )
        field_solve_ms = (time.time() - t_f0) * 1000.0
        if verbose:
            print(f"[pcbmorph.settle] LayerFieldStack initialized on layers "
                  f"{field_stack.layers} in {field_solve_ms:.1f} ms")

    def pad_poly(i):
        r = P_REF[i]
        if r in D:
            dx, dy = D[r]
            return [(x + dx, y + dy) for x, y in P_POLY0[i]]
        return P_POLY0[i]

    def pad_xy(i):
        r = P_REF[i]
        x, y = P_XY0[i]
        if r in D:
            return x + D[r][0], y + D[r][1]
        return x, y

    def cy_poly(r):
        dx, dy = D[r]
        return [(x + dx, y + dy) for x, y in CY0[r]]

    adj = defaultdict(set)
    node_segs = defaultdict(list)
    net_segs = defaultdict(list)

    def rebuild_adj():
        adj.clear()
        node_segs.clear()
        net_segs.clear()
        for si, s in enumerate(segs):
            a, b, w, l, net = s
            adj[a].add(b)
            adj[b].add(a)
            node_segs[a].append(si)
            node_segs[b].append(si)
            net_segs[net].append(si)

    rebuild_adj()

    def net_len(net):
        return sum(math.hypot(P[segs[si][0]][0] - P[segs[si][1]][0],
                              P[segs[si][0]][1] - P[segs[si][1]][1])
                   for si in net_segs[net])

    LEN0 = {net: net_len(net) for net in net_segs}
    LEN_SLACK = float(cfg.get('net_len_slack_mm', 4.0))
    LEN_RATIO = float(cfg.get('net_len_ratio', 1.25))
    LEN_CAP = {net: max(l0 * LEN_RATIO, l0 + LEN_SLACK) for net, l0 in LEN0.items()}

    AT = []
    AZ = []
    PZ_SAME = defaultdict(list)  # ref -> list of (pi, zi, min_inside_dist)
    for a in m['attachments']:
        if a['kind'] == 'pad':
            nid = a['node']
            key = (a['ref'], a['pad'])
            if key in pad_lookup:
                pi = pad_lookup[key]
                d0 = poly_pt_dist(pad_poly(pi), P[nid][0], P[nid][1])
                AT.append((nid, pi, max(0.05, d0 + 0.02)))
        elif a['kind'] == 'zone':
            nid = a['node']
            zi = a['zone_idx']
            AZ.append((nid, zi, 0.02))
        elif a['kind'] == 'pad_zone':
            key = (a['ref'], a['pad'])
            if key in pad_lookup:
                pi = pad_lookup[key]
                zi = a['zone_idx']
                px0, py0 = pad_xy(pi)
                d_in0 = inside_dist(px0, py0, zones[zi]['poly'])
                xs_ = [q[0] for q in P_POLY0[pi]]
                ys_ = [q[1] for q in P_POLY0[pi]]
                rin = 0.5 * min(max(xs_) - min(xs_), max(ys_) - min(ys_))
                PZ_SAME[a['ref']].append((pi, zi, min(rin, d_in0 - 1e-3)))

    GUARD = float(cfg.get('guard_mm', 0.015))

    # Optional CUDA GPU backend
    gpu = None
    if use_gpu and _load_cuda_lib() is not None:
        try:
            gpu = CudaSettlerBackend(
                pads, P_POLY0, P_XY0, P_DRILL, P_REF,
                st_tracks, st_vias, zones, outline
            )
            gpu.update_graph(P, P0, is_via, segs, adj, AT, AZ)
        except Exception:
            gpu = None

    edges = []
    part_edges = defaultdict(list)
    node_edges = defaultdict(list)
    shape_edges = []
    d0_cache = {}
    CULL = 1.8
    BB_CULL = 2.6

    def eval_edge(g):
        k, a, b = g.kind, g.a, g.b
        if k == 'SS':
            s1, s2 = segs[a], segs[b]
            p1, p2 = P[s1[0]], P[s1[1]]
            q1, q2 = P[s2[0]], P[s2[1]]
            return seg_seg_dist(p1[0], p1[1], p2[0], p2[1],
                                q1[0], q1[1], q2[0], q2[1]) - (s1[2] + s2[2]) * 0.5
        if k == 'SP':
            s = segs[a]
            p1, p2 = P[s[0]], P[s[1]]
            return poly_seg_dist(pad_poly(b), p1[0], p1[1], p2[0], p2[1]) - s[2] * 0.5
        if k == 'SV':
            s = segs[a]
            p1, p2 = P[s[0]], P[s[1]]
            vx, vy = P[b]
            return pt_seg_dist(vx, vy, p1[0], p1[1], p2[0], p2[1]) - s[2] * 0.5 - is_via[b][0]
        if k == 'SST':
            s = segs[a]
            p1, p2 = P[s[0]], P[s[1]]
            tx1, ty1, tx2, ty2, hr, _, _ = st_tracks[b]
            return seg_seg_dist(p1[0], p1[1], p2[0], p2[1], tx1, ty1, tx2, ty2) - s[2] * 0.5 - hr
        if k == 'SSV':
            s = segs[a]
            p1, p2 = P[s[0]], P[s[1]]
            vx, vy, vr, _, _ = st_vias[b]
            return pt_seg_dist(vx, vy, p1[0], p1[1], p2[0], p2[1]) - s[2] * 0.5 - vr
        if k == 'SH_P':
            s = segs[a]
            p1, p2 = P[s[0]], P[s[1]]
            px, py = pad_xy(b)
            return pt_seg_dist(px, py, p1[0], p1[1], p2[0], p2[1]) - s[2] * 0.5 - P_DRILL[b] * 0.5
        if k == 'SH_V':
            s = segs[a]
            p1, p2 = P[s[0]], P[s[1]]
            vx, vy = P[b]
            return pt_seg_dist(vx, vy, p1[0], p1[1], p2[0], p2[1]) - s[2] * 0.5 - is_via[b][1]
        if k == 'SH_SV':
            s = segs[a]
            p1, p2 = P[s[0]], P[s[1]]
            vx, vy, _, dr, _ = st_vias[b]
            return pt_seg_dist(vx, vy, p1[0], p1[1], p2[0], p2[1]) - s[2] * 0.5 - dr
        if k == 'VP':
            vx, vy = P[a]
            return poly_pt_dist(pad_poly(b), vx, vy) - is_via[a][0]
        if k == 'VV':
            v1, v2 = P[a], P[b]
            return math.hypot(v1[0] - v2[0], v1[1] - v2[1]) - is_via[a][0] - is_via[b][0]
        if k == 'VSV':
            v1 = P[a]
            vx, vy, vr, _, _ = st_vias[b]
            return math.hypot(v1[0] - vx, v1[1] - vy) - is_via[a][0] - vr
        if k == 'VST':
            v1 = P[a]
            tx1, ty1, tx2, ty2, hr, _, _ = st_vias[b] if False else st_tracks[b]
            return pt_seg_dist(v1[0], v1[1], tx1, ty1, tx2, ty2) - is_via[a][0] - hr
        if k == 'VH_P':
            v1 = P[a]
            px, py = pad_xy(b)
            return math.hypot(v1[0] - px, v1[1] - py) - is_via[a][0] - P_DRILL[b] * 0.5
        if k == 'HH_VV':
            v1, v2 = P[a], P[b]
            return math.hypot(v1[0] - v2[0], v1[1] - v2[1]) - is_via[a][1] - is_via[b][1]
        if k == 'HH_VSV':
            v1 = P[a]
            vx, vy, _, dr, _ = st_vias[b]
            return math.hypot(v1[0] - vx, v1[1] - vy) - is_via[a][1] - dr
        if k == 'HH_VP':
            v1 = P[a]
            px, py = pad_xy(b)
            return math.hypot(v1[0] - px, v1[1] - py) - is_via[a][1] - P_DRILL[b] * 0.5
        if k == 'PP':
            return poly_poly_dist(pad_poly(a), pad_poly(b))
        if k == 'PST':
            tx1, ty1, tx2, ty2, hr, _, _ = st_tracks[b]
            return poly_seg_dist(pad_poly(a), tx1, ty1, tx2, ty2) - hr
        if k == 'PSV':
            vx, vy, vr, _, _ = st_vias[b]
            return poly_pt_dist(pad_poly(a), vx, vy) - vr
        if k == 'SZ':
            s = segs[a]
            p1, p2 = P[s[0]], P[s[1]]
            return poly_seg_dist(zones[b]['poly'], p1[0], p1[1], p2[0], p2[1]) - s[2] * 0.5
        if k == 'VZ':
            v1 = P[a]
            return poly_pt_dist(zones[b]['poly'], v1[0], v1[1]) - is_via[a][0]
        if k == 'PZ':
            return poly_poly_dist(pad_poly(a), zones[b]['poly'])
        if k == 'SE':
            s = segs[a]
            p1, p2 = P[s[0]], P[s[1]]
            return inside_seg_dist(p1[0], p1[1], p2[0], p2[1], outline) - s[2] * 0.5
        if k == 'VE':
            v1 = P[a]
            return inside_dist(v1[0], v1[1], outline) - is_via[a][0]
        if k == 'PE':
            return inside_poly_dist(pad_poly(a), outline)
        return 99.0

    cand_kab = []
    cand_meta = []
    clr_cache = {}

    def get_clr(n1, n2):
        key = (n1, n2)
        res = clr_cache.get(key)
        if res is None:
            res = pair_clearance(rules, n1, n2)
            clr_cache[key] = res
            clr_cache[(n2, n1)] = res
        return res

    st_by_layer = defaultdict(list)
    for ti, st in enumerate(st_tracks):
        st_by_layer[st[5]].append((ti, st[6], 0.5 * (st[0] + st[2]), 0.5 * (st[1] + st[3])))

    pads_by_layer = defaultdict(list)
    drill_pads = []
    mov_pads = []
    stat_pads = []
    for pi in range(len(pads)):
        for lyr in P_LAYERS[pi]:
            pads_by_layer[lyr].append(pi)
        if P_DRILL[pi] > 0:
            drill_pads.append(pi)
        if P_MOV[pi]:
            mov_pads.append(pi)
        else:
            stat_pads.append(pi)

    def _commit_edge(kind, a, b, rule, layer, r_list, n_list, d):
        cull_lim = CULL + 1.8 if (has_shape and kind in ('SE', 'VE', 'PE')) else CULL
        if d > cull_lim:
            return
        g = GEdge(kind, a, b, rule, 0.0, layer)
        key = (kind, a, b)
        if key not in d0_cache:
            d0_cache[key] = d
        g.d0 = d0_cache[key]
        g.cur = d
        idx = len(edges)
        edges.append(g)
        if kind in ('SE', 'VE', 'PE'):
            shape_edges.append(idx)
        for r in set(r_list):
            if r in D:
                part_edges[r].append(idx)
        for n in set(n_list):
            node_edges[n].append(idx)

    def reg_edge(kind, a, b, rule, layer=-1, r_list=(), n_list=()):
        if gpu is not None:
            cand_kab.append((KIND_TO_ID[kind], a, b))
            cand_meta.append((kind, a, b, rule, layer, r_list, n_list))
            return
        g = GEdge(kind, a, b, rule, 0.0, layer)
        d = eval_edge(g)
        _commit_edge(kind, a, b, rule, layer, r_list, n_list, d)

    def build_edges():
        edges.clear()
        part_edges.clear()
        node_edges.clear()
        shape_edges.clear()
        cand_kab.clear()
        cand_meta.clear()
        h_clr = rules['hole_clearance']
        hh_clr = rules['hole_to_hole']
        e_clr = rules['copper_edge']

        segs_by_layer = defaultdict(list)
        for idx_s, s in enumerate(segs):
            segs_by_layer[s[3]].append(idx_s)
        seg_nl = [(s[0], s[1]) for s in segs]
        seg_rl = [[part_of[x] for x in (s[0], s[1]) if x in part_of] for s in segs]
        vids = sorted(is_via.keys())
        via_rl = {vid: ([part_of[vid]] if vid in part_of else []) for vid in vids}

        seg_mid = [(0.5 * (P[s[0]][0] + P[s[1]][0]), 0.5 * (P[s[0]][1] + P[s[1]][1])) for s in segs]
        pad_cur = [pad_xy(pi) for pi in range(len(pads))]

        for l1, l_segs in segs_by_layer.items():
            l_pads = pads_by_layer[l1]
            l_st = st_by_layer[l1]
            n_ls = len(l_segs)
            for idx1 in range(n_ls):
                i = l_segs[idx1]
                a1, b1, w1, _, net1 = segs[i]
                mx1, my1 = seg_mid[i]
                nl1 = seg_nl[i]
                rl1 = seg_rl[i]
                if outline:
                    reg_edge('SE', i, -1, e_clr, l1, rl1, nl1)
                for idx2 in range(idx1 + 1, n_ls):
                    j = l_segs[idx2]
                    net2 = segs[j][4]
                    if net1 == net2:
                        continue
                    mx2, my2 = seg_mid[j]
                    if abs(mx1 - mx2) > BB_CULL or abs(my1 - my2) > BB_CULL:
                        continue
                    rl = get_clr(net1, net2)
                    nl2 = nl1 + seg_nl[j]
                    rl2 = rl1 + seg_rl[j]
                    reg_edge('SS', i, j, rl, l1, rl2, nl2)
                for pi in l_pads:
                    if P_NET[pi] == net1:
                        continue
                    px, py = pad_cur[pi]
                    if abs(mx1 - px) > BB_CULL + 1.5 or abs(my1 - py) > BB_CULL + 1.5:
                        continue
                    rl = get_clr(net1, P_NET[pi])
                    reg_edge('SP', i, pi, rl, l1, rl1 + [P_REF[pi]], nl1)
                for pi in drill_pads:
                    if P_NET[pi] == net1:
                        continue
                    px, py = pad_cur[pi]
                    if abs(mx1 - px) > BB_CULL + 1.5 or abs(my1 - py) > BB_CULL + 1.5:
                        continue
                    reg_edge('SH_P', i, pi, h_clr, l1, rl1 + [P_REF[pi]], nl1)
                for vid in vids:
                    if N_NET[vid] != net1:
                        vx, vy = P[vid]
                        if abs(mx1 - vx) > BB_CULL or abs(my1 - vy) > BB_CULL:
                            continue
                        rl = get_clr(net1, N_NET[vid])
                        rlv = rl1 + via_rl[vid]
                        nlv = nl1 + (vid,)
                        reg_edge('SV', i, vid, rl, l1, rlv, nlv)
                        reg_edge('SH_V', i, vid, h_clr, l1, rlv, nlv)
                for ti, st_net, smx, smy in l_st:
                    if st_net != net1:
                        if abs(mx1 - smx) > BB_CULL + 2.0 or abs(my1 - smy) > BB_CULL + 2.0:
                            continue
                        rl = get_clr(net1, st_net)
                        reg_edge('SST', i, ti, rl, l1, rl1, nl1)
                for vi, sv in enumerate(st_vias):
                    if sv[4] != net1:
                        if abs(mx1 - sv[0]) > BB_CULL or abs(my1 - sv[1]) > BB_CULL:
                            continue
                        rl = get_clr(net1, sv[4])
                        reg_edge('SSV', i, vi, rl, l1, rl1, nl1)
                        reg_edge('SH_SV', i, vi, h_clr, l1, rl1, nl1)

        for idx_v, vid in enumerate(vids):
            vnet = N_NET[vid]
            vx, vy = P[vid]
            rlv = via_rl[vid]
            if outline:
                reg_edge('VE', vid, -1, e_clr, -1, rlv, (vid,))
            for vid2 in vids[idx_v + 1:]:
                if abs(vx - P[vid2][0]) > BB_CULL or abs(vy - P[vid2][1]) > BB_CULL:
                    continue
                rlv2 = rlv + via_rl[vid2]
                reg_edge('HH_VV', vid, vid2, hh_clr, -1, rlv2, (vid, vid2))
                if N_NET[vid2] != vnet:
                    rl = get_clr(vnet, N_NET[vid2])
                    reg_edge('VV', vid, vid2, rl, -1, rlv2, (vid, vid2))
            for pi in range(len(pads)):
                px, py = pad_cur[pi]
                if abs(vx - px) > BB_CULL + 1.5 or abs(vy - py) > BB_CULL + 1.5:
                    continue
                if P_DRILL[pi] > 0:
                    reg_edge('HH_VP', vid, pi, hh_clr, -1, rlv + [P_REF[pi]], (vid,))
                if P_NET[pi] != vnet:
                    rl = get_clr(vnet, P_NET[pi])
                    reg_edge('VP', vid, pi, rl, -1, rlv + [P_REF[pi]], (vid,))
                    if P_DRILL[pi] > 0:
                        reg_edge('VH_P', vid, pi, h_clr, -1, rlv + [P_REF[pi]], (vid,))
            for l1, l_st in st_by_layer.items():
                for ti, st_net, smx, smy in l_st:
                    if st_net != vnet:
                        if abs(vx - smx) > BB_CULL + 2.0 or abs(vy - smy) > BB_CULL + 2.0:
                            continue
                        rl = get_clr(vnet, st_net)
                        reg_edge('VST', vid, ti, rl, l1, rlv, (vid,))
            for vi, sv in enumerate(st_vias):
                if abs(vx - sv[0]) > BB_CULL or abs(vy - sv[1]) > BB_CULL:
                    continue
                reg_edge('HH_VSV', vid, vi, hh_clr, -1, rlv, (vid,))
                if sv[4] != vnet:
                    rl = get_clr(vnet, sv[4])
                    reg_edge('VSV', vid, vi, rl, -1, rlv, (vid,))

        for idx_m, pi in enumerate(mov_pads):
            r1 = P_REF[pi]
            net1 = P_NET[pi]
            px1, py1 = pad_cur[pi]
            if outline:
                reg_edge('PE', pi, -1, e_clr, -1, [r1], ())
            for pj in mov_pads[idx_m + 1:]:
                if P_REF[pj] == r1 or P_NET[pj] == net1:
                    continue
                px2, py2 = pad_cur[pj]
                if abs(px1 - px2) > BB_CULL + 2.0 or abs(py1 - py2) > BB_CULL + 2.0:
                    continue
                if P_LAYERS[pi].intersection(P_LAYERS[pj]):
                    rl = get_clr(net1, P_NET[pj])
                    reg_edge('PP', pi, pj, rl, -1, [r1, P_REF[pj]], ())
            for pj in stat_pads:
                if P_NET[pj] == net1:
                    continue
                px2, py2 = pad_cur[pj]
                if abs(px1 - px2) > BB_CULL + 2.0 or abs(py1 - py2) > BB_CULL + 2.0:
                    continue
                if P_LAYERS[pi].intersection(P_LAYERS[pj]):
                    rl = get_clr(net1, P_NET[pj])
                    reg_edge('PP', pi, pj, rl, -1, [r1], ())
            for l1 in P_LAYERS[pi]:
                for ti, st_net, smx, smy in st_by_layer[l1]:
                    if st_net != net1:
                        if abs(px1 - smx) > BB_CULL + 2.5 or abs(py1 - smy) > BB_CULL + 2.5:
                            continue
                        rl = get_clr(net1, st_net)
                        reg_edge('PST', pi, ti, rl, l1, [r1], ())
            for vi, sv in enumerate(st_vias):
                if sv[4] != net1:
                    if abs(px1 - sv[0]) > BB_CULL + 1.5 or abs(py1 - sv[1]) > BB_CULL + 1.5:
                        continue
                    rl = get_clr(net1, sv[4])
                    reg_edge('PSV', pi, vi, rl, -1, [r1], ())

        if gpu is not None and cand_kab:
            d_arr = gpu.eval_candidates(P, D, outline, cand_kab)
            for idx_c, (kind, a, b, rule, layer, r_list, n_list) in enumerate(cand_meta):
                _commit_edge(kind, a, b, rule, layer, r_list, n_list, float(d_arr[idx_c]))
            gpu.upload_edges_and_colors(
                edges, node_edges, part_edges, shape_edges,
                fixed, part_of, adj, GUARD
            )

    build_edges()

    def _poly_centroid(pts):
        return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))

    CY_PAIRS = []
    for i, r1 in enumerate(part_refs):
        c1_0 = cy_poly(r1)
        cx1, cy1 = _poly_centroid(c1_0)
        for r2 in part_refs[i + 1:]:
            if SIDE[r1] == SIDE[r2]:
                c2_0 = cy_poly(r2)
                d0 = poly_poly_dist(c1_0, c2_0)
                cx2, cy2 = _poly_centroid(c2_0)
                cd0 = math.hypot(cx1 - cx2, cy1 - cy2)
                fl = max(0.005, min(CY_GAP, d0 - 1e-5)) if d0 > 0.005 else 0.0
                CY_PAIRS.append(('MM', r1, r2, None, fl, d0, cd0))
        for sref, sside, scy in static_cys:
            if SIDE[r1] == sside:
                d0 = poly_poly_dist(c1_0, scy)
                if d0 < CULL:
                    cx2, cy2 = _poly_centroid(scy)
                    cd0 = math.hypot(cx1 - cx2, cy1 - cy2)
                    fl = max(0.005, min(CY_GAP, d0 - 1e-5)) if d0 > 0.005 else 0.0
                    CY_PAIRS.append(('MS', r1, sref, scy, fl, d0, cd0))

    GUARD = float(cfg.get('guard_mm', 0.015))

    def req_f(g):
        return min(g.rule + GUARD, g.d0)

    def edge_ok(g):
        d = eval_edge(g)
        return d >= g.d0 or d >= g.rule + GUARD

    press = defaultdict(lambda: [0.0, 0.0])

    def edge_grad_for_node(g, nid):
        px, py = P[nid]
        eps = 1e-3
        P[nid][0] = px + eps
        dx = eval_edge(g)
        P[nid][0] = px - eps
        dx -= eval_edge(g)
        P[nid][0] = px
        P[nid][1] = py + eps
        dy = eval_edge(g)
        P[nid][1] = py - eps
        dy -= eval_edge(g)
        P[nid][1] = py
        gn = math.hypot(dx, dy)
        if gn < 1e-9:
            return (0.0, 0.0)
        return (dx / gn, dy / gn)

    def edge_free_nodes(g):
        k, a, b = g.kind, g.a, g.b
        out = []
        if k in ('SS', 'SP', 'SV', 'SST', 'SSV', 'SH_P', 'SH_V', 'SH_SV', 'SZ', 'SE'):
            s = segs[a]
            for n in (s[0], s[1]):
                if n not in fixed and n not in part_of:
                    out.append(n)
        if k == 'SS':
            s2 = segs[b]
            for n in (s2[0], s2[1]):
                if n not in fixed and n not in part_of:
                    out.append(n)
        if k in ('SV', 'SH_V') and b not in fixed and b not in part_of:
            out.append(b)
        if k in ('VP', 'VV', 'VSV', 'VST', 'VH_P', 'HH_VV', 'HH_VSV', 'HH_VP', 'VZ', 'VE'):
            if a not in fixed and a not in part_of:
                out.append(a)
            if k in ('VV', 'HH_VV') and b not in fixed and b not in part_of:
                out.append(b)
        return out

    def check_part_Trial(r, oD, oN):
        c1 = cy_poly(r)
        for kind, r1, r2, scy, fl, d0_cy, cd0 in CY_PAIRS:
            if r1 != r and r2 != r:
                continue
            other = cy_poly(r2 if r1 == r else r1) if kind == 'MM' else scy
            d_cy = poly_poly_dist(c1, other)
            if d0_cy > 0.005:
                if d_cy < fl:
                    return ('CY', r2 if r1 == r else r1, None)
            else:
                if d_cy < d0_cy - 1e-6:
                    return ('CY', r2 if r1 == r else r1, None)
                if d_cy <= 0.005:
                    cx1, cy1 = _poly_centroid(c1)
                    cx2, cy2 = _poly_centroid(other)
                    if math.hypot(cx1 - cx2, cy1 - cy2) < cd0 - 1e-5:
                        return ('CY', r2 if r1 == r else r1, None)
        for pi, zi, fl_z in PZ_SAME.get(r, ()):
            px, py = pad_xy(pi)
            if inside_dist(px, py, zones[zi]['poly']) < fl_z:
                return ('PZ_SAME', zones[zi]['net'], pi)
        for nid, pi, cap in AT:
            if P_REF[pi] == r or nid in part_nodes[r]:
                if poly_pt_dist(pad_poly(pi), P[nid][0], P[nid][1]) > cap:
                    return ('AT', P_REF[pi], nid)
        aff_nets = {N_NET[n] for n in part_nodes[r] if N_NET[n] in LEN_CAP}
        for net in aff_nets:
            if net_len(net) > LEN_CAP[net]:
                return ('LEN', net, None)
        if gpu is not None:
            bad_ei = gpu.check_edges_subset(P, D, outline, gpu.part_edge_subsets.get(r))
            if bad_ei >= 0:
                return ('DRC', edges[bad_ei].kind, bad_ei)
            return None
        for ei in part_edges[r]:
            g = edges[ei]
            if not edge_ok(g):
                return ('DRC', g.kind, ei)
        return None

    def check_node_trial(nid, check_drc=True):
        for an, pi, cap in AT:
            if an == nid:
                if poly_pt_dist(pad_poly(pi), P[nid][0], P[nid][1]) > cap:
                    return ('AT', P_REF[pi], nid)
        for an, zi, _ in AZ:
            if an == nid:
                if not point_in_poly(P[nid][0], P[nid][1], zones[zi]['poly']):
                    return ('AZ', zones[zi]['net'], nid)
        net = N_NET[nid]
        if net in LEN_CAP and net_len(net) > LEN_CAP[net]:
            return ('LEN', net, nid)
        if not check_drc:
            return None
        if gpu is not None:
            bad_ei = gpu.check_edges_subset(P, D, outline, gpu.node_edge_subsets.get(nid))
            if bad_ei >= 0:
                return ('DRC', edges[bad_ei].kind, bad_ei)
            return None
        for ei in node_edges[nid]:
            g = edges[ei]
            if not edge_ok(g):
                return ('DRC', g.kind, ei)
        return None

    def line_search(apply_fn):
        if apply_fn(1.0) is None:
            return 1.0, None
        lo, hi = 0.0, 1.0
        blocker = apply_fn(1.0)
        for _ in range(8):
            mid = 0.5 * (lo + hi)
            res = apply_fn(mid)
            if res is None:
                lo = mid
            else:
                hi = mid
                blocker = res
        apply_fn(lo)
        return lo, blocker

    last_blockers = {}

    def part_pass(step):
        press.clear()
        order = sorted(part_refs,
                       key=lambda r: math.hypot(TX[r] - D[r][0], TY[r] - D[r][1]),
                       reverse=True)
        moved = 0.0
        for r in order:
            rx, ry = TX[r] - D[r][0], TY[r] - D[r][1]
            rem = math.hypot(rx, ry)
            if rem < 1e-4:
                continue
            sc = min(1.0, step / rem)
            vx, vy = rx * sc, ry * sc
            oD = (D[r][0], D[r][1])
            oN = [(nid, P[nid][0], P[nid][1]) for nid in part_nodes[r]]

            def mv(a):
                D[r][0] = oD[0] + vx * a
                D[r][1] = oD[1] + vy * a
                for nid, x0, y0 in oN:
                    P[nid][0] = x0 + vx * a
                    P[nid][1] = y0 + vy * a
                return check_part_Trial(r, oD, oN)

            al, blk = line_search(mv)
            moved += al * math.hypot(vx, vy)

            # Coupled harmonic boundary-layer co-move: when field_stack is enabled
            # and uncoupled coordinate descent is blocked by adjacent free copper,
            # co-advect the local copper corridor along the harmonic decay profile
            # inside the DRC-projected line search.
            if field_stack is not None and al < 0.95 and blk is not None and blk[0] == 'DRC':
                rem_frac = 1.0 - al
                dvx, dvy = vx * rem_frac, vy * rem_frac
                r_pads = [pad_xy(pi) for pi in range(len(pads)) if P_REF[pi] == r]
                cand_nodes = set()
                for nid in part_nodes[r]:
                    for nb in adj[nid]:
                        if nb not in fixed and nb not in part_of:
                            cand_nodes.add(nb)
                for ei in part_edges[r]:
                    for fn in edge_free_nodes(edges[ei]):
                        cand_nodes.add(fn)
                        for nb in adj[fn]:
                            if nb not in fixed and nb not in part_of:
                                cand_nodes.add(nb)
                if cand_nodes and r_pads:
                    oD2 = (D[r][0], D[r][1])
                    oN2 = [(nid, P[nid][0], P[nid][1]) for nid in part_nodes[r]]
                    co_list = []
                    for fn in sorted(cand_nodes):
                        fx0, fy0 = P[fn]
                        d_pad = min(math.hypot(fx0 - px, fy0 - py) for px, py in r_pads)
                        w_fn = math.exp(-max(0.0, d_pad - 0.15) / 1.5)
                        if w_fn > 0.08:
                            co_list.append((fn, fx0, fy0, w_fn))
                    if co_list:
                        coupled_sub = None
                        if gpu is not None:
                            e_set = set(part_edges[r])
                            for fn, _, _, _ in co_list:
                                e_set.update(node_edges[fn])
                            coupled_sub = np.array(sorted(e_set), dtype=np.int32)

                        def mv_coupled(a):
                            D[r][0] = oD2[0] + dvx * a
                            D[r][1] = oD2[1] + dvy * a
                            for nid, x0, y0 in oN2:
                                P[nid][0] = x0 + dvx * a
                                P[nid][1] = y0 + dvy * a
                            for fn, fx0, fy0, w_fn in co_list:
                                P[fn][0] = fx0 + dvx * w_fn * a
                                P[fn][1] = fy0 + dvy * w_fn * a
                            res = check_part_Trial(r, oD2, oN2)
                            if res is not None:
                                return res
                            for fn, _, _, _ in co_list:
                                res = check_node_trial(fn, check_drc=(gpu is None))
                                if res is not None:
                                    return res
                            if gpu is not None and coupled_sub is not None:
                                bad_ei = gpu.check_edges_subset(P, D, outline, coupled_sub)
                                if bad_ei >= 0:
                                    return ('DRC', edges[bad_ei].kind, bad_ei)
                            return None

                        al2, blk2 = line_search(mv_coupled)
                        if al2 > 0.0:
                            moved += al2 * math.hypot(dvx, dvy)
                            al = al + rem_frac * al2
                        if blk2 is not None:
                            blk = blk2

            if blk is not None:
                last_blockers[r] = blk
            if al < 1.0:
                mv(min(1.0, al + 0.25))
                if gpu is not None and part_edges[r]:
                    sub_kab = [(KIND_TO_ID[edges[ei].kind], edges[ei].a, edges[ei].b) for ei in part_edges[r]]
                    d_sub = gpu.eval_candidates(P, D, outline, sub_kab)
                    for idx_e, ei in enumerate(part_edges[r]):
                        g = edges[ei]
                        d = float(d_sub[idx_e])
                        fl = req_f(g)
                        if d < fl + 0.12:
                            w = max(0.0, (fl + 0.12 - d) / 0.12)
                            for fn in edge_free_nodes(g):
                                gx, gy = edge_grad_for_node(g, fn)
                                press[fn][0] += gx * w
                                press[fn][1] += gy * w
                else:
                    for ei in part_edges[r]:
                        g = edges[ei]
                        d = eval_edge(g)
                        fl = req_f(g)
                        if d < fl + 0.12:
                            w = max(0.0, (fl + 0.12 - d) / 0.12)
                            for fn in edge_free_nodes(g):
                                gx, gy = edge_grad_for_node(g, fn)
                                press[fn][0] += gx * w
                                press[fn][1] += gy * w
                mv(al)
        return moved

    def shape_pass(step_s):
        """Advance board outline homotopy `s_edge` toward 1.0 under DRC projection."""
        nonlocal shape_blocker
        if not has_shape or s_edge >= 1.0 - 1e-5:
            return 0.0
        rem_s = 1.0 - s_edge
        ds = min(rem_s, step_s)
        s0_cur = s_edge

        def check_shape_trial():
            if gpu is not None:
                bad_ei = gpu.check_edges_subset(P, D, outline, gpu.shape_edge_subset)
                if bad_ei >= 0:
                    return ('DRC', edges[bad_ei].kind, bad_ei)
                return None
            for ei in shape_edges:
                g = edges[ei]
                if not edge_ok(g):
                    return ('DRC', g.kind, ei)
            return None

        def mv_edge(a):
            set_outline_s(s0_cur + ds * a)
            return check_shape_trial()

        al, blk = line_search(mv_edge)
        moved_s = al * ds

        # Coupled boundary-layer co-move when free copper blocks the advancing board edge
        if al < 0.95 and blk is not None and blk[0] == 'DRC':
            rem_frac = 1.0 - al
            dds = ds * rem_frac
            s1_cur = s_edge
            # Compute net boundary displacement vector
            edge_dx = edge_dy = 0.0
            for p0, p1 in zip(outline0, target_outline):
                if math.hypot(p1[0] - p0[0], p1[1] - p0[1]) > math.hypot(edge_dx, edge_dy):
                    edge_dx, edge_dy = p1[0] - p0[0], p1[1] - p0[1]
            cand_nodes = set()
            if gpu is not None and shape_edges:
                sub_kab = [(KIND_TO_ID[edges[ei].kind], edges[ei].a, edges[ei].b) for ei in shape_edges]
                d_sub = gpu.eval_candidates(P, D, outline, sub_kab)
                for idx_e, ei in enumerate(shape_edges):
                    g = edges[ei]
                    if float(d_sub[idx_e]) < req_f(g) + 1.2:
                        for fn in edge_free_nodes(g):
                            cand_nodes.add(fn)
                            for nb in adj[fn]:
                                if nb not in fixed and nb not in part_of:
                                    cand_nodes.add(nb)
            else:
                for ei in shape_edges:
                    g = edges[ei]
                    if eval_edge(g) < req_f(g) + 1.2:
                        for fn in edge_free_nodes(g):
                            cand_nodes.add(fn)
                            for nb in adj[fn]:
                                if nb not in fixed and nb not in part_of:
                                    cand_nodes.add(nb)
            if cand_nodes and math.hypot(edge_dx, edge_dy) > 1e-4:
                co_list = []
                for fn in sorted(cand_nodes):
                    fx0, fy0 = P[fn]
                    d_edge = max(0.0, inside_dist(fx0, fy0, outline))
                    w_fn = math.exp(-max(0.0, d_edge - 0.25) / 2.2)
                    if w_fn > 0.08:
                        co_list.append((fn, fx0, fy0, w_fn))
                if co_list:
                    shape_coupled_sub = None
                    if gpu is not None:
                        e_set = set(shape_edges)
                        for fn, _, _, _ in co_list:
                            e_set.update(node_edges[fn])
                        shape_coupled_sub = np.array(sorted(e_set), dtype=np.int32)

                    def mv_edge_coupled(a):
                        set_outline_s(s1_cur + dds * a)
                        for fn, fx0, fy0, w_fn in co_list:
                            P[fn][0] = fx0 + edge_dx * dds * w_fn * a
                            P[fn][1] = fy0 + edge_dy * dds * w_fn * a
                        for fn, _, _, _ in co_list:
                            res = check_node_trial(fn, check_drc=(gpu is None))
                            if res is not None:
                                return res
                        if gpu is not None and shape_coupled_sub is not None:
                            bad_ei = gpu.check_edges_subset(P, D, outline, shape_coupled_sub)
                            if bad_ei >= 0:
                                return ('DRC', edges[bad_ei].kind, bad_ei)
                            return None
                        return check_shape_trial()

                    al2, blk2 = line_search(mv_edge_coupled)
                    if al2 > 0.0:
                        moved_s += al2 * dds
                        al = al + rem_frac * al2
                    if blk2 is not None:
                        blk = blk2

        shape_blocker = blk
        if al < 1.0:
            s_cur = s_edge
            set_outline_s(min(1.0, s_cur + 0.15))
            if gpu is not None and shape_edges:
                sub_kab = [(KIND_TO_ID[edges[ei].kind], edges[ei].a, edges[ei].b) for ei in shape_edges]
                d_sub = gpu.eval_candidates(P, D, outline, sub_kab)
                for idx_e, ei in enumerate(shape_edges):
                    g = edges[ei]
                    d = float(d_sub[idx_e])
                    fl = req_f(g)
                    if d < fl + 0.18:
                        w = max(0.0, (fl + 0.18 - d) / 0.18) * 1.5
                        for fn in edge_free_nodes(g):
                            gx, gy = edge_grad_for_node(g, fn)
                            press[fn][0] += gx * w
                            press[fn][1] += gy * w
            else:
                for ei in shape_edges:
                    g = edges[ei]
                    d = eval_edge(g)
                    fl = req_f(g)
                    if d < fl + 0.18:
                        w = max(0.0, (fl + 0.18 - d) / 0.18) * 1.5
                        for fn in edge_free_nodes(g):
                            gx, gy = edge_grad_for_node(g, fn)
                            press[fn][0] += gx * w
                            press[fn][1] += gy * w
            set_outline_s(s_cur)
        return moved_s

    def update_harmonic_field():
        """Update LayerFieldStack with current realized D[r] + lookahead toward TX/TY."""
        if field_stack is None:
            return
        beta = float(fcfg.get('lookahead', 0.35))
        max_la = float(fcfg.get('max_lookahead_mm', 0.6))
        pvecs = {}
        for r in part_refs:
            rx, ry = TX[r] - D[r][0], TY[r] - D[r][1]
            rem = math.hypot(rx, ry)
            if rem > 1e-6 and beta > 0.0:
                la = min(rem * beta, max_la)
                lx, ly = rx * (la / rem), ry * (la / rem)
            else:
                lx, ly = 0.0, 0.0
            pvecs[r] = (D[r][0] + lx, D[r][1] + ly)
        s_la = min(1.0, s_edge + (1.0 - s_edge) * beta) if has_shape else 0.0
        field_stack.update(pvecs, s_edge=s_la)

    def spring_node(nid):
        nb = adj[nid]
        if not nb:
            return (0.0, 0.0)
        du_x = P[nid][0] - P0[nid][0]
        du_y = P[nid][1] - P0[nid][1]
        sx = sum((P[u][0] - P0[u][0]) for u in nb) / len(nb) - du_x
        sy = sum((P[u][1] - P0[u][1]) for u in nb) / len(nb) - du_y
        if field_stack is not None:
            # Anchor gentle spring to the harmonic equilibrium displacement
            hx, hy = field_stack.target_pos(nid)
            ax = (hx - P[nid][0]) * 0.12
            ay = (hy - P[nid][1]) * 0.12
        else:
            ax = -du_x * 0.08
            ay = -du_y * 0.08
        return (sx * 0.45 + ax, sy * 0.45 + ay)

    def stretch_vec(nid):
        vx = vy = 0.0
        for u in adj[nid]:
            dx = P[u][0] - P[nid][0]
            dy = P[u][1] - P[nid][1]
            d = math.hypot(dx, dy)
            if d > SUB * 1.6:
                ex = (d - SUB) / d
                vx += dx * ex * 0.5
                vy += dy * ex * 0.5
        return (vx, vy)

    def copper_pass(step, k_sp=0.35, w_pr=0.45):
        w_field = float(fcfg.get('gain', 0.65)) if field_stack is not None else 0.0
        dz = float(fcfg.get('deadzone_mm', 0.015))
        for nid in range(len(P)):
            if nid in fixed or nid in part_of:
                continue
            sx, sy = spring_node(nid)
            tx, ty = stretch_vec(nid)
            px, py = press.get(nid, (0.0, 0.0))
            pn = math.hypot(px, py)
            if pn > 1e-6:
                px, py = px / pn * min(step, pn * w_pr), py / pn * min(step, pn * w_pr)
            fx = fy = 0.0
            if field_stack is not None:
                ux, uy = field_stack.node_u[nid]
                if math.hypot(ux, uy) > dz:
                    hx, hy = P0[nid][0] + ux, P0[nid][1] + uy
                    fx = (hx - P[nid][0]) * w_field
                    fy = (hy - P[nid][1]) * w_field
            vx = px + tx + fx + sx * k_sp
            vy = py + ty + fy + sy * k_sp
            vm = math.hypot(vx, vy)
            if vm < 1e-5:
                continue
            if vm > step:
                vx, vy = vx / vm * step, vy / vm * step
            x0, y0 = P[nid]

            def mvn(a):
                P[nid][0] = x0 + vx * a
                P[nid][1] = y0 + vy * a
                return check_node_trial(nid)

            line_search(mvn)

    def split_stretched_segs():
        added = 0
        for si in range(len(segs)):
            a, b, w, l, net = segs[si]
            d = math.hypot(P[a][0] - P[b][0], P[a][1] - P[b][1])
            if d > SUB * 2.2:
                mx = 0.5 * (P[a][0] + P[b][0])
                my = 0.5 * (P[a][1] + P[b][1])
                mx0 = 0.5 * (P0[a][0] + P0[b][0])
                my0 = 0.5 * (P0[a][1] + P0[b][1])
                nid = len(P)
                P.append([mx, my])
                P0.append((mx0, my0))
                N_NET.append(net)
                nodes.append({
                    'id': nid, 'x': mx, 'y': my, 'x0': mx0, 'y0': my0,
                    'net': net, 'kind': 'free', 'fixed': False
                })
                if field_stack is not None:
                    field_stack.node_p0.append((mx0, my0))
                    field_stack.node_layers.append({l})
                    wf = field_stack.fields.get(l)
                    if wf is not None:
                        ux, uy = wf.eval_u(mx0, my0)
                        field_stack.node_u.append((float(ux), float(uy)))
                    else:
                        field_stack.node_u.append((0.0, 0.0))
                segs[si][1] = nid
                segs.append([nid, b, w, l, net])
                added += 1
        if added:
            rebuild_adj()
            if gpu is not None:
                gpu.update_graph(P, P0, is_via, segs, adj, AT, AZ)
            build_edges()
        return added

    def reroute_net_astar(net, layer, n_start, n_end, width):
        rect = m['active_rect']
        g_step = 0.25
        gx0, gy0 = rect[0] - 2.0, rect[1] - 2.0
        gx1, gy1 = rect[2] + 2.0, rect[3] + 2.0
        nx = int(math.ceil((gx1 - gx0) / g_step))
        ny = int(math.ceil((gy1 - gy0) / g_step))

        def to_ij(x, y):
            return (max(0, min(nx - 1, int(round((x - gx0) / g_step)))),
                    max(0, min(ny - 1, int(round((y - gy0) / g_step)))))

        def to_xy(i, j):
            return (gx0 + i * g_step, gy0 + j * g_step)

        si, sj = to_ij(P[n_start][0], P[n_start][1])
        gi, gj = to_ij(P[n_end][0], P[n_end][1])

        obs_segs = []
        for idx_s, s in enumerate(segs):
            if s[3] == layer and s[4] != net:
                rl = pair_clearance(rules, net, s[4]) + (width + s[2]) * 0.5
                obs_segs.append((P[s[0]][0], P[s[0]][1], P[s[1]][0], P[s[1]][1], rl))
        for st in st_tracks:
            if st[5] == layer and st[6] != net:
                rl = pair_clearance(rules, net, st[6]) + width * 0.5 + st[4]
                obs_segs.append((st[0], st[1], st[2], st[3], rl))
        obs_pts = []
        for vid, (vr, dr) in is_via.items():
            if N_NET[vid] != net:
                rl = pair_clearance(rules, net, N_NET[vid]) + width * 0.5 + vr
                obs_pts.append((P[vid][0], P[vid][1], rl))
        for sv in st_vias:
            if sv[4] != net:
                rl = pair_clearance(rules, net, sv[4]) + width * 0.5 + sv[2]
                obs_pts.append((sv[0], sv[1], rl))
        obs_polys = []
        for pi in range(len(pads)):
            if P_NET[pi] != net and layer in P_LAYERS[pi]:
                rl = pair_clearance(rules, net, P_NET[pi]) + width * 0.5
                obs_polys.append((pad_poly(pi), rl))
            elif P_DRILL[pi] > 0 and P_NET[pi] != net:
                rl = rules['hole_clearance'] + width * 0.5 + P_DRILL[pi] * 0.5
                px, py = pad_xy(pi)
                obs_pts.append((px, py, rl))

        def cell_ok(i, j):
            if (i, j) in ((si, sj), (gi, gj)):
                return True
            x, y = to_xy(i, j)
            if outline and inside_dist(x, y, outline) < rules['copper_edge'] + width * 0.5:
                return False
            for ox, oy, r in obs_pts:
                if math.hypot(x - ox, y - oy) < r:
                    return False
            for ax, ay, bx, by, r in obs_segs:
                if pt_seg_dist(x, y, ax, ay, bx, by) < r:
                    return False
            for poly, r in obs_polys:
                if poly_pt_dist(poly, x, y) < r:
                    return False
            return True

        DIRS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
        open_q = [(0.0, si, sj)]
        g_cost = {(si, sj): 0.0}
        parent = {}
        while open_q:
            _, ci, cj = heapq.heappop(open_q)
            if (ci, cj) == (gi, gj):
                path = [to_xy(gi, gj)]
                cur = (gi, gj)
                while cur in parent:
                    cur = parent[cur]
                    path.append(to_xy(cur[0], cur[1]))
                path.reverse()
                path[0] = (P[n_start][0], P[n_start][1])
                path[-1] = (P[n_end][0], P[n_end][1])
                return rdp(path, 0.08)
            for di, dj, sc in DIRS:
                ni, nj = ci + di, cj + dj
                if not (0 <= ni < nx and 0 <= nj < ny):
                    continue
                ng = g_cost[(ci, cj)] + sc * g_step
                if ng >= g_cost.get((ni, nj), 1e9):
                    continue
                if not cell_ok(ni, nj):
                    continue
                g_cost[(ni, nj)] = ng
                h = math.hypot(ni - gi, nj - gj) * g_step
                parent[(ni, nj)] = (ci, cj)
                heapq.heappush(open_q, (ng + h, ni, nj))
        return None

    reroutes_done = []

    def try_reroute_blocker(r, blk):
        if not blk or blk[0] != 'DRC':
            return False
        ei = blk[2]
        if ei is None or ei >= len(edges):
            return False
        g = edges[ei]
        if g.kind not in ('SP', 'SST', 'SS', 'SV'):
            return False
        s_idx = g.a
        s = segs[s_idx]
        layer, w, net = s[3], s[2], s[4]
        chain_segs = [si for si in net_segs[net] if segs[si][3] == layer]
        if not chain_segs:
            return False
        deg = defaultdict(int)
        for si in chain_segs:
            deg[segs[si][0]] += 1
            deg[segs[si][1]] += 1
        ends = [n for n, d in deg.items() if d == 1]
        if len(ends) != 2:
            return False
        n_start, n_end = ends[0], ends[1]
        for target_layer in [layer, B_CU if layer == F_CU else F_CU]:
            if target_layer != layer:
                ends_ok = True
                for en in (n_start, n_end):
                    if en in is_via:
                        continue
                    ok_pad = any(P_NET[pi] == net and target_layer in P_LAYERS[pi] and
                                 poly_pt_dist(pad_poly(pi), P[en][0], P[en][1]) < 0.1
                                 for pi in range(len(pads)))
                    if not ok_pad:
                        ends_ok = False
                        break
                if not ends_ok:
                    continue
            pts = reroute_net_astar(net, target_layer, n_start, n_end, w)
            if pts and len(pts) >= 2:
                keep = set(range(len(segs))) - set(chain_segs)
                new_segs_list = [segs[i] for i in sorted(keep)]
                prev = n_start
                for k in range(1, len(pts) - 1):
                    nid = len(P)
                    P.append([pts[k][0], pts[k][1]])
                    P0.append((pts[k][0], pts[k][1]))
                    N_NET.append(net)
                    nodes.append({
                        'id': nid, 'x': pts[k][0], 'y': pts[k][1],
                        'x0': pts[k][0], 'y0': pts[k][1],
                        'net': net, 'kind': 'free', 'fixed': False
                    })
                    if field_stack is not None:
                        field_stack.node_p0.append((pts[k][0], pts[k][1]))
                        field_stack.node_layers.append({target_layer})
                        field_stack.node_u.append((0.0, 0.0))
                    new_segs_list.append([prev, nid, w, target_layer, net])
                    prev = nid
                new_segs_list.append([prev, n_end, w, target_layer, net])
                segs[:] = new_segs_list
                LEN_CAP[net] = max(LEN_CAP.get(net, 0.0), net_len(net) * 1.15 + 2.0)
                rebuild_adj()
                if gpu is not None:
                    gpu.update_graph(P, P0, is_via, segs, adj, AT, AZ)
                build_edges()
                reroutes_done.append({'part': r, 'net': net, 'from_layer': layer, 'to_layer': target_layer})
                return True
        return False

    MAX_CYCLES = int(cfg.get('max_cycles', 60))
    STEP_PART = float(cfg.get('step_part_mm', 0.20))
    STEP_COPPER = float(cfg.get('step_copper_mm', 0.25))
    STEP_SHAPE = float(cfg.get('step_shape', 0.16))
    COPPER_ITERS = int(cfg.get('copper_iters', 5))

    stagnant_count = 0
    reroute_attempts = set()
    w_field_val = float(fcfg.get('gain', 0.65)) if field_stack is not None else 0.0
    dz_val = float(fcfg.get('deadzone_mm', 0.015))

    for cyc in range(MAX_CYCLES):
        max_rem = max((math.hypot(TX[r] - D[r][0], TY[r] - D[r][1]) for r in part_refs), default=0.0)
        rem_shape = (1.0 - s_edge) if has_shape else 0.0
        if max_rem < 1e-3 and rem_shape < 1e-4:
            break
        if field_stack is not None:
            update_harmonic_field()
        mv = part_pass(STEP_PART)
        mv_s = shape_pass(STEP_SHAPE) if has_shape else 0.0
        if field_stack is not None:
            update_harmonic_field()
        if gpu is not None:
            gpu.copper_sweeps(
                P, D, outline, press, field_stack, N_NET, LEN_CAP, net_len,
                COPPER_ITERS, SUB, STEP_COPPER,
                k_sp=0.35, w_pr=0.45, w_field=w_field_val, dz_mm=dz_val
            )
        else:
            for _ in range(COPPER_ITERS):
                copper_pass(STEP_COPPER)
        if cyc % 4 == 3:
            if not split_stretched_segs():
                build_edges()
        if mv < 1e-3 and mv_s < 1e-3:
            stagnant_count += 1
            if stagnant_count >= 2:
                did_rr = False
                for r in part_refs:
                    if math.hypot(TX[r] - D[r][0], TY[r] - D[r][1]) > 0.02:
                        blk = last_blockers.get(r)
                        key = (r, str(blk))
                        if key not in reroute_attempts:
                            reroute_attempts.add(key)
                            if try_reroute_blocker(r, blk):
                                did_rr = True
                                break
                if not did_rr and stagnant_count >= 5:
                    break
        else:
            stagnant_count = 0

    if field_stack is not None:
        update_harmonic_field()
    if gpu is not None:
        gpu.copper_sweeps(
            P, D, outline, press, field_stack, N_NET, LEN_CAP, net_len,
            15, SUB, 0.15, k_sp=0.5, w_pr=0.2, w_field=w_field_val, dz_mm=dz_val
        )
    else:
        for _ in range(15):
            copper_pass(0.15, k_sp=0.5, w_pr=0.2)
    if has_shape and s_edge < 1.0 - 1e-4:
        shape_pass(1.0 - s_edge)
        if gpu is not None:
            gpu.copper_sweeps(
                P, D, outline, press, field_stack, N_NET, LEN_CAP, net_len,
                10, SUB, 0.15, k_sp=0.5, w_pr=0.2, w_field=w_field_val, dz_mm=dz_val
            )
        else:
            for _ in range(10):
                copper_pass(0.15, k_sp=0.5, w_pr=0.2)

    build_edges()
    min_slack = 99.0
    worst_edge = None
    violations = 0
    for g in edges:
        d = g.cur
        fl = req_f(g)
        sl = d - fl
        if sl < min_slack:
            min_slack = sl
            worst_edge = (g.kind, d, g.rule, g.d0)
        if sl < -1e-3:
            violations += 1

    # Measure copper smoothness (mean interior bend angle on free nodes of degree 2)
    bend_angles = []
    for nid in range(len(P)):
        if nid in fixed or nid in part_of or nid in is_via:
            continue
        nbs = list(adj[nid])
        if len(nbs) == 2:
            u, v = nbs[0], nbs[1]
            v1x, v1y = P[nid][0] - P[u][0], P[nid][1] - P[u][1]
            v2x, v2y = P[v][0] - P[nid][0], P[v][1] - P[nid][1]
            l1, l2 = math.hypot(v1x, v1y), math.hypot(v2x, v2y)
            if l1 > 1e-4 and l2 > 1e-4:
                cos_t = max(-1.0, min(1.0, (v1x * v2x + v1y * v2y) / (l1 * l2)))
                bend_angles.append(math.degrees(math.acos(cos_t)))

    part_report = {}
    for r in part_refs:
        tx, ty = TX[r], TY[r]
        dx, dy = D[r]
        t_dist = math.hypot(tx, ty)
        r_dist = math.hypot(tx - dx, ty - dy)
        pct = 100.0 if t_dist < 1e-6 else max(0.0, 100.0 * (1.0 - r_dist / t_dist))
        part_report[r] = {
            'target_dx': round(tx, 4), 'target_dy': round(ty, 4),
            'dx': round(dx, 4), 'dy': round(dy, 4),
            'rem_mm': round(r_dist, 4),
            'pct': round(pct, 1),
            'blocker': list(last_blockers[r]) if r in last_blockers and r_dist > 0.01 else None,
        }

    out_nodes = []
    for i, n in enumerate(nodes):
        nd = dict(n)
        nd['x'] = round(P[i][0], 6)
        nd['y'] = round(P[i][1], 6)
        out_nodes.append(nd)

    out_segs = [{'a': s[0], 'b': s[1], 'w': s[2], 'layer': s[3], 'net': s[4]} for s in segs]
    final_outline = [[round(p[0], 6), round(p[1], 6)] for p in outline]

    settled = {
        'config_name': m['config_name'],
        'board_path': m['board_path'],
        'parts': part_report,
        'nodes': out_nodes,
        'segs': out_segs,
        'outline': m['outline'],
        'target_outline': m.get('target_outline', m['outline']),
        'final_outline': final_outline,
        'shape': m.get('shape', {}),
        'active_track_sigs': m['active_track_sigs'],
        'active_via_sigs': m['active_via_sigs'],
        'reroutes': reroutes_done,
        'summary': {
            'cycles': cyc + 1,
            'elapsed_s': round(time.time() - t_start, 3),
            'edge_count': len(edges),
            'min_slack_mm': round(min_slack, 4),
            'violations': violations,
            'worst_edge': worst_edge,
            'field_enabled': bool(field_stack is not None),
            'gpu_enabled': bool(gpu is not None),
            'field_init_ms': round(field_solve_ms, 1),
            'field_admissibility': field_stack.admissibility_report() if field_stack is not None else None,
            'shape_enabled': has_shape,
            'shape_progress_pct': round(100.0 * s_edge, 1) if has_shape else None,
            'shape_blocker': list(shape_blocker) if (has_shape and shape_blocker and s_edge < 0.999) else None,
            'mean_bend_deg': round(sum(bend_angles) / len(bend_angles), 2) if bend_angles else 0.0,
            'max_bend_deg': round(max(bend_angles), 2) if bend_angles else 0.0,
            'p95_bend_deg': round(sorted(bend_angles)[int(0.95 * (len(bend_angles) - 1))], 2) if bend_angles else 0.0,
        },
    }

    save_json(data_path(cfg, 'settled.json'), settled)
    save_json(data_path(cfg, 'settle_report.json'), {
        'config_name': m['config_name'],
        'summary': settled['summary'],
        'parts': part_report,
        'reroutes': reroutes_done,
    })

    if verbose:
        mode_tag = "HYBRID-FIELD" if field_stack is not None else "CLASSIC"
        if gpu is not None:
            mode_tag += "+GPU"
        print(f"[pcbmorph.settle] ({mode_tag}) {m['config_name']} done in {settled['summary']['elapsed_s']}s "
              f"({cyc+1} cycles, {len(edges)} edges, min_slack={min_slack:.4f}mm, "
              f"mean_bend={settled['summary']['mean_bend_deg']:.2f}deg, "
              f"p95_bend={settled['summary']['p95_bend_deg']:.2f}deg)")
        if has_shape:
            s_blk = f"  blocked_by={settled['summary']['shape_blocker']}" if settled['summary']['shape_blocker'] else ""
            print(f"  SHAPE : progress={settled['summary']['shape_progress_pct']:.1f}%{s_blk}")
        for r, pr in part_report.items():
            blk_s = f"  blocked_by={pr['blocker']}" if pr['blocker'] else ""
            print(f"  {r:6s}: ({pr['dx']:+.3f}, {pr['dy']:+.3f}) / "
                  f"({pr['target_dx']:+.3f}, {pr['target_dy']:+.3f}) mm  [{pr['pct']:5.1f}%]{blk_s}")

    return settled


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    if not argv:
        print("Usage: python3 -m pcbmorph settle CONFIG.json [--field | --no-field] [--gpu | --no-gpu]", file=sys.stderr)
        sys.exit(1)
    cfg_path = None
    use_field = None
    use_gpu = None
    for a in argv:
        if a == '--field':
            use_field = True
        elif a == '--no-field':
            use_field = False
        elif a == '--gpu':
            use_gpu = True
        elif a == '--no-gpu':
            use_gpu = False
        elif not a.startswith('-'):
            cfg_path = a
    if not cfg_path:
        print("Usage: python3 -m pcbmorph settle CONFIG.json [--field | --no-field] [--gpu | --no-gpu]", file=sys.stderr)
        sys.exit(1)
    cfg = load_config(cfg_path)
    run_settle(cfg, use_field=use_field, use_gpu=use_gpu, verbose=True)


if __name__ == '__main__':
    main()
