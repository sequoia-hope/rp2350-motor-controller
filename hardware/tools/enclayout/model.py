#!/usr/bin/env python3
"""enclayout: the abstract corner model — poses, geometry, and the cost of a layout.

A layout is (pose per movable part: x, y, rot in {0,90,180,270}; dy of the phase-connector stack).
Cost terms (weights in the config):
  overlap   courtyard-vs-courtyard, courtyard/pad vs fixed copper circles, courtyard vs keepout, region/edge
  spread    max-min of the per-channel pair length (mean of P and N path), the thing routing cannot fix
  mismatch  |P-N| per channel
  length    sum of pair paths
  copper    MST length of every non-plane net on movable pads (stubs, hall parts, control lines; anchored nets
            pull toward where they leave the corner)
  companion decoupling caps stay by their chip's power pad
  tracks    movable pads over fixed B.Cu tracks
"""
import json, math, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
ROT = {0: (1, 0), 90: (0, 1), 180: (-1, 0), 270: (0, -1)}   # cos, sin


def rot_xy(x, y, deg):
    c, s = ROT[deg % 360]
    return (x * c + y * s, -x * s + y * c)


class Corner:
    def __init__(self, path=None):
        d = json.load(open(path or os.path.join(HERE, 'data', 'corner.json')))
        self.d = d
        self.refs = list(d['parts'].keys()); self.idx = {r: i for i, r in enumerate(self.refs)}
        self.n = len(self.refs)
        P = d['parts']
        self.crt = np.array([P[r]['crt'] for r in self.refs])            # local l,t,r,b
        self.pads = [P[r]['pads'] for r in self.refs]
        self.region = np.array(d['region']); self.outline = d['outline']
        # fixed geometry
        self.frects = np.array([r['box'] for r in d['rects'] if r['ref'] != 'keepout']).reshape(-1, 4)
        self.frect_ref = [r['ref'] for r in d['rects'] if r['ref'] != 'keepout']
        self.keep = np.array([r['circle'] for r in d['rects'] if r['ref'] == 'keepout']).reshape(-1, 3)
        circ = d['circles']; holes = d['holes']
        self.circ = np.array([[c['x'], c['y'], c['r']] for c in circ]).reshape(-1, 3)
        self.circ_slider = np.array([c['slider'] for c in circ], bool)
        self.circ_ref = [c['ref'] for c in circ]
        self.holes = np.array([[c['x'], c['y'], c['r']] for c in holes]).reshape(-1, 3)
        self.hole_slider = np.array([c['slider'] for c in holes], bool)
        self.tpts = np.array(d['track_pts']).reshape(-1, 3)
        self.spts = np.array([p[:3] for p in d.get('slider_pts', [])], float).reshape(-1, 3)
        self.spts_net = [p[3] for p in d.get('slider_pts', [])]
        self.slider_pads = self.circ[self.circ_slider]
        self._slide0 = None
        self.j12 = d['j12']; self.chain = d['chain']; self.anchors = d['anchors']; self.planes = set(d['planes'])
        # nets on movable pads
        self.net_pads = {}
        for i, r in enumerate(self.refs):
            for k, p in enumerate(self.pads[i]):
                if p['net'] and not p['net'].startswith('unconnected'):
                    self.net_pads.setdefault(p['net'], []).append((i, k))
        self.part_nets = [sorted({p['net'] for p in self.pads[i] if p['net'] in self.net_pads}) for i in range(self.n)]
        # fixed nodes per net: J12 pins + anchor
        self.net_fixed = {}
        for num, pin in self.j12.items():
            if pin['net'] in self.net_pads: self.net_fixed.setdefault(pin['net'], []).append((pin['x'], pin['y']))
        for net, a in self.anchors.items():
            if net in self.net_pads and net not in self.net_fixed: self.net_fixed[net] = [tuple(a)]
        # chain paths: list of pad refs (ref, padnum) per channel, P and N; J12 pin first
        self.paths = {}
        for ch, c in self.chain.items():
            for side, pin in zip('PN', c['pins']):
                seq = [('J12', pin)]
                order = [c['tvs']] + ([c['sw'][0 if side == 'P' else 1]] if c['sw'] else []) + [c['fl'], c['trx']]
                nets = [f'/ENC_{ch}_{side}'] + ([f'/ENC_{ch}_{side}_SW'] if c['sw'] else []) + [f'/ENC_{ch}_TRX_{"A" if side == "P" else "B"}']
                # walk: on each net, the pad of the next part in order that carries it
                cur_part = None
                for net in nets:
                    on = [(i, k) for i, k in self.net_pads[net]]
                    for part in order:
                        hit = [(i, k) for i, k in on if self.refs[i] == part and part != cur_part]
                        if hit: seq.append(hit[0]); cur_part = part
                    # the next net starts from the same part's other pad
                    if cur_part:
                        nxt = nets[nets.index(net) + 1] if nets.index(net) + 1 < len(nets) else None
                        if nxt:
                            hit = [(i, k) for i, k in self.net_pads[nxt] if self.refs[i] == cur_part]
                            if hit: seq.append(hit[0])
                self.paths[(ch, side)] = seq
        # channel membership (for group moves / rendering)
        self.channel = {}
        for ch, c in self.chain.items():
            for r in [c['tvs'], c['fl'], c['trx'], *c['term'], *(c['sw'] or [])]:
                self.channel[r] = ch
        for i, r in enumerate(self.refs):
            if r in self.channel: continue
            for net in self.part_nets[i]:
                m = re.match(r'/(ENC|TERM)_([ABCD])', net)
                if m: self.channel[r] = m.group(2); break
        # companions: decoupling caps and chip power pads on the same rail; every cap wants a chip within
        # 1.5 mm and every chip wants a cap within 1.5 mm (no fixed assignment: caps are interchangeable)
        self.comp_caps, self.comp_chips = {}, {}
        for i, r in enumerate(self.refs):
            for k, p in enumerate(self.pads[i]):
                if p['net'] in ('Vdrive', '+3V3'):
                    if r.startswith('C'): self.comp_caps.setdefault(p['net'], []).append((i, k))
                    elif r.startswith('U'): self.comp_chips.setdefault(p['net'], []).append((i, k))
        self.comp_parts = {i for d_ in (self.comp_caps, self.comp_chips) for v in d_.values() for i, k in v}
        self.slider_dy_max = 1.2

    # --- geometry --------------------------------------------------------------------
    def pose0(self):
        pose = np.zeros((self.n, 3))
        for i, r in enumerate(self.refs): pose[i] = self.d['parts'][r]['pose']
        return pose

    def crt_world(self, pose, i=None):
        idx = range(self.n) if i is None else [i]
        out = np.zeros((len(idx), 4))
        for o, j in enumerate(idx):
            l, t, r, b_ = self.crt[j]; deg = int(pose[j, 2]) % 360
            xs, ys = zip(*[rot_xy(x, y, deg) for x, y in ((l, t), (r, t), (r, b_), (l, b_))])
            out[o] = (pose[j, 0] + min(xs), pose[j, 1] + min(ys), pose[j, 0] + max(xs), pose[j, 1] + max(ys))
        return out if i is None else out[0]

    def pad_xy(self, pose, i, k):
        p = self.pads[i][k]; x, y = rot_xy(p['x'], p['y'], int(pose[i, 2]) % 360)
        return (pose[i, 0] + x, pose[i, 1] + y)

    def pad_rects(self, pose, i):
        deg = int(pose[i, 2]) % 360; out = []
        for p in self.pads[i]:
            x, y = rot_xy(p['x'], p['y'], deg); w, h = (p['w'], p['h']) if deg % 180 == 0 else (p['h'], p['w'])
            out.append((pose[i, 0] + x - w / 2, pose[i, 1] + y - h / 2, pose[i, 0] + x + w / 2, pose[i, 1] + y + h / 2))
        return np.array(out)

    # --- cost pieces ---------------------------------------------------------------
    @staticmethod
    def rect_overlap(a, B, clr):
        """overlap area of rect a (4,) against rects B (m,4), each grown by clr."""
        w = np.minimum(a[2], B[:, 2]) - np.maximum(a[0], B[:, 0]) + clr
        h = np.minimum(a[3], B[:, 3]) - np.maximum(a[1], B[:, 1]) + clr
        return np.sum(np.clip(w, 0, None) * np.clip(h, 0, None))

    @staticmethod
    def rect_circle_pen(a, C, clr):
        """penetration depth of circles C (m,3) into rect a grown by clr -> sum of squares."""
        dx = np.maximum(np.maximum(a[0] - C[:, 0], C[:, 0] - a[2]), 0)
        dy = np.maximum(np.maximum(a[1] - C[:, 1], C[:, 1] - a[3]), 0)
        pen = np.clip(C[:, 2] + clr - np.hypot(dx, dy), 0, None)
        return np.sum(pen * (pen + 0.2))          # quadratic plus a linear floor: small penetrations still cost

    def part_overlap(self, pose, i, crts, dy, cfg):
        """all overlap terms of part i against everything else (crts = world courtyards of all parts)."""
        a = crts[i]; g = cfg['guard']
        others = np.delete(crts, i, axis=0)
        s = self.rect_overlap(a, others, g)
        if len(self.frects): s += self.rect_overlap(a, self.frects, g)
        holes = self.holes.copy(); holes[self.hole_slider, 1] -= dy
        s += self.rect_circle_pen(a, holes, 0.05)          # no body over a drill
        circ = self.circ.copy(); circ[self.circ_slider, 1] -= dy
        for pr in self.pad_rects(pose, i):                  # pads clear of annuli / THT pads
            s += self.rect_circle_pen(pr, circ, cfg['clearance'])
        if len(self.keep): s += self.rect_circle_pen(a, self.keep, 0.0) * 2
        # region / edge: courtyard outside the region box
        R = self.region; m = cfg['edge_margin']
        s += max(0, R[0] + m - a[0]) ** 2 + max(0, a[2] - R[2]) ** 2 + max(0, R[1] - a[1]) ** 2 + max(0, a[3] - (R[3] - m)) ** 2
        # rounded corner of the board (bottom-left)
        cr = self.outline['corner_r']; cx, cy = R[0] + cr, R[3] - cr
        if a[0] < cx and a[3] > cy:
            dd = math.hypot(a[0] - cx, a[3] - cy)
            if dd > cr - m: s += (dd - (cr - m)) ** 2
        # pads over fixed tracks (soft)
        if len(self.tpts) and cfg['w_tracks'] > 0:
            for pr in self.pad_rects(pose, i):
                s += cfg['w_tracks'] * self.rect_circle_pen(pr, self.tpts, cfg['clearance'])
        return s

    def path_len(self, pose, key):
        seq = self.paths[key]; pts = []
        for node in seq:
            if node[0] == 'J12': pin = self.j12[node[1]]; pts.append((pin['x'], pin['y']))
            else: pts.append(self.pad_xy(pose, *node))
        return sum(math.dist(pts[k], pts[k + 1]) for k in range(len(pts) - 1))

    def net_mst(self, pose, net):
        pts = [self.pad_xy(pose, i, k) for i, k in self.net_pads[net]] + list(self.net_fixed.get(net, []))
        n = len(pts)
        if n < 2: return 0.0
        inT = [False] * n; dmin = [math.inf] * n; dmin[0] = 0; tot = 0.0
        for _ in range(n):
            u = min((d, k) for k, d in enumerate(dmin) if not inT[k])[1]
            inT[u] = True; tot += dmin[u]
            for k in range(n):
                if not inT[k]:
                    dd = math.dist(pts[u], pts[k])
                    if dd < dmin[k]: dmin[k] = dd
        return tot

    def slide_pen(self, dy, clr):
        """connector pins (moved up by dy) against fixed copper of other nets on any layer — the NEW conflicts only
        (relative to dy = 0), i.e. the re-routing the move would cost."""
        if not len(self.spts) or not len(self.slider_pads): return 0.0
        def raw(dy):
            s = 0.0
            for px, py, pr in self.slider_pads:
                d = np.hypot(self.spts[:, 0] - px, self.spts[:, 1] - (py - dy))
                pen = np.clip(pr + self.spts[:, 2] + clr - d, 0, None); s += np.sum(pen * (pen + 0.2))
            return s
        if self._slide0 is None: self._slide0 = raw(0.0)
        return max(0.0, raw(dy) - self._slide0)

    def slide_hits(self, dy, clr):
        """fixed copper items (by net) a connector pin would touch at this dy but not at dy = 0."""
        hits = {}
        for px, py, pr in self.slider_pads:
            d1 = np.hypot(self.spts[:, 0] - px, self.spts[:, 1] - (py - dy)); d0 = np.hypot(self.spts[:, 0] - px, self.spts[:, 1] - py)
            lim = pr + self.spts[:, 2] + clr
            for k in np.nonzero((d1 < lim) & (d0 >= lim))[0]: hits[self.spts_net[k]] = hits.get(self.spts_net[k], 0) + 1
        return hits

    def companion_cost(self, pose):
        s = 0.0
        for net in self.comp_caps:
            caps = [self.pad_xy(pose, i, k) for i, k in self.comp_caps[net]]
            chips = [self.pad_xy(pose, i, k) for i, k in self.comp_chips.get(net, [])]
            if not chips: continue
            for c in caps: s += max(0, min(math.dist(c, u) for u in chips) - 1.5) ** 2
            for u in chips: s += max(0, min(math.dist(c, u) for c in caps) - 1.5) ** 2
        return s

    # --- full evaluation -----------------------------------------------------------
    def evaluate(self, pose, dy, cfg):
        crts = self.crt_world(pose)
        ov = sum(self.part_overlap(pose, i, crts, dy, cfg) for i in range(self.n))
        slide = self.slide_pen(dy, cfg['clearance'])
        # (pairwise courtyard terms are counted from both sides; the search uses the same convention)
        paths = {k: self.path_len(pose, k) for k in self.paths}
        chan = {ch: (paths[(ch, 'P')] + paths[(ch, 'N')]) / 2 for ch in self.chain}
        mism = sum(abs(paths[(ch, 'P')] - paths[(ch, 'N')]) for ch in self.chain)
        spread = max(chan.values()) - min(chan.values())
        nets = {net: self.net_mst(pose, net) for net in self.net_pads if net not in self.planes}
        pair_nets = {n for n in nets if re.match(r'/ENC_[ABCD]_(P|N)(_SW)?$|/ENC_[ABCD]_TRX_[AB]$', n)}
        copper_pairs = sum(v for n, v in nets.items() if n in pair_nets)
        copper_other = sum(v for n, v in nets.items() if n not in pair_nets)
        comp = self.companion_cost(pose)
        terms = dict(overlap=ov, slide=slide, spread=spread, maxlen=max(chan.values()), mismatch=mism, length=sum(chan.values()), copper=copper_pairs,
                     other=copper_other, companion=comp, dy=dy)
        cost = (cfg['w_overlap'] * ov + cfg['w_slide'] * slide + cfg['w_spread'] * spread + cfg['w_max'] * terms['maxlen'] + cfg['w_mismatch'] * mism + cfg['w_length'] * terms['length']
                + cfg['w_copper'] * copper_pairs + cfg['w_other'] * copper_other + cfg['w_companion'] * comp + cfg['w_dy'] * dy)
        return cost, terms, paths, chan


import re
DEFAULT_CFG = dict(guard=0.1, clearance=0.2, edge_margin=0.45, w_overlap=150.0, w_spread=1.0, w_max=2.0, w_mismatch=1.0,
                   w_length=0.5, w_copper=0.5, w_other=0.15, w_companion=0.6, w_tracks=1.0, w_dy=0.3, w_slide=5.0)

if __name__ == '__main__':
    import sys
    C = Corner(); pose = C.pose0()
    cost, terms, paths, chan = C.evaluate(pose, 0.0, DEFAULT_CFG)
    print('parts', C.n, 'nets', len(C.net_pads), 'companion caps', {k: len(v) for k, v in C.comp_caps.items()}, 'chips', {k: len(v) for k, v in C.comp_chips.items()})
    print('channels:', {r: c for r, c in C.channel.items()})
    for k, seq in C.paths.items():
        print(k, ' -> '.join(f'{n[0]}.{n[1]}' if n[0] == 'J12' else f'{C.refs[n[0]]}.{C.pads[n[0]][n[1]]["n"]}' for n in seq), f'{paths[k]:.1f}')
    print('cost', round(cost, 2), {k: round(v, 2) for k, v in terms.items()})
    print('per part overlap:', {C.refs[i]: round(C.part_overlap(pose, i, C.crt_world(pose), 0.0, DEFAULT_CFG), 3) for i in range(C.n) if C.part_overlap(pose, i, C.crt_world(pose), 0.0, DEFAULT_CFG) > 1e-4})
