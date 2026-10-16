#!/usr/bin/env python3
"""enclayout stage 1: simulated annealing over corner layouts, many starts in parallel.

    python3 search.py CONFIG.json            -> data/runs/<name>/results.json (+ per-start layouts)

Config: {"name", "steps", "starts": [{"seed": "current|rows|<arrangement>", "rng": k, "weights": {...}}...],
         "weights": {...overrides of model.DEFAULT_CFG}, "dy_max": 1.2, "procs": N}
Incremental cost: moving a set of parts recomputes only their courtyard rows, obstacle terms, the nets they
carry, the paths of their channel and the companion term.
"""
import json, math, os, sys, time, random
import numpy as np
from multiprocessing import Pool
from model import Corner, DEFAULT_CFG, rot_xy
import seeds as SEEDS
HERE = os.path.dirname(os.path.abspath(__file__))
PAIR_RE = __import__('re').compile(r'/ENC_[ABCD]_(P|N)(_SW)?$|/ENC_[ABCD]_TRX_[AB]$')


class State:
    def __init__(self, C, cfg, pose, dy):
        self.C, self.cfg = C, cfg
        self.pose = pose.copy(); self.dy = dy; self._dy_cache = None
        n = C.n
        self.crts = C.crt_world(pose)
        self.O = np.zeros((n, n))
        for i in range(n): self.O[i] = self.row(i)
        self.obst = np.array([self.obst_of(i) for i in range(n)])
        self.nets = {net: C.net_mst(pose, net) for net in C.net_pads if net not in C.planes}
        self.paths = {k: C.path_len(pose, k) for k in C.paths}
        self.comp = C.companion_cost(pose)
        self.slide = C.slide_pen(dy, cfg['clearance'])
        self.part_paths = {}
        for k, seq in C.paths.items():
            for node in seq:
                if node[0] != 'J12': self.part_paths.setdefault(node[0], set()).add(k)
        self.cost = self.total()

    def row(self, i, crts=None):
        crts = self.crts if crts is None else crts
        a = crts[i]; g = self.cfg['guard']
        w = np.minimum(a[2], crts[:, 2]) - np.maximum(a[0], crts[:, 0]) + g
        h = np.minimum(a[3], crts[:, 3]) - np.maximum(a[1], crts[:, 1]) + g
        r = np.clip(w, 0, None) * np.clip(h, 0, None); r[i] = 0
        return r

    def obst_of(self, i, pose=None, crts=None, dy=None):
        C = self.C; cfg = self.cfg; pose = self.pose if pose is None else pose; crts = self.crts if crts is None else crts
        dy = self.dy if dy is None else dy
        a = crts[i]; g = cfg['guard']; s = 0.0
        if len(C.frects): s += C.rect_overlap(a, C.frects, g)
        if dy != self._dy_cache:
            self._holes = C.holes.copy(); self._holes[C.hole_slider, 1] -= dy
            self._circ = C.circ.copy(); self._circ[C.circ_slider, 1] -= dy; self._dy_cache = dy
        holes, circ = self._holes, self._circ
        s += C.rect_circle_pen(a, holes, 0.05)
        prs = C.pad_rects(pose, i)
        for pr in prs:
            s += C.rect_circle_pen(pr, circ, cfg['clearance'])
            if len(C.tpts) and cfg['w_tracks'] > 0:
                s += cfg['w_tracks'] * C.rect_circle_pen(pr, C.tpts, cfg['clearance'])   # fixed tracks are walls for pads
        if len(C.keep): s += C.rect_circle_pen(a, C.keep, 0.0) * 2
        R = C.region; m = cfg['edge_margin']
        s += max(0, R[0] + m - a[0]) ** 2 + max(0, a[2] - R[2]) ** 2 + max(0, R[1] - a[1]) ** 2 + max(0, a[3] - (R[3] - m)) ** 2
        cr = C.outline['corner_r']; cx, cy = R[0] + cr, R[3] - cr
        if a[0] < cx and a[3] > cy:
            dd = math.hypot(a[0] - cx, a[3] - cy)
            if dd > cr - m: s += (dd - (cr - m)) ** 2
        return s

    def terms(self, nets=None, paths=None, obst=None, O=None, comp=None, dy=None, slide=None):
        C = self.C; nets = self.nets if nets is None else nets; paths = self.paths if paths is None else paths
        obst = self.obst if obst is None else obst; O = self.O if O is None else O; comp = self.comp if comp is None else comp
        dy = self.dy if dy is None else dy; slide = self.slide if slide is None else slide
        chan = {ch: (paths[(ch, 'P')] + paths[(ch, 'N')]) / 2 for ch in C.chain}
        t = dict(overlap=float(O.sum() / 2 + obst.sum()), slide=float(slide), spread=max(chan.values()) - min(chan.values()), maxlen=max(chan.values()),
                 mismatch=sum(abs(paths[(ch, 'P')] - paths[(ch, 'N')]) for ch in C.chain), length=sum(chan.values()),
                 copper=sum(v for n, v in nets.items() if PAIR_RE.match(n)), other=sum(v for n, v in nets.items() if not PAIR_RE.match(n)),
                 companion=comp, dy=dy)
        t['chan'] = chan
        return t

    def total(self, **kw):
        t = self.terms(**kw); w = self.cfg
        return (w['w_overlap'] * t['overlap'] + w['w_slide'] * t['slide'] + w['w_spread'] * t['spread'] + w['w_max'] * t['maxlen'] + w['w_mismatch'] * t['mismatch'] + w['w_length'] * t['length']
                + w['w_copper'] * t['copper'] + w['w_other'] * t['other'] + w['w_companion'] * t['companion'] + w['w_dy'] * t['dy'])

    def try_move(self, moved, newpose, newdy):
        """cost of the state with parts `moved` at newpose rows (and dy); returns (cost, updates)."""
        C = self.C; S = list(moved)
        crts = self.crts.copy()
        for i in S: crts[i] = C.crt_world(newpose, i)
        O = self.O.copy()
        for i in S:
            r = self.row(i, crts); O[i] = r; O[:, i] = r
        obst = self.obst.copy()
        for i in (range(C.n) if newdy != self.dy else S): obst[i] = self.obst_of(i, newpose, crts, newdy)
        if newdy != self.dy: self._dy_cache = None   # caches now hold newdy; rebuilt lazily on the next call
        nets = self.nets
        aff = {net for i in S for net in C.part_nets[i] if net not in C.planes}
        if aff:
            nets = dict(self.nets)
            for net in aff: nets[net] = C.net_mst(newpose, net)
        paths = self.paths
        pk = {k for i in S for k in self.part_paths.get(i, ())}
        if pk:
            paths = dict(self.paths)
            for k in pk: paths[k] = C.path_len(newpose, k)
        comp = C.companion_cost(newpose) if any(i in C.comp_parts for i in S) else self.comp
        slide = C.slide_pen(newdy, self.cfg['clearance']) if newdy != self.dy else self.slide
        cost = self.total(nets=nets, paths=paths, obst=obst, O=O, comp=comp, dy=newdy, slide=slide)
        return cost, (crts, O, obst, nets, paths, comp, slide)

    def commit(self, newpose, newdy, cost, upd):
        self.pose = newpose; self.dy = newdy; self.cost = cost
        self.crts, self.O, self.obst, self.nets, self.paths, self.comp, self.slide = upd


def anneal(C, cfg, pose, dy, steps, rng, T0=4.0, T1=0.02, s0=2.5, s1=0.12, dy_max=1.2, log=None):
    st = State(C, cfg, pose, dy)
    same_fp = {}
    for i, r in enumerate(C.refs): same_fp.setdefault(C.d['parts'][r]['fp'], []).append(i)
    groups = {}
    for r, ch in C.channel.items(): groups.setdefault(ch, []).append(C.idx[r])
    R = C.region; best = (st.cost, st.pose.copy(), st.dy)
    n = C.n; acc = 0
    def pick():
        if f < 0.5 or rng.random() < 0.5: return rng.randrange(n)
        w = st.O.sum(axis=1) / 2 + st.obst
        if w.sum() < 1e-9: return rng.randrange(n)
        return int(rng.choices(range(n), weights=(0.02 + w))[0])
    for step in range(steps):
        f = step / max(1, steps - 1); T = T0 * (T1 / T0) ** f; sig = s0 * (s1 / s0) ** f
        u = rng.random(); pose = st.pose.copy(); newdy = st.dy; moved = []
        if u < 0.42:
            i = pick(); pose[i, :2] += rng.gauss(0, sig), rng.gauss(0, sig); moved = [i]
        elif u < 0.52:
            i = pick(); pose[i, 2] = (pose[i, 2] + rng.choice((90, 180, 270))) % 360; moved = [i]
        elif u < 0.60:
            i = pick(); pose[i] = (rng.uniform(R[0], R[2]), rng.uniform(R[1], R[3]), rng.choice((0, 90, 180, 270))); moved = [i]
        elif u < 0.70:
            fam = rng.choice([v for v in same_fp.values() if len(v) > 1]); i, j = rng.sample(fam, 2)
            pose[i], pose[j] = st.pose[j].copy(), st.pose[i].copy(); moved = [i, j]
        elif u < 0.82:   # attach: bring one pad of part i next to a partner pad on the same net
            i = rng.randrange(n); nets = [net for net in C.part_nets[i] if net not in C.planes] or C.part_nets[i]
            if not nets: continue
            net = rng.choice(nets); k = next(k for k, p in enumerate(C.pads[i]) if p['net'] == net)
            partners = [C.pad_xy(st.pose, j, kk) for j, kk in C.net_pads[net] if j != i] + list(C.net_fixed.get(net, []))
            if not partners: continue
            tx, ty = rng.choice(partners); ang = rng.uniform(0, 2 * math.pi); d = rng.uniform(0.7, 1.6)
            rot = rng.choice((0, 90, 180, 270)) if rng.random() < 0.5 else pose[i, 2]
            lx, ly = rot_xy(C.pads[i][k]['x'], C.pads[i][k]['y'], int(rot))
            pose[i] = (tx + d * math.cos(ang) - lx, ty + d * math.sin(ang) - ly, rot); moved = [i]
        elif u < 0.94:   # group move of one channel
            ch = rng.choice(list(groups)); S = groups[ch]; moved = S
            if rng.random() < 0.7:
                dx, dy_ = rng.gauss(0, sig), rng.gauss(0, sig)
                for i in S: pose[i, 0] += dx; pose[i, 1] += dy_
            else:
                cx = np.mean(st.pose[S, 0]); cy = np.mean(st.pose[S, 1]); q = rng.choice((90, 180, 270))
                for i in S:
                    x, y = rot_xy(st.pose[i, 0] - cx, st.pose[i, 1] - cy, q)
                    pose[i] = (cx + x, cy + y, (st.pose[i, 2] + q) % 360)
        else:
            newdy = min(dy_max, max(0.0, st.dy + rng.gauss(0, 0.3)))
            if newdy == st.dy: continue
        cost, upd = st.try_move(moved, pose, newdy)
        if cost <= st.cost or rng.random() < math.exp(-(cost - st.cost) / T):
            st.commit(pose, newdy, cost, upd); acc += 1
            if cost < best[0]: best = (cost, pose.copy(), newdy)
        if log and step % log == 0:
            t = st.terms(); print(f'{step:7d} T={T:.3f} cost={st.cost:8.2f} best={best[0]:8.2f} ov={t["overlap"]:.3f} spread={t["spread"]:.2f} len={t["length"]:.1f} acc={acc}', flush=True)
    st2 = State(C, cfg, best[1], best[2])
    return st2


def run_start(args):
    name, spec, cfg_over, steps, dy_max, outdir = args
    C = Corner(); cfg = dict(DEFAULT_CFG); cfg.update(cfg_over); cfg.update(spec.get('weights', {}))
    rng = random.Random(spec.get('rng', 0))
    pose, dy = SEEDS.make(C, spec['seed'], rng)
    dy = spec.get('dy', dy)
    t0 = time.time()
    st = anneal(C, cfg, pose, dy, steps, rng, dy_max=dy_max, T0=spec.get('T0', 4.0), s0=spec.get('s0', 2.5))
    t = st.terms(); t['cost'] = st.cost; t['seconds'] = round(time.time() - t0, 1)
    rec = dict(name=name, seed=spec['seed'], rng=spec.get('rng', 0), weights=spec.get('weights', {}), terms={k: (v if not isinstance(v, dict) else v) for k, v in t.items()},
               dy=st.dy, pose={r: [round(float(v), 4) for v in st.pose[i]] for i, r in enumerate(C.refs)},
               paths={f'{k[0]}{k[1]}': round(v, 2) for k, v in st.paths.items()},
               part_overlap={r: round(float(st.O[i].sum() / 2 + st.obst[i]), 4) for i, r in enumerate(C.refs) if st.O[i].sum() / 2 + st.obst[i] > 1e-3})
    json.dump(rec, open(os.path.join(outdir, f'{name}.json'), 'w'))
    return rec


if __name__ == '__main__':
    cfgp = sys.argv[1]; cfg = json.load(open(cfgp))
    outdir = os.path.join(HERE, 'data', 'runs', cfg['name']); os.makedirs(outdir, exist_ok=True)
    jobs = [(f'{s["seed"].replace("from:", "").replace("/", "-")}_{s.get("rng", 0)}_{k}', s, cfg.get('weights', {}), cfg.get('steps', 100000), cfg.get('dy_max', 1.2), outdir)
            for k, s in enumerate(cfg['starts'])]
    t0 = time.time()
    with Pool(cfg.get('procs', 20)) as pool:
        recs = pool.map(run_start, jobs)
    recs.sort(key=lambda r: r['terms']['cost'])
    json.dump(recs, open(os.path.join(outdir, 'results.json'), 'w'))
    print(f'{len(recs)} starts in {time.time() - t0:.0f} s')
    for r in recs:
        t = r['terms']; ch = t['chan']
        print(f'{r["name"]:22} cost {t["cost"]:8.2f} ov {t["overlap"]:6.3f} spread {t["spread"]:5.2f} max {t["maxlen"]:5.1f} mism {t["mismatch"]:5.2f} '
              f'chan A {ch["A"]:5.1f} B {ch["B"]:5.1f} C {ch["C"]:5.1f} D {ch["D"]:5.1f} dy {r["dy"]:.2f} bad {len(r["part_overlap"])}')
