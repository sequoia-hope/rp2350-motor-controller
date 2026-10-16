"""enclayout: starting arrangements. A seed names, per channel, where its chain starts and which way it runs;
parts are dropped along that line with sensible rotations and the annealer does the rest."""
import math, random
import numpy as np
from model import rot_xy

DIRS = dict(L=(-1, 0), R=(1, 0), U=(0, -1), D=(0, 1), UL=(-0.7071, -0.7071), DL=(-0.7071, 0.7071), UR=(0.7071, -0.7071))
# arrangement -> {channel: (start x, start y, dir)} ; start = where the chain enters (just off the J12 pins)
ARR = {
    'rows':      dict(A=(121.0, 116.0, 'L'), B=(121.0, 120.0, 'L'), C=(121.0, 124.5, 'L'), D=(123.0, 113.5, 'UL')),
    'rows_dpk':  dict(A=(121.0, 116.0, 'L'), B=(121.0, 120.0, 'L'), C=(121.0, 124.5, 'L'), D=(125.5, 122.5, 'R')),
    'hat_cd':    dict(A=(121.0, 116.5, 'L'), B=(121.0, 121.0, 'L'), C=(121.0, 125.5, 'L'), D=(122.0, 113.0, 'L')),
    'a_up':      dict(A=(121.5, 113.5, 'UL'), B=(121.0, 119.0, 'L'), C=(121.0, 124.0, 'L'), D=(125.5, 113.0, 'U')),
    'fan':       dict(A=(121.0, 115.0, 'UL'), B=(121.0, 120.0, 'L'), C=(121.0, 125.0, 'DL'), D=(123.0, 113.0, 'U')),
    'column':    dict(A=(120.5, 113.0, 'U'), B=(117.5, 113.0, 'U'), C=(114.5, 113.0, 'U'), D=(123.5, 113.0, 'U')),
}


def rot_for(local, world):
    best = min((0, 90, 180, 270), key=lambda d: -np.dot(rot_xy(*local, d), world))
    return best


def make(C, seed, rng):
    pose = C.pose0(); dy = 0.0
    if seed == 'current': return pose, dy
    if seed.startswith('from:'):          # polish an earlier result: from:<run>/<name>
        import json, os
        run, name = seed[5:].split('/')
        rec = next(r for r in json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'runs', run, 'results.json'))) if r['name'] == name)
        for i, r in enumerate(C.refs): pose[i] = rec['pose'][r]
        return pose, rec['dy']
    if seed.startswith('random'):
        arr = {ch: (rng.uniform(114, 126), rng.uniform(98, 126), rng.choice(list(DIRS))) for ch in 'ABCD'}
    else:
        arr = ARR[seed]
    for ch, (sx, sy, dname) in arr.items():
        d = np.array(DIRS[dname]); p = np.array([-d[1], d[0]]); c = C.chain[ch]
        def put(ref, along, across, rot):
            i = C.idx[ref]; pose[i] = (sx + along * d[0] + across * p[0], sy + along * d[1] + across * p[1], rot)
        put(c['tvs'], 1.2, 0, rot_for((1, 0), d))
        if c['sw']:
            put(c['sw'][0], 3.6, 1.5, rot_for((1, 0), -d)); put(c['sw'][1], 3.6, -1.5, rot_for((1, 0), -d))
            put(c['fl'], 6.2, 0, rot_for((0, 1), -d)); put(c['trx'], 9.0, 0, rot_for((0, 1), -d)); tx = 9.0
        else:
            put(c['fl'], 3.4, 0, rot_for((0, 1), -d)); put(c['trx'], 6.2, 0, rot_for((0, 1), -d)); tx = 6.2
        put(c['term'][0], tx, 3.2, rot_for((1, 0), p)); put(c['term'][1], tx - 2.2, 2.6, 0)
        # satellites of this channel: pull-ups/downs and caps, spread around the chain
        sats = [r for r, cc in C.channel.items() if cc == ch and r not in (c['tvs'], c['fl'], c['trx'], *c['term'], *(c['sw'] or []))]
        for k, r in enumerate(sats):
            put(r, 2.0 + 2.0 * (k // 2), (-3.0 if k % 2 else 3.0), rng.choice((0, 90)))
    # caps without a channel: next to a chip on their rail (round robin)
    for net, caps in C.comp_caps.items():
        chips = C.comp_chips.get(net, [])
        for k, (i, _) in enumerate(caps):
            j, kk = chips[k % len(chips)]; x, y = C.pad_xy(pose, j, kk); ang = rng.uniform(0, 2 * math.pi)
            pose[i] = (x + 1.6 * math.cos(ang), y + 1.6 * math.sin(ang), rng.choice((0, 90)))
    for r in ('D27',):   # J12 pins 10/12 protector: at the pins
        pose[C.idx[r]] = (125.4, 116.0, 0)
    return pose, dy
