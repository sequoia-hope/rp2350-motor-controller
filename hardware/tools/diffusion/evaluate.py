#!/usr/bin/env python3
"""diffusion experiment: score the generated placements.

Reads chipdiffusion's sample{i}.pkl outputs (board coords, bottom-left per
object) and computes metrics the model's own report can't be trusted for —
its legality masks out ports∪fixed, so mover-vs-fixed overlap goes uncounted.
Rect model matches build_dataset's: courtyard bboxes, grandfathered pad
blockers, canvas bounds.

Per sample: overlap (mover-mover + mover-fixed, sum/max/pairs>10um), oob,
HPWL over the star netlist, mean travel vs rev-A shipped and rev-B target.
Same metrics for the two anchors (shipped / target) for context.

Writes diffusion_results.json for render_page.py.  Run with the clone venv.
"""
import glob, json, os, pickle, re, sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CLONE = os.path.join(HERE, '..', '..', '..', 'tmp', 'chipdiffusion')
SAMPLES = os.path.join(CLONE, 'logs', 'diffusion_debug',
                       'rp2350.eval_guided.300', 'samples')

with open(os.path.join(HERE, 'rp2350_netlist.json')) as f:
    NL = json.load(f)
objs, nets, canvas = NL['objects'], NL['nets'], NL['canvas']
cx0, cy0, cx1, cy1 = canvas
V = len(objs)
kinds = [o['kind'] for o in objs]
W = np.array([o['w'] for o in objs])
H = np.array([o['h'] for o in objs])
ref = np.array([[o['x0'], o['y0']] for o in objs])
ship = ref.copy()
for i, o in enumerate(objs):
    if o['kind'] == 'mov':
        ship[i] = [o['ship_x0'], o['ship_y0']]
mov = np.array([k == 'mov' for k in kinds])
solid = np.array([k != 'port' for k in kinds])

edges = [(u, ux, uy, v, vx, vy) for n in nets
         for u, ux, uy, v, vx, vy in n['edges']]


def metrics(pos):
    """pos: (V,2) bottom-left. Movers judged against movers + fixed."""
    x0, y0 = pos[:, 0], pos[:, 1]
    x1, y1 = x0 + W, y0 + H
    tot = mx = 0.0
    pairs = []
    idx = np.where(solid)[0]
    for a_i, a in enumerate(idx):
        for b in idx[a_i + 1:]:
            if not (mov[a] or mov[b]):
                continue
            ox = min(x1[a], x1[b]) - max(x0[a], x0[b])
            oy = min(y1[a], y1[b]) - max(y0[a], y0[b])
            if ox > 1e-4 and oy > 1e-4:
                d = min(ox, oy)
                tot += d
                mx = max(mx, d)
                if d > 0.01:
                    pairs.append([objs[a]['name'], objs[b]['name'], round(d, 3)])
    oob = float(np.sum(np.maximum(0, cx0 - x0[mov])) +
                np.sum(np.maximum(0, cy0 - y0[mov])) +
                np.sum(np.maximum(0, x1[mov] - cx1)) +
                np.sum(np.maximum(0, y1[mov] - cy1)))
    hpwl = sum(abs((pos[u][0] + ux) - (pos[v][0] + vx)) +
               abs((pos[u][1] + uy) - (pos[v][1] + vy))
               for u, ux, uy, v, vx, vy in edges)
    return dict(overlap=round(tot, 4), overlap_max=round(mx, 4),
                pairs=sorted(pairs, key=lambda p: -p[2]), oob=round(oob, 4),
                hpwl=round(float(hpwl), 2))


def travel(pos, base):
    d = np.hypot(*(pos[mov] - base[mov]).T)
    return dict(mean=round(float(d.mean()), 3), max=round(float(d.max()), 3))


out = dict(canvas=canvas,
           anchors=dict(shipped=metrics(ship), target=metrics(ref)),
           samples=[])
files = sorted(glob.glob(os.path.join(SAMPLES, 'sample*.pkl')),
               key=lambda p: int(re.search(r'\d+', os.path.basename(p)).group()))
for path in files:
    with open(path, 'rb') as f:
        pos = np.asarray(pickle.load(f), dtype=float)
    assert pos.shape == (V, 2), pos.shape
    m = metrics(pos)
    m.update(idx=int(re.search(r'\d+', os.path.basename(path)).group()),
             travel_ship=travel(pos, ship), travel_target=travel(pos, ref),
             movers={objs[i]['name']: [round(pos[i][0], 4), round(pos[i][1], 4)]
                     for i in range(V) if mov[i]})
    out['samples'].append(m)

# their own per-sample report, for the page footer
csv = os.path.join(CLONE, 'logs', 'diffusion_debug',
                   'rp2350.eval_guided.300', 'metrics.csv')
if os.path.exists(csv):
    import csv as csvmod
    with open(csv) as f:
        out['model_report'] = list(csvmod.DictReader(f))

tgt_hpwl = out['anchors']['target']['hpwl']
print(f"anchors: shipped hpwl {out['anchors']['shipped']['hpwl']}  "
      f"target hpwl {tgt_hpwl} overlap {out['anchors']['target']['overlap']}")
for s in out['samples']:
    print(f"s{s['idx']:02d}  hpwl {s['hpwl']:8.2f} ({s['hpwl']/tgt_hpwl:.3f}x)  "
          f"overlap {s['overlap']:7.4f} max {s['overlap_max']:.3f} "
          f"({len(s['pairs'])} pairs)  oob {s['oob']:.3f}  "
          f"travel {s['travel_ship']['mean']:.2f}mm")
with open(os.path.join(HERE, 'diffusion_results.json'), 'w') as f:
    json.dump(out, f)
print(f"{len(out['samples'])} samples -> diffusion_results.json")
