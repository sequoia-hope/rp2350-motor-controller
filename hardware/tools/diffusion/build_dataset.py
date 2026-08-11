#!/usr/bin/env python3
"""diffusion experiment: rp2350_netlist.json -> chipdiffusion dataset.

Writes datasets/graph/rp2350/ inside the chipdiffusion clone: K identical
copies of the graph (graph{i}.pickle) + the rev-B-target reference placement
(output{i}.pickle) so eval.py's val loop draws K independent samples, plus
config.yaml and meta.json (object names/kinds for later rendering).

Mask semantics (see models.py patch in the clone):
  is_ports       ports ∪ fixed — pinned to x_in during sampling, flagged
                 "does not move" to the network
  legality_mask  ports only — ONLY these are excluded from the legality
                 guidance potential, so fixed blockers still repel movers
  is_macros      movers ∪ fixed (metrics only)

Run with the clone's venv python (needs torch + torch_geometric).
"""
import json, os, pickle, sys

import numpy as np
import torch
from torch_geometric.data import Data

HERE = os.path.dirname(os.path.abspath(__file__))
CLONE = os.path.join(HERE, '..', '..', '..', 'tmp', 'chipdiffusion')
K = int(sys.argv[1]) if len(sys.argv) > 1 else 16

with open(os.path.join(HERE, 'rp2350_netlist.json')) as f:
    NL = json.load(f)
objs, nets, canvas = NL['objects'], NL['nets'], NL['canvas']
V = len(objs)

sizes = torch.tensor([[o['w'], o['h']] for o in objs], dtype=torch.float32)
is_ports = torch.tensor([o['kind'] != 'mov' for o in objs])
legality_mask = torch.tensor([o['kind'] == 'port' for o in objs])
is_macros = torch.tensor([o['kind'] != 'port' for o in objs])
ref_place = np.array([[o['x0'], o['y0']] for o in objs], dtype=np.float64)

fwd = [[], []]
attr_fwd = []
for n in nets:
    for u, ux, uy, v, vx, vy in n['edges']:
        fwd[0].append(u)
        fwd[1].append(v)
        attr_fwd.append([ux, uy, vx, vy])
edge_index = torch.tensor([fwd[0] + fwd[1], fwd[1] + fwd[0]], dtype=torch.int64)
attr_rev = [[vx, vy, ux, uy] for ux, uy, vx, vy in attr_fwd]
edge_attr = torch.tensor(attr_fwd + attr_rev, dtype=torch.float64)

data = Data(x=sizes, edge_index=edge_index, edge_attr=edge_attr,
            is_ports=is_ports, is_macros=is_macros,
            legality_mask=legality_mask, chip_size=list(canvas))

# rectangle-model sanity: the reference placement itself should be ~legal
def overlaps(i, j):
    ax0, ay0 = ref_place[i]
    bx0, by0 = ref_place[j]
    ox = min(ax0 + objs[i]['w'], bx0 + objs[j]['w']) - max(ax0, bx0)
    oy = min(ay0 + objs[i]['h'], by0 + objs[j]['h']) - max(ay0, by0)
    return min(ox, oy) if (ox > 0 and oy > 0) else 0.0

solid = [i for i in range(V) if objs[i]['kind'] != 'port']
worst = []
for a in range(len(solid)):
    for b in range(a + 1, len(solid)):
        d = overlaps(solid[a], solid[b])
        if d > 1e-6:
            worst.append((round(d, 3), objs[solid[a]]['name'],
                          objs[solid[b]]['name']))
worst.sort(reverse=True)
print(f'reference-placement overlaps (rect model): {len(worst)}')
for w in worst[:12]:
    print('  ', w)

out_dir = os.path.join(CLONE, 'datasets', 'graph', 'rp2350')
os.makedirs(out_dir, exist_ok=True)
for i in range(K):
    with open(os.path.join(out_dir, f'graph{i}.pickle'), 'wb') as f:
        pickle.dump(data, f)
    with open(os.path.join(out_dir, f'output{i}.pickle'), 'wb') as f:
        pickle.dump(ref_place, f)
with open(os.path.join(out_dir, 'config.yaml'), 'w') as f:
    f.write(f'train_samples: 0\nval_samples: {K}\nscale: 1\n')
with open(os.path.join(out_dir, 'meta.json'), 'w') as f:
    json.dump(dict(canvas=canvas,
                   names=[o['name'] for o in objs],
                   kinds=[o['kind'] for o in objs],
                   sizes=[[o['w'], o['h']] for o in objs],
                   shipped=[[o.get('ship_x0'), o.get('ship_y0')]
                            for o in objs]), f)
print(f'{V} objects, {edge_index.shape[1]} directed edges -> {out_dir} (K={K})')
