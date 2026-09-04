#!/usr/bin/env python3
"""settle: encoder pair-chain metrics for one board.

    python3 pairs.py BOARD.kicad_pcb [--json OUT]

Per channel A-D: the copper length of each pair net today (P and N paths,
J12 -> transceiver), the airwires still open on them, the straight-line chain
J12 pin -> TVS -> switch -> filter -> transceiver (the floor any route has to
cover), and how far each part of the chain sits from its J12 pins.
"""
import sys, math, json, re
from collections import defaultdict
from common import load_board, NM
path = sys.argv[1]
b = load_board(path)
fps = {f.GetReference(): f for f in b.GetFootprints()}
def pos(r):
    p = fps[r].GetPosition(); return (p.x * NM, p.y * NM)
def padpos(r, n):
    for p in fps[r].Pads():
        if p.GetNumber() == n: q = p.GetPosition(); return (q.x * NM, q.y * NM)
L = defaultdict(float)
for t in b.GetTracks():
    if t.GetClass() == 'PCB_TRACK': L[t.GetNetname()] += t.GetLength() * NM
CH = {  # J12 pins (P, N), then the chain: TVS, P-switch, N-switch, filter, transceiver, term R, term switch, new R
 'A': dict(pins=('11', '9'), chain=['D23', 'U22', 'U26', 'FL2', 'U18'], new=['R82', 'U30', 'R97']),
 'B': dict(pins=('7', '5'),  chain=['D24', 'U23', 'U27', 'FL1', 'U19'], new=['R83', 'U31', 'R98']),
 'C': dict(pins=('3', '1'),  chain=['D25', 'U24', 'U25', 'FL3', 'U20'], new=['R84', 'U32', 'R99']),
 'D': dict(pins=('6', '8'),  chain=['D26', 'FL4', 'U21'],               new=['R85', 'U33', 'R100']),
}
out = {}
print(f'{"ch":2} {"P copper":>9} {"N copper":>9} {"chain floor":>11} {"J12->TRX":>9}  chain (mm from J12 pins)')
for ch, d in CH.items():
    pp, pn = padpos('J12', d['pins'][0]), padpos('J12', d['pins'][1])
    j = ((pp[0] + pn[0]) / 2, (pp[1] + pn[1]) / 2)
    P = L[f'/ENC_{ch}_P'] + L[f'/ENC_{ch}_P_SW']; N = L[f'/ENC_{ch}_N'] + L[f'/ENC_{ch}_N_SW']
    pts = [j] + [pos(r) for r in d['chain'] if r in fps]
    floor = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
    trx = math.dist(j, pos(d['chain'][-1]))
    dist = {r: round(math.dist(j, pos(r)), 1) for r in d['chain'] + d['new'] if r in fps}
    out[ch] = dict(P=round(P, 1), N=round(N, 1), floor=round(floor, 1), trx=round(trx, 1), dist=dist)
    print(f'{ch:2} {P:9.1f} {N:9.1f} {floor:11.1f} {trx:9.1f}  ' + ' '.join(f'{r}({v})' for r, v in dist.items()))
if '--json' in sys.argv:
    json.dump(out, open(sys.argv[sys.argv.index('--json') + 1], 'w'), indent=1)
