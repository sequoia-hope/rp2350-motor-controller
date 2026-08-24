#!/usr/bin/env python3
"""Payoff probe: pad-to-pad gaps at the named routing blockers (the boxed-in
pads from review/modules.html) on any board.  usage: gaps.py BOARD [BOARD2...]"""
import sys, os, math
S = os.path.dirname(os.path.abspath(__file__)); HW = os.path.dirname(os.path.dirname(S))
sys.path.insert(0, os.path.join(HW, 'tools', 'jiggle2'))
from common import load_board, NM
PAIRS = [('U20', 'FL3'), ('U18', 'FL2'), ('U19', 'FL1'),
         ('R106', 'H1'), ('R109', 'H4'), ('R111', 'H2'), ('R112', 'H3'),
         ('U30', 'U32'), ('U32', 'U33')]
def gaps(path):
    bd = load_board(path)
    fp = {f.GetReference(): f for f in bd.GetFootprints()}
    out = {}
    for a, b in PAIRS:
        if a not in fp or b not in fp: out[(a, b)] = None; continue
        best = 1e9
        for pa in fp[a].Pads():
            ba = pa.GetBoundingBox()
            for pb in fp[b].Pads():
                bb = pb.GetBoundingBox()
                dx = max(ba.GetLeft()-bb.GetRight(), bb.GetLeft()-ba.GetRight(), 0)*NM
                dy = max(ba.GetTop()-bb.GetBottom(), bb.GetTop()-ba.GetBottom(), 0)*NM
                best = min(best, math.hypot(dx, dy))
        out[(a, b)] = best
    bb = bd.GetBoardEdgesBoundingBox()
    out[('board', 'size')] = (bb.GetWidth()*NM, bb.GetHeight()*NM)
    return out
if __name__ == '__main__':
    res = [gaps(p) for p in sys.argv[1:]]
    print(f"{'pair':12s}" + ''.join(f'{os.path.basename(p)[:18]:>20s}' for p in sys.argv[1:]))
    for k in res[0]:
        if k == ('board', 'size'):
            print(f"{'board-mm':12s}" + ''.join(f"{r[k][0]:.1f}×{r[k][1]:.1f}".rjust(20) for r in res))
        else:
            print(f"{k[0]+'-'+k[1]:12s}" + ''.join(f"{(r[k]*1000 if r[k] is not None else -1):19.0f}u" for r in res))
