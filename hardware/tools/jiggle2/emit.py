#!/usr/bin/env python3
"""jiggle2 stage 3: emit clean relaxed copper into a work-copy board and let
real KiCad DRC referee the result. Never touches rp2350_driver.kicad_pcb."""
import json, math, subprocess, sys
import pcbnew
from common import NM, BOARD_BASE, BOARD_OUT, DATA, load_board, load_json

sol = load_json('solution.json')
board = load_board(BOARD_BASE)

def rdp(pts, eps=0.01):
    """Simplify polyline; keeps endpoints."""
    if len(pts) < 3:
        return pts
    ax, ay = pts[0]; bx, by = pts[-1]
    L = math.hypot(bx-ax, by-ay)
    dmax, imax = 0.0, 0
    for i in range(1, len(pts)-1):
        px, py = pts[i]
        if L < 1e-9:
            d = math.hypot(px-ax, py-ay)
        else:
            d = abs((bx-ax)*(ay-py) - (ax-px)*(by-ay)) / L
        if d > dmax:
            dmax, imax = d, i
    if dmax > eps:
        return rdp(pts[:imax+1], eps)[:-1] + rdp(pts[imax:], eps)
    return [pts[0], pts[-1]]

def to_v(x, y):
    return pcbnew.VECTOR2I(int(round(x/NM)), int(round(y/NM)))

nets = board.GetNetsByName()
def find_net(name):
    n = nets.find(name)
    return None if n == nets.end() else n.value()[1]

EMIT_ALL = '--all' in sys.argv   # trust the DRC referee, not the classifier
added_t = added_v = skipped_dirty_t = skipped_dirty_v = withdrawn = no_net = 0
for e in sol['edges']:
    if e.get('withdrawn'):       # referee veto always wins
        withdrawn += 1
        continue
    if not e['clean'] and not EMIT_ALL:
        skipped_dirty_t += 1
        continue
    net = find_net(e['net'])
    if net is None:
        no_net += 1
        continue
    pts = rdp(e['pts'])
    for k in range(len(pts)-1):
        (x1, y1), (x2, y2) = pts[k], pts[k+1]
        if math.hypot(x2-x1, y2-y1) < 0.001:
            continue
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(to_v(x1, y1)); t.SetEnd(to_v(x2, y2))
        t.SetWidth(int(round(e['w']/NM)))
        t.SetLayer(e['layer'])
        t.SetNet(net)
        board.Add(t)
        added_t += 1
for v in sol['vias']:
    if v.get('withdrawn'):
        withdrawn += 1
        continue
    if not v['clean'] and not EMIT_ALL:
        skipped_dirty_v += 1
        continue
    net = find_net(v['net'])
    if net is None:
        no_net += 1
        continue
    via = pcbnew.PCB_VIA(board)
    via.SetPosition(to_v(v['x'], v['y']))
    via.SetDrill(int(round(v['drill']/NM)))
    try:
        via.SetWidth(int(round(v['dia']/NM)))
    except TypeError:
        via.SetWidth(pcbnew.PADSTACK.ALL_LAYERS, int(round(v['dia']/NM)))
    via.SetViaType(pcbnew.VIATYPE_THROUGH)
    via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    via.SetNet(net)
    board.Add(via)
    added_v += 1

print(f'emitted {added_t} track segments, {added_v} vias '
      f'(withheld: {skipped_dirty_t} dirty chains, {skipped_dirty_v} dirty vias, '
      f'{withdrawn} referee-withdrawn, {no_net} net lookup failures)')

filler = pcbnew.ZONE_FILLER(board)
filler.Fill(board.Zones())
pcbnew.SaveBoard(BOARD_OUT, board)
print(f'saved {BOARD_OUT}')

# ---- referee: real DRC on the work copy vs the baseline -----------------------
def drc(path, out):
    subprocess.run(['kicad-cli', 'pcb', 'drc', '--severity-error',
                    '--format', 'json', '-o', out, path],
                   check=True, capture_output=True)
    with open(out) as f:
        d = json.load(f)
    from collections import Counter
    return (len(d.get('unconnected_items', [])),
            Counter(v['type'] for v in d.get('violations', [])))

if '--no-drc' not in sys.argv:
    u0, t0 = drc(BOARD_BASE, f'{DATA}/drc_baseline.json')
    u1, t1 = drc(BOARD_OUT, f'{DATA}/drc_jiggle2.json')
    print(f'\nDRC referee:')
    print(f'  unconnected: {u0} -> {u1}')
    print(f'  violations:  {sum(t0.values())} -> {sum(t1.values())}')
    for k in sorted(set(t0) | set(t1)):
        print(f'    {k:24s} {t0.get(k, 0):4d} -> {t1.get(k, 0):4d}')
