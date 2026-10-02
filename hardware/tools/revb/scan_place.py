#!/usr/bin/env python3
"""Where can a new footprint legally go? Feasibility scan behind sync_pcb.py.

For each part in JOBS this drops the real footprint on a copy of the board at
every 0.25 mm / 90 degree pose in a window and asks KiCad for the geometry, so
courtyards, pad shapes, pad rotation and PTH drills are the ones DRC will use.
An earlier version of this modelled pads from GetSize() and missed both J3's
through-hole shield pad and U8's PTH ground pad -- hence "ask KiCad, don't
model it".

Constraints are split:

  hard  same-side courtyard of a real part, any other-net pad, board edge
        (outline shrunk by the edge clearance, so cutouts count too)
  soft  other-net track segments the new pads cross -- rippable, but each one
        is copper somebody has to route again, so it is the cost function

Poses with any hard conflict are discarded; the rest are ranked by
segments-to-rip and then distance from the anchor. Nets in FREE are copper the
sync is ripping anyway, so crossing them is free. Nets in PROTECT (the USB
differential pair) are never rippable and disqualify a pose outright.

Note: run against the PRE-sync board to reproduce the original decision --
sync_pcb.py has since moved these parts, and the current board therefore scores
the poses that were chosen, not the ones that were available:

  git show e5e7a12:hardware/rp2350_driver.kicad_pcb > /tmp/pre.kicad_pcb
  scan_place.py --board /tmp/pre.kicad_pcb

  usage: scan_place.py [--board PCB] [--part U29] [--window 8] [--top 8]
"""
import os, sys, math, argparse, collections

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(HW, 'tools', 'jiggle2'))
from common import load_board, NM                        # noqa: E402
import pcbnew                                            # noqa: E402

BOARD = os.path.join(HW, 'rp2350_driver.kicad_pcb')
KLIB = '/usr/share/kicad/footprints'
PARTS = os.path.join(HW, 'parts')
CLR = 0.18
EDGE = 0.3
STEP = 0.25

# copper the sync rips regardless -- crossing it costs nothing
FREE = {'VBUS', 'Net-(Q5-G)', 'Net-(JP1-A)', 'Net-(U29-VCAP)'}
# the USB pair is not up for renegotiation
PROTECT = {'/USB_DP', '/USB_DN', '/USB_D+', '/USB_D-'}

JOBS = {
    'U29': dict(lib=f'{KLIB}/Package_SO.pretty', fp='MSOP-8_3x3mm_P0.65mm',
                nets={'1': 'Net-(Q5-G)', '2': 'VBUS', '4': 'VBUS',
                      '5': 'VBUS', '6': 'GND', '8': 'Net-(JP1-A)'},
                anchor=(166.745, 95.825), back=True, rots=(0, 90, 180, 270)),
    'Q5':  dict(lib=PARTS, fp='PowerPAK_SO-8_123',
                nets={'1': 'VBUS', '2': 'Net-(Q5-G)', '3': 'Net-(JP1-A)'},
                anchor=(169.400, 104.075), back=True, rots=(180,),
                note='rot 180 only: source east to J3, drain west to JP1'),
    'D31': dict(lib=f'{KLIB}/Diode_SMD.pretty', fp='D_SOD-123F',
                nets={'1': 'VBUS', '2': 'GND'},
                anchor=(185.725, 116.355), back=None, rots=(0, 90, 180, 270)),
}


def pad_box(pad, extra=0.0):
    bb = pad.GetBoundingBox()
    return (bb.GetLeft() * NM - extra, bb.GetTop() * NM - extra,
            bb.GetRight() * NM + extra, bb.GetBottom() * NM + extra)


def overlap(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def seg_pt_dist(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--board', default=BOARD)
    ap.add_argument('--part', action='append', choices=sorted(JOBS))
    ap.add_argument('--window', type=float, default=8.0)
    ap.add_argument('--top', type=int, default=8)
    args = ap.parse_args()
    parts = args.part or sorted(JOBS)

    bd = load_board(args.board)
    outline = pcbnew.SHAPE_POLY_SET()
    bd.GetBoardPolygonOutlines(outline)
    inner = pcbnew.SHAPE_POLY_SET(outline)
    inner.Inflate(int(-EDGE / NM), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS,
                  int(0.01 / NM))

    for name in parts:
        job = JOBS[name]
        ax, ay = job['anchor']
        win = args.window

        def near(x, y):
            return abs(x - ax) <= win + 6 and abs(y - ay) <= win + 6

        fpads, crts, segs = [], [], []
        for fp in bd.GetFootprints():
            ref = fp.GetReference()
            if ref == name:
                continue
            for lay in (pcbnew.F_CrtYd, pcbnew.B_CrtYd):
                poly = fp.GetCourtyard(lay)
                if poly and poly.OutlineCount():
                    b = poly.BBox()
                    box = (b.GetLeft() * NM, b.GetTop() * NM,
                           b.GetRight() * NM, b.GetBottom() * NM)
                    if near((box[0] + box[2]) / 2, (box[1] + box[3]) / 2):
                        # a test point is a bare probe pad: cheap to shove
                        crts.append((lay, box, ref, ref.startswith('TP')))
            for p in fp.Pads():
                pb = pad_box(p)
                if near((pb[0] + pb[2]) / 2, (pb[1] + pb[3]) / 2):
                    fpads.append((pb, p.GetNetname(),
                                  p.IsOnLayer(pcbnew.F_Cu),
                                  p.IsOnLayer(pcbnew.B_Cu), ref,
                                  ref.startswith('TP')))
        for t in bd.GetTracks():
            via = t.Type() == pcbnew.PCB_VIA_T
            if via:
                p = t.GetPosition()
                a = b = (p.x * NM, p.y * NM)
                hw = t.GetDrill() / 2 * NM + 0.15
                lay = None
            else:
                s, e = t.GetStart(), t.GetEnd()
                a, b = (s.x * NM, s.y * NM), (e.x * NM, e.y * NM)
                hw = t.GetWidth() / 2 * NM
                lay = t.GetLayer()
            if near(*a) or near(*b):
                segs.append((lay, a, b, hw, t.GetNetname()))

        probe = pcbnew.FootprintLoad(job['lib'], job['fp'])
        if probe is None:
            print(f'{name}: cannot load {job["fp"]}'); continue
        bd.Add(probe)
        for p in probe.Pads():
            nn = job['nets'].get(p.GetNumber())
            if nn:
                n = bd.FindNet(nn)
                if n:
                    p.SetNet(n)

        sides = (True, False) if job['back'] is None else (job['back'],)
        results = []
        n = int(win / STEP)
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                x, y = ax + i * STEP, ay + j * STEP
                d = math.hypot(x - ax, y - ay)
                if d > win:
                    continue
                for back in sides:
                    for deg in job['rots']:
                        probe.SetPosition(pcbnew.VECTOR2I(int(x / NM), int(y / NM)))
                        if probe.IsFlipped() != back:
                            probe.Flip(probe.GetPosition(), False)
                        probe.SetOrientationDegrees(deg)
                        poly = probe.GetCourtyard(
                            pcbnew.B_CrtYd if back else pcbnew.F_CrtYd)
                        if not (poly and poly.OutlineCount()):
                            continue
                        bb = poly.BBox()
                        c = (bb.GetLeft() * NM, bb.GetTop() * NM,
                             bb.GetRight() * NM, bb.GetBottom() * NM)
                        if any(not inner.Contains(
                                pcbnew.VECTOR2I(int(px / NM), int(py / NM)))
                               for px, py in ((c[0], c[1]), (c[2], c[1]),
                                              (c[0], c[3]), (c[2], c[3]))):
                            continue
                        want = pcbnew.B_CrtYd if back else pcbnew.F_CrtYd
                        shove = set()
                        hard = False
                        for lay, box, ref, soft in crts:
                            if lay != want or not overlap(c, box):
                                continue
                            if soft:
                                shove.add(ref)
                            else:
                                hard = True; break
                        if hard:
                            continue
                        rip = collections.Counter()
                        for p in probe.Pads():
                            mb = pad_box(p, CLR)
                            mn = p.GetNetname()
                            mf = p.IsOnLayer(pcbnew.F_Cu)
                            mbk = p.IsOnLayer(pcbnew.B_Cu)
                            for pb, pn, pf, pbk, ref, soft in fpads:
                                if (pn == mn and pn) or not ((mf and pf) or (mbk and pbk)):
                                    continue
                                if overlap(mb, pb):
                                    (shove.add(ref) if soft else None)
                                    if not soft:
                                        hard = True; break
                            if hard:
                                break
                            cx = (mb[0] + mb[2]) / 2; cy = (mb[1] + mb[3]) / 2
                            hx = (mb[2] - mb[0]) / 2; hy = (mb[3] - mb[1]) / 2
                            for lay, a, b, hw, sn in segs:
                                if sn == mn or sn in FREE:
                                    continue
                                if lay is not None and (lay == pcbnew.B_Cu) != back:
                                    continue
                                if lay is not None and lay not in (pcbnew.F_Cu, pcbnew.B_Cu):
                                    continue
                                if seg_pt_dist((cx, cy), a, b) < hw + min(hx, hy):
                                    rip[sn] += 1
                        if hard or any(k in PROTECT for k in rip):
                            continue
                        results.append((sum(rip.values()), len(shove), d,
                                        x, y, deg, back, dict(rip),
                                        sorted(shove)))
        bd.Remove(probe)
        results.sort()
        print(f'\n=== {name}: {job["fp"]} anchor=({ax}, {ay}) '
              f'window={win}mm ===')
        if job.get('note'):
            print(f'    {job["note"]}')
        if not results:
            print('    no pose without hard conflicts'); continue
        print(f'    {len(results)} hard-clean poses; best {args.top}:')
        for r in results[:args.top]:
            nrip, nshove, d, x, y, deg, back, rip, shove = r
            print(f'      rip={nrip} shove={nshove} d={d:4.2f} '
                  f'({x:8.3f},{y:8.3f}) rot{deg:3d} '
                  f'{"B" if back else "F"}.Cu {rip} {shove}')


if __name__ == '__main__':
    main()
