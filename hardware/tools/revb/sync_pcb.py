#!/usr/bin/env python3
"""Apply the rev-B F-27/F-39 schematic changes to rp2350_driver.kicad_pcb.

kicad-cli has no "update PCB from schematic", and the pcbnew SWIG bindings
expose no BOARD_NETLIST_UPDATER, so the sync is done explicitly here. The
delta below is not hand-written guesswork: it is exactly the set of
differences `check_sync.py` reports between the schematic netlist and the
board, and that script is the acceptance test for this one.

  U29  Package_TO_SOT_SMD:SOT-23-6 -> Package_SO:MSOP-8_3x3mm_P0.65mm
       (LM74700 -> LTC4359CMS8, 6 pads -> 8, full re-net)
  Q5   Package_TO_SOT_SMD:SOT-89-3 -> parts:PowerPAK_SO-8_123
       (TMG08N10SI -> E100N4P0HL1, S/G/D re-net)
  C79  deleted (it was the LM74700 charge-pump cap)
  D31  added on VBUS at J3
  U11  pad 5 VBUS -> +3V3

Placement comes from the feasibility scan in `scan_place.py`: every new
courtyard clears same-side courtyards, other-net pads and the board edge.
Copper the new pads sit on is ripped here and left as airwires; routing is a
separate pass. Rip is deliberately conservative (bbox + clearance).
"""
import os, sys, math, argparse

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(HW, 'tools', 'jiggle2'))
from common import load_board, NM, quiet_stderr        # noqa: E402
import pcbnew                                          # noqa: E402

BOARD = os.path.join(HW, 'rp2350_driver.kicad_pcb')
KLIB = '/usr/share/kicad/footprints'
PARTS = os.path.join(HW, 'parts')
CLR = 0.18          # board min_clearance, mm
RIP_MARGIN = 0.05   # extra mm when deciding what copper to rip

# --- the sync spec -----------------------------------------------------
SWAPS = [
    dict(ref='U29', lib=f'{KLIB}/Package_SO.pretty', fp='MSOP-8_3x3mm_P0.65mm',
         fpid='Package_SO:MSOP-8_3x3mm_P0.65mm', value='LTC4359CMS8',
         at=(167.745, 95.825), rot=0, back=True, lcsc='C688235',
         nets={'1': 'Net-(Q5-G)', '2': 'VBUS',
               '3': 'unconnected-(U29-NC-Pad3)', '4': 'VBUS', '5': 'VBUS',
               '6': 'GND', '7': 'unconnected-(U29-NC-Pad7)',
               '8': 'Net-(JP1-A)'}),
    dict(ref='Q5', lib=PARTS, fp='PowerPAK_SO-8_123',
         fpid='parts:PowerPAK_SO-8_123', value='E100N4P0HL1',
         at=(170.400, 103.325), rot=180, back=True, lcsc='C29781176',
         nets={'1': 'VBUS', '2': 'Net-(Q5-G)', '3': 'Net-(JP1-A)'}),
]
ADDS = [
    dict(ref='D31', lib=f'{KLIB}/Diode_SMD.pretty', fp='D_SOD-123F',
         fpid='Diode_SMD:D_SOD-123F', value='SMF24A',
         at=(183.725, 111.605), rot=180, back=False, lcsc='C169430',
         nets={'1': 'VBUS', '2': 'GND'}),
]
DELETES = ['C79']
PAD_RENET = [('U11', '5', '+3V3')]
TP_RELOCATE = ['TP15', 'TP17']

# copper on these nets, inside the work box, is rebuilt by the routing pass
RIP_NETS = {'VBUS', 'Net-(Q5-G)', 'Net-(JP1-A)', 'Net-(U29-VCAP)'}
RIP_BOX = (159.0, 90.0, 178.0, 110.0)
# U11's old pad-5 VBUS stub lives outside the box; rip it separately
RIP_BOX_U11 = (169.0, 110.0, 175.0, 116.0)


def mm(v):
    return int(round(v / NM))


def vec(x, y):
    return pcbnew.VECTOR2I(mm(x), mm(y))


def net(bd, name):
    """Existing net by name, created if the schematic introduces it."""
    n = bd.FindNet(name)
    if n is None:
        n = pcbnew.NETINFO_ITEM(bd, name)
        bd.Add(n)
    return n


def fp_by_ref(bd, ref):
    for f in bd.GetFootprints():
        if f.GetReference() == ref:
            return f
    return None


def seg_pt_dist(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def track_pts(t):
    if t.Type() == pcbnew.PCB_VIA_T:
        p = t.GetPosition()
        return (p.x * NM, p.y * NM), (p.x * NM, p.y * NM)
    s, e = t.GetStart(), t.GetEnd()
    return (s.x * NM, s.y * NM), (e.x * NM, e.y * NM)


def in_box(x, y, b):
    return b[0] <= x <= b[2] and b[1] <= y <= b[3]


def place(bd, spec, log):
    """Load spec['fp'], drop it on the board, net its pads. Returns footprint."""
    fp = pcbnew.FootprintLoad(spec['lib'], spec['fp'])
    if fp is None:
        raise SystemExit(f"cannot load {spec['lib']}/{spec['fp']}")
    bd.Add(fp)
    fp.SetPosition(vec(*spec['at']))
    if spec['back']:
        fp.Flip(fp.GetPosition(), False)
    fp.SetOrientationDegrees(spec['rot'])
    fp.SetReference(spec['ref'])
    fp.SetValue(spec['value'])
    try:
        fp.SetFPIDAsString(spec['fpid'])
    except Exception:
        pass
    if spec.get('lcsc'):
        fp.SetField('LCSC', spec['lcsc'])
        # SetField drops a *visible, unmirrored* field on the silkscreen, which
        # is both a DRC warning on back-side parts and stray solder-mask text.
        # Park it on the Fab layer, hidden, mirrored to match the side.
        for fld in fp.GetFields():
            if fld.GetName() == 'LCSC':
                fld.SetLayer(pcbnew.B_Fab if spec['back'] else pcbnew.F_Fab)
                fld.SetVisible(False)
                fld.SetMirrored(bool(spec['back']))
    for pad in fp.Pads():
        nm_ = spec['nets'].get(pad.GetNumber())
        if nm_:
            pad.SetNet(net(bd, nm_))
    side = 'B.Cu' if spec['back'] else 'F.Cu'
    log(f"  placed {spec['ref']:4s} {spec['fpid']:38s} at "
        f"({spec['at'][0]:.3f},{spec['at'][1]:.3f}) rot {spec['rot']} {side}")
    return fp


def pad_box(pad, extra=0.0):
    """KiCad's own pad bbox in mm. Unlike GetSize() this respects rotation,
    roundrect/oval shapes and the drill of a PTH pad -- getting this wrong is
    what let a test point land on U8's through-hole GND pad."""
    bb = pad.GetBoundingBox()
    return (bb.GetLeft() * NM - extra, bb.GetTop() * NM - extra,
            bb.GetRight() * NM + extra, bb.GetBottom() * NM + extra)


def boxes_overlap(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def verify(bd, refs, log):
    """Report pad conflicts for the placed/moved parts, from real geometry."""
    mine, others = [], []
    for fp in bd.GetFootprints():
        tgt = fp.GetReference() in refs
        for pad in fp.Pads():
            rec = (fp.GetReference(), pad.GetNumber(), pad.GetNetname(), pad,
                   pad.IsOnLayer(pcbnew.F_Cu), pad.IsOnLayer(pcbnew.B_Cu))
            (mine if tgt else others).append(rec)
    bad = 0
    for ref, num, nname, pad, onf, onb in mine:
        box = pad_box(pad, CLR)
        for oref, onum, onname, opad, oonf, oonb in others:
            if onname == nname and onname:
                continue
            if not ((onf and oonf) or (onb and oonb)):
                continue
            if boxes_overlap(box, pad_box(opad)):
                log(f'   CONFLICT {ref}.{num} [{nname}] vs '
                    f'{oref}.{onum} [{onname}]')
                bad += 1
    log(f'8. pad-level verification: {bad} conflict(s)')
    return bad


def new_pad_boxes(fps):
    """(x0,y0,x1,y1,net,back_only,front_only) per pad, inflated by clearance."""
    out = []
    for fp in fps:
        for pad in fp.Pads():
            bb = pad.GetBoundingBox()
            out.append((bb.GetLeft() * NM - CLR - RIP_MARGIN,
                        bb.GetTop() * NM - CLR - RIP_MARGIN,
                        bb.GetRight() * NM + CLR + RIP_MARGIN,
                        bb.GetBottom() * NM + CLR + RIP_MARGIN,
                        pad.GetNetname(),
                        pad.IsOnLayer(pcbnew.F_Cu), pad.IsOnLayer(pcbnew.B_Cu)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--apply', action='store_true', help='write the board')
    ap.add_argument('--out', default=BOARD)
    ap.add_argument('--force', action='store_true',
                    help='write even if verification reports conflicts')
    args = ap.parse_args()
    lines = []

    def log(s):
        lines.append(s)
        print(s)

    bd = load_board(BOARD)
    log(f'loaded {BOARD}')

    # 1. rip the affected nets inside the work box -----------------------
    ripped = 0
    for t in list(bd.GetTracks()):
        if t.GetNetname() not in RIP_NETS:
            continue
        a, b = track_pts(t)
        if (in_box(*a, RIP_BOX) or in_box(*b, RIP_BOX) or
                in_box(*a, RIP_BOX_U11) or in_box(*b, RIP_BOX_U11)):
            bd.Delete(t); ripped += 1
    log(f'1. ripped {ripped} segments/vias on {sorted(RIP_NETS)} in the work box')

    # 2. delete withdrawn parts and their stubs --------------------------
    for ref in DELETES:
        fp = fp_by_ref(bd, ref)
        if fp is None:
            log(f'2. {ref}: already absent'); continue
        # read everything out as plain floats before the object goes away
        px, py = fp.GetPosition().x * NM, fp.GetPosition().y * NM
        nets_here = {p.GetNetname() for p in fp.Pads()}
        bd.Delete(fp)
        n = 0
        for t in list(bd.GetTracks()):
            if t.GetNetname() not in nets_here:
                continue
            a, b = track_pts(t)
            if min(math.hypot(a[0] - px, a[1] - py),
                   math.hypot(b[0] - px, b[1] - py)) < 2.0:
                bd.Delete(t); n += 1
        log(f'2. deleted {ref} and {n} adjacent stub segments')

    # 3. swap footprints -------------------------------------------------
    placed = []
    for spec in SWAPS:
        old = fp_by_ref(bd, spec['ref'])
        if old is not None:
            log(f"3. removing old {spec['ref']} ({old.GetFPIDAsString()})")
            bd.Delete(old)
        placed.append(place(bd, spec, log))

    # 4. add new parts ---------------------------------------------------
    for spec in ADDS:
        if fp_by_ref(bd, spec['ref']) is not None:
            log(f"4. {spec['ref']} already present, skipping"); continue
        placed.append(place(bd, spec, log))

    # 5. re-net pads whose schematic connection changed ------------------
    for ref, padnum, netname in PAD_RENET:
        fp = fp_by_ref(bd, ref)
        hit = [p for p in fp.Pads() if p.GetNumber() == padnum]
        if not hit:
            raise SystemExit(f'{ref} has no pad {padnum}')
        old = hit[0].GetNetname()
        hit[0].SetNet(net(bd, netname))
        log(f'5. {ref} pad {padnum}: {old} -> {netname}')

    # 6. shove the test points the new PowerPAK lands on -----------------
    occupied = []
    for fp in bd.GetFootprints():
        for lay in (pcbnew.F_CrtYd, pcbnew.B_CrtYd):
            poly = fp.GetCourtyard(lay)
            if poly and poly.OutlineCount():
                bb = poly.BBox()
                occupied.append((fp.GetReference(), lay,
                                 bb.GetLeft() * NM, bb.GetTop() * NM,
                                 bb.GetRight() * NM, bb.GetBottom() * NM))
    for ref in TP_RELOCATE:
        fp = fp_by_ref(bd, ref)
        if fp is None:
            log(f'6. {ref} missing'); continue
        poly = fp.GetCourtyard(pcbnew.B_CrtYd) or fp.GetCourtyard(pcbnew.F_CrtYd)
        bb = poly.BBox()
        w = (bb.GetRight() - bb.GetLeft()) * NM / 2
        h = (bb.GetBottom() - bb.GetTop()) * NM / 2
        lay = pcbnew.B_CrtYd if fp.GetLayer() == pcbnew.B_Cu else pcbnew.F_CrtYd
        back = fp.GetLayer() == pcbnew.B_Cu
        p0 = fp.GetPosition(); ox, oy = p0.x * NM, p0.y * NM
        own = {p.GetNetname() for p in fp.Pads()}
        prad = max((max(p.GetSize().x, p.GetSize().y) * NM / 2
                    for p in fp.Pads()), default=0.5)
        # a test point is a probe pad: it may not land on foreign copper either
        foreign = []
        for t in bd.GetTracks():
            if t.GetNetname() in own:
                continue
            tl = None if t.Type() == pcbnew.PCB_VIA_T else t.GetLayer()
            if tl is not None and tl != (pcbnew.B_Cu if back else pcbnew.F_Cu):
                continue
            a, b = track_pts(t)
            if abs(a[0] - ox) > 8 and abs(b[0] - ox) > 8:
                continue
            hw = 0.0 if t.Type() == pcbnew.PCB_VIA_T else t.GetWidth() / 2 * NM
            foreign.append((a, b, hw))
        # own pad extents relative to the footprint origin, and every foreign
        # pad that shares a copper layer with it (PTH pads share both)
        mypads = [(pad_box(p)[0] - ox, pad_box(p)[1] - oy,
                   pad_box(p)[2] - ox, pad_box(p)[3] - oy,
                   p.GetNetname(), p.IsOnLayer(pcbnew.F_Cu),
                   p.IsOnLayer(pcbnew.B_Cu)) for p in fp.Pads()]
        fpads = []
        for ofp in bd.GetFootprints():
            if ofp.GetReference() == ref:
                continue
            for p in ofp.Pads():
                pb = pad_box(p)
                if abs((pb[0] + pb[2]) / 2 - ox) > 10:
                    continue
                fpads.append((pb, p.GetNetname(), p.IsOnLayer(pcbnew.F_Cu),
                              p.IsOnLayer(pcbnew.B_Cu)))
        best = None
        for i in range(-24, 25):
            for j in range(-24, 25):
                x, y = ox + i * 0.25, oy + j * 0.25
                if any(oref != ref and olay == lay and
                       x - w < a1 and a0 < x + w and y - h < b1 and b0 < y + h
                       for oref, olay, a0, b0, a1, b1 in occupied):
                    continue
                if any(seg_pt_dist((x, y), a, b) < hw + CLR + prad
                       for a, b, hw in foreign):
                    continue
                clash = False
                for mx0, my0, mx1, my1, mnet, monf, monb in mypads:
                    mb = (x + mx0 - CLR, y + my0 - CLR,
                          x + mx1 + CLR, y + my1 + CLR)
                    for pb, pnet, ponf, ponb in fpads:
                        if pnet == mnet and pnet:
                            continue
                        if not ((monf and ponf) or (monb and ponb)):
                            continue
                        if boxes_overlap(mb, pb):
                            clash = True; break
                    if clash:
                        break
                if clash:
                    continue
                d = math.hypot(x - ox, y - oy)
                if best is None or d < best[0]:
                    best = (d, x, y)
        if best is None:
            log(f'6. {ref}: no free spot found'); continue
        d, x, y = best
        # its own stub no longer reaches: rip only this net's local copper
        n = 0
        for t in list(bd.GetTracks()):
            if t.GetNetname() not in own:
                continue
            a, b = track_pts(t)
            if min(math.hypot(a[0] - ox, a[1] - oy),
                   math.hypot(b[0] - ox, b[1] - oy)) < 1.2:
                bd.Delete(t); n += 1
        fp.SetPosition(vec(x, y))
        for k, (oref, olay, a0, b0, a1, b1) in enumerate(occupied):
            if oref == ref and olay == lay:
                occupied[k] = (oref, olay, x - w, y - h, x + w, y + h)
        log(f'6. moved {ref} ({ox:.3f},{oy:.3f}) -> ({x:.3f},{y:.3f}) '
            f'd={d:.2f}mm, ripped {n} stub segs')

    # 7. rip whatever copper the new pads now sit on ---------------------
    boxes = new_pad_boxes(placed)
    n = 0
    for t in list(bd.GetTracks()):
        a, b = track_pts(t)
        tl = None if t.Type() == pcbnew.PCB_VIA_T else t.GetLayer()
        hw = 0.0 if t.Type() == pcbnew.PCB_VIA_T else t.GetWidth() / 2 * NM
        for x0, y0, x1, y1, nname, onf, onb in boxes:
            if t.GetNetname() == nname:
                continue
            if tl is not None:
                if tl == pcbnew.F_Cu and not onf:
                    continue
                if tl == pcbnew.B_Cu and not onb:
                    continue
                if tl not in (pcbnew.F_Cu, pcbnew.B_Cu):
                    continue
            if (max(a[0], b[0]) + hw < x0 or min(a[0], b[0]) - hw > x1 or
                    max(a[1], b[1]) + hw < y0 or min(a[1], b[1]) - hw > y1):
                continue
            bd.Delete(t); n += 1
            break
    log(f'7. ripped {n} further segments crossing the new pads')

    # 8. verify against real pad geometry -------------------------------
    touched = {s['ref'] for s in SWAPS} | {s['ref'] for s in ADDS} | set(TP_RELOCATE)
    conflicts = verify(bd, touched, log)

    # 9. refill the pours so DRC sees the copper that will actually exist
    with quiet_stderr():
        pcbnew.ZONE_FILLER(bd).Fill(bd.Zones())
    log(f'9. refilled {len(list(bd.Zones()))} zones')

    if conflicts and not args.force:
        log('REFUSING to write: pad conflicts above. Adjust the spec '
            'coordinates, or pass --force to write anyway.')
        with open(os.path.join(HERE, 'sync_log.txt'), 'w') as f:
            f.write('\n'.join(lines) + '\n')
        return 1

    if args.apply:
        with quiet_stderr():
            pcbnew.SaveBoard(args.out, bd)
        log(f'saved {args.out}')
    else:
        log('dry run — pass --apply to write')
    with open(os.path.join(HERE, 'sync_log.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')


if __name__ == '__main__':
    sys.exit(main() or 0)
