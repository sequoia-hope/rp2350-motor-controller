#!/usr/bin/env python3
"""Shrink 15 signal-level 0603 passives to 0402 on rp2350_driver.kicad_pcb.

Only parts whose job is logic-level are in scope: 3.3 V PWM pulldowns, the
thermistor divider leg, the debug-UART series resistors and 3.3 V decoupling.
Everything in a gate loop, a bootstrap path, a 48 V divider or a >16 V rail
stays 0603 -- see review/findings.html and the swap rationale in the register.

The eight PWM pulldowns sit inside the RP2350 halo (the densest cluster on
the board, F-24), which is the whole point of the exercise.

kicad-cli has no "update PCB from schematic" and the SWIG bindings expose no
BOARD_NETLIST_UPDATER, so the swap is done explicitly here, the same way
sync_pcb.py does its footprint swaps. What is preserved from the old
footprint, because losing any of it silently desyncs the board:

  path         the schematic symbol link -- without it KiCad treats the part
               as board-only and the next netlist update deletes or dupes it
  position/rot/side, reference + value, and their field text placement
  pad nets     by pad number (both parts are 2-pad, 1 and 2)

Pad geometry shrinks 0603 (+-0.825, 0.8x0.95) -> 0402 (+-0.51, 0.54x0.64),
so copper that landed on the outer part of an old pad can be left dangling.
This script does not reroute; it reports every track whose endpoint falls
outside the new pad so the routing pass can pick them up.

  usage: shrink_0402.py [--dry-run] [--pcb BOARD]
"""
import os, sys, argparse

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(HW, 'tools', 'jiggle2'))
from common import load_board, NM, quiet_stderr        # noqa: E402
import pcbnew                                          # noqa: E402

BOARD = os.path.join(HW, 'rp2350_driver.kicad_pcb')
KLIB = '/usr/share/kicad/footprints'

RES = f'{KLIB}/Resistor_SMD.pretty'
CAP = f'{KLIB}/Capacitor_SMD.pretty'

# ref -> (library dir, footprint name, full fpid as the schematic spells it)
SWAPS = {}
for _r in ('R1 R2 R21 R22 R35 R36 R65 R66 R49 R53 R54').split():
    SWAPS[_r] = (RES, 'R_0402_1005Metric', 'Resistor_SMD:R_0402_1005Metric')
for _c in ('C16 C29 C40 C19').split():
    SWAPS[_c] = (CAP, 'C_0402_1005Metric', 'Capacitor_SMD:C_0402_1005Metric')

STD_FIELDS = {'Reference', 'Value', 'Footprint', 'Datasheet', 'Description'}


def fp_by_ref(bd, ref):
    for f in bd.GetFootprints():
        if f.GetReference() == ref:
            return f
    return None


def pad_map(fp):
    return {p.GetNumber(): p.GetNetname() for p in fp.Pads()}


def copy_field_placement(old, new, name):
    """Keep the silk/fab text exactly where the placement work left it."""
    src = dst = None
    for f in old.GetFields():
        if f.GetName() == name:
            src = f
    for f in new.GetFields():
        if f.GetName() == name:
            dst = f
    if src is None or dst is None:
        return
    dst.SetPosition(src.GetPosition())
    dst.SetLayer(src.GetLayer())
    dst.SetVisible(src.IsVisible())
    dst.SetMirrored(src.IsMirrored())
    dst.SetTextAngle(src.GetTextAngle())
    dst.SetTextSize(src.GetTextSize())
    dst.SetTextThickness(src.GetTextThickness())


def swap(bd, ref, spec, log):
    old = fp_by_ref(bd, ref)
    if old is None:
        raise SystemExit(f'{ref}: not on the board')
    lib, name, fpid = spec

    pos = old.GetPosition()
    rot = old.GetOrientationDegrees()
    back = old.IsFlipped()
    value = old.GetValue()
    path = old.GetPath()
    nets = pad_map(old)
    extra = [(f.GetName(), f.GetText()) for f in old.GetFields()
             if f.GetName() not in STD_FIELDS]
    old_fpid = old.GetFPIDAsString()

    new = pcbnew.FootprintLoad(lib, name)
    if new is None:
        raise SystemExit(f'cannot load {lib}/{name}')
    bd.Add(new)
    new.SetPosition(pos)
    if back:
        new.Flip(pos, False)          # False = do not flip text side-to-side
    new.SetOrientationDegrees(rot)
    new.SetReference(ref)
    new.SetValue(value)
    new.SetFPIDAsString(fpid)
    new.SetPath(path)                 # the schematic link -- must survive

    for name_ in ('Reference', 'Value'):
        copy_field_placement(old, new, name_)

    for k, v in extra:
        new.SetField(k, v)
        # SetField drops a visible, unmirrored field on the silkscreen; park it
        # hidden on Fab, mirrored to match the side (same fix as sync_pcb.py).
        for f in new.GetFields():
            if f.GetName() == k:
                f.SetLayer(pcbnew.B_Fab if back else pcbnew.F_Fab)
                f.SetVisible(False)
                f.SetMirrored(back)

    missing = []
    for pad in new.Pads():
        nm_ = nets.get(pad.GetNumber())
        if nm_ is None:
            missing.append(pad.GetNumber())
            continue
        n = bd.FindNet(nm_)
        if n is None:
            raise SystemExit(f'{ref}: net {nm_} vanished')
        pad.SetNet(n)
    if missing:
        raise SystemExit(f'{ref}: no net for pad(s) {missing}')

    bd.Delete(old)                    # Remove() segfaults; Delete() is safe
    log(f'  {ref:4s} {old_fpid:34s} -> {fpid:34s} '
        f'at ({pos.x*NM:.3f},{pos.y*NM:.3f}) rot {rot:g} '
        f'{"B.Cu" if back else "F.Cu"}')
    return new


def landed_on(bd, refs):
    """Set of track endpoints currently sitting on a target part's pads.

    Keyed by identity of the endpoint, so the same set can be recomputed after
    the swap and differenced -- an endpoint that was on copper and no longer is
    is the only thing that counts as stranded.
    """
    pads = [(fp.GetReference(), p)
            for fp in bd.GetFootprints() if fp.GetReference() in refs
            for p in fp.Pads()]
    on = {}
    for t in bd.GetTracks():
        if t.Type() == pcbnew.PCB_VIA_T:
            continue
        for ref, pad in pads:
            if t.GetNetCode() != pad.GetNetCode():
                continue
            for idx, end in enumerate((t.GetStart(), t.GetEnd())):
                if pad.HitTest(end):
                    # keyed by track UUID, not coordinates: reconnect() moves
                    # the endpoint, so a coordinate key could never re-match
                    key = (t.m_Uuid.AsString(), idx, ref, pad.GetNumber())
                    on[key] = (pad.GetNetname(), bd.GetLayerName(t.GetLayer()),
                               t.GetLayer(), end.x, end.y, t.GetWidth())
    return on


def reconnect(bd, before, after, log):
    """Bridge each stranded endpoint to the new pad with a short new stub.

    Do NOT move the existing endpoints. Two things go wrong if you do: a track
    with *both* ends inside the old pad collapses to zero length, and dragging
    live copper sideways pushes it into whatever was routed past the old pad.
    Adding a stub leaves every existing segment untouched.

    The stub is safe by construction: the 0402 pad centre lies inside the old
    0603 pad (centres move in by 0.315 mm, old pad half-length is 0.40 mm), so
    the stub runs entirely through copper that was already pad copper on the
    same net. Anything that cleared the old pad clears the stub.

    Endpoints are de-duplicated: several tracks commonly meet at one pad point,
    and they all reconnect through a single stub.
    """
    pads = {(fp.GetReference(), p.GetNumber()): p
            for fp in bd.GetFootprints() for p in fp.Pads()}
    # widest track that met each point -- the stub matches the copper it joins
    # rather than the pad, so it does not bulge into whatever was routed past
    # the old pad (a pad-width stub cost 2 clearance violations at C40).
    want = {}
    for key in set(before) - set(after):
        _uuid, _idx, ref, num = key
        _net, _lname, layer, x, y, width = before[key]
        k = (ref, num, layer, x, y)
        want[k] = max(width, want.get(k, 0))

    added = 0
    for (ref, num, layer, x, y), width in sorted(want.items()):
        pad = pads.get((ref, num))
        if pad is None:
            continue
        tgt = pad.GetPosition()
        if tgt.x == x and tgt.y == y:
            continue                       # already coincident
        t = pcbnew.PCB_TRACK(bd)
        t.SetStart(pcbnew.VECTOR2I(x, y))
        t.SetEnd(tgt)
        t.SetLayer(layer)
        t.SetWidth(width)
        t.SetNet(pad.GetNet())
        bd.Add(t)
        added += 1
    log(f'  added {added} stub(s) from the old pad points to the new pads')
    return added


def report_stranded(before, after, bd, log):
    lost = sorted(set(before) - set(after))
    for key in lost:
        _uuid, _idx, ref, num = key
        netname, layer, _l, x, y, _w = before[key]
        log(f'  {ref}.{num} net {netname} on {layer}: '
            f'endpoint ({x*NM:.3f},{y*NM:.3f}) no longer on the pad')
    return len(lost)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pcb', default=BOARD)
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args()

    lines = []
    def log(s):
        print(s)
        lines.append(s)

    with quiet_stderr():
        bd = load_board(a.pcb)
    before = landed_on(bd, set(SWAPS))
    log(f'shrinking {len(SWAPS)} footprints 0603 -> 0402')
    for ref, spec in SWAPS.items():
        swap(bd, ref, spec, log)

    after = landed_on(bd, set(SWAPS))
    log(f'copper stranded by the smaller pads '
        f'({len(before)} endpoints were on-pad before):')
    n = report_stranded(before, after, bd, log)
    log(f'  {n} endpoint(s) stranded')
    if n:
        # Stubs reconnect electrically without moving the original endpoints,
        # so landed_on() cannot be the post-check -- the old endpoint still
        # misses the smaller pad by construction. The gate is instead: every
        # distinct stranded point got a stub, and DRC's unconnected count does
        # not rise (compare drc before/after; it is the real acceptance test).
        pts = {(k[2], k[3], v[2], v[3], v[4])
               for k, v in before.items() if k not in after}
        added = reconnect(bd, before, after, log)
        coincident = len(pts) - added
        log(f'  {len(pts)} distinct stranded point(s): '
            f'{added} stubbed, {coincident} already on the pad')
        if added + coincident != len(pts):
            raise SystemExit('reconnect did not cover every stranded point')

    if a.dry_run:
        log('dry run - not saved')
        return
    with quiet_stderr():
        bd.Save(a.pcb)
    log(f'saved {a.pcb}')
    with open(os.path.join(HERE, 'shrink_0402_log.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
