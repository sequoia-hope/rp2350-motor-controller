#!/usr/bin/env python3
"""Generate the rev-b-lean library footprints into hardware/parts (Tier A of
review/shrink.html):

  *_JLC        KiCad-library pads, courtyard shrunk to half of JLCPCB's SMT
               spacing minimum (0402: 0.18 mm -> 0.09 margin; 0603/0805 and
               SC-70: 0.25 mm -> 0.125 margin).  A4 in the report.
  SC-70 _JLC   true-shape courtyard (union of pads + body, inflated), one
               footprint for all 14 TS5A3159DCK.  A3.
  TestPoint_Pad_D1.0mm_no_courtyard   KiCad D1.0 probe pad, courtyard removed. A2.
  TestPoint_Pad_D1.5mm_no_courtyard   the board-only footprint TP1-TP10 already
               use, saved into the library so the link resolves.
  LED_WS2812B-2020_PLCC4_2.0x2.0mm_MotorBuck   KiCad's 2020 footprint with pads
               renumbered to this project's MotorBuck:WS2812B symbol
               (1 GND, 2 DIN, 3 VDD, 4 DOUT; KiCad's symbol is 1 DOUT, 2 VSS,
               3 DIN, 4 VDD).  A5.

Idempotent: re-running overwrites the same files.
  usage: make_footprints.py [--board BOARD]   (a board still carrying the rev-A footprints)
"""
import os, sys, argparse

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(HW, 'tools', 'jiggle2'))
from common import load_board, NM, quiet_stderr        # noqa: E402
import pcbnew                                          # noqa: E402

KLIB = '/usr/share/kicad/footprints'
PARTS = os.path.join(HW, 'parts')
BOARD = os.path.join(HW, 'rp2350_driver.kicad_pcb')
IO = pcbnew.PCB_IO_KICAD_SEXPR()

CHIPS = [  # (lib, name, courtyard margin mm)
    ('Resistor_SMD', 'R_0402_1005Metric', 0.09),
    ('Capacitor_SMD', 'C_0402_1005Metric', 0.09),
    ('Resistor_SMD', 'R_0603_1608Metric', 0.125),
    ('Capacitor_SMD', 'C_0603_1608Metric', 0.125),
    ('Resistor_SMD', 'R_0805_2012Metric', 0.125),
    ('Capacitor_SMD', 'C_0805_2012Metric', 0.125),
]
SC70_MARGIN = 0.125
LED_RENUMBER = {'1': '4', '2': '1', '3': '2', '4': '3'}   # KiCad pad -> MotorBuck pin


def nm(v):
    return int(round(v / NM))


def load(lib, name):
    with quiet_stderr():
        fp = pcbnew.FootprintLoad(f'{KLIB}/{lib}.pretty', name)
    if fp is None:
        raise SystemExit(f'cannot load {lib}:{name}')
    return fp


def save(fp, name, descr_suffix):
    fp.SetFPID(pcbnew.LIB_ID('parts', name))
    fp.SetLibDescription((fp.GetLibDescription() + ' ' + descr_suffix).strip())
    with quiet_stderr():
        IO.FootprintSave(PARTS, fp)
    return name


def pads_extent(fp):
    x0 = y0 = 1e9; x1 = y1 = -1e9
    for p in fp.Pads():
        b = p.GetBoundingBox()
        x0 = min(x0, b.GetLeft() * NM); y0 = min(y0, b.GetTop() * NM)
        x1 = max(x1, b.GetRight() * NM); y1 = max(y1, b.GetBottom() * NM)
    return x0, y0, x1, y1


def courtyard_items(fp):
    return [g for g in fp.GraphicalItems()
            if g.GetLayer() in (pcbnew.F_CrtYd, pcbnew.B_CrtYd)]


def drop_courtyard(fp):
    for g in courtyard_items(fp):
        fp.Delete(g)


def rect_poly(x0, y0, x1, y1):
    ps = pcbnew.SHAPE_POLY_SET()
    ps.NewOutline()
    for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
        ps.Append(nm(x), nm(y))
    return ps


def poly_add(acc, other):
    """Boolean union; the polygon-mode argument went away in KiCad 9."""
    try:
        acc.BooleanAdd(other)
    except TypeError:
        acc.BooleanAdd(other, pcbnew.SHAPE_POLY_SET.PM_FAST)


def board_copy(src):
    """A library-style copy of a board footprint: origin, rot 0, front, no nets."""
    fp = pcbnew.FOOTPRINT(src)
    if fp.IsFlipped():
        fp.Flip(fp.GetPosition(), False)
    fp.SetOrientationDegrees(0)
    fp.SetPosition(pcbnew.VECTOR2I(0, 0))
    fp.SetReference('REF**')
    fp.SetPath(pcbnew.KIID_PATH())
    for p in fp.Pads():
        p.SetNetCode(0)
    return fp


def add_poly_courtyard(fp, poly):
    sh = pcbnew.PCB_SHAPE(fp)
    sh.SetShape(pcbnew.SHAPE_T_POLY)
    sh.SetPolyShape(poly)
    sh.SetLayer(pcbnew.F_CrtYd)
    sh.SetWidth(nm(0.05))
    sh.SetFilled(False)
    fp.Add(sh)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--board', default=BOARD)
    a = ap.parse_args()
    made = []

    # Chip passives: take the pads exactly as the shipped board carries them
    # (a KiCad-9 library reload would move pad edges by a few hundredths of a
    # millimetre and strand copper that was routed to the clearance limit);
    # only the courtyard rectangle is rebuilt.
    bd = load_board(a.board)
    for lib, name, m in CHIPS:
        fpid = f'{lib}:{name}'
        src = next((f for f in bd.GetFootprints() if f.GetFPIDAsString() == fpid), None)
        if src is None:
            raise SystemExit(f'{fpid}: no instance on {a.board} to copy pads from')
        fp = board_copy(src)
        x0, y0, x1, y1 = pads_extent(fp)
        for g in fp.GraphicalItems():
            if g.GetClass() == 'PCB_SHAPE' and g.GetLayer() == pcbnew.F_Fab and g.GetShapeStr() == 'Rect':
                b = g.GetBoundingBox()
                x0 = min(x0, b.GetLeft() * NM); y0 = min(y0, b.GetTop() * NM)
                x1 = max(x1, b.GetRight() * NM); y1 = max(y1, b.GetBottom() * NM)
        drop_courtyard(fp)
        sh = pcbnew.PCB_SHAPE(fp)
        sh.SetShape(pcbnew.SHAPE_T_RECT)
        sh.SetStart(pcbnew.VECTOR2I(nm(x0 - m), nm(y0 - m)))
        sh.SetEnd(pcbnew.VECTOR2I(nm(x1 + m), nm(y1 + m)))
        sh.SetLayer(pcbnew.F_CrtYd); sh.SetWidth(nm(0.05)); sh.SetFilled(False)
        fp.Add(sh)
        fp.SetLibDescription(f'{name} as shipped on rev A')
        made.append(save(fp, f'{name}_JLC',
                         f'(courtyard = pads+body +{m} mm, JLCPCB SMT spacing)'))

    # SC-70: the shipped board's pads (0.65 x 0.40 at +-0.95, fab-proven and
    # 0.34 mm shorter than KiCad 9's hand-solder pads) with a true-shape
    # courtyard = pads + body, inflated.  Built from the board copy so the ten
    # existing parts swap with zero pad change and U30-U33 match them.
    src = next(f for f in bd.GetFootprints()
               if f.GetFPIDAsString() == 'Package_TO_SOT_SMD:SOT-363_SC-70-6')
    fp = board_copy(src); fp.SetValue('TS5A3159DCK')
    poly = pcbnew.SHAPE_POLY_SET()
    for p in fp.Pads():
        b = p.GetBoundingBox()
        poly_add(poly, rect_poly(b.GetLeft() * NM - SC70_MARGIN, b.GetTop() * NM - SC70_MARGIN,
                                 b.GetRight() * NM + SC70_MARGIN, b.GetBottom() * NM + SC70_MARGIN))
    bx0 = by0 = 1e9; bx1 = by1 = -1e9
    for g in fp.GraphicalItems():
        if g.GetClass() == 'PCB_SHAPE' and g.GetLayer() == pcbnew.F_Fab:
            b = g.GetBoundingBox()
            bx0 = min(bx0, b.GetLeft() * NM); by0 = min(by0, b.GetTop() * NM)
            bx1 = max(bx1, b.GetRight() * NM); by1 = max(by1, b.GetBottom() * NM)
    if bx0 < bx1:
        poly_add(poly, rect_poly(bx0 - SC70_MARGIN, by0 - SC70_MARGIN, bx1 + SC70_MARGIN, by1 + SC70_MARGIN))
    poly.Simplify()
    drop_courtyard(fp)
    add_poly_courtyard(fp, poly)
    fp.SetLibDescription('SC-70-6 as shipped on rev A')
    made.append(save(fp, 'SOT-363_SC-70-6_JLC',
                     f'(true-shape courtyard = pads+body +{SC70_MARGIN} mm)'))

    fp = load('TestPoint', 'TestPoint_Pad_D1.0mm')
    drop_courtyard(fp)
    made.append(save(fp, 'TestPoint_Pad_D1.0mm_no_courtyard', '(no courtyard)'))

    src = next(f for f in bd.GetFootprints()
               if f.GetFPIDAsString() == 'parts:TestPoint_Pad_D1.5mm_no_courtyard')
    fp = board_copy(src); fp.SetValue('TestPoint')
    drop_courtyard(fp)
    fp.SetLibDescription('1.5 mm probe pad, no courtyard')
    made.append(save(fp, 'TestPoint_Pad_D1.5mm_no_courtyard', ''))

    fp = load('LED_SMD', 'LED_WS2812B-2020_PLCC4_2.0x2.0mm')
    for p in fp.Pads():
        p.SetNumber(LED_RENUMBER[p.GetNumber()])
    made.append(save(fp, 'LED_WS2812B-2020_PLCC4_2.0x2.0mm_MotorBuck',
                     '(pads renumbered for MotorBuck:WS2812B: 1 GND 2 DIN 3 VDD 4 DOUT)'))

    print('made:')
    for name in made:
        with quiet_stderr():
            fp = pcbnew.FootprintLoad(PARTS, name)
        cy = fp.GetCourtyard(pcbnew.F_CrtYd)
        pads = ' '.join(f'{p.GetNumber()}@({p.GetPosition().x*NM:+.3f},{p.GetPosition().y*NM:+.3f})' for p in fp.Pads())
        print(f'  {name:48s} courtyard {cy.Area()*NM*NM:5.2f} mm2  pads {pads}')


if __name__ == '__main__':
    main()
