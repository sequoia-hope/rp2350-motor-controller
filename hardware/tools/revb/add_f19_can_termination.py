#!/usr/bin/env python3
"""F-19: jumper-selected 120R CAN termination + RC barrier stitch GND<->GND_ISO.

Appends new schematic items to the flat sheet just before the closing paren.
Every net connection is made by landing a pin exactly on a wire endpoint --
no junctions on passing wires (those do not connect in KiCad).
"""
import uuid

SCH = "/home/sequoia/pcb/rp2350-motor-controller/hardware/rp2350_driver.kicad_sch"
SHEET = "/889c04f7-ab14-429c-99d2-ed773c17e58f"

U = lambda: str(uuid.uuid4())


def wire(x1, y1, x2, y2):
    return f"""\t(wire
\t\t(pts
\t\t\t(xy {x1} {y1}) (xy {x2} {y2})
\t\t)
\t\t(stroke
\t\t\t(width 0)
\t\t\t(type default)
\t\t)
\t\t(uuid "{U()}")
\t)
"""


def label(name, x, y, rot=0, justify="left bottom"):
    return f"""\t(label "{name}"
\t\t(at {x} {y} {rot})
\t\t(effects
\t\t\t(font
\t\t\t\t(size 1.27 1.27)
\t\t\t)
\t\t\t(justify {justify})
\t\t)
\t\t(uuid "{U()}")
\t)
"""


def note(text, x, y):
    return f"""\t(text "{text}"
\t\t(exclude_from_sim no)
\t\t(at {x} {y} 0)
\t\t(effects
\t\t\t(font
\t\t\t\t(size 1.27 1.27)
\t\t\t)
\t\t\t(justify left)
\t\t)
\t\t(uuid "{U()}")
\t)
"""


def prop(name, value, x, y, hide=False, extra="", angle=0):
    h = "\n\t\t\t\t(hide yes)" if hide else ""
    j = f"\n\t\t\t\t{extra}" if extra else ""
    return f"""\t\t(property
\t\t\t"{name}"
\t\t\t"{value}"
\t\t\t(at {x} {y} {angle})
\t\t\t(effects
\t\t\t\t(font
\t\t\t\t\t(size 1.27 1.27)
\t\t\t\t){j}{h}
\t\t\t)
\t\t)
"""


def symbol(lib_id, ref, value, fp, x, y, rot, npins, desc="", ref_off=(0, -2.79),
           val_off=(0, 2.79), text_angle=0, hide_value=False):
    # NOTE: property text angle is stored RELATIVE to the symbol -- a rot-90
    # symbol with angle 0 renders its text sideways (cf. R32). Pass
    # text_angle=90 on rotated parts to get horizontal text (cf. R54).
    props = (
        prop("Reference", ref, round(x + ref_off[0], 4), round(y + ref_off[1], 4),
             angle=text_angle)
        + prop("Value", value, round(x + val_off[0], 4), round(y + val_off[1], 4),
               hide=hide_value, angle=text_angle)
        + prop("Footprint", fp, x, y, hide=True)
        + prop("Datasheet", "~", x, y, hide=True)
        + prop("Description", desc, x, y, hide=True)
        + prop("LCSC", "", x, y, hide=True)
    )
    pins = "".join(
        f'\t\t(pin\n\t\t\t"{i + 1}"\n\t\t\t(uuid "{U()}")\n\t\t)\n' for i in range(npins)
    )
    return f"""\t(symbol
\t\t(lib_id "{lib_id}")
\t\t(at {x} {y} {rot})
\t\t(unit 1)
\t\t(exclude_from_sim no)
\t\t(in_bom yes)
\t\t(on_board yes)
\t\t(dnp no)
\t\t(uuid "{U()}")
{props}{pins}\t\t(instances
\t\t\t(project
\t\t\t\t"rp2350_driver"
\t\t\t\t(path
\t\t\t\t\t"{SHEET}"
\t\t\t\t\t(reference "{ref}")
\t\t\t\t\t(unit 1)
\t\t\t\t)
\t\t\t)
\t\t)
\t)
"""


def power(lib_id, ref, value, x, y, rot=0):
    props = (
        prop("Reference", ref, x, round(y + 6.35, 4), hide=True)
        + prop("Value", value, round(x - 5.08, 4), y)
        + prop("Footprint", "", x, y, hide=True)
        + prop("Datasheet", "", x, y, hide=True)
        + prop("Description", "", x, y, hide=True)
    )
    return f"""\t(symbol
\t\t(lib_id "{lib_id}")
\t\t(at {x} {y} {rot})
\t\t(unit 1)
\t\t(exclude_from_sim no)
\t\t(in_bom yes)
\t\t(on_board yes)
\t\t(dnp no)
\t\t(uuid "{U()}")
{props}\t\t(pin
\t\t\t"1"
\t\t\t(uuid "{U()}")
\t\t)
\t\t(instances
\t\t\t(project
\t\t\t\t"rp2350_driver"
\t\t\t\t(path
\t\t\t\t\t"{SHEET}"
\t\t\t\t\t(reference "{ref}")
\t\t\t\t\t(unit 1)
\t\t\t\t)
\t\t\t)
\t\t)
\t)
"""


out = []

# ---------------------------------------------------------------- termination
# CAN0_P -- R114 (120R) -- JP2 (solder jumper, open) -- CAN0_N
out.append(label("CAN0_P", 546.1, 264.16, 0))
out.append(wire(546.1, 264.16, 546.1, 265.43))
out.append(symbol("Device:R", "R114", "120R", "Resistor_SMD:R_0603_1608Metric",
                  546.1, 269.24, 0, 2, "Resistor",
                  ref_off=(2.54, -1.27), val_off=(2.54, 1.27)))
out.append(wire(546.1, 273.05, 546.1, 276.86))
out.append(symbol("Jumper:Jumper_2_Open", "JP2", "Jumper_2_Open",
                  "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm",
                  551.18, 276.86, 0, 2, "Jumper, 2-pole, open",
                  ref_off=(0, -2.794), val_off=(0, 4.064), hide_value=True))
out.append(wire(556.26, 276.86, 556.26, 281.94))
out.append(label("CAN0_N", 556.26, 281.94, 0))

# --------------------------------------------------------------- barrier stitch
# GND_ISO -- C115 (1n 2kV) -- R115 (100R) -- GND
# C on the barrier side so ONE part straddles the isolation gap; R sits
# entirely on the board-GND side.
out.append(power("motor_parts:GND_ISO", "#PWR0374", "GND_ISO", 541.02, 302.26))
out.append(wire(541.02, 302.26, 541.02, 297.18))
out.append(wire(541.02, 297.18, 546.1, 297.18))
out.append(symbol("Device:C", "C115", "1n 2kV", "Capacitor_SMD:C_1206_3216Metric",
                  549.91, 297.18, 90, 2, "Unpolarized capacitor",
                  ref_off=(0, -3.81), val_off=(0, 3.81), text_angle=90))
out.append(wire(553.72, 297.18, 558.8, 297.18))
out.append(symbol("Device:R", "R115", "100R", "Resistor_SMD:R_0603_1608Metric",
                  562.61, 297.18, 90, 2, "Resistor",
                  # value nudged left so "100R" clears the GND symbol's text
                  ref_off=(0, -3.81), val_off=(-2.54, 3.81), text_angle=90))
out.append(wire(566.42, 297.18, 571.5, 297.18))
out.append(wire(571.5, 297.18, 571.5, 302.26))
out.append(power("power:GND", "#PWR0375", "GND", 571.5, 302.26))

# Both explanatory notes live together in the clear area below the CAN block.
out.append(note(
    "F-19 (a): end-of-bus termination. JP2 open = un-terminated, the\\n"
    "shipping default for a mid-bus node. Bridge JP2 to put R114 (120R)\\n"
    "across CANH/CANL when this board sits at a bus end. Termination is\\n"
    "on the isolated side, so it is powered by the harness like U16.",
    487.0, 312.0))
out.append(note(
    "F-19 (b): isolation-barrier stitch. GND_ISO is powered and referenced\\n"
    "ONLY by the CAN harness (J11) -- no board 5V/GND feeds the isolated\\n"
    "side, so isolation depends on the harness supplying +5V_ISO. C115\\n"
    "blocks DC so the barrier holds, while giving common-mode current a\\n"
    "defined return instead of letting it radiate off the harness. R115\\n"
    "damps the C115/trace resonance and limits surge current.\\n"
    "LAYOUT: C115 must be the ONLY part straddling the barrier gap --\\n"
    "keep R115 wholly on the board-GND side of the split.",
    487.0, 328.0))
# NOTE: KiCad centres multi-line text vertically on its anchor, so these two
# blocks are spaced by half-height + half-height + margin, not by line count.

src = open(SCH).read()
assert src.rstrip().endswith(")")
i = src.rstrip().rfind(")")
new = src.rstrip()[:i] + "".join(out) + ")\n"
open(SCH, "w").write(new)
print(f"inserted {len(out)} items; {len(src)} -> {len(new)} bytes")
