#!/usr/bin/env python3
"""make_fixture.py -- build the passing fixture project the check tests start from.

    python3 make_fixture.py            writes fixtures/blinky/

A three-part board (connector, resistor, LED) that passes sync_check,
nets_check --rules and silk_check. Each failing case in test_checks.py is
this project with one thing broken, so every rule has a pass and a fail.
"""
import json
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import pcbnew  # noqa: E402
from kicad_write import Schematic, update_pcb  # noqa: E402

OUT = HERE / "fixtures" / "blinky"
HW = OUT / "hw" / "blinky" / "a"


def schematic(path: Path):
    s = Schematic("blinky", title="BLINKY", rev="A", date="2026-10-22")
    # Power in on the left, LED on the right, ground along the bottom.
    j1 = s.symbol("Connector_Generic:Conn_01x02", "J1", "VIN", (63.5, 76.2), mirror="y",
                  footprint="Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
                  in_bom=False, ref_at=(60.96, 73.66), value_at=(60.96, 81.28), justify="right")
    r1 = s.symbol("Device:R", "R1", "1k", (83.82, 76.2), rot=90,
                  footprint="Resistor_SMD:R_0603_1608Metric", fields={"LCSC": "C21190"},
                  ref_at=(83.82, 72.39), value_at=(83.82, 80.01), justify="")
    d1 = s.symbol("Device:LED", "D1", "RED", (99.06, 76.2), rot=180,
                  footprint="LED_SMD:LED_0603_1608Metric", fields={"LCSC": "C2286"},
                  ref_at=(99.06, 72.39), value_at=(99.06, 80.01), justify="")
    vin = j1.pin("1")
    s.wire(vin, r1.pin("1"))
    s.label("VIN", (72.39, vin[1]))
    s.wire(r1.pin("2"), d1.pin("2"))
    s.label("LED_A", (90.17, vin[1]))
    k = d1.pin("1")
    gnd_y = 88.9
    s.wire(k, (k[0] + 3.81, k[1]), (k[0] + 3.81, gnd_y))
    s.wire(j1.pin("2"), (71.12, j1.pin("2")[1]), (71.12, gnd_y))
    s.wire((71.12, gnd_y), (k[0] + 3.81, gnd_y))
    s.power("power:GND", (86.36, gnd_y))
    s.power("power:PWR_FLAG", (76.2, gnd_y), rot=180, value="PWR_FLAG")
    s.power("power:PWR_FLAG", (76.2, vin[1]), value="PWR_FLAG")
    s.save(path)


def project(path: Path):
    path.write_text(json.dumps({
        "board": {"design_settings": {"rule_severities": {
            "silk_overlap": "error", "silk_over_copper": "error", "silk_edge_clearance": "error"}}},
        "net_settings": {
            "classes": [
                {"name": "Default", "clearance": 0.15, "track_width": 0.2, "via_diameter": 0.6,
                 "via_drill": 0.3, "priority": 2147483647},
                {"name": "POWER", "clearance": 0.15, "track_width": 0.4, "via_diameter": 0.6,
                 "via_drill": 0.3, "priority": 0},
            ],
            "netclass_patterns": [{"netclass": "POWER", "pattern": "/VIN"},
                                  {"netclass": "POWER", "pattern": "GND"}],
        },
        "meta": {"filename": path.name, "version": 3},
    }, indent=2) + "\n")


def board(pcb: Path):
    b = pcbnew.LoadBoard(str(pcb))
    mm = pcbnew.VECTOR2I_MM
    x0, y0, w, h = 100.0, 100.0, 30.0, 16.0
    corners = [(x0, y0), (x0 + w, y0), (x0 + w, y0 + h), (x0, y0 + h)]
    for a, c in zip(corners, corners[1:] + corners[:1]):
        e = pcbnew.PCB_SHAPE(b, pcbnew.SHAPE_T_SEGMENT)
        e.SetStart(mm(*a))
        e.SetEnd(mm(*c))
        e.SetLayer(pcbnew.Edge_Cuts)
        e.SetWidth(pcbnew.FromMM(0.1))
        b.Add(e)
    fps = {fp.GetReference(): fp for fp in b.GetFootprints()}
    fps["J1"].SetPosition(mm(x0 + 4, y0 + 7))
    fps["R1"].SetPosition(mm(x0 + 14, y0 + 8))
    fps["D1"].SetPosition(mm(x0 + 22, y0 + 8))
    fps["J1"].Reference().SetPosition(mm(x0 + 4, y0 + 2.5))
    fps["R1"].Reference().SetPosition(mm(x0 + 14, y0 + 6.4))
    fps["D1"].Reference().SetPosition(mm(x0 + 22, y0 + 6.4))
    for fp in fps.values():
        fp.Value().SetVisible(False)
    for txt, at in [("+ VIN 5V\n- GND", (x0 + 8.5, y0 + 8.3)), ("BLINKY rev A 2026-10", (x0 + 15, y0 + 13.5))]:
        t = pcbnew.PCB_TEXT(b)
        t.SetText(txt)
        t.SetLayer(pcbnew.F_SilkS)
        t.SetPosition(mm(*at))
        t.SetTextSize(pcbnew.VECTOR2I_MM(0.8, 0.8))
        t.SetTextThickness(pcbnew.FromMM(0.15))
        b.Add(t)
    pcbnew.SaveBoard(str(pcb), b)


def main():
    if OUT.exists():
        shutil.rmtree(OUT)
    HW.mkdir(parents=True)
    (OUT / "design").mkdir()
    (OUT / "design" / "nets.csv").write_text(
        "net,class,v_max,i_cont,i_peak,impedance,length_limit,reason\n"
        "VIN,POWER,5.5,0.02,0.05,,,\"Input supply, LED current only\"\n"
        "GND,POWER,0,0.02,0.05,,,\"Return\"\n"
        "LED_A,SIGNAL,5.5,,,,,\"Resistor to LED anode\"\n")
    (OUT / "design" / "checks.toml").write_text('[silk]\nidentity = ["name", "rev", "date"]\n')
    schematic(HW / "blinky.kicad_sch")
    project(HW / "blinky.kicad_pro")
    update_pcb(HW / "blinky.kicad_sch", HW / "blinky.kicad_pcb", new_parts_at=(110, 110), log=lambda *_: None)
    board(HW / "blinky.kicad_pcb")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
