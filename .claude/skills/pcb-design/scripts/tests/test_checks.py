#!/usr/bin/env python3
"""test_checks.py -- every check rule against the blinky fixture, passing and broken.

    python3 test_checks.py            (python3 make_fixture.py first, once)

Each test copies fixtures/blinky/ to a scratch folder, breaks one thing, runs
the check with --json and asserts which rule fired.
"""
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCRIPTS = HERE.parent
FIXTURE = HERE / "fixtures" / "blinky"
sys.path.insert(0, str(SCRIPTS))


class Case(unittest.TestCase):
    def setUp(self):
        if not FIXTURE.exists():
            self.skipTest("run make_fixture.py first")
        self.tmp = Path(tempfile.mkdtemp(prefix="checks-"))
        shutil.copytree(FIXTURE, self.tmp / "p")
        self.root = self.tmp / "p"
        self.hw = self.root / "hw" / "blinky" / "a"
        self.pcb = self.hw / "blinky.kicad_pcb"
        self.sch = self.hw / "blinky.kicad_sch"

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def run_check(self, script, *args):
        out = self.tmp / "out.json"
        r = subprocess.run([sys.executable, str(SCRIPTS / script), str(self.hw), "--json", str(out), *args],
                           capture_output=True, text=True)
        self.assertTrue(out.exists(), r.stdout + r.stderr)
        res = json.loads(out.read_text())
        self.assertEqual(r.returncode == 0, res["pass"])
        res["stdout"] = r.stdout
        return res

    def rules(self, res, kind="errors"):
        return {e["rule"] for e in res[kind]}

    def assert_pass(self, res):
        self.assertTrue(res["pass"], res["stdout"])

    def assert_fails(self, res, rule):
        self.assertFalse(res["pass"], res["stdout"])
        self.assertIn(rule, self.rules(res), res["stdout"])

    def edit_board(self, code):
        """Run `code` with b (the board) and f (footprints by ref) in a fresh
        process: pcbnew can't load a second project reliably in one process."""
        prog = ("import pcbnew, sys\n"
                "b = pcbnew.LoadBoard(sys.argv[1])\n"
                "f = {fp.GetReference(): fp for fp in b.GetFootprints()}\n"
                f"{code}\n"
                "pcbnew.SaveBoard(sys.argv[1], b)\n")
        r = subprocess.run([sys.executable, "-c", prog, str(self.pcb)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)

    def replace(self, path, old, new):
        text = path.read_text()
        self.assertIn(old, text)
        path.write_text(text.replace(old, new, 1))


class Sync(Case):
    def test_pass(self):
        self.assert_pass(self.run_check("sync_check.py"))

    def test_value(self):
        self.edit_board('f["R1"].SetValue("2k2")')
        self.assert_fails(self.run_check("sync_check.py"), "value")

    def test_orphan(self):
        self.edit_board('b.Remove(f["D1"])')
        self.assert_fails(self.run_check("sync_check.py"), "orphan")

    def test_footprint(self):
        self.edit_board('f["R1"].SetFPID(pcbnew.LIB_ID("Resistor_SMD", "R_0805_2012Metric"))')
        self.assert_fails(self.run_check("sync_check.py"), "footprint")

    def test_field(self):
        self.edit_board('f["D1"].GetFieldByName("LCSC").SetText("C999")')
        self.assert_fails(self.run_check("sync_check.py"), "field")

    def test_dnp(self):
        self.edit_board('f["R1"].SetDNP(True)')
        self.assert_fails(self.run_check("sync_check.py"), "dnp")

    def test_missing_lcsc(self):
        self.replace(self.sch, '"LCSC" "C2286"', '"LCSC" ""')
        self.edit_board('f["D1"].GetFieldByName("LCSC").SetText("")')
        self.assert_fails(self.run_check("sync_check.py"), "missing-field")

    def bom(self, rows):
        p = self.tmp / "bom.csv"
        p.write_text("Comment,Designator,Footprint,LCSC\n" + "".join(r + "\n" for r in rows))
        return p

    def test_bom_pass(self):
        bom = self.bom(['1k,R1,R_0603_1608Metric,C21190', 'RED,D1,LED_0603_1608Metric,C2286'])
        self.assert_pass(self.run_check("sync_check.py", "--bom", str(bom)))

    def test_bom_missing_part(self):
        bom = self.bom(['1k,R1,R_0603_1608Metric,C21190'])
        self.assert_fails(self.run_check("sync_check.py", "--bom", str(bom)), "bom")

    def test_bom_lists_unbuilt_part(self):
        bom = self.bom(['1k,R1,R_0603_1608Metric,C21190', 'RED,D1,LED_0603_1608Metric,C2286', 'VIN,J1,,'])
        self.assert_fails(self.run_check("sync_check.py", "--bom", str(bom)), "bom")

    def test_bom_duplicate(self):
        bom = self.bom(['1k,R1,R_0603_1608Metric,C21190', 'RED,D1,LED_0603_1608Metric,C2286',
                        '1k,R1,R_0603_1608Metric,C21190'])
        self.assert_fails(self.run_check("sync_check.py", "--bom", str(bom)), "bom")

    def test_bom_short_row_and_quotes(self):
        p = self.tmp / "bom.csv"
        p.write_text('Comment,Designator,Footprint,LCSC\n"1k","R1","R_0603 ""small""",C21190\nRED,D1\n')
        res = self.run_check("sync_check.py", "--bom", str(p))
        self.assert_fails(res, "bom")   # D1 has no LCSC on its short row
        self.assertEqual(sum(e["rule"] == "bom" for e in res["errors"]), 1, res["stdout"])

    def test_bom_ref_range(self):
        from sync_check import split_refs
        self.assertEqual(split_refs("R1-R3, C5 C7-9"), ["R1", "R2", "R3", "C5", "C7", "C8", "C9"])

    def test_bom_without_lcsc_column(self):
        bom = self.tmp / "bom.csv"
        bom.write_text("Comment,Designator\n1k,R1\nRED,D1\n")
        self.assert_fails(self.run_check("sync_check.py", "--bom", str(bom)), "bom")

    def test_cpl_one_column(self):
        p = self.tmp / "cpl.csv"
        p.write_text("Designator\nR1\nD1\n")
        self.assert_pass(self.run_check("sync_check.py", "--cpl", str(p)))

    def test_cpl(self):
        p = self.tmp / "cpl.csv"
        p.write_text("Designator,Mid X,Mid Y,Layer,Rotation\nR1,14,8,Top,0\nD1,22,8,Top,0\n")
        self.assert_pass(self.run_check("sync_check.py", "--cpl", str(p)))
        p.write_text("Designator,Mid X,Mid Y,Layer,Rotation\nR1,14,8,Bottom,0\n")
        res = self.run_check("sync_check.py", "--cpl", str(p))
        self.assert_fails(res, "cpl")
        self.assertEqual(sum(e["rule"] == "cpl" for e in res["errors"]), 2, res["stdout"])  # side + missing D1


class Writer(Case):
    """kicad_write: pins land where KiCad puts them; update_pcb keeps what it should."""

    def test_pins_every_rotation_and_mirror(self):
        prog = """
import re, subprocess, sys
sys.path.insert(0, sys.argv[1])
from kicad_write import Schematic
bad = []
for sym in ["Transistor_BJT:BC547", "Device:D", "Regulator_Linear:AMS1117-3.3"]:
    for rot in (0, 90, 180, 270):
        for m in (None, "x", "y"):
            s = Schematic("m")
            q = s.symbol(sym, "U1", "x", (100, 100), rot=rot, mirror=m)
            for n, pt in q.pins().items():
                s.label(f"P{n}", pt)
            s.save(sys.argv[2] + "/m.kicad_sch")
            subprocess.run(["kicad-cli", "sch", "export", "netlist", "--format", "kicadxml", "-o",
                            sys.argv[2] + "/m.xml", sys.argv[2] + "/m.kicad_sch"], capture_output=True)
            x = open(sys.argv[2] + "/m.xml").read()
            got = {n: re.findall(r'pin="(\\w+)"', b) for n, b in
                   re.findall(r'<net code="\\d+" name="/?P(\\w+)"[^>]*>(.*?)</net>', x, re.S)}
            if got != {n: [n] for n in q.pins()}:
                bad.append((sym, rot, m, got))
print(bad)
sys.exit(1 if bad else 0)
"""
        r = subprocess.run([sys.executable, "-c", prog, str(SCRIPTS), str(self.tmp)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)

    def update(self):
        prog = ("import sys; sys.path.insert(0, sys.argv[1]); from kicad_write import update_pcb; "
                "update_pcb(sys.argv[2], sys.argv[3], log=lambda *a: None)")
        r = subprocess.run([sys.executable, "-c", prog, str(SCRIPTS), str(self.sch), str(self.pcb)],
                           capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)

    def probe(self, expr):
        r = subprocess.run([sys.executable, "-c", "import pcbnew, sys; b = pcbnew.LoadBoard(sys.argv[1]); "
                            "f = {fp.GetReference(): fp for fp in b.GetFootprints()}; print(" + expr + ")",
                            str(self.pcb)], capture_output=True, text=True)
        return r.stdout.strip().splitlines()[-1]

    def test_update_is_idempotent(self):
        self.update()
        self.assert_pass(self.run_check("sync_check.py"))
        self.assertEqual(self.probe("f['R1'].GetPosition()"), "(114000000, 108000000)")

    def test_board_only_part_keeps_net(self):
        self.edit_board("h = pcbnew.FootprintLoad('/usr/share/kicad/footprints/MountingHole.pretty', "
                        "'MountingHole_2.2mm_M2_Pad'); h.SetFPID(pcbnew.LIB_ID('MountingHole', "
                        "'MountingHole_2.2mm_M2_Pad')); h.SetReference('H1'); h.SetBoardOnly(True); b.Add(h)\n"
                        "h.SetPosition(pcbnew.VECTOR2I_MM(127, 104)); [p.SetNet(b.FindNet('GND')) for p in h.Pads()]")
        self.update()
        self.assertEqual(self.probe("f['H1'].Pads()[0].GetNetname()"), "GND")

    def test_bottom_side_rotation_kept_on_footprint_change(self):
        self.edit_board("f['R1'].Flip(f['R1'].GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)\n"
                        "f['R1'].SetOrientationDegrees(-30)\n"
                        "f['R1'].SetFPID(pcbnew.LIB_ID('Resistor_SMD', 'R_0805_2012Metric'))")
        self.update()
        self.assertEqual(self.probe("(f['R1'].GetFPIDAsString(), f['R1'].IsFlipped(), "
                                    "round(f['R1'].GetOrientationDegrees()))"),
                         "('Resistor_SMD:R_0603_1608Metric', True, -30)")


class Nets(Case):
    def csv(self):
        return self.root / "design" / "nets.csv"

    def test_pass(self):
        self.assert_pass(self.run_check("nets_check.py", "--rules"))

    def test_missing_row(self):
        self.replace(self.csv(), 'LED_A,SIGNAL,5.5,,,,,"Resistor to LED anode"\n', "")
        self.assert_fails(self.run_check("nets_check.py"), "coverage")

    def test_extra_row(self):
        self.csv().write_text(self.csv().read_text() + 'BOGUS,SIGNAL,3.3,,,,,"not a net"\n')
        self.assert_fails(self.run_check("nets_check.py"), "coverage")

    def test_unnamed_net(self):
        # Without its label LED_A is a net KiCad names itself; it still needs a row.
        sch = self.root / "hw" / "blinky" / "a" / "blinky.kicad_sch"
        text = sch.read_text()
        start = text.index('\t(label "LED_A"')
        sch.write_text(text[:start] + text[text.index("\n\t)\n", start) + 4:])
        self.replace(self.csv(), 'LED_A,SIGNAL,5.5,,,,,"Resistor to LED anode"\n', "")
        self.assert_fails(self.run_check("nets_check.py"), "coverage")
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from kicad_files import load_schematic
        auto = [n.name for n in load_schematic(sch).nets if n.name.startswith("Net-(") and len(n.nodes) > 1]
        self.csv().write_text(self.csv().read_text() + f'{auto[0]},SIGNAL,5.5,,,,,"Resistor to LED anode"\n')
        self.assert_pass(self.run_check("nets_check.py"))

    def test_unknown_class(self):
        self.replace(self.csv(), "LED_A,SIGNAL", "LED_A,WIGGLY")
        self.assert_fails(self.run_check("nets_check.py"), "class")

    def test_routed_length(self):
        self.replace(self.csv(), 'LED_A,SIGNAL,5.5,,,,,"Resistor to LED anode"',
                     'LED_A,SIGNAL,5.5,,,,2,"Resistor to LED anode, no vias"')
        self.edit_board("n = b.FindNet('/LED_A') or b.FindNet('LED_A')\n"
                        "t = pcbnew.PCB_TRACK(b); t.SetStart(pcbnew.VECTOR2I_MM(110, 110))\n"
                        "t.SetEnd(pcbnew.VECTOR2I_MM(113, 110)); t.SetNet(n); b.Add(t)\n"
                        "v = pcbnew.PCB_VIA(b); v.SetPosition(pcbnew.VECTOR2I_MM(113, 110)); v.SetNet(n); b.Add(v)")
        res = self.run_check("nets_check.py", "--routed")
        self.assert_fails(res, "length")
        self.assertIn("vias", self.rules(res))
        # the derived table still prints, with VIN's derived row, not the routed one
        out = res["stdout"]
        self.assertIn("derived (mm;", out)
        self.assertIn("POWER", next(l for l in out.split("derived (mm;")[1].splitlines() if l.strip().startswith("VIN")))

    def test_no_reason(self):
        self.replace(self.csv(), '"Resistor to LED anode"', "")
        self.assert_fails(self.run_check("nets_check.py"), "reason")

    def test_power_needs_current(self):
        self.replace(self.csv(), "VIN,POWER,5.5,0.02", "VIN,POWER,5.5,")
        self.assert_fails(self.run_check("nets_check.py"), "i_cont")

    def test_netclass_mismatch(self):
        self.replace(self.csv(), "LED_A,SIGNAL", "LED_A,ANALOG")
        self.assertIn("netclass", self.rules(self.run_check("nets_check.py"), "warnings"))
        self.assert_fails(self.run_check("nets_check.py", "--rules"), "netclass")

    def test_width_from_current(self):
        self.replace(self.csv(), "VIN,POWER,5.5,0.02,0.05", "VIN,POWER,5.5,3,4")
        res = self.run_check("nets_check.py", "--rules")
        self.assert_fails(res, "width")
        self.assertIn("8 ", next(l for l in res["stdout"].splitlines() if l.strip().startswith("VIN")))  # 4 A x1.5 / 0.8 A

    def test_short_row(self):
        self.csv().write_text(self.csv().read_text() + "BOGUS,SIGNAL,3.3\n")
        self.assert_fails(self.run_check("nets_check.py"), "reason")

    def test_negative_voltage(self):
        self.replace(self.csv(), "VIN,POWER,5.5", "VIN,POWER,-48")
        self.assert_fails(self.run_check("nets_check.py", "--rules"), "clearance")

    def test_pattern_must_match_full_name(self):
        pro = self.hw / "blinky.kicad_pro"
        self.replace(pro, '"pattern": "/VIN"', '"pattern": "VIN"')
        self.assert_fails(self.run_check("nets_check.py", "--rules"), "netclass")

    def test_clearance_from_voltage(self):
        self.replace(self.csv(), "VIN,POWER,5.5", "VIN,POWER,48")
        self.assert_fails(self.run_check("nets_check.py", "--rules"), "clearance")


class Style(Case):
    def test_pass(self):
        self.assert_pass(self.run_check("schematic_style_check.py"))

    def test_ground_symbol_rotated(self):
        import re
        text = self.sch.read_text()
        m = re.search(r'\(lib_id "power:GND"\)\s*\(at ([\d.]+) ([\d.]+) 0\)', text)
        self.assertIsNotNone(m)
        self.sch.write_text(text[:m.start()] + m.group(0)[:-2] + "180)" + text[m.end():])
        self.assert_fails(self.run_check("schematic_style_check.py"), "power-symbol")

    def test_generic_value(self):
        self.replace(self.sch, '"Value" "1k"', '"Value" "R"')
        self.assert_fails(self.run_check("schematic_style_check.py"), "fields")

    def test_missing_lcsc(self):
        self.replace(self.sch, '"LCSC" "C21190"', '"LCSC" ""')
        self.assert_fails(self.run_check("schematic_style_check.py"), "fields")


class Placement(Case):
    def test_pass(self):
        self.assert_pass(self.run_check("placement_check.py"))

    def test_decoupler_far_from_pin(self):
        self.edit_board('f["R1"].SetField("Decouples", "D1.2")')      # R1 pad 2 shares LED_A with D1 pad 2, 8 mm away
        self.assert_fails(self.run_check("placement_check.py"), "decoupler")
        (self.root / "design").mkdir(exist_ok=True)
        (self.root / "design" / "checks.toml").write_text("[placement]\ndecoupler_mm = 10.0\n")
        self.assert_pass(self.run_check("placement_check.py"))

    def test_keep_apart(self):
        (self.root / "design").mkdir(exist_ok=True)
        (self.root / "design" / "checks.toml").write_text('[placement]\nkeep_apart = [["R1", "D1", 20.0]]\n')
        self.assert_fails(self.run_check("placement_check.py"), "keep-apart")

    def test_outside(self):
        self.edit_board('f["D1"].SetPosition(pcbnew.VECTOR2I_MM(160, 160))')
        self.assert_fails(self.run_check("placement_check.py"), "outside")


class Silk(Case):
    def test_pass(self):
        self.assert_pass(self.run_check("silk_check.py"))

    def move_ref(self, ref, x, y):
        self.edit_board(f"f[{ref!r}].Reference().SetPosition(pcbnew.VECTOR2I_MM({x}, {y}))")

    def test_designator_on_pads(self):
        self.move_ref("R1", 114, 108)
        res = self.run_check("silk_check.py")
        self.assert_fails(res, "silk_over_copper")
        self.assertIn("under-part", self.rules(res))

    def test_designators_overlap(self):
        self.move_ref("R1", 122, 106.4)
        self.assert_fails(self.run_check("silk_check.py"), "silk_overlap")

    def test_association(self):
        self.move_ref("R1", 19.8 + 100, 106.4)
        self.assert_fails(self.run_check("silk_check.py"), "association")

    def test_hidden(self):
        self.edit_board('f["R1"].Reference().SetVisible(False)')
        self.assert_fails(self.run_check("silk_check.py"), "hidden")

    def test_selective_policy_allows_hidden_resistor(self):
        (self.root / "design" / "checks.toml").write_text(
            '[silk]\nidentity = ["name", "rev", "date"]\ndesignators = "selective"\n')
        self.edit_board('f["R1"].Reference().SetVisible(False)')
        self.assert_pass(self.run_check("silk_check.py"))

    def test_rotations(self):
        (self.root / "design" / "checks.toml").write_text(
            '[silk]\nidentity = ["name", "rev", "date"]\nmax_rotations = 1\n')
        self.edit_board('f["D1"].Reference().SetTextAngleDegrees(90)')
        self.assert_fails(self.run_check("silk_check.py"), "rotation")

    def drop_text(self, startswith):
        self.edit_board("for d in list(b.GetDrawings()):\n"
                        f"    if d.GetClass() == 'PCB_TEXT' and d.GetText().startswith({startswith!r}):\n"
                        "        b.Remove(d)")

    def test_connector_label(self):
        self.drop_text("+ VIN")
        self.assert_fails(self.run_check("silk_check.py"), "connector-label")

    def test_connector_own_text_is_not_a_label(self):
        self.drop_text("+ VIN")
        self.edit_board("t = pcbnew.PCB_TEXT(f['J1']); t.SetText('+'); t.SetLayer(pcbnew.F_SilkS)\n"
                        "t.SetPosition(f['J1'].GetPosition() + pcbnew.VECTOR2I_MM(2, 0)); f['J1'].Add(t)")
        self.assert_fails(self.run_check("silk_check.py"), "connector-label")

    def test_small_board_text(self):
        self.edit_board("[d.SetTextSize(pcbnew.VECTOR2I_MM(0.5, 0.5)) for d in b.GetDrawings() "
                        "if d.GetClass() == 'PCB_TEXT']")
        self.assert_fails(self.run_check("silk_check.py"), "size")

    def test_identity(self):
        self.drop_text("BLINKY")
        res = self.run_check("silk_check.py")
        self.assert_fails(res, "identity")
        self.assertEqual(sum(e["rule"] == "identity" for e in res["errors"]), 3, res["stdout"])

    def test_identity_open_source(self):
        (self.root / "design" / "checks.toml").write_text(
            '[silk]\nidentity = ["name", "rev", "date", "repo", "license"]\n')
        self.assert_fails(self.run_check("silk_check.py"), "identity")

    def test_ignored_rule_still_checked(self):
        pro = self.hw / "blinky.kicad_pro"
        self.replace(pro, '"silk_overlap": "error"', '"silk_overlap": "ignore"')
        self.move_ref("R1", 122, 106.4)
        res = self.run_check("silk_check.py")
        self.assert_fails(res, "silk_overlap")
        self.assertIn("severity", self.rules(res, "warnings"))

    def test_small_text(self):
        self.edit_board('f["R1"].Reference().SetTextSize(pcbnew.VECTOR2I_MM(0.5, 0.5))')
        self.assert_fails(self.run_check("silk_check.py"), "size")

    def test_silk_over_via(self):
        # tented vias pass KiCad's silk_over_copper; the check catches them
        self.edit_board("v = pcbnew.PCB_VIA(b); v.SetPosition(f['R1'].Reference().GetPosition())\n"
                        "v.SetWidth(pcbnew.FromMM(0.5)); v.SetDrill(pcbnew.FromMM(0.2)); b.Add(v)")
        self.assert_fails(self.run_check("silk_check.py"), "silk-over-via")

    def test_pin1_mark(self):
        # trimming an LED's silk takes its cathode mark with it
        self.edit_board("fp = f['D1']\n"
                        "for i in list(fp.GraphicalItems()):\n"
                        "    if i.GetLayer() == pcbnew.F_SilkS and i.GetClass() != 'PCB_TEXT': fp.Remove(i)")
        self.assert_fails(self.run_check("silk_check.py"), "pin1")


if __name__ == "__main__":
    unittest.main(verbosity=2)
