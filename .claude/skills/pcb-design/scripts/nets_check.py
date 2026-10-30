#!/usr/bin/env python3
"""nets_check.py -- design/nets.csv against the schematic, and the rules it implies.

    nets_check.py hw/main/a                  phase 3: coverage, classes, derived table
    nets_check.py hw/main/a --rules          phase 5+: KiCad net classes must match too
    nets_check.py hw/main/a --nets path.csv --json out.json
    nets_check.py hw/main/a --rules --routed  phase 7+: routed lengths too

Checks every schematic net has a row (KiCad's own Net-(...) names too, where
they join two or more pins) and every row names a real net, every row has a
known class and a reason, and unnamed nets don't touch power pins. Then derives, per row, the trace width (IPC-2152, Brooks & Adam fit),
via count and clearance (IPC-2221 Table 6-1) and prints the table for the
gate file. With --rules, each net's KiCad net class must equal its CSV class
and that class's track width and clearance must cover the derived values;
without it those comparisons are warnings, since classes are set in phase 5.
With --routed, every row with a length_limit is measured on the board (all
the net's track copper, as KiCad's net length counts it) and fails over its
limit; a row whose reason says "no vias" fails if the net has any.
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from kicad_files import Report, find_project, load_checks, load_pro, load_schematic, repo_root  # noqa: E402

CLASSES = {"POWER_HI", "POWER", "SWITCH_HI", "SWITCH", "GATE", "ANALOG", "ANALOG_DIFF",
           "CLOCK", "DIFF_HS", "SE_HS", "RF", "HV", "SIGNAL"}
COLUMNS = ["net", "class", "v_max", "i_cont", "i_peak", "impedance", "length_limit", "reason"]

MIL = 0.0254           # mm
OZ_MILS = 1.378        # 1 oz/ft^2 copper is ~35 um

# Brooks & Adam (2015) fit to the IPC-2152 charts, dT = K * I^a * W^b * Th^c
# (W, Th in mils). Same coefficients as KiCad's calculator
# (common/track_width_calculations.cpp); internal fits vary by copper weight.
B_A_EXTERNAL = (215.3, 2.0, -1.15, -1.0)
B_A_INTERNAL = [(0.5, (120.0, 2.0, -1.10, -1.52)), (1.0, (200.0, 1.9, -1.10, -1.52)),
                (2.0, (300.0, 2.0, -1.15, -1.52)), (3.0, (262.5, 1.9, -1.15, -1.52))]

# IPC-2221B Table 6-1, minimum conductor spacing in mm by peak voltage.
V_STEPS = [15, 30, 50, 100, 150, 170, 250, 300, 500]
SPACING = {  # (mm per step, mm/V above 500 V)
    "B1": ([0.05, 0.05, 0.1, 0.1, 0.2, 0.2, 0.2, 0.2, 0.25], 0.0025),   # internal
    "B2": ([0.1, 0.1, 0.6, 0.6, 0.6, 1.25, 1.25, 1.25, 2.5], 0.005),     # external, uncoated
    "B4": ([0.05, 0.05, 0.13, 0.13, 0.4, 0.4, 0.4, 0.4, 0.8], 0.00305),  # external, permanent coating
    "A5": ([0.13, 0.13, 0.13, 0.13, 0.4, 0.4, 0.4, 0.4, 0.8], 0.00305),  # external, conformal coat
}


def width_mm(i: float, oz: float, dt: float, internal: bool) -> float:
    if internal:
        k, a, b, c = min(B_A_INTERNAL, key=lambda w: abs(w[0] - oz))[1]
    else:
        k, a, b, c = B_A_EXTERNAL
    th = oz * OZ_MILS
    return (dt / (k * i ** a * th ** c)) ** (1 / b) * MIL


def via_amps(drill: float, plating: float, dt: float) -> float:
    """Barrel as an internal conductor, IPC-2221 I = 0.024 dT^0.44 A^0.725 (A in mil^2)."""
    area_mil2 = math.pi * (drill + plating) * plating / MIL ** 2
    return 0.024 * dt ** 0.44 * area_mil2 ** 0.725


def spacing_mm(v: float, table: str) -> float:
    """Per net, from its own worst-case |V|. Spacing really depends on the voltage
    between neighbouring nets (-48 V beside +48 V is 96 V): set v_max to the
    largest difference the net can see to anything near it."""
    v = abs(v)
    steps, per_v = SPACING[table]
    for top, mm in zip(V_STEPS, steps):
        if v <= top:
            return mm
    return v * per_v


def num(row: dict, key: str, rep: Report) -> float | None:
    raw = row[key].strip()
    if not raw:
        return None
    try:
        return float(raw)
    except ValueError:
        rep.error("csv", f"{row['net']}: {key} {raw!r} is not a number")
        return None


def norm(name: str) -> str:
    """Root-sheet local labels come out as /NAME; accept NAME in the CSV for those."""
    return name[1:] if name.startswith("/") and name.count("/") == 1 else name


def kicad_classes(net) -> list[str]:
    """KiCad's effective classes, from the netlist (patterns, assignments and
    directive labels already applied). "POWER,Default" means POWER."""
    names = [c for c in net.netclass.split(",") if c] or ["Default"]
    return [c for c in names if c != "Default"] or ["Default"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("project", type=Path, help="variant folder, .kicad_pro, .kicad_sch or .kicad_pcb")
    ap.add_argument("--nets", type=Path, help="net table (default: design/nets.csv)")
    ap.add_argument("--rules", action="store_true", help="phase 5+: KiCad net classes must match")
    ap.add_argument("--routed", action="store_true", help="phase 7+: check length_limit on the board")
    ap.add_argument("--json", help="also write findings as JSON here")
    args = ap.parse_args()

    pro, sch_path, pcb_path = find_project(args.project, need_pcb=args.routed)
    root = repo_root(pro.parent)
    cfg = load_checks(root)
    fab, cu, nc = cfg["fab"], cfg["copper"], cfg["nets"]
    classes = CLASSES | set(nc.get("extra_classes", []))
    nets_path = args.nets or root / "design" / "nets.csv"
    rep = Report("nets_check", str(pro.parent))
    flag = rep.error if args.rules else rep.warn

    if not nets_path.exists():
        sys.exit(f"{nets_path} not found")
    with nets_path.open(newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        missing_cols = [c for c in COLUMNS if c not in (reader.fieldnames or [])]
        rows = [{k: (v or "") for k, v in row.items() if k is not None} for row in reader]
        for row in rows:
            for c in COLUMNS:
                row.setdefault(c, "")
    if missing_cols:
        rep.error("csv", f"{nets_path.name} is missing columns {missing_cols}; header must be {','.join(COLUMNS)}")
        return rep.finish(args.json)

    sch = load_schematic(sch_path)
    named = {n.name: n for n in sch.nets if n.named}
    # /VCC may be written VCC, unless a separate global VCC exists too.
    by_norm = {norm(n): net for n, net in named.items() if norm(n) == n or norm(n) not in named}
    # Nets KiCad named itself (Net-(D1-A)) that join two or more pins: classified
    # too, under KiCad's name, unless the schematic gives them one.
    auto = {n.name: n for n in sch.nets if not n.named and len(n.nodes) > 1}
    table = {}
    for row in rows:
        name = row["net"].strip()
        if name in table:
            rep.error("csv", f"{name} has two rows")
        table[name] = row

    rep.note(f"{len(named)} named nets in {sch_path.name}, {len(table)} rows in {nets_path.name}")

    # Coverage, both directions.
    for name in sorted(named):
        if name not in table and not (by_norm.get(norm(name)) is named[name] and norm(name) in table):
            rep.error("coverage", f"schematic net {name} has no row in {nets_path.name}")
    for name in sorted(auto):
        if name not in table:
            refs = ", ".join(f"{r}.{p}" for r, p, _ in auto[name].nodes)
            rep.error("coverage", f"unnamed net {name} ({refs}) has no row in {nets_path.name}; "
                                  "name it in the schematic, or add a row under this name")
    for name in sorted(table):
        if name not in named and name not in by_norm and name not in auto:
            rep.error("coverage", f"{nets_path.name} row {name} is not a net in the schematic")

    # Unnamed nets on power pins should be named and classified.
    for n in sch.nets:
        if n.named:
            continue
        pins = [f"{r}.{p}" for r, p, t in n.nodes if t.split("+")[0] in ("power_in", "power_out")]
        if pins:
            rep.warn("unnamed-power", f"{n.name} reaches power pin(s) {', '.join(pins)}; "
                                      "give it a name and a row")

    # Per-row content and derived values.
    per_via = via_amps(cu["via_drill_mm"], cu["via_plating_mm"], nc["temp_rise_c"])
    ns = load_pro(pro).get("net_settings", {})
    default = next((c for c in ns.get("classes", []) if c["name"] == "Default"), {})
    kc = {c["name"]: c for c in ns.get("classes", [])}
    lines = [f"{'net':<18} {'class':<11} {'V':>5} {'I':>5} {'Ipk':>5}  {'w_out':>6} {'w_in':>6} "
             f"{'vias':>4} {'clr':>5}  kicad class"]
    for name, row in table.items():
        cls = row["class"].strip()
        if cls not in classes:
            rep.error("class", f"{name}: class {cls!r} is not one of {sorted(classes)}")
        if not row["reason"].strip():
            rep.error("reason", f"{name}: no reason given")
        v, i, ipk = num(row, "v_max", rep), num(row, "i_cont", rep), num(row, "i_peak", rep)
        if v is None:
            rep.error("v_max", f"{name}: v_max is empty (every net has a worst-case voltage)")
        if cls in ("POWER_HI", "POWER", "SWITCH_HI", "SWITCH") and i is None:
            rep.error("i_cont", f"{name}: {cls} net needs i_cont")
        if cls == "DIFF_HS" and not row["impedance"].strip():
            rep.error("impedance", f"{name}: DIFF_HS net needs a target impedance")

        w_out = width_mm(i, cu["outer_oz"], nc["temp_rise_c"], False) if i else None
        w_in = width_mm(i, cu["inner_oz"], nc["temp_rise_c"], True) if i else None
        if w_out is not None:
            w_out, w_in = max(w_out, fab["min_track_mm"]), max(w_in, fab["min_track_mm"])
        vias = math.ceil((ipk or i) * nc["via_safety"] / per_via) if (ipk or i) else 1
        clr = max(spacing_mm(v or 0, nc["clearance_table"]), fab["min_clearance_mm"])

        sch_net = named.get(name) or by_norm.get(name) or auto.get(name)
        assigned = kicad_classes(sch_net) if sch_net else []
        if cls == "SIGNAL" and assigned == ["Default"]:
            assigned = ["SIGNAL"]  # KiCad's Default class is the SIGNAL class
        if sch_net and cls not in assigned:
            flag("netclass", f"{name}: KiCad net class {'/'.join(assigned)}, nets.csv says {cls}")
        k = kc.get(cls) or (kc.get("Default") if cls == "SIGNAL" else None)
        if sch_net and cls in assigned:
            if not k:
                flag("netclass", f"{name}: net class {cls} is not defined in {pro.name}")
            else:
                # KiCad 9 classes may leave a value unset and inherit it from Default.
                tw = k.get("track_width", default.get("track_width"))
                cl = k.get("clearance", default.get("clearance"))
                if w_out and tw is not None and tw + 1e-6 < w_out:
                    flag("width", f"{name}: class {cls} track width {tw} mm < derived {w_out:.3f} mm")
                if cl is not None and cl + 1e-6 < clr:
                    flag("clearance", f"{name}: class {cls} clearance {cl} mm < derived {clr:.3f} mm")

        def f(x, w=5, d=2):
            return f"{x:>{w}.{d}f}" if x is not None else " " * (w - 1) + "-"
        lines.append(f"{name:<18} {cls:<11} {f(v, 5, 1)} {f(i)} {f(ipk)}  {f(w_out, 6, 3)} {f(w_in, 6, 3)} "
                     f"{vias:>4} {clr:>5.2f}  {'/'.join(assigned) or '-'}")

    if args.routed:
        import pcbnew
        board = pcbnew.LoadBoard(str(pcb_path))
        length, vias = {}, {}
        for tr in board.GetTracks():
            n = tr.GetNetname().lstrip("/")
            if tr.Type() == pcbnew.PCB_VIA_T:
                vias[n] = vias.get(n, 0) + 1
            else:
                length[n] = length.get(n, 0.0) + pcbnew.ToMM(tr.GetLength())
        zoned = {z.GetNetname().lstrip("/") for z in board.Zones() if not z.GetIsRuleArea()}
        rlines = [f"{'net':<18} {'routed':>7} {'limit':>6} {'vias':>4}"]
        for name, row in table.items():
            lim = num(row, "length_limit", rep)
            key = norm(name)
            no_via = "no via" in row["reason"].lower()
            if lim is None and not no_via:
                continue
            got, nv = length.get(key, 0.0), vias.get(key, 0)
            rlines.append(f"{name:<18} {got:>7.2f} {lim if lim is not None else '-':>6} {nv:>4}"
                          + ("  (zone copper)" if key in zoned else ""))
            if key not in length and key not in zoned:
                rep.warn("routed", f"{name}: no tracks on the board")
            elif lim is not None and got > lim + 1e-6:
                rep.error("length", f"{name}: routed {got:.2f} mm > length_limit {lim:g} mm")
            if no_via and nv:
                rep.error("vias", f"{name}: {nv} via(s), but its reason says no vias")

    rep.table(f"derived (mm; {cu['outer_oz']:g} oz outer / {cu['inner_oz']:g} oz inner, "
              f"dT {nc['temp_rise_c']:g} C, {cu['via_drill_mm']} mm via = {per_via:.2f} A, "
              f"x{nc['via_safety']:g} safety, clearance IPC-2221 {nc['clearance_table']})", lines)
    if args.routed:
        rep.table("routed length (mm, track copper on the net; zones not counted) against nets.csv", rlines)
    return rep.finish(args.json)


if __name__ == "__main__":
    sys.exit(main())
