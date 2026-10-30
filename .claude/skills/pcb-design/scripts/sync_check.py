#!/usr/bin/env python3
"""sync_check.py -- schematic vs PCB vs BOM/CPL, part by part.

    sync_check.py hw/main/a                          schematic vs PCB
    sync_check.py hw/main/a --bom out/bom.csv --cpl out/cpl.csv
    sync_check.py hw/main/a --json design/gates/sync.json

The schematic is the source of truth. For every reference designator this
checks presence on both sides, value, footprint, every user field (MPN,
Manufacturer, LCSC, ...), DNP and exclude-from-BOM, and that the fields in
design/checks.toml [sync].require_fields are filled in. With --bom/--cpl it
also checks the generated fab files list exactly the parts that get built.
Exit status is non-zero on any mismatch. Stock is not checked here.
"""
from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from kicad_files import Report, find_project, load_board, load_checks, load_schematic, repo_root  # noqa: E402

# Fields KiCad manages itself or that legitimately differ between the two sides.
SKIP_FIELDS = {"Reference", "Value", "Footprint", "Datasheet", "Description",
               "Sheetname", "Sheetfile"}


def natural(ref: str):
    return [int(t) if t.isdigit() else t for t in re.split(r'(\d+)', ref)]


def read_csv(path: Path) -> tuple[list[str], list[dict]]:
    text = path.read_text(encoding="utf-8-sig")
    lines = text.splitlines()
    try:
        delim = csv.Sniffer().sniff(lines[0], delimiters=",;\t").delimiter
    except (csv.Error, IndexError):
        delim = ","
    reader = csv.DictReader(lines, delimiter=delim, doublequote=True)
    rows = [{k: (v or "") for k, v in row.items() if k is not None} for row in reader]
    return list(reader.fieldnames or []), rows


def column(keys, *names) -> str | None:
    low = {k.lower().strip(): k for k in keys}
    for n in names:
        if n in low:
            return low[n]
    return None


def split_refs(cell: str) -> list[str]:
    """R1, R2 R3;R4 and KiCad's ranges R5-R8."""
    out = []
    for tok in re.split(r'[\s,;]+', cell.strip()):
        m = re.fullmatch(r'([A-Za-z_]+)(\d+)-(?:\1)?(\d+)', tok)
        if m:
            out += [f"{m.group(1)}{i}" for i in range(int(m.group(2)), int(m.group(3)) + 1)]
        elif tok:
            out.append(tok)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("project", type=Path, help="variant folder, .kicad_pro, .kicad_sch or .kicad_pcb")
    ap.add_argument("--bom", type=Path, help="generated BOM CSV (JLC or KiCad format)")
    ap.add_argument("--cpl", type=Path, help="generated CPL / position CSV")
    ap.add_argument("--json", help="also write findings as JSON here")
    args = ap.parse_args()

    pro, sch_path, pcb_path = find_project(args.project)
    cfg = load_checks(repo_root(pro.parent)).get("sync", {})
    require = cfg.get("require_fields", ["LCSC"])
    rep = Report("sync_check", str(pro.parent))

    sch = load_schematic(sch_path)
    pcb = load_board(pcb_path)
    s_parts = {r: p for r, p in sch.parts.items() if not p.exclude_from_board}
    p_parts = {r: f for r, f in pcb.footprints.items() if r and not f.board_only}
    rep.note(f"schematic {sch_path.name}: {len(sch.parts)} symbols "
             f"({len(sch.parts) - len(s_parts)} excluded from board)")
    rep.note(f"board {pcb_path.name}: {len(pcb.footprints)} footprints "
             f"({len(pcb.footprints) - len(p_parts)} board-only)")

    for ref in pcb.duplicates:
        rep.error("duplicate", f"{ref} appears more than once on the board")
    for ref in sorted(s_parts.keys() - p_parts.keys(), key=natural):
        rep.error("orphan", f"{ref} is in the schematic but not on the board")
    for ref in sorted(p_parts.keys() - s_parts.keys(), key=natural):
        rep.error("orphan", f"{ref} is on the board but not in the schematic")

    for ref in sorted(s_parts.keys() & p_parts.keys(), key=natural):
        s, p = s_parts[ref], p_parts[ref]
        if s.value != p.value:
            rep.error("value", f"{ref}: schematic {s.value!r}, board {p.value!r}")
        if s.footprint != p.lib_id:
            rep.error("footprint", f"{ref}: schematic {s.footprint or '(none)'!r}, board {p.lib_id!r}")
        for name in sorted((s.fields.keys() | p.properties.keys()) - SKIP_FIELDS):
            if name.startswith("ki_"):
                continue
            sv, pv = s.fields.get(name), p.properties.get(name)
            if sv is None and not pv:
                continue
            if sv != pv:
                rep.error("field", f"{ref} {name}: schematic {sv!r}, board {pv!r}")
        if s.dnp != p.dnp:
            rep.error("dnp", f"{ref}: DNP is {s.dnp} in the schematic, {p.dnp} on the board")
        if s.exclude_from_bom != p.exclude_from_bom:
            rep.error("bom-flag", f"{ref}: exclude-from-BOM is {s.exclude_from_bom} in the schematic, "
                                  f"{p.exclude_from_bom} on the board")

    built = {r for r, s in s_parts.items() if not s.dnp and not s.exclude_from_bom}
    for ref in sorted(built, key=natural):
        for name in require:
            if not s_parts[ref].fields.get(name, "").strip():
                rep.error("missing-field", f"{ref} ({s_parts[ref].value}) has no {name}; "
                                           f"mark it DNP or exclude-from-BOM if it is not assembled")

    if args.bom:
        keys, rows = read_csv(args.bom)
        c_ref = column(keys, "designator", "reference", "references", "ref", "refs")
        c_val = column(keys, "comment", "value", "val")
        c_lcsc = column(keys, "lcsc", "lcsc part #", "lcsc part", "jlcpcb part #", "supplier part")
        c_mpn = column(keys, "mpn", "manufacturer part number", "mfr part #", "part number")
        c_mfr = column(keys, "manufacturer", "mfr")
        if not c_lcsc and "LCSC" in require:
            rep.error("bom", f"{args.bom}: no LCSC column in {list(keys)}, so part numbers can't be checked")
        if not c_ref:
            rep.error("bom", f"{args.bom}: no designator column in {list(keys)}")
        else:
            seen = set()
            for row in rows:
                for ref in split_refs(row[c_ref]):
                    if ref in seen:
                        rep.error("bom", f"{ref} is listed more than once in the BOM")
                    seen.add(ref)
                    s = s_parts.get(ref)
                    if not s:
                        rep.error("bom", f"BOM lists {ref}, which is not in the schematic")
                        continue
                    if ref not in built:
                        rep.error("bom", f"BOM lists {ref}, which is DNP or excluded from BOM")
                    if c_val and row[c_val].strip() != s.value:
                        rep.error("bom", f"{ref}: BOM value {row[c_val]!r}, schematic {s.value!r}")
                    for col, fname in ((c_lcsc, "LCSC"), (c_mpn, "MPN"), (c_mfr, "Manufacturer")):
                        if col and row[col].strip() != s.fields.get(fname, "").strip():
                            rep.error("bom", f"{ref}: BOM {fname} {row[col]!r}, schematic {s.fields.get(fname, '')!r}")
            for ref in sorted(built - seen, key=natural):
                rep.error("bom", f"{ref} should be built but is missing from the BOM")
            rep.note(f"BOM {args.bom.name}: {len(seen)} designators")

    if args.cpl:
        keys, rows = read_csv(args.cpl)
        c_ref = column(keys, "designator", "ref", "reference")
        c_side = column(keys, "layer", "side")
        if not c_ref:
            rep.error("cpl", f"{args.cpl}: no designator column in {list(keys)}")
        else:
            seen = set()
            for row in rows:
                ref = row[c_ref].strip()
                if ref in seen:
                    rep.error("cpl", f"{ref} is placed more than once in the CPL")
                seen.add(ref)
                p = p_parts.get(ref)
                if not p:
                    rep.error("cpl", f"CPL places {ref}, which is not on the board")
                    continue
                if ref not in built:
                    rep.error("cpl", f"CPL places {ref}, which is DNP or excluded from BOM")
                if c_side:
                    side = row[c_side].strip().lower()
                    want = "top" if p.layer == "F.Cu" else "bottom"
                    if side[:1] not in (want[0], "f" if want == "top" else "b"):
                        rep.error("cpl", f"{ref}: CPL side {row[c_side]!r}, board {want}")
            missing = {r for r in built if r in p_parts
                       and not p_parts[r].attrs & {"through_hole", "exclude_from_pos_files"}} - seen
            for ref in sorted(missing, key=natural):
                rep.error("cpl", f"{ref} is an assembled SMD part missing from the CPL")
            rep.note(f"CPL {args.cpl.name}: {len(seen)} placements")

    rep.note(f"{len(built)} parts to build; required fields: {', '.join(require) or 'none'}")
    rep.note("stock not checked: re-check every LCSC part at release (phase 10)")
    return rep.finish(args.json)


if __name__ == "__main__":
    sys.exit(main())
