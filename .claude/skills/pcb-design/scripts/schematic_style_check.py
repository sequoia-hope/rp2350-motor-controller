#!/usr/bin/env python3
"""schematic_style_check.py -- the mechanical part of the phase 2 style rules.

    schematic_style_check.py hw/main/a
    schematic_style_check.py hw/main/a --json design/gates/style.json

Reads every sheet of the schematic and checks what a script can see
(references/schematic-style.md):

  power-symbol   ground symbols point down, supply symbols point up
  fields         every placed part has a real value and a footprint; built
                 parts have the fields in [sync].require_fields
  label-wire     two labels of one net closer than [style].label_wire_mm:
                 draw the wire instead
  global-label   global labels on a one-sheet design
  decoupler      a capacitor with a Decouples field ("U1.11") drawn more than
                 [style].decoupler_mm from the pin it serves (a warning when
                 the pin is stacked under others, e.g. an MCU's IOVDD pins)
  grid           six or more symbols of one kind on a 2-D lattice (the
                 programmatic look); a single row of decouplers is fine
  text-overlap   visible texts whose boxes overlap (approximate: KiCad's
                 stroke font, ~0.9 x height per character); a warning

It cannot judge readability: the visual review still does that.
"""
from __future__ import annotations

import argparse
import itertools
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from kicad_files import Report, child, children, find_project, head, load_checks, parse_sexpr, repo_root  # noqa: E402
from kicad_write import place, symbol_pins  # noqa: E402

GENERIC_VALUES = {"R", "C", "L", "D", "LED", "Q", "U", "~", ""}
CHAR_W = 0.9   # stroke font advance per character, in units of text height (approximate)


def sheets(root: Path) -> list[Path]:
    """The root sheet and every sub-sheet it references, once each."""
    out, todo = [], [root]
    while todo:
        p = todo.pop()
        if p in out or not p.exists():
            continue
        out.append(p)
        tree = parse_sexpr(p.read_text(encoding="utf-8"))
        for sh in children(tree, "sheet"):
            for prop in children(sh, "property"):
                if prop[1] in ("Sheetfile", "Sheet file"):
                    todo.append(p.parent / prop[2])
    return out


def props(node) -> dict[str, list]:
    return {p[1]: p for p in children(node, "property")}


def hidden(prop) -> bool:
    eff = child(prop, "effects")
    if eff is None:
        return False
    h = child(eff, "hide")
    return h is not None and (len(h) == 1 or h[1] in ("yes", "true")) or "hide" in eff


def text_box(s: str, at, size: float, angle: float, justify: set[str]):
    """Approximate box (x0, y0, x1, y1) of a single- or multi-line text."""
    lines = s.split("\n")
    w = max(len(line) for line in lines) * size * CHAR_W
    h = size * (1.6 * (len(lines) - 1) + 1.0)
    x, y = float(at[0]), float(at[1])
    if "right" in justify:
        x0 = x - w
    elif "left" in justify:
        x0 = x
    else:
        x0 = x - w / 2
    if "top" in justify:
        y0 = y
    elif "bottom" in justify:
        y0 = y - h
    else:
        y0 = y - h / 2
    box = (x0, y0, x0 + w, y0 + h)
    if round(angle) % 180 == 90:          # rotate the box about the anchor
        bx0, by0, bx1, by1 = box
        box = (x + (by0 - y), y - (bx1 - x), x + (by1 - y), y - (bx0 - x))
    return box


def overlap(a, b, margin=0.15) -> bool:
    return a[0] + margin < b[2] and b[0] + margin < a[2] and a[1] + margin < b[3] and b[1] + margin < a[3]


def lattice(points: list[tuple[float, float]]) -> bool:
    """True when the points fill at least 2 rows x 3 columns of an evenly spaced grid."""
    xs = sorted({round(p[0], 2) for p in points})
    ys = sorted({round(p[1], 2) for p in points})
    if len(xs) < 2 or len(ys) < 2 or len(xs) * len(ys) < 6:
        return False

    def even(v):
        steps = {round(b - a, 2) for a, b in zip(v, v[1:])}
        return len(steps) == 1
    filled = len({(round(p[0], 2), round(p[1], 2)) for p in points})
    return even(xs) and even(ys) and filled >= 0.8 * len(xs) * len(ys)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("project", type=Path, help="variant folder, .kicad_pro or .kicad_sch")
    ap.add_argument("--json")
    args = ap.parse_args()

    pro, sch_path, _ = find_project(args.project, need_pcb=False)
    cfg = load_checks(repo_root(pro.parent))
    style = cfg.get("style", {})
    label_mm = style.get("label_wire_mm", 25.0)
    decap_mm = style.get("decoupler_mm", 40.0)
    require = cfg.get("sync", {}).get("require_fields", ["LCSC"])
    rep = Report("schematic_style_check", str(sch_path))

    files = sheets(sch_path)
    pins_at: dict[str, dict[str, tuple]] = {}      # ref -> pin number -> (sheet, x, y)
    caps = []                                        # (ref, sheet, at, decouples)
    n_parts = 0
    for sheet in files:
        tree = parse_sexpr(sheet.read_text(encoding="utf-8"))
        libs = {s[1]: s for s in children(child(tree, "lib_symbols") or [None], "symbol")}
        texts = []                                   # (owner, box, text)
        by_kind = defaultdict(list)
        labels = defaultdict(list)
        for sym in children(tree, "symbol"):
            lib_id = child(sym, "lib_id")[1]
            lib = libs.get(lib_id)
            at = child(sym, "at")
            x, y, rot = float(at[1]), float(at[2]), float(at[3]) if len(at) > 3 else 0.0
            m = child(sym, "mirror")
            mirror = str(m[1]) if m else None
            pr = props(sym)
            ref, value = pr["Reference"][2], pr["Value"][2]
            is_power = lib is not None and child(lib, "power") is not None
            if is_power:
                if value == "PWR_FLAG":
                    continue
                down = "GND" in value.upper() or "VSS" in value.upper()
                want = 0
                if round(rot) % 360 != want or mirror == "x":
                    rep.error("power-symbol", f"{sheet.name}: {value} symbol at ({x:.2f}, {y:.2f}) is rotated "
                                              f"{rot:g}: {'ground points down' if down else 'supplies point up'}")
                vp = pr["Value"]
                if not hidden(vp):
                    texts.append((ref, text_box(value, child(vp, "at")[1:3], 1.27, 0, {""}), value))
                continue
            n_parts += 1
            on_board = child(sym, "on_board") is None or child(sym, "on_board")[1] == "yes"
            in_bom = child(sym, "in_bom") is None or child(sym, "in_bom")[1] == "yes"
            dnp = child(sym, "dnp") is not None and child(sym, "dnp")[1] == "yes"
            if on_board:
                if value.strip() in GENERIC_VALUES:
                    rep.error("fields", f"{ref}: value {value!r} says nothing; give the real value")
                if not pr.get("Footprint", [0, 0, ""])[2]:
                    rep.error("fields", f"{ref}: no footprint")
            if on_board and in_bom and not dnp:
                for f in require:
                    if not (pr.get(f, [0, 0, ""])[2] or "").strip():
                        rep.error("fields", f"{ref}: no {f} (mark it DNP or exclude it from the BOM if it isn't built)")
            by_kind[lib_id].append((x, y, ref))
            if lib is not None:
                for num, p in symbol_pins(lib, int(child(sym, "unit")[1]) if child(sym, "unit") else 1).items():
                    pins_at.setdefault(ref, {})[num] = (sheet, *place(p["x"], p["y"], (x, y), rot, mirror))
            if lib_id.endswith(":C") or lib_id.split(":")[-1].startswith("C_"):
                caps.append((ref, sheet, (x, y), (pr.get("Decouples", [0, 0, ""])[2] or "").strip()))
            for name in ("Reference", "Value"):
                p = pr[name]
                if hidden(p):
                    continue
                pat = child(p, "at")
                eff = child(p, "effects")
                j = child(eff, "justify") if eff else None
                just = {str(v) for v in j[1:]} if j else set()
                if mirror == "y":
                    just = {{"left": "right", "right": "left"}.get(v, v) for v in just}
                ang = (float(pat[3]) if len(pat) > 3 else 0) + rot
                texts.append((ref, text_box(p[2], pat[1:3], 1.27, ang, just), p[2]))
        for kind in ("label", "global_label", "hierarchical_label", "text"):
            for lab in children(tree, kind):
                at = child(lab, "at")
                eff = child(lab, "effects")
                j = child(eff, "justify") if eff else None
                just = {str(v) for v in j[1:]} if j else set()
                size = float(child(child(eff, "font"), "size")[1]) if eff and child(eff, "font") else 1.27
                ang = float(at[3]) if len(at) > 3 else 0
                if kind == "label":
                    labels[lab[1]].append((float(at[1]), float(at[2])))
                    just = {"left", "bottom"} if round(ang) % 360 in (0, 90) else {"right", "bottom"}
                texts.append((f"{kind}:{lab[1][:20]}", text_box(lab[1], at[1:3], size, ang, just), lab[1]))
                if kind == "global_label" and len(files) == 1:
                    rep.warn("global-label", f"{sheet.name}: global label {lab[1]!r} on a one-sheet design; "
                                             "use a wire or a local label")

        for name, pts in labels.items():
            for a, b in itertools.combinations(pts, 2):
                d = math.dist(a, b)
                if d < label_mm:
                    rep.error("label-wire", f"{sheet.name}: two {name!r} labels {d:.1f} mm apart at "
                                            f"({a[0]:.1f}, {a[1]:.1f}) and ({b[0]:.1f}, {b[1]:.1f}): draw the wire")
                    break

        for kind, items in by_kind.items():
            if len(items) >= 6 and lattice([(x, y) for x, y, _ in items]):
                rep.error("grid", f"{sheet.name}: {len(items)} {kind} symbols sit on a regular 2-D grid "
                                  f"({', '.join(r for *_, r in items[:6])}...): place each with what it serves")

        seen = set()
        for (oa, ba, ta), (ob, bb, tb) in itertools.combinations(texts, 2):
            if oa == ob or not overlap(ba, bb):
                continue
            key = tuple(sorted((f"{oa} {ta!r}", f"{ob} {tb!r}")))
            if key not in seen:
                seen.add(key)
                rep.warn("text-overlap", f"{sheet.name}: {key[0]} overlaps {key[1]} near "
                                         f"({ba[0]:.1f}, {ba[1]:.1f})")

    for ref, sheet, at, dec in caps:
        if not dec:
            continue
        m = re.fullmatch(r"([A-Za-z_#]+\d+)\.(\w+)", dec)
        if not m or m.group(1) not in pins_at or m.group(2) not in pins_at[m.group(1)]:
            rep.error("decoupler", f"{ref}: Decouples={dec!r} names no pin in the schematic")
            continue
        psheet, px, py = pins_at[m.group(1)][m.group(2)]
        stacked = sum(1 for q in pins_at[m.group(1)].values() if q == (psheet, px, py)) > 1
        if psheet != sheet:
            rep.error("decoupler", f"{ref} is drawn on {sheet.name}, the pin it serves ({dec}) on {psheet.name}")
            continue
        d = math.dist(at, (px, py))
        if d > decap_mm and stacked:
            rep.warn("decoupler", f"{ref} is drawn {d:.0f} mm from {dec}, a stacked pin: the drawing can't "
                                  "put it beside that pin, so placement relies on the Decouples field")
        elif d > decap_mm:
            rep.error("decoupler", f"{ref} is drawn {d:.0f} mm from {dec}, the pin it serves "
                                   f"(limit {decap_mm:g} mm): draw it beside the pin")

    rep.note(f"{len(files)} sheet(s), {n_parts} parts, {sum(1 for c in caps if c[3])} of {len(caps)} "
             f"capacitors name the pin they decouple")
    return rep.finish(args.json)


if __name__ == "__main__":
    sys.exit(main())
