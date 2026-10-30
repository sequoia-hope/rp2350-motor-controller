#!/usr/bin/env python3
"""silk_check.py -- silkscreen rules for phase 9.

    silk_check.py hw/main/a
    silk_check.py hw/main/a --json design/gates/silk.json

Geometry (silk over silk, pads, vias, board edge; text size) comes from KiCad's
own DRC, run on a temporary copy of the project with every silk rule forced to
"error" so a rule switched off in the project can't hide anything. Excluded
DRC markers still count: silk has zero tolerance. On top of that, using the
pcbnew module on the rendered text outlines, it fails on:

  - a shown designator under any courtyard, its own included
  - a shown designator whose nearest courtyard belongs to another part
  - a hidden designator the policy says should show ([silk].designators)
  - more designator rotations than [silk].max_rotations
  - a connector with no board-level silk label within [silk].connector_label_mm
  - a missing board identity item ([silk].identity: name, rev, date, repo, license)
  - silk text over a via, or a plated hole under the mask (thermal vias),
    on either side (they are tented, so KiCad's silk_over_copper passes them, but the ink breaks up over the drill and
    the ring: [silk].via_clearance_mm, default 0.1, from the via's pad)
  - a polarised part (prefixes in [silk].pin1_prefixes, default U, D, LED, Q)
    with no pin-1 / cathode mark: no silk item (the footprint's own, or a
    board-level shape) within [silk].pin1_mm (default 1.2) of pad 1 whose
    centre is nearer pad 1 than every other pad. Trimming footprint silk off
    pads often takes the mark with it; a side line spanning both ends doesn't
    count
"""
from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from kicad_files import Report, find_project, load_checks, load_pro, repo_root  # noqa: E402

try:
    import pcbnew
except ImportError:
    sys.exit("silk_check needs KiCad's python module (pcbnew); see scripts/doctor.sh")

SILK_RULES = ["silk_overlap", "silk_over_copper", "silk_edge_clearance", "text_height",
              "text_thickness", "mirrored_text_on_front_layer", "nonmirrored_text_on_back_layer"]

# Prefixes the "selective" designator policy keeps visible (silkscreen-and-art.md).
SELECTIVE = ["J", "P", "CN", "USB", "U", "TP", "JP", "SW", "LED", "D", "RV", "BT", "F", "Y", "H"]

IDENTITY = {
    "rev": r"(\brev\.?\s*[A-Z0-9]|\bv\d+(\.\d+)+)",
    "date": r"\b20\d\d([-/.]\d\d)?\b",
    "repo": r"(https?://|github\.com/|gitlab\.com/|codeberg\.org/)",
    "license": r"(CERN-OHL|OHL|MIT\b|Apache|GPL|CC[- ]BY|Solderpad|TAPR)",
}

MM = 1e6  # pcbnew internal units per mm


def prefix(ref: str) -> str:
    return re.match(r"[A-Za-z_#]*", ref).group(0)


def run_drc(pro: Path, pcb: Path, rep: Report) -> list[dict]:
    """DRC on a scratch copy with every silk rule switched on."""
    settings = load_pro(pro)
    sev = settings.get("board", {}).get("design_settings", {}).get("rule_severities", {})
    off = [r for r in SILK_RULES if sev.get(r) == "ignore"]
    if off:
        rep.warn("severity", f"{pro.name} ignores {', '.join(off)}; KiCad's own DRC will not show "
                             "these (this check runs them anyway). Set them to error.")
    with tempfile.TemporaryDirectory() as tmp:
        t = Path(tmp)
        for f in pro.parent.glob(pro.stem + ".*"):
            if f.suffix in (".kicad_pcb", ".kicad_pro", ".kicad_dru"):
                shutil.copy(f, t / f.name)
        settings.setdefault("board", {}).setdefault("design_settings", {}) \
            .setdefault("rule_severities", {}).update({r: "error" for r in SILK_RULES})
        (t / pro.name).write_text(json.dumps(settings, indent=2))
        out = t / "drc.json"
        r = subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-all",
                            "--severity-exclusions",
                            "--units", "mm", "-o", str(out), str(t / pcb.name)],
                           capture_output=True, text=True)
        if not out.exists():
            sys.exit(f"kicad-cli pcb drc failed:\n{r.stdout}\n{r.stderr}")
        return [v for v in json.loads(out.read_text())["violations"] if v["type"] in SILK_RULES]


def poly(text) -> "pcbnew.SHAPE_POLY_SET":
    ps = pcbnew.SHAPE_POLY_SET()
    text.TransformTextToPolySet(ps, 0, pcbnew.FromMM(0.005), pcbnew.ERROR_INSIDE)
    return ps


def overlaps(a, b) -> bool:
    x = pcbnew.SHAPE_POLY_SET(a)
    x.BooleanIntersection(b)
    return x.Area() > (0.01 * MM) ** 2


def nearest_part(text, court) -> str | None:
    """The part whose courtyard is closest to a text's centre: a label belongs to it."""
    c = pcbnew.VECTOR2I(text.GetBoundingBox().GetCenter())
    side = text.GetLayer() == pcbnew.B_SilkS
    dist = {r: poly.SquaredDistance(c) for r, (flipped, poly) in court.items()
            if poly is not None and flipped == side}
    return min(dist, key=dist.get) if dist else None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("project", type=Path, help="variant folder, .kicad_pro, .kicad_sch or .kicad_pcb")
    ap.add_argument("--json", help="also write findings as JSON here")
    args = ap.parse_args()

    pro, _, pcb_path = find_project(args.project)
    cfg = load_checks(repo_root(pro.parent))
    silk, fab = cfg["silk"], cfg["fab"]
    policy = silk.get("designators", "all")
    rep = Report("silk_check", str(pro.parent))

    # 1. Geometry from KiCad DRC. It reports A-vs-B and B-vs-A separately; keep one.
    seen = set()
    for v in run_drc(pro, pcb_path, rep):
        items = "; ".join(sorted(i["description"] for i in v.get("items", [])))
        where = v["items"][0]["pos"] if v.get("items") else {}
        key = (v["type"], items, round(where.get("x", 0), 1), round(where.get("y", 0), 1))
        if key in seen:
            continue
        seen.add(key)
        at = f" @ ({where.get('x', 0):.2f}, {where.get('y', 0):.2f})" if where else ""
        excl = " (excluded in KiCad, still counts)" if v.get("excluded") else ""
        rep.error(v["type"], f"{items}{at}{excl}")

    # 2. Designators, using the rendered text outlines.
    board = pcbnew.LoadBoard(str(pcb_path))
    fps = [fp for fp in board.GetFootprints() if fp.GetReference() and not fp.GetReference().startswith("#")]
    court = {}
    for fp in fps:
        fp.BuildCourtyardCaches()
        layer = pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd
        c = fp.GetCourtyard(layer)
        court[fp.GetReference()] = (fp.IsFlipped(), c if c.OutlineCount() else None)
        if not c.OutlineCount():
            rep.warn("courtyard", f"{fp.GetReference()} ({fp.GetFPIDAsString()}) has no courtyard")

    angles = {}
    shown = 0
    for fp in fps:
        if fp.IsBoardOnly():   # logos, stitching, mechanical: no designator expected
            continue
        ref = fp.GetReference()
        field = fp.Reference()
        on_silk = field.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS)
        visible = field.IsVisible() and on_silk
        wanted = policy == "all" or (policy == "selective" and prefix(ref) in SELECTIVE)
        if wanted and not visible:
            rep.error("hidden", f"{ref}: designator is hidden or not on silk (policy: {policy})")
        if not visible:
            continue
        shown += 1
        h, w = pcbnew.ToMM(field.GetTextHeight()), pcbnew.ToMM(field.GetTextThickness())
        if h + 1e-6 < fab["min_silk_height_mm"]:
            rep.error("size", f"{ref}: text height {h:.2f} mm < {fab['min_silk_height_mm']} mm")
        if w + 1e-6 < fab["min_silk_width_mm"]:
            rep.error("size", f"{ref}: line width {w:.2f} mm < {fab['min_silk_width_mm']} mm")
        angles.setdefault(round(field.GetDrawRotation().AsDegrees()) % 180, []).append(ref)

        tp = poly(field)
        center = pcbnew.VECTOR2I(field.GetBoundingBox().GetCenter())
        flipped = fp.IsFlipped()
        under = [r for r, (side, c) in court.items()
                 if c is not None and side == flipped and overlaps(tp, c)]
        if under:
            rep.error("under-part", f"{ref}: designator sits under the courtyard of {', '.join(under)}")
        mine = court[ref][1]
        if mine is not None:
            d_own = mine.SquaredDistance(center) ** 0.5
            closer = [r for r, (side, c) in court.items()
                      if r != ref and c is not None and side == flipped
                      and c.SquaredDistance(center) ** 0.5 + 0.05 * MM < d_own]
            if closer:
                rep.error("association", f"{ref}: designator is closer to {', '.join(closer[:3])} "
                                         f"than to its own part ({d_own / MM:.2f} mm away)")

    if len(angles) > silk["max_rotations"]:
        summary = ", ".join(f"{a}°: {len(r)}" for a, r in sorted(angles.items()))
        rep.error("rotation", f"designators use {len(angles)} rotations ({summary}); "
                              f"max {silk['max_rotations']}")

    # 3. Board-level silk text: connector labels and identity.
    # Text written for this board: board-level text, and text inside board-only or
    # project-library footprints (logos, label blocks). A stock library
    # footprint's own "+" or "1" is not a label someone chose.
    stock = {p.stem for p in Path("/usr/share/kicad/footprints").glob("*.pretty")}
    texts, owner = [], {}
    for d in board.GetDrawings():
        if d.GetClass() in ("PCB_TEXT", "PCB_TEXTBOX") and d.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS):
            texts.append((d, d.GetShownText(True)))
    for fp in fps:
        if not fp.IsBoardOnly() and fp.GetFPID().GetLibNickname().wx_str() in stock:
            continue
        for item in fp.GraphicalItems():
            if item.GetClass() == "PCB_TEXT" and item.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS) \
                    and item.IsVisible() and "${" not in item.GetText():
                texts.append((item, item.GetShownText(True)))
                owner[id(item)] = fp.GetReference()
    for t, s_ in texts:
        h, w = pcbnew.ToMM(t.GetTextHeight()), pcbnew.ToMM(t.GetTextThickness())
        if h + 1e-6 < fab["min_silk_height_mm"] or w + 1e-6 < fab["min_silk_width_mm"]:
            rep.error("size", f"silk text {s_[:30]!r}: {h:.2f} mm high, {w:.2f} mm line "
                              f"(min {fab['min_silk_height_mm']} / {fab['min_silk_width_mm']})")

    reach = silk["connector_label_mm"] * MM
    conns = [fp for fp in fps if prefix(fp.GetReference()) in silk["connector_prefixes"]]
    for fp in conns:
        ref = fp.GetReference()
        c = court[ref][1]
        bbox = c.BBox() if c is not None else fp.GetBoundingBox(False, False)
        bbox.Inflate(int(reach))
        near = [t for t, s in texts if s.strip() and bbox.Intersects(t.GetBoundingBox())
                and t.GetLayer() == (pcbnew.B_SilkS if fp.IsFlipped() else pcbnew.F_SilkS)
                and owner.get(id(t)) != ref and nearest_part(t, court) == ref]
        if not near:
            rep.error("connector-label", f"{ref} ({fp.GetValue()}) has no silk label within "
                                         f"{silk['connector_label_mm']} mm (function, polarity, rating)")

    # 4. Silk text over vias (a via goes through both sides).
    gap = int(silk.get("via_clearance_mm", 0.1) * MM)
    vias = [v for v in board.GetTracks() if v.Type() == pcbnew.PCB_VIA_T]
    # plated holes under the mask (an exposed pad's thermal vias) break the ink too
    holes = [q for fp in fps for q in fp.Pads() if q.HasHole()]
    visible_refs = [(fp.Reference(), fp.GetReference()) for fp in fps
                    if fp.Reference().IsVisible() and fp.Reference().GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS)]
    for t, s_ in texts + visible_refs:
        tp = poly(t)
        hits = [v for v in vias
                if tp.Collide(pcbnew.VECTOR2I(v.GetPosition()), v.GetWidth(pcbnew.F_Cu) // 2 + gap)]
        hits += [q for q in holes
                 if not q.IsOnLayer(pcbnew.B_Mask if t.GetLayer() == pcbnew.B_SilkS else pcbnew.F_Mask)
                 and tp.Collide(pcbnew.VECTOR2I(q.GetPosition()), q.GetDrillSizeX() // 2 + gap)]
        if hits:
            side = "bottom" if t.GetLayer() == pcbnew.B_SilkS else "top"
            p0 = hits[0].GetPosition()
            rep.error("silk-over-via", f"{side} silk {s_[:30]!r} over {len(hits)} via(s), first at "
                                       f"({pcbnew.ToMM(p0.x):.2f}, {pcbnew.ToMM(p0.y):.2f})")

    # 5. Pin-1 and polarity marks.
    import math
    reach1 = silk.get("pin1_mm", 1.2) * MM
    pfx = silk.get("pin1_prefixes", ["U", "D", "LED", "Q"])
    board_marks = [d for d in board.GetDrawings() if d.GetClass() == "PCB_SHAPE"]
    for fp in fps:
        pads = {q.GetNumber(): q for q in fp.Pads()}
        if prefix(fp.GetReference()) not in pfx or "1" not in pads or len(pads) < 2:
            continue
        layer = pcbnew.B_SilkS if fp.IsFlipped() else pcbnew.F_SilkS
        p1 = pads["1"].GetPosition()
        half = max(pads["1"].GetBoundingBox().GetWidth(), pads["1"].GetBoundingBox().GetHeight()) / 2
        rest = [q.GetPosition() for n, q in pads.items() if n != "1" and q.GetPosition() != p1]
        marks = [i for i in fp.GraphicalItems() if i.GetLayer() == layer and i.GetClass() != "PCB_TEXT"]
        marks += [d for d in board_marks if d.GetLayer() == layer]
        ok = False
        for m in marks:
            c = m.GetBoundingBox().GetCenter()
            d1 = math.dist((c.x, c.y), (p1.x, p1.y))
            if d1 <= half + reach1 and all(d1 <= math.dist((c.x, c.y), (o.x, o.y)) for o in rest):
                ok = True
                break
        if not ok:
            rep.error("pin1", f"{fp.GetReference()} ({fp.GetFPID().GetLibItemName()}): no pin-1 / polarity "
                              f"mark on the silk beside pad 1")

    alltext = "\n".join(s for _, s in texts)
    tb = board.GetTitleBlock()
    name = silk.get("board_name") or tb.GetTitle()
    for item in silk["identity"]:
        if item == "name":
            if not name:
                rep.error("identity", "no board name: set the title block title or [silk].board_name")
            elif name.lower() not in alltext.lower():
                rep.error("identity", f"board name {name!r} is not on the silkscreen")
        elif item in IDENTITY:
            if not re.search(IDENTITY[item], alltext, re.I):
                rep.error("identity", f"no {item} on the silkscreen (looked for /{IDENTITY[item]}/)")
        else:
            rep.error("config", f"unknown identity item {item!r}; use name, rev, date, repo, license")

    rep.note(f"{len(fps)} parts, {shown} designators shown (policy: {policy}), "
             f"{len(conns)} connectors, {len(texts)} board silk texts")
    rep.note(f"designator rotations: " + (", ".join(f"{a}°×{len(r)}" for a, r in sorted(angles.items())) or "none"))
    return rep.finish(args.json)


if __name__ == "__main__":
    sys.exit(main())
