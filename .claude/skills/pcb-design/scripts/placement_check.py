#!/usr/bin/env python3
"""placement_check.py -- the mechanical part of the phase 6 placement rules.

    placement_check.py hw/main/a
    placement_check.py hw/main/a --json design/gates/placement.json

Reads the board (pcbnew) and checks what a script can see
(references/placement.md):

  decoupler   every capacitor with a Decouples field ("U1.11", set in the
              schematic) has a pad within [placement].decoupler_mm of that pin,
              measured pad edge to pad edge along the shared net
  crystal     every crystal (Y*) pad is within [placement].crystal_mm of the
              IC pin on the same net
  row         four or more same-footprint passives in a straight, evenly
              spaced line with the same rotation: the "dumped in a row" look.
              A warning: a row of LEDs or a decoupler bank along a pin row can
              be right; the reviewer decides
  keep-apart  pairs in [placement].keep_apart = [["U3", "Y1", 5.0], ...] are
              at least that far apart, courtyard to courtyard
  outside     every part's courtyard is inside the board outline

It also prints the total ratsnest length (sum of each net's minimum spanning
tree over its pads), a single number to compare placements by.
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
from kicad_files import Report, find_project, load_checks, repo_root  # noqa: E402


def mst_length(points: list[tuple[float, float]]) -> float:
    if len(points) < 2:
        return 0.0
    done, total = {0}, 0.0
    best = {i: math.dist(points[0], p) for i, p in enumerate(points) if i}
    while best:
        i = min(best, key=best.get)
        total += best.pop(i)
        done.add(i)
        for j in best:
            best[j] = min(best[j], math.dist(points[i], points[j]))
    return total


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("project", type=Path)
    ap.add_argument("--json")
    args = ap.parse_args()
    import pcbnew

    pro, _, pcb_path = find_project(args.project)
    cfg = load_checks(repo_root(pro.parent)).get("placement", {})
    dec_mm = cfg.get("decoupler_mm", 3.0)
    xtal_mm = cfg.get("crystal_mm", 6.0)
    rep = Report("placement_check", str(pcb_path))

    b = pcbnew.LoadBoard(str(pcb_path))
    fps = {f.GetReference(): f for f in list(b.GetFootprints())}
    mm = pcbnew.ToMM

    def pads(ref):
        return list(fps[ref].Pads())

    def centre_gap(p, q) -> float:
        return mm(int(math.dist((p.GetPosition().x, p.GetPosition().y), (q.GetPosition().x, q.GetPosition().y))))

    # decouplers
    lines = []
    for ref, f in sorted(fps.items()):
        dec = f.GetFieldText("Decouples") if f.HasFieldByName("Decouples") else ""
        if not dec:
            continue
        m = re.fullmatch(r"(\w+?\d+)\.(\w+)", dec.strip())
        if not m or m.group(1) not in fps:
            rep.error("decoupler", f"{ref}: Decouples={dec!r} names no part on the board")
            continue
        target = [p for p in pads(m.group(1)) if p.GetNumber() == m.group(2)]
        if not target:
            rep.error("decoupler", f"{ref}: {m.group(1)} has no pad {m.group(2)}")
            continue
        t = target[0]
        mine = [p for p in pads(ref) if p.GetNetCode() == t.GetNetCode()]
        if not mine:
            rep.error("decoupler", f"{ref}: no pad on {t.GetNetname()}, the net of {dec}")
            continue
        d = min(centre_gap(p, t) for p in mine)
        lines.append(f"{ref:5} -> {dec:7} {t.GetNetname():10} {d:5.2f} mm")
        if d > dec_mm:
            rep.error("decoupler", f"{ref} is {d:.1f} mm from {dec} (pad centre to pad centre, limit {dec_mm:g} mm)")
    if lines:
        rep.table("decoupler -> pin, pad centre to pad centre", lines)

    # crystals
    for ref, f in sorted(fps.items()):
        if not ref.startswith("Y"):
            continue
        for p in pads(ref):
            if p.GetNetname() in ("", "GND") or p.GetNetname().upper().startswith("GND"):
                continue
            ics = [q for r, g in fps.items() if r.startswith("U") for q in g.Pads()
                   if q.GetNetCode() == p.GetNetCode()]
            via = [q for r, g in fps.items() if not r.startswith(("U", "Y")) for q in g.Pads()
                   if q.GetNetCode() == p.GetNetCode()]
            # A crystal pin may reach the IC through a series resistor: follow one hop.
            for q in via:
                other = [o for o in q.GetParentFootprint().Pads() if o.GetNumber() != q.GetNumber()]
                for o in other:
                    ics += [s for r, g in fps.items() if r.startswith("U") for s in g.Pads()
                            if s.GetNetCode() == o.GetNetCode() and o.GetNetname() not in ("GND",)]
            if not ics:
                continue
            d = min(centre_gap(p, q) for q in ics)
            rep.note(f"{ref} pad {p.GetNumber()} ({p.GetNetname()}) is {d:.1f} mm from its IC pin")
            if d > xtal_mm:
                rep.error("crystal", f"{ref} pad {p.GetNumber()} is {d:.1f} mm from its IC pin (limit {xtal_mm:g} mm)")

    # rows of passives
    groups = defaultdict(list)
    for ref, f in fps.items():
        if re.match(r"^(R|C|L|FB)\d", ref) and not f.IsFlipped():
            groups[(f.GetFPIDAsString(), round(f.GetOrientationDegrees()) % 180)].append(f)
    for (fpid, rot), items in groups.items():
        pts = {f.GetReference(): (mm(f.GetPosition().x), mm(f.GetPosition().y)) for f in items}
        for axis in (0, 1):
            lines_by = defaultdict(list)
            for r, p in pts.items():
                lines_by[round(p[axis], 1)].append((p[1 - axis], r))
            for _, row in lines_by.items():
                if len(row) < 4:
                    continue
                row.sort()
                steps = [round(b[0] - a[0], 1) for a, b in zip(row, row[1:])]
                if max(steps) - min(steps) <= 0.2 and max(steps) < 5:
                    rep.warn("row", f"{len(row)} × {fpid.split(':')[-1]} in a line at {steps[0]} mm pitch: "
                                    f"{', '.join(r for _, r in row)} — is each with what it serves?")

    # keep-apart pairs
    for a, c, dmin in cfg.get("keep_apart", []):
        if a not in fps or c not in fps:
            rep.error("keep-apart", f"{a} or {c} is not on the board")
            continue
        ba, bc = fps[a].GetBoundingBox(False), fps[c].GetBoundingBox(False)
        dx = max(0, max(ba.GetLeft(), bc.GetLeft()) - min(ba.GetRight(), bc.GetRight()))
        dy = max(0, max(ba.GetTop(), bc.GetTop()) - min(ba.GetBottom(), bc.GetBottom()))
        d = mm(int(math.hypot(dx, dy)))
        rep.note(f"keep-apart {a}-{c}: {d:.1f} mm (min {dmin:g})")
        if d < dmin:
            rep.error("keep-apart", f"{a} and {c} are {d:.1f} mm apart (body to body), want >= {dmin:g} mm")

    # outside the outline
    edge = b.GetBoardEdgesBoundingBox()
    for ref, f in sorted(fps.items()):
        bb = f.GetBoundingBox(False)
        if f.IsBoardOnly() or not ref:
            continue
        if not edge.Contains(bb.Centre()):
            rep.error("outside", f"{ref} sits outside the board outline")

    # ratsnest score
    by_net = defaultdict(list)
    for f in fps.values():
        for p in f.Pads():
            if p.GetNetCode() > 0:
                by_net[p.GetNetname()].append((mm(p.GetPosition().x), mm(p.GetPosition().y)))
    total = sum(mst_length(v) for k, v in by_net.items() if k != "GND")
    rep.note(f"ratsnest (MST, all nets except GND): {total:.0f} mm over {len(by_net)} nets")
    return rep.finish(args.json)


if __name__ == "__main__":
    sys.exit(main())
