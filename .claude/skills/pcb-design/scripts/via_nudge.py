#!/usr/bin/env python3
"""via_nudge.py -- move router vias out from under silk text.

    via_nudge.py hw/main/a                 move every unlocked via under silk text
    via_nudge.py hw/main/a --dry-run       report them only

Reserving the art zone before routing (references/silkscreen-and-art.md) keeps
vias off most fixed text, but not off text that can't be reserved without
blocking the router (the example's header labels beside the flash escape).
After routing, a via may land under such a label. This moves it:

  * a via counts as "under" text when the text strokes come within
    [silk].via_clearance_mm (0.1) of its pad, the same test silk_check.py makes
  * a via inside a no-via rule area (a reservation added after it was routed)
    moves too, and so does a via DRC reports in a clearance error
  * unlocked vias move, and pad vias (locked, in a group named "padvias...")
    with their stub; other locked copper (pre-routes, a vendor copy) stays,
    and so does any via a locked track ends on
  * candidate spots are rings round the via, nearest first, up to --reach mm;
    the via moves with the ends of every track that ends on it, on each layer
  * a spot must clear the silk text, the board edge, keepouts, holes, and every
    other net on every copper layer, for the via and each dragged track
  * after moving, `kicad-cli pcb drc` must show no more errors or unconnected
    items than before, or nothing is saved
  * it reports what it couldn't move and exits 0: silk_check.py is the gate

Run it after the silk is placed and before the stitching vias and final fill.
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from kicad_files import find_project, load_checks, load_pro, repo_root  # noqa: E402


def drc_counts(pcb: Path) -> tuple[int, int]:
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "drc.json"
        subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-error", "-o", str(out),
                        str(pcb)], capture_output=True, text=True)
        d = json.loads(out.read_text())
    return len(d.get("violations", [])), len(d.get("unconnected_items", []))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("project", type=Path)
    ap.add_argument("--reach", type=float, default=2.5, help="mm, furthest a via may move")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    import pcbnew
    F, T = pcbnew.FromMM, pcbnew.ToMM

    pro, _, pcb_path = find_project(args.project)
    try:
        gap = F(load_checks(repo_root(pro.parent))["silk"].get("via_clearance_mm", 0.1))
    except Exception:
        gap = F(0.1)
    b = pcbnew.LoadBoard(str(pcb_path))
    ds = b.GetDesignSettings()
    copper = [l for l in range(pcbnew.PCB_LAYER_ID_COUNT) if pcbnew.IsCopperLayer(l) and b.IsLayerEnabled(l)]
    hole_to_hole = getattr(ds, "m_HoleToHoleMin", F(0.25))
    edge_gap = ds.m_CopperEdgeClearance
    outline = pcbnew.SHAPE_POLY_SET()
    b.GetBoardPolygonOutlines(outline)

    ns = load_pro(pro).get("net_settings", {})
    classes = {c["name"]: c for c in ns.get("classes", [])}
    patterns = [(p["pattern"], p["netclass"]) for p in ns.get("netclass_patterns", [])]

    def clearance(net):
        import fnmatch
        name = net.GetNetname() if net is not None else ""
        for pat, cls in patterns:
            if fnmatch.fnmatchcase(name, pat) and cls in classes:
                return max(F(classes[cls].get("clearance", 0.1)), ds.m_MinClearance)
        return max(F(classes.get("Default", {}).get("clearance", 0.1)), ds.m_MinClearance)

    texts = []
    for d in b.GetDrawings():
        if d.GetClass() == "PCB_TEXT" and d.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS):
            ps = pcbnew.SHAPE_POLY_SET()
            d.TransformTextToPolySet(ps, 0, F(0.005), pcbnew.ERROR_INSIDE)
            texts.append((d.GetText(), ps))
    for fp in b.GetFootprints():
        if fp.Reference().IsVisible() and fp.Reference().GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS):
            ps = pcbnew.SHAPE_POLY_SET()
            fp.Reference().TransformTextToPolySet(ps, 0, F(0.005), pcbnew.ERROR_INSIDE)
            texts.append((fp.GetReference(), ps))

    zones = list(b.Zones())
    keepouts = [z for z in zones if z.GetIsRuleArea() and z.GetDoNotAllowVias()]

    def under(pos, r):
        return [s for s, ps in texts if ps.Collide(pos, r + gap)] + \
            [z.GetZoneName() for z in keepouts if z.Outline().Collide(pos, r)]

    def movable(v):
        g = v.GetParentGroup()
        return not v.IsLocked() or (g is not None and g.GetName().startswith("padvias"))

    items = list(b.GetTracks())
    pads = [p for f in b.GetFootprints() for p in f.Pads()]
    # and vias DRC reports too close to another net (the router's rounding)
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "drc.json"
        subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-error", "-o", str(out),
                        str(pcb_path)], capture_output=True, text=True)
        crowded = {i["uuid"] for v in json.loads(out.read_text()).get("violations", [])
                   if v["type"] == "clearance" for i in v["items"] if i["description"].startswith("Via")}
    vias = [v for v in items if v.Type() == pcbnew.PCB_VIA_T and movable(v)
            and (under(v.GetPosition(), v.GetWidth(pcbnew.F_Cu) // 2) or v.m_Uuid.AsString() in crowded)]
    print(f"via_nudge: {pcb_path.name}: {len(vias)} via(s) under silk text, in a no-via area or too close")
    if not vias or args.dry_run:
        for v in vias:
            p = v.GetPosition()
            print(f"  {v.GetNetname()} at ({T(p.x):.2f}, {T(p.y):.2f}) under {under(p, v.GetWidth(pcbnew.F_Cu) // 2)}")
        return 0

    def fits(v, pos, legs):
        """legs: [(track, end_name)] that move with the via."""
        r, drill, net = v.GetWidth(pcbnew.F_Cu) // 2, v.GetDrillValue(), v.GetNet()
        if not outline.Contains(pos) or any(outline.COutline(i).SquaredDistance(pos, True) < (r + edge_gap) ** 2
                                           for i in range(outline.OutlineCount())):
            return False
        if under(pos, r):
            return False
        circ = pcbnew.SHAPE_CIRCLE(pos, r)
        moved = {id(t) for t, _ in legs}
        segs = []
        for t, end in legs:
            a = pos if end == "start" else t.GetStart()
            c = pos if end == "end" else t.GetEnd()
            segs.append((t.GetLayer(), pcbnew.SHAPE_SEGMENT(a, c, t.GetWidth())))
            for z in zones:
                if z.GetIsRuleArea() and z.GetDoNotAllowTracks() and z.IsOnLayer(t.GetLayer()) \
                        and z.Outline().Collide(pcbnew.SHAPE_SEGMENT(a, c, t.GetWidth())):
                    return False
        cl = clearance(net)
        for it in pads + items:
            if it is v or id(it) in moved:
                continue
            hole = (it.GetDrillValue() if it.Type() == pcbnew.PCB_VIA_T
                    else it.GetDrillSizeX() if it.Type() == pcbnew.PCB_PAD_T and it.HasHole() else 0)
            if hole and math.dist((it.GetPosition().x, it.GetPosition().y), (pos.x, pos.y)) < \
                    (drill + hole) / 2 + hole_to_hole:
                return False
            if it.GetNetCode() == net.GetNetCode():
                continue
            need = max(cl, clearance(it.GetNet()))
            for l in copper:
                if not it.IsOnLayer(l):
                    continue
                sh = it.GetEffectiveShape(l)
                if sh.Collide(circ, need):
                    return False
                if any(sl == l and sh.Collide(sg, need) for sl, sg in segs):
                    return False
        for z in zones:                             # other nets' fixed zones (not plane pours)
            if z.GetIsRuleArea() or z.GetNetCode() == net.GetNetCode() or z.GetZoneName().startswith("plane"):
                continue
            for l in copper:
                if z.IsOnLayer(l) and (z.Outline().Collide(pos, r + cl)
                                       or any(sl == l and z.Outline().Collide(sg, cl) for sl, sg in segs)):
                    return False
        return True

    moves = []
    for v in vias:
        c = v.GetPosition()
        r0 = v.GetWidth(pcbnew.F_Cu) // 2           # track ends can sit nanometres off the centre
        near = lambda q: math.dist((q.x, q.y), (c.x, c.y)) <= r0
        legs = [(t, "start" if near(t.GetStart()) else "end") for t in items
                if t.Type() == pcbnew.PCB_TRACE_T and t.GetNetCode() == v.GetNetCode()
                and (near(t.GetStart()) or near(t.GetEnd()))]
        if any(t.IsLocked() and not movable(t) for t, _ in legs):
            print(f"  {v.GetNetname()} at ({T(c.x):.2f}, {T(c.y):.2f}): joined to locked copper; left")
            continue
        spots = sorted(((d, k) for d in range(1, int(args.reach / 0.1) + 1) for k in range(16)),
                       key=lambda dk: dk[0])
        for d, k in spots:
            a = 2 * math.pi * k / 16
            pos = pcbnew.VECTOR2I(int(c.x + F(0.1 * d) * math.cos(a)), int(c.y + F(0.1 * d) * math.sin(a)))
            if fits(v, pos, legs):
                moves.append((v, c, pos, legs))
                v.SetPosition(pos)
                for t, end in legs:
                    (t.SetStart if end == "start" else t.SetEnd)(pos)
                print(f"  {v.GetNetname()}: ({T(c.x):.2f}, {T(c.y):.2f}) -> ({T(pos.x):.2f}, {T(pos.y):.2f}), "
                      f"{0.1 * d:.1f} mm")
                break
        else:
            print(f"  {v.GetNetname()} at ({T(c.x):.2f}, {T(c.y):.2f}): no spot within {args.reach} mm")
    if not moves:
        return 0                                    # reported above; silk_check fails the board
    pcbnew.ZONE_FILLER(b).Fill(zones)
    before = drc_counts(pcb_path)
    with tempfile.TemporaryDirectory() as tmp:
        trial = Path(tmp) / pcb_path.name
        shutil.copy(pro, trial.with_suffix(".kicad_pro"))
        pcbnew.SaveBoard(str(trial), b)
        after = drc_counts(trial)
    if after[0] > before[0] or after[1] > before[1]:
        print(f"via_nudge: DRC would go from {before} to {after} (errors, unconnected); nothing saved")
        return 0
    pcbnew.SaveBoard(str(pcb_path), b)
    print(f"via_nudge: moved {len(moves)} via(s); DRC errors/unconnected {before} -> {after}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
