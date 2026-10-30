#!/usr/bin/env python3
"""stitch.py -- stitching vias for pours and planes, placed where they fit.

    stitch.py hw/main/a --net GND --grid 1.5            via grid where GND pours overlap
    stitch.py hw/main/a --net GND --net +3V3 --fix      vias for whatever DRC says is cut off
    stitch.py hw/main/a --net GND --grid 1.5 --fix --via 0.5/0.2
    stitch.py hw/main/a --net GND --pads 'C*' --pads U3:6 --tag padvias --lock
    stitch.py hw/main/a --net VBUS --twin --tag twin                 a second via at each layer change

Before routing (references/routing.md step 3):

  --pads   REF or glob[:N]: a via of the net within --reach mm of every pad of
           those parts that is on a --net, beside the pad with a short stub;
           with :N, at least N vias around the part's pads on that net (an LDO
           or a switcher's ground). Decouplers whose ground pad only reaches
           the plane through a pour fail the review; a via at the pad, placed
           before routing, is the fix. Use a --tag of its own and --lock so the
           autorouter keeps them and a later --grid run leaves them alone.
           --wired counts only a via joined to the pad by a track: for a net
           with no outer pour (a 3V3 plane on L3), where a via of the next pad
           over isn't connected to this one.

After routing:

  --twin   a second via beside every via of a --net that joins tracks on two
           layers, linked to it on each of those layers: for a net whose
           current needs two vias per layer change (nets_check.py's "vias"
           column) when the router drew one. Clear of silk text, so run it
           after the silk.

After routing and pouring, two things are usually left:

  --grid   ground pours on the outer layers need vias to the plane, or they
           are islands joined only through a pad. A via goes at each grid
           point where its pad fits inside the net's filled copper on every
           outer layer the net is poured on, clear of keepouts and holes,
           and clear of silk text on either side (place the silk first:
           tented vias under text break the ink up).
  --fix    pads and tracks the router left on a plane net without a via
           (it thought a zone joined them). Reads `kicad-cli pcb drc` JSON,
           and for each unconnected pad or track on a --net, puts a via on
           the track's end or beside the pad with a short stub, at the first
           spot that clears every other net on every layer and the silk text
           (as --grid; so do the fragment vias). Repeats until DRC
           stops improving.

Vias are drawn with the net class's via size unless --via is given. Every
added via is in a group named "stitch" (or --tag) so a re-run replaces them;
with no mode, the run only removes that group. Zones are
refilled at the end. Run DRC afterwards: this script checks clearance with
the board's own shape tests, but DRC is the judge.
"""
from __future__ import annotations

import argparse
import fnmatch
import json
import math
import re
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from kicad_files import find_project, load_pro  # noqa: E402



def drc_unconnected(pcb: Path) -> list[dict]:
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "drc.json"
        subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-error", "-o", str(out),
                        str(pcb)], capture_output=True, text=True)
        data = json.loads(out.read_text())
    return data.get("unconnected_items", [])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("project", type=Path)
    ap.add_argument("--net", action="append", required=True)
    ap.add_argument("--grid", type=float, help="grid pitch in mm for pour stitching")
    ap.add_argument("--fix", action="store_true", help="add vias for unconnected pads/tracks")
    ap.add_argument("--via", help="diameter/drill in mm (default: the net class's)")
    ap.add_argument("--rounds", type=int, default=4)
    ap.add_argument("--twin", action="store_true", help="a second via beside each layer-changing via of --net")
    ap.add_argument("--pads", action="append", default=[], help="REF or glob[:N]: vias at these parts' pads")
    ap.add_argument("--reach", type=float, default=0.8, help="mm, pad edge to via edge, for --pads")
    ap.add_argument("--wired", action="store_true",
                    help="--pads: count only vias joined to the pad by a track (or in it), for a net with no "
                         "outer-layer pour to join a nearby via to the pad")
    ap.add_argument("--tag", default="stitch", help="group name; a re-run replaces this group only")
    ap.add_argument("--lock", action="store_true", help="lock the vias (pre-route: the autorouter keeps them)")
    args = ap.parse_args()
    TAG = args.tag
    import pcbnew

    _, _, pcb_path = find_project(args.project)
    b = pcbnew.LoadBoard(str(pcb_path))
    ds = b.GetDesignSettings()
    mm, FromMM = pcbnew.ToMM, pcbnew.FromMM
    copper = [l for l in range(pcbnew.PCB_LAYER_ID_COUNT) if pcbnew.IsCopperLayer(l) and b.IsLayerEnabled(l)]
    hole_gap = ds.m_HoleClearance if hasattr(ds, "m_HoleClearance") else FromMM(0.2)
    hole_to_hole = ds.m_HoleToHoleMin if hasattr(ds, "m_HoleToHoleMin") else FromMM(0.25)

    # Snapshot everything first (pcbnew proxies go bad after Remove()).
    groups = list(b.Groups())
    tracks = list(b.GetTracks())
    old = [t for t in tracks if t.GetParentGroup() is not None and t.GetParentGroup().GetName() == TAG]
    old += [g for g in groups if g.GetName() == TAG]
    # Another run's router may have threaded a track through one of these vias.
    # Snap such track ends to the via's centre first: ends that met inside it
    # can be nanometres apart, and without the via they no longer join.
    keep = [t for t in tracks if t.Type() == pcbnew.PCB_TRACE_T and t not in old
            and not t.IsLocked()]                     # locked copper (a vendor copy) keeps its geometry
    for v in [o for o in old if o.Type() == pcbnew.PCB_VIA_T]:
        c, r = v.GetPosition(), v.GetWidth(pcbnew.F_Cu) // 2
        for t in keep:
            if t.GetNetCode() != v.GetNetCode():
                continue
            if math.dist((t.GetStart().x, t.GetStart().y), (c.x, c.y)) <= r:
                t.SetStart(c)
            if math.dist((t.GetEnd().x, t.GetEnd().y), (c.x, c.y)) <= r:
                t.SetEnd(c)
    for item in old:
        b.Remove(item)
    tracks = [t for t in tracks if t not in old]
    zones = list(b.Zones())
    pads = [p for f in b.GetFootprints() for p in f.Pads()]
    group = pcbnew.PCB_GROUP(b)
    group.SetName(TAG)
    b.Add(group)
    added = []

    # Net classes from the project file (pcbnew 9's Python can't reach NETCLASS).
    pro = load_pro(pcb_path.with_suffix(".kicad_pro")).get("net_settings", {})
    classes = {c["name"]: c for c in pro.get("classes", [])}
    patterns = [(p["pattern"], p["netclass"]) for p in pro.get("netclass_patterns", [])]

    def netclass(net):
        name = net.GetNetname()
        for pat, cls in patterns:
            if fnmatch.fnmatchcase(name, pat) and cls in classes:
                return classes[cls]
        return classes.get("Default", {})

    def via_size(net):
        if args.via:
            d, h = (float(v) for v in args.via.split("/"))
            return FromMM(d), FromMM(h)
        c = netclass(net)
        return FromMM(c.get("via_diameter", 0.6)), FromMM(c.get("via_drill", 0.3))

    def clearance(net):
        return max(FromMM(netclass(net).get("clearance", 0.2)), ds.m_MinClearance)

    def track_width(net):
        return FromMM(netclass(net).get("track_width", 0.25))

    outline = pcbnew.SHAPE_POLY_SET()
    b.GetBoardPolygonOutlines(outline)
    edge_gap = ds.m_CopperEdgeClearance

    def fits(pos, net, dia, drill, stub=None):
        """True if a via (and its stub segment) clears every other net on every layer."""
        r = dia // 2
        if not outline.Contains(pos):
            return False
        if any(outline.COutline(i).SquaredDistance(pos, True) < (r + edge_gap) ** 2
               for i in range(outline.OutlineCount())):
            return False
        circ = pcbnew.SHAPE_CIRCLE(pos, r)
        seg = pcbnew.SHAPE_SEGMENT(stub[0], stub[1], stub[2]) if stub else None
        cl = clearance(net)
        for z in zones:
            if z.GetIsRuleArea() and z.GetDoNotAllowVias():
                if any(z.IsOnLayer(l) for l in copper) and z.Outline().Collide(pos, r):
                    return False
        for item in pads + tracks + added:
            hole = (item.GetDrillValue() if item.Type() == pcbnew.PCB_VIA_T
                    else item.GetDrillSizeX() if item.Type() == pcbnew.PCB_PAD_T and item.HasHole() else 0)
            if hole and math.dist((item.GetPosition().x, item.GetPosition().y), (pos.x, pos.y)) < \
                    (drill + hole) / 2 + hole_to_hole:
                return False
            if item.GetNetCode() == net.GetNetCode():
                continue
            need = max(cl, clearance(item.GetNet())) if item.GetNet() is not None else cl
            for l in copper:
                if not item.IsOnLayer(l):
                    continue
                shape = item.GetEffectiveShape(l)
                if shape.Collide(circ, need) or (seg and l == (stub[3] if len(stub) > 3 else pcbnew.F_Cu)
                                                 and shape.Collide(seg, need)):
                    return False
        # other nets' zones on outer layers that are not ordinary pours (e.g. a
        # copied regulator layout) are obstacles, not copper to flow around
        for z in zones:
            if z.GetIsRuleArea() or z.GetNetCode() == net.GetNetCode():
                continue
            if z.GetZoneName().startswith("plane"):
                continue
            for l in copper:
                if z.IsOnLayer(l) and z.Outline().Collide(pos, r + cl):
                    return False
        return True

    def add_via(pos, net, dia, drill, stub=None):
        v = pcbnew.PCB_VIA(b)
        v.SetPosition(pos)
        v.SetWidth(dia)
        v.SetDrill(drill)
        v.SetNet(net)
        v.SetLocked(args.lock)
        b.Add(v)
        group.AddItem(v)
        added.append(v)
        if stub:
            t = pcbnew.PCB_TRACK(b)
            t.SetStart(stub[0])
            t.SetEnd(stub[1])
            t.SetWidth(stub[2])
            t.SetLayer(stub[3] if len(stub) > 3 else pcbnew.F_Cu)
            t.SetNet(net)
            t.SetLocked(args.lock)
            b.Add(t)
            group.AddItem(t)
            added.append(t)

    nets = {n: b.FindNet(n) for n in args.net}
    for n, net in nets.items():
        if net is None:
            sys.exit(f"net {n!r} not on the board")

    pcbnew.ZONE_FILLER(b).Fill(zones)
    report = []

    if args.pads:
        reach = FromMM(args.reach)
        fps = {f.GetReference(): f for f in b.GetFootprints()}

        # Vias in the default "stitch" group don't count: the next --grid/--fix
        # run deletes and re-places them, and may not find the same spot.
        def lasting(v):
            g = v.GetParentGroup()
            return g is None or g.GetName() != "stitch"

        def near(q, net, dia):
            """Vias of the net within reach of pad q (pad edge to via edge); with
            --wired, only those a track joins to the pad (or inside it)."""
            layer = pcbnew.F_Cu if q.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
            shape = q.GetEffectiveShape(layer)
            found = [v for v in tracks + added if v.Type() == pcbnew.PCB_VIA_T and lasting(v)
                     and v.GetNetCode() == net.GetNetCode()
                     and shape.Collide(pcbnew.SHAPE_CIRCLE(v.GetPosition(), v.GetWidth(layer) // 2), reach)]
            if not args.wired:
                return found
            segs = [t for t in tracks + added if t.Type() == pcbnew.PCB_TRACE_T and t.GetNetCode() == net.GetNetCode()
                    and t.GetLayer() == layer]
            def wired(v):
                c = v.GetPosition()
                if shape.Collide(c, 0):
                    return True
                return any((shape.Collide(t.GetStart(), 0) and t.GetEnd() == c) or
                           (shape.Collide(t.GetEnd(), 0) and t.GetStart() == c) for t in segs)
            return [v for v in found if wired(v)]

        def beside(q, dia):
            """Via spots round pad q, nearest first: off each side and corner."""
            bb = q.GetBoundingBox()
            c = q.GetPosition()
            layer = pcbnew.F_Cu if q.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
            out = []
            for g in (0.12, 0.25, 0.4, 0.6):
                d = dia // 2 + FromMM(g)
                xs = [bb.GetLeft() - d, bb.GetRight() + d]
                ys = [bb.GetTop() - d, bb.GetBottom() + d]
                for f in (0.5, 0.25, 0.75, 0.0, 1.0):
                    x = int(bb.GetLeft() + f * bb.GetWidth())
                    y = int(bb.GetTop() + f * bb.GetHeight())
                    out += [pcbnew.VECTOR2I(x, ys[0]), pcbnew.VECTOR2I(x, ys[1]),
                            pcbnew.VECTOR2I(xs[0], y), pcbnew.VECTOR2I(xs[1], y)]
                out += [pcbnew.VECTOR2I(x, y) for x in xs for y in ys]
            out.sort(key=lambda p: math.dist((p.x, p.y), (c.x, c.y)))
            return [(p, layer) for p in out]

        for spec in args.pads:
            pat, _, want = spec.partition(":")
            for ref in sorted(r for r in fps if fnmatch.fnmatchcase(r, pat)):
                for n, net in nets.items():
                    mine = [q for q in fps[ref].Pads() if q.GetNetCode() == net.GetNetCode()]
                    if not mine:
                        continue
                    dia, drill = via_size(net)
                    have = lambda: {id(v) for q in mine for v in near(q, net, dia)}
                    need = int(want) if want else 0
                    new = []
                    for q in mine:                        # one in reach of every pad
                        if near(q, net, dia):
                            continue
                        for pos, layer in beside(q, dia):
                            stub = (q.GetPosition(), pos, track_width(net), layer)
                            if fits(pos, net, dia, drill, stub):
                                add_via(pos, net, dia, drill, stub)
                                new.append(pos)
                                break
                        else:
                            report.append(f"{ref} pad {q.GetNumber()} ({n}): no room for a via")
                    for q in mine:                        # then up to N round the part
                        for pos, layer in beside(q, dia):
                            if len(have()) >= need:
                                break
                            stub = (q.GetPosition(), pos, track_width(net), layer)
                            if fits(pos, net, dia, drill, stub):
                                add_via(pos, net, dia, drill, stub)
                                new.append(pos)
                    if want and len(have()) < need:
                        report.append(f"{ref} ({n}): {len(have())} vias, wanted {need}")
                    if new:
                        report.append(f"{ref} ({n}): {len(new)} vias added")
        pcbnew.ZONE_FILLER(b).Fill(zones)
        pcbnew.SaveBoard(str(pcb_path), b)

    silk_text = []
    for d in b.GetDrawings():
        if d.GetClass() == "PCB_TEXT" and d.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS):
            ps = pcbnew.SHAPE_POLY_SET()
            d.TransformTextToPolySet(ps, 0, FromMM(0.005), pcbnew.ERROR_INSIDE)
            silk_text.append(ps)

    def under_silk(pos, dia):
        return any(ps.Collide(pos, dia // 2 + FromMM(0.1)) for ps in silk_text)

    if args.twin:
        for n, net in nets.items():
            dia, drill = via_size(net)
            w = track_width(net)
            count = 0
            for v in [t for t in tracks if t.Type() == pcbnew.PCB_VIA_T and t.GetNetCode() == net.GetNetCode()]:
                c = v.GetPosition()
                layers = sorted({t.GetLayer() for t in tracks if t.Type() == pcbnew.PCB_TRACE_T
                                 and t.GetNetCode() == net.GetNetCode() and c in (t.GetStart(), t.GetEnd())})
                if len(layers) < 2:
                    continue
                if any(a.Type() == pcbnew.PCB_VIA_T and a.GetNetCode() == net.GetNetCode()
                       and math.dist((a.GetPosition().x, a.GetPosition().y), (c.x, c.y)) < FromMM(1.2)
                       for a in tracks + added if a is not v):
                    continue                              # already has a neighbour
                for d in (dia + FromMM(0.15), dia + FromMM(0.35)):
                    spots = [pcbnew.VECTOR2I(int(c.x + d * math.cos(math.radians(a))),
                                             int(c.y + d * math.sin(math.radians(a)))) for a in range(0, 360, 30)]
                    ok = [q for q in spots if not under_silk(q, dia)
                          and all(fits(q, net, dia, drill, (c, q, w, l)) for l in layers)]
                    if ok:
                        add_via(ok[0], net, dia, drill, (c, ok[0], w, layers[0]))
                        for l in layers[1:]:
                            t = pcbnew.PCB_TRACK(b)
                            t.SetStart(c)
                            t.SetEnd(ok[0])
                            t.SetWidth(w)
                            t.SetLayer(l)
                            t.SetNet(net)
                            t.SetLocked(args.lock)
                            b.Add(t)
                            group.AddItem(t)
                            added.append(t)
                        count += 1
                        break
                else:
                    report.append(f"{n}: no room for a second via at ({pcbnew.ToMM(c.x):.2f}, {pcbnew.ToMM(c.y):.2f})")
            report.append(f"{n}: {count} second vias")
        pcbnew.ZONE_FILLER(b).Fill(zones)
        pcbnew.SaveBoard(str(pcb_path), b)

    if args.grid:
        edge = b.GetBoardEdgesBoundingBox()
        step = FromMM(args.grid)
        for n, net in nets.items():
            dia, drill = via_size(net)
            pours = [z for z in zones if not z.GetIsRuleArea() and z.GetNetCode() == net.GetNetCode()
                     and (z.IsOnLayer(pcbnew.F_Cu) or z.IsOnLayer(pcbnew.B_Cu))]
            count = 0
            y = edge.GetTop() + step // 2
            while y < edge.GetBottom():
                x = edge.GetLeft() + step // 2
                while x < edge.GetRight():
                    pos = pcbnew.VECTOR2I(x, y)
                    # the via pad must sit wholly inside the fill on every outer layer poured
                    rim = [pos] + [pcbnew.VECTOR2I(int(x + (dia // 2 + FromMM(0.05)) * math.cos(a)),
                                                   int(y + (dia // 2 + FromMM(0.05)) * math.sin(a)))
                                   for a in (k * math.pi / 4 for k in range(8))]
                    layers = [l for l in (pcbnew.F_Cu, pcbnew.B_Cu) if any(z.IsOnLayer(l) for z in pours)]
                    inside = all(any(z.GetFilledPolysList(l).Contains(q) for z in pours if z.IsOnLayer(l))
                                 for q in rim for l in layers)
                    if inside and not under_silk(pos, dia) and fits(pos, net, dia, drill):
                        add_via(pos, net, dia, drill)
                        count += 1
                    x += step
                y += step
            # Every fragment of the pour needs at least one via: search a finer grid
            # inside any fragment the coarse grid missed.
            frag_added = 0
            for z in pours:
                for l in (pcbnew.F_Cu, pcbnew.B_Cu):
                    if not z.IsOnLayer(l):
                        continue
                    fill = z.GetFilledPolysList(l)
                    for i in range(fill.OutlineCount()):
                        frag = pcbnew.SHAPE_POLY_SET()
                        frag.AddOutline(fill.COutline(i))
                        joined = any(frag.Contains(v.GetPosition()) for v in tracks + added
                                     if v.Type() == pcbnew.PCB_VIA_T and v.GetNetCode() == net.GetNetCode()) or \
                            any(frag.Contains(q.GetPosition()) for q in pads
                                if q.GetNetCode() == net.GetNetCode() and q.HasHole())
                        if joined:
                            continue
                        bb = frag.BBox()
                        fine = FromMM(0.2)
                        done = False
                        yy = bb.GetTop()
                        while yy < bb.GetBottom() and not done:
                            xx = bb.GetLeft()
                            while xx < bb.GetRight() and not done:
                                pos = pcbnew.VECTOR2I(xx, yy)
                                rim = [pos] + [pcbnew.VECTOR2I(int(xx + (dia // 2 + FromMM(0.05)) * math.cos(a)),
                                                               int(yy + (dia // 2 + FromMM(0.05)) * math.sin(a)))
                                               for a in (k * math.pi / 4 for k in range(8))]
                                if all(frag.Contains(q) for q in rim) and not under_silk(pos, dia) \
                                        and fits(pos, net, dia, drill):
                                    add_via(pos, net, dia, drill)
                                    frag_added += 1
                                    done = True
                                xx += fine
                            yy += fine
            report.append(f"{n}: {count} grid vias at {args.grid} mm, {frag_added} more for pour fragments")
        pcbnew.ZONE_FILLER(b).Fill(zones)
        pcbnew.SaveBoard(str(pcb_path), b)

    if args.fix:
        for rnd in range(args.rounds):
            pcbnew.SaveBoard(str(pcb_path), b)
            items = drc_unconnected(pcb_path)
            todo = []
            for u in items:
                for it in u.get("items", []):
                    m = re.search(r"\[([^\]]+)\]", it.get("description", ""))
                    if m and m.group(1) in nets:
                        todo.append((m.group(1), it))
            if not todo:
                break
            fixed = 0
            seen = set()
            for n, it in todo:
                net = nets[n]
                dia, drill = via_size(net)
                p = it["pos"]
                at = pcbnew.VECTOR2I(FromMM(p["x"]), FromMM(p["y"]))
                key = (n, round(p["x"], 2), round(p["y"], 2))
                if key in seen:
                    continue
                seen.add(key)
                desc = it.get("description", "")
                cands = []
                if desc.startswith("Track"):
                    tr = [t for t in tracks if t.GetNetCode() == net.GetNetCode() and t.Type() != pcbnew.PCB_VIA_T
                          and (t.GetStart() == at or t.GetEnd() == at or
                               t.HitTest(at, FromMM(0.01)))]
                    for t in tr:
                        cands += [(t.GetStart(), None), (t.GetEnd(), None)]
                        # then beside the track, joined by a stub, every 0.25 mm along it
                        s, e = t.GetStart(), t.GetEnd()
                        length = max(1, int(math.dist((s.x, s.y), (e.x, e.y))))
                        ux, uy = (e.x - s.x) / length, (e.y - s.y) / length
                        k = 0
                        while k <= length:
                            base = pcbnew.VECTOR2I(int(s.x + ux * k), int(s.y + uy * k))
                            for d in (t.GetWidth() // 2 + dia // 2 + FromMM(0.15), t.GetWidth() // 2 + dia // 2 + FromMM(0.45)):
                                for sgn in (1, -1):
                                    pos = pcbnew.VECTOR2I(int(base.x - uy * d * sgn), int(base.y + ux * d * sgn))
                                    cands.append((pos, (base, pos, track_width(net))))
                            k += FromMM(0.25)
                elif desc.startswith("Pad") or desc.startswith("PTH pad"):
                    pad = [q for q in pads if q.GetNetCode() == net.GetNetCode() and q.HitTest(at, 0)]
                    if pad:
                        q = pad[0]
                        c = q.GetPosition()
                        half = max(q.GetBoundingBox().GetWidth(), q.GetBoundingBox().GetHeight()) // 2
                        for d in (half + dia // 2 + FromMM(0.15), half + dia // 2 + FromMM(0.4)):
                            for ang in range(0, 360, 30):
                                a = math.radians(ang)
                                pos = pcbnew.VECTOR2I(int(c.x + d * math.cos(a)), int(c.y + d * math.sin(a)))
                                cands.append((pos, (c, pos, track_width(net))))
                for pos, stub in cands:
                    if not under_silk(pos, dia) and fits(pos, net, dia, drill, stub):
                        add_via(pos, net, dia, drill, stub)
                        fixed += 1
                        break
            report.append(f"round {rnd + 1}: {len(todo)} unconnected items on {sorted(nets)}, {fixed} vias added")
            pcbnew.ZONE_FILLER(b).Fill(list(b.Zones()))
            if not fixed:
                break
        pcbnew.SaveBoard(str(pcb_path), b)

    if not (args.pads or args.twin or args.grid or args.fix):
        pcbnew.SaveBoard(str(pcb_path), b)          # no mode: just remove the --tag group
    print(f"stitch: {pcb_path.name}")
    for line in report:
        print(f"  {line}")
    print(f"  {len([a for a in added if a.Type() == pcbnew.PCB_VIA_T])} vias in group '{TAG}'; run DRC")
    return 0


if __name__ == "__main__":
    sys.exit(main())
