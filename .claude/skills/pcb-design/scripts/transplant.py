#!/usr/bin/env python3
"""transplant.py -- copy a block of a reference board into this one, relative to an anchor part.

    transplant.py REF.kicad_pcb OURS.kicad_pcb --anchor U1=U1 \\
        --refs L1=L1,C6=C14,C7=C15 --nets +1V1=+1V1,+3V3=+3V3,GND=GND,/VREG_LX=VREG_LX \\
        --box -8,-9,8,10 [--exclude x0,y0,x1,y1;...] [--clip] [--tag NAME]
        [--layers F.Cu=F.Cu] [--dry-run]

For layouts a vendor says to copy, such as the RP2350's regulator
(chips/rp2350.md): "transplant, don't redraw". Positions are taken relative to
the anchor (REF's part = OUR part) in the reference and replayed relative to
ours, rotation included. It copies:

  footprints  each --refs pair: position, rotation (our part keeps its own
              footprint; the two must match pad for pad)
  zones       every zone on a --nets net whose outline lies inside --box (mm,
              anchor frame), with priority, pad connection, clearance,
              minimum width, thermal gap and spoke width
  tracks/vias every track or via on a --nets net inside --box

--clip keeps the inside part of a track that crosses the box edge instead of
dropping it: use it to copy a reference's pin escapes (fan-out) and let the
router continue from the cut ends. --tag names the copy (default
"transplant"); a re-run replaces only items with the same tag, so a power
block and a fan-out can be copied separately.

--exclude drops any item touching one of the given rectangles (anchor frame):
use it for the reference's tracks that lead out of the copied block, or for
vias that would land on our connectors.

Copied items are locked so the autorouter can't move them. Zones are named
"transplant ..." and tracks and vias go in a group named "transplant", so a
re-run replaces them rather than duplicating. Run DRC afterwards.

The reference is read in a child process (pcbnew can't load two projects in
one process).
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

TAG = "transplant"   # default zone-name prefix and group name for what this script adds


def pairs(s: str | None) -> dict[str, str]:
    out = {}
    for item in (s or "").split(","):
        if item.strip():
            a, b = item.split("=", 1)
            out[a.strip()] = b.strip()
    return out


def clip(s, e, box):
    """Liang-Barsky: the part of segment s-e inside box, or None."""
    (x0, y0), (x1, y1) = s, e
    dx, dy = x1 - x0, y1 - y0
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, x0 - box[0]), (dx, box[2] - x0), (-dy, y0 - box[1]), (dy, box[3] - y0)):
        if p == 0:
            if q < 0:
                return None
            continue
        r = q / p
        if p < 0:
            t0 = max(t0, r)
        else:
            t1 = min(t1, r)
        if t0 > t1:
            return None
    return (x0 + t0 * dx, y0 + t0 * dy), (x0 + t1 * dx, y0 + t1 * dy)


def extract(ref_pcb: str, anchor: str, refs: list[str], nets: list[str], box, exclude=(),
            clipped=False) -> dict:
    import pcbnew
    b = pcbnew.LoadBoard(ref_pcb)
    fps = {f.GetReference(): f for f in b.GetFootprints()}
    a = fps[anchor]
    ax, ay = pcbnew.ToMM(a.GetPosition().x), pcbnew.ToMM(a.GetPosition().y)
    arot = a.GetOrientationDegrees()

    def rel(p):
        x, y = pcbnew.ToMM(p.x) - ax, pcbnew.ToMM(p.y) - ay
        t = math.radians(arot)       # undo the anchor's rotation (KiCad: CCW on screen, y down)
        return (x * math.cos(t) - y * math.sin(t), x * math.sin(t) + y * math.cos(t))

    def inside(pts):
        if any(e[0] <= x <= e[2] and e[1] <= y <= e[3] for e in exclude for x, y in pts):
            return False
        return all(box[0] <= x <= box[2] and box[1] <= y <= box[3] for x, y in pts)

    out = {"anchor_rot": arot, "footprints": {}, "zones": [], "tracks": [], "vias": []}
    for r in refs:
        f = fps[r]
        out["footprints"][r] = {"at": rel(f.GetPosition()), "rot": f.GetOrientationDegrees() - arot,
                                "side": "B" if f.IsFlipped() else "F", "fpid": f.GetFPIDAsString()}
    for z in b.Zones():
        if z.GetNetname() not in nets or z.GetIsRuleArea():
            continue
        poly = z.Outline()
        pts = [rel(poly.CVertex(i)) for i in range(poly.TotalVertices())]
        if not inside(pts):
            continue
        out["zones"].append({
            "net": z.GetNetname(), "layer": b.GetLayerName(z.GetLayer()), "pts": pts,
            "priority": z.GetAssignedPriority(), "connection": int(z.GetPadConnection()),
            "clearance": pcbnew.ToMM(z.GetLocalClearance()), "min_width": pcbnew.ToMM(z.GetMinThickness()),
            "thermal_gap": pcbnew.ToMM(z.GetThermalReliefGap()),
            "spoke": pcbnew.ToMM(z.GetThermalReliefSpokeWidth()), "islands": int(z.GetIslandRemovalMode()),
        })
    for t in b.GetTracks():
        if t.GetNetname() not in nets:
            continue
        if t.Type() == pcbnew.PCB_VIA_T:
            p = rel(t.GetPosition())
            if inside([p]):
                out["vias"].append({"net": t.GetNetname(), "at": p, "size": pcbnew.ToMM(t.GetWidth()),
                                    "drill": pcbnew.ToMM(t.GetDrillValue())})
        else:
            s, e = rel(t.GetStart()), rel(t.GetEnd())
            if clipped and not inside([s, e]):
                c = clip(s, e, box)
                if c is None or math.dist(*c) < 0.05:
                    continue
                s, e = c
            if inside([s, e]):
                out["tracks"].append({"net": t.GetNetname(), "layer": b.GetLayerName(t.GetLayer()),
                                      "start": s, "end": e, "width": pcbnew.ToMM(t.GetWidth())})
    return out


def apply(our_pcb: str, anchor: str, data: dict, refmap: dict, netmap: dict, layermap: dict, log=print,
          tag: str = TAG):
    import pcbnew
    b = pcbnew.LoadBoard(our_pcb)
    fps = {f.GetReference(): f for f in b.GetFootprints()}
    a = fps[anchor]
    ax, ay = pcbnew.ToMM(a.GetPosition().x), pcbnew.ToMM(a.GetPosition().y)
    arot = a.GetOrientationDegrees()

    def put(p):
        t = -math.radians(arot)
        x, y = p[0] * math.cos(t) - p[1] * math.sin(t), p[0] * math.sin(t) + p[1] * math.cos(t)
        return pcbnew.VECTOR2I_MM(ax + x, ay + y)

    def layer(name):
        return b.GetLayerID(layermap.get(name, name))

    def net(name):
        n = b.FindNet(netmap[name])
        if n is None:
            raise SystemExit(f"net {netmap[name]!r} is not on {our_pcb}")
        return n

    for src, dst in refmap.items():
        f, d = fps[dst], data["footprints"][src]
        if d["side"] != ("B" if f.IsFlipped() else "F"):
            raise SystemExit(f"{src} is on the {d['side']} side of the reference; {dst} is not")
        f.SetPosition(put(d["at"]))
        f.SetOrientationDegrees(d["rot"] + arot)
        f.SetLocked(True)
        log(f"  {dst} <- {src} ({d['fpid']})")

    # Snapshot every list before removing anything: after a Remove(), pcbnew's
    # SWIG wrappers can stop yielding typed objects for the rest of the process.
    zones, tracks, groups = list(b.Zones()), list(b.GetTracks()), list(b.Groups())
    old = [z for z in zones if z.GetZoneName().startswith(tag + " ")]
    old += [t for t in tracks if t.GetParentGroup() is not None and t.GetParentGroup().GetName() == tag]
    old += [g for g in groups if g.GetName() == tag]
    for item in old:
        b.Remove(item)
    removed = len(old)
    group = pcbnew.PCB_GROUP(b)
    group.SetName(tag)
    b.Add(group)

    for i, z in enumerate(data["zones"]):
        zone = pcbnew.ZONE(b)
        zone.SetZoneName(f"{tag} {i} {z['net']}")
        zone.SetLayer(layer(z["layer"]))
        zone.SetNet(net(z["net"]))
        ol = zone.Outline()
        ol.NewOutline()
        for p in z["pts"]:
            v = put(p)
            ol.Append(v.x, v.y)
        zone.SetAssignedPriority(z["priority"])
        zone.SetPadConnection(z["connection"])
        zone.SetLocalClearance(pcbnew.FromMM(z["clearance"]))
        zone.SetMinThickness(pcbnew.FromMM(z["min_width"]))
        zone.SetThermalReliefGap(pcbnew.FromMM(z["thermal_gap"]))
        zone.SetThermalReliefSpokeWidth(pcbnew.FromMM(z["spoke"]))
        zone.SetIslandRemovalMode(z["islands"])
        zone.SetLocked(True)
        b.Add(zone)
    for t in data["tracks"]:
        tr = pcbnew.PCB_TRACK(b)
        tr.SetStart(put(t["start"]))
        tr.SetEnd(put(t["end"]))
        tr.SetWidth(pcbnew.FromMM(t["width"]))
        tr.SetLayer(layer(t["layer"]))
        tr.SetNet(net(t["net"]))
        tr.SetLocked(True)
        b.Add(tr)
        group.AddItem(tr)
    for v in data["vias"]:
        via = pcbnew.PCB_VIA(b)
        via.SetPosition(put(v["at"]))
        via.SetWidth(pcbnew.FromMM(v["size"]))
        via.SetDrill(pcbnew.FromMM(v["drill"]))
        via.SetNet(net(v["net"]))
        via.SetLocked(True)
        b.Add(via)
        group.AddItem(via)
    pcbnew.SaveBoard(our_pcb, b)
    log(f"  {len(refmap)} parts, {len(data['zones'])} zones, {len(data['tracks'])} tracks, "
        f"{len(data['vias'])} vias copied ({removed} items from an earlier run replaced)")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ref_pcb")
    ap.add_argument("our_pcb", nargs="?")
    ap.add_argument("--anchor", required=True, help="REF=OURS, e.g. U1=U1")
    ap.add_argument("--refs", default="", help="reference part = our part, comma-separated")
    ap.add_argument("--nets", required=True, help="reference net = our net, comma-separated")
    ap.add_argument("--layers", default="", help="reference layer = our layer (default: same name)")
    ap.add_argument("--box", required=True, help="x0,y0,x1,y1 in mm around the anchor (anchor frame)")
    ap.add_argument("--exclude", default="", help="x0,y0,x1,y1;... rectangles to leave out (anchor frame)")
    ap.add_argument("--clip", action="store_true", help="keep the inside part of tracks crossing the box")
    ap.add_argument("--tag", default=TAG, help="name for this copy (zones and group)")
    ap.add_argument("--extract", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--dry-run", action="store_true", help="print what would be copied")
    args = ap.parse_args()

    src_anchor, dst_anchor = args.anchor.split("=")
    refmap, netmap = pairs(args.refs), pairs(args.nets)
    box = [float(v) for v in args.box.split(",")]
    exclude = [[float(v) for v in r.split(",")] for r in args.exclude.split(";") if r.strip()]
    if args.extract:
        print(json.dumps(extract(args.ref_pcb, src_anchor, list(refmap), list(netmap), box, exclude, args.clip)))
        return 0
    cmd = [sys.executable, __file__, args.ref_pcb, "--extract", "--anchor", args.anchor,
           "--refs", args.refs, "--nets", args.nets, "--box", args.box, "--exclude", args.exclude]
    if args.clip:
        cmd.append("--clip")
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        sys.exit(r.stderr)
    data = json.loads(r.stdout.strip().splitlines()[-1])
    if args.dry_run or not args.our_pcb:
        print(json.dumps(data, indent=1))
        return 0
    apply(args.our_pcb, dst_anchor, data, refmap, netmap, pairs(args.layers), tag=args.tag)
    return 0


if __name__ == "__main__":
    sys.exit(main())
