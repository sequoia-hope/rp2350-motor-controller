#!/usr/bin/env python3
"""autoroute.py -- route a board with ultraroute, governed, a few seeds in turn.

    autoroute.py hw/main/a --planes In1.Cu,In2.Cu      inner planes: no tracks there
    autoroute.py hw/main/a --planes In1.Cu --seeds 8 --timeout 600

What it does (references/routing.md):

  1. runs `ultraroute route` (vendor/ultraroute, built by scripts/route.sh) on
     the board with --strip: unlocked tracks and vias go, locked copper (the
     critical pre-routes, pad vias, a vendor copy) stays and is routed round.
     Rules and net classes come from the .kicad_pro beside the board; rule
     areas (keepouts) are honoured, tracks and vias separately
  2. passes --planes on, so nothing routes on a plane layer, other nets' vias
     pass through it, and its net counts as joined through the fill
  3. tries seeds 0, 1, ... one after another (the router is deterministic per
     seed, and seeds differ by one or two connections on a dense board),
     stopping at the first that KiCad judges complete
  4. judges every result with KiCad, not the router's own count: zones
     refilled, then `kicad-cli pcb drc`; signal connections left open (plane
     nets are joined by their pours after the fill and don't count) and
     errors the route added (any already on the board before routing, such as
     locked copper inside a routing keepout, don't count)
  5. keeps the best (clean first, then fewest open, then fewest vias), writes
     it over the board, and records every run in route/autoroute.json

Every run is governed (routing.md, "Running the autorouter without hurting
the host"): one at a time, `nice -n 15`, a systemd scope with a memory
ceiling and no swap when systemd-run is available, a wall-clock timeout, and
one thread unless --threads says otherwise (at most half the cores).

Zones that should not be obstacles (full-board pours on routing layers) must
not exist yet: add them after routing, then fill. Parts never move unless
--nudge is given (placement passed its gate; a move needs the user).
"""
from __future__ import annotations

import argparse
import collections
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from kicad_files import find_project  # noqa: E402

# What routed the example board and the ultraroute test boards completely:
# negotiated congestion, rip-up, the region plough for trapped pins (rips round
# a trapped path and renegotiates), fan-out of fine-pitch parts first, round by
# round, differential pairs coupled, and copper pulled tight.
RECIPE = ["--negotiate", "--rip-up", "--tighten", "--prefanout", "--prefanout-order", "round", "--diff-pairs",
          "--shove-plough", "region", "--shove-reach", "0.5", "--shove-repair", "20", "--shove-ties",
          "--shove-passes", "60", "--shove-patience", "8"]


def repo_root(start: Path) -> Path:
    for p in [start, *start.parents]:
        if (p / "scripts" / "route.sh").exists():
            return p
    sys.exit("autoroute.py: scripts/route.sh not found above the project (is this an easy_pcb repo?)")


def ultraroute(root: Path) -> Path:
    r = subprocess.run([str(root / "scripts" / "route.sh"), "--build"], capture_output=True, text=True)
    if r.returncode:
        sys.exit(r.stderr.strip())
    return Path(r.stdout.strip().splitlines()[-1])


def pcbnew_step(code: str, *args: str):
    """Run a short pcbnew snippet in its own process (one board per process)."""
    r = subprocess.run([sys.executable, "-c", code, *args], capture_output=True, text=True)
    if r.returncode:
        sys.exit(r.stderr[-3000:])
    return r.stdout


REFILL = """
import sys, pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
pcbnew.SaveBoard(sys.argv[1], b)
"""

PLANE_NETS = """
import sys, pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
planes = set(sys.argv[2].split(",")) - {""}
for z in b.Zones():
    if not z.GetIsRuleArea() and z.GetNetname() and any(b.GetLayerName(l) in planes for l in z.GetLayerSet().Seq()):
        print(z.GetNetname())
"""


def judge(pcb: Path, pro: Path, plane_nets: set[str]) -> dict:
    """KiCad's verdict on a board: refill a copy, DRC it."""
    with tempfile.TemporaryDirectory() as tmp:
        copy = Path(tmp) / pcb.name
        shutil.copy(pcb, copy)
        shutil.copy(pro, copy.with_suffix(".kicad_pro"))
        dru = pro.with_suffix(".kicad_dru")
        if dru.exists():
            shutil.copy(dru, copy.with_suffix(".kicad_dru"))
        pcbnew_step(REFILL, str(copy))
        out = Path(tmp) / "drc.json"
        subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-error", "-o", str(out),
                        str(copy)], capture_output=True, text=True)
        d = json.loads(out.read_text())
    def plane(u):
        return all(any(f"[{n}]" in i["description"] for n in plane_nets) for i in u["items"])
    opens = [" / ".join(i["description"] for i in u["items"]) for u in d.get("unconnected_items", []) if not plane(u)]
    errors = collections.Counter((v["type"], v["description"], tuple(sorted(i["description"] for i in v["items"])))
                                 for v in d["violations"])
    return {"opens": opens, "errors": errors}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("project", type=Path)
    ap.add_argument("--planes", default="", help="comma-separated plane layers (no tracks), e.g. In1.Cu,In2.Cu")
    ap.add_argument("--seeds", type=int, default=4, help="seeds to try in turn, stopping at a complete route")
    ap.add_argument("--timeout", type=int, default=240, help="seconds per run")
    ap.add_argument("--threads", type=int, default=1, help="router threads (capped at half the cores)")
    ap.add_argument("--memory", default="6G", help="memory ceiling per run")
    ap.add_argument("--nudge", metavar="LOCKED", help="let the router move parts, all but these (REF,...)")
    ap.add_argument("--extra", default="", help="more ultraroute options, as one string")
    args = ap.parse_args()

    threads = max(1, min(args.threads, (os.cpu_count() or 2) // 2))
    pro, _, pcb = find_project(args.project)
    exe = ultraroute(repo_root(pcb.parent.resolve()))
    out = pcb.parent / "route"
    out.mkdir(exist_ok=True)
    planes = [p for p in args.planes.split(",") if p]
    plane_nets = set(pcbnew_step(PLANE_NETS, str(pcb), ",".join(planes)).split())

    options = ["--strip", *RECIPE, "--threads", str(threads), "--timeout", str(args.timeout)]
    if planes:
        options += ["--planes", ",".join(planes)]
    if args.nudge:
        options += ["--nudge", "--lock", args.nudge]
    options += args.extra.split()

    before = judge(pcb, pro, plane_nets)["errors"]
    built = exe.parent.parent / ".built-from"
    version = built.read_text().strip()[:7] if built.exists() else str(exe)
    print(f"autoroute: {pcb.name}: ultraroute {version}; planes {planes or 'none'} "
          f"(nets {sorted(plane_nets) or 'none'}); {threads} thread(s), nice 15, {args.timeout} s per run",
          flush=True)
    results = []
    for seed in range(args.seeds):
        routed = out / f"s{seed}.kicad_pcb"
        cmd = [str(exe), "route", str(pcb), "-o", str(routed), *options, "--seed", str(seed)]
        gov = ["nice", "-n", "15", "timeout", str(args.timeout + 120)]
        if shutil.which("systemd-run"):
            gov = ["systemd-run", "--user", "--scope", "--quiet", "-p", f"MemoryMax={args.memory}",
                   "-p", "MemorySwapMax=0", "-p", f"CPUQuota={threads * 100}%"] + gov
        routed.unlink(missing_ok=True)
        r = subprocess.run(gov + cmd, capture_output=True, text=True)
        (out / f"s{seed}.log").write_text(r.stdout + r.stderr)
        if r.returncode or not routed.exists():
            print(f"  seed {seed}: router failed (exit {r.returncode}): {(r.stderr or r.stdout).strip()[-400:]}")
            if "unknown option" in r.stderr + r.stdout:
                return 2
            continue
        verdict = judge(routed, pro, plane_nets)
        new = verdict["errors"] - before
        vias = len(re.findall(r"^\s*\(via\b", routed.read_text(), re.M))
        res = {"seed": seed, "open": len(verdict["opens"]), "new_errors": sum(new.values()), "vias": vias,
               "router": (r.stdout + r.stderr).strip().splitlines()[-1] if (r.stdout + r.stderr).strip() else "",
               "open_items": verdict["opens"], "error_items": [" | ".join(k[2]) + f" ({k[0]})" for k in new]}
        results.append(res)
        print(f"  seed {seed}: {res['open']} signal open, {res['new_errors']} new errors, {vias} vias "
              f"({res['router']})", flush=True)
        for o in verdict["opens"]:
            print(f"    open: {o}")
        for e in res["error_items"]:
            print(f"    error: {e}")
        if res["open"] == 0 and res["new_errors"] == 0:
            break
    if not results:
        sys.exit("autoroute: no run produced a board")
    best = min(results, key=lambda r: (r["new_errors"] > 0, r["open"], r["new_errors"], r["vias"]))
    shutil.copy(out / f"s{best['seed']}.kicad_pcb", pcb)
    pcbnew_step(REFILL, str(pcb))
    (out / "autoroute.json").write_text(json.dumps({"options": options, "best": best["seed"], "runs": results},
                                                   indent=2) + "\n")
    print(f"  kept seed {best['seed']}: {best['open']} signal open, {best['new_errors']} new errors, "
          f"{best['vias']} vias; zones refilled")
    return 0 if best["open"] == 0 and best["new_errors"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
