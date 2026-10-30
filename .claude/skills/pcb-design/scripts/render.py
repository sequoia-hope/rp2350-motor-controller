#!/usr/bin/env python3
"""render.py -- the gate renders for a KiCad project, in one command.

    render.py hw/main/a design/renders

Writes, from the board and schematic of the project folder:

  top.png, bottom.png     copper with the silk painted over it (+ fab on top),
                          bottom mirrored as seen from below
  <layer>.png             each copper layer alone, with the outline
  3d-top.png, 3d-bottom.png
  assembly-top.png/.pdf   F.Fab with pads sketched, ≈ 4:1 on A4: the assembly
                          drawing (designators readable)
  schematic.pdf

SVGs go through ImageMagick (`magick`; rsvg-convert is not needed). Run it at
every visual gate (phases 2, 6, 7, 9, 10) so the gate file and the reviewer
look at the board as it is now.
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from kicad_files import find_project  # noqa: E402


def run(cmd: list[str]) -> None:
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        sys.exit(f"render: {' '.join(cmd[:4])} failed:\n{r.stderr[-1500:]}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("project", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--width", type=int, default=900, help="PNG width in px")
    args = ap.parse_args()
    for tool in ("kicad-cli", "magick"):
        if not shutil.which(tool):
            sys.exit(f"render: {tool} not found")
    _, sch, pcb = find_project(args.project)
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    svg = ["kicad-cli", "pcb", "export", "svg", "--exclude-drawing-sheet", "--fit-page-to-board", "--mode-single"]
    import pcbnew
    b = pcbnew.LoadBoard(str(pcb))
    copper = [b.GetLayerName(l) for l in b.GetEnabledLayers().CuStack()]
    # Each view is a stack of plots painted bottom to top: KiCad paints one
    # plot's layers in its own order (silk under copper), which hides the silk.
    views = {"top": [["F.Cu", "F.Fab", "Edge.Cuts"], ["F.Silkscreen", "Edge.Cuts"]],
             "bottom": [["B.Cu", "Edge.Cuts"], ["B.Silkscreen", "Edge.Cuts"]]}
    views.update({l: [[l, "Edge.Cuts"]] for l in copper})
    with tempfile.TemporaryDirectory() as tmp:
        for name, stack in views.items():
            pngs = []
            for i, layers in enumerate(stack):
                s = Path(tmp) / f"{name}{i}.svg"
                run(svg + ["--layers", ",".join(layers)] + (["--mirror"] if name == "bottom" else [])
                    + ["-o", str(s), str(pcb)])
                pngs.append(Path(tmp) / f"{name}{i}.png")
                run(["magick", "-density", "900", "-background", "none", str(s),
                     "-resize", f"{args.width}x", str(pngs[-1])])
            cmd = ["magick", str(pngs[0])] + [x for p in pngs[1:] for x in (str(p), "-composite")]
            run(cmd + ["-background", "white", "-flatten", str(out / f"{name}.png")])
    for side in ("top", "bottom"):
        run(["kicad-cli", "pcb", "render", "--side", side, "--width", str(args.width),
             "--height", str(args.width * 2), "--quality", "high", "-o", str(out / f"3d-{side}.png"), str(pcb)])
    # The assembly drawing: F.Fab with pads sketched, big enough to read the
    # designators (kicad-cli's PDF export plots 1:1, where 0402 designators are
    # 0.4 mm). A 2400 px PNG (≈ 110 px/mm on a 21 mm board) and the same on A4.
    with tempfile.TemporaryDirectory() as tmp:
        s = Path(tmp) / "fab.svg"
        run(svg + ["--layers", "F.Fab,Edge.Cuts", "--sketch-pads-on-fab-layers", "-o", str(s), str(pcb)])
        run(["magick", "-density", "2400", str(s), "-background", "white", "-flatten",
             "-resize", "2400x", str(out / "assembly-top.png")])
    run(["magick", str(out / "assembly-top.png"), "-resize", "2200x3200", "-gravity", "center",
         "-background", "white", "-extent", "2480x3508", "-units", "PixelsPerInch", "-density", "300",
         str(out / "assembly-top.pdf")])
    if sch:
        run(["kicad-cli", "sch", "export", "pdf", "-o", str(out / "schematic.pdf"), str(sch)])
    print(f"render: {len(views) + 4 + bool(sch)} files in {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
