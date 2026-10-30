#!/usr/bin/env python3
"""gerber_render.py -- render the fab gerbers layer by layer, outside KiCad.

    gerber_render.py hw/main/a/fab/gerbers design/renders/gerbers

The phase 10 gerber review (references/sync-and-release.md) looks at what the
fab will read, not at the CAD board. This parses the gerbers and drill files
with gerbonara (an independent parser), prints what it found per layer (object
counts, holes, outline) and writes one PNG per layer plus `sheet.png`, all
layers side by side.

Needs gerbonara (`pip install gerbonara`, in a venv if the system Python is
managed: run this script with that venv's python) and Chrome or Chromium for
the raster: ImageMagick's own SVG renderer drops gerbonara's strokes and shows
blank silk.
"""
from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

try:
    from gerbonara import LayerStack
except ImportError:
    sys.exit("gerber_render needs gerbonara: python3 -m venv .venv && .venv/bin/pip install gerbonara, "
             "then run this script with .venv/bin/python")

ORDER = [("top", "silk"), ("top", "paste"), ("top", "mask"), ("top", "copper"), ("inner_1", "copper"),
         ("inner_2", "copper"), ("bottom", "copper"), ("bottom", "mask"), ("bottom", "paste"),
         ("bottom", "silk")]


def chrome() -> str:
    for name in ("google-chrome", "chromium", "chromium-browser"):
        if shutil.which(name):
            return name
    sys.exit("gerber_render needs Chrome or Chromium for the raster (headless)")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("gerbers", type=Path, help="folder of gerber and drill files")
    ap.add_argument("out", type=Path)
    ap.add_argument("--width", type=int, default=420, help="px per layer")
    args = ap.parse_args()
    browser = chrome()
    import warnings
    warnings.simplefilter("ignore")
    st = LayerStack.open(args.gerbers)
    args.out.mkdir(parents=True, exist_ok=True)
    layers = [(f"{s}_{u}", st.graphic_layers[(s, u)]) for s, u in ORDER if (s, u) in st.graphic_layers]
    layers += [(f"drill_{d.original_path.stem}", d) for d in st.drill_layers]
    if st.outline:
        layers.append(("outline", st.outline))
    print(f"gerber_render: {args.gerbers} (board {st.board_name})")
    pngs = []
    with tempfile.TemporaryDirectory() as tmp:
        for name, layer in layers:
            src = layer.original_path.name if layer.original_path else "?"
            print(f"  {name:20s} {src:32s} {len(layer.objects):5d} objects")
            if not layer.objects:
                continue
            svg = str(layer.to_svg(fg="black", bg="white", margin=1))
            w = re.search(r'width="([\d.]+)mm"', svg)
            h = re.search(r'height="([\d.]+)mm"', svg)
            ph = int(args.width * float(h.group(1)) / float(w.group(1))) if w and h else args.width * 2
            svg = re.sub(r'width="[\d.]+mm" height="[\d.]+mm"', f'width="{args.width}" height="{ph}"', svg, 1)
            f = Path(tmp) / f"{name}.svg"
            f.write_text(svg)
            png = args.out / f"{name}.png"
            subprocess.run([browser, "--headless=new", "--disable-gpu", "--no-sandbox", "--hide-scrollbars",
                            f"--window-size={args.width},{ph}", f"--screenshot={png}", f.as_uri()],
                           capture_output=True, timeout=120)
            if png.exists():
                pngs.append(png)
    if pngs and shutil.which("magick"):
        subprocess.run(["magick", *map(str, pngs), "+append", str(args.out / "sheet.png")], check=False)
    print(f"  {len(pngs)} layer images in {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
