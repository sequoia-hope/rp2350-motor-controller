# KiCad adapter

Default for the user's open-source boards. Check the installed version first (`kicad-cli version`); APIs and file formats differ between 8, 9 and 10.

## Automation surfaces

- **`kicad-cli`**: ERC, DRC (`kicad-cli pcb drc --severity-all --format json`), exports (schematic PDF/SVG, gerbers, drill, BOM, CPL, STEP, 3D renders on recent versions). Use it for every check and export, and paste its output into gate files.
- **IPC API (kipy, KiCad 9+)** or legacy `pcbnew` Python: placement, zones, via arrays, net classes, footprint edits. Prefer the IPC API where available.
- **File formats**: `.kicad_sch` / `.kicad_pcb` are S-expressions. Parsing them directly is fine for checks (style, silk, sync scripts). For writes, prefer the API; if editing files directly, use a proper S-expression library, preserve UUIDs, and re-open in KiCad to validate.
- **Custom DRC rules**: `.kicad_dru` file with net class and area conditions. Put every derived rule here with a comment citing its source.
- **ultraroute** for autorouting (`vendor/ultraroute`, run through `scripts/route.sh` and the skill's `autoroute.py`): it reads and writes `.kicad_pcb` directly, takes rules and net classes from the `.kicad_pro`, keeps locked copper and honours rule areas. No DSN/SES round trip. Lock pre-routed tracks first. Always run it governed (see `references/routing.md`). Freerouting is no longer used.
- **Image converter / bitmap2component** for silk art.
- **JLC**: use the JLC fields (`LCSC`) on symbols and a rotation correction (e.g. the JLCPCB fabrication plugin or a per-part rotation offset field) for CPL.

## Schematic authoring

When generating a schematic programmatically, the coordinates are the design. Plan the layout per block on paper first (a sketch in the gate file: which symbols where, left to right), then place symbols at those positions on the 1.27 mm (50 mil) grid, then wires. Never place symbols in a loop over a list with a fixed x/y step: that is what produces the grid look.

Use `scripts/kicad_write.py` (`Schematic`) rather than writing S-expressions by hand: it embeds library symbols, computes pin positions, and splits wires at T-junctions (KiCad does not connect a pin or junction that sits mid-wire). Push to the board with `update_pcb()`, the headless equivalent of Update PCB from Schematic. Check every write with `kicad-cli sch erc` and the exported PDF.

## Board-local footprint edits

Copy the footprint into a project library (`<project>/lib/<project>.pretty`), edit, and re-link the symbol's footprint field in the schematic (schematic remains source of truth). Note it in `decisions.md`.

## pcbnew Python quirks (KiCad 9)

Each of these cost time on the example board; the scripts work around them.

- **One board per process.** A second `LoadBoard` in the same process can
  return a bare `SwigPyObject`. Run each board step as its own process.
- **After `BOARD.Remove()`, proxies go bad.** Lists and objects fetched before
  or after a removal can start yielding `SwigPyObject`s or garbage geometry
  for the rest of the process. Read everything you need (positions, nets,
  polygons) first, then remove, then add.
- **`ZONE.SetOutline(poly)` takes ownership.** Set `poly.thisown = False`
  first, or Python frees the polygon under the zone.
- **A zone holds one outline.** A boolean subtraction can leave slivers as
  extra outlines; keep the largest, or the zone saves as the sliver.
- **Flipped footprints rotate the other way.** A header flipped to the back
  needs 180° where the front needed 0° to run in the same direction.
- **`ImportSpecctraSES` takes a via's drill from the net class,** not the
  padstack the session chose (thin rings). Only matters if a Specctra router
  is ever used again; ultraroute writes the board itself.
- **KiCad 7 boards keep reference and value as `fp_text`,** not properties;
  `kicad_files.load_board` reads both.
- **`NETINFO_ITEM.GetNetClass()` is not wrapped.** Read net classes and
  patterns from the `.kicad_pro` JSON instead.
- **A text's `GetBoundingBox()` is ~1.8× its height** (0.8 mm text gives a
  1.46 mm box). For placement, use the outline of the strokes:
  `TransformTextToPolySet(...)` then `.BBox()`, and `Collide()` the polygon
  with vias.
- **A rule area on every copper layer:** `zone.SetLayerSet(pcbnew.LSET.AllCuMask())`;
  `SetLayer` gives a keepout on one layer only.
- **Clear a scripted group's items yourself.** Removing a `PCB_GROUP` leaves
  its tracks and vias on the board, ungrouped; a script that finds its old
  copper by group then can't see them. Remove the items, then the group.

## Schematic quirks

- A plain label named like a power net (`GND`, `+3V3`) joins that power net.
  Convenient on a connector pinout, but draw rails with power symbols
  wherever a wire can reach.
- The field angle of a rotated symbol is relative to the symbol:
  `kicad_write` gives rotated symbols matching field angles so the text
  reads horizontally, and swaps left/right justification for mirrored ones.

