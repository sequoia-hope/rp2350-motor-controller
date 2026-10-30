# Phase 10: sync and release

## Schematic is the source of truth

Values, footprints, MPNs and supplier part numbers are changed **only in the schematic**, then pushed to the PCB (KiCad: Update PCB from Schematic; Altium: ECO). Never edit them in the PCB editor. Never edit the BOM by hand.

## Sync check (`scripts/sync_check.py`) — run at every gate from phase 6 on

```sh
python3 .claude/skills/pcb-design/scripts/sync_check.py hw/<board>/<variant>
python3 .claude/skills/pcb-design/scripts/sync_check.py hw/<board>/<variant> --bom <bom.csv> --cpl <cpl.csv>   # release
```

Compares, for every reference designator, across schematic, PCB and generated BOM/CPL:
- present in all three (no orphans)
- value
- footprint (library ID and, for board-local edits, that the base footprint matches)
- MPN, manufacturer, LCSC/supplier PN
- DNP flag
Fails on any mismatch. Also confirms every BOM part number resolves to an in-stock part (re-check stock at release).

## Release outputs

- Gerbers + drill (fab's preferred settings; for JLC use their KiCad/Altium guides)
- BOM in fab format
- CPL / pick-and-place
- Schematic PDF, assembly drawing PDF (top/bottom, with designators even if hidden on silk)
- 3D STEP
- `design/` folder, with gates and decisions
- Version tag in git: `<board>-rev<X>`

## Assembly drawing

With designators off the silk (policy "none" or "selective"), the assembly
drawing is how people find parts, so it must be readable: `render.py` writes
F.Fab at ≈ 4:1 on A4 (`kicad-cli pcb export pdf` plots 1:1, where 0402
designators are 0.4 mm high).

## Stackup on the order

The gerber job file carries the KiCad board's stackup table, often still
KiCad's default. The fab builds what the order form says: name the fab's
stackup code there, and put it in `stackup.md`.

## CPL rotation check

Rotation and origin offsets between library footprints and JLC's part orientation are a classic silent failure. Use JLC's rotation correction approach (e.g. a rotation offset field per part or a correction database), then render each part's assembly preview (or the CPL overlaid on the board image) and verify pin 1 and polarity for **every** polarized part and IC. Paste the list.

## Gerber review

Render the gerbers (gerbv / tracespace / KiCad gerber viewer) layer by layer, separate from the CAD tool, and check: outline closed, drills present, mask openings correct, paste on SMD pads only, silk not clipped, no missing pours.

`scripts/gerber_render.py` does it with gerbonara (an independent parser) and
headless Chrome. Look at every image: on the example the first attempt went
through ImageMagick, whose SVG renderer silently dropped the silk strokes, and
a blank layer is easy to read as "fine". Compare each layer's object count
with what the board should have (holes = vias + plated pins; bottom paste
empty when nothing is assembled there).

## Final independent review

The stronger model, fresh context, gets: `intent.md`, all `parts/*.md`, schematic PDF, `nets.csv`, `stackup.md`, all renders, DRC/ERC/sync outputs and gate files. It answers: would you send this to fab? What's the highest-risk thing on this board? Anything in `intent.md` not met?

## Bring-up plan

Take every expected value from the part audit's own numbers, not from memory
of a similar board: the example's first plan used Schottky numbers for a
silicon rectifier, and a good board would have failed its VLED step. Probe
points must be probe-able (a capacitor pad, not a DFN pin).


Write `design/bringup.md`: power-up sequence with current limit, what to measure at each test point and expected value, isolation jumpers to open first, firmware smoke test. Link it from the project page (`index.html`).

## Gate checklist (phase 10)

- [ ] All earlier gates current (not stale)
- [ ] Sync check passes
- [ ] DRC/ERC re-run on final files: clean
- [ ] Stock re-checked
- [ ] CPL rotation verified for every polarized part
- [ ] Gerbers reviewed in a separate viewer
- [ ] Final independent review: no blocking findings
- [ ] Bring-up plan written
- [ ] Git tagged
- [ ] **User approves release**
