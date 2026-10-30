# Check scripts

The automated checks the gates rely on. Paste a check's summary into the gate
file; every script exits non-zero on failure and can write the findings as JSON
with `--json`. Paths below are from the project root.

```sh
S=.claude/skills/pcb-design/scripts
python3 $S/nets_check.py hw/main/a             # phase 3
python3 $S/nets_check.py hw/main/a --rules     # phase 5 on
python3 $S/nets_check.py hw/main/a --rules --routed   # phase 7 on: length_limit, "no vias"
python3 $S/sync_check.py hw/main/a             # phase 6 on; add --bom/--cpl at release
python3 $S/silk_check.py hw/main/a             # phase 9
python3 $S/jlc_parts.py show C1525 C97521      # phase 1 and 10: stock, basic/extended, lifecycle
python3 $S/schematic_style_check.py hw/main/a  # phase 2
python3 $S/placement_check.py hw/main/a        # phase 6
python3 $S/transplant.py ref.kicad_pcb hw/main/a/main.kicad_pcb --anchor U1=U1 ...   # phase 6: copy a vendor block
python3 $S/autoroute.py hw/main/a --planes In1.Cu,In2.Cu    # phase 7: ultraroute, governed, judged by KiCad
python3 $S/stitch.py hw/main/a --net GND --pads 'C*' --pads U3:6 --tag padvias --lock   # phase 7, before routing
python3 $S/stitch.py hw/main/a --net GND --net +3V3 --grid 1.6 --fix   # phase 7: stitching vias
python3 $S/render.py hw/main/a design/renders    # every visual gate: top/bottom/layers/3D/assembly/schematic
.venv/bin/python $S/gerber_render.py hw/main/a/fab/gerbers design/renders/gerbers   # phase 10 (pip install gerbonara)
```

| Priority | Script | Status | Used at | Catches |
|----------|--------|--------|---------|---------|
| 1 | `sync_check.py` | **done** | 6–10 | schematic / PCB / BOM / CPL mismatches in presence, value, footprint, every user field (MPN, LCSC, ...), DNP and exclude-from-BOM; built parts missing a required field |
| 2 | `silk_check.py` | **done** | 9 | silk overlaps, silk on pads/edge (KiCad DRC, forced on), silk text over tented vias, missing pin-1 / polarity marks, designators hidden, under a part, or nearer another part, too many rotations, small text, unlabeled connectors, missing board identity |
| 3 | `nets_check.py` | **done** | 3, 5 | nets missing from `nets.csv` (both directions), bad classes, missing reasons; derives widths (IPC-2152), via counts, clearances (IPC-2221); with `--rules`, checks KiCad net classes match and cover them; with `--routed`, routed length against `length_limit` and "no vias" rows |
| — | `jlc_parts.py` | **done** | 1, 10 | JLC part search; stock, basic/extended and lifecycle per LCSC number; datasheet download (LCSC, then wget) and `sources.csv` re-fetch |
| 4 | `placement_check.py` | **done** | 6 | decouplers far from the pin their `Decouples` field names, crystals far from the IC, passives in rows, keep-apart pairs, parts off the board; ratsnest score |
| 5 | `schematic_style_check.py` | **done** | 2 | grid-placed symbols, labels where wires belong, rotated power/ground symbols, missing values/footprints/fields, decouplers drawn far from their pin, global labels on one sheet, overlapping text |
| — | `transplant.py` | **done** | 6 | copies a vendor layout block (parts, zones with settings, tracks, vias) relative to an anchor part; `--clip`, `--exclude`, `--tag` |
| — | `autoroute.py` | **done** | 7 | ultraroute (`vendor/ultraroute`, built by `scripts/route.sh`) on the board with its locked copper kept and plane layers marked, seeds tried in turn under the governance in routing.md; every result judged by KiCad after a refill (signal opens, errors the route added), the best kept; parts move only with `--nudge` |
| — | `stitch.py` | **done** | 7 | ground vias at decoupler and regulator pads (before routing, locked); stitching vias: a grid in the pours that keeps off silk text, one in every pour fragment, and vias for plane-net pads/tracks the router left without one; every via clear of silk text. `--pads REF:N` means N vias round REF, not pin N; pad vias ignore the regenerated "stitch" group, and with `--wired` count only a via a track joins to the pad (a net with no outer pour), so give any power pin the router may strand (a flash VCC) a pad via before routing |
| — | `render.py` | **done** | 2, 6, 7, 9, 10 | the gate renders in one command: top and bottom with the silk painted over the copper (a single KiCad plot hides silk under copper), each copper layer, 3D both sides, assembly drawing, schematic PDF |
| — | `gerber_render.py` | **done** | 10 | the gerber review outside KiCad: parses the fab gerbers and drills with gerbonara, prints objects per layer, renders each layer (headless Chrome; ImageMagick's SVG renderer blanks the silk) |
| — | `via_nudge.py` | **done** | 7 | moves unlocked router vias out from under silk text (labels that can't be reserved without blocking the router), dragging their track ends, keeping the change only if DRC gets no worse |
| 6 | `current_check.py` | spec | 7 | narrowest copper neck and via count per power net vs `nets.csv` |
| 7 | `plane_check.py` | spec | 7 | GND plane splits/slots, high-speed traces crossing them |
| 8 | `crosstalk_check.py` | spec | 8 | aggressor/victim parallel runs |
| 9 | `cpl_check.py` | spec | 10 | renders CPL overlay for rotation/pin 1 verification |

## Requirements

KiCad 9 (`kicad-cli`) for all of them; `silk_check.py` also needs KiCad's
python module (`pcbnew`). `scripts/doctor.sh` checks both.

## How they read the design

- Schematic: `kicad-cli sch export netlist` (hierarchy, annotation and DNP
  resolved by KiCad). Board: the `.kicad_pcb` S-expression, or `pcbnew` where
  geometry is needed. Shared code is in `kicad_files.py`.
- `silk_check.py` runs KiCad's DRC on a scratch copy with every silk rule
  forced to error, so a rule set to "ignore" in the project can't hide an
  overlap, and DRC exclusions still count.
- Thresholds live in `design/checks.toml`; defaults are in
  `kicad_files.DEFAULTS`. Loosen a value there with a comment giving the
  reason, and log it in `decisions.md`.

## Writing KiCad files

`kicad_write.py` does the two things `kicad-cli` can't:

- `Schematic`: place library symbols, wires, labels and power symbols at
  coordinates you planned, and save a `.kicad_sch`. On save it splits wires
  at every T and adds junctions, as the editor would (KiCad leaves a pin or
  junction in the middle of a wire unconnected).
- `update_pcb(sch, pcb)`: Update PCB from Schematic, headless. Footprints,
  fields, DNP/BOM flags and pad nets come from the schematic; parts already
  on the board keep their position.

Always follow a write with `kicad-cli sch erc` and a look at the exported PDF.

## Tests

```sh
cd .claude/skills/pcb-design/scripts/tests
python3 make_fixture.py     # builds fixtures/blinky, a 3-part board that passes everything
python3 test_checks.py      # each rule: the fixture passes, a copy broken in one way fails
```

A new rule gets a pass case and a fail case here before it is used at a gate.

## Next

The remaining scripts follow the same pattern: a `kicad_files` loader, a
`Report`, thresholds from `checks.toml`, and a broken copy of the fixture per
rule. `placement_check.py` is next, since placement is where the most
babysitting happens.
