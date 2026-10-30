# Phase 9: silkscreen and art

Silkscreen is the board's user interface. Someone holding the board should know what it is, how to power it, and what every connector does, without the schematic.

## Board identity (required)

- Board name
- Revision and date (e.g. "rev B 2026-10")
- **Open source**: GitHub repo URL, and license (e.g. "CERN-OHL-S")
- **Work**: internal part number, company per policy; no public URLs
- Optional: author, a small QR code to the repo or docs

## Connectors (required, every one)

Each connector gets a reasoned label next to it:
- **Function**: "MOTOR", "CAN", "USB", "BATT IN"
- **Polarity / pinout** next to the pins: "+" and "−", signal names, pin 1 marker
- **Ratings**: "12–48 V DC", "60 V MAX", "5 A", "3.3 V LOGIC"
- **Warnings** where relevant: "NO REVERSE POLARITY PROTECTION", "HOT SURFACE"
Decide wording in a table in the gate file first (connector, label text, reason), then place.

## Designators

Policy from `brief.md`:
- **All**: every part, visible.
- **Selective**: connectors, ICs, test points, jumpers, LEDs, user-adjustable parts, anything referenced in docs.
- **None**: small hobby boards. Keep connector, LED and test point labels anyway. Generate an assembly drawing (fab layer) with designators so the board can still be debugged.

Every designator that is shown:
- adjacent to its own part, clearly associated (not closer to another part)
- **not under any part**, including its own
- **not crossed by vias or pads** (silk over exposed copper gets clipped by the fab)
- readable orientation: at most two rotations on the board (0° and 90°), consistent within a block
- consistent size (default 0.8–1.0 mm height, ≥ 0.15 mm line width; check JLC minimums)

## Reserve the silk before routing

Silk over a tented via breaks up over the ring and drill, and after routing a
dense board has no room left: the example (21 × 51 mm, 4 layers, 150 vias) had
no via-free strip on the bottom longer than 13 mm, and the 0.8 mm identity
text needs 15–22 mm per line. So, in phase 6, before routing:

- fix where the text that must sit in one place goes: pin labels beside
  header pads, the identity block, a URL;
- put a **no-via rule area** (tracks and pours allowed, all copper layers)
  under each, sized from the rendered text outline plus 0.15 mm, so the
  router, pad vias and stitching keep off it (`examples/easy2350/scripts/board.py`,
  `bottom_silk_fixed`);
- keep it off part rows that need ground vias (crystal, decouplers, LEDs);
- keep it out of channels where nets change layer: a reservation is a wall
  for vias. On the example, reserving all 40 pin labels left GPIO0-5
  unroutable (they drop to L4 beside the left header's top labels, the only
  way past the flash), even with the labels reserved at 0.6 mm wide. Those
  eleven stay unreserved: after the silk, `via_nudge.py` moves any router via
  that landed under one, and the placer nudges the text. Re-route after
  adding a reservation and compare;
- wrap long lines (a URL over two or three lines) rather than shrink text
  below the fab minimum.
- space wrapped lines for their descenders: at 0.8 mm text and 0.15 mm
  stroke, lines with q, p, g or y need a 1.4 mm pitch to keep the board's
  0.15 mm silk clearance (1.25 mm left 0.07 mm). Test stroke-to-stroke
  clearance with the board's own silk clearance, not a smaller margin.

Reserve the identity, limit and warning lines and any top function labels
too, not only the pin labels: on the example they were placed by search after
routing, which worked on one route and found no room on the next two (the
VBUS warning, the GPIO25 LED label). Find their spots once (by search, on a
good route), then fix them and reserve them like the rest. Where a locked
via already sits under a label, cut a hole round it rather than dropping the
reservation, or the router fills the rest with vias.

Text that can go anywhere is placed afterwards by search, testing the
rendered strokes (not KiCad's text box, which is ~1.8× the height) against
vias, plated holes and other silk, and keeping searched text ≈ 0.45 mm from
unrelated text so two words don't read as one.

## Overlaps: zero tolerance

No silk-to-silk, silk-to-pad, or silk-to-via overlap, and no silk over board edge. If two footprint outlines collide, **edit a board-local copy of the footprint** to trim the outline. Don't leave it.

Also: indicators for polarity (diodes, electrolytics, LEDs) and pin 1 on every IC and connector must remain visible after trimming.
Trimming silk off pads usually takes them: on the example it removed the pin-1
marks of the QFN, DFN and SOT-23-5, both diodes' cathode bars and the pixels'
chamfer line, and the first review missed it. After trimming, put a 0.3 mm
dot back beside each pad 1 that lost its mark, on the outside, clearly nearer
pad 1 than its neighbours (`board.py` `dot()`); `silk_check.py` fails a part in
`[silk].pin1_prefixes` with no mark beside pad 1.

## Art

Every board gets a little art in the art zone from `brief.md`.
- If the user supplies art (SVG/PNG), convert it to silk and/or copper/mask (KiCad Image Converter; Altium image import) at the reserved size.
- If Claude drafts it: propose 2–3 small concepts on the project page, related to the board's purpose, as simple line art. The user picks.
- Consider exposed copper (ENIG looks great), copper + mask openings, or silk on copper for contrast.
- Art follows the same overlap rules and stays out of keepouts and away from sensitive nets (exposed copper art is not grounded unless intended).

## Silkscreen check script (`scripts/silk_check.py`)

`python3 .claude/skills/pcb-design/scripts/silk_check.py hw/<board>/<variant>`. Policy, minimums and identity items come from `design/checks.toml` `[silk]`; set `designators` from `brief.md`, and add `"repo"` and `"license"` to `identity` for open-source boards.

Computes the actual geometry (rendered text outline, not bounding box of the text anchor) and fails on:
- any silk-silk overlap
- silk over pads or exposed copper (KiCad DRC), and silk text over vias on either side (the script itself: vias are tented, so KiCad's `silk_over_copper` passes them, but the ink breaks over the ring and drill; `[silk].via_clearance_mm`)
- silk outside board or within copper-to-edge distance
- designator under a courtyard (including its own)
- designator whose nearest courtyard isn't its own part
- text below minimum height or line width
- connector without a label within X mm
- missing board identity items

## Gate checklist (phase 9)

- [ ] Connector label table with reasoning
- [ ] Board identity present
- [ ] Silk check passes (output pasted)
- [ ] Top and bottom silk renders reviewed by the stronger model
- [ ] Art placed and approved
- [ ] User reviewed silk render in PCB view
