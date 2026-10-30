# Phase 6: placement

Placement determines most of the board's quality. Good placement makes routing nearly automatic; bad placement can't be fixed by routing.

## Order

1. **Fixed items** from `brief.md`: outline, mounting holes, connectors, keepouts, art zone. Lock them.
2. **Blocks in floorplan order**: work from connectors inward, following power and signal flow. Place each block as a unit before moving to the next.
3. **Within a block, critical loops first**:
   - switching power stages: input cap, switch, diode/low-side FET, ground return as one tight loop; then inductor, then output caps; feedback divider near the IC's FB pin, routed away from the switch node
   - half-bridges: high-side and low-side FETs, bulk and ceramic caps across the bridge, shunt, all in the tightest possible loop
   - gate drivers right beside their FETs
   - crystals right at the oscillator pins
4. **ICs** in their block region, oriented so their pins face what they connect to.
5. **Supporting passives with their pins**: decoupling caps at each power pin (smallest value closest), pull-ups at their pins, termination resistors at the driver (series) or receiver (parallel).
6. **Test points and debug features.**
7. **Silkscreen space**: leave room for designators and connector labels now, not after routing.

**Follow the datasheet's layout guidelines** from each `parts/*.md`. If the datasheet shows a recommended layout, reproduce it.

## Rules

- **Never** place passives in a row, a grid, or any arrangement that is about tidiness rather than function. Every passive belongs to a pin.
- Align parts within a block where it costs nothing (neat rows of decoupling caps along an IC edge are fine), but function wins over neatness.
- Consistent orientation for polarized parts within a block helps assembly inspection.
- Keep switching nodes and high-dV/dt copper away from analog, clocks and board edges.
- Keep the ground return path in mind for every high-current loop: where does the current come back?
- Hot parts: spread out, near copper area and thermal vias, away from electrolytics and temperature-sensitive parts.
- Parts that are hand-soldered or reworked need clearance around them.
- Fiducials if assembly needs them (JLC generally doesn't for standard assembly, check).

## Placement check script (`scripts/placement_check.py`)

```sh
python3 .claude/skills/pcb-design/scripts/placement_check.py hw/<board>/<variant>
```

- **decoupler**: each capacitor whose `Decouples` field (set in the schematic,
  e.g. `U1.11`) names a pin has a pad within `[placement].decoupler_mm` of it.
  The field is how the schematic says which pin a cap serves when the symbol
  stacks pins.
- **crystal**: each crystal pad within `[placement].crystal_mm` of its IC pin,
  through one series resistor.
- **row**: four or more same-footprint passives evenly spaced in a line (a
  warning: a row of LEDs can be right).
- **keep-apart**: pairs in `[placement].keep_apart`, body to body (e.g. an LDO
  away from a crystal).
- **outside**: every part inside the outline. Prints the total ratsnest length
  to compare placements.

## Placement for routability

The router fails where placement left no way out. Before routing a fine-pitch
IC, look at every pin's escape:

- Nothing sits in front of a signal pin within ~1.5 mm: a decoupler placed at
  its own pin can block the next pin's escape (keep it, move it a pad over,
  or share it as the vendor does).
- Copying a vendor layout? Copy its escape directions too: put nothing on the
  diagonal a vendor route takes out of a corner pin.
- Keep channels to the connectors: count the signals that must pass between
  the IC and each header and leave them room on the outer layers, which are
  the only routing layers when the inner two are planes.
- Small parts with no position constraint (SWD resistors, status LEDs, test
  points) go where nothing needs to pass, not in the nearest gap. A test
  point with no part can go on the bottom.

## Visual review

Render top/bottom 2D and 3D with the ratsnest visible. Reviewer answers:
1. Does each block sit together, near its connectors, in a sensible flow?
2. Is every passive with the part it serves?
3. Are the critical loops tight?
4. Is noisy separated from sensitive?
5. Is there room for routing, vias, and silkscreen?
6. Would an experienced layout engineer place it like this?

## Gate checklist (phase 6)

- [ ] Fixed items match `brief.md` and are locked
- [ ] Placement check passes (output pasted)
- [ ] Datasheet layout guidelines followed per part (list)
- [ ] Visual review passed
- [ ] User reviewed placement render in PCB view

## Layout rules from the audits become rule areas

Every "no copper under …", "keep … clear" or "nothing within … of …" line in
a part audit's layout guidance goes onto the board as a rule area when the
part is placed, so DRC enforces it on every route. Text in an audit is not
checked by anything: on the example, "no copper under the pixel's centre
strip" (WS2812B-2020 metal polarity mark) was in the audit and the brief but
not drawn, and every route put copper under all four pixels; no check or
review caught it until a release fix looked.

Also orient parts for their own signal flow: the pixels' DOUT faced away from
the next DIN, so every hop had to cross under two bodies. Turned 180°, each
hop is 2 mm on the top layer.

