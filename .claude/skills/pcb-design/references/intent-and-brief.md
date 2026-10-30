# Phase 0: design intent, and Phase 4: mechanical and layout brief

Both need user sign-off. They are the two documents every later decision should point back to.

## Phase 0: `design/intent.md`

Ask about anything the user hasn't specified. Each item needs a number, or an explicit "don't care".

- **Purpose**: one paragraph. What the board does, where it lives, who uses it.
- **Open source or work/confidential.**
- **Every external interface**: connector or wireless link, direction, signal type, voltage range (normal and worst-case including transients, hot-plug, reverse polarity, miswiring), current (continuous and peak), what is plugged into it.
- **Power**: input range, every rail with load current, sequencing needs, battery or not, regen or backfeed.
- **Environment**: temperature range, enclosure, airflow, vibration, moisture, ESD exposure (handled by users? cables to the outside world?).
- **Performance targets**: e.g. motor current, switching frequency, ADC resolution, data rates, RF bands.
- **Mechanical**: size limit, mounting pattern, enclosure, height limits per side, connector access directions.
- **Manufacturing**: fab and assembler (default JLCPCB), quantity, cost target, single- or double-sided assembly, hand-assembled parts allowed?
- **Serviceability**: hobby board vs repairable product (drives designator policy, test points, debug headers).
- **Compliance**: anything formal (EMC, safety)? Usually not for hobby boards, but ask.
- **Known risks and open questions.**

## Phase 4: `design/brief.md`

Lay a board out from the outside in. Decide these before placing a single part, and render an outline-only board with connectors and holes placed for the user to approve.

1. **Board outline and size.** Fixed by enclosure, or free? Corner radius.
2. **Mounting holes.** Count, size (e.g. M3 → 3.2 mm plated or NPTH), pattern, keepout around each for screw heads and standoffs; grounded or isolated.
3. **Connector placement.** For each connector: edge, position, facing direction, mating clearance, cable strain path. Group by function (power in on one side, outputs on another). Connectors define where every circuit block will end up.
4. **Assembly sides.** Single-sided is cheaper and easier at JLC; double-sided buys density. State which and why.
5. **Layer count.** 2 vs 4 (vs more). Default to 4 if there's any switching power, motor drive, high-speed or impedance-controlled signal: a solid ground plane fixes many problems for little cost.
6. **Density philosophy.** Compact and portable, or roomy and easy to debug and rework? Decide explicitly; it changes part sizes (0402 vs 0603), test point count and spacing.
7. **Block floorplan.** Rough region for each functional block, ordered by the power and signal flow from the connectors inward. Noisy blocks (switchers, motor bridge) away from sensitive ones (analog, crystals, antennas).
8. **Thermal.** Where heat is generated, where it goes (copper area, thermal vias, heatsink, enclosure).
9. **Art zone.** Reserve a region for board art now, sized and kept clear of sensitive nets. Note whether the user is supplying art or Claude should draft it.
10. **Silkscreen policy.** Designators everywhere, only where useful, or removed (small hobby boards that won't be repaired)? Board identity text: name, revision, date, repo URL (open source) or internal part number (work).
11. **Keepouts.** Antenna areas, mounting hardware, high-voltage creepage, enclosure ribs.
12. **Test and debug.** Test points on every rail and key signal, debug header, isolation jumpers (0 Ω or solder bridges) to split sections during bring-up.
