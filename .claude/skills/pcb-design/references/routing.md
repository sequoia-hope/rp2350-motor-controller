# Phase 7: routing

Claude is bad at hand-routing by picking coordinates. So route in this order, and let tools do as much as possible.

## Order

1. **Planes and pours.** GND plane (L2 on 4-layer), power planes/pours, and POWER_HI copper as zones, not traces. Set zone priorities and clearances. Pours are drawn as polygons around regions, which is far more reliable than tracing paths.
2. **Via arrays** for every net with a via count from `nets.csv`: stitch power pours between layers at the points where current transfers, with the computed count. Place as arrays (script-generated grids within a zone), not single vias.
3. **Short critical connections by hand, only when short.** Gate drive, crystal, shunt Kelvin sense, switch node: these should be a few mm thanks to placement. Route them directly with straight or single-bend tracks.
4. **Differential and impedance-controlled pairs** with the tool's diff pair router or interactive router; keep on one layer over solid reference; length match per rules.
5. **Autorouter for everything else** (ultraroute for KiCad, Situs/ActiveRoute for Altium) with full net class rules loaded. Lock everything routed in steps 1–4 first so the autorouter can't disturb it. Always run it governed; see "Running the autorouter without hurting the host" below.
6. **Cleanup**: remove autorouter artifacts (acute angles, unnecessary vias, long detours), re-pour zones.
7. **Ground stitching vias** around the board edge and between ground pours, and around RF/sensitive areas.

## The KiCad flow, with the scripts

What routed the example board (`examples/easy2350`, RP2350A QFN-60, 4 layers,
21 × 51 mm); each step is a script so a re-run gives the same board.

1. **Copy what the vendor says to copy** before anything routes:
   `transplant.py` (power blocks, zones, spokes, short critical pairs such as
   USB from pin to series resistor). Locked. The vendor's copper is the
   reference layout; the router's own pair routing (`--diff-pairs`) does
   the rest of a pair.
2. **Planes first, and only planes.** Whole-board GND and power planes on the
   inner layers, present while routing, so the router drops vias into them.
   A power pour added after routing leaves islands the router never joined.
   Mark plane layers so nothing routes on them (`autoroute.py --planes`),
   and fill the planes before routing: ultraroute joins a plane net's pads
   and vias through the fill.
3. **Routing-only keepouts** where the vendor layout must stay clean (inside
   an MCU's pad ring, the regulator corner): tracks-only rule areas, removed
   before the pours. A no-pour rule area also stops the protected zones
   filling.
4. **No outer-layer pours while routing**: a full-board ground pour is an
   obstacle to every other net.
4a. **Critical connections drawn, locked, in a group** (step 3 above): the
   crystal pins to the crystal and its load caps on L1 only, the USB pair
   from the receptacle to the ESD array, any net the placement makes short
   but the router would wander on. A router cost-optimises: on the
   example Freerouting took XIN through two vias and 6 mm of L4, looped D+ 7 mm past
   the flash clock, and ran a pixel-to-pixel link 25 mm under the MCU. Draw
   them from pad positions, not typed coordinates, and re-run DRC on the
   locked copper alone (delete unlocked tracks in a scratch copy) before
   routing. A locked pre-route is a wall: before drawing one, look at which
   nets escape through the same gap (on the example, the D+ run down the
   side of the ESD array sat in the only corridor the flash's three far-row
   pins had, and QSPI_SD2 became unroutable). Take the pre-route to the
   other outer layer for that stretch.
4a'. **The art zone as no-via rule areas** (silkscreen-and-art.md, "Reserve
   the silk before routing"): pin labels and identity text get their room
   now, or there is none after routing.
4a''. **Edge strips**: a no-track, no-via rule area (pours allowed) along each
   edge where connector pads sit, as wide as the pads' distance from the edge
   minus a hair. Otherwise the router runs long tracks between the pads and
   the edge: on the example BOOTSEL ran 14 mm at 0.35 mm from the routed edge
   (JLC's outline tolerance is ±0.2 mm) and 0.14 mm from the header joints
   the user hand-solders. The board's copper-to-edge rule alone doesn't stop
   it.
4b. **Ground vias at the pads, before routing**: `stitch.py --net GND --pads
   'C*' --pads U3:6 --tag padvias --lock` puts a via within reach of every
   capacitor's ground pad and N round a regulator. After routing there is
   rarely room, and a decoupler whose ground reaches the plane only through
   the top pour fails review.
   Do the same for power pins on a plane net with no outer pour (3V3 on an
   inner plane): the example's flash VCC was joined only by a `--fix` via,
   which landed under a pin label; once vias kept off the silk it had no
   other spot. `--pads U5` (one via per pad of U5 on the net); `U5:8` would
   ask for 8 vias.
   On a re-route, remove the old route's unlocked copper *before* placing
   the pad vias (the example's `stage_unroute`): the router clears it anyway,
   but pad vias placed first find its tracks in their spots (C25/C27 lost
   their ground vias that way).
   Every capacitor's pad on a plane net needs one too (the router leaves
   plane-net pads to vias, and after routing there may be no room: the
   example's flash decoupler C22 ended up unconnected).
5. `autoroute.py`: ultraroute (`vendor/ultraroute`, pinned; `scripts/route.sh`
   builds it) with locked copper kept (`--strip` takes only unlocked
   tracks), negotiated congestion, rip-up, fan-out of fine-pitch parts, the
   plough search for trapped pins, differential pairs and pull-tight. Seeds
   are tried one at a time until KiCad, after a zone refill, reports no
   signal open and no error the route added; the best is kept. Judge by
   KiCad, never by the router's own count (it counts plane nets as open
   before the fill). Parts don't move unless `--nudge` is given: placement
   passed its gate, and a move goes back to it.
   On the example, Freerouting needed 16 settings variants and a maze
   finisher, about 50 minutes a route; ultraroute takes about a minute a seed.
6. Changing ultraroute's pin (`git -C vendor/ultraroute pull`) can change the
   route: re-run phase 7 after a bump. Results depend on `--threads` (1 is
   one negotiation order, 2+ another, the same for any count) and on
   `--timeout` (later stages share out the time): keep both fixed for a
   board, as `autoroute.py` does.
7. Pours, then `stitch.py --grid --fix`: ground stitching on a grid, a via in
   every pour fragment, and vias for pads and tracks the router left on a
   plane net without one, all clear of the silk (place silk first). Then
   fill, delete dangling stubs and vias (router leftovers; the example's
   `stage_clean`, repeated because removing a stub can strand its via), and
   DRC. Before deleting a via DRC calls dangling, snap the track ends inside
   it to its centre: ends that met inside the via can differ by nanometres,
   and deleting it cut the example's VBUS at U2. Snap only unlocked tracks: a vendor
   copy's joint that happens to sit inside a pad via was pulled 0.22 mm and
   broke a clearance.

When the router can't finish, the fix was placement every time, not
compute: a part sitting in a pin's escape (a decoupler in front of the next
pin, a resistor on the vendor's diagonal), a column of small parts across a
channel that seven GPIO need. Run `placement_check.py`, read which nets are
stuck and where they leave the IC, and move what blocks them.

## Running the autorouter without hurting the host

Autorouters default to taking the whole machine. Run one unthrottled on every new project and it pegs the desktop. Every autorouter run is governed. No exceptions, including "just a quick test".

- **One autorouter run at a time per machine.** Never launch routes in parallel to try variants; queue them. `autoroute.py` tries its seeds in turn and stops at the first complete route.
- **Low priority:** `nice -n 15`.
- **Threads:** at most half the cores. Use 1 unless you have a measured reason (and see step 6: the thread count changes the route).
- **Memory:** a hard ceiling via a systemd scope, with swap off so a runaway dies alone. The example board peaks at ~0.5 GB.
- **Time:** a wall-clock budget. The router's own `--timeout` (240 s by default in `autoroute.py`) plus a `timeout` two minutes longer as the backstop.
- **Good enough is done:** stop at 0 open and no new errors. Don't burn seeds chasing via counts.

By hand (what `autoroute.py` runs):

```bash
systemd-run --user --scope --quiet -p MemoryMax=6G -p MemorySwapMax=0 -p CPUQuota=100% \
  nice -n 15 timeout 360 \
  scripts/route.sh route board.kicad_pcb -o out.kicad_pcb --strip --negotiate --rip-up --tighten \
    --prefanout --diff-pairs --planes In1.Cu,In2.Cu --seed 0 --threads 1 --timeout 240
```

If a run hits its time or memory budget, report that, along with the unrouted count. Don't silently retry with bigger limits. Usually the fix is placement, rules or layer count, not more compute.

## Rules of thumb

- Every signal has a continuous reference plane under it. No high-speed or SE_HS trace crosses a plane split. If a layer change is needed, put a ground via next to the signal via.
- Keep the area of high di/dt loops minimal. Return current flows directly under the trace on the plane.
- No traces under switch nodes on the adjacent layer except ground.
- Analog and Kelvin sense lines as tight pairs, away from switching copper.
- Thermal reliefs off for POWER_HI connections, on for hand-soldered pins and big ground pads that need to be solderable.
- Teardrops on (where supported).

## Checks

- DRC: **0 errors, 0 unrouted**. Paste the report.
- Schematic parity: 0 differences.
- `nets_check.py --rules --routed`: every `length_limit` met, no via on a "no vias" net. A limit placement can't meet is a placement finding, or a logged decision with the physics; never a silent edit of `nets.csv`.
- Every DRC warning is explained in the gate file, or fixed.
- Current capacity check script: for each POWER/POWER_HI net, the narrowest copper neck along the path and the via count at each layer transition meet `nets.csv` derived values.
- Plane integrity script: report splits and slots in GND plane, and any DIFF_HS/SE_HS trace that crosses one.
- Visual review on per-layer copper renders.

## Gate checklist (phase 7)

- [ ] DRC 0 errors, 0 unrouted (pasted)
- [ ] Parity 0 differences
- [ ] Current capacity check passes
- [ ] Plane integrity check passes
- [ ] Every warning explained
- [ ] Per-layer renders reviewed
- [ ] PCB view regenerated with copper viewer
