# settle: feasibility-projected settling of a board region

"Jiggle the board like a bag of chips until things fit": every movable unit
moves only as far as the DRC clearance geometry admits, pressure propagates
through copper AND parts, trace length is a spring with a hard cap, and the
one gate that must always pass is real kicad-cli DRC on every emitted board.

    prep.py   CONFIG.json     board + config -> data/<name>/model.json
    settle.py CONFIG.json     the engine -> traj.json + settle_report.json
    emit.py   CONFIG.json     every checkpoint as a board + the 5-check gate -> gate.json
    render.py CONFIG.json     figures + review/settle_<name>.html

Configs live in `configs/`, run data in `configs/data/<name>/` (gitignored).
The lean-jiggle worktree's `review/` is served on port 8016.

## Config

```
name, board (relative to the config), region [x0,y0,x1,y1] (mm, default whole board)
rules:    clearance / hole_clearance / edge_clearance default from the .kicad_pro;
          guard = floor above rule for pairs that had room (0.015)
locked:   refs [...], sides ["F"], nets [regex...], rects [[x0,y0,x1,y1]...]
          (a movable part carrying locked/out-of-region copper is fixed too)
mobility: default 1.0, refs {ref: m}, rects [{rect, m}]   (0 = locked)
groups:   {name: {refs [...] | rect, rigid: true}}       rigid = one shared step
elastic:  default / classes {netclass: ...} / nets {regex: ...}, each
          {k, allow, cap, allow_mm, cap_mm}: growth beyond max(allow*L, allow_mm)
          pulls back with stiffness k; growth past max(cap*L, cap_mm) is a hard
          line-search constraint. L = the net's copper inside the region.
drive:    inflate [{ref, at: [x,y] | "auto" | "search" | {near, offset}, rot, side, radius}]
          targets {ref: [dx,dy]}
          shove   [{refs | rect, vector [dx,dy] | toward [x,y] + mm}]
step (0.05), snapshots (10), cycles (600), ghost_anchor (0.3)
```

`at: "auto"` = centroid of the partner pads (same nets, power excluded);
`"search"` = least-obstructed courtyard box within `radius` of that centroid
(pads/courtyards weight 1, pushable copper 0.3, plus 0.03/mm distance).

## What the engine does per cycle

1. **ghost pass** — each unborn ghost drifts under pressure (+ a weak pull to
   its site) and then tries to grow by `step` at its perimeter; what clamps it
   is pushed radially (copper nodes, movable parts, other ghosts). Growth
   clamped by unmovable geometry pushes the ghost the other way (slides off
   walls).
2. **part pass** — every unit (part or rigid group) with a drive moves:
   target term + accumulated pressure + spring tension from its own chains,
   capped at `step * mobility`, line-searched over all its pairs, courtyards,
   edge and the length caps of the nets it touches. Blocked units deposit
   pressure on their blockers.
3. **group pass** — mutually wedged units step jointly.
4. **copper pass** — pressured nodes yield, over-long sub-segments relax,
   nodes on over-allowance nets are pulled toward their neighbours (springs).
5. stall (no advance for 40 cycles) → mid-flight A* reroute of the clamped
   chains, verified by the same oracle; retraced paths are detected as no-ops.

**Non-worsening rule.** A pair already below its floor (a pre-existing tight
pair, or a ghost speck born on top of a trace) is admissible as long as the
move does not reduce its margin. Ghost pairs are never grandfathered, so a
ghost only ever grows into legal room; the copper under it has to leave first.

## The gate (emit.py)

1. DRC at severity error: nothing new vs the checkpoint-0 roundtrip baseline
   (footprint pairs keyed by name, copper by position);
2. unconnected items not above baseline;
3. every static copper object of the base present byte-for-byte, every fixed
   footprint where it was;
4. net length growth inside the caps;
5. check_sync on the final board.

## Lessons

- Length caps must have an absolute floor (`cap_mm`): 15 % of a 4 mm stub is
  0.6 mm and froze the encoder fabric at half a millimetre of motion.
- A pre-existing courtyard overlap that rides along with a moved part is not a
  new violation; key footprint-pair signatures by name, not position.
- pcbnew courtyard polygons are ~45 um smaller than what DRC tests; the model
  floor for courtyard pairs is conservative only because the JLC-tight
  courtyards were built with that in mind — watch `courtyards_overlap` in the
  gate when new footprints appear.
