# Rubber stretch: warping fields + violation nudging

Answer to: "stretch the whole board like rubber, tolerate the damage with local
trace nudges, hand the rest to a human."  Pages: `review/field.html` (the
current engine), `review/rubber.html` (the first attempt and the F-46 census).

    field.py     THE SOLVER, no pcbnew.  A displacement field on a grid:
                   u(x) = ambient(x) + r(x),  u = v_i inside rigid inclusion i,
                   r harmonic in free space, r -> 0 on the domain boundary.
                 `ambient` is any callable (None for a pure push-these-apart
                 field) and inclusion vectors are arbitrary -- the hook for
                 steerable "add space here" fields.  `sigma_min()` returns the
                 admissibility map (the field closes a gap iff the smallest
                 singular value of I+grad u drops below 1).
    stretch.py   applies it to the board: KX/KY dilation about the centre,
                 footprints rigid at their PAD CENTROID, ONE FIELD PER COPPER
                 LAYER, every drill rigid on all layers (NPTH included), via
                 barrels coupled across layers in two passes, adaptive track
                 splitting so a chord follows the curved field, zones follow,
                 silk art stretched point-by-point and silk text scaled by k,
                 Edge.Cuts on pure ambient, corner arcs map exactly.
                   SRC=... OUT=... KX=1.01 KY=1.01 python3 stretch.py
                 env: H (grid mm), MARGIN, CLOSE, ENCLOSE, ANCHOR, SPLIT_TOL,
                      VIA_PASSES, STATS, FIELD_DUMP
    fieldmap.py  static board-referenced map of the field stretch.py APPLIED
                 (it draws the FIELD_DUMP, never a rebuilt field): lag heat map
                 + arrows + rigid-territory outlines on the left, sigma_min on
                 the right, copper/holes/refs superimposed, mm axes.
                   FIELD=dump.npz LAYER=In3.Cu OUT=map.png [ZOOM=cx,cy,w]
    nudge.py     DRC-refereed local repair: cluster new violations into sites,
                 push offending track chains apart (deficit+margin, falloff
                 2 mm), tee ends and via-rim ends RIDE their bar, vias movable,
                 courtyard overlaps fixed by footprint micro-moves.
                   python3 nudge.py BOARD BASELINE_DRC.json LOG.json [ROUNDS]
    status.py    the manual-repair loop: DRC diff vs pre-stretch baseline, site
                 list worst-first, airwires, check_sync.
    animate.py   side-by-side mp4 of both engines, 0 -> +KMAX -> 0.
                   OUT=... KMAX=0.10 FRAMES=240 ZOOM=cx,cy,width_mm LAYER=F.Cu
    sigma_probe.py  admissibility: bins sigma_min by distance from the nearest rigid
                 territory and converts depth into um lost on a reference gap.
                   KX=1.01 LAYER=F.Cu,B.Cu OUTJ=... python3 sigma_probe.py
    gaps.py      payoff probe: pad gaps at the F-25 boxed-in blockers.
    render_field_page.py + field_results.json  ->  review/field.html

## Result

At +1%, measured with `status.py` on the same board: **465 -> 47** new DRC
violations, 257 -> 41 sites; after nudging, 92 -> 30 sites for hand repair
(median deficit 15 -> 6 um, worst 121 -> 29 um).  84-91% of the damage removed
across +0.25%..+3%; check_sync IN SYNC and no airwire regression at any k.
Algorithm A at +3% is cleaner than the old engine at +0.25%.
Staged board: `hardware/rp2350_driver_rubber_A.kicad_pcb`.

## Lessons (hard-won, see also ../modules/README.md)

- **Interpolate the lag through SPACE, not along the net.**  The graph-harmonic
  engine (git `21f8fce`) blended each footprint's residual along its own net's
  copper, and that graph knows nothing about distance: two traces 0.2 mm apart
  on different nets got uncorrelated displacements and the gap between them
  changed by the difference of two unrelated residuals.  That was the entire
  F-46 census -- every site a different-net pair, deficits in the rigid-lag
  range, site count flat in k while depth scales.
- pure position-scaling is still illegal at any k: pad copper lags full dilation
  by (k-1)*|pad offset from footprint origin|.  Strain must route around rigid
  footprints -- a spatial field does that smoothly.
- **Rigid territories are pads, not courtyards.**  Copper passing under a
  courtyard is free to deform (a trace under a QFN is exactly that), and
  abutting courtyards fight over the seam cells between them.
- **...plus what the pads enclose** (morphological closing of each part's own
  pad set): a trace threading a connector's pin field is held on both sides and
  can only move with the part.
- **One field per copper layer.**  Clearance lives inside a layer, never between
  them.  A single 2-D field makes opposite-side parts that overlap in projection
  fight over the same cells -- Y1's pad owned by U16, U7's by CL1, up to 57 um
  of bogus shear.  This was worth 150 -> 53 violations on its own.
- **A via barrel is one rigid body piercing the stack**, so it can only be in one
  place: mean of the layers it spans, then handed back to every layer as a rigid
  inclusion and re-solved.  Without that second pass a via shears against the
  copper beside it by however much the layers disagreed.
- **A DRILL is rigid on every layer, annulus or not.**  It pierces the whole
  stack and carries its own DRC clearance, so an NPTH mounting hole with no net
  and no copper is as rigid as any pad.  (On this board every hole happens to
  have an annulus on all six layers, so pinning the copper covered it by luck;
  the drills are now pinned explicitly and the tool prints the count.)
- **Anchor a rigid part at its PAD CENTROID, not its footprint origin.**  A part
  gets one translation, and the least-squares choice over its own pads --
  ambient(pad centroid), ambient being affine -- is the one that least disturbs
  the copper around it.  The origin is an arbitrary CAD anchor: the phase
  connectors J1/J2/J9 carry theirs 5.59 mm off centroid (J12 5.10, J11 3.83),
  which dragged every one of their pads, mounting holes included, through 56 um
  of needless lag at +1%.  Worth 47 -> 37 violations on its own, and it smooths
  the field generally: splits 134 -> 93, worst chord sag 46 -> 42 um, F.Cu
  sigma_min 0.63 -> 0.75.
- **The enclosure must scale with the part, but the convex hull is too much.**
  A fixed closing radius cannot serve both a 0.5 mm pin field and a connector
  whose pads are 12.8 mm apart, and it left 43 um of sag inside J9's body.  But
  the hull of those four pads is mostly open board with *other parts in it*
  (BH1, C80, live traces), and forcing that rigid was worse than the disease:
  61 violations against 47, with fresh 70-78 um sites at the hull boundary.
  Kept `close`; `ENCLOSE=hull` is available and, once the anchor is right, is a
  wash (38 vs 37).  The honest residue is that a connector's four pads stay four
  rigid islands with a saddle between them -- visible directly in fieldmap.py.
- a straight segment cannot follow a curved field: split adaptively (15 um
  tolerance here).  This is also what holds tee ends (364 on this board) and
  same-net pads crossed mid-body onto their host -- KiCad connects by overlap,
  not topology, so any of those slipping means a net silently splits.
- read sigma_min as a PERCENTILE, not a minimum: harmonic gradients blow up at
  the sharp corners of a polygonal pad, so the global min is a corner artifact
  that gets worse as the grid is refined while the bulk sits just above 1.
- both engines are exactly linear in (k-1) (pinned residuals are
  (k-1)(origin_i - x), and both solves are linear) -- solve once, scale for free.
  The courtyard relax is the only non-linear part, and it contributes no
  correction at all below +3% on this board.
- pcbnew GetCourtyard geometry disagrees with the DRC courtyard test in BOTH
  directions on this board -- model-based courtyard relax cannot work; let the
  DRC referee footprint micro-moves instead.
- moving U8 at all triggers 9 cosmetic "padstack" warnings on its thermal stack
  (negative mask clearance, pre-existing property); filtered everywhere.
- kicad-cli DRC on this board takes ~1 s: a DRC-in-the-loop referee is cheap.
