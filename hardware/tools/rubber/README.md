# Rubber stretch + violation nudging (2026-08-24)

Answer to: "stretch the whole board like rubber, tolerate the damage with
local trace nudges, hand the rest to a human."  Page: `review/rubber.html`.

    stretch.py   KX/KY dilation about the board centre; footprints rigid
                 (vector = dilation of their origin), copper = ambient field
                 + graph-harmonic residuals so trace ends stay glued to pads;
                 zones/silk/edge follow; corner arcs map exactly; courtyard
                 relax pre-corrects rigid-lag pair closures.
                   SRC=... OUT=... KX=1.01 KY=1.01 python3 stretch.py
    nudge.py     DRC-refereed local repair: cluster new violations into sites,
                 push offending track chains apart (deficit+margin, falloff
                 2 mm), tee ends and via-rim ends RIDE their bar, vias movable,
                 courtyard overlaps fixed by footprint micro-moves.
                   python3 nudge.py BOARD BASELINE_DRC.json LOG.json [ROUNDS]
    status.py    the manual-repair loop: DRC diff vs pre-stretch baseline,
                 site list worst-first, airwires, check_sync.
    gaps.py      payoff probe: pad gaps at the F-25 boxed-in blockers.

Lessons (hard-won, see also ../modules/README.md):
- pure position-scaling is illegal at any k: pad copper lags full dilation by
  (k-1)*|pad offset from footprint origin| and sweeps through at-limit copper
  (F-46 on the page).  Strain must route around rigid footprints.
- track ends that sit on a segment BODY (tees, 364 on this board) or on a via
  rim are overlap-connections: any local mover must make them ride, or nets
  silently split.  Same for segments crossing same-net pads mid-body.
- pcbnew GetCourtyard geometry disagrees with the DRC courtyard test in BOTH
  directions on this board -- model-based courtyard relax cannot work; let the
  DRC referee footprint micro-moves instead.
- moving U8 at all triggers 9 cosmetic "padstack" warnings on its thermal
  stack (negative mask clearance, pre-existing property); filtered everywhere.
- kicad-cli DRC on this board takes ~1 s: a DRC-in-the-loop referee is cheap.
