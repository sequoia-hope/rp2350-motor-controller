# Module stretch + rev-B placement/routing tooling (2026-08-20)

Pipeline behind `review/modules.html`. Everything reads/writes JSON artefacts
in a work directory (the scripts default to their own directory) and uses
kicad-cli DRC as the oracle; copy `rp2350_driver.kicad_pro` beside every board
copy or the rules are wrong.

    dump_board.py  [board out.json]   footprints/pads/courtyards -> board.json (+ netlist.json via kicad-cli)
    modules.py                        leaf/parent definitions -> modules.json (REPLACE set excluded from geometry)
    slack.py / relax2.py --probe      per-leaf translational slack (exact polygons) -> probe.json
    relax3.py                         sequential-LP coupled stretch -> vectors.json (relax.py/relax2.py: PBD + greedy variants)
    warp2.py                          apply vectors: rigid footprints, connectivity-owned copper (warp.py = spatial IDW, superseded)
    referee.py                        warp -> DRC -> blame -> revert worst leaf -> repeat -> vectors_clean.json
    place_scan.py board jobs.json [--apply --out X]   pose scanner/placer (existing parts, new parts, footprint swaps)
    clean_corner.py in out            remove copper colliding with pads (vias under pads, segments in pad clearance)
    route.py board tag passes         lock all copper, DSN export, Freerouting headless, SES import, DRC delta
    drcdiff.py base.json board out.json   position-independent DRC diff
    map_modules.py / render_page.py   figures + review/modules.html

Lessons: pcbnew courtyard polygons are ~45 um smaller than what DRC tests
(CYMARGIN); custom-shape pads report an empty GetBoundingBox until placed;
Freerouting writes all instances' logs to /tmp/freerouting/freerouting.log.

Routing (2026-08-20): Freerouting with all copper locked hangs on this board;
`router.py in out rounds` is the in-house grid router (see its docstring);
`gnd_stitch.py in out NET` fans plane-net pads out first; `route_result.py`
gives the stats; `draw_new_copper.py` the figure. Hand-off state: DRC 167 -> 4
(cosmetic), airwires 238 -> 54 (named placement fixes, see the page).
