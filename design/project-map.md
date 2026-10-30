# Project map

Where things are in this repository, for Boardroom and for anyone opening it cold.
This project is older than the Boardroom layout and the `pcb-design` phases, so it
keeps its own layout: nothing was moved into `hw/` (the research tooling, the review
hub and three branches name `hardware/` paths). Boards are listed by git ref in
[`boards.toml`](boards.toml).

## Branches

| Branch | What it is |
|---|---|
| `main` | Firmware, plus the rev-A hardware as found before rev-B work began |
| `rev-b-fixes` | Mainline: the rev-B schematic, the re-placed rev-B board, the review hub (`review/`) and most of the research tooling. Boardroom project files live here |
| `rev-b-lean` | Rev-A copper + rev-B schematic + Tier A footprints: the base rev-B routing restarted from |
| `lean-jiggle` | `rev-b-lean` + the settle and encoder-layout work; holds the plan-of-record board |
| tag `rev-A-fab` | The fabricated board (c6b9b75) |

`rev-b-lean` and `lean-jiggle` are checked out as worktrees beside this one
(`../rp2350-motor-controller-rev-b-lean`, `../rp2350-motor-controller-lean-jiggle`).

## Folders

| Path | Contents |
|---|---|
| `hardware/` | KiCad projects: `rp2350_driver.*` (the controller), `encoder/` (the encoder board), `parts/` and `RP2040_minimal.pretty/` (project libraries), `production/` (rev-A fab files), `docs/` (datasheets) |
| `hardware/tools/` | Research tooling: `pcbmorph` (warp field, settler, DRC gate), `rubber`, `jiggle2`, `modules`, `diffusion`, `revb`. On `lean-jiggle` also `settle`, `enclayout`, `lean`. Their run data (`data/`, `runs/`) is gitignored and local to each worktree |
| `review/` | The design-review hub: findings, rev-A proof, rev-B schematic changes, every study's write-up, the rev-A viewer. Served with `proj up rp2350-motor-controller` |
| `firmware/` | RP2350 firmware |
| `design/` | Boardroom's files: `status.json`, `boards.toml`, this map |
| `compose/` | Circuit Composer's view of the controller (`board.json` pointing at the KiCad board; read-only use) |
| `tmp/` | Gitignored scratch (several GB of run output); nothing in it is the record |

## The record so far

No gate has been run under the `pcb-design` skill. What exists instead:

- [`review/findings.html`](../review/findings.html): every finding with severity and status
- [`review/rev-a.html`](../review/rev-a.html): proof that the gerbers in `hardware/production/` match tag `rev-A-fab`
- [`review/revb-schematic.html`](../review/revb-schematic.html): the rev-B schematic changes
- [`review/settle/renders.html`](../review/settle/renders.html): the plan-of-record board, and how to land it
- [`review/settle/enclayout.html`](../review/settle/enclayout.html): the encoder-corner alternatives

When gates start, phases 0–6 for rev B can cite these pages as evidence rather
than redoing them.

## Routing with UltraRoute

Studio opens boards from `hw/<board>/<variant>/`, which this project does not have, so
use the CLI on a board file directly. Keep the existing copper: never pass `--strip`
(it deletes every unlocked track) or `--nudge`/`--place` (moving a part removes the
unlocked tracks on its pads). UltraRoute applies only some `.kicad_dru` rules and does
not fill zones; refill and run DRC afterwards.
