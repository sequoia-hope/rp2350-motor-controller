# rp2350-motor-controller

A PCB project in the Boardroom suite (Boardroom = `~/Software/easy_pcb`, Circuit Composer,
UltraRoute Studio), run with the `pcb-design` skill vendored in `.claude/skills/pcb-design/`
(never hand-edit that copy; `VENDORED_FROM` names the upstream commit). Load it for any board
work. Start every session by reading the skill's `LESSONS.md`, `design/status.json` and
`design/project-map.md`.

- This project predates the Boardroom layout and keeps its own: the KiCad files stay in
  `hardware/` and are **not** moved into `hw/`. Boards are listed by git ref in
  `design/boards.toml`; add a variant there rather than copying files.
- Variants live on branches: `rev-A-fab` (tag, fabricated), `rev-b-fixes` (mainline, where the
  Boardroom files live), `rev-b-lean` and `lean-jiggle` (worktrees beside this checkout; the
  rev-B plan-of-record board is on `lean-jiggle`). Say which branch you are on before editing
  a board.
- No gate has been run under the skill yet. The record is the review hub `review/`
  (findings in `review/findings.html`); cite it as evidence when gates start.
- The review hub is served the registered way: `proj up rp2350-motor-controller`.
- Before writing a `.kicad_pcb` or `.kicad_sch`, check for a running KiCad, `~*.lck` and
  `_autosave-*` beside it. After any hand edit, diff the netlist and the DRC result against
  the previous version.
- Routing with UltraRoute: keep the existing copper. Never pass `--strip`, `--nudge` or
  `--place`; refill zones and run DRC afterwards.
- The repo is public: track only third-party files whose licence allows it (see `.gitignore`).
- Update `design/status.json` (schema 1, Boardroom's `docs/shell-spec.md`) at every gate and
  route; router results go under `apps.studio.<board>`.
- Heavy jobs (routing, settle runs, full DRC sweeps, renders) go through `~/.claude/bin/reserve`.
- When the user corrects you: read `~/.config/boardroom/config.toml`; if its `skill_upstream`
  exists on this machine, add the rule to that copy's `LESSONS.md`; otherwise add it to this
  project's copy.
