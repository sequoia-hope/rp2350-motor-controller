# PCB view

The project page is how the user sees what is happening. They often drive Claude over SSH from a phone, so keep it current. It has two parts:

- **`index.html`** at the project root: the status page. It reads `design/status.json` (phase, what you're doing, what you need from the user, latest check results, gate list) and shows the main board's viewer embedded below it.
- **pcbview** (<https://github.com/sequoia-hope/pcbview>): generates the board viewer from the KiCad files with KiCad's own exporters. **SCH** tab (every sheet, plus a PDF), **PCB** tab (every copper, silk and fab layer, in register, with a part pane), **3D** tab (GLB from `kicad-cli`, click a part for its fields and nets), and an **Overview** (source commit, stackup, counts, parts with no 3D model).

## Use

- pcbview is **pinned as a git submodule** at `vendor/pcbview`. Run it through `scripts/view.sh`, which forwards to `vendor/pcbview/bin/pcbview` (or `$PCBVIEW` if set). Read `vendor/pcbview/README.md` before changing `pcbview.toml`; don't guess its options.
- Needs KiCad 9's `kicad-cli`, KiCad's Python module (`pcbnew`), Python 3.11+ and numpy. `scripts/doctor.sh` checks all of them.
- **Regenerate at every gate** and after any significant change:
  ```sh
  scripts/view.sh build pcbview.toml                   # every board, every tab
  scripts/view.sh build pcbview.toml --only sch        # stages: sch, layers, 3d
  scripts/view.sh build pcbview.toml --board main-a    # one board
  scripts/view.sh models pcbview.toml                  # which parts lack a 3D model, and why
  ```
- Update `design/status.json` at the same time. Then give the user the page link (`./serve.sh` serves the project root) and a one-line summary of what changed.

## `pcbview.toml`: every board and variant

Every board and variant in the project is a `[[boards]]` entry in `pcbview.toml`, so every variant gets the same viewer. The site goes to `viewer/`; one board's viewer is also embedded into `index.html` between the `<!-- pcbview:begin -->` / `<!-- pcbview:end -->` markers.

```toml
[[boards]]
id = "main-b"                        # directory and page name: <board>-<variant>
name = "Main board, variant B"
pcb = "hw/main/b/main.kicad_pcb"
sch = "hw/main/b/main.kicad_sch"
note = "Parent: main-a. Bridge moved to the board edge; 2 oz outer copper."
embed = "index.html"                 # only on the board the status page should show

[[boards]]
id = "main-a"
name = "Main board, variant A (as fabricated)"
git = "main-revA"                    # a fabricated variant is read from its tag, so the page
pcb = "hw/main/a/main.kicad_pcb"     # shows exactly what was built
sch = "hw/main/a/main.kicad_sch"
note = "First layout, 4-layer, single-sided."
```

- A variant's `note` names its parent and what changed.
- Once a variant is fabricated, tag it and add `git = "<tag>"` so its page stops following the working tree.
- Parts whose footprint names a VRML-only model (common with EasyEDA/JLC footprints) draw as orange courtyard boxes. Give them stand-ins under `[models."<file>.wrl"]`; the pcbview README explains how to work out the rotation.

## `design/status.json`

```json
{
  "board": "Farm motor controller",
  "phase": "6 placement",
  "doing": "Placing gate driver block; bridge loop at 4.2 cm²",
  "needs_user": "Approve connector placement in design/brief.md",
  "checks": { "ERC": "0 errors", "DRC": "not run", "sync": "clean" },
  "gates_passed": [0, 1, 2, 3, 4, 5],
  "updated": "2026-10-17T16:30"
}
```

`needs_user` is the most important field: empty when you need nothing, one concrete question when you do.

## Changes to pcbview

pcbview is shared by many projects. **Never edit or copy its code inside a project** (and never commit changes inside `vendor/pcbview`).

If it's missing something:
1. Work around it in the project only with data (config, notes, `status.json`), not with code.
2. Write a change request to `design/pcbview-requests.md`: what's needed, why, proposed interface.
3. Tell the user. Fixes land in pcbview's own repo; the project then bumps its pin (`git -C vendor/pcbview pull && git add vendor/pcbview`).

## Confidential projects

Don't publish the page anywhere public. Serve it locally or to the internal location the user specifies. Remove the `repo` line from `pcbview.toml`.
