# Review protocol

The agent that did the work is not allowed to be the one that says it's good enough. Every gate gets an independent review by a fresh-context subagent, using the stronger model (e.g. Fable) for judgment-heavy gates.

## Rules

- The reviewer gets **artifacts and the checklist only**: files, renders, tool output. Not the conversation, not the author's explanation of why things are fine.
- The reviewer must look at the actual rendered images for any visual check.
- The reviewer re-derives instead of re-reading: it recomputes calculations from the datasheet, rather than checking the author's arithmetic.
- Findings are ranked: **BLOCKER** (must fix), **MAJOR** (fix unless justified in `decisions.md`), **MINOR** (fix if cheap), **NOTE**.
- The author fixes or disputes each finding. Disputes go in `decisions.md`. If BLOCKERs or MAJORs were fixed, re-review.
- Findings and resolutions are pasted into the gate file.

## Which model

| Gate | Reviewer |
|------|----------|
| 0 intent | stronger model: what's missing or underspecified? |
| 1 datasheets | stronger model, **one subagent per complex part** in parallel |
| 2 schematic | stronger model with rendered schematic |
| 3 nets | main model is fine; stronger if the board has SI/EMC risks |
| 4 brief | stronger model |
| 5 rules | main model with checklist |
| 6 placement | stronger model with renders |
| 7 routing | main model for checks; stronger for render review |
| 8 sim | stronger model |
| 9 silk | stronger model with renders |
| 10 release | stronger model, full package |

## Prompt templates

### Datasheet cross-check
> You are reviewing one part on a PCB design. Read the attached datasheet in full. Then read the audit file and the schematic excerpt for this part. Independently: (1) list every external part the datasheet requires or recommends and check it's in the schematic with a suitable value, voltage and dielectric; (2) recompute every calculation from the datasheet formulas using the operating conditions in intent.md; (3) list every footnote, warning or condition in the datasheet that the design may violate; (4) check the footprint against the package drawing for this exact MPN. Report findings as BLOCKER / MAJOR / MINOR / NOTE with datasheet page numbers.

### Schematic visual review
> You are a senior hardware engineer reviewing a schematic for readability. Here are the rendered sheets. A schematic should read like a person drew it: grouped by function, signal flowing left to right, power top and ground bottom, supporting parts next to the pins they serve, wires (not labels) within a block. Report anything that looks machine-generated, hard to follow, cramped, or ambiguous, with the location. Then: could you trace the power path from input to every load? Rate overall, and list findings by severity.

### Placement / silk review
> Here are top, bottom and 3D renders of a PCB with ratsnest, plus the floorplan from brief.md. Assess as an experienced layout engineer: block grouping and flow, passives with their ICs, tightness of critical loops (list them), separation of noisy and sensitive circuits, routability, thermal, and assembly. For silkscreen: every designator visible and clearly associated, no overlaps, every connector labeled with function/polarity/rating, board identity present. Findings by severity, with locations.

### Final release
> You are the final reviewer before this board goes to fab. You have the full design package. Would you send it? Identify the highest-risk issue, any requirement in intent.md not met, and anything that would make bring-up painful.
