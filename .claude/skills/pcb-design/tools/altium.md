# Altium adapter

Used for the user's work projects. Assume confidential: no public repos, URLs, or PCB view publishing unless told otherwise. Follow the company's library, part numbering, templates and release process over anything in this skill where they conflict.

## Automation surfaces (limited compared to KiCad)

Altium automation is mostly GUI-driven. Options, in order of preference:
1. **Exports the user generates** (or that run from an OutJob): DRC/ERC reports, BOM, pick-and-place, gerbers/ODB++/IPC-2581, schematic PDF, STEP. Claude parses these for every check script. Ask the user to run the OutJob when a gate needs fresh output.
2. **IPC-2581 or ODB++** as a readable full-board netlist + geometry for scripted checks (silk overlap, placement, sync, via counts).
3. **DelphiScript / Altium scripting** for repetitive edits, if the user is willing to run scripts.
4. **Rules** expressed as a written table the user enters, or a rules export file (`.RUL`) to import.

Be explicit with the user about which steps Claude can do and which need them to click.

## Mapping

| Skill concept | Altium |
|---------------|--------|
| net classes from `nets.csv` | Net classes + Design Rules (Width, Clearance, Routing Via Style, Differential Pairs) |
| custom DRC rules | Rule queries (`InNetClass`, `InComponent`, room-scoped rules) |
| board-local footprint edit | Project-local PCB library copy, linked through the company library process |
| sync check | ECO / Compare and Update, plus the sync script on exports |
| autorouter | Situs / ActiveRoute, with pre-routed nets locked |
| SI | Altium Signal Integrity, PDN Analyzer if licensed; otherwise the open-source sims |
| PCB view | Generated locally from exports; no public publishing |

## Silkscreen

Board identity uses internal part number and revision per company policy, no repo link.
