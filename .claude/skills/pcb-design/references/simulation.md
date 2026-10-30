# Phase 8: simulation and signal integrity

Simulate what is likely to fail or hard to fix later. Not every net. The triggers come from `nets.csv`; a judgment pass by the stronger model adds anything the triggers miss.

## Triggers

| Situation | Simulation |
|-----------|-----------|
| Any DIFF_HS or SE_HS above length threshold | impedance + crosstalk (2D field solver on actual cross-section; 2.5D/3D for vias and breakouts if > ~1 GHz content) |
| RF | antenna match and pattern (openEMS), matching network in circuit sim |
| SWITCH_HI near ANALOG/CLOCK | coupling estimate: aggressor/victim spacing, overlap length, mutual capacitance/inductance |
| POWER_HI with tight drop budget | DC IR drop / current density (KiCad has none built in; use a script or Altium PDN Analyzer, or openEMS DC) |
| Switching converter, control loop, filters | SPICE (ngspice) with vendor models: loop stability, startup, transient |
| Motor bridge | SPICE of bridge with parasitics from layout: ringing, gate drive, dead time |
| Thermal hot spots | spreadsheet estimate first; FEM only if the estimate is marginal |

## Aggressor / victim check

A script lists, for every pair of nets where one is an aggressor class (SWITCH, SWITCH_HI, CLOCK, DIFF_HS, SE_HS) and the other is a victim class (ANALOG, ANALOG_DIFF, CLOCK, RF, sensitive SIGNAL), any segments that run parallel within a spacing threshold (default 3× trace height above plane) for more than a length threshold, on the same or adjacent layers. Each finding is fixed, simulated, or waived with reasoning.

## Tooling (open source first)

- ngspice for circuit sim (KiCad's integrated sim or standalone)
- openEMS for EM (RF, via transitions, antenna)
- a 2D field solver for cross-sections (e.g. atlc2, or a Python one)
- For work projects, use whatever the company has (e.g. Altium SI, HyperLynx, Ansys).

## Output

`design/sim/<name>/` containing: the question being answered, setup, input files, results (plots as PNG), conclusion, and what changed in the design. Each sim gets a link from the project page (`index.html`).

## Gate checklist (phase 8)

- [ ] Trigger list generated from `nets.csv` (pasted)
- [ ] Stronger-model SI/EMC review added anything missed
- [ ] Each triggered item has a result or a waiver in `decisions.md`
- [ ] Aggressor/victim script run, findings resolved
- [ ] Design changes from sims fed back (stale gates re-run)
