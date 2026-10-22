# `pcbmorph` — Generalized PCB Morphing & DRC-Gated Board Modification Suite

`pcbmorph` unifies the continuous per-layer harmonic warp field (`WarpField` / `LayerFieldStack`) and the discrete feasibility-projected line-searched settler (`run_settle`) behind a single shared Python package and 5-check differential DRC verification gate.

---

## Package Structure

| Module | Responsibility |
|---|---|
| `pcbmorph.common` | Shared constants (`NM`, `COPPER_LAYERS`, `F_CU`, `B_CU`, `LAYER_NAMES`), config I/O (`load_config`, `data_path`, `save_json`, `load_json`), `quiet_stderr`, `load_board`, copper signatures (`tsig`, `vsig`, `sig_key`), 2D geometry primitives (`pt_seg_dist`, `seg_seg_dist`, `point_in_poly`, `poly_seg_dist`, `poly_pt_dist`, `poly_poly_dist`, `inside_dist`, `inflate_poly`, `in_rect`, `seg_hits_rect`, `rdp`), and `resolve_elastic`. |
| `pcbmorph.field` | Single-layer harmonic Laplace solver (`WarpField`, with cached SuperLU factorization `splu` for ~1-2 ms vector updates), analytical boundary/dipole maps (`dilation`, `dipole`), 2x2 Jacobian singular-value diagnostics (`sigma_min_2x2`), and the multi-layer barrel-coupled stack (`LayerFieldStack`). |
| `pcbmorph.model` | Stage 1 extractor (`extract_model`): parses `.kicad_pcb`, `.kicad_pro`, and `.kicad_dru` into `model.json` (project netclass clearance matrix, footprint hulls/courtyards, pad polygons and pad centroids, steerable field target evaluation, subdivided copper graph, pad/zone-island attachments, and ghost obstacles). |
| `pcbmorph.settle` | Stage 2 solver (`run_settle`): couples the per-layer harmonic warp field (`LayerFieldStack`) with the DRC-floor line-searched feasibility settler, stagnation detector, dynamic segment splitter, and mid-flight multi-layer A* rerouter. |
| `pcbmorph.gate` | Stage 3 verifier (`emit_and_gate`): applies `rdp()` polyline simplification, emits the checkpoint `.kicad_pcb`, refills zones, and runs the 5-check verification gate (static copper byte-identity, net-length caps, differential DRC fingerprint comparison, total DRC regression check, and `check_sync`). |

---

## CLI Usage

Run from the repository root with `PYTHONPATH=hardware/tools`:

```bash
# Run individual stages:
PYTHONPATH=hardware/tools python3 -m pcbmorph model  hardware/tools/pcbmorph/configs/power_squeeze.json
PYTHONPATH=hardware/tools python3 -m pcbmorph settle hardware/tools/pcbmorph/configs/power_squeeze.json --field
PYTHONPATH=hardware/tools python3 -m pcbmorph gate   hardware/tools/pcbmorph/configs/power_squeeze.json

# Or run the full pipeline (model -> settle -> gate):
PYTHONPATH=hardware/tools python3 -m pcbmorph run    hardware/tools/pcbmorph/configs/dipole_fan_push.json --field

# Run a board-shape modification (Edge.Cuts notch + coupled interior copper/footprint morph):
PYTHONPATH=hardware/tools python3 -m pcbmorph run    hardware/tools/pcbmorph/configs/top_edge_notch.json --field

# Compare classic reactive settle vs. hybrid harmonic-field settle:
PYTHONPATH=hardware/tools python3 -m pcbmorph settle hardware/tools/pcbmorph/configs/power_squeeze.json --no-field
PYTHONPATH=hardware/tools python3 -m pcbmorph settle hardware/tools/pcbmorph/configs/power_squeeze.json --field
```

---

## Hybrid Operator: Coupled Harmonic Field + Feasibility-Projected Settler

In classic `settle` (`--no-field`), free copper nodes (`nid not in fixed and nid not in part_of`) remain stationary until a moving pad or trace physically collides with them (`al < 1.0`, depositing point pressure `press[nid]`) or a sub-segment stretches past `1.6 * SUB`. In tight multi-net corridors, traces pile up one-by-one and form sharp local kinks.

When `--field` (or `"field": {"enabled": true}`) is active:
1. **Pre-factored Per-Layer Laplace Stack (`LayerFieldStack`):**
   - Builds one `WarpField` per active copper layer (`F.Cu`, `In1.Cu`..`In4.Cu`, `B.Cu`).
   - Registers each movable footprint's pads (inflated by `pad_inflate` and morphologically closed across intra-footprint gaps via `bridge_mm`) on the copper layers those pads inhabit.
   - Registers deformed board-edge segments (`__edge_k__`, when `cfg["shape"]` is active) across **all** copper layers so inner-layer corridors (`In2.Cu`, `In3.Cu`) without SMD pads still experience the advancing boundary's harmonic flow.
   - Pins all static pads on their layer and pins all through-hole drills and static vias across **all** copper layers so no layer's harmonic flow sweeps copper across a stationary barrel.
   - Pre-factors the discrete Laplace matrix `A` once per layer via `scipy.sparse.linalg.splu`.
2. **Lookahead Coupled Field Update (`update_harmonic_field`):**
   - Each cycle, `LayerFieldStack.update(part_vecs, s_edge)` updates the Dirichlet inclusion vectors to $V_r = D_r + \beta \cdot v_{\text{rem},r}$ (realized displacement plus a bounded lookahead fraction $\beta$) and re-solves every copper layer using the cached LU factors in ~1-2 ms/layer.
   - Dynamic vias are barrel-coupled across layers by averaging $U_L(x_0, y_0)$ across all copper layers incident on the via.
3. **DRC-Projected Laminar Copper Flow (`copper_pass`):**
   - Each free node receives a smooth spatial harmonic drift $v_{\text{field}}(\text{nid}) = \gamma \cdot (P_{\text{harm}}(\text{nid}) - P[\text{nid}])$ and anchors its gentle rest-shape-preserving displacement Laplacian spring $\nabla^2(P - P_0)$ to $P_{\text{harm}}(\text{nid})$.
   - Every proposed node step is still projected through `line_search()` against the exact per-edge DRC floor `req_f(g) = min(g.rule + GUARD, g.d0)`, attachments, courtyards, and net-length caps.
4. **Steerable Dipoles & Ambient Fields (`cfg["field"]`):**
   - Instead of hand-authoring `(dx, dy)` targets for every passive in a crowded region, configs can specify `field.ambient` (`dilation`, `shift`, `dipole`) and/or `field.dipoles` (`[{"x", "y", "vector": [dx, dy], "radius", "decay"}]`), which `model.py` evaluates at each movable footprint's **pad centroid** (`pad_cx, pad_cy`).
5. **Board-Shape Morphing (`cfg["shape"]`):**
   - Supports `"type": "notch"` (`{"edge": "top"|"bottom"|"left"|"right", "span": [s0, s1], "depth_mm": d, "shoulder_mm": sh}`) and `"type": "edge_shift"` (`{"edge": ..., "delta_mm": d}`).
   - `settle.py` advances the board outline homotopy $O(s_{\text{edge}}) = (1 - s_{\text{edge}}) O_0 + s_{\text{edge}} O_1$ via `shape_pass` with non-convex exact segment/polygon inside-distance (`inside_seg_dist`, `inside_poly_dist`) and coupled boundary-layer co-moves (`mv_edge_coupled`).
   - `gate.py` emits the realized `Edge.Cuts` segments into the checkpoint `.kicad_pcb`, refills all copper zones (`ZONE_FILLER`), and verifies 0 `copper_edge_clearance` or `malformed_outline` errors.
