"""pcbmorph — Generalized PCB Morphing & DRC-Gated Board Modification Suite.

Core modules:
  - `common`: 2D geometry primitives, KiCad board/signature helpers, config I/O
  - `field`: Per-layer harmonic warp field (`WarpField`) & multi-layer coupled stack (`LayerFieldStack`)
  - `model`: KiCad `.kicad_pcb`/`.kicad_pro`/`.kicad_dru` -> `model.json` extractor
  - `settle`: Feasibility-projected settler with hybrid per-layer harmonic copper flow
  - `gate`: RDP polyline simplification, KiCad checkpoint emission, and 5-check DRC gate
"""
from .common import load_config, data_path, load_json, save_json, resolve_elastic
from .field import WarpField, LayerFieldStack, dilation, dipole, sigma_min_2x2
from .model import extract_model, load_rules, pair_clearance
from .settle import run_settle
from .gate import emit_and_gate

__all__ = [
    'load_config', 'data_path', 'load_json', 'save_json', 'resolve_elastic',
    'WarpField', 'LayerFieldStack', 'dilation', 'dipole', 'sigma_min_2x2',
    'extract_model', 'load_rules', 'pair_clearance',
    'run_settle',
    'emit_and_gate',
]
