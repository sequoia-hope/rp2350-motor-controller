"""Stage 1: Exact KiCad board -> PCB morph model extractor.

Extracts:
  - project netclasses & custom rule matrix (from .kicad_pro + .kicad_dru)
  - footprints (exact silk/fab/courtyard hulls, pad polygons, pad centroids)
  - steerable field targets (optional ambient dilation / localized dipoles
    evaluated at each movable footprint's pad centroid)
  - active vs. static copper classification (including elastic envelope)
  - subdivided copper graph (nodes, segments, attachments, zone-island locks)
  - pre-inflated ghost obstacles for multi-layer net-change routing
"""
import os
import re
import sys
import json
import math
from collections import defaultdict
import pcbnew

from .common import (
    NM, COPPER_LAYERS, F_CU, B_CU,
    load_config, data_path, save_json, load_board,
    tsig, vsig, sig_key,
    pt_seg_dist, point_in_poly, poly_seg_dist, poly_pt_dist,
    inflate_poly, in_rect, seg_hits_rect, resolve_elastic,
)
from .field import compose_ambient, dipole


# ---------------------------------------------------------------------------
# Rules extraction (.kicad_pro + .kicad_dru)
# ---------------------------------------------------------------------------

def load_rules(board_path, board):
    pro_path = os.path.splitext(board_path)[0] + '.kicad_pro'
    dru_path = os.path.splitext(board_path)[0] + '.kicad_dru'
    ds = board.GetDesignSettings()

    rules = {
        'default_clearance': getattr(ds, 'm_MinClearance', 150_000) / NM or 0.15,
        'hole_clearance': getattr(ds, 'm_HoleClearance', 250_000) / NM or 0.25,
        'hole_to_hole': getattr(ds, 'm_HoleToHoleMin', 250_000) / NM or 0.25,
        'copper_edge': getattr(ds, 'm_CopperEdgeClearance', 300_000) / NM or 0.30,
        'min_track': getattr(ds, 'm_TrackMinWidth', 100_000) / NM or 0.10,
        'min_via': getattr(ds, 'm_ViasMinSize', 500_000) / NM or 0.50,
        'min_drill': getattr(ds, 'm_MinThroughDrill', 200_000) / NM or 0.20,
        'netclasses': {},
        'net_to_class': {},
        'class_matrix': {},
    }

    if os.path.exists(pro_path):
        with open(pro_path) as f:
            pro = json.load(f)
        p_rules = pro.get('board', {}).get('design_settings', {}).get('rules', {})
        if 'min_clearance' in p_rules and p_rules['min_clearance'] > 0:
            rules['default_clearance'] = float(p_rules['min_clearance'])
        if 'min_hole_clearance' in p_rules and p_rules['min_hole_clearance'] > 0:
            rules['hole_clearance'] = float(p_rules['min_hole_clearance'])
        if 'min_hole_to_hole' in p_rules and p_rules['min_hole_to_hole'] > 0:
            rules['hole_to_hole'] = float(p_rules['min_hole_to_hole'])
        if 'min_copper_edge_clearance' in p_rules and p_rules['min_copper_edge_clearance'] > 0:
            rules['copper_edge'] = float(p_rules['min_copper_edge_clearance'])
        ns = pro.get('net_settings', {})
        for nc in ns.get('classes', []):
            name = nc.get('name', 'Default')
            rules['netclasses'][name] = {
                'clearance': float(nc.get('clearance', rules['default_clearance'])),
                'track_width': float(nc.get('track_width', 0.2)),
                'via_diameter': float(nc.get('via_diameter', 0.6)),
                'via_drill': float(nc.get('via_drill', 0.3)),
            }
        for net, cls in (ns.get('netclass_assignments') or {}).items():
            rules['net_to_class'][net] = cls[0] if isinstance(cls, list) else cls
        for pat in (ns.get('netclass_patterns') or []):
            rx = re.compile('^' + pat['pattern'].replace('*', '.*').replace('?', '.') + '$')
            nc = pat['netclass']
            for net_code, net_item in board.GetNetsByNetcode().items():
                nname = net_item.GetNetname()
                if nname not in rules['net_to_class'] and rx.match(nname):
                    rules['net_to_class'][nname] = nc

    if os.path.exists(dru_path):
        with open(dru_path) as f:
            dru = f.read()
        blocks = re.findall(r'\(rule\s+"([^"]+)"(.*?)\n\)', dru, re.S)
        for rname, body in blocks:
            classes = set(re.findall(r"NetClass\s*==\s*'([^']+)'", body))
            m = re.search(r'\(constraint\s+clearance\s+\(min\s+([0-9.]+)mm\)', body)
            if not m:
                continue
            val = float(m.group(1))
            has_b_not_a = ('B.NetClass' in body and '!=' in body)
            if len(classes) == 2:
                c1, c2 = sorted(classes)
                rules['class_matrix'][f'{c1}|{c2}'] = max(
                    rules['class_matrix'].get(f'{c1}|{c2}', 0.0), val)
            elif len(classes) == 1 and has_b_not_a:
                c1 = list(classes)[0]
                for c2 in rules['netclasses']:
                    if c2 != c1:
                        k = '|'.join(sorted([c1, c2]))
                        rules['class_matrix'][k] = max(rules['class_matrix'].get(k, 0.0), val)

    return rules


def pair_clearance(rules, net_a, net_b):
    if net_a == net_b and net_a != '':
        return 0.0
    ca = rules['net_to_class'].get(net_a, 'Default')
    cb = rules['net_to_class'].get(net_b, 'Default')
    cla = rules['netclasses'].get(ca, {}).get('clearance', rules['default_clearance'])
    clb = rules['netclasses'].get(cb, {}).get('clearance', rules['default_clearance'])
    base = max(cla, clb, rules['default_clearance'])
    if ca != cb:
        k = '|'.join(sorted([ca, cb]))
        base = max(base, rules['class_matrix'].get(k, 0.0))
    return base


# ---------------------------------------------------------------------------
# Hull / courtyard extraction
# ---------------------------------------------------------------------------

def shape_points(gs):
    if not hasattr(gs, 'GetShape'):
        return []
    st = gs.GetShape()
    pts = []
    if st in (pcbnew.SHAPE_T_SEGMENT, pcbnew.SHAPE_T_RECT):
        s, e = gs.GetStart(), gs.GetEnd()
        pts.extend([(s.x / NM, s.y / NM), (e.x / NM, e.y / NM)])
        if st == pcbnew.SHAPE_T_RECT:
            pts.extend([(s.x / NM, e.y / NM), (e.x / NM, s.y / NM)])
    elif st == pcbnew.SHAPE_T_CIRCLE:
        c = gs.GetCenter()
        r = gs.GetRadius() / NM
        for k in range(12):
            ang = 2 * math.pi * k / 12
            pts.append((c.x / NM + r * math.cos(ang), c.y / NM + r * math.sin(ang)))
    elif st == pcbnew.SHAPE_T_POLY:
        ps = gs.GetPolyShape()
        for ci in range(ps.OutlineCount()):
            ol = ps.Outline(ci)
            for pi in range(ol.PointCount()):
                p = ol.CPoint(pi)
                pts.append((p.x / NM, p.y / NM))
    else:
        bb = gs.GetBoundingBox()
        pts.extend([(bb.GetLeft() / NM, bb.GetTop() / NM),
                    (bb.GetRight() / NM, bb.GetBottom() / NM)])
    return pts


def extract_courtyard(fp, pads):
    try:
        fp.BuildCourtyardCaches()
    except AttributeError:
        pass
    cy_pts = []
    for lay in (pcbnew.F_CrtYd, pcbnew.B_CrtYd):
        ps = fp.GetCourtyard(lay)
        for ci in range(ps.OutlineCount()):
            ol = ps.Outline(ci)
            for pi in range(ol.PointCount()):
                pt = ol.CPoint(pi)
                cy_pts.append((round(pt.x / NM, 4), round(pt.y / NM, 4)))
    if len(cy_pts) >= 3:
        # If single outline polygon, return its exact vertices inflated by 2 um
        side_lay = pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd
        ps = fp.GetCourtyard(side_lay)
        if ps.OutlineCount() == 1 and ps.Outline(0).PointCount() >= 3:
            ol = ps.Outline(0)
            pts = [(round(ol.CPoint(pi).x / NM, 4), round(ol.CPoint(pi).y / NM, 4))
                   for pi in range(ol.PointCount())]
            return inflate_poly(pts, 0.002)
        cxs = [p[0] for p in cy_pts]
        cys = [p[1] for p in cy_pts]
        x0, x1 = min(cxs) - 0.005, max(cxs) + 0.005
        y0, y1 = min(cys) - 0.005, max(cys) + 0.005
        return [(round(x0, 4), round(y0, 4)), (round(x1, 4), round(y0, 4)),
                (round(x1, 4), round(y1, 4)), (round(x0, 4), round(y1, 4))]

    brd = fp.GetBoard()
    b_fab = brd.GetLayerID('B.Fab')
    f_fab = brd.GetLayerID('F.Fab')
    b_silk = brd.GetLayerID('B.SilkS')
    f_silk = brd.GetLayerID('F.SilkS')

    fab_pts, silk_pts = [], []
    for gi in fp.GraphicalItems():
        lid = gi.GetLayer()
        if lid in (b_fab, f_fab):
            fab_pts.extend(shape_points(gi))
        elif lid in (b_silk, f_silk):
            silk_pts.extend(shape_points(gi))

    pad_pts = []
    for p in pads:
        for px, py in p['poly']:
            pad_pts.append((px, py))

    all_pts = silk_pts + fab_pts + pad_pts
    if not all_pts:
        pos = fp.GetPosition()
        cx, cy = pos.x / NM, pos.y / NM
        return [(cx - 0.5, cy - 0.5), (cx + 0.5, cy - 0.5),
                (cx + 0.5, cy + 0.5), (cx - 0.5, cy + 0.5)]

    xs = [p[0] for p in all_pts]
    ys = [p[1] for p in all_pts]
    pad_m = 0.08 if (silk_pts or fab_pts) else 0.15
    x0, x1 = min(xs) - pad_m, max(xs) + pad_m
    y0, y1 = min(ys) - pad_m, max(ys) + pad_m
    if cy_pts:
        cxs = [p[0] for p in cy_pts]
        cys = [p[1] for p in cy_pts]
        x0, x1 = min(x0, min(cxs)), max(x1, max(cxs))
        y0, y1 = min(y0, min(cys)), max(y1, max(cys))
    return [(round(x0, 4), round(y0, 4)), (round(x1, 4), round(y0, 4)),
            (round(x1, 4), round(y1, 4)), (round(x0, 4), round(y1, 4))]


def extract_pads(fp):
    ref = fp.GetReference()
    pads = []
    for pad in fp.Pads():
        pos = pad.GetPosition()
        px, py = pos.x / NM, pos.y / NM
        lset = pad.GetLayerSet()
        layers = [l for l in COPPER_LAYERS if lset.Contains(l)]
        poly_pts = []
        if layers:
            ps = pad.GetEffectivePolygon(layers[0])
            if ps.OutlineCount() > 0:
                ol = ps.Outline(0)
                n = ol.PointCount()
                step = max(1, n // 16)
                for i in range(0, n, step):
                    pt = ol.CPoint(i)
                    poly_pts.append((round(pt.x / NM, 4), round(pt.y / NM, 4)))
        if not poly_pts:
            sz = pad.GetSize()
            hw, hh = sz.x / (2 * NM), sz.y / (2 * NM)
            poly_pts = [(round(px - hw, 4), round(py - hh, 4)),
                        (round(px + hw, 4), round(py - hh, 4)),
                        (round(px + hw, 4), round(py + hh, 4)),
                        (round(px - hw, 4), round(py + hh, 4))]
        drill = pad.GetDrillSize().x / NM if pad.GetDrillSize().x > 0 else 0.0
        pads.append({
            'ref': ref,
            'num': pad.GetNumber(),
            'net': pad.GetNetname(),
            'x': round(px, 4), 'y': round(py, 4),
            'layers': layers,
            'poly': poly_pts,
            'drill': round(drill, 4),
            'is_smd': drill == 0.0,
        })
    return pads


def extract_zones(board):
    zones = []
    for z in board.Zones():
        try:
            if z.GetIsRuleArea():
                continue
        except AttributeError:
            pass
        net = z.GetNetname()
        if not net:
            continue
        lset = z.GetLayerSet()
        zlays = [l for l in COPPER_LAYERS if lset.Contains(l)]
        ol = z.Outline()
        for ci in range(ol.OutlineCount()):
            o = ol.Outline(ci)
            n = o.PointCount()
            if n < 3:
                continue
            pts = [(round(o.CPoint(i).x / NM, 4), round(o.CPoint(i).y / NM, 4)) for i in range(n)]
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            for l in zlays:
                zones.append({
                    'net': net, 'layer': l,
                    'bbox': [round(min(xs), 4), round(min(ys), 4),
                             round(max(xs), 4), round(max(ys), 4)],
                    'poly': pts,
                })
    return zones


def extract_board_outline(board):
    edge_id = board.GetLayerID('Edge.Cuts')
    pts = []
    for d in board.GetDrawings():
        if d.GetLayer() == edge_id:
            if hasattr(d, 'GetStart') and hasattr(d, 'GetEnd'):
                s, e = d.GetStart(), d.GetEnd()
                pts.append((s.x / NM, s.y / NM))
                pts.append((e.x / NM, e.y / NM))
            else:
                pts.extend(shape_points(d))
    if not pts:
        return []
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return [(round(min(xs), 4), round(min(ys), 4)),
            (round(max(xs), 4), round(min(ys), 4)),
            (round(max(xs), 4), round(max(ys), 4)),
            (round(min(xs), 4), round(max(ys), 4))]


def build_shape_outlines(base_outline, shape_cfg):
    """Build vertex-matched `(outline0, target_outline, shape_box)` for `cfg['shape']`.

    Supports:
      - `type = 'notch'`: indents `edge` (`top` | `bottom` | `left` | `right`) over
        `span = [s0, s1]` by `depth_mm` inward with chamfer shoulders `shoulder_mm`.
      - `type = 'edge_shift'`: shifts `edge` inward by `delta_mm` (or `depth_mm`).
    """
    if not base_outline or not shape_cfg or not isinstance(shape_cfg, dict):
        return base_outline, base_outline, None

    xs = [p[0] for p in base_outline]
    ys = [p[1] for p in base_outline]
    x0, y0, x1, y1 = min(xs), min(ys), max(xs), max(ys)
    stype = shape_cfg.get('type', 'notch')
    edge = shape_cfg.get('edge', 'top').lower()

    if stype == 'notch':
        s0, s1 = float(shape_cfg['span'][0]), float(shape_cfg['span'][1])
        if s0 > s1:
            s0, s1 = s1, s0
        depth = float(shape_cfg.get('depth_mm', 1.0))
        sh = float(shape_cfg.get('shoulder_mm', 1.0))
        if edge == 'top':
            o0 = [(x0, y0), (s0 - sh, y0), (s0, y0), (s1, y0), (s1 + sh, y0),
                  (x1, y0), (x1, y1), (x0, y1)]
            o1 = [(x0, y0), (s0 - sh, y0), (s0, round(y0 + depth, 4)),
                  (s1, round(y0 + depth, 4)), (s1 + sh, y0),
                  (x1, y0), (x1, y1), (x0, y1)]
            sbox = [s0 - sh, min(y0, y0 + depth), s1 + sh, max(y0, y0 + depth)]
        elif edge == 'bottom':
            o0 = [(x0, y0), (x1, y0), (x1, y1),
                  (s1 + sh, y1), (s1, y1), (s0, y1), (s0 - sh, y1), (x0, y1)]
            o1 = [(x0, y0), (x1, y0), (x1, y1),
                  (s1 + sh, y1), (s1, round(y1 - depth, 4)),
                  (s0, round(y1 - depth, 4)), (s0 - sh, y1), (x0, y1)]
            sbox = [s0 - sh, min(y1, y1 - depth), s1 + sh, max(y1, y1 - depth)]
        elif edge == 'left':
            o0 = [(x0, y0), (x1, y0), (x1, y1), (x0, y1),
                  (x0, s1 + sh), (x0, s1), (x0, s0), (x0, s0 - sh)]
            o1 = [(x0, y0), (x1, y0), (x1, y1), (x0, y1),
                  (x0, s1 + sh), (round(x0 + depth, 4), s1),
                  (round(x0 + depth, 4), s0), (x0, s0 - sh)]
            sbox = [min(x0, x0 + depth), s0 - sh, max(x0, x0 + depth), s1 + sh]
        elif edge == 'right':
            o0 = [(x0, y0), (x1, y0),
                  (x1, s0 - sh), (x1, s0), (x1, s1), (x1, s1 + sh),
                  (x1, y1), (x0, y1)]
            o1 = [(x0, y0), (x1, y0),
                  (x1, s0 - sh), (round(x1 - depth, 4), s0),
                  (round(x1 - depth, 4), s1), (x1, s1 + sh),
                  (x1, y1), (x0, y1)]
            sbox = [min(x1, x1 - depth), s0 - sh, max(x1, x1 - depth), s1 + sh]
        else:
            return base_outline, base_outline, None
        return o0, o1, sbox

    if stype == 'edge_shift':
        delta = float(shape_cfg.get('delta_mm', shape_cfg.get('depth_mm', 1.0)))
        o0 = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        if edge == 'top':
            o1 = [(x0, round(y0 + delta, 4)), (x1, round(y0 + delta, 4)), (x1, y1), (x0, y1)]
            sbox = [x0, min(y0, y0 + delta), x1, max(y0, y0 + delta)]
        elif edge == 'bottom':
            o1 = [(x0, y0), (x1, y0), (x1, round(y1 - delta, 4)), (x0, round(y1 - delta, 4))]
            sbox = [x0, min(y1, y1 - delta), x1, max(y1, y1 - delta)]
        elif edge == 'left':
            o1 = [(round(x0 + delta, 4), y0), (x1, y0), (x1, y1), (round(x0 + delta, 4), y1)]
            sbox = [min(x0, x0 + delta), y0, max(x0, x0 + delta), y1]
        elif edge == 'right':
            o1 = [(x0, y0), (round(x1 - delta, 4), y0), (round(x1 - delta, 4), y1), (x0, y1)]
            sbox = [min(x1, x1 - delta), y0, max(x1, x1 - delta), y1]
        else:
            return base_outline, base_outline, None
        return o0, o1, sbox

    return base_outline, base_outline, None


def apply_shape_targets(cfg, parts, base_outline):
    """Assign inward boundary-push targets to movable parts near a deformed board edge."""
    scfg = cfg.get('shape')
    if not scfg or not isinstance(scfg, dict) or not scfg.get('auto_targets', True):
        return
    if not base_outline:
        return
    xs = [p[0] for p in base_outline]
    ys = [p[1] for p in base_outline]
    x0, y0, x1, y1 = min(xs), min(ys), max(xs), max(ys)

    edge = scfg.get('edge', 'top').lower()
    depth = float(scfg.get('depth_mm', scfg.get('delta_mm', 1.0)))
    sh = float(scfg.get('shoulder_mm', 1.0))
    span = scfg.get('span')
    if span:
        s0, s1 = min(float(span[0]), float(span[1])) - sh, max(float(span[0]), float(span[1])) + sh
    else:
        s0, s1 = (x0, x1) if edge in ('top', 'bottom') else (y0, y1)

    inf_mm = float(scfg.get('influence_mm', 6.5))
    lat_mm = float(scfg.get('lateral_margin_mm', 2.0))
    push_gain = float(scfg.get('part_gain', 1.15))

    for ref, pt in parts.items():
        if abs(pt['tx']) > 1e-6 or abs(pt['ty']) > 1e-6:
            continue
        cx, cy = pt.get('pad_cx', pt['x']), pt.get('pad_cy', pt['y'])
        if edge in ('top', 'bottom'):
            s_coord = cx
            d_perp = (cy - y0) if edge == 'top' else (y1 - cy)
            vx, vy = (0.0, depth * push_gain) if edge == 'top' else (0.0, -depth * push_gain)
        else:
            s_coord = cy
            d_perp = (cx - x0) if edge == 'left' else (x1 - cx)
            vx, vy = (depth * push_gain, 0.0) if edge == 'left' else (-depth * push_gain, 0.0)

        if d_perp < -0.5 or d_perp > inf_mm:
            continue
        d_lat = 0.0 if s0 <= s_coord <= s1 else min(abs(s_coord - s0), abs(s_coord - s1))
        if d_lat > lat_mm:
            continue
        w_perp = 1.0 if d_perp <= 2.2 else max(0.0, 1.0 - (d_perp - 2.2) / max(0.5, inf_mm - 2.2))
        w_lat = 1.0 if d_lat <= 0.0 else max(0.0, 1.0 - d_lat / max(0.2, lat_mm))
        pt['tx'] = round(vx * w_perp * w_lat, 4)
        pt['ty'] = round(vy * w_perp * w_lat, 4)


# ---------------------------------------------------------------------------
# Steerable Field Target Resolution
# ---------------------------------------------------------------------------

def apply_field_targets(cfg, parts, pads_by_ref, outline):
    """Apply steerable field directives (`cfg['field']`) to movable part targets.

    Supports:
      - `ambient`: dilation / shift / dipole evaluated at each part's pad centroid
      - `dipoles`: list of `{"x", "y", "dx", "dy", "radius", "decay", "refs"}`
        evaluated at each part's pad centroid (additive or setting targets for
        movable parts that have zero explicit target offset).
    """
    fcfg = cfg.get('field')
    if not fcfg or not isinstance(fcfg, dict):
        return

    if outline:
        xs = [p[0] for p in outline]
        ys = [p[1] for p in outline]
        bbox = (min(xs), min(ys), max(xs), max(ys))
    else:
        bbox = (0.0, 0.0, 100.0, 100.0)

    amb = compose_ambient(fcfg.get('ambient'), bbox) if fcfg.get('ambient') else None
    dipoles = fcfg.get('dipoles', [])
    overwrite = bool(fcfg.get('overwrite_targets', False))

    for ref, pt in parts.items():
        has_explicit = abs(pt['tx']) > 1e-6 or abs(pt['ty']) > 1e-6
        if has_explicit and not overwrite and not fcfg.get('additive', False):
            continue
        cx, cy = pt.get('pad_cx', pt['x']), pt.get('pad_cy', pt['y'])
        dx, dy = (pt['tx'], pt['ty']) if fcfg.get('additive', False) else (0.0, 0.0)
        if amb is not None:
            qx, qy = amb(cx, cy)
            dx += float(qx - cx)
            dy += float(qy - cy)
        for dp in dipoles:
            if 'refs' in dp and ref not in dp['refs']:
                continue
            sx = float(dp.get('x', cx))
            sy = float(dp.get('y', cy))
            ddx, ddy = dp.get('vector', [dp.get('dx', 0.0), dp.get('dy', 0.0)])
            q_dp = dipole(sx, sy, float(ddx), float(ddy),
                          radius=float(dp.get('radius', 8.0)),
                          decay=float(dp.get('decay', 2.0)))
            qx, qy = q_dp(cx, cy)
            dx += float(qx - cx)
            dy += float(qy - cy)
        pt['tx'] = round(dx, 4)
        pt['ty'] = round(dy, 4)


# ---------------------------------------------------------------------------
# Copper graph builder
# ---------------------------------------------------------------------------

def build_model(cfg, board, rules):
    rect = cfg['active_rect']
    sub_mm = float(cfg.get('subdivide_mm', 0.8))
    movable_refs = set(cfg.get('movable', []))
    targets = cfg.get('targets', {})

    parts = {}
    all_pads = []
    pads_by_ref = {}
    raw_static_cys = []

    for fp in board.GetFootprints():
        ref = fp.GetReference()
        pos = fp.GetPosition()
        fx, fy = round(pos.x / NM, 4), round(pos.y / NM, 4)
        ang = round(fp.GetOrientation().AsDegrees(), 2)
        side = 'B' if fp.IsFlipped() else 'F'
        pads = extract_pads(fp)
        cy = extract_courtyard(fp, pads)

        if pads:
            pad_cx = round(sum(p['x'] for p in pads) / len(pads), 4)
            pad_cy = round(sum(p['y'] for p in pads) / len(pads), 4)
        else:
            pad_cx, pad_cy = fx, fy

        pads_by_ref[ref] = pads
        all_pads.extend(pads)

        if ref in movable_refs:
            t = targets.get(ref, {'dx': 0.0, 'dy': 0.0})
            parts[ref] = {
                'ref': ref, 'x': fx, 'y': fy,
                'pad_cx': pad_cx, 'pad_cy': pad_cy,
                'angle': ang, 'side': side,
                'tx': float(t.get('dx', 0.0)),
                'ty': float(t.get('dy', 0.0)),
                'courtyard': cy,
            }
        else:
            raw_static_cys.append({'ref': ref, 'side': side, 'courtyard': cy})

    base_outline = extract_board_outline(board)
    outline, target_outline, shape_box = build_shape_outlines(base_outline, cfg.get('shape'))
    apply_shape_targets(cfg, parts, base_outline)
    apply_field_targets(cfg, parts, pads_by_ref, base_outline)

    zones = extract_zones(board)

    # Elastic copper envelope: per-movable-part trajectory halo (2.8 mm) unless
    # overridden by explicit `elastic_copper` or `elastic_pad`.
    elastic_cfg = cfg.get('elastic_copper')
    ep_rects = []
    if elastic_cfg:
        er = elastic_cfg.get('rect', rect)
        ep_pad = float(elastic_cfg.get('pad', 0.0))
        ep_rects.append([er[0] - ep_pad, er[1] - ep_pad, er[2] + ep_pad, er[3] + ep_pad])
    if cfg.get('elastic_pad') is not None:
        ep = float(cfg['elastic_pad'])
        ep_rects.append([rect[0] - ep, rect[1] - ep, rect[2] + ep, rect[3] + ep])
    if not ep_rects and parts:
        halo = float(cfg.get('part_halo_mm', 2.8))
        for pt in parts.values():
            cxs = [p[0] for p in pt['courtyard']]
            cys = [p[1] for p in pt['courtyard']]
            tx, ty = pt['tx'], pt['ty']
            x0 = min(min(cxs), min(cxs) + tx) - halo
            y0 = min(min(cys), min(cys) + ty) - halo
            x1 = max(max(cxs), max(cxs) + tx) + halo
            y1 = max(max(cys), max(cys) + ty) + halo
            ep_rects.append([x0, y0, x1, y1])
    if shape_box is not None:
        sh_halo = float((cfg.get('shape') or {}).get('halo_mm', 3.2))
        ep_rects.append([shape_box[0] - sh_halo, shape_box[1] - sh_halo,
                         shape_box[2] + sh_halo, shape_box[3] + sh_halo])

    search_rects = ep_rects if ep_rects else [rect]

    static_courtyards = []
    for sc in raw_static_cys:
        xs = [p[0] for p in sc['courtyard']]
        ys = [p[1] for p in sc['courtyard']]
        if any(max(xs) >= r[0] - 4.0 and min(xs) <= r[2] + 4.0 and
               max(ys) >= r[1] - 4.0 and min(ys) <= r[3] + 4.0 for r in search_rects):
            static_courtyards.append(sc)

    def in_elastic(x, y):
        if not ep_rects:
            return in_rect(x, y, rect, 0.0)
        return any(in_rect(x, y, r, 0.0) for r in ep_rects)

    def seg_in_elastic(ax, ay, bx, by):
        if not ep_rects:
            return seg_hits_rect(ax, ay, bx, by, rect, 0.0)
        return any(seg_hits_rect(ax, ay, bx, by, r, 0.0) for r in ep_rects)

    active_tracks = []
    static_tracks = []
    active_vias = []
    static_vias = []
    active_track_sigs = []
    active_via_sigs = []

    movable_pad_pts = []
    for ref in movable_refs:
        for p in pads_by_ref.get(ref, []):
            movable_pad_pts.append((p['x'], p['y'], p['layers'], p['net'], ref, p['num'], p['poly']))

    def touches_movable_pad(x, y, layer, net):
        for px, py, players, pnet, ref, pnum, ppoly in movable_pad_pts:
            if pnet == net and layer in players:
                if math.hypot(x - px, y - py) <= 0.12 or point_in_poly(x, y, ppoly) or poly_pt_dist(ppoly, x, y) <= 0.06:
                    return (ref, pnum, px, py)
        return None

    for t in board.GetTracks():
        if t.GetClass() == 'PCB_VIA':
            pos = t.GetPosition()
            vx, vy = pos.x / NM, pos.y / NM
            sz = t.GetWidth(F_CU) / NM
            dr = t.GetDrillValue() / NM
            net = t.GetNetname()
            if in_elastic(vx, vy):
                active_vias.append({'x': round(vx, 4), 'y': round(vy, 4),
                                    'size': round(sz, 4), 'drill': round(dr, 4),
                                    'net': net})
                active_via_sigs.append(sig_key(vsig(t)))
            elif any(in_rect(vx, vy, r, 2.5) for r in (ep_rects or [rect])):
                static_vias.append({'x': round(vx, 4), 'y': round(vy, 4),
                                    'size': round(sz, 4), 'drill': round(dr, 4),
                                    'net': net})
        elif t.GetClass() == 'PCB_TRACK':
            l = t.GetLayer()
            if l not in COPPER_LAYERS:
                continue
            s, e = t.GetStart(), t.GetEnd()
            ax, ay = s.x / NM, s.y / NM
            bx, by = e.x / NM, e.y / NM
            w = t.GetWidth() / NM
            net = t.GetNetname()
            hit = (seg_in_elastic(ax, ay, bx, by) or
                   touches_movable_pad(ax, ay, l, net) is not None or
                   touches_movable_pad(bx, by, l, net) is not None)
            if hit:
                active_tracks.append({
                    'ax': round(ax, 4), 'ay': round(ay, 4),
                    'bx': round(bx, 4), 'by': round(by, 4),
                    'w': round(w, 4), 'layer': l, 'net': net,
                })
                active_track_sigs.append(sig_key(tsig(t)))
            elif any(seg_hits_rect(ax, ay, bx, by, r, 2.5) for r in (ep_rects or [rect])):
                static_tracks.append({
                    'ax': round(ax, 4), 'ay': round(ay, 4),
                    'bx': round(bx, 4), 'by': round(by, 4),
                    'w': round(w, 4), 'layer': l, 'net': net,
                })

    # Build exact topological graph:
    # - Via nodes are shared across all layers at (x, y)
    # - Track endpoint nodes are keyed by (x, y, layer) unless a via sits at (x, y)
    # - Each active track is subdivided into an ordered chain [a, sub_1, ..., b]
    #   preserving exact endpoint identity for topological reconstruction in gate.py.
    nodes = []
    segs = []
    chains = []
    SNAP = 0.005  # 5 um endpoint snap on the same layer

    via_nodes = []  # list of (x, y, vr, net, nid)
    layer_nodes = defaultdict(list)  # (net, layer) -> list of (x, y, nid)

    def make_node(x, y, net, kind='free', layer=-1, via_info=None, part_ref=None, pad_num=None, fixed=False):
        nid = len(nodes)
        nd = {
            'id': nid,
            'x': round(x, 4), 'y': round(y, 4),
            'x0': round(x, 4), 'y0': round(y, 4),
            'net': net,
            'kind': kind,
            'layer': layer,
            'fixed': bool(fixed),
        }
        if via_info:
            nd['size'] = via_info['size']
            nd['drill'] = via_info['drill']
        if part_ref:
            nd['part'] = part_ref
            nd['pad'] = pad_num
        nodes.append(nd)
        return nid

    for v in active_vias:
        vx, vy, vnet = v['x'], v['y'], v['net']
        vr = 0.5 * v['size']
        found = None
        for ox, oy, ovr, onet, onid in via_nodes:
            if onet == vnet and math.hypot(ox - vx, oy - vy) <= SNAP:
                found = onid
                break
        if found is None:
            nid = make_node(vx, vy, vnet, kind='via', layer=-1, via_info=v)
            via_nodes.append((vx, vy, vr, vnet, nid))

    def find_or_make_endpoint(x, y, net, layer):
        for vx, vy, vr, vnet, vnid in via_nodes:
            if vnet == net and math.hypot(vx - x, vy - y) <= vr + 0.02:
                return vnid
        for ox, oy, onid in layer_nodes[(net, layer)]:
            if math.hypot(ox - x, oy - y) <= SNAP:
                return onid
        nid = make_node(x, y, net, kind='end', layer=layer)
        layer_nodes[(net, layer)].append((x, y, nid))
        return nid

    # First register all track endpoints so T-junctions on the same layer share a node
    track_end_ids = []
    for tr in active_tracks:
        na = find_or_make_endpoint(tr['ax'], tr['ay'], tr['net'], tr['layer'])
        nb = find_or_make_endpoint(tr['bx'], tr['by'], tr['net'], tr['layer'])
        track_end_ids.append((na, nb))

    # Split active tracks if another same-layer endpoint or via sits on their interior (T-junction)
    for tr_idx, tr in enumerate(active_tracks):
        ax, ay, bx, by = tr['ax'], tr['ay'], tr['bx'], tr['by']
        l, w, net = tr['layer'], tr['w'], tr['net']
        na, nb = track_end_ids[tr_idx]
        if na == nb:
            continue
        L = math.hypot(bx - ax, by - ay)
        # Find any interior T-junction nodes on the same net & layer (or via)
        interior = []
        candidates = [(vx, vy, vr + 0.02, vnid) for vx, vy, vr, vnet, vnid in via_nodes if vnet == net] + [
            (ox, oy, 0.5 * w + 0.015, onid) for ox, oy, onid in layer_nodes[(net, l)]
        ]
        for cx, cy, snap_r, cnid in candidates:
            if cnid in (na, nb):
                continue
            if pt_seg_dist(cx, cy, ax, ay, bx, by) <= snap_r:
                t_proj = ((cx - ax) * (bx - ax) + (cy - ay) * (by - ay)) / max(L * L, 1e-12)
                if 0.01 < t_proj < 0.99:
                    interior.append((t_proj, cnid, cx, cy))
        interior.sort(key=lambda item: item[0])
        waypoints = [(0.0, na, ax, ay)] + interior + [(1.0, nb, bx, by)]

        chain_ids = [na]
        for k in range(len(waypoints) - 1):
            _, n1, x1, y1 = waypoints[k]
            _, n2, x2, y2 = waypoints[k + 1]
            if n1 == n2:
                continue
            seg_len = math.hypot(x2 - x1, y2 - y1)
            n_sub = max(1, int(math.ceil(seg_len / sub_mm)))
            prev_id = n1
            for s_i in range(1, n_sub):
                frac = s_i / n_sub
                px = x1 + (x2 - x1) * frac
                py = y1 + (y2 - y1) * frac
                mid_id = make_node(px, py, net, kind='sub', layer=l)
                segs.append({'a': prev_id, 'b': mid_id, 'w': w, 'layer': l, 'net': net})
                chain_ids.append(mid_id)
                prev_id = mid_id
            segs.append({'a': prev_id, 'b': n2, 'w': w, 'layer': l, 'net': net})
            chain_ids.append(n2)
        chains.append({
            'ids': chain_ids,
            'w': w,
            'layer': l,
            'net': net,
        })

    # Determine incident layers per node
    node_layers = defaultdict(set)
    for s in segs:
        node_layers[s['a']].add(s['layer'])
        node_layers[s['b']].add(s['layer'])
    for n in nodes:
        if n['kind'] == 'via':
            node_layers[n['id']] = set(COPPER_LAYERS)
        elif n.get('layer', -1) >= 0:
            node_layers[n['id']].add(n['layer'])

    # Bind nodes to pads or pin to static copper
    attachments = []
    for n in nodes:
        nid = n['id']
        nx, ny, nnet = n['x'], n['y'], n['net']
        nlayers = node_layers[nid]

        # 1. Check pad binding (point in pad polygon or within 0.06 mm of pad polygon)
        for p in all_pads:
            if p['net'] != nnet:
                continue
            if not (p['drill'] > 0 or nlayers.intersection(p['layers'])):
                continue
            if point_in_poly(nx, ny, p['poly']) or poly_pt_dist(p['poly'], nx, ny) <= 0.06:
                if p['ref'] in movable_refs:
                    n['part'] = p['ref']
                    n['pad'] = p['num']
                else:
                    n['fixed'] = True
                break
        if n.get('part') or n.get('fixed'):
            continue

        # 2. Check static track / static via junction (including T-junctions onto static tracks)
        pinned = False
        for sv in static_vias:
            if sv['net'] == nnet and math.hypot(nx - sv['x'], ny - sv['y']) <= sv['size'] * 0.5 + 0.02:
                pinned = True
                break
        if not pinned:
            for st in static_tracks:
                if st['net'] == nnet and (n['kind'] == 'via' or st['layer'] in nlayers):
                    if pt_seg_dist(nx, ny, st['ax'], st['ay'], st['bx'], st['by']) <= st['w'] * 0.5 + 0.02:
                        pinned = True
                        break
        if not pinned and n['kind'] in ('end', 'via') and not in_elastic(nx, ny):
            pinned = True
        if pinned:
            n['fixed'] = True
            continue

    adj = defaultdict(list)
    for s in segs:
        adj[s['a']].append((s['b'], s['layer']))
        adj[s['b']].append((s['a'], s['layer']))

    # Pin any degree-1 non-via endpoint that is not bound to a movable part so
    # smoothing springs never pull a stub endpoint off its termination.
    for n in nodes:
        if n['kind'] != 'via' and not n.get('part') and not n.get('fixed'):
            if len({nb for nb, _ in adj[n['id']]}) <= 1:
                n['fixed'] = True

    node_by_id = {n['id']: n for n in nodes}
    visited = set()
    for n in nodes:
        nid = n['id']
        if nid in visited:
            continue
        comp = []
        q = [nid]
        visited.add(nid)
        while q:
            cur = q.pop()
            comp.append(cur)
            for nb, _ in adj[cur]:
                if nb not in visited:
                    visited.add(nb)
                    q.append(nb)

        has_anchor = any(
            node_by_id[c].get('part') or node_by_id[c].get('fixed') or
            any(a.get('node') == c for a in attachments)
            for c in comp
        )
        for c in comp:
            cn = node_by_id[c]
            deg = len(adj[c])
            if deg > 1 and has_anchor:
                continue
            cx, cy, cnet = cn['x'], cn['y'], cn['net']
            clayers = {l for _, l in adj[c]}
            if cn['kind'] == 'via':
                clayers = set(COPPER_LAYERS)
            for zi, z in enumerate(zones):
                if z['net'] != cnet or z['layer'] not in clayers:
                    continue
                bb = z['bbox']
                if bb[0] - 0.1 <= cx <= bb[2] + 0.1 and bb[1] - 0.1 <= cy <= bb[3] + 0.1:
                    if point_in_poly(cx, cy, z['poly']):
                        attachments.append({
                            'node': c, 'kind': 'zone',
                            'zone_idx': zi, 'layer': z['layer']
                        })
                        if not has_anchor:
                            cn['fixed'] = True
                        break

    ghost_obstacles = []
    if cfg.get('net_changes'):
        cl = rules['default_clearance'] + 0.12
        for p in all_pads:
            if F_CU in p['layers'] or B_CU in p['layers']:
                ghost_obstacles.append({
                    'kind': 'pad', 'ref': p['ref'], 'num': p['num'], 'net': p['net'],
                    'layers': p['layers'],
                    'poly': inflate_poly(p['poly'], cl),
                })

    active_pads = [
        p for p in all_pads
        if p['ref'] in movable_refs or any(in_rect(p['x'], p['y'], r, 4.0) for r in search_rects)
    ]

    # Attach movable pads sitting inside a same-net zone outline so they never
    # step outside their pour during settling.
    for pi, p in enumerate(active_pads):
        if p['ref'] not in movable_refs or not p['net']:
            continue
        for zi, z in enumerate(zones):
            if z['net'] != p['net']:
                continue
            if not (p['drill'] > 0 or z['layer'] in p['layers']):
                continue
            bb = z['bbox']
            if bb[0] <= p['x'] <= bb[2] and bb[1] <= p['y'] <= bb[3]:
                if point_in_poly(p['x'], p['y'], z['poly']):
                    attachments.append({
                        'kind': 'pad_zone',
                        'ref': p['ref'],
                        'pad': p['num'],
                        'zone_idx': zi,
                    })
                    break

    return {
        'config_name': cfg.get('name', 'unnamed'),
        'board_path': cfg['board'],
        'active_rect': rect,
        'subdivide_mm': sub_mm,
        'rules': rules,
        'outline': outline,
        'target_outline': target_outline,
        'shape': cfg.get('shape', {}),
        'parts': parts,
        'static_courtyards': static_courtyards,
        'pads': active_pads,
        'nodes': nodes,
        'segs': segs,
        'chains': chains,
        'static_tracks': static_tracks,
        'static_vias': static_vias,
        'zones': zones,
        'attachments': attachments,
        'ghost_obstacles': ghost_obstacles,
        'active_track_sigs': active_track_sigs,
        'active_via_sigs': active_via_sigs,
        'net_changes': cfg.get('net_changes', []),
        'field': cfg.get('field', {}),
    }


def extract_model(cfg, verbose=True):
    """Extract and persist `model.json` for a config dict."""
    resolve_elastic(cfg)
    board = load_board(cfg['board'])
    rules = load_rules(cfg['board'], board)
    model = build_model(cfg, board, rules)
    out = data_path(cfg, 'model.json')
    save_json(out, model)
    if verbose:
        print(f"[pcbmorph.model] {model['config_name']}: "
              f"{len(model['parts'])} movable parts, "
              f"{len(model['nodes'])} nodes ({sum(1 for n in model['nodes'] if n['kind']=='via')} vias), "
              f"{len(model['segs'])} sub-segs, "
              f"{len(model['attachments'])} attachments -> {out}")
    return model


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    if not argv:
        print("Usage: python3 -m pcbmorph model CONFIG.json", file=sys.stderr)
        sys.exit(1)
    cfg = load_config(argv[0])
    extract_model(cfg, verbose=True)


if __name__ == '__main__':
    main()
