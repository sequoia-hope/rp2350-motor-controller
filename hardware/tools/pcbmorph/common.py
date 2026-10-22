#!/usr/bin/env python3
"""pcbmorph.common: shared constants, config I/O, pcbnew wrappers, and 2D geometry primitives."""
import contextlib
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(os.path.dirname(HERE))
REPO_ROOT = os.path.dirname(HW)
NM = 1_000_000.0  # pcbnew nanometres per millimetre (1 mm = 1e6 nm)

# KiCad 9 copper layer IDs on a 6-layer stack (F.Cu, In1..In4.Cu, B.Cu)
COPPER_LAYERS = [0, 4, 6, 8, 10, 2]
F_CU, B_CU = 0, 2
LAYER_NAMES = {
    0: 'F.Cu',
    4: 'In1.Cu',
    6: 'In2.Cu',
    8: 'In3.Cu',
    10: 'In4.Cu',
    2: 'B.Cu',
}


def _resolve_repo_path(p, cfg_dir):
    if not p or os.path.isabs(p):
        return p
    # Check relative to repo root / cwd first, then relative to config file dir
    cand_cwd = os.path.abspath(p)
    if os.path.exists(cand_cwd) or p.startswith('hardware/'):
        return cand_cwd
    return os.path.normpath(os.path.join(cfg_dir, p))


def load_config(path=None):
    """Load and normalize a run configuration JSON file."""
    if isinstance(path, dict):
        return path
    path = path or (
        sys.argv[1] if len(sys.argv) > 1 and sys.argv[1].endswith('.json') else None
    )
    if not path:
        sys.exit('usage: python3 -m pcbmorph <command> CONFIG.json [options]')
    with open(path) as f:
        cfg = json.load(f)
    cfg['_path'] = os.path.abspath(path)
    cfg.setdefault('name', os.path.splitext(os.path.basename(path))[0])
    cfg_dir = os.path.dirname(cfg['_path'])
    cfg['board'] = _resolve_repo_path(cfg['board'], cfg_dir)
    raw_data = cfg.get('data_dir') or cfg.get('data') or os.path.join(HERE, 'data', cfg['name'])
    cfg['data'] = _resolve_repo_path(raw_data, cfg_dir)
    if 'out_board' in cfg:
        cfg['out_board'] = _resolve_repo_path(cfg['out_board'], cfg_dir)
    os.makedirs(cfg['data'], exist_ok=True)
    return cfg


def data_path(cfg, name):
    return os.path.join(cfg['data'], name)


def save_json(path_or_cfg, obj_or_name, maybe_obj=None):
    if maybe_obj is not None:
        p = data_path(path_or_cfg, obj_or_name)
        obj = maybe_obj
    else:
        p = path_or_cfg
        obj = obj_or_name
    os.makedirs(os.path.dirname(os.path.abspath(p)), exist_ok=True)
    with open(p, 'w') as f:
        json.dump(obj, f, indent=2)
    return p


def load_json(path_or_cfg, name=None):
    p = data_path(path_or_cfg, name) if name is not None else path_or_cfg
    with open(p) as f:
        return json.load(f)


@contextlib.contextmanager
def quiet_stderr():
    """Silence wx/pcbnew C-level chatter on file descriptor 2."""
    fd = os.dup(2)
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, 2)
    try:
        yield
    finally:
        os.dup2(fd, 2)
        os.close(fd)
        os.close(devnull)


def load_board(path):
    import pcbnew
    with quiet_stderr():
        return pcbnew.LoadBoard(path)


def nm(x):
    return int(round(x * NM))


def tsig(t):
    s, e = t.GetStart(), t.GetEnd()
    a, b = (s.x, s.y), (e.x, e.y)
    if b < a:
        a, b = b, a
    return ('T', int(t.GetLayer()), a, b, t.GetWidth())


def vsig(v):
    p = v.GetPosition()
    return ('V', p.x, p.y, v.GetDrillValue() if hasattr(v, 'GetDrillValue') else v.GetDrill())


def sig_key(sig):
    """JSON-serializable signature string for a copper track or via."""
    return '|'.join(
        str(x)
        for x in (sig[0],)
        + tuple(
            c
            for part in sig[1:]
            for c in (part if isinstance(part, tuple) else (part,))
        )
    )


def pt_seg_dist(px, py, ax, ay, bx, by):
    """Euclidean distance from point (px, py) to segment (ax, ay)-(bx, by)."""
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-18:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def seg_seg_dist(ax, ay, bx, by, cx=None, cy=None, dx=None, dy=None):
    """Minimum distance between 2D segments in mm.

    Supports both 8-float `(ax, ay, bx, by, cx, cy, dx, dy)` and
    4-tuple `(p1, p2, q1, q2)` calling conventions.
    """
    if cx is None:
        p1, p2, q1, q2 = ax, ay, bx, by
        ax, ay = p1[0], p1[1]
        bx, by = p2[0], p2[1]
        cx, cy = q1[0], q1[1]
        dx, dy = q2[0], q2[1]

    def orient(x1, y1, x2, y2, x3, y3):
        return (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)

    d1 = orient(cx, cy, dx, dy, ax, ay)
    d2 = orient(cx, cy, dx, dy, bx, by)
    d3 = orient(ax, ay, bx, by, cx, cy)
    d4 = orient(ax, ay, bx, by, dx, dy)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return 0.0
    return min(
        pt_seg_dist(ax, ay, cx, cy, dx, dy),
        pt_seg_dist(bx, by, cx, cy, dx, dy),
        pt_seg_dist(cx, cy, ax, ay, bx, by),
        pt_seg_dist(dx, dy, ax, ay, bx, by),
    )


def point_in_poly(x, y, pts):
    """Ray-casting point-in-polygon test."""
    inside = False
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi:
            inside = not inside
        j = i
    return inside


def poly_seg_dist(pts, x1, y1, x2=None, y2=None):
    if x2 is None:
        p1, p2 = x1, y1
        x1, y1 = p1[0], p1[1]
        x2, y2 = p2[0], p2[1]
    n = len(pts)
    d = min(
        seg_seg_dist(x1, y1, x2, y2, pts[i][0], pts[i][1], pts[(i + 1) % n][0], pts[(i + 1) % n][1])
        for i in range(n)
    )
    if d > 0 and point_in_poly((x1 + x2) * 0.5, (y1 + y2) * 0.5, pts):
        return 0.0
    return d


def poly_pt_dist(pts, x, y=None):
    if y is None:
        x, y = x[0], x[1]
    n = len(pts)
    d = min(
        pt_seg_dist(x, y, pts[i][0], pts[i][1], pts[(i + 1) % n][0], pts[(i + 1) % n][1])
        for i in range(n)
    )
    if d > 0 and point_in_poly(x, y, pts):
        return 0.0
    return d


def poly_poly_dist(A, B):
    na, nb = len(A), len(B)
    d = min(
        seg_seg_dist(A[i][0], A[i][1], A[(i + 1) % na][0], A[(i + 1) % na][1],
                     B[j][0], B[j][1], B[(j + 1) % nb][0], B[(j + 1) % nb][1])
        for i in range(na)
        for j in range(nb)
    )
    if d > 0 and (point_in_poly(A[0][0], A[0][1], B) or point_in_poly(B[0][0], B[0][1], A)):
        return 0.0
    return d


def inside_dist(x, y, pts):
    """Signed distance from (x, y) to polygon boundary (positive inside)."""
    n = len(pts)
    d = min(
        pt_seg_dist(x, y, pts[i][0], pts[i][1], pts[(i + 1) % n][0], pts[(i + 1) % n][1])
        for i in range(n)
    )
    return d if point_in_poly(x, y, pts) else -d


def inside_seg_dist(x1, y1, x2, y2, pts):
    """Signed distance from segment (x1,y1)-(x2,y2) to polygon boundary (positive when fully inside)."""
    mx, my = 0.5 * (x1 + x2), 0.5 * (y1 + y2)
    if not (point_in_poly(x1, y1, pts) and point_in_poly(x2, y2, pts) and point_in_poly(mx, my, pts)):
        return min(inside_dist(x1, y1, pts), inside_dist(x2, y2, pts), inside_dist(mx, my, pts))
    n = len(pts)
    return min(
        seg_seg_dist(x1, y1, x2, y2, pts[i][0], pts[i][1], pts[(i + 1) % n][0], pts[(i + 1) % n][1])
        for i in range(n)
    )


def inside_poly_dist(poly, pts):
    """Signed distance from polygon `poly` to enclosing boundary `pts` (positive when fully inside)."""
    for x, y in poly:
        if not point_in_poly(x, y, pts):
            return min(inside_dist(qx, qy, pts) for qx, qy in poly)
    na, nb = len(poly), len(pts)
    return min(
        seg_seg_dist(poly[i][0], poly[i][1], poly[(i + 1) % na][0], poly[(i + 1) % na][1],
                     pts[j][0], pts[j][1], pts[(j + 1) % nb][0], pts[(j + 1) % nb][1])
        for i in range(na)
        for j in range(nb)
    )


def inflate_poly(pts, r):
    """Outward miter-offset of a polygon by distance r."""
    n = len(pts)
    if n < 3:
        return [list(q) for q in pts]
    cx = sum(q[0] for q in pts) / n
    cy_ = sum(q[1] for q in pts) / n

    def nrm(a, b):
        ex, ey = b[0] - a[0], b[1] - a[1]
        L = math.hypot(ex, ey) or 1e-12
        nx, ny = ey / L, -ex / L
        mx, my = (a[0] + b[0]) / 2 - cx, (a[1] + b[1]) / 2 - cy_
        return (nx, ny) if nx * mx + ny * my > 0 else (-nx, -ny)

    out = []
    for i in range(n):
        p0, p1, p2 = pts[i - 1], pts[i], pts[(i + 1) % n]
        n1, n2 = nrm(p0, p1), nrm(p1, p2)
        mx, my = n1[0] + n2[0], n1[1] + n2[1]
        L = math.hypot(mx, my)
        if L < 1e-9:
            out.append([round(p1[0] + n1[0] * r, 4), round(p1[1] + n1[1] * r, 4)])
        else:
            s = min(2 * r, 2 * r / L)
            out.append([round(p1[0] + mx / L * s, 4), round(p1[1] + my / L * s, 4)])
    return out


def in_rect(x, y, r, m=0.0):
    return r[0] - m <= x <= r[2] + m and r[1] - m <= y <= r[3] + m


def seg_hits_rect(ax, ay, bx, by=None, r=None, m=0.0):
    """Liang-Barsky segment-rectangle intersection test (inflated by m)."""
    if r is None:
        # 4-arg form: seg_hits_rect(a, b, r, m)
        a, b, r = ax, ay, bx
        m = 0.0 if by is None else by
        ax, ay = a[0], a[1]
        bx, by = b[0], b[1]
    if in_rect(ax, ay, r, m) or in_rect(bx, by, r, m):
        return True
    x0, y0, x1, y1 = r[0] - m, r[1] - m, r[2] + m, r[3] + m
    dx, dy = bx - ax, by - ay
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, ax - x0), (dx, x1 - ax), (-dy, ay - y0), (dy, y1 - ay)):
        if abs(p) < 1e-12:
            if q < 0:
                return False
            continue
        t = q / p
        if p < 0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 > t1:
            return False
    return True


def rdp(pts, eps=0.002):
    """Ramer-Douglas-Peucker polyline simplification."""
    if len(pts) < 3:
        return pts
    ax, ay = pts[0]
    bx, by = pts[-1]
    L = math.hypot(bx - ax, by - ay)
    dmax, imax = 0.0, 0
    for i in range(1, len(pts) - 1):
        px, py = pts[i]
        d = (
            math.hypot(px - ax, py - ay)
            if L < 1e-9
            else abs((bx - ax) * (ay - py) - (ax - px) * (by - ay)) / L
        )
        if d > dmax:
            dmax, imax = d, i
    if dmax > eps:
        return rdp(pts[: imax + 1], eps)[:-1] + rdp(pts[imax:], eps)
    return [pts[0], pts[-1]]


def resolve_elastic(cfg, net=None, netclass=None):
    """Normalize elastic config envelope or return per-net spring parameters."""
    if net is None and netclass is None:
        if 'elastic_pad' in cfg and 'elastic_copper' not in cfg:
            cfg['elastic_copper'] = {
                'rect': cfg['active_rect'],
                'pad': float(cfg['elastic_pad']),
            }
        return cfg
    el = cfg.get('elastic', {})
    base = dict(k=0.5, allow=0.2, cap=0.5, allow_mm=0.5, cap_mm=2.0)
    base.update(el.get('default', {}))
    cls = el.get('classes', {}).get(netclass)
    if cls:
        base.update(cls)
    for pat, spec in el.get('nets', {}).items():
        if re.search(pat, net or ''):
            base.update(spec)
            break
    return base
