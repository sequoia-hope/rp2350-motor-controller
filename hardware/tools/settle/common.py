#!/usr/bin/env python3
"""settle: shared bits — config, paths, geometry helpers.

A settle run is described by one JSON config (see README.md). Every script
takes the config path as its first argument and reads/writes JSON artefacts
in the run's data directory (config['data'], default data/<name>/ beside
this file).
"""
import os, sys, math, json, re, contextlib

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(os.path.dirname(HERE))
NM = 1e-6  # pcbnew nm -> mm

# KiCad 9 copper layer ids on a 6-layer board (F, In1..In4, B)
COPPER_LAYERS = [0, 4, 6, 8, 10, 2]
F_CU, B_CU = 0, 2
LAYER_NAMES = {0: 'F.Cu', 4: 'In1.Cu', 6: 'In2.Cu', 8: 'In3.Cu', 10: 'In4.Cu', 2: 'B.Cu'}


def load_config(path=None):
    path = path or (sys.argv[1] if len(sys.argv) > 1 and sys.argv[1].endswith('.json')
                    else None)
    if not path:
        sys.exit('usage: <script> CONFIG.json [options]')
    with open(path) as f:
        cfg = json.load(f)
    cfg['_path'] = os.path.abspath(path)
    cfg.setdefault('name', os.path.splitext(os.path.basename(path))[0])
    root = os.path.dirname(cfg['_path'])
    cfg['board'] = os.path.normpath(os.path.join(root, cfg['board'])) \
        if not os.path.isabs(cfg['board']) else cfg['board']
    cfg['data'] = os.path.normpath(os.path.join(root, cfg.get('data', f"data/{cfg['name']}"))) \
        if not os.path.isabs(cfg.get('data', '')) else cfg['data']
    os.makedirs(cfg['data'], exist_ok=True)
    return cfg


def data_path(cfg, name):
    return os.path.join(cfg['data'], name)


def save_json(cfg, name, obj):
    p = data_path(cfg, name)
    with open(p, 'w') as f:
        json.dump(obj, f)
    return p


def load_json(cfg, name):
    with open(data_path(cfg, name)) as f:
        return json.load(f)


@contextlib.contextmanager
def quiet_stderr():
    """Silence wx/pcbnew C-level chatter on fd 2."""
    fd = os.dup(2)
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, 2)
    try:
        yield
    finally:
        os.dup2(fd, 2)
        os.close(fd); os.close(devnull)


def load_board(path):
    import pcbnew
    with quiet_stderr():
        return pcbnew.LoadBoard(path)


def tsig(t):
    s, e = t.GetStart(), t.GetEnd()
    a, b = (s.x, s.y), (e.x, e.y)
    if b < a: a, b = b, a
    return ('T', int(t.GetLayer()), a, b, t.GetWidth())


def vsig(v):
    p = v.GetPosition()
    return ('V', p.x, p.y, v.GetDrill())


def sig_key(sig):
    """JSON-able form of a signature."""
    return '|'.join(str(x) for x in (sig[0],) + tuple(
        c for part in sig[1:] for c in (part if isinstance(part, tuple) else (part,))))


def seg_seg_dist(p1, p2, q1, q2):
    """Min distance between segments p1-p2 and q1-q2 (2D tuples, mm)."""
    def pt_seg(p, a, b):
        ax, ay = a; bx, by = b; px, py = p
        dx, dy = bx-ax, by-ay
        L2 = dx*dx + dy*dy
        if L2 < 1e-18: return math.hypot(px-ax, py-ay)
        t = max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy) / L2))
        return math.hypot(px-ax-t*dx, py-ay-t*dy)
    def orient(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
    d1, d2 = orient(q1, q2, p1), orient(q1, q2, p2)
    d3, d4 = orient(p1, p2, q1), orient(p1, p2, q2)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return 0.0
    return min(pt_seg(p1, q1, q2), pt_seg(p2, q1, q2),
               pt_seg(q1, p1, p2), pt_seg(q2, p1, p2))


def in_rect(x, y, r, m=0.0):
    return r[0]-m <= x <= r[2]+m and r[1]-m <= y <= r[3]+m


def match_any(name, patterns):
    return any(re.search(p, name) for p in patterns)


def resolve_elastic(cfg, net, netclass):
    """Per-net spring parameters: k (stiffness), allow (free growth
    fraction), cap (hard growth fraction). Resolution order: nets patterns
    (first match) > classes > default."""
    el = cfg.get('elastic', {})
    base = dict(k=0.5, allow=0.2, cap=0.5, allow_mm=0.5, cap_mm=2.0)
    base.update(el.get('default', {}))
    cls = el.get('classes', {}).get(netclass)
    if cls:
        base.update(cls)
    for pat, spec in el.get('nets', {}).items():
        if re.search(pat, net):
            base.update(spec)
            break
    return base
