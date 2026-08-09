#!/usr/bin/env python3
"""jiggle2 shared bits: paths, region, geometry helpers."""
import os, sys, math, json, contextlib

HW = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BOARD_CUR = os.path.join(HW, 'rp2350_driver.kicad_pcb')
BOARD_PRE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'board_prerip_2238aca.kicad_pcb')

# Variants (JIGGLE2_VARIANT env): '' = full board; 'nonew' = the 28 rev-B
# footprints removed everywhere (obstacles, base board, referee baseline).
VARIANT = os.environ.get('JIGGLE2_VARIANT', '')
_sfx = f'_{VARIANT}' if VARIANT else ''
BOARD_OUT = os.path.join(HW, f'rp2350_driver_jiggle2{_sfx}.kicad_pcb')
DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'data{_sfx}')
# The board the work copy is built from and DRC-baselined against.
BOARD_BASE = os.path.join(DATA, 'board_base.kicad_pcb') if VARIANT else BOARD_CUR
os.makedirs(DATA, exist_ok=True)

NM = 1e-6  # pcbnew nm -> mm

# Region containing all ripped copper, with working margin (mm).
REGION = dict(x0=110.0, y0=90.0, x1=142.0, y1=130.0)

def in_region(x, y, m=0.0):
    return REGION['x0']-m <= x <= REGION['x1']+m and REGION['y0']-m <= y <= REGION['y1']+m

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
    return ('T', t.GetLayer(), a, b, t.GetWidth())

def vsig(v):
    p = v.GetPosition()
    return ('V', p.x, p.y, v.GetDrill())

def seg_seg_dist(p1, p2, q1, q2):
    """Min distance between segments p1-p2 and q1-q2 (2D tuples, mm)."""
    def pt_seg(p, a, b):
        ax, ay = a; bx, by = b; px, py = p
        dx, dy = bx-ax, by-ay
        L2 = dx*dx + dy*dy
        if L2 < 1e-18: return math.hypot(px-ax, py-ay), a
        t = max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy) / L2))
        cx, cy = ax + t*dx, ay + t*dy
        return math.hypot(px-cx, py-cy), (cx, cy)
    def orient(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
    # proper intersection => distance 0
    d1, d2 = orient(q1, q2, p1), orient(q1, q2, p2)
    d3, d4 = orient(p1, p2, q1), orient(p1, p2, q2)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return 0.0
    return min(pt_seg(p1, q1, q2)[0], pt_seg(p2, q1, q2)[0],
               pt_seg(q1, p1, p2)[0], pt_seg(q2, p1, p2)[0])

def save_json(name, obj):
    path = os.path.join(DATA, name)
    with open(path, 'w') as f:
        json.dump(obj, f)
    return path

def load_json(name):
    with open(os.path.join(DATA, name)) as f:
        return json.load(f)
