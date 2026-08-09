#!/usr/bin/env python3
"""Exact-geometry clearance analysis + relaxation using pcbnew courtyard polys."""
import pcbnew, math, json
from collections import defaultdict

SCRATCH = '/tmp/claude-1000/-home-sequoia-pcb-rp2350-motor-controller/e49668e9-e4e8-48c1-bc66-ca2c73581db5/scratchpad'
board = pcbnew.LoadBoard('/home/sequoia/pcb/rp2350-motor-controller/hardware/rp2350_driver.kicad_pcb')
NM = 1e-6  # nm -> mm

def convex_hull(pts):
    pts = sorted(set(pts))
    if len(pts) <= 2: return pts
    def cross(o, a, b): return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0: lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0: upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]

parts = {}
for fp in board.GetFootprints():
    ref = fp.GetReference()
    side = 'F' if fp.GetLayer() == pcbnew.F_Cu else 'B'
    lay = pcbnew.F_CrtYd if side == 'F' else pcbnew.B_CrtYd
    pts = []
    try:
        cy = fp.GetCourtyard(lay)
        for i in range(cy.OutlineCount()):
            ol = cy.Outline(i)
            for k in range(ol.PointCount()):
                p = ol.CPoint(k)
                pts.append((p.x * NM, p.y * NM))
    except Exception:
        pass
    if not pts:
        for pad in fp.Pads():
            bb = pad.GetBoundingBox()
            pts += [(bb.GetLeft()*NM - .2, bb.GetTop()*NM - .2),
                    (bb.GetRight()*NM + .2, bb.GetTop()*NM - .2),
                    (bb.GetRight()*NM + .2, bb.GetBottom()*NM + .2),
                    (bb.GetLeft()*NM - .2, bb.GetBottom()*NM + .2)]
    if not pts: continue
    hull = convex_hull([(round(x, 4), round(y, 4)) for x, y in pts])
    pos = fp.GetPosition()
    parts[ref] = dict(x=pos.x*NM, y=pos.y*NM, side=side,
                      hull=[(hx - pos.x*NM, hy - pos.y*NM) for hx, hy in hull])

bb = board.GetBoardEdgesBoundingBox()
BX0, BY0, BX1, BY1 = bb.GetLeft()*NM, bb.GetTop()*NM, bb.GetRight()*NM, bb.GetBottom()*NM
print(f'board {BX1-BX0:.1f} x {BY1-BY0:.1f} mm, parts {len(parts)}')

# phantoms: 4x SC70-6 switch (courtyard 2.4x2.6) + 4x 0402 (1.5x0.9)
def rect_hull(w, h):
    return [(-w/2,-h/2),(w/2,-h/2),(w/2,h/2),(-w/2,h/2)]
phantoms = {
    'SWT_A': dict(x=114.8, y=110.6, side='B', hull=rect_hull(2.4,2.6)),
    'SWT_B': dict(x=120.5, y=127.0, side='B', hull=rect_hull(2.4,2.6)),
    'SWT_C': dict(x=123.0, y=106.5, side='B', hull=rect_hull(2.4,2.6)),
    'SWT_D': dict(x=114.4, y=105.0, side='B', hull=rect_hull(2.4,2.6)),
    'RPU_A': dict(x=116.2, y=109.3, side='B', hull=rect_hull(1.5,0.9)),
    'RPU_B': dict(x=119.3, y=126.8, side='B', hull=rect_hull(1.5,0.9)),
    'RPU_C': dict(x=121.5, y=104.8, side='B', hull=rect_hull(1.5,0.9)),
    'RPU_D': dict(x=113.6, y=103.3, side='B', hull=rect_hull(1.5,0.9)),
}
parts.update(phantoms)

import re
PINNED = {r for r in parts if re.match(r'^(J\d+|H\d+)$', r)}
PINNED |= {'AH1','AL1','BH1','BL1','CH1','CL1','CH2','CL2',
           'R20','R23','R39','R40','R55','R56','C39','C42','C80'} & set(parts)

def world(p, x=None, y=None):
    x = p['x'] if x is None else x; y = p['y'] if y is None else y
    return [(x+dx, y+dy) for dx, dy in p['hull']]

def poly_gap(A, B):
    n_a, n_b = len(A), len(B)
    max_axis_gap = -1e9
    for P, Q, n in ((A, B, n_a), (B, A, n_b)):
        for i in range(n):
            ex = P[(i+1)%n][0]-P[i][0]; ey = P[(i+1)%n][1]-P[i][1]
            L = math.hypot(ex, ey)
            if L < 1e-9: continue
            nx, ny = -ey/L, ex/L
            pa = [c[0]*nx+c[1]*ny for c in A]; pb = [c[0]*nx+c[1]*ny for c in B]
            max_axis_gap = max(max_axis_gap, max(min(pb)-max(pa), min(pa)-max(pb)))
    if max_axis_gap > 0:
        d = 1e9
        for P, Q, n in ((A, B, len(B)), (B, A, len(A))):
            for c in P:
                for i in range(n):
                    x1, y1 = Q[i]; x2, y2 = Q[(i+1)%n]
                    dx, dy = x2-x1, y2-y1
                    t = max(0, min(1, ((c[0]-x1)*dx + (c[1]-y1)*dy)/(dx*dx+dy*dy+1e-12)))
                    d = min(d, math.hypot(c[0]-x1-t*dx, c[1]-y1-t*dy))
        return d
    return max_axis_gap

refs = list(parts)
NEIGHBOR_R = 6.0
def all_pairs_gaps(positions, cutoff=1.0):
    grid = defaultdict(list)
    for r in refs:
        grid[(int(positions[r][0]//NEIGHBOR_R), int(positions[r][1]//NEIGHBOR_R))].append(r)
    out, seen = [], set()
    for (gx, gy), rlist in grid.items():
        cand = []
        for dx in (-1,0,1):
            for dy in (-1,0,1): cand += grid.get((gx+dx, gy+dy), [])
        for a in rlist:
            for b in cand:
                if a >= b or (a, b) in seen: continue
                seen.add((a, b))
                if parts[a]['side'] != parts[b]['side']: continue
                pa, pb = positions[a], positions[b]
                if abs(pa[0]-pb[0]) > 10 or abs(pa[1]-pb[1]) > 10: continue
                g = poly_gap(world(parts[a], *pa), world(parts[b], *pb))
                if g < cutoff: out.append((g, a, b))
    return out

pos0 = {r: (parts[r]['x'], parts[r]['y']) for r in refs}
before = sorted(g for g in all_pairs_gaps(pos0))
real_before = [(g,a,b) for g,a,b in before if a not in phantoms and b not in phantoms]
print('\n=== BEFORE (exact courtyards, real parts) ===')
print('overlapping (<-0.01):', sum(1 for g,_,_ in real_before if g < -0.01))
print('touching/near (<0.05):', sum(1 for g,_,_ in real_before if g < 0.05))
print('gap <0.1:', sum(1 for g,_,_ in real_before if g < 0.1))
print('gap <0.25:', sum(1 for g,_,_ in real_before if g < 0.25))
print('tightest 30:')
for g, a, b in real_before[:30]:
    print(f'  {g:7.3f}  {a:7s} {b:7s} @({(pos0[a][0]+pos0[b][0])/2:6.1f},{(pos0[a][1]+pos0[b][1])/2:6.1f}) {parts[a]["side"]}')
json.dump(real_before[:40], open(f'{SCRATCH}/tight_pairs.json', 'w'))

# ---- relaxation ----
TARGET, SPRING, PUSH, MAXSTEP, ITERS = 0.25, 0.012, 0.45, 0.12, 400
pos = {r: [parts[r]['x'], parts[r]['y']] for r in refs}
for it in range(ITERS):
    forces = {r: [0.0, 0.0] for r in refs}
    active = 0
    for g, a, b in all_pairs_gaps({r: tuple(pos[r]) for r in refs}, cutoff=TARGET):
        deficit = TARGET - g
        dx = pos[b][0]-pos[a][0]; dy = pos[b][1]-pos[a][1]
        L = math.hypot(dx, dy) or 1e-6
        ux, uy = dx/L, dy/L
        f = PUSH * deficit / 2
        wa = 0 if a in PINNED else (2.0 if a in phantoms else 1.0)
        wb = 0 if b in PINNED else (2.0 if b in phantoms else 1.0)
        if wa + wb == 0: continue
        active += 1
        forces[a][0] -= ux*f*2*wa/(wa+wb); forces[a][1] -= uy*f*2*wa/(wa+wb)
        forces[b][0] += ux*f*2*wb/(wa+wb); forces[b][1] += uy*f*2*wb/(wa+wb)
    for r in refs:
        if r in PINNED: continue
        if r not in phantoms:
            forces[r][0] += SPRING*(parts[r]['x']-pos[r][0])
            forces[r][1] += SPRING*(parts[r]['y']-pos[r][1])
        fx, fy = forces[r]
        m = math.hypot(fx, fy)
        if m > MAXSTEP: fx, fy = fx/m*MAXSTEP, fy/m*MAXSTEP
        pos[r][0] = min(max(pos[r][0]+fx, BX0+2), BX1-2)
        pos[r][1] = min(max(pos[r][1]+fy, BY0+2), BY1-2)
    if it % 100 == 0: print(f'iter {it}: active {active}')

after = sorted(all_pairs_gaps({r: tuple(pos[r]) for r in refs}, cutoff=TARGET))
unpinned_bad = [(g,a,b) for g,a,b in after if not (a in PINNED and b in PINNED)]
print('\n=== AFTER ===')
print('remaining pairs < target 0.25 (any):', len(after))
print('  of which not pinned-pinned:', len(unpinned_bad))
print('  overlapping (<-0.01):', sum(1 for g,_,_ in after if g < -0.01),
      'not-pinned-pinned:', sum(1 for g,_,_ in unpinned_bad if g < -0.01))
for g, a, b in unpinned_bad[:15]:
    print(f'  {g:7.3f}  {a:7s} {b:7s}')

moves = sorted(((math.hypot(pos[r][0]-parts[r]['x'], pos[r][1]-parts[r]['y']), r)
                for r in refs if r not in phantoms), reverse=True)
moved = [(d, r) for d, r in moves if d > 0.05]
print(f'\nmoved >0.05mm: {len(moved)} / {len(refs)-len(phantoms)}')
for d, r in moved[:15]:
    print(f'  {r:7s} {d:5.2f}mm ({parts[r]["x"]:.1f},{parts[r]["y"]:.1f})->({pos[r][0]:.1f},{pos[r][1]:.1f})')
if moved:
    print(f'mean {sum(d for d,_ in moved)/len(moved):.3f} max {moved[0][0]:.3f}')
print('\nphantoms:')
for ph in phantoms: print(f'  {ph}: ({pos[ph][0]:.2f},{pos[ph][1]:.2f})')

# affected tracks
def point_in_poly(pt, poly):
    n = len(poly); inside = False
    j = n-1
    for i in range(n):
        xi, yi = poly[i]; xj, yj = poly[j]
        if (yi > pt[1]) != (yj > pt[1]) and pt[0] < (xj-xi)*(pt[1]-yi)/(yj-yi+1e-12)+xi:
            inside = not inside
        j = i
    return inside
moved_set = {r for d, r in moved}
polys0 = {r: world(parts[r]) for r in moved_set}
bboxes = {r: (min(c[0] for c in p), min(c[1] for c in p), max(c[0] for c in p), max(c[1] for c in p))
          for r, p in polys0.items()}
n_aff = 0; by_layer = defaultdict(int); total = 0
for t in board.GetTracks():
    if t.GetClass() != 'PCB_TRACK': continue
    total += 1
    s_ = t.GetStart(); e = t.GetEnd()
    p1 = (s_.x*NM, s_.y*NM); p2 = (e.x*NM, e.y*NM)
    for r in moved_set:
        b0, b1, b2, b3 = bboxes[r]
        hit = False
        for pt in (p1, p2):
            if b0-.05 <= pt[0] <= b2+.05 and b1-.05 <= pt[1] <= b3+.05 and point_in_poly(pt, polys0[r]):
                hit = True; break
        if hit:
            n_aff += 1; by_layer[board.GetLayerName(t.GetLayer())] += 1
            break
print(f'\ntrack segments with endpoint under a moved part: {n_aff} / {total}')
print('by layer:', dict(by_layer))

json.dump({
    'board': [BX0, BY0, BX1, BY1],
    'target': TARGET,
    'parts': {r: {'before': [parts[r]['x'], parts[r]['y']],
                  'after': [round(pos[r][0],3), round(pos[r][1],3)],
                  'quad_before': [[round(parts[r]['x']+dx,2), round(parts[r]['y']+dy,2)] for dx,dy in parts[r]['hull']],
                  'quad_after': [[round(pos[r][0]+dx,2), round(pos[r][1]+dy,2)] for dx,dy in parts[r]['hull']],
                  'side': parts[r]['side'],
                  'pinned': r in PINNED,
                  'phantom': r in phantoms} for r in refs},
}, open(f'{SCRATCH}/jiggle_result.json', 'w'))
print('wrote jiggle_result.json')
