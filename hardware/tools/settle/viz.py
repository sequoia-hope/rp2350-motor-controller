#!/usr/bin/env python3
"""settle: shared drawing of a model state (used by animate.py / explain.py).

A state is {P, D, G, T, chains, press, npress, blocked} in model coordinates
(mm). Everything is drawn with collections so a frame costs ~0.1 s.
"""
import math
import numpy as np
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.patches import Polygon, FancyArrow, Circle

LCOL = {0: '#c0392b', 2: '#2471a3', 4: '#1e8449', 6: '#7d3c98', 8: '#b9770e', 10: '#117a65'}
LNAME = {0: 'F.Cu', 2: 'B.Cu', 4: 'In1.Cu', 6: 'In2.Cu', 8: 'In3.Cu', 10: 'In4.Cu'}
C_FIXED, C_MOV, C_MOVED, C_GHOST, C_STATIC = '#7f8c8d', '#f5b041', '#e67e22', '#c0392b', '#a6a6a6'


def state_from_frame(M, FR, fr):
    """Resolve a frames.json entry into a drawable state."""
    cs = FR['chain_sets'].get(str(fr['cver'])) if fr['cver'] else None
    chains = cs or FR.get('chains0')
    if chains is None:
        raise ValueError('frames.json has no chains0: pass traj.json checkpoint 0 chains (fix_chains0)')
    return dict(P=fr['P'], D=fr['D'], G=fr['G'], T=fr['T'], chains=chains,
                press=fr.get('press', {}), npress=fr.get('npress', []), blocked=fr.get('blocked', []),
                cycle=fr['cycle'], s=fr['s'])


def fix_chains0(FR, TR):
    """Older frames.json files stored chains0 only when no reroute happened;
    checkpoint 0 of traj.json is the same subdivided start topology."""
    if FR.get('chains0') is None and TR:
        FR['chains0'] = TR['checkpoints'][0]['chains']
    return FR


def state_from_checkpoint(M, ck):
    return dict(P=ck['P'], D=ck['D'], G=ck['G'], T=ck['T'], chains=ck['chains'],
                press={}, npress=[], blocked=[], cycle=None, s=ck['s'])


def state0(M):
    return dict(P=[[n['x'], n['y']] for n in M['nodes']], D={}, G=[0.05] * len(M['ghosts']),
                T=[[0, 0] for _ in M['ghosts']], chains=[[e['a'], e['b']] for e in M['edges']],
                press={}, npress=[], blocked=[], cycle=0, s=0.0)


def ghost_polys(M, st, gi):
    gh = M['ghosts'][gi]
    g = st['G'][gi]
    tx, ty = st['T'][gi]
    cx, cy = gh['at']
    pads = [[[cx + tx + (q[0] - cx) * g, cy + ty + (q[1] - cy) * g] for q in p['pts']] for p in gh['pads']]
    crt = [[[cx + tx + (q[0] - cx) * g, cy + ty + (q[1] - cy) * g] for q in poly]
           for polys in gh['courtyards'].values() for poly in polys]
    return pads, crt


def draw_state(ax, M, st, window=None, title=None, show_press=True, show_blocked=True,
               show_courtyards=True, label_ghosts=True, lw_scale=3.0, static_alpha=0.55):
    x0, y0, x1, y1 = window or M['region']
    ax.set_xlim(x0, x1)
    ax.set_ylim(y1, y0)
    ax.set_aspect('equal')
    ax.tick_params(labelsize=7)
    if title:
        ax.set_title(title, fontsize=10)
    ex0, ey0, ex1, ey1 = M['rules'].get('outline', {}).get('rect', M['rules']['board_edge'])
    ax.add_patch(Polygon([[ex0, ey0], [ex1, ey0], [ex1, ey1], [ex0, ey1]], closed=True, fill=False,
                         ec='black', lw=1.2, zorder=1))

    def inwin(x, y, m=1.5):
        return x0 - m <= x <= x1 + m and y0 - m <= y <= y1 + m

    # static copper
    segs, ws = [], []
    for t in M['tracks']:
        if inwin(*t['a']) or inwin(*t['b']):
            segs.append([t['a'], t['b']]); ws.append(max(0.4, t['w'] * lw_scale))
    if segs:
        ax.add_collection(LineCollection(segs, colors=C_STATIC, linewidths=ws, alpha=static_alpha,
                                         capstyle='round', zorder=2))
    for v in M['vias']:
        if inwin(v['x'], v['y']):
            ax.add_patch(Circle((v['x'], v['y']), v['dia'] / 2, color=C_STATIC, alpha=static_alpha, zorder=2))
    # dynamic copper by layer
    P = st['P']
    by_layer = {}
    for e, ids in zip(M['edges'], st['chains']):
        pts = [P[i] for i in ids]
        if not any(inwin(x, y) for x, y in pts):
            continue
        by_layer.setdefault(e['layer'], ([], []))
        by_layer[e['layer']][0].append(pts)
        by_layer[e['layer']][1].append(max(0.4, e['w'] * lw_scale))
    for lay, (polys, lws) in by_layer.items():
        ax.add_collection(LineCollection(polys, colors=LCOL.get(lay, '#333'), linewidths=lws, alpha=0.85,
                                         capstyle='round', joinstyle='round', zorder=4))
    vias = [(P[i][0], P[i][1], n['dia'] / 2) for i, n in enumerate(M['nodes']) if n['kind'] == 'via' and inwin(P[i][0], P[i][1])]
    for x, y, r in vias:
        ax.add_patch(Circle((x, y), r, color='#555', alpha=0.85, zorder=5))
    # pads
    fixed_polys, mov_polys, moved_polys = [], [], []
    D = st['D']
    for p in M['pads']:
        if p.get('ghost') is not None:
            continue
        r = p['ref']
        d = D.get(r, [0, 0])
        if not inwin(p['x'] + d[0], p['y'] + d[1], 2.5):
            continue
        pts = [[q[0] + d[0], q[1] + d[1]] for q in p['pts']]
        mv = M['parts'].get(r, {}).get('movable')
        if not mv:
            fixed_polys.append(pts)
        elif abs(d[0]) > 1e-3 or abs(d[1]) > 1e-3:
            moved_polys.append(pts)
        else:
            mov_polys.append(pts)
    for polys, col in ((fixed_polys, C_FIXED), (mov_polys, C_MOV), (moved_polys, C_MOVED)):
        if polys:
            ax.add_collection(PolyCollection(polys, facecolors=col, edgecolors='none', alpha=0.8, zorder=3))
    if show_courtyards:
        crt = []
        for r, entry in M['courtyards'].items():
            d = D.get(r, [0, 0])
            for lay, polys in entry.items():
                for poly in polys:
                    pts = [[q[0] + d[0], q[1] + d[1]] for q in poly]
                    if any(inwin(x, y) for x, y in pts):
                        crt.append(pts)
        if crt:
            ax.add_collection(PolyCollection(crt, facecolors='none', edgecolors='#666', linewidths=0.4, zorder=3))
    # ghosts
    for gi, gh in enumerate(M['ghosts']):
        pads, crt = ghost_polys(M, st, gi)
        cx, cy = gh['at'][0] + st['T'][gi][0], gh['at'][1] + st['T'][gi][1]
        if not inwin(cx, cy, 3):
            continue
        ax.add_collection(PolyCollection(pads, facecolors=C_GHOST, edgecolors='black', linewidths=0.4, alpha=0.9, zorder=6))
        if crt:
            ax.add_collection(PolyCollection(crt, facecolors='none', edgecolors=C_GHOST, linewidths=0.8,
                                             linestyles='--', zorder=6))
        ax.plot([gh['at'][0]], [gh['at'][1]], marker='+', color=C_GHOST, ms=6, zorder=6)
        if label_ghosts:
            ax.annotate(f"{gh['ref']} {st['G'][gi]:.2f}", (cx, cy), fontsize=6, color=C_GHOST,
                        xytext=(4, 4), textcoords='offset points', zorder=7)
    # pressure on parts, pressured nodes, blocked markers
    if show_press:
        for r, (fx, fy) in st['press'].items():
            p = M['parts'].get(r)
            if not p:
                continue
            d = D.get(r, [0, 0])
            L = math.hypot(fx, fy)
            if L < 1e-4 or not inwin(p['cx'] + d[0], p['cy'] + d[1]):
                continue
            sc = min(1.2, 6 * L)                          # 0.05 mm demand -> 0.3 mm arrow
            ax.add_patch(FancyArrow(p['cx'] + d[0], p['cy'] + d[1], fx / L * sc, fy / L * sc, width=0.04,
                                    head_width=0.22, length_includes_head=True, color='#1a5276', alpha=0.9, zorder=8))
        if st['npress']:
            xs = [P[i][0] for i in st['npress'] if inwin(P[i][0], P[i][1])]
            ys = [P[i][1] for i in st['npress'] if inwin(P[i][0], P[i][1])]
            ax.scatter(xs, ys, s=6, color='#1a5276', alpha=0.8, zorder=8)
    if show_blocked:
        for name, kind, pos, m in st['blocked']:
            if pos and inwin(*pos):
                ax.plot([pos[0]], [pos[1]], marker='x', color='black', ms=5, mew=1.2, zorder=9)
    # part displacement arrows
    for r, d in D.items():
        if math.hypot(*d) < 0.05:
            continue
        p = M['parts'][r]
        if inwin(p['cx'], p['cy']):
            ax.add_patch(FancyArrow(p['cx'], p['cy'], d[0], d[1], width=0.03, head_width=0.2,
                                    length_includes_head=True, color='#0b5345', alpha=0.9, zorder=7))


def legend_handles():
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D
    h = [Patch(fc=C_FIXED, label='fixed part pad'), Patch(fc=C_MOV, label='movable part pad'),
         Patch(fc=C_MOVED, label='part that moved'), Patch(fc=C_GHOST, label='ghost (inflating part)'),
         Line2D([0], [0], color=C_STATIC, lw=3, label='static copper'),
         Line2D([0], [0], color=LCOL[0], lw=2, label='dynamic F.Cu'), Line2D([0], [0], color=LCOL[2], lw=2, label='dynamic B.Cu'),
         Line2D([0], [0], color=LCOL[6], lw=2, label='dynamic In2.Cu'), Line2D([0], [0], color=LCOL[8], lw=2, label='dynamic In3.Cu'),
         Line2D([0], [0], color='#1a5276', marker='>', lw=2, label='pressure demand on a part'),
         Line2D([0], [0], color='#1a5276', marker='o', lw=0, ms=4, label='pressured copper node'),
         Line2D([0], [0], color='black', marker='x', lw=0, ms=6, label='clamp (blocker)'),
         Line2D([0], [0], color='#0b5345', marker='>', lw=2, label='net displacement so far')]
    return h
