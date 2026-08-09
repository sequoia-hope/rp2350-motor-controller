#!/usr/bin/env python3
"""Generate before/after jiggle preview SVGs (per side) + encoder-corner zoom."""
import json, math

d = json.load(open('/tmp/claude-1000/-home-sequoia-pcb-rp2350-motor-controller/e49668e9-e4e8-48c1-bc66-ca2c73581db5/scratchpad/jiggle_result.json'))
BX0, BY0, BX1, BY1 = d['board']
parts = d['parts']

SC = 12  # px per mm
PAD = 6

def svg_side(side, x0, y0, x1, y1, out, label_movers=0.8, label_all_phantoms=True, mirror=False):
    W = (x1-x0)*SC + 2*PAD; H = (y1-y0)*SC + 2*PAD
    def X(x):
        if mirror: return PAD + (x1-x)*SC
        return PAD + (x-x0)*SC
    def Y(y): return PAD + (y-y0)*SC
    parts_here = {r: p for r, p in parts.items() if p['side'] == side}
    o = []
    o.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W:.0f} {H:.0f}" width="{W:.0f}" height="{H:.0f}" font-family="ui-monospace,monospace">')
    o.append(f'<rect x="0" y="0" width="{W:.0f}" height="{H:.0f}" fill="#20242c" rx="8"/>')
    # board outline (clip to window)
    bx0, by0 = max(BX0,x0), max(BY0,y0); bx1, by1 = min(BX1,x1), min(BY1,y1)
    xs = sorted([X(bx0), X(bx1)]);
    o.append(f'<rect x="{xs[0]:.0f}" y="{Y(by0):.0f}" width="{xs[1]-xs[0]:.0f}" height="{(by1-by0)*SC:.0f}" fill="#2a3038" stroke="#4a5568" stroke-width="1.5"/>')
    def quad_path(q):
        return 'M' + ' L'.join(f'{X(cx):.1f},{Y(cy):.1f}' for cx, cy in q) + ' Z'
    # before ghosts
    for r, p in parts_here.items():
        if p['phantom']: continue
        q = p['quad_before']
        if max(c[0] for c in q) < x0 or min(c[0] for c in q) > x1: continue
        if max(c[1] for c in q) < y0 or min(c[1] for c in q) > y1: continue
        o.append(f'<path d="{quad_path(q)}" fill="none" stroke="#6b7280" stroke-width="0.8" opacity="0.55"/>')
    # after
    for r, p in parts_here.items():
        q = p['quad_after']
        if max(c[0] for c in q) < x0 or min(c[0] for c in q) > x1: continue
        if max(c[1] for c in q) < y0 or min(c[1] for c in q) > y1: continue
        dx = p['after'][0]-p['before'][0]; dy = p['after'][1]-p['before'][1]
        dist = math.hypot(dx, dy)
        if p['phantom']:
            fill, stroke = '#7c5cbf', '#b79ce8'
            op = '0.9'
        elif p['pinned']:
            fill, stroke = '#3d4653', '#5b6470'
            op = '0.85'
        elif dist > 0.05:
            fill, stroke = '#2e6e8e', '#7ec3e0'
            op = '0.85'
        else:
            fill, stroke = '#3a4552', '#66707e'
            op = '0.7'
        o.append(f'<path d="{quad_path(q)}" fill="{fill}" stroke="{stroke}" stroke-width="1" opacity="{op}"/>')
        # arrow
        if dist > 0.25 and not p['phantom']:
            o.append(f'<line x1="{X(p["before"][0]):.1f}" y1="{Y(p["before"][1]):.1f}" x2="{X(p["after"][0]):.1f}" y2="{Y(p["after"][1]):.1f}" stroke="#e0a44c" stroke-width="1.2" marker-end="url(#arr)"/>')
        # labels
        cx = sum(c[0] for c in q)/4; cy = sum(c[1] for c in q)/4
        if p['phantom'] and label_all_phantoms:
            o.append(f'<text x="{X(cx):.1f}" y="{Y(cy)+3:.1f}" text-anchor="middle" font-size="9" fill="#e8dff7" font-weight="bold">{r}</text>')
        elif dist > label_movers or (p['pinned'] and r.startswith(("J","H")) and SC >= 12):
            o.append(f'<text x="{X(cx):.1f}" y="{Y(cy)+3:.1f}" text-anchor="middle" font-size="8" fill="#c8cdd4">{r}</text>')
    o.append('<defs><marker id="arr" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto"><path d="M0,0 L6,3 L0,6 Z" fill="#e0a44c"/></marker></defs>')
    o.append('</svg>')
    open(out, 'w').write('\n'.join(o))
    print('wrote', out)

IMG = '/tmp/claude-1000/-home-sequoia-pcb-rp2350-motor-controller/e49668e9-e4e8-48c1-bc66-ca2c73581db5/scratchpad/report/img/'
svg_side('F', BX0, BY0, BX1, BY1, IMG+'jiggle_front.svg')
svg_side('B', BX0, BY0, BX1, BY1, IMG+'jiggle_back.svg')
# encoder corner zoom (B side), bigger scale
SC = 34
svg_side('B', 111.5, 97.5, 131, 129.5, IMG+'jiggle_encoder_zoom.svg', label_movers=0.5)
