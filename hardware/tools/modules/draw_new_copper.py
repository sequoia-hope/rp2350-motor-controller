#!/usr/bin/env python3
"""SVG of the copper the routing pass added (new segments/vias vs the pre-route board), per layer colour, both views."""
import sys, os, json
sys.path.insert(0, '/home/sequoia/pcb/rp2350-motor-controller/hardware/tools/jiggle2')
from common import load_board, NM
import pcbnew
PRE, POST, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
pre = load_board(PRE); post = load_board(POST)
def sig(t):
    if t.GetClass() == 'PCB_VIA': p = t.GetPosition(); return ('V', p.x, p.y)
    s, e = t.GetStart(), t.GetEnd(); a, b = (s.x, s.y), (e.x, e.y)
    if b < a: a, b = b, a
    return ('T', int(t.GetLayer()), a, b, t.GetWidth())
ps = {sig(t) for t in pre.GetTracks()}
new = [t for t in post.GetTracks() if sig(t) not in ps]
bb = post.GetBoardEdgesBoundingBox(); x0, y0, x1, y1 = bb.GetLeft()*NM, bb.GetTop()*NM, bb.GetRight()*NM, bb.GetBottom()*NM
SC = 12; PAD = 8; W = (x1-x0)*SC+2*PAD; H = (y1-y0)*SC+2*PAD+20
X = lambda x: PAD+(x-x0)*SC; Y = lambda y: PAD+(y-y0)*SC
COL = {'F.Cu': '#c83c3c', 'In2.Cu': '#c89a1e', 'In3.Cu': '#1e8cc8', 'B.Cu': '#2e8c4b'}
o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W:.0f} {H:.0f}" width="{W:.0f}" height="{H:.0f}" font-family="ui-monospace,monospace">', f'<rect width="{W:.0f}" height="{H:.0f}" fill="#fff"/>',
     f'<rect x="{X(x0):.1f}" y="{Y(y0):.1f}" width="{(x1-x0)*SC:.1f}" height="{(y1-y0)*SC:.1f}" fill="#f4f2ee" stroke="#333"/>']
# faint footprints (both sides)
for f in post.GetFootprints():
    b = f.GetBoundingBox(False, False)
    o.append(f'<rect x="{X(b.GetLeft()*NM):.1f}" y="{Y(b.GetTop()*NM):.1f}" width="{b.GetWidth()*NM*SC:.1f}" height="{b.GetHeight()*NM*SC:.1f}" fill="#d8d5cf" fill-opacity="0.5" stroke="none"/>')
# old copper faint
for t in post.GetTracks():
    if sig(t) in ps and t.GetClass() == 'PCB_TRACK':
        s, e = t.GetStart(), t.GetEnd(); o.append(f'<line x1="{X(s.x*NM):.1f}" y1="{Y(s.y*NM):.1f}" x2="{X(e.x*NM):.1f}" y2="{Y(e.y*NM):.1f}" stroke="#b8b4ad" stroke-width="0.6" stroke-opacity="0.6"/>')
for t in new:
    if t.GetClass() == 'PCB_VIA':
        p = t.GetPosition(); o.append(f'<circle cx="{X(p.x*NM):.1f}" cy="{Y(p.y*NM):.1f}" r="{0.25*SC:.1f}" fill="#222" fill-opacity="0.8"/>')
    else:
        s, e = t.GetStart(), t.GetEnd(); ln = post.GetLayerName(t.GetLayer())
        o.append(f'<line x1="{X(s.x*NM):.1f}" y1="{Y(s.y*NM):.1f}" x2="{X(e.x*NM):.1f}" y2="{Y(e.y*NM):.1f}" stroke="{COL.get(ln, "#000")}" stroke-width="{max(1.2, t.GetWidth()*NM*SC):.1f}" stroke-linecap="round"/>')
# remaining airwires from DRC json if given
if len(sys.argv) > 4:
    d = json.load(open(sys.argv[4]))
    for it in d['unconnected_items']:
        a, b = it['items'][0]['pos'], it['items'][1]['pos']
        o.append(f'<line x1="{X(a["x"]):.1f}" y1="{Y(a["y"]):.1f}" x2="{X(b["x"]):.1f}" y2="{Y(b["y"]):.1f}" stroke="#b03434" stroke-width="1.2" stroke-dasharray="3,2"/>')
lx = PAD; ly = H-6
for k, c in COL.items():
    o.append(f'<line x1="{lx}" y1="{ly-4}" x2="{lx+18}" y2="{ly-4}" stroke="{c}" stroke-width="3"/><text x="{lx+22}" y="{ly}" font-size="9" fill="#222">{k}</text>'); lx += 75
o.append(f'<circle cx="{lx+5}" cy="{ly-4}" r="3" fill="#222"/><text x="{lx+12}" y="{ly}" font-size="9" fill="#222">new via</text>')
o.append(f'<line x1="{lx+70}" y1="{ly-4}" x2="{lx+88}" y2="{ly-4}" stroke="#b03434" stroke-width="1.2" stroke-dasharray="3,2"/><text x="{lx+92}" y="{ly}" font-size="9" fill="#222">remaining airwire</text>')
o.append('</svg>'); open(OUT, 'w').write('\n'.join(o)); print('wrote', OUT, len(new), 'new items')
