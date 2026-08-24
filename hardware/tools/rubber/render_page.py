#!/usr/bin/env python3
"""Render review/rubber.html from the sweep + handoff artefacts.
usage: render_page.py SWEEP_DIR   (expects sweep2_results.txt, s2_<k>/ dirs,
hardware/rp2350_driver_rubber.kicad_pcb staged, review/rubber_status.json)"""
import json, math, os, re, subprocess, sys, html
S = os.path.dirname(os.path.abspath(__file__)); HW = os.path.dirname(os.path.dirname(S))
ROOT = os.path.dirname(HW); REV = os.path.join(ROOT, 'review')
SW = sys.argv[1]
res_txt = open(os.path.join(SW, 'sweep2_results.txt')).read()
rows = []
for m in re.finditer(r'K=([\d.]+) pre=(\d+) final=(\d+) sites=(\d+) air=(\d+) rounds=(\d+) '
                     r'def_med=(\d+) def_p90=(\d+) def_max=(\d+)', res_txt):
    rows.append(dict(k=float(m.group(1)), pre=int(m.group(2)), fin=int(m.group(3)),
                     sites=int(m.group(4)), air=int(m.group(5)), rounds=int(m.group(6)),
                     med=int(m.group(7)), p90=int(m.group(8)), mx=int(m.group(9))))
rows.sort(key=lambda r: r['k'])
traj = {}
for r in rows:
    lg = json.load(open(os.path.join(SW, f"s2_{r['k']:g}", 'nudge_log.json')))
    traj[r['k']] = [x['sites'] for x in lg['rounds']]
status = json.load(open(os.path.join(REV, 'rubber_status.json')))
sites = status['sites']

# gaps table across boards
boards = [os.path.join(HW, 'rp2350_driver.kicad_pcb')] + \
         [os.path.join(SW, f"s2_{r['k']:g}", 'nudged.kicad_pcb') for r in rows]
gp = subprocess.run([sys.executable, os.path.join(S, 'gaps.py')] + boards,
                    capture_output=True, text=True).stdout.strip().splitlines()

# board bbox for the map (handoff board)
sys.path.insert(0, os.path.join(HW, 'tools', 'jiggle2'))
from common import load_board, NM
bd = load_board(os.path.join(HW, 'rp2350_driver_rubber.kicad_pcb'))
bb = bd.GetBoardEdgesBoundingBox()
X0, Y0, X1, Y1 = bb.GetLeft()*NM, bb.GetTop()*NM, bb.GetRight()*NM, bb.GetBottom()*NM

def svg_map():
    SC = 11.5; PAD = 8
    W = (X1-X0)*SC + 2*PAD; H = (Y1-Y0)*SC + 2*PAD
    X = lambda x: PAD + (x-X0)*SC; Y = lambda y: PAD + (y-Y0)*SC
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W:.0f} {H:.0f}" '
         f'width="100%" style="max-width:{W:.0f}px" font-family="ui-monospace,monospace">']
    o.append(f'<rect x="{X(X0):.1f}" y="{Y(Y0):.1f}" width="{(X1-X0)*SC:.1f}" height="{(Y1-Y0)*SC:.1f}" '
             f'rx="8" fill="none" stroke="#888" stroke-width="1.5"/>')
    for s in sites:
        r = 2.0 + min(s['deficit'], 0.12)/0.12*6.0
        col = '#b03434' if s['deficit'] > 0.05 else ('#a05a00' if s['deficit'] > 0.02 else '#6b4fa0')
        t = html.escape(f"{s['deficit']*1000:.0f} um: " + ' | '.join(i[:60] for i in s['items']))
        o.append(f'<circle cx="{X(s["x"]):.1f}" cy="{Y(s["y"]):.1f}" r="{r:.1f}" fill="{col}" '
                 f'fill-opacity="0.55" stroke="{col}"><title>{t}</title></circle>')
    o.append(f'<text x="{X(X0)+4}" y="{Y(Y1)-6}" font-size="10" fill="#888">'
             f'{X1-X0:.1f} × {Y1-Y0:.1f} mm · dot area ∝ deficit · red &gt;50 µm, amber &gt;20 µm</text>')
    o.append('</svg>')
    return '\n'.join(o)

def svg_traj():
    W, H, P = 460, 220, 34
    kmax = max(max(v) for v in traj.values()); rmax = max(len(v) for v in traj.values())
    xs = lambda i: P + i/(max(rmax-1, 1))*(W-P-10)
    ys = lambda n: H-P - n/kmax*(H-P-14)
    cols = ['#6b4fa0', '#2c7a4b', '#a05a00', '#b03434']
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="100%" style="max-width:{W}px" font-family="ui-monospace,monospace" font-size="10">']
    o.append(f'<line x1="{P}" y1="{H-P}" x2="{W-8}" y2="{H-P}" stroke="#888"/>'
             f'<line x1="{P}" y1="{H-P}" x2="{P}" y2="8" stroke="#888"/>')
    for n in range(0, kmax+1, 50):
        o.append(f'<text x="{P-4}" y="{ys(n)+3}" text-anchor="end" fill="#888">{n}</text>'
                 f'<line x1="{P}" y1="{ys(n):.1f}" x2="{W-8}" y2="{ys(n):.1f}" stroke="#8883" stroke-dasharray="2 4"/>')
    for i, (k, v) in enumerate(sorted(traj.items())):
        pts = ' '.join(f'{xs(j):.1f},{ys(n):.1f}' for j, n in enumerate(v))
        o.append(f'<polyline points="{pts}" fill="none" stroke="{cols[i%4]}" stroke-width="1.8"/>')
        o.append(f'<text x="{xs(len(v)-1)+3:.0f}" y="{ys(v[-1])+3:.1f}" fill="{cols[i%4]}">{(k-1)*100:g}%</text>')
    o.append(f'<text x="{(W+P)/2}" y="{H-8}" text-anchor="middle" fill="#888">nudge round</text>')
    o.append(f'<text x="10" y="{H/2}" transform="rotate(-90 10 {H/2})" text-anchor="middle" fill="#888">open sites</text>')
    o.append('</svg>')
    return '\n'.join(o)

def gaps_table():
    hdr = ['pair', 'rev-B board'] + [f"+{(r['k']-1)*100:g}%" for r in rows]
    out = ['<div class="tablewrap"><table><tr>' + ''.join(f'<th>{h}</th>' for h in hdr) + '</tr>']
    for ln in gp[1:]:
        cells = ln.split()
        name = cells[0]; vals = cells[1:]
        out.append('<tr><td class="mono">' + name + '</td>' +
                   ''.join(f'<td class="num">{v}</td>' for v in vals) + '</tr>')
    out.append('</table></div>')
    return '\n'.join(out)

srow = ''.join(
    f"<tr><td class='num'>+{(r['k']-1)*100:g}%</td><td class='num'>{r['pre']}</td>"
    f"<td class='num'>{r['fin']}</td><td class='num'><strong>{r['sites']}</strong></td>"
    f"<td class='num'>{r['med']}</td><td class='num'>{r['p90']}</td><td class='num'>{r['mx']}</td>"
    f"<td class='num'>{r['air']}</td><td class='num'>{r['rounds']}</td></tr>" for r in rows)

top = ''.join(
    f"<tr><td class='num'>{s['deficit']*1000:.0f}</td><td class='num'>({s['x']:.1f}, {s['y']:.1f})</td>"
    f"<td>{html.escape(s['type'])}</td><td class='mono small'>{html.escape(' | '.join(i[:70] for i in s['items']))}</td></tr>"
    for s in sites[:40])

STYLE = open(os.path.join(REV, 'modules.html')).read()
STYLE = STYLE[STYLE.index('<style>'):STYLE.index('</style>')+8]

page = f"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Rubber Stretch &amp; Violation Nudging — RP2350 Motor Controller</title>
{STYLE}</head><body><main>
<header class="masthead"><p style="margin:0 0 .4rem"><a href="index.html" style="text-decoration:none">← project hub</a></p>
<h1>Rubber Stretch &amp; Violation Nudging</h1>
<p class="subtitle">Stretch the whole board like rubber · absorb the damage with DRC-refereed trace nudges · hand the remainder to a human · 2026-08-24</p></header>

<div class="callout"><strong>In one paragraph.</strong> The board was dilated about its centre like a rubber sheet with the
footprints as rigid chips glued on: every footprint translates by the dilation of its origin, copper takes the ambient
stretch field plus a graph-harmonic correction that keeps every trace end glued to its pad, zones/silk/outline follow, and
the corner arcs map exactly. The damage is precisely the predicted rigid-inclusion lag — pad copper trails full dilation by
(k−1)·|pad offset|, so every at-limit pair sitting in a footprint's shadow violates — and it is <strong>flat in count but
linear in depth</strong>: {rows[0]['sites']} residual sites at +0.25% and {rows[-2]['sites'] if len(rows)>1 else '—'} at +1%,
while the median deficit scales roughly ∝&nbsp;k. A violation-driven nudger (site-clustered, tee-aware, DRC-refereed every
round, ~1&nbsp;s per round) absorbs roughly half of the damage automatically and <em>never</em> loses a connection: airwires
stay at the baseline 54 and <code>check_sync</code> stays IN SYNC at every k. The +1% board is staged as
<code>hardware/rp2350_driver_rubber.kicad_pcb</code> with {len(sites)} sites (median {int(sorted(s['deficit'] for s in sites)[len(sites)//2]*1000)}&nbsp;µm)
left for hand repair via <code>status.py</code>.</div>

<h2 id="theory">1 · Why "just scale everything" is illegal (F-46)</h2>
<p>Tracks, vias and pour outlines are defined by scalable points with fixed radii — a pure dilation ×k strictly grows every
copper gap between them. Footprints break the picture: a pad's copper moves with the footprint <em>origin</em>, lagging full
dilation by (k−1)·|ρ| where ρ is the pad's offset from the origin. For J12-class connectors |ρ| reaches ~10&nbsp;mm, so at
k&nbsp;=&nbsp;1.05 the far pads trail the rubber by ~500&nbsp;µm while an at-limit neighbour gained ~20. Worse, the sign is
geometric: whenever other-net copper sits at rule clearance <em>between a footprint's origin and one of its pads</em>, the gap
change is negative at <em>any</em> k&nbsp;&gt;&nbsp;1 — QFN fanout and connector pin fields guarantee such pairs. The measured
census below confirms it: the number of violating sites barely moves with k (it is the at-limit set with negative geometric
factor), only the depth scales. Strain must route <em>around</em> rigid inclusions — this page's engine does that smoothly;
the provably-clean alternative is the slice-insertion family (staircase rubber) from the stretch discussion.</p>

<h2 id="engine">2 · The stretch engine</h2>
<p><code>hardware/tools/rubber/stretch.py</code> — footprints rigid (vector = dilation of origin), copper owned by
connectivity exactly as in the module-stretch warp: track ends hit-tested onto pads are pinned to that footprint's vector,
free graph nodes take <em>ambient dilation + harmonic residual</em> so the rigid-inclusion lag decays along each net's own
copper, pad-less via farms ride pure ambient, zone outlines follow the footprint field inside courtyards and ambient outside,
zones refill, Edge.Cuts lines stretch and the corner arcs map start/mid/end through the field (exact under isotropic scale).
On this board: 5196 copper-graph nodes, residual lag max 102&nbsp;µm / mean 17&nbsp;µm at +1%; the stretch itself never
touches an airwire (54 before, 54 after) because overlap junctions move coherently under a smooth field.</p>
<div class="callout warn"><strong>Two model honesty notes.</strong> (1) pcbnew's <code>GetCourtyard</code> polygons disagree
with the DRC courtyard test in <em>both</em> directions here (five connector/passive pairs read as crossing in pcbnew yet
clean in DRC) — a model-based courtyard relax cannot work, so courtyard overlaps are handed to the nudger as DRC-refereed
footprint micro-moves. (2) Moving U8 at all wakes 9 cosmetic <code>padstack</code> warnings on its thermal-via stack
(negative mask clearance, a pre-existing footprint property); they are filtered and documented, not repaired.</div>

<h2 id="nudge">3 · The nudge engine</h2>
<p><code>nudge.py</code> runs DRC (~1&nbsp;s on this board), diffs position-independently against the pre-stretch baseline,
clusters instances into physical sites, and pushes the offending track's chain apart by deficit&nbsp;+&nbsp;15&nbsp;µm with a
2&nbsp;mm along-chain falloff — re-refereeing with real DRC every round, so bundle cascades propagate honestly. What made it
safe took three iterations: <strong>tee endpoints</strong> (364 track ends on this board rest on a passing segment's body —
KiCad connects by overlap, not topology) and <strong>via-rim ends</strong> now <em>ride</em> their bar instead of being moved
or snapped; segments crossing same-net pads mid-body pin their ends; vias are movable items; courtyard overlaps move the
lighter footprint in 20&nbsp;µm steps with its glued copper dragged along. Per-node moves cap at 0.12&nbsp;mm/round,
0.35&nbsp;mm total; the loop keeps the best round and stops after four rounds without improvement.</p>
<figure><div class="scrollfig">{svg_traj()}</div>
<figcaption>Open sites per nudge round, one line per stretch amount. Roughly half the sites close; the rest plateau —
squeezed bundles where the push has nowhere to go without a reroute.</figcaption></figure>

<h2 id="sweep">4 · Tolerance sweep</h2>
<div class="tablewrap"><table>
<tr><th>stretch</th><th>new violations<br>post-stretch</th><th>post-nudge</th><th>sites left</th>
<th>median deficit (µm)</th><th>p90 (µm)</th><th>max (µm)</th><th>airwires</th><th>rounds</th></tr>
{srow}
</table></div>
<p class="small">Airwires never regress; the 53 at +2% is one of the four baseline +3V3 airwires closing by accident
(moved same-net copper overlaps). Nothing new opens at any k.</p>
<p>The census is the theory made visible: the at-limit pair set (negative geometric factor) violates at every k — count is
nearly flat — while deficits scale with k. Nudging tolerates all of it in the sense that <em>nothing is lost</em> (airwires
and sync invariant) and half the sites are closed outright; what remains is exactly the set needing a human's reroute
judgement, and its size is the honest price of the stretch.</p>

<h2 id="payoff">5 · What the stretch buys</h2>
{gaps_table()}
<p class="small">Pad-bbox gaps in µm at the F-25 blockers (choke↔transceiver noses, pull-downs in mounting-hole rings,
termination switches), and board size. Corridor-type gaps grow by ~(k−1)·pair distance — tens of µm at these k — and
<strong>R111 stays inside H2's ring at any small k</strong>: uniform rubber does not unbox the placement blockers. For the 54
open airwires the fixes remain placement moves (or targeted slice-inserts); the rubber's value is global breathing room, at
the price of the residual table above.</p>

<h2 id="handoff">6 · Hand-repair workflow</h2>
<p>The staged board is <code>hardware/rp2350_driver_rubber.kicad_pcb</code> (+1%, nudged, airwires 54,
<code>check_sync</code> IN SYNC). Open it in pcbnew, run DRC, and work the map worst-first — every dot below is a small
local slide or reroute (median {int(sorted(s['deficit'] for s in sites)[len(sites)//2]*1000)}&nbsp;µm). After each session run
<code>python3 hardware/tools/rubber/status.py</code>: it re-diffs against the pre-stretch baseline, reprints this list,
checks airwires (must stay 54) and schematic sync, and exits 0 when the board is clean. To try a different stretch, re-run
<code>stretch.py</code> / <code>nudge.py</code> at any KX/KY — every stage is a fresh derivation from
<code>rp2350_driver.kicad_pcb</code>, which is never modified.</p>
<figure>{svg_map()}
<figcaption>The {len(sites)} remaining sites on the +1% board. Hover a dot for nets and deficit.</figcaption></figure>
<h3>Worst 40 sites</h3>
<div class="tablewrap"><table>
<tr><th>deficit µm</th><th>position</th><th>type</th><th>items</th></tr>
{top}
</table></div>
<p class="small">Generated by <code>hardware/tools/rubber/render_page.py</code>. Companion pages:
<a href="modules.html">module stretch</a> (F-44: no module-level slack) · <a href="findings.html">findings register</a>.</p>
</main></body></html>"""
open(os.path.join(REV, 'rubber.html'), 'w').write(page)
print('wrote review/rubber.html', len(page), 'bytes')
