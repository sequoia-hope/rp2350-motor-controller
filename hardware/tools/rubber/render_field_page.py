#!/usr/bin/env python3
"""Render review/field.html from field_results.json (+ the two mp4s).

    python3 render_field_page.py

field_results.json is written by the sweep; the videos come from animate.py.
"""
import html, json, os, sys
S = os.path.dirname(os.path.abspath(__file__)); HW = os.path.dirname(os.path.dirname(S))
ROOT = os.path.dirname(HW); REV = os.path.join(ROOT, 'review')
R = json.load(open(os.path.join(S, 'field_results.json')))
sw, eng, stg, pri = R['sweep'], R['engine'], R['staged'], R['prior']
abl = R['ablation']
prof = R['sigma_profile']
ks = sorted(sw, key=float)

STYLE = open(os.path.join(REV, 'modules.html')).read()
STYLE = STYLE[STYLE.index('<style>'):STYLE.index('</style>') + 8]


def pct(k):
    return f'+{(float(k)-1)*100:g}%'


def sweep_table():
    o = ['<div class="tablewrap"><table>',
         '<tr><th rowspan="2">stretch</th><th colspan="3">graph-harmonic (before)</th>'
         '<th colspan="3">spatial field (algorithm A)</th><th rowspan="2">violations<br>removed</th></tr>',
         '<tr><th>violations</th><th>sites</th><th>worst</th>'
         '<th>violations</th><th>sites</th><th>worst</th></tr>']
    for k in ks:
        a, b = sw[k]['old'], sw[k]['new']
        drop = 100.0 * (a['viol'] - b['viol']) / max(a['viol'], 1)
        o.append(f"<tr><td class='num'>{pct(k)}</td>"
                 f"<td class='num'>{a['viol']}</td><td class='num'>{a['sites']}</td>"
                 f"<td class='num'>{a['mx']} µm</td>"
                 f"<td class='num'><strong>{b['viol']}</strong></td>"
                 f"<td class='num'><strong>{b['sites']}</strong></td>"
                 f"<td class='num'>{b['mx']} µm</td>"
                 f"<td class='num'>−{drop:.0f}%</td></tr>")
    o.append('</table></div>')
    return '\n'.join(o)


def bars():
    """violations vs k, both engines, on one log-ish bar chart."""
    W, H, P = 470, 220, 40
    vmax = max(sw[k]['old']['viol'] for k in ks)
    bw = (W - P - 12) / (len(ks) * 2.6)
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="100%" '
         f'style="max-width:{W}px" font-family="ui-monospace,monospace" font-size="10">']
    o.append(f'<line x1="{P}" y1="{H-P}" x2="{W-8}" y2="{H-P}" stroke="#888"/>'
             f'<line x1="{P}" y1="{H-P}" x2="{P}" y2="8" stroke="#888"/>')
    for n in (0, 200, 400, 600):
        y = H - P - n / vmax * (H - P - 18)
        o.append(f'<text x="{P-4}" y="{y+3:.1f}" text-anchor="end" fill="#888">{n}</text>'
                 f'<line x1="{P}" y1="{y:.1f}" x2="{W-8}" y2="{y:.1f}" stroke="#8883" stroke-dasharray="2 4"/>')
    for i, k in enumerate(ks):
        x0 = P + 10 + i * (W - P - 22) / len(ks)
        for j, (alg, col) in enumerate((('old', '#b03434'), ('new', '#2c7a4b'))):
            v = sw[k][alg]['viol']
            hh = v / vmax * (H - P - 18)
            o.append(f'<rect x="{x0+j*bw:.1f}" y="{H-P-hh:.1f}" width="{bw:.1f}" height="{hh:.1f}" '
                     f'fill="{col}" fill-opacity="0.75"><title>{pct(k)} {alg}: {v}</title></rect>')
            o.append(f'<text x="{x0+j*bw+bw/2:.1f}" y="{H-P-hh-3:.1f}" text-anchor="middle" '
                     f'fill="{col}">{v}</text>')
        o.append(f'<text x="{x0+bw:.1f}" y="{H-P+13}" text-anchor="middle" fill="#888">{pct(k)}</text>')
    o.append(f'<rect x="{W-150}" y="12" width="9" height="9" fill="#b03434" fill-opacity="0.75"/>'
             f'<text x="{W-137}" y="20" fill="#888">graph-harmonic</text>')
    o.append(f'<rect x="{W-150}" y="26" width="9" height="9" fill="#2c7a4b" fill-opacity="0.75"/>'
             f'<text x="{W-137}" y="34" fill="#888">spatial field</text>')
    o.append(f'<text x="10" y="{H/2}" transform="rotate(-90 10 {H/2})" text-anchor="middle" '
             f'fill="#888">new DRC violations</text>')
    o.append('</svg>')
    return '\n'.join(o)


def layer_table():
    o = ['<div class="tablewrap"><table><tr><th>layer</th><th>σ<sub>min</sub></th>'
         '<th>1st pct</th><th>median</th><th>contracting cells</th></tr>']
    for name, d in eng['layers'].items():
        o.append(f"<tr><td class='mono'>{name}</td><td class='num'>{d['sigma_min']:.3f}</td>"
                 f"<td class='num'>{d['p01']:.4f}</td><td class='num'>{d['p50']:.4f}</td>"
                 f"<td class='num'>{100.0*d['contracting']/max(d['cells'],1):.1f}%</td></tr>")
    o.append('</table></div>')
    return '\n'.join(o)


def sigma_table():
    g = R['sigma_profile_gap_mm']
    o = ['<div class="tablewrap"><table><tr><th>distance from<br>nearest pad</th>'
         + ''.join(f'<th colspan="3">{n}</th>' for n in prof) + '</tr><tr>'
         + ''.join('<th>share of<br>contracting</th><th>worst σ</th>'
                   f'<th>worst loss on<br>a {g} mm gap</th>' for _ in prof) + '</tr>']
    for i, b in enumerate(prof[list(prof)[0]]['bins']):
        cells = ''
        for n in prof:
            d = prof[n]['bins'][i]
            cells += (f"<td class='num'>{d['share_bad']:.1f}%</td>"
                      f"<td class='num'>{d['worst']:.3f}</td>"
                      f"<td class='num'>{d['loss_um']:.1f} µm</td>")
        o.append(f"<tr><td class='num'>&gt; {b['thr']:.2f} mm</td>{cells}</tr>")
    o.append('</table></div>')
    return '\n'.join(o)


def ablation_table():
    o = ['<div class="tablewrap"><table><tr><th>engine, cumulative</th><th>violations</th>'
         '<th>sites</th><th>why</th></tr>']
    for a in abl:
        o.append(f"<tr><td class='mono'>{html.escape(a['name'])}</td>"
                 f"<td class='num'><strong>{a['viol']}</strong></td>"
                 f"<td class='num'>{a['sites']}</td>"
                 f"<td class='small'>{html.escape(a['note'])}</td></tr>")
    o.append('</table></div>')
    return '\n'.join(o)


def site_rows():
    return ''.join(
        f"<tr><td class='num'>{s['deficit']*1000:.0f}</td>"
        f"<td class='num'>({s['x']:.1f}, {s['y']:.1f})</td><td>{html.escape(s['type'])}</td>"
        f"<td class='mono small'>{html.escape(' | '.join(i[:70] for i in s['items']))}</td></tr>"
        for s in stg['site_list'])


k1 = sw['1.01']
page = f"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Warping Fields — RP2350 Motor Controller</title>
{STYLE}</head><body><main>
<header class="masthead"><p style="margin:0 0 .4rem"><a href="index.html" style="text-decoration:none">← project hub</a></p>
<h1>Warping Fields</h1>
<p class="subtitle">Interpolate the rigid-inclusion lag through <em>space</em>, not along each net's copper ·
algorithm A for the rubber stretch · {R['date']}</p></header>

<div class="callout"><strong>In one paragraph.</strong> The <a href="rubber.html">rubber stretch</a> failed for a
reason that turned out to be in the algorithm, not in the geometry. Its residual — the lag of a rigid footprint behind
the ambient dilation — was interpolated along <em>each net's own copper graph</em>, so two traces 0.2&nbsp;mm apart on
different nets got completely uncorrelated displacements and the gap between them changed by the difference of two
unrelated residuals. Replacing that with one harmonic displacement field <em>per copper layer</em>, evaluated by
position for every track, via, pad and zone vertex, removes
<strong>{100.0*(k1['old']['viol']-k1['new']['viol'])/k1['old']['viol']:.0f}% of the DRC damage</strong> at +1%
({k1['old']['viol']} → {k1['new']['viol']} violations) and between
{min(100.0*(sw[k]['old']['viol']-sw[k]['new']['viol'])/sw[k]['old']['viol'] for k in ks):.0f}% and
{max(100.0*(sw[k]['old']['viol']-sw[k]['new']['viol'])/sw[k]['old']['viol'] for k in ks):.0f}% across the sweep.
The +1% board now hands a human <strong>{stg['sites']} sites</strong> (median {stg['med']}&nbsp;µm, worst
{stg['mx']}&nbsp;µm) instead of {pri['sites']} (median {pri['med']}&nbsp;µm, worst {pri['mx']}&nbsp;µm), with
<code>check_sync</code> IN SYNC and no airwire regression at any k.</div>

<h2 id="why">1 · The bug was the interpolation domain</h2>
<p>Both engines agree on the physics: a footprint is a rigid chip glued to the sheet, so it takes a single translation
and its pads therefore trail full dilation by (k−1)·|ρ|, where ρ is each pad's offset from whatever point the part is
anchored at. (Which point that should be turns out to matter a great deal — see §4.) That lag has to be blended back
into the surrounding copper somewhere, and <em>where</em> is the whole question.</p>
<p>The old engine blended it along the copper graph — the same graph KiCad uses for connectivity. Nothing in that
construction knows about distance. A trace pinned to a connector's far pad rides the lagging footprint vector; the
trace running beside it, whose own pins are elsewhere on the board, rides near-pure ambient; and the gap between them
closes by up to the full lag. Every site in the old census is a different-net pair, every deficit sits in the
rigid-lag range, and the site count is nearly flat in k while the depth scales — which is exactly the signature of a
fixed set of pairs straddling a residual discontinuity whose magnitude grows with k.</p>
<p>A field that is a function of <em>position</em> cannot do that. Neighbouring copper moves together whatever nets it
belongs to, because it is evaluated at neighbouring points of one continuous field.</p>

<h2 id="video">2 · Watch it happen</h2>
<p>The whole top layer, stretched 0 → +10% and back, both engines side by side. Copper is coloured by its
<strong>lag behind the rubber</strong>. On the left, neighbouring traces take unrelated colours and visibly slide past
each other — that is the shear. On the right, colour varies smoothly with position: parts still lag, but everything
near them lags <em>with</em> them.</p>
<figure>
<video src="img/rubber_stretch.mp4" autoplay loop muted playsinline controls
       style="width:100%;border-radius:8px;background:#0d1117"></video>
<figcaption>F.Cu, 0 → +10% → 0, 8&nbsp;s loop. Both engines are exactly linear in (k−1), so each is solved once and
every frame is a scalar multiple — the animation is exact, not interpolated.</figcaption>
</figure>
<figure>
<video src="img/rubber_stretch_zoom.mp4" autoplay loop muted playsinline controls
       style="width:100%;border-radius:8px;background:#0d1117"></video>
<figcaption>The same run zoomed to a 26&nbsp;mm window on the dense corner, where the shear is at trace scale.
<code>ZOOM=cx,cy,width_mm python3 animate.py</code> re-aims it anywhere.</figcaption>
</figure>

<h2 id="engine">3 · The field</h2>
<p><code>hardware/tools/rubber/field.py</code> is a standalone solver — no pcbnew — so it can be reused for the
steerable "add space here" fields later. It solves, on a regular grid,</p>
<p class="mono" style="text-align:center">u(x) = ambient(x) + r(x)&nbsp;&nbsp;·&nbsp;&nbsp;u = v<sub>i</sub> inside
rigid inclusion i&nbsp;&nbsp;·&nbsp;&nbsp;∇²r = 0 in free space&nbsp;&nbsp;·&nbsp;&nbsp;r → 0 on the domain boundary</p>
<p>with <code>ambient</code> an arbitrary callable (a dilation here, <code>None</code> for a pure push-these-apart
field) and inclusion vectors arbitrary. Four decisions were forced by measurement rather than taste:</p>
<ul>
<li><strong>Territories are pads, not courtyards.</strong> What must stay rigid is a part's pad geometry; copper
merely passing under a courtyard is free to deform, and a trace under a QFN is exactly that. Courtyard territories
also make abutting parts fight over the seam cells between them. All pads of one footprint share one vector, so the
part stays rigid and the space between its own pads comes out rigid for free.</li>
<li><strong>Plus what the pads enclose.</strong> A morphological closing of each part's own pad set: a trace threading
a connector's pin field is held on both sides and can do nothing but move with the part. Without it the field sags
toward ambient between the pins while the pins hold, and shears precisely those traces.</li>
<li><strong>One field per copper layer.</strong> Clearance rules live inside a layer, never between them, and this
board is six layers. A single 2-D field forces parts on opposite sides that overlap in projection to fight over the
same cells — measured here as Y1's pad owned by U16 and U7's by CL1, worth up to 57&nbsp;µm of bogus shear.</li>
<li><strong>Vias couple the layers, in two passes.</strong> A via barrel is one rigid body piercing the whole stack,
so it can only be in one place: it takes the mean of what the layers it spans ask for, and then that displacement is
handed back to every layer as a rigid inclusion and the fields re-solved. Without the second pass a via shears against
the copper beside it by however much the layers disagreed — which was most of what remained.</li>
</ul>
<p>Two discretisation passes keep it honest. A straight segment cannot follow a curved field, so each track is
subdivided until its chord is within <strong>{eng['split_tol']*1000:.0f}&nbsp;µm</strong> of the field
({eng['split']} tracks split, +{eng['new_segments']} segments at +1%, worst chord sag
{eng['sag_max_um']:.0f}&nbsp;µm). That is also what holds tee junctions — the 364 track ends resting on a passing
segment's body, which KiCad connects by overlap and not by topology — and same-net pads crossed mid-body, onto their
host. And every track end that lands on a pad is pinned to that footprint's vector directly; the field already equals
it there, so the two agree to <strong>{eng['pin_err_um']:.0f}&nbsp;µm</strong>, which is the honest measure of how
well the territories cover the pads.</p>
<p>Everything that pierces the board is rigid on every layer it pierces, which is not the same as everything that
has copper on it: a drill carries its own DRC clearance whether or not it has an annulus, so all {eng['holes']}
holes ({eng['npth']} of them NPTH, with no net and no copper obligation) are pinned on all six layers rather than
relying on a pad polygon happening to cover them. Off the copper layers, free silk art is stretched point by point
through the field ({eng['silk_art']} graphics) and silk text moves through it and scales by k ({eng['silk_text']}
texts) — text cannot deform, so scaling is the honest compromise for something that has to stay legible. Footprint
silk is untouched: it belongs to a rigid part and already moved with it.</p>
<p>Each of those four decisions was kept because DRC said so, at +1%, cumulatively:</p>
{ablation_table()}

<h2 id="anchor">4 · What the field map found: mounting holes</h2>
<p>Drawing the field over the board is what turned up the next bug, and it was not where it looked. The question was
whether NPTH holes and stitching vias were being pinned at all. They were — every one of this board's
{eng['holes']} drilled holes ({eng['npth']} NPTH) is rigid on all six layers and all {eng['via_inclusions']} via
barrels are coupled across the stack — and yet the field around the phase connectors' mounting holes was visibly
inconsistent, and copper was colliding there.</p>
<p>The cause was the <em>anchor</em>. A rigid part gets one translation to choose, and the engine was using
ambient(footprint origin). The footprint origin is an arbitrary CAD anchor, and on this board the phase connectors
J1/J2/J9 carry theirs <strong>{eng['anchor_worst_mm']} mm</strong> off their own pad centroid (J12 5.10&nbsp;mm, J11
3.83&nbsp;mm; {eng['anchor_off_count']} footprints are off by more than 50&nbsp;µm). Anchoring there dragged every one
of those parts' pads — mounting holes included — through an extra 56&nbsp;µm of lag at +1% for no reason whatever.</p>
<p>The right choice is the least-squares one: the translation minimising Σ|v − ambient(pad<sub>i</sub>)|² over the
part's own pads, which for an affine ambient is exactly <strong>ambient(pad centroid)</strong>. That single change
took +1% from 47 to <strong>37</strong> violations, and it smooths the field everywhere, not just at connectors:
tracks needing a split fell 134 → {eng['split']}, worst chord sag 46 → {eng['sag_max_um']:.0f}&nbsp;µm, F.Cu
σ<sub>min</sub> 0.63 → {eng['layers']['F.Cu']['sigma_min']:.2f}.</p>
<figure>
<img src="img/fieldmap_conn.png" alt="warp field around the phase connector J9" style="width:100%;border-radius:6px">
<figcaption>B.Cu around phase connector J9, +1%. Left: lag behind the ambient dilation, with arrows and the rigid
territories outlined in green; blue circles are plated holes, red are NPTH. J9's four pads — two power pads and two
mounting holes 12.8&nbsp;mm apart — read as four bright plateaus, and the <em>dark spot at their centre</em> is the
pad centroid the part is now anchored at. Right: σ<sub>min</sub>, where the dark red bands land exactly on the pad
rims facing the connector interior.</figcaption>
</figure>
<p>The map also shows what has <em>not</em> been solved. A connector's four pads remain four rigid islands with a
saddle between them, because the space they span is not part of the connector at all — it is open board with other
parts and live traces in it. Making that span rigid (<code>ENCLOSE=hull</code>) was tried and is worse with the
origin anchor (61 violations against 47, with fresh 70–78&nbsp;µm sites along the hull boundary) and a wash with the
centroid anchor (38 against 37), so the closing is kept. The residual sag inside those bodies is 29&nbsp;µm, down
from 43.</p>
<figure>
<img src="img/fieldmap_In3.png" alt="warp field over the whole board, In3.Cu" style="width:100%;border-radius:6px">
<figcaption>The whole board on In3.Cu, where the only rigid things are through-hole pads, drills and via barrels — so
the through-hole connectors stand out as the bright plateaus and the stitching-via farms as the regular dotted
lattices. <code>fieldmap.py</code> draws the field <code>stretch.py</code> dumped, never a rebuilt one, so the picture
is the field the board actually got.</figcaption>
</figure>

<h2 id="admissibility">5 · Does the field close anything, anywhere?</h2>
<p>The map is F(x) = x + u(x), so a gap between two nearby points survives if and only if the smallest singular value
of J = I + ∇u is ≥ 1 — a geometry-only test that does not care what copper happens to be there. It is worth knowing
that the construction is non-contracting <em>by design</em> in the clean case: for a single circular rigid inclusion
of radius a in a dilation field, the harmonic residual gives exactly</p>
<p class="mono" style="text-align:center">σ<sub>min</sub> = 1 + (k−1)(1 − a²/r²) ≥ 1 for r ≥ a</p>
<p>with equality on the inclusion boundary — gaps just outside a part are preserved, and everything further out
grows. Real footprints are neither circular nor isolated, so the grid map below says how far the board strays from
that. Read the percentile, not the minimum: harmonic gradients blow up at the sharp corners of a polygonal pad, so
the global minimum is a corner artifact that gets <em>worse</em> as the grid is refined, while the bulk of the board
sits just above 1.</p>
{layer_table()}
<p class="small">Grid {eng['h']} mm. The median is above 1 on every layer — the board is expanding almost
everywhere.</p>
<p>A share of contracting cells is not by itself alarming, because σ<sub>min</sub> = 0.9999 and
σ<sub>min</sub> = 0.53 both count. What matters is <em>how deep</em> the dip is and <em>where</em> it sits, so
<code>sigma_probe.py</code> bins it by distance from the nearest rigid territory and converts the depth into what it
would actually cost a {R['sigma_profile_gap_mm']}&nbsp;mm clearance:</p>
{sigma_table()}
<p>Contraction is now, to the cell, a near-territory phenomenon: beyond 2&nbsp;mm from any rigid shape there is
<strong>not one contracting cell on either layer</strong>, and beyond 1&nbsp;mm the worst survivor costs
{max(prof[l]['bins'][4]['loss_um'] for l in prof):.1f}&nbsp;µm of a {R['sigma_profile_gap_mm']}&nbsp;mm gap while the
median costs {(1-prof['F.Cu']['bins'][4]['median'])*R['sigma_profile_gap_mm']*1000:.1f}&nbsp;µm. The deep values —
{prof['B.Cu']['bins'][0]['worst']:.2f} on B.Cu — sit hard against pad rims, in seams where nothing but those two pads
lives and they are moving apart anyway.</p>
<p class="small">This is stricter than it was before the anchor fix, and worth recording as a correction: with the
parts anchored at their footprint origins, a tenth to a quarter of the contraction really was out in open routing
space, and an earlier draft of this page that called it "all pad rims" was wrong at the time. Moving the anchor to the
pad centroid is what emptied the open-space bins.</p>
<p class="small">This is the map earning its keep as an oracle: it predicts a median cost of well under a micron and
a worst case of a few tens, and the DRC that follows reports a median deficit of {sw['1.01']['new']['med']}&nbsp;µm
and a worst of {sw['1.01']['new']['mx']}&nbsp;µm. A proposed field can be judged before it is applied.</p>

<h2 id="sweep">6 · Tolerance sweep, head to head</h2>
{sweep_table()}
{bars()}
<p>Both engines measured with the same tool (<code>status.py</code>: DRC, position-independent diff against the
pre-stretch baseline, site clustering), on the same board, at the same k, before any nudging.
<code>check_sync</code> reports IN SYNC in all {2*len(ks)} runs. Algorithm A at <strong>+3%</strong> is still
cleaner than the old engine at <strong>+0.25%</strong> — {sw['1.03']['new']['viol']} violations against
{sw['1.0025']['old']['viol']} — a twelve-fold larger stretch for
{sw['1.0025']['old']['viol']/sw['1.03']['new']['viol']:.0f}× less damage. The old engine also opens an
airwire at +3% ({sw['1.03']['old']['air']} against a baseline of 54); algorithm A does not
({sw['1.03']['new']['air']}).</p>

<h2 id="handoff">7 · The staged board</h2>
<p><code>{stg['board']}</code> — +1%, nudged, {stg['viol']} violations in
<strong>{stg['sites']} sites</strong> (median {stg['med']}&nbsp;µm, p90 {stg['p90']}&nbsp;µm, worst
{stg['mx']}&nbsp;µm), airwires {stg['airwires']} against a baseline of {stg['airwires_base']},
<code>{html.escape(stg['sync'])}</code>. The prior pipeline handed over {pri['sites']} sites at median
{pri['med']}&nbsp;µm and worst {pri['mx']}&nbsp;µm, so this is {pri['sites']/stg['sites']:.1f}× less hand work at
{pri['mx']/stg['mx']:.1f}× less worst-case depth. Work it worst-first with <code>python3 hardware/tools/rubber/status.py --board
hardware/rp2350_driver_rubber_A.kicad_pcb</code>.</p>
<div class="tablewrap"><table><tr><th>µm</th><th>position</th><th>type</th><th>items</th></tr>
{site_rows()}</table></div>

<h2 id="next">8 · Where this goes</h2>
<p>The field solver is deliberately more general than this one use. <code>ambient</code> is any callable and
inclusion vectors are arbitrary, so the same machinery expresses "push these two parts apart and let the copper
between them relax harmonically" with no dilation at all — the shape a steerable <em>add space here</em> operator
needs. The pieces that already exist for that: the σ<sub>min</sub> map is an admissibility oracle that says whether a
proposed field is legal <em>before</em> DRC runs, and exact linearity in the drive amplitude means a candidate field
is solved once and then scaled to find the largest legal amount. Combining that with jiggling is the next step; the
open question is the same one the <a href="creep.html">creep</a> and <a href="modules.html">module</a> work hit — no
field of any kind reroutes a trace, so the {stg['airwires']} open airwires stay a topological problem.</p>

<p class="small" style="margin-top:2rem">Tooling: <code>hardware/tools/rubber/field.py</code> (solver),
<code>stretch.py</code> (board application), <code>animate.py</code> (the videos),
<code>render_field_page.py</code> (this page), <code>field_results.json</code> (its data).
The superseded graph-harmonic engine is preserved at git <code>cbc2240</code>.</p>
</main></body></html>"""

out = os.path.join(REV, 'field.html')
open(out, 'w').write(page)
print('wrote', out, f'({len(page)} bytes)')
