#!/usr/bin/env python3
"""diffusion experiment: build review/diffusion.html from the results.

Pure stdlib; run with system python after evaluate.py + footgeom.py. Inline
SVG panels in the review hub's house style. View matches the KiCad editor
(y down, front view); the movers are back-side parts seen through the board.

Two page-wide views, toggled client-side: footprints (real pad + fab/silk
body geometry from footprints.json, drawn at each placement in shipped
orientation) and designators (courtyard boxes + refs, ICs highlighted with
a hand-written descriptor legend).
"""
import html, json, os

HERE = os.path.dirname(os.path.abspath(__file__))
REVIEW = os.path.join(HERE, '..', '..', '..', 'review')

with open(os.path.join(HERE, 'rp2350_netlist.json')) as f:
    NL = json.load(f)
with open(os.path.join(HERE, 'diffusion_results.json')) as f:
    R = json.load(f)
with open(os.path.join(HERE, 'footprints.json')) as f:
    FP = json.load(f)
objs, nets = NL['objects'], NL['nets']
cx0, cy0, cx1, cy1 = NL['canvas']
name_of = [o['name'] for o in objs]
idx_of = {o['name']: i for i, o in enumerate(objs)}
CW, CH = cx1 - cx0, cy1 - cy0

# Hand-written descriptors for the ICs (plus the one discrete transistor).
# (part, package, role) — from the schematic netlist, entered manually.
ICS = {
    'U5':  ('INA240A1D', 'SOIC-8',
            'bidirectional PWM-rejecting current-sense amplifier (20 V/V) — '
            'phase-shunt monitor feeding A_SENSE'),
    'U18': ('SIT3088ETK', 'DFN-8',
            'RS-485/RS-422 transceiver — encoder channel A differential data'),
    'U19': ('SIT3088ETK', 'DFN-8',
            'RS-485/RS-422 transceiver — encoder channel B differential data'),
    'U20': ('SIT3088ETK', 'DFN-8',
            'RS-485/RS-422 transceiver — encoder channel C differential data'),
    'U21': ('SIT3088ETK', 'DFN-8',
            'RS-485/RS-422 transceiver — encoder channel D differential data'),
    'U22': ('TS5A3159DCK', 'SC-70-6',
            '1-Ω SPDT analog switch — encoder A+ line ↔ HALL1 alternate'),
    'U23': ('TS5A3159DCK', 'SC-70-6',
            '1-Ω SPDT analog switch — encoder B+ line ↔ HALL3 alternate'),
    'U24': ('TS5A3159DCK', 'SC-70-6',
            '1-Ω SPDT analog switch — encoder C+ line ↔ EXT_ANALOG1 alternate'),
    'U25': ('TS5A3159DCK', 'SC-70-6',
            '1-Ω SPDT analog switch — encoder C− line ↔ EXT_ANALOG2 alternate'),
    'U26': ('TS5A3159DCK', 'SC-70-6',
            '1-Ω SPDT analog switch — encoder A− line ↔ HALL2 alternate'),
    'U27': ('TS5A3159DCK', 'SC-70-6',
            '1-Ω SPDT analog switch — encoder B− line ↔ I2C_SDA alternate'),
    'Q4':  ('PNP small-signal', 'SOT-23',
            'gate turn-off clamp — fast low-side FET gate discharge (/2DGL)'),
}


def positions(sample=None, shipped=False):
    pos = []
    for o in objs:
        if o['kind'] == 'mov':
            if sample is not None:
                pos.append(sample['movers'][o['name']])
            elif shipped:
                pos.append([o['ship_x0'], o['ship_y0']])
            else:
                pos.append([o['x0'], o['y0']])
        else:
            pos.append([o['x0'], o['y0']])
    return pos


def part_title(o):
    g = FP.get(o['name'])
    if not g:
        return o['name']
    t = f"{o['name']} · {g['value']} · {g['lib']}"
    if o['name'] in ICS:
        t += f" — {ICS[o['name']][2]}"
    return t


def fp_group(o, x, y, scale, X, Y, bad, label):
    """One mover drawn as its real footprint at bbox-min (x, y)."""
    g = FP[o['name']]
    cls = 'part' + (' bad' if o['name'] in bad else '')
    s = [f'<g class="{cls}"><title>{html.escape(part_title(o))}</title>']
    s.append(f'<rect x="{X(x):.1f}" y="{Y(y):.1f}" '
             f'width="{o["w"]*scale:.1f}" height="{o["h"]*scale:.1f}" '
             'class="cy"/>')
    for x1, y1, x2, y2 in g['lines']:
        if abs(x2 - x1) < 0.01 and abs(y2 - y1) < 0.01:
            continue
        s.append(f'<line x1="{X(x+x1):.1f}" y1="{Y(y+y1):.1f}" '
                 f'x2="{X(x+x2):.1f}" y2="{Y(y+y2):.1f}" class="fab"/>')
    for cx, cy_, r in g['circles']:
        s.append(f'<circle cx="{X(x+cx):.1f}" cy="{Y(y+cy_):.1f}" '
                 f'r="{r*scale:.1f}" class="fab"/>')
    for p in g['pads']:
        if p['s'] == 'c':
            s.append(f'<circle cx="{X(x+p["x"]):.1f}" cy="{Y(y+p["y"]):.1f}" '
                     f'r="{p["w"]*scale/2:.1f}" class="pad"/>')
        else:
            rx = (f' rx="{min(p["w"], p["h"])*scale/2:.1f}"' if p['s'] == 'o'
                  else f' rx="{min(p["w"], p["h"])*scale/4:.1f}"'
                  if p['s'] == 'rr' else '')
            s.append(f'<rect x="{X(x+p["x"]-p["w"]/2):.1f}" '
                     f'y="{Y(y+p["y"]-p["h"]/2):.1f}" '
                     f'width="{p["w"]*scale:.1f}" height="{p["h"]*scale:.1f}"'
                     f'{rx} class="pad"/>')
    if label and o['w'] * scale > 30 and o['h'] * scale > 13:
        s.append(f'<text x="{X(x+o["w"]/2):.1f}" '
                 f'y="{Y(y+o["h"]/2)+3.5:.1f}" class="lbl">'
                 f'{html.escape(o["name"])}</text>')
    s.append('</g>')
    return ''.join(s)


def panel(pos, scale=20, bad_pairs=(), label=True, edges=True, mode='fp'):
    Wp, Hp = CW * scale, CH * scale
    bad = {n for p in bad_pairs for n in p[:2]}

    def X(x):
        return (x - cx0) * scale

    def Y(y):
        return (y - cy0) * scale
    s = [f'<svg viewBox="0 0 {Wp:.0f} {Hp:.0f}" class="board-svg" '
         f'role="img" aria-label="placement panel ({mode} view)">']
    s.append(f'<rect x="0" y="0" width="{Wp:.0f}" height="{Hp:.0f}" '
             'class="cnv"/>')
    for i, o in enumerate(objs):
        if o['kind'] != 'fix':
            continue
        x, y = pos[i]
        s.append(f'<rect x="{X(x):.1f}" y="{Y(y):.1f}" '
                 f'width="{o["w"]*scale:.1f}" height="{o["h"]*scale:.1f}" '
                 'class="fix"/>')
    if edges:
        for n in nets:
            for u, ux, uy, v, vx, vy in n['edges']:
                s.append(f'<line x1="{X(pos[u][0]+ux):.1f}" '
                         f'y1="{Y(pos[u][1]+uy):.1f}" '
                         f'x2="{X(pos[v][0]+vx):.1f}" '
                         f'y2="{Y(pos[v][1]+vy):.1f}" class="net"/>')
    for i, o in enumerate(objs):
        if o['kind'] == 'port':
            x, y = pos[i]
            s.append(f'<circle cx="{X(x+o["w"]/2):.1f}" '
                     f'cy="{Y(y+o["h"]/2):.1f}" r="3" class="port"/>')
    for i, o in enumerate(objs):
        if o['kind'] != 'mov':
            continue
        x, y = pos[i]
        if mode == 'fp':
            s.append(fp_group(o, x, y, scale, X, Y, bad, label))
            continue
        cls = 'mov' + (' bad' if o['name'] in bad else '') \
            + (' ic' if o['name'] in ICS else '')
        s.append(f'<g class="part"><title>{html.escape(part_title(o))}</title>'
                 f'<rect x="{X(x):.1f}" y="{Y(y):.1f}" '
                 f'width="{o["w"]*scale:.1f}" height="{o["h"]*scale:.1f}" '
                 f'class="{cls}"/>')
        if label and o['w'] * scale > 30 and o['h'] * scale > 13:
            lcls = 'lbl ic' if o['name'] in ICS else 'lbl'
            s.append(f'<text x="{X(x+o["w"]/2):.1f}" '
                     f'y="{Y(y+o["h"]/2)+3.5:.1f}" class="{lcls}">'
                     f'{html.escape(o["name"])}</text>')
        s.append('</g>')
    s.append('</svg>')
    return ''.join(s)


def both_views(pos, **kw):
    return (f'<div class="vw fpv">{panel(pos, mode="fp", **kw)}</div>'
            f'<div class="vw desv">{panel(pos, mode="des", **kw)}</div>')


tgt = R['anchors']['target']
shipm = R['anchors']['shipped']
samples = R['samples']
legal = [s for s in samples if s['overlap_max'] <= 0.01 and s['oob'] <= 0.01]
best = min(legal or samples, key=lambda s: s['hpwl'])
n_beat = sum(1 for s in (legal or []) if s['hpwl'] < tgt['hpwl'])

rows = []
for s in samples:
    ok = s['overlap_max'] <= 0.01 and s['oob'] <= 0.01
    tag = ('<span class="sev good">clean</span>' if ok else
           f'<span class="sev med">{len(s["pairs"])} overlaps</span>')
    star = ' ★' if s is best else ''
    rows.append(
        f'<tr{" class=best" if s is best else ""}>'
        f'<td>s{s["idx"]:02d}{star}</td>'
        f'<td>{s["hpwl"]:.1f}</td><td>{s["hpwl"]/tgt["hpwl"]:.3f}×</td>'
        f'<td>{tag}</td><td>{s["overlap_max"]:.3f}</td>'
        f'<td>{s["travel_ship"]["mean"]:.2f}</td>'
        f'<td>{s["travel_target"]["mean"]:.2f}</td></tr>')

thumbs = []
for s in samples:
    cls = 'thumb' + (' clean' if s['overlap_max'] <= 0.01 and s['oob'] <= 0.01
                     else '')
    thumbs.append(
        f'<figure class="{cls}">'
        + both_views(positions(sample=s), scale=6.5, bad_pairs=s['pairs'],
                     label=False, edges=False)
        + f'<figcaption>s{s["idx"]:02d} · {s["hpwl"]/tgt["hpwl"]:.2f}× · '
          f'{len(s["pairs"])} ov</figcaption></figure>')

ic_rows = ''.join(
    f'<tr><td><code>{r}</code></td><td>{html.escape(part)}</td>'
    f'<td>{pkg}</td><td>{html.escape(role)}</td></tr>'
    for r, (part, pkg, role) in ICS.items())

page = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Diffusion Placement Counterpoint — RP2350 Motor Controller</title>
<style>
  :root {{
    --bg: #f7f6f3; --panel: #ffffff; --ink: #1e2126; --muted: #5b6470;
    --line: #d9d5cc; --accent: #6b4fa0; --good: #2c7a4b; --warn: #a05a00;
    --bad: #b03434; --code-bg: #efede8; --copper: #b0783c;
  }}
  @media (prefers-color-scheme: dark) {{
    :root {{
      --bg: #16181d; --panel: #1e2128; --ink: #e8e6e1; --muted: #9aa3af;
      --line: #343945; --accent: #a98fd8; --good: #5cb884; --warn: #e0a44c;
      --bad: #e06c6c; --code-bg: #262a33; --copper: #d09a55;
    }}
  }}
  * {{ box-sizing: border-box; }}
  html, body {{ margin: 0; padding: 0; }}
  body {{ background: var(--bg); color: var(--ink);
         font: 16px/1.6 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; }}
  main {{ max-width: 1100px; margin: 0 auto; padding: 2rem 1.25rem 5rem; }}
  header.masthead {{ border-bottom: 3px solid var(--accent);
                     padding: 2.2rem 0 1rem; margin-bottom: 1.5rem; }}
  h1 {{ font-size: 1.7rem; line-height: 1.2; margin: 0 0 .4rem; }}
  .subtitle {{ color: var(--muted); margin: 0; }}
  h2 {{ font-size: 1.3rem; margin: 2.2rem 0 .8rem; }}
  a {{ color: var(--accent); }}
  code {{ font-family: ui-monospace, "SF Mono", Consolas, monospace;
         font-size: .92em; background: var(--code-bg); padding: .08em .35em;
         border-radius: 4px; }}
  .sev {{ display: inline-block; font-size: .72rem; font-weight: 700;
         letter-spacing: .05em; padding: .12em .55em; border-radius: 99px;
         white-space: nowrap; }}
  .sev.med {{ background: color-mix(in srgb, var(--warn) 18%, transparent);
             color: var(--warn); }}
  .sev.good {{ background: color-mix(in srgb, var(--good) 16%, transparent);
              color: var(--good); }}
  .viewbar {{ display: flex; align-items: center; gap: .5rem; flex-wrap: wrap;
             margin: 1rem 0 0; }}
  .vt {{ font: inherit; font-size: .85rem; padding: .25rem .9rem;
        border-radius: 99px; border: 1px solid var(--line);
        background: var(--panel); color: var(--muted); cursor: pointer; }}
  .vt.on {{ border-color: var(--accent); color: var(--accent);
           font-weight: 600; }}
  .viewbar .hint {{ font-size: .8rem; color: var(--muted); }}
  .desv {{ display: none; }}
  body[data-view="des"] .desv {{ display: block; }}
  body[data-view="des"] .fpv {{ display: none; }}
  body:not([data-view="des"]) .desonly {{ display: none; }}
  .panels {{ display: grid; grid-template-columns: repeat(3, 1fr); gap: 1rem;
            margin: 1.4rem 0; }}
  @media (max-width: 860px) {{ .panels {{ grid-template-columns: 1fr; }} }}
  .panels figure, .thumb {{ margin: 0; background: var(--panel);
    border: 1px solid var(--line); border-radius: 12px; padding: .8rem; }}
  figcaption {{ font-size: .85rem; color: var(--muted); margin-top: .5rem;
               text-align: center; }}
  .board-svg {{ width: 100%; height: auto; display: block; }}
  .cnv {{ fill: var(--code-bg); stroke: var(--line); }}
  .fix {{ fill: color-mix(in srgb, var(--muted) 28%, transparent);
         stroke: var(--muted); stroke-width: 1; }}
  .mov {{ fill: color-mix(in srgb, var(--accent) 30%, transparent);
         stroke: var(--accent); stroke-width: 1.5; }}
  .mov.ic {{ fill: color-mix(in srgb, var(--accent) 55%, transparent); }}
  .mov.bad {{ fill: color-mix(in srgb, var(--bad) 35%, transparent);
             stroke: var(--bad); }}
  .cy {{ fill: color-mix(in srgb, var(--accent) 8%, transparent);
        stroke: color-mix(in srgb, var(--accent) 45%, transparent);
        stroke-width: .7; stroke-dasharray: 3 2; }}
  .fab {{ stroke: var(--ink); opacity: .5; stroke-width: 1; fill: none;
         stroke-linecap: round; }}
  .pad {{ fill: var(--copper); }}
  g.bad .cy {{ stroke: var(--bad);
              fill: color-mix(in srgb, var(--bad) 14%, transparent); }}
  g.bad .fab {{ stroke: var(--bad); opacity: .8; }}
  g.bad .pad {{ fill: var(--bad); }}
  .net {{ stroke: color-mix(in srgb, var(--accent) 45%, transparent);
         stroke-width: .8; }}
  .port {{ fill: var(--warn); }}
  .lbl {{ font: 600 9.5px ui-monospace, monospace; fill: var(--ink);
         text-anchor: middle; }}
  .lbl.ic {{ font-weight: 800; }}
  table {{ border-collapse: collapse; width: 100%; font-size: .9rem;
          background: var(--panel); border: 1px solid var(--line);
          border-radius: 12px; }}
  th, td {{ text-align: left; padding: .45rem .8rem;
            border-bottom: 1px solid var(--line); }}
  th {{ color: var(--muted); font-weight: 600; }}
  tr.best td {{ background: color-mix(in srgb, var(--good) 9%, transparent); }}
  .thumbs {{ display: grid; grid-template-columns: repeat(4, 1fr); gap: .8rem;
            margin: 1.4rem 0; }}
  @media (max-width: 860px) {{ .thumbs {{ grid-template-columns: repeat(2, 1fr); }} }}
  .thumb figcaption {{ margin-top: .3rem; }}
  .thumb.clean {{ border-color: var(--good); }}
  .note {{ background: var(--panel); border: 1px solid var(--line);
          border-radius: 12px; padding: 1rem 1.3rem; margin: 1.2rem 0;
          font-size: .93rem; }}
  .note.desonly h3 {{ margin: .1rem 0 .6rem; font-size: 1rem; }}
  .note.desonly table {{ border: none; }}
  footer {{ margin-top: 3rem; color: var(--muted); font-size: .85rem;
           border-top: 1px solid var(--line); padding-top: 1rem; }}
</style>
</head>
<body data-view="fp">
<main>

<header class="masthead">
  <h1>Diffusion Placement Counterpoint</h1>
  <p class="subtitle">Zero-shot fresh placement of the 49 morph parts by a
  pretrained chip-placement diffusion model
  (<a href="research/diffusion-placement-2024.pdf">Lee et&nbsp;al., ICML 2025
  / arXiv 2407.12282</a>) · counterpoint to the
  <a href="creep.html">jiggle2 creep morph</a> · <a href="index.html">hub</a></p>
</header>

<p>The creep stepper morphs the shipped rev-A placement continuously toward
the rev-B targets and stalls at mean&nbsp;u&nbsp;≈&nbsp;0.435 against a
topological wall. This experiment asks the opposite question: forget
continuity — if a generative placer lays these parts out <em>fresh</em> in
the same region, with the same fixed context, how does its answer compare
with the hand-designed rev-B target placement?</p>

<p>Setup: the authors' released <em>Large+v2</em> checkpoint (6.3M params,
trained purely on synthetic netlists), run zero-shot with their
universal-guidance sampling (legality + HPWL potentials, Lagrangian
weighting). The morph region becomes a {CW:.1f}&nbsp;×&nbsp;{CH:.1f}&nbsp;mm
canvas with {sum(1 for o in objs if o['kind']=='mov')} movable parts,
{sum(1 for o in objs if o['kind']=='fix')} fixed blockers (back-side
courtyards; THT connectors block only at their pins — the rev-B design
itself tucks parts under the J2/J12 bodies), and
{sum(1 for o in objs if o['kind']=='port')} pinned net terminals.
GND is exempt, exactly like the morph's connectivity rule.</p>

<h2>Rev-A → Rev-B → best sample</h2>
<div class="viewbar" role="tablist" aria-label="panel view">
  <button class="vt on" data-view="fp">footprints</button>
  <button class="vt" data-view="des">designators</button>
  <span class="hint">footprints: real pad + fab-outline geometry (back-side
  parts seen through the board, shipped orientation; dashed = courtyard) ·
  designators: courtyard boxes + refs, ICs highlighted — hover any part for
  its description</span>
</div>
<div class="panels">
  <figure>{both_views(positions(shipped=True))}
  <figcaption>rev-A shipped — where the parts are today
  (HPWL {shipm['hpwl']:.0f}&nbsp;mm)</figcaption></figure>
  <figure>{both_views(positions())}
  <figcaption>rev-B target — hand-designed goal the morph walks toward
  (HPWL {tgt['hpwl']:.0f}&nbsp;mm)</figcaption></figure>
  <figure>{both_views(positions(sample=best), bad_pairs=best['pairs'])}
  <figcaption>diffusion s{best['idx']:02d} — best sample
  (HPWL {best['hpwl']:.0f}&nbsp;mm,
  {best['hpwl']/tgt['hpwl']:.2f}× target,
  {len(best['pairs'])} overlaps)</figcaption></figure>
</div>

<div class="note desonly">
<h3>The ICs (descriptors by hand; passives not annotated)</h3>
<table>
<tr><th>ref</th><th>part</th><th>package</th><th>role</th></tr>
{ic_rows}
</table>
</div>

<h2>All {len(samples)} samples</h2>
<table>
<tr><th>sample</th><th>HPWL (mm)</th><th>vs target</th><th>legality</th>
<th>worst overlap (mm)</th><th>travel vs rev-A (mm)</th>
<th>vs rev-B (mm)</th></tr>
{''.join(rows)}
</table>

<div class="thumbs">
{''.join(thumbs)}
</div>

<h2>Reading</h2>
<div class="note" id="reading">
<p><strong>The region has materially shorter-wire placements than rev-B, and
the model finds them zero-shot.</strong> All {len(samples)} raw samples beat
the rev-B target on wirelength
({min(s['hpwl'] for s in samples)/tgt['hpwl']:.2f}–{max(s['hpwl'] for s in samples)/tgt['hpwl']:.2f}×),
and s{best['idx']:02d} does it courtyard-clean under the same rectangle
model that scores the rev-B target exactly legal — an
{100*(1-best['hpwl']/tgt['hpwl']):.0f}% HPWL reduction with zero overlap.
Most other samples carry a handful of residual overlaps (worst
≈1&nbsp;mm); that is expected for raw guided samples — the authors run a
separate legalization pass for their benchmark numbers, which this
experiment deliberately skips to see the model's own output.</p>
<p><strong>These are different layouts, not perturbations.</strong> Mean
part travel from rev-A shipped is
{min(s['travel_ship']['mean'] for s in samples):.0f}–{max(s['travel_ship']['mean'] for s in samples):.0f}&nbsp;mm
— the model rearranges the region wholesale. Note the anchors' own numbers:
rev-B target HPWL ({tgt['hpwl']:.0f}) is essentially rev-A's
({shipm['hpwl']:.0f}) — the rev-B edit inserts the termination switches
with minimal disruption; it never tried to shorten wire. The diffusion
answer sits in a different basin entirely, which is precisely the kind of
placement the creep morph <em>cannot</em> reach: creep is
homotopy-constrained by construction, and its s&nbsp;=&nbsp;0.435 wall is a
property of the path to <em>this</em> target, not of the region's
capacity.</p>
<p><strong>Caveats before falling in love.</strong> HPWL over a star
netlist (GND exempt) is a placement proxy, not routability: two copper
layers, pours, the pin-protrusion realities under J2/J12, and the encoder
differential pairs all live outside the model. No rotation either — every
part keeps its shipped orientation. The honest use of s{best['idx']:02d} is
as a <em>counterpoint existence proof</em> (the wall is about the target,
not the territory) and possibly as a seed for a rev-C hand layout — not as
a drop-in placement.</p>
</div>

<h2>Method notes</h2>
<div class="note">
<ul>
<li>Objects are courtyard bounding boxes; pins are pad centres, star topology
per net from the driving terminal (external nets terminate in a virtual port
clamped to the canvas edge, partitioning-style).</li>
<li>Fixed context is pinned via the model's port mask (inpainting-style: those
coordinates never denoise), but a one-line patch passes a separate
<code>legality_mask</code> to the guidance potential so fixed blockers still
repel movers — upstream excludes all pinned objects from legality.</li>
<li>Pad blockers are grandfathered by shipping: where the rev-B design places
a courtyard inside a THT pad's halo, the blocker shrinks to fit — the
reference placement scores exactly legal by construction.</li>
<li>The footprint view is cosmetic: pads and fab/silk body outlines are read
from the board (<code>footgeom.py</code>) and drawn at each placement in
shipped orientation. The model itself only ever saw the courtyard
rectangles.</li>
<li>Not modelled: routing/copper (the morph's entire difficulty), rotation
(the model only translates), pours, and same-net contact persistence.
A clean sample here is a placement-level existence proof, not a routed
board.</li>
</ul>
</div>

<footer>rp2350-motor-controller review · diffusion placement experiment ·
generated by hardware/tools/diffusion/render_page.py</footer>
</main>
<script>
for (const b of document.querySelectorAll('.vt'))
  b.addEventListener('click', () => {{
    document.body.dataset.view = b.dataset.view;
    for (const x of document.querySelectorAll('.vt'))
      x.classList.toggle('on', x.dataset.view === b.dataset.view);
  }});
</script>
</body>
</html>
"""

out = os.path.join(REVIEW, 'diffusion.html')
with open(out, 'w') as f:
    f.write(page)
print(f'{len(samples)} samples, best s{best["idx"]:02d} '
      f'({best["hpwl"]/tgt["hpwl"]:.3f}x target, {len(best["pairs"])} ov), '
      f'{len(legal)} clean, {n_beat} clean+beat-target -> {out}')
