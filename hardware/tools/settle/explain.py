#!/usr/bin/env python3
"""settle: the narrated page — review/settle_explained.html.

Builds figures from the recorded runs (model.json, frames.json,
settle_report.json, gate.json, settle.log) and writes a step-by-step
explanation of the algorithm with the videos from animate.py embedded.

    python3 explain.py            (uses configs enc_corner_wide, enc_corner, power_squeeze)
"""
import os, re, json, math, html
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from common import HERE, HW
from viz import state_from_frame, state0, draw_state, legend_handles, LCOL, C_GHOST, fix_chains0

REPO = os.path.dirname(HW)
REVIEW = os.path.join(REPO, 'review')
IMG = os.path.join(REVIEW, 'img')
os.makedirs(IMG, exist_ok=True)
CFG = os.path.join(HERE, 'configs')


def load_run(name):
    d = os.path.join(CFG, 'data', name)

    def j(fn):
        p = os.path.join(d, fn)
        return json.load(open(p)) if os.path.exists(p) else None
    log = open(os.path.join(d, 'settle.log')).read() if os.path.exists(os.path.join(d, 'settle.log')) else ''
    FR, TR = j('frames.json'), j('traj.json')
    if FR:
        fix_chains0(FR, TR)
    return dict(name=name, M=j('model.json'), FR=FR, REP=j('settle_report.json') or {},
                GATE=j('gate.json') or {}, TR=TR, log=log,
                cfg=json.load(open(os.path.join(CFG, name + '.json'))))

W = load_run('enc_corner_wide')
N = load_run('enc_corner')
S = load_run('power_squeeze')
figs = {}


def save(fig, key):
    p = os.path.join(IMG, f'explain_{key}.png')
    fig.tight_layout()
    fig.savefig(p, dpi=130)
    plt.close(fig)
    figs[key] = os.path.relpath(p, REVIEW)

# ---- F1: the model -----------------------------------------------------------
M = W['M']
fig, ax = plt.subplots(figsize=(7.5, 12))
draw_state(ax, M, state0(M), title='enc_corner_wide: the model at cycle 0', show_press=False, show_blocked=False)
for gh in M['ghosts']:
    ax.annotate(gh['ref'], gh['at'], fontsize=7, color=C_GHOST, xytext=(5, -8), textcoords='offset points')
ax.legend(handles=legend_handles(), loc='upper left', fontsize=6, framealpha=0.9)
save(fig, 'model')

# ---- F2: site search ---------------------------------------------------------
ras = M.get('rasters', {}).get('B')
sel = [gh for gh in M['ghosts'] if gh.get('search')]
sel = sorted(sel, key=lambda gh: -gh['search']['dist'])[:2]
if ras and sel:
    fig, axes = plt.subplots(1, len(sel), figsize=(6.5 * len(sel), 8))
    axes = np.atleast_1d(axes)
    data = np.array(ras['data']).reshape(ras['H'], ras['W'])
    ext = [ras['x0'], ras['x0'] + ras['W'] * ras['step'], ras['y0'] + ras['H'] * ras['step'], ras['y0']]
    for ax, gh in zip(axes, sel):
        ax.imshow(data, extent=ext, cmap='Greys', vmin=0, vmax=1, interpolation='nearest')
        sr = gh['search']
        cands = np.array(sr['cands'])
        ax.scatter(cands[:, 0], cands[:, 1], c=cands[:, 2], cmap='RdYlGn_r', s=4, vmin=0, vmax=1, alpha=0.8)
        ax.add_patch(Circle(sr['centre'], sr['radius'], fill=False, ec='#1a5276', lw=1, ls='--'))
        pp = np.array(gh['partner_pads'])
        ax.scatter(pp[:, 0], pp[:, 1], marker='o', s=40, fc='none', ec='#1a5276', lw=1.5, label='partner pads')
        ax.plot([sr['centre'][0]], [sr['centre'][1]], marker='+', color='#1a5276', ms=10, label='partner centroid')
        ax.plot([gh['at'][0]], [gh['at'][1]], marker='*', color=C_GHOST, ms=14, label=f"chosen site, obstruction {sr['obstruction']:.2f}")
        ax.set_title(f"{gh['ref']}: least-obstructed courtyard box within {sr['radius']} mm", fontsize=9)
        ax.set_xlim(ext[0], ext[1]); ax.set_ylim(ext[2], ext[3]); ax.set_aspect('equal')
        ax.legend(fontsize=7, loc='lower right')
        ax.tick_params(labelsize=7)
    save(fig, 'sites')

# ---- F3: margin census -------------------------------------------------------
m = re.search(r'margin census over rule \(20um bins, <0 first\): (.*)', W['log'])
if m:
    bins = [(int(k), int(v)) for k, v in re.findall(r'([+-]\d+):(\d+)', m.group(1))]
    fig, ax = plt.subplots(figsize=(8, 3))
    xs = [b for b, _ in bins]
    ax.bar([f'{b:+d}' if b < 180 else '≥180' for b in xs], [v for _, v in bins], color='#2471a3')
    ax.set_yscale('log')
    ax.set_xlabel('margin over the rule, µm (20 µm bins)'); ax.set_ylabel('pairs')
    ax.set_title('Shipped margins of every pair the movable copper can meet (enc_corner_wide)', fontsize=9)
    gm = re.search(r'grandfathered (\d+) shipped at/below-guard pairs \((\d+) below model rule\), (\d+) same-net contacts', W['log'])
    if gm:
        ax.text(0.98, 0.9, f'{gm.group(1)} pairs below the 20 µm cut are grandfathered at their shipped margin\n'
                           f'{gm.group(2)} of them below the model rule (polygon bias) · {gm.group(3)} same-net contacts held as attachments',
                transform=ax.transAxes, ha='right', va='top', fontsize=7)
    save(fig, 'census')

# ---- F4: line search traces --------------------------------------------------
traces = [t for run in (W, N) for t in (run['FR'] or {}).get('ls_traces', [])]
traces = [t for t in traces if t['pairs']]
if traces:
    # prefer a partial step (0 < alpha* < 1): it shows the bisection at work
    partial = [t for t in traces if 0 < t['alpha_star'] < 1]
    tr = max(partial or traces, key=lambda t: len(t['pairs']))
    fig, ax = plt.subplots(figsize=(8, 4))
    for p in tr['pairs'][:6]:
        ys = [v if v is not None else np.nan for v in p['margins']]
        lab = f"{p['kind']} {p.get('ref') or p.get('net') or ''} (floor {p['floor'] * 1000:+.0f} µm)"
        ax.plot(tr['alphas'], [y * 1000 for y in ys], marker='o', ms=3, label=lab)
        ax.axhline(p['floor'] * 1000, color='grey', lw=0.5, ls=':')
    ax.axvline(tr['alpha_star'], color='black', ls='--', lw=1, label=f"accepted fraction α* = {tr['alpha_star']}")
    ax.set_xlabel('fraction α of the proposed growth step'); ax.set_ylabel('pair margin, µm')
    ax.set_title(f"Line search: ghost {tr['ghost']} at scale {tr['scale']} tries to grow one step (cycle {tr['cycle']})", fontsize=9)
    ax.legend(fontsize=6, loc='best')
    save(fig, 'linesearch')

# ---- F5: inflation stills + F6: pressure frame -------------------------------


def frame_states(run):
    FR = run['FR']
    return [state_from_frame(run['M'], FR, fr) for fr in FR['frames']] if FR else []

WS = frame_states(W)
if WS:
    M = W['M']
    # the ghost that took the longest to be born
    born_at = []
    for gi, gh in enumerate(M['ghosts']):
        k = next((i for i, st in enumerate(WS) if st['G'][gi] >= 1.0), None)
        born_at.append((k if k is not None else len(WS), gi))
    kb, gi = max(born_at)
    gh = M['ghosts'][gi]
    cx, cy = gh['at']
    win = [cx - 4, cy - 4, cx + 4, cy + 4]
    picks = sorted({0, kb // 3, 2 * kb // 3, min(kb, len(WS) - 1)})
    fig, axes = plt.subplots(1, len(picks), figsize=(4.2 * len(picks), 4.6))
    for ax, k in zip(np.atleast_1d(axes), picks):
        st = WS[k]
        draw_state(ax, M, st, window=win, title=f"cycle {st['cycle']} · {gh['ref']} scale {st['G'][gi]:.2f}",
                   label_ghosts=False, lw_scale=4)
    save(fig, 'inflate')
    W['inflate_ref'] = gh['ref']
    kmax = max(range(len(WS)), key=lambda k: sum(math.hypot(*v) for v in WS[k]['press'].values()))
    st = WS[kmax]
    fig, ax = plt.subplots(figsize=(7.5, 12))
    draw_state(ax, M, st, title=f"cycle {st['cycle']}: pressure deposited this cycle "
                                f"({len(st['press'])} parts, {len(st['npress'])} nodes)")
    save(fig, 'pressure')

# ---- F7: springs -------------------------------------------------------------
if WS:
    M = W['M']
    nets = M['nets']
    RF = W['FR']['frames']
    L0 = RF[0]['net_len']
    grown = sorted(((n, RF[-1]['net_len'][n] - L0[n]) for n in L0 if n in RF[-1]['net_len']), key=lambda kv: -kv[1])[:6]
    fig, ax = plt.subplots(figsize=(8, 4))
    for n, _ in grown:
        ys = [fr['net_len'].get(n, L0[n]) - L0[n] for fr in RF]
        xs = [fr['cycle'] for fr in RF]
        el = nets.get(n, {})
        line, = ax.plot(xs, ys, label=n)
        static = W['REP'].get('lengths', [])
        row = next((r for r in static if r['net'] == n), None)
        if row and row.get('allow_mm') is not None:
            ax.axhline(row['allow_mm'], color=line.get_color(), lw=0.6, ls=':')
        if row and row.get('cap_mm') is not None and row['cap_mm'] < 8:
            ax.axhline(row['cap_mm'], color=line.get_color(), lw=0.6, ls='--')
    ax.set_xlabel('cycle'); ax.set_ylabel('copper grown inside the region, mm')
    ax.set_title('Springs: the six nets that grew most (dotted = allowance, dashed = hard cap)', fontsize=9)
    ax.legend(fontsize=7)
    save(fig, 'springs')

# ---- F8: reroute before/after (narrow run) -----------------------------------
NS = frame_states(N)
if NS:
    M = N['M']
    kchg = next((k for k in range(1, len(NS)) if NS[k]['chains'] is not NS[k - 1]['chains'] and
                 N['FR']['frames'][k]['cver'] != N['FR']['frames'][k - 1]['cver']), None)
    if kchg is not None:
        a, b = NS[kchg - 1], NS[kchg]
        changed = [ei for ei, (ca, cb) in enumerate(zip(a['chains'], b['chains'])) if ca != cb]
        pts = [b['P'][i] for ei in changed for i in b['chains'][ei]] + [a['P'][i] for ei in changed for i in a['chains'][ei]]
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
        w = max(6.0, max(max(xs) - min(xs), max(ys) - min(ys)) + 2)
        win = [cx - w / 2, cy - w / 2, cx + w / 2, cy + w / 2]
        fig, axes = plt.subplots(1, 2, figsize=(11, 6))
        for ax, st, ttl in zip(axes, (a, b), ('before', 'after')):
            draw_state(ax, M, st, window=win, title=f"reroute round: {ttl} (cycle {st['cycle']}, {len(changed)} chains re-planned)",
                       label_ghosts=False, lw_scale=4)
            for ei in changed:
                ids = st['chains'][ei]
                ax.plot([st['P'][i][0] for i in ids], [st['P'][i][1] for i in ids], color='magenta', lw=1.2, alpha=0.9, zorder=10)
        save(fig, 'reroute')
        N['reroute_n'] = len(changed)

# ---- F9: gate ----------------------------------------------------------------
fig, axes = plt.subplots(1, 3, figsize=(12, 2.8))
for ax, run in zip(axes, (W, N, S)):
    steps = run['GATE'].get('steps', [])
    if not steps:
        continue
    ax.plot([e['s'] for e in steps], [e.get('unconnected', 0) for e in steps], marker='o', ms=3, label='unconnected')
    ax.plot([e['s'] for e in steps], [e.get('new_violations', 0) for e in steps], marker='s', ms=3, label='new DRC violations')
    ax.plot([e['s'] for e in steps], [e.get('static_missing', 0) for e in steps], marker='^', ms=3, label='static copper missing')
    ax.set_title(f"{run['name']}: {sum(1 for e in steps if e['ok'])}/{len(steps)} checkpoints pass", fontsize=9)
    ax.set_xlabel('progress s'); ax.legend(fontsize=6); ax.tick_params(labelsize=7)
save(fig, 'gate')

# ---- F10: squeeze ------------------------------------------------------------
SS = frame_states(S)
if SS:
    M = S['M']
    fig, axes = plt.subplots(1, 2, figsize=(11, 12))
    draw_state(axes[0], M, SS[0], title='power_squeeze: cycle 0', show_press=False, show_blocked=False)
    draw_state(axes[1], M, SS[-1], title=f"power_squeeze: cycle {SS[-1]['cycle']}, progress {SS[-1]['s']:.2f}", show_press=False)
    save(fig, 'squeeze')
    rows = [r for r in S['REP'].get('lengths', []) if r.get('at_cap')]
    if rows:
        fig, ax = plt.subplots(figsize=(7, 2.8))
        ax.barh([r['net'] for r in rows], [r['grown_mm'] for r in rows], color='#c0392b', label='grown')
        ax.barh([r['net'] for r in rows], [r['cap_mm'] for r in rows], fill=False, ec='black', label='hard cap')
        ax.set_xlabel('mm'); ax.legend(fontsize=7); ax.set_title('Gate and bootstrap nets that reached their cap and stopped the squeeze', fontsize=9)
        ax.tick_params(labelsize=7)
        save(fig, 'squeeze_caps')

# ---- page --------------------------------------------------------------------
E = html.escape


def img(key, cap=''):
    if key not in figs:
        return ''
    return f'<figure><img src="{E(figs[key])}"><figcaption>{cap}</figcaption></figure>'


def video(fn, cap=''):
    p = os.path.join(IMG, fn)
    if not os.path.exists(p):
        return f'<p class=small><i>video {E(fn)} not rendered</i></p>'
    return (f'<figure><video controls muted loop playsinline preload="metadata" src="img/{E(fn)}"></video>'
            f'<figcaption>{cap}</figcaption></figure>')


def verdict(run):
    r, g = run['REP'], run['GATE']
    if not r:
        return ''
    born = sum(1 for v in r['ghosts'].values() if v['g'] >= 1) if r['ghosts'] else None
    ok = sum(1 for e in g.get('steps', []) if e['ok'])
    return (f"{run['name']}: {r['cycles']} cycles, progress {r['s']}, " +
            (f"{born}/{len(r['ghosts'])} ghosts born, " if born is not None else '') +
            f"{len(r['parts_moved'])} parts moved, gate {ok}/{len(g.get('steps', []))}, check_sync {g.get('check_sync')}")

el = W['cfg'].get('elastic', {})
el_rows = ''.join(f'<tr><td><code>{E(k)}</code></td><td>{E(json.dumps(v))}</td></tr>' for k, v in el.get('nets', {}).items())
inflate_ref = W.get('inflate_ref', 'U33')
zoom_video = f'settle_enc_corner_wide_zoom_{inflate_ref}.mp4'
if not os.path.exists(os.path.join(IMG, zoom_video)):
    zoom_video = next((f for f in os.listdir(IMG) if f.startswith('settle_enc_corner_wide_zoom_')), zoom_video)

page = f"""<!doctype html><meta charset=utf-8><title>settle, explained</title>
<style>
body{{font:15px/1.55 system-ui,sans-serif;max-width:1100px;margin:1.5em auto;padding:0 1em;color:#222}}
h1{{font-size:1.7em}}h2{{margin-top:2em;border-bottom:1px solid #ddd;padding-bottom:.2em}}h3{{margin-top:1.4em}}
figure{{margin:1em 0}}figcaption{{font-size:12.5px;color:#555;margin-top:.3em}}img,video{{max-width:100%;border:1px solid #eee}}
table{{border-collapse:collapse;font-size:12.5px;margin:.6em 0}}td,th{{border:1px solid #ccc;padding:3px 7px;text-align:left;vertical-align:top}}th{{background:#f3f3f3}}
code{{background:#f4f4f4;padding:0 3px}}.small{{color:#666;font-size:12.5px}}.verdict{{background:#f7f7f7;padding:.6em 1em;border-left:4px solid #c0392b}}
.math{{font-family:Georgia,serif;font-style:italic}}
</style>
<h1>The settle engine, step by step</h1>
<p>The question this tool answers: <b>rev-B needs parts that rev-A's copper has no room for. Where can they go without
tearing up copper that already shipped, and what has to move to make room?</b> The engine treats the board like a bag of chips
being settled: every part, trace node, via and unborn part is a unit that moves only as far as the design rules allow, pressure
propagates through the packing, trace lengths act as springs with hard caps, and the one gate that must always pass is real
KiCad DRC on every board the engine emits.</p>
<p class=verdict>{E(verdict(W))}<br>{E(verdict(N))}<br>{E(verdict(S))}</p>
<p class=small>Branch <code>lean-jiggle</code>, rules 0.10/0.10, region of the encoder corner (left 20 mm of the board) with the F side and the
connectors locked. Code: <code>hardware/tools/settle/</code>. Per-run detail pages: <a href="settle_enc_corner_wide.html">enc_corner_wide</a>,
<a href="settle_enc_corner.html">enc_corner</a>, <a href="settle_power_squeeze.html">power_squeeze</a>.</p>

<h2>1. The whole run in one video</h2>
{video('settle_enc_corner_wide.mp4', 'enc_corner_wide, one frame per cycle. Red: the seven unborn parts inflating at their sites. '
       'Orange pads: movable parts (dark orange once moved). Grey: fixed parts and static copper. Blue arrows: pressure deposited on a part that cycle; '
       'blue dots: copper nodes under pressure; black x: the clamp that stopped a unit. Green arrows: net displacement so far.')}
{video(zoom_video, f'The same run, 8 mm window around {E(inflate_ref)}: the ghost grows step by step, the copper under it is pushed out of the way, '
       'a neighbouring part yields when its own room allows, and the ghost reaches full size.')}

<h2>2. The model: what moves, what does not</h2>
<p><code>prep.py</code> reads the board and one JSON config and builds a region-scoped model. A part is <b>movable</b> when its pad centroid
is inside the region and it is not locked (by reference, side, or rectangle). Every track or via whose segment intersects the region
and whose net is not locked becomes <b>dynamic copper</b>: a graph of nodes (track endpoints and vias) and edges (one per original segment).
An endpoint that lies on a pad is <b>bound</b> to that pad: if the part is movable the node rides with it, otherwise it is fixed.
An endpoint that meets copper outside the region, or copper that stays static, is <b>pinned</b> so a junction can never open.
Everything else within a 3 mm halo of the region is a <b>static obstacle</b>: pads at their true polygons with local clearances, tracks, vias.</p>
{img('model', 'The enc_corner_wide model at cycle 0. Each red cross is a site where an unplaced part will be born.')}
<p>The parts to be born are <b>ghosts</b>: the real footprint placed at its target pose, its pads and courtyard scaled about the pad
centroid by a factor <span class=math>g</span> that starts at 0.05 and must reach 1. A ghost's pads carry no net while it inflates,
so every pair it forms is a clearance pair; nothing may be inside it when it is full size.</p>

<h3>2a. Where to seed a ghost</h3>
<p>The site matters more than any amount of force. With <code>at: "search"</code> the model rasterizes the ghost's side at 0.1 mm
(pads and courtyards weight 1, copper that can be pushed 0.3), then scores every candidate centre within a radius of the partner-pad
centroid by the obstruction under the footprint's courtyard box plus 0.03 per mm of distance, and takes the minimum. Placed ghosts are
stamped into the raster so later ghosts avoid them. At 8 mm radius four of the seven parts were seeded into the dense switch fabric and
never got out; at 14 mm all seven found pockets.</p>
{img('sites', 'Site search for the two parts that travelled furthest. Grey: obstruction raster. Coloured dots: candidate scores (green = free). '
     'Dashed circle: search radius around the partner centroid (+). Star: the chosen site.')}

<h2>3. Admissibility: the DRC geometry as a set of margins</h2>
<p>Every pair of objects that could interact has a <b>margin</b>: its distance minus the clearance the rules demand. Track-track,
track-via, track-pad, via-via, pad-pad, hole-to-copper and hole-to-hole pairs use true pad polygons, netclass clearances with max semantics,
per-pad local overrides, the board outline (centreline plus the corner arcs) and courtyards. Pad polygons are keyed per edge, because within one
straight edge the model is exact and the polygonization bias only jumps between edges.</p>
<p>A move is admissible when every margin stays above its <b>floor</b>. The floor is the rule plus a 15 µm guard for pairs that had room on the
shipped board, and the <em>shipped margin itself</em> for pairs the shipped board carried tighter than that: proof by shipping. Same-net contacts
(a stub ending on a pad, a T junction) are held as <b>attachments</b>: at least one contact point of the pair must survive every move, so a part
cannot slide off its own copper. GND is exempt because the pour reconnects it.</p>
{img('census', 'The shipped margins of every pair the movable copper can meet, at the new 0.10 mm rule. The pile at 0–20 µm is the '
     'JLC-tight courtyard pairs and copper that shipped at rule pitch.')}
<p>Two rules keep the engine honest on an imperfect start. <b>Non-worsening:</b> a pair that is already below its floor (a pre-existing
overlap, or a ghost speck born on top of a trace) is admissible as long as the move does not reduce its margin. <b>Ghost pairs are never
grandfathered:</b> a ghost only ever grows into legal room; the copper under it has to leave first.</p>

<h2>4. One move: the line search</h2>
<p>Each unit proposes a step of at most 0.05 mm (for a ghost: 0.05 mm at its perimeter). The engine evaluates every margin at the full step;
if all pass the step is taken. Otherwise the failing pairs are the <b>clamps</b>, and a bisection finds the largest fraction α whose margins
all stay above floor. α = 0 means the unit waits. Because a step is far below any rule corridor, endpoint feasibility is sufficient: nothing can
tunnel through a trace between two admissible positions.</p>
{img('linesearch', 'Margins of the clamping pairs against the fraction of one growth step, for a ghost stopped mid-inflation. '
     'Each pair must stay above its own floor (dotted); the accepted fraction is where the first one would cross.')}

<h2>5. Inflation, and where the pressure goes</h2>
<p>A ghost that could only take a fraction of its step <b>deposits pressure</b> on what stopped it: the displacement each blocker would need,
directed away from the ghost's centre. A copper node receives it directly; a node bound to a part hands it to the part; a courtyard clamp goes to
the part. If the blocker is unmovable (a locked part's pad, the board edge) the demand is <b>absorbed</b> and counted, and the ghost gets a
push-back so it slides off walls toward open space. A ghost also feels a weak spring back to its site.</p>
{img('inflate', f'{E(inflate_ref)} being born. Its pads (red) grow, the trace beneath is pushed out (blue dots), the neighbour yields (dark orange), '
     'and the courtyard (dashed) clears.')}
<p>Parts move by the same rule. A part's drive is its target term (if it has one), the pressure it accumulated this cycle, and the tension of its
own copper; it is line-searched over all of its pads, its bound copper, its courtyard, the outline and the length caps of every net it touches.
Blocked parts deposit pressure on their blockers in turn, so a packed corridor creeps like a traffic jam instead of shearing. Mutually wedged
units are detected from the clamp graph and stepped jointly, and declared rigid groups always move as one.</p>
{img('pressure', 'The cycle with the most pressure in flight: blue arrows on parts, blue dots on copper nodes, black x at the clamps.')}
<p>After the units, the <b>copper pass</b>: pressured nodes yield as far as their own floors allow, sub-segments longer than 1.3 mm are pulled
straight, and nodes of nets over their allowance are pulled toward their neighbours.</p>

<h2>6. Springs and caps: the elastic modifier</h2>
<p>For every net the engine tracks the copper length inside the region. Growth beyond an <b>allowance</b>
(<span class=math>max(allow·L, allow_mm)</span>) pulls the net's nodes toward their neighbours and its end parts toward their copper with stiffness
<span class=math>k</span>. Growth beyond a <b>cap</b> (<span class=math>max(cap·L, cap_mm)</span>) enters the line search as one more margin, so no
step can exceed it; a unit that would has to wait, and the cap shows up in the stall report by net name. The table is the tunable part:</p>
<table><tr><th>net pattern</th><th>k, allow, cap, allow_mm, cap_mm</th></tr>{el_rows}</table>
<p class=small>Default {E(json.dumps(el.get('default', {})))}. The absolute floors matter: 15 % of a 4 mm stub is 0.6 mm, and without
<code>cap_mm</code> the first run froze the encoder fabric after half a millimetre.</p>
{img('springs', 'Copper grown per net over the run for the six nets that grew most. None reached a cap in this run; the squeeze below did.')}

<h2>7. Stalls, reroutes, and what remains</h2>
<p>When progress (ghost scale plus target progress) has not improved for 40 cycles the run has stalled and the clamp set names the wall.
Chains on that wall get an <b>atomic reroute</b>: A* on a 0.1 mm grid re-plans the chain between its current endpoints through the current
geometry (ghosts stamped at full size so the corridor is pre-cleared), the chain is re-subdivided, and the same oracle verifies it, attachments
included; a rejected route rolls back bit-perfectly and its blockers are stamped wider for the retry. A route that merely retraces the old path
is a no-op and is retired. Each round is a topology-change checkpoint at unchanged progress: both neighbouring boards are legal, the rip
happens between boards, never on one.</p>
{img('reroute', f'A reroute round in the narrow enc_corner run: the re-planned chains in magenta, before and after.') if 'reroute' in figs else
 '<p class=small>The narrow run recorded no verified reroute in this capture.</p>'}
<p>What is left after the last round is the honest residue, reported by unit and blocker: in the narrow run, pressure absorbed by J2's
mounting pads and the board edge. That table is the input to the next decision: a different site, a hand reroute, or a board stretch.</p>

<h2>8. The gate</h2>
<p>Every checkpoint is emitted as a complete board in its own pcbnew process: the dynamic originals are stripped once, the deformed chains and
vias are added back, moved parts sit at origin plus displacement, ghosts at full scale are placed as real footprints, zones refill. Then, in order:
KiCad DRC at severity error with nothing new versus the checkpoint-0 baseline (footprint pairs keyed by name, copper by position, the project's
excluded violations included); unconnected items not above baseline; every static copper object present byte for byte and every fixed footprint
where it was; net growth inside the caps; and check_sync against the schematic on the final board.</p>
{img('gate', 'Per-checkpoint gate results for the three runs: flat at baseline is the point.')}

<h2>9. Internal to the power stage: the squeeze</h2>
<p>The same engine with a different drive. All four half-bridge bands were shoved 1 mm toward their centre line, with gate, bootstrap and
turn-off nets capped at 0.3 mm of growth. The bands compressed until those caps held and the A-phase sense copper met the fixed INA240 outside
the region. That is the power stage's internal vertical slack at the new rules with the loops kept short.</p>
{img('squeeze', 'power_squeeze: cycle 0 and the settled state. Green arrows: how far each part moved.')}
{img('squeeze_caps', '')}
{video('settle_power_squeeze.mp4', 'power_squeeze, one frame every two cycles.')}

<h2>10. What the runs changed in the engine</h2>
<ul>
<li>Length caps need an absolute floor (<code>cap_mm</code>).</li>
<li>The DRC edge is the Edge.Cuts centreline with 1.5 mm corner arcs, not the bounding box; two tracks landed 10 µm inside the clearance before this.</li>
<li>Tracks are classified by segment intersection with the region, not by endpoints: a 36 mm sense track crossing the power region was invisible.</li>
<li>Obstacles are included by extent, not centroid: J9's courtyard reaches BH1 from 9 mm away.</li>
<li>DRC exclusions are position-keyed; a pre-existing excluded overlap resurfaces the moment its part moves and belongs to the baseline.</li>
<li>Stall detection is progress-based; springs jitter parts by microns forever.</li>
<li>Sites matter more than force: widening the search radius did what no pressure did.</li>
</ul>
{video('settle_enc_corner.mp4', 'The narrow enc_corner run (search radius 8 mm) for comparison: four ghosts never get out of the switch fabric.')}
"""
out = os.path.join(REVIEW, 'settle_explained.html')
open(out, 'w').write(page)
print(f'wrote {out} with {len(figs)} figures')
