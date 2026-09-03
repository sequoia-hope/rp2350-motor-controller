#!/usr/bin/env python3
"""settle stage 3: figures + review page for one run.

    python3 render.py CONFIG.json [--out review/settle.html]

Draws the region at checkpoint 0 and at the final checkpoint (static copper
grey, dynamic copper by layer, movable parts' pads, ghosts at their current
scale, part displacement arrows), plus the run tables: gate per checkpoint,
ghosts, moved parts, net length growth, stall blockers, absorbed pressure.
The page is written into the repo's review/ directory (one page per run
name) with the figures beside it under review/img/settle_<name>_*.png.
"""
import json, math, os, sys, html
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, FancyArrow
from common import HW, load_config, load_json, data_path, LAYER_NAMES

cfg = load_config()
M = load_json(cfg, 'model.json')
name = cfg['name']
REPO = os.path.dirname(HW)
REVIEW = os.path.join(REPO, 'review')
IMG = os.path.join(REVIEW, 'img')
os.makedirs(IMG, exist_ok=True)
out_html = sys.argv[sys.argv.index('--out') + 1] if '--out' in sys.argv else os.path.join(REVIEW, f'settle_{name}.html')


def maybe(fn):
    p = data_path(cfg, fn)
    return json.load(open(p)) if os.path.exists(p) else None

TR = maybe('traj.json')
REP = maybe('settle_report.json') or {}
GATE = maybe('gate.json') or {}
LOG = open(data_path(cfg, 'settle.log')).read() if os.path.exists(data_path(cfg, 'settle.log')) else ''

LCOL = {0: '#c0392b', 2: '#2471a3', 4: '#1e8449', 6: '#7d3c98', 8: '#b9770e', 10: '#117a65'}
region = M['region']
ghost_refs = {gh['ref'] for gh in M['ghosts']}


def draw(ax, ck, title):
    x0, y0, x1, y1 = region
    ax.set_xlim(x0 - 0.5, x1 + 0.5)
    ax.set_ylim(y1 + 0.5, y0 - 0.5)
    ax.set_aspect('equal')
    ax.set_title(title, fontsize=10)
    ex0, ey0, ex1, ey1 = M['rules']['board_edge']
    ax.add_patch(Polygon([[ex0, ey0], [ex1, ey0], [ex1, ey1], [ex0, ey1]], closed=True,
                         fill=False, ec='black', lw=1.2))
    for t in M['tracks']:
        ax.plot([t['a'][0], t['b'][0]], [t['a'][1], t['b'][1]], color='#999', lw=t['w'] * 3, alpha=0.6,
                solid_capstyle='round')
    for v in M['vias']:
        ax.add_patch(plt.Circle((v['x'], v['y']), v['dia'] / 2, color='#999', alpha=0.6))
    P = ck['P'] if ck else [[n['x'], n['y']] for n in M['nodes']]
    D = ck['D'] if ck else {}
    chains = ck['chains'] if ck else None
    for ei, e in enumerate(M['edges']):
        ids = chains[ei] if chains else [e['a'], e['b']]
        xs = [P[i][0] for i in ids]; ys = [P[i][1] for i in ids]
        ax.plot(xs, ys, color=LCOL.get(e['layer'], '#333'), lw=max(0.4, e['w'] * 3), alpha=0.8,
                solid_capstyle='round')
    for nid, n in enumerate(M['nodes']):
        if n['kind'] == 'via':
            ax.add_patch(plt.Circle((P[nid][0], P[nid][1]), n['dia'] / 2, color='#555', alpha=0.8))
    for p in M['pads']:
        r = p['ref']
        mv = M['parts'].get(r, {}).get('movable')
        d = D.get(r, [0, 0])
        pts = [[q[0] + d[0], q[1] + d[1]] for q in p['pts']]
        col = '#e67e22' if (mv and (abs(d[0]) > 1e-3 or abs(d[1]) > 1e-3)) else ('#f5b041' if mv else '#7f8c8d')
        ax.add_patch(Polygon(pts, closed=True, fc=col, ec='none', alpha=0.75))
    for r, entry in M['courtyards'].items():
        if r in ghost_refs:
            continue
        d = D.get(r, [0, 0])
        for lay, polys in entry.items():
            for poly in polys:
                pts = [[q[0] + d[0], q[1] + d[1]] for q in poly]
                ax.add_patch(Polygon(pts, closed=True, fill=False, ec='#555' if lay == 'B' else '#aaa', lw=0.4,
                                     ls='-' if lay == 'B' else ':'))
    for gi, gh in enumerate(M['ghosts']):
        g = ck['G'][gi] if ck else 0.05
        tx, ty = ck['T'][gi] if ck else (0, 0)
        cx, cy = gh['at']
        for p in gh['pads']:
            pts = [[cx + tx + (q[0] - cx) * g, cy + ty + (q[1] - cy) * g] for q in p['pts']]
            ax.add_patch(Polygon(pts, closed=True, fc='#c0392b', ec='black', lw=0.4, alpha=0.9))
        for lay, polys in gh['courtyards'].items():
            for poly in polys:
                pts = [[cx + tx + (q[0] - cx) * g, cy + ty + (q[1] - cy) * g] for q in poly]
                ax.add_patch(Polygon(pts, closed=True, fill=False, ec='#c0392b', lw=0.8, ls='--'))
        ax.plot([cx], [cy], marker='+', color='#c0392b', ms=6)
        ax.annotate(f"{gh['ref']} {g:.2f}", (cx + tx, cy + ty), fontsize=6, color='#c0392b',
                    xytext=(3, 3), textcoords='offset points')
    for r, d in D.items():
        if math.hypot(*d) < 0.05:
            continue
        p = M['parts'][r]
        ax.add_patch(FancyArrow(p['cx'], p['cy'], d[0], d[1], width=0.03, head_width=0.25,
                                length_includes_head=True, color='#1a5276', alpha=0.9))
    ax.set_xlabel('mm'); ax.tick_params(labelsize=7)

figs = []
cks = TR['checkpoints'] if TR else [None]
sel = [0, len(cks) - 1] if len(cks) > 1 else [0]
fig, axes = plt.subplots(1, len(sel), figsize=(7.5 * len(sel), 7.5 * (region[3] - region[1]) / (region[2] - region[0]) + 1))
if len(sel) == 1:
    axes = [axes]
for ax, k in zip(axes, sel):
    ck = cks[k]
    draw(ax, ck, f'{name}: checkpoint {k}' + (f" s={ck['s']:.2f} {ck.get('tag', '')}" if ck else ''))
fig.tight_layout()
png = os.path.join(IMG, f'settle_{name}_region.png')
fig.savefig(png, dpi=130)
plt.close(fig)
figs.append(os.path.relpath(png, REVIEW))

if REP.get('history'):
    h = REP['history']
    fig, ax = plt.subplots(figsize=(8, 2.6))
    ax.plot([r['cycle'] for r in h], [r['s'] for r in h], label='progress s', color='#c0392b')
    ax2 = ax.twinx()
    ax2.plot([r['cycle'] for r in h], [r['adv'] for r in h], label='unit advance mm', color='#2471a3', lw=0.8)
    ax2.plot([r['cycle'] for r in h], [r['copper'] for r in h], label='copper motion mm', color='#1e8449', lw=0.8)
    ax.set_xlabel('cycle'); ax.set_ylabel('s'); ax2.set_ylabel('mm / cycle')
    ax.legend(loc='upper left', fontsize=7); ax2.legend(loc='upper right', fontsize=7)
    fig.tight_layout()
    png = os.path.join(IMG, f'settle_{name}_history.png')
    fig.savefig(png, dpi=120)
    plt.close(fig)
    figs.append(os.path.relpath(png, REVIEW))

# ---- page --------------------------------------------------------------------
E = html.escape


def table(rows, cols, limit=None):
    if not rows:
        return '<p><i>none</i></p>'
    out = ['<table><tr>' + ''.join(f'<th>{E(c)}</th>' for c in cols) + '</tr>']
    for r in rows[:limit] if limit else rows:
        out.append('<tr>' + ''.join(f'<td>{E(str(r.get(c, "")))}</td>' for c in cols) + '</tr>')
    out.append('</table>')
    if limit and len(rows) > limit:
        out.append(f'<p class=small>{len(rows) - limit} more not shown</p>')
    return '\n'.join(out)

parts = M['parts']
moved = sorted(((r, d) for r, d in REP.get('parts_moved', {}).items()), key=lambda kv: -math.hypot(*kv[1]))
moved_rows = [dict(ref=r, dx=d[0], dy=d[1], mm=round(math.hypot(*d), 3),
                   group=parts[r].get('group') or '') for r, d in moved]
ghost_rows = [dict(ref=r, scale=v['g'], drift_x=v['T'][0], drift_y=v['T'][1],
                   site=f"({gh['at'][0]:.2f}, {gh['at'][1]:.2f})", born='yes' if v['g'] >= 1 else 'no')
              for gh in M['ghosts'] for r, v in REP.get('ghosts', {}).items() if r == gh['ref']]
len_rows = [dict(net=r['net'], region_mm=round(r['L0'] + r['static'], 2), grown_mm=r.get('grown_mm'),
                 growth_pct=round(r['growth'] * 100, 1), allow_mm=r.get('allow_mm'), cap_mm=r.get('cap_mm'),
                 at_cap='YES' if r['at_cap'] else '') for r in REP.get('lengths', []) if abs(r['growth']) > 1e-4]
stall_rows = []
for s in REP.get('stalled', []):
    for b in s['blockers'][:4]:
        stall_rows.append(dict(unit=s['unit'], hard_walled='yes' if s['hard_walled'] else '',
                               blocker=b['kind'], net=b.get('net', ''), ref=b.get('ref', ''),
                               margin=b['margin'], floor=b['floor'], pos=b.get('pos', '')))
absorb_rows = REP.get('absorb', [])[:25]
gate_rows = GATE.get('steps', [])
rr = REP.get('reroutes', [])
rr_ok = sum(1 for e in rr if e.get('result') == 'ok')

summary = []
if REP:
    summary.append(f"<b>{REP['cycles']}</b> cycles in {REP['seconds']} s, progress s = <b>{REP['s']}</b>; "
                   f"{sum(1 for v in REP['ghosts'].values() if v['g'] >= 1)}/{len(REP['ghosts'])} ghosts born; "
                   f"{len(REP['parts_moved'])} parts moved; {rr_ok}/{len(rr)} reroutes verified in {REP['reroute_rounds']} rounds.")
if GATE:
    ok = sum(1 for e in gate_rows if e.get('ok'))
    summary.append(f"Gate: <b>{ok}/{len(gate_rows)}</b> checkpoints pass (baseline {GATE.get('baseline_violations')} violations); "
                   f"length caps hit: {len(GATE.get('over_cap', []))}; check_sync: {GATE.get('check_sync')}.")
cfg_show = {k: v for k, v in cfg.items() if not k.startswith('_') and k not in ('elastic',)}

page = f"""<!doctype html><meta charset=utf-8><title>settle: {E(name)}</title>
<style>body{{font:14px/1.45 system-ui,sans-serif;max-width:1400px;margin:1.5em auto;padding:0 1em;color:#222}}
table{{border-collapse:collapse;font-size:12px;margin:.5em 0}}td,th{{border:1px solid #ccc;padding:2px 6px;text-align:left}}
th{{background:#f3f3f3}}img{{max-width:100%}}pre{{background:#f7f7f7;padding:.6em;font-size:11px;overflow:auto;max-height:22em}}
.small{{color:#666;font-size:12px}}h2{{margin-top:1.6em;border-bottom:1px solid #ddd}}</style>
<h1>settle run: {E(name)}</h1>
<p>{' '.join(summary) or 'no run yet'}</p>
<p class=small>Region {region}; rules clearance {M['rules']['clearance']} mm, hole {M['rules']['hole_clearance']}, edge {M['rules']['edge_clearance']}, guard {M['rules'].get('guard')}.
{sum(1 for p in parts.values() if p['movable'])} movable parts, {len(M['edges'])} dynamic segments, {len(M['nodes'])} nodes, {len(M['ghosts'])} ghosts.
Orange pads = movable parts (dark orange = moved); grey = fixed; red = ghosts at their current scale; blue arrows = part displacement.
Copper by layer: F red, B blue, In1 green, In2 purple, In3 brown, In4 teal; static copper grey.</p>
{''.join(f'<p><img src="{E(f)}"></p>' for f in figs)}
<h2>Gate per checkpoint</h2>
{table(gate_rows, ['step', 's', 'tag', 'unconnected', 'new_violations', 'static_missing', 'fixed_moved', 'ok', 'new_desc'])}
<h2>Ghosts</h2>
{table(ghost_rows, ['ref', 'site', 'scale', 'drift_x', 'drift_y', 'born'])}
<h2>Parts moved</h2>
{table(moved_rows, ['ref', 'dx', 'dy', 'mm', 'group'], 40)}
<h2>Net length growth (copper inside the region)</h2>
{table(len_rows, ['net', 'region_mm', 'grown_mm', 'growth_pct', 'allow_mm', 'cap_mm', 'at_cap'], 40)}
<h2>Stalled units and their blockers</h2>
{table(stall_rows, ['unit', 'hard_walled', 'blocker', 'net', 'ref', 'margin', 'floor', 'pos'], 60)}
<h2>Absorbed pressure (where demand died)</h2>
{table(absorb_rows, ['mover', 'blocker', 'what', 'hits'])}
<h2>Config</h2>
<pre>{E(json.dumps(cfg_show, indent=1))}</pre>
<h2>Elastic table</h2>
<pre>{E(json.dumps(cfg.get('elastic', {}), indent=1))}</pre>
<h2>Log tail</h2>
<pre>{E(LOG[-6000:])}</pre>
"""
open(out_html, 'w').write(page)
print(f'wrote {out_html} + {len(figs)} figures')
