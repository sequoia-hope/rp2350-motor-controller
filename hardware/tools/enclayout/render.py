#!/usr/bin/env python3
"""enclayout stage 2: figures + the review page for one or more runs.

    python3 render.py RUN [RUN...] [--top N]     -> review/img/enclayout/<run>_<name>.png, review/enclayout.html
"""
import json, math, os, sys, re
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Arc
from model import Corner, rot_xy, DEFAULT_CFG
from search import State
HERE = os.path.dirname(os.path.abspath(__file__))
REVIEW = os.path.normpath(os.path.join(HERE, '..', '..', '..', 'review'))
IMG = os.path.join(REVIEW, 'img', 'enclayout'); os.makedirs(IMG, exist_ok=True)
COL = dict(A='#d62728', B='#1f77b4', C='#2ca02c', D='#ff7f0e')
GREY = '#888'


def draw(C, pose, dy, title, path, paths=None, chan=None, bad=None):
    fig, ax = plt.subplots(figsize=(7.2, 9.6), dpi=110)
    ax.set_xlim(111.0, 134.0); ax.set_ylim(129.8, 94.0); ax.set_aspect('equal'); ax.set_facecolor('#f6f4ee')
    ax.set_xlabel('x (mm)'); ax.set_ylabel('y (mm)')
    # board edge
    o = C.outline['box']; r = C.outline['corner_r']
    ax.plot([o[0], o[0]], [94, o[3] - r], color='k', lw=1.2); ax.plot([o[0] + r, 134], [o[3], o[3]], color='k', lw=1.2)
    ax.add_patch(Arc((o[0] + r, o[3] - r), 2 * r, 2 * r, theta1=90, theta2=180, color='k', lw=1.2))
    ax.axhspan(o[3], 129.8, color='#ddd', zorder=0); ax.axvspan(111, o[0], color='#ddd', zorder=0)
    # fixed rects
    for ref, box in zip(C.frect_ref, C.frects):
        ax.add_patch(Rectangle((box[0], box[1]), box[2] - box[0], box[3] - box[1], fc='#d9d9d9', ec='#999', lw=0.5))
        if box[2] - box[0] > 1.6 and box[3] - box[1] > 1.2:
            ax.text((box[0] + box[2]) / 2, (box[1] + box[3]) / 2, ref.split('.')[0], ha='center', va='center', fontsize=5.5, color='#555', clip_on=True)
    for k in C.keep: ax.add_patch(Circle((k[0], k[1]), k[2], fc='none', ec='#b55', ls='--', lw=0.8))
    # tracks + circles
    if len(C.tpts): ax.scatter(C.tpts[:, 0], C.tpts[:, 1], s=1.5, c='#bbb', marker='.', zorder=1)
    for c, ref, sl in zip(C.circ, C.circ_ref, C.circ_slider):
        y = c[1] - (dy if sl else 0)
        if ref.startswith('J12'):
            net = C.j12[ref.split('.')[1]]['net']; m = re.match(r'/ENC_([ABCD])_', net)
            ax.add_patch(Circle((c[0], y), c[2], fc=COL[m.group(1)] if m else '#777', ec='k', lw=0.4, zorder=3))
            ax.text(c[0], y, ref.split('.')[1], ha='center', va='center', fontsize=4.5, color='w', zorder=4, clip_on=True)
        elif ref == 'via':
            ax.add_patch(Circle((c[0], y), c[2], fc='#666', ec='none', zorder=2))
        else:
            ax.add_patch(Circle((c[0], y), c[2], fc='#999', ec='k', lw=0.4, zorder=2))
    for ref, s in C.d['sliders'].items():
        b = s['crt']; ax.add_patch(Rectangle((b[0], b[1] - dy), b[2] - b[0], b[3] - b[1], fc='none', ec='#777', ls=':', lw=0.8))
        ax.text(b[2] - 0.3, b[1] - dy + 0.9, f'{ref} (top side){f" ↑{dy:.1f}" if dy else ""}', ha='right', fontsize=6, color='#777', clip_on=True)
    # movable parts
    crts = C.crt_world(pose)
    for i, ref in enumerate(C.refs):
        ch = C.channel.get(ref); col = COL.get(ch, GREY); b = crts[i]
        ax.add_patch(Rectangle((b[0], b[1]), b[2] - b[0], b[3] - b[1], fc=col, alpha=0.28, ec=col, lw=0.8, zorder=5))
        for pr in C.pad_rects(pose, i):
            ax.add_patch(Rectangle((pr[0], pr[1]), pr[2] - pr[0], pr[3] - pr[1], fc=col, alpha=0.6, ec='none', zorder=6))
        big = (b[2] - b[0]) * (b[3] - b[1]) > 2.5
        ax.text((b[0] + b[2]) / 2, (b[1] + b[3]) / 2, ref, ha='center', va='center', fontsize=6 if big else 4.2, color='k', zorder=7, clip_on=True)
        if bad and ref in bad: ax.add_patch(Rectangle((b[0], b[1]), b[2] - b[0], b[3] - b[1], fc='none', ec='magenta', lw=1.4, zorder=8))
    # chain paths
    for (ch, side), seq in C.paths.items():
        pts = [(C.j12[n[1]]['x'], C.j12[n[1]]['y']) if n[0] == 'J12' else C.pad_xy(pose, *n) for n in seq]
        xs, ys = zip(*pts); ax.plot(xs, ys, color=COL[ch], lw=1.1, ls='-' if side == 'P' else '--', zorder=9, alpha=0.9)
    if chan:
        ax.text(111.4, 94.6, '   '.join(f'{ch}: {chan[ch]:.1f}' for ch in 'ABCD') + f'   spread {max(chan.values()) - min(chan.values()):.1f} mm',
                fontsize=8, va='top', bbox=dict(fc='w', ec='none', alpha=0.85))
    ax.set_title(title, fontsize=9)
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


def cluster_key(C, rec):
    p = rec['pose']
    return tuple(int(round(p[u][0] / 3)) * 100 + int(round(p[u][1] / 3)) for u in ('U18', 'U19', 'U20', 'U21')) + (round(rec['dy'], 1),)


def main():
    runs = [a for a in sys.argv[1:] if not a.startswith('--')]
    top = int(sys.argv[sys.argv.index('--top') + 1]) if '--top' in sys.argv else 8
    C = Corner()
    # reference: current layout
    st0 = State(C, DEFAULT_CFG, C.pose0(), 0.0); t0 = st0.terms()
    draw(C, C.pose0(), 0.0, 'plan of record today (bottom side, viewed from top)', os.path.join(IMG, 'current.png'), chan=t0['chan'])
    sections = []; allrows = []
    for run in runs:
        rp = os.path.join(HERE, 'data', 'runs', run, 'results.json')
        if os.path.exists(rp): recs = json.load(open(rp))
        else:
            import glob
            recs = [json.load(open(f)) for f in glob.glob(os.path.join(HERE, 'data', 'runs', run, '*.json'))]
            recs = [r for r in recs if 'terms' in r]; recs.sort(key=lambda r: r['terms']['cost'])
        seen = {}; picks = []
        for rec in recs:
            k = cluster_key(C, rec)
            if k in seen: continue
            seen[k] = rec; picks.append(rec)
            if len(picks) >= top: break
        cards = []
        for rec in picks:
            pose = np.array([rec['pose'][r] for r in C.refs]); t = rec['terms']
            fn = f'{run}_{rec["name"]}.png'
            draw(C, pose, rec['dy'], f'{run}/{rec["name"]}  cost {t["cost"]:.1f}  overlap {t["overlap"]:.3f}', os.path.join(IMG, fn),
                 chan=t['chan'], bad=set(rec['part_overlap']))
            P = rec['paths']
            rows = ''.join(f'<tr><td style="color:{COL[ch]}"><b>{ch}</b></td><td>{P[ch + "P"]:.1f}</td><td>{P[ch + "N"]:.1f}</td><td>{t["chan"][ch]:.1f}</td></tr>' for ch in 'ABCD')
            bad = ', '.join(f'{r} {v:.2f}' for r, v in sorted(rec['part_overlap'].items(), key=lambda kv: -kv[1])[:8]) or 'none'
            hits = C.slide_hits(rec['dy'], DEFAULT_CFG['clearance']) if rec['dy'] > 0.02 else {}
            slide = ('connector slide touches fixed copper: ' + ', '.join(f'{n} ({k} pts)' for n, k in hits.items())) if hits else 'connector slide touches nothing fixed'
            cards.append(f'''<div class="card"><img src="img/enclayout/{fn}" alt="{rec['name']}">
<h4>{rec['name']}</h4>
<table class="mini"><tr><th>ch</th><th>P</th><th>N</th><th>mean</th></tr>{rows}</table>
<p><b>spread {t['spread']:.1f} mm</b> · sum {t['length']:.1f} · P/N mismatch {t['mismatch']:.1f} · pair copper (MST) {t['copper']:.0f} ·
connectors up {rec['dy']:.2f} mm · overlap score {t['overlap']:.3f}<br><span class="muted">parts with residual overlap: {bad}<br>{slide}</span></p></div>''')
            allrows.append((run, rec, t))
        sections.append(f'<h2 id="{run}">run {run}</h2><div class="grid">{"".join(cards)}</div>')
    table = ''.join(f'<tr><td>{run}</td><td>{rec["name"]}</td><td>{t["cost"]:.1f}</td><td>{t["overlap"]:.3f}</td><td><b>{t["spread"]:.1f}</b></td>'
                    f'<td>{t["chan"]["A"]:.1f}</td><td>{t["chan"]["B"]:.1f}</td><td>{t["chan"]["C"]:.1f}</td><td>{t["chan"]["D"]:.1f}</td>'
                    f'<td>{t["mismatch"]:.1f}</td><td>{t["length"]:.1f}</td><td>{rec["dy"]:.2f}</td><td>{len(rec["part_overlap"])}</td></tr>'
                    for run, rec, t in sorted(allrows, key=lambda x: x[2]['cost']))
    tmpl = open(os.path.join(HERE, 'page.html')).read()
    html = tmpl.replace('{{TABLE}}', table).replace('{{SECTIONS}}', ''.join(sections)) \
        .replace('{{CUR}}', '   '.join(f'{ch} {t0["chan"][ch]:.1f}' for ch in 'ABCD') + f' — spread {t0["spread"]:.1f} mm')
    open(os.path.join(REVIEW, 'enclayout.html'), 'w').write(html)
    print('wrote', os.path.join(REVIEW, 'enclayout.html'), len(allrows), 'cards')


if __name__ == '__main__':
    main()
