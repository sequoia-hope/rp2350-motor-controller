#!/usr/bin/env python3
"""jiggle2 stage 4: referee loop. Diff real DRC (work copy vs baseline),
attribute every NEW violation to emitted copper, withdraw the offenders
(mark dirty in solution.json), re-emit, repeat until the work copy
introduces zero new violations or nothing more can be withdrawn."""
import json, math, subprocess, sys, os
from common import DATA, load_json, save_json

MAX_ROUNDS = 14
NEAR = 0.6      # mm: violation-to-copper attribution radius


def vsig(v):
    """Stable signature to match a violation against the baseline set."""
    pts = tuple(sorted((round(i['pos']['x'], 2), round(i['pos']['y'], 2))
                       for i in v['items'] if 'pos' in i))
    return (v['type'], pts)


def pt_polyline_dist(x, y, pts):
    best = 1e9
    for k in range(len(pts) - 1):
        (ax, ay), (bx, by) = pts[k], pts[k+1]
        dx, dy = bx-ax, by-ay
        L2 = dx*dx + dy*dy
        t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((x-ax)*dx + (y-ay)*dy) / L2))
        best = min(best, math.hypot(x - ax - t*dx, y - ay - t*dy))
    return best


base = json.load(open(f'{DATA}/drc_baseline.json'))
base_sigs = {vsig(v) for v in base['violations']}
base_n = len(base['violations'])
base_u = len(base['unconnected_items'])

for rnd in range(1, MAX_ROUNDS + 1):
    cur = json.load(open(f'{DATA}/drc_jiggle2.json'))
    new = [v for v in cur['violations'] if vsig(v) not in base_sigs]
    u = len(cur['unconnected_items'])
    print(f'round {rnd}: {len(cur["violations"])} violations '
          f'({len(new)} new vs baseline {base_n}), unconnected {u} (baseline {base_u})')
    if not new:
        print('no new violations — work copy is DRC-neutral. done.')
        break

    sol = load_json('solution.json')
    withdrew = 0
    unattributed = []
    for v in new:
        # withdraw only the single most-implicated object (PathFinder-style
        # minimal escalation) — bystanders keep their copper
        best = None   # (dist, kind, obj)
        for it in v['items']:
            if 'pos' not in it:
                continue
            x, y = it['pos']['x'], it['pos']['y']
            for e in sol['edges']:
                if e.get('withdrawn'):
                    continue
                d = pt_polyline_dist(x, y, e['pts']) - e['w']/2
                if d < NEAR and (best is None or d < best[0]):
                    best = (d, e)
            for w in sol['vias']:
                if w.get('withdrawn'):
                    continue
                d = math.hypot(x - w['x'], y - w['y']) - w['dia']/2
                if d < NEAR and (best is None or d < best[0]):
                    best = (d, w)
        if best is not None:
            best[1]['withdrawn'] = True
            withdrew += 1
        else:
            unattributed.append((v['type'], v['description'][:70]))
    print(f'  withdrew {withdrew} copper objects; {len(unattributed)} violations unattributed')
    for t, d in unattributed[:8]:
        print(f'    ? {t}: {d}')
    if withdrew == 0:
        print('nothing more to withdraw — remaining new violations are not ours '
              'or need hand attention. stopping.')
        break
    save_json('solution.json', sol)
    cmd = [sys.executable, os.path.join(os.path.dirname(__file__), 'emit.py')]
    cmd += [a for a in sys.argv[1:] if a == '--all']
    r = subprocess.run(cmd, capture_output=True, text=True)
    tail = [l for l in r.stdout.splitlines() if l.strip()][-9:]
    print('  ' + '\n  '.join(tail))
    if r.returncode != 0:
        print(r.stderr[-500:])
        break
