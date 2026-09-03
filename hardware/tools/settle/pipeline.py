#!/usr/bin/env python3
"""settle: run the whole pipeline for one config and print the verdict.

    python3 pipeline.py CONFIG.json [settle args...]

prep -> settle -> emit -> render, logs in data/<name>/*.log, then a one-line
verdict: progress, ghosts born, gate pass count, nets at cap, edge hits.
"""
import json, os, subprocess, sys
from common import load_config, data_path, HERE

cfg = load_config()
extra = [a for a in sys.argv[2:]]
py = sys.executable


def run(script, args, log):
    with open(data_path(cfg, log), 'w') as f:
        r = subprocess.run([py, os.path.join(HERE, script), cfg['_path']] + args, stdout=f, stderr=subprocess.STDOUT)
    tail = open(data_path(cfg, log)).read().strip().splitlines()[-1:] or ['']
    print(f'{script}: rc={r.returncode} — {tail[0][:150]}', flush=True)
    return r.returncode

if run('prep.py', [], 'prep.log'):
    sys.exit(1)
if run('settle.py', extra, 'settle.log'):
    sys.exit(2)
run('emit.py', [], 'emit.log')
run('render.py', [], 'render.log')
rep = json.load(open(data_path(cfg, 'settle_report.json')))
gate = json.load(open(data_path(cfg, 'gate.json')))
born = sum(1 for v in rep['ghosts'].values() if v['g'] >= 1)
ok = sum(1 for e in gate['steps'] if e['ok'])
edge = sum(a['hits'] for a in rep['absorb'] if a['blocker'] == 'board_edge')
print(f"\n{cfg['name']}: s={rep['s']} ghosts {born}/{len(rep['ghosts'])} born, {len(rep['parts_moved'])} parts moved, "
      f"gate {ok}/{len(gate['steps'])}, {len(gate['over_cap'])} nets at cap, {edge} edge hits, "
      f"check_sync={gate['check_sync']} — review/settle_{cfg['name']}.html")
