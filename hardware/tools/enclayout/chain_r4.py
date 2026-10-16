"""build r4 = polish the best of r2 and r3 with the fixed packing rules, plus fresh long starts; then run it."""
import json, os, subprocess
HERE = os.path.dirname(os.path.abspath(__file__))
starts = []
for run in ('r2', 'r3'):
    recs = json.load(open(os.path.join(HERE, 'data', 'runs', run, 'results.json')))
    for rec in recs[:5]:
        for k in range(2):
            starts.append({'seed': f'from:{run}/{rec["name"]}', 'rng': 400 + len(starts), 'T0': 0.8, 's0': 0.8})
for k, s in enumerate(['a_up', 'a_up', 'hat_cd', 'rows', 'fan', 'random']):
    starts.append({'seed': s, 'rng': 500 + k})
json.dump({'name': 'r4', 'steps': 300000, 'dy_max': 1.2, 'procs': 24, 'starts': starts}, open(os.path.join(HERE, 'data', 'r4.json'), 'w'), indent=1)
print(len(starts), 'starts for r4')
subprocess.run(['python3', os.path.join(HERE, 'search.py'), os.path.join(HERE, 'data', 'r4.json')], stdout=open(os.path.join(HERE, 'data', 'r4.log'), 'w'), stderr=subprocess.STDOUT)
