#!/usr/bin/env python3
"""kicad-cli DRC runner + position-independent diff (so a warped board can be
compared with its own baseline). Signature = (type, sorted normalised item
descriptions) with lengths/coordinates stripped."""
import json, re, subprocess, collections, os, sys
def run_drc(board, out):
    subprocess.run(['kicad-cli', 'pcb', 'drc', '--severity-error', '--format', 'json', '--all-track-errors', '-o', out, board], check=True, capture_output=True)
    return json.load(open(out))
def norm(desc):
    d = re.sub(r', length [\d.]+ mm', '', desc)
    d = re.sub(r'\(\s*[-\d.]+\s*,\s*[-\d.]+\s*\)', '', d)
    return d.strip()
def sig(v):
    return (v['type'], tuple(sorted(norm(i['description']) for i in v['items'])))
def diff(base, cur):
    b = collections.Counter(sig(v) for v in base['violations']); c = collections.Counter(sig(v) for v in cur['violations'])
    new = []; gone = []
    for v in cur['violations']:
        s = sig(v)
        if c[s] > b.get(s, 0): new.append(v); c[s] -= 1
    for v in base['violations']:
        s = sig(v)
        if b[s] > collections.Counter(sig(x) for x in cur['violations']).get(s, 0): gone.append(v); b[s] -= 1
    return new, gone
def summary(d):
    return f"{len(d['violations'])} violations ({dict(collections.Counter(v['type'] for v in d['violations']))}), {len(d['unconnected_items'])} unconnected"
if __name__ == '__main__':
    base = json.load(open(sys.argv[1])); cur = run_drc(sys.argv[2], sys.argv[3]) if len(sys.argv) > 3 else json.load(open(sys.argv[2]))
    new, gone = diff(base, cur)
    print('baseline:', summary(base)); print('current: ', summary(cur))
    print(f'new: {len(new)}  gone: {len(gone)}')
    for v in new[:40]:
        p = v['items'][0].get('pos', {}); print(f"  NEW {v['type']:22s} ({p.get('x',0):.2f},{p.get('y',0):.2f}) " + ' | '.join(norm(i['description'])[:55] for i in v['items']))
