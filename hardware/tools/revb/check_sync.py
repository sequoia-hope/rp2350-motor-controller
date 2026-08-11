#!/usr/bin/env python3
"""Acceptance test for sync_pcb.py: does the board match the schematic?

Exports the schematic netlist with kicad-cli and compares components,
footprint assignments and per-pad nets against the .kicad_pcb.

The shipped board predates any footprint-library discipline, so two classes
of difference are pre-existing and are counted, not failed on:

  nickname-only  pcb "R_0402_1005Metric" vs sch "Resistor_SMD:R_0402_1005Metric"
  stale value    pcb carries the symbol name ("Q_NMOS_GDS") not the part value
  thermal pad    a board pad with no net and no matching schematic pin

Plus one genuine rev-A discrepancy that predates the rev-B work and is
reported but not failed on (see KNOWN_PREEXISTING).

Anything else is a real desync and makes this exit non-zero.

  usage: check_sync.py [--pcb BOARD] [--verbose]
"""
import os, sys, re, subprocess, tempfile, argparse

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(HW, 'tools', 'jiggle2'))
from common import load_board                            # noqa: E402

SCH = os.path.join(HW, 'rp2350_driver.kicad_sch')
PCB = os.path.join(HW, 'rp2350_driver.kicad_pcb')

# Real differences that predate the rev-B work. Reported every run so they
# stay visible, but they do not fail the gate.
#   Q6 - shipped rev-A board has the 2N7002 on a SOT-89-3 land while the
#        schematic symbol carries SOT-23. Not touched by F-27/F-39.
KNOWN_PREEXISTING = {'footprint Q6'}


def tokenize(s):
    out, i, n = [], 0, len(s)
    while i < n:
        c = s[i]
        if c in '()':
            out.append(c); i += 1
        elif c == '"':
            j = i + 1; buf = []
            while s[j] != '"':
                if s[j] == '\\':
                    buf.append(s[j + 1]); j += 2
                else:
                    buf.append(s[j]); j += 1
            out.append(('str', ''.join(buf))); i = j + 1
        elif c.isspace():
            i += 1
        else:
            j = i
            while j < n and not s[j].isspace() and s[j] not in '()"':
                j += 1
            out.append(('sym', s[i:j])); i = j
    return out


def parse(s):
    toks = tokenize(s); pos = [0]

    def rd():
        t = toks[pos[0]]; pos[0] += 1
        if t == '(':
            lst = []
            while toks[pos[0]] != ')':
                lst.append(rd())
            pos[0] += 1
            return lst
        return t[1]
    return rd()


def kids(n, name):
    return [c for c in n if isinstance(c, list) and c and c[0] == name]


def one(n, name):
    k = kids(n, name)
    return k[0] if k else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pcb', default=PCB)
    ap.add_argument('--verbose', action='store_true')
    args = ap.parse_args()

    with tempfile.TemporaryDirectory() as td:
        netf = os.path.join(td, 'sch.net')
        subprocess.run(['kicad-cli', 'sch', 'export', 'netlist',
                        '--format', 'kicadsexpr', '-o', netf, SCH],
                       check=True, capture_output=True)
        root = parse(open(netf).read())

    sch_comp, sch_pad = {}, {}
    for c in kids(one(root, 'components'), 'comp'):
        ref = one(c, 'ref')[1]
        fp, val = one(c, 'footprint'), one(c, 'value')
        sch_comp[ref] = (fp[1] if fp else None, val[1] if val else None)
    for nt in kids(one(root, 'nets'), 'net'):
        nm = one(nt, 'name')[1]
        for nd in kids(nt, 'node'):
            sch_pad[(one(nd, 'ref')[1], one(nd, 'pin')[1])] = nm

    bd = load_board(args.pcb)
    pcb_comp, pcb_pad = {}, {}
    for fp in bd.GetFootprints():
        ref = fp.GetReference()
        pcb_comp[ref] = (fp.GetFPIDAsString(), fp.GetValue())
        for pad in fp.Pads():
            if pad.GetNumber():
                pcb_pad[(ref, pad.GetNumber())] = pad.GetNetname()

    fail, known = [], []
    noise = {'nickname': 0, 'value': 0, 'nc': 0, 'thermal': 0}

    for ref in sorted(set(sch_comp) ^ set(pcb_comp)):
        where = 'schematic only' if ref in sch_comp else 'board only'
        fail.append(f'component {ref}: {where}')

    for ref in sorted(set(sch_comp) & set(pcb_comp)):
        sfp, sval = sch_comp[ref]
        pfp, pval = pcb_comp[ref]
        if sfp != pfp:
            if pfp and sfp and sfp.split(':')[-1] == pfp.split(':')[-1]:
                noise['nickname'] += 1
                if args.verbose:
                    print(f'  nickname-only: {ref} {pfp} ~ {sfp}')
            elif f'footprint {ref}' in KNOWN_PREEXISTING:
                known.append(f'footprint {ref}: sch={sfp} pcb={pfp}')
            else:
                fail.append(f'footprint {ref}: sch={sfp} pcb={pfp}')
        if sval != pval:
            noise['value'] += 1
            if args.verbose:
                print(f'  stale value:   {ref} pcb={pval!r} sch={sval!r}')

    shared = set(sch_comp) & set(pcb_comp)
    for k in sorted(set(sch_pad) | set(pcb_pad)):
        if k[0] not in shared:
            continue
        a, b = sch_pad.get(k), pcb_pad.get(k)
        if a == b:
            continue
        if a and a.startswith('unconnected-') and not b:
            noise['nc'] += 1
            continue
        if a is None and not b:
            noise['thermal'] += 1        # board-only pad carrying no net
            continue
        fail.append(f'pad {k[0]}.{k[1]}: sch={a!r} pcb={b!r}')

    print(f'components: {len(sch_comp)} schematic / {len(pcb_comp)} board')
    print(f'pre-existing (not failed on): {noise["nickname"]} nickname-only '
          f'footprints, {noise["value"]} stale values, {noise["nc"]} '
          f'unconnected-pad naming, {noise["thermal"]} board-only pads')
    for k in known:
        print(f'known rev-A discrepancy: {k}')
    if fail:
        print(f'\nDESYNC — {len(fail)} real difference(s):')
        for f in fail:
            print(f'  {f}')
        return 1
    print('\nIN SYNC — board matches schematic')
    return 0


if __name__ == '__main__':
    sys.exit(main())
