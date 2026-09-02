#!/usr/bin/env python3
"""Re-point schematic Footprint fields for the rev-b-lean Tier A swaps.

Edits symbol *instances* only (Reference with a digit), never lib_symbols.
Per-ref rules win over the global footprint-class rules.

  usage: edit_sch_footprints.py [--sch FILE] [--dry-run]
"""
import os, re, sys, argparse

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(os.path.dirname(HERE))
SCH = os.path.join(HW, 'rp2350_driver.kicad_sch')

# Tier A4: tight-courtyard variants (same pads, JLCPCB spacing margins)
GLOBAL = {
    'Resistor_SMD:R_0402_1005Metric':  'parts:R_0402_1005Metric_JLC',
    'Capacitor_SMD:C_0402_1005Metric': 'parts:C_0402_1005Metric_JLC',
    'Resistor_SMD:R_0603_1608Metric':  'parts:R_0603_1608Metric_JLC',
    'Capacitor_SMD:C_0603_1608Metric': 'parts:C_0603_1608Metric_JLC',
    'Resistor_SMD:R_0805_2012Metric':  'parts:R_0805_2012Metric_JLC',
    'Capacitor_SMD:C_0805_2012Metric': 'parts:C_0805_2012Metric_JLC',
    # Tier A3: one SC-70 footprint for all 14 TS5A3159DCK
    'Package_TO_SOT_SMD:SOT-363_SC-70-6': 'parts:SOT-363_SC-70-6_JLC',
}
PER_REF = {}
# Tier A1: DNP snubbers 0805 -> R 0603 (75 V), C 0402 (1 nF/100 V stock)
for r in 'R17 R18 R37 R38 R48 R52 R80 R81'.split():
    PER_REF[r] = 'parts:R_0603_1608Metric_JLC'
for c in 'C7 C9 C25 C26 C33 C34 C53 C54'.split():
    PER_REF[c] = 'parts:C_0402_1005Metric_JLC'
# Tier A2: courtyard-free test points (pad size kept: D1.0 where it was D1.0)
for t in 'TP12 TP13 TP14 TP15 TP16 TP17 TP18 TP19 TP20'.split():
    PER_REF[t] = 'parts:TestPoint_Pad_D1.0mm_no_courtyard'
PER_REF['TP8'] = 'parts:TestPoint_Pad_D1.5mm_no_courtyard'
# Tier A5: WS2812B 3528 -> 2020, pads renumbered to the MotorBuck:WS2812B symbol
PER_REF['D1'] = 'parts:LED_WS2812B-2020_PLCC4_2.0x2.0mm_MotorBuck'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sch', default=SCH)
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args()
    s = open(a.sch).read()
    ref_re = re.compile(r'\(property "Reference" "([^"]+)"')
    fp_re = re.compile(r'\(property "Footprint" "([^"]*)"')
    out, pos, changes = [], 0, []
    for m in ref_re.finditer(s):
        ref = m.group(1)
        if not re.search(r'\d', ref):
            continue                      # lib_symbols template, not an instance
        fm = fp_re.search(s, m.end())
        if not fm:
            break
        old = fm.group(1)
        new = PER_REF.get(ref) or GLOBAL.get(old)
        if not new or new == old:
            continue
        out.append(s[pos:fm.start(1)]); out.append(new); pos = fm.end(1)
        changes.append((ref, old, new))
    out.append(s[pos:])
    from collections import Counter
    c = Counter((o, n) for _, o, n in changes)
    for (o, n), k in sorted(c.items(), key=lambda kv: -kv[1]):
        print(f'{k:4d}  {o} -> {n}')
    print(f'{len(changes)} footprint fields changed')
    if not a.dry_run:
        open(a.sch, 'w').write(''.join(out))
        print('written', a.sch)


if __name__ == '__main__':
    main()
