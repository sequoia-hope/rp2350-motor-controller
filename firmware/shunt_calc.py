#!/usr/bin/env python3
"""Shunt resistor <-> max current calculator for INA240A1D (20x gain, 3.3V ADC)."""
import argparse

GAIN = 20.0
V_ADC_MAX = 3.3  # ADC reference voltage

def main():
    p = argparse.ArgumentParser(description=__doc__)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("-r", "--resistance", type=float, help="Shunt resistance in mOhm")
    g.add_argument("-i", "--current", type=float, help="Max current in amps")
    p.add_argument("-p", "--max-power", type=float, default=None, help="Max resistor power in watts (warn if exceeded)")
    args = p.parse_args()

    if args.resistance:
        r = args.resistance / 1000.0
        i_max = V_ADC_MAX / (GAIN * r)
        power = i_max ** 2 * r
        print(f"Shunt:  {args.resistance} mOhm")
        print(f"Imax:   {i_max:.2f} A")
        print(f"Power:  {power:.2f} W @ Imax")
    else:
        i_max = args.current
        r = V_ADC_MAX / (GAIN * i_max)
        power = i_max ** 2 * r
        print(f"Imax:   {i_max:.2f} A")
        print(f"Shunt:  {r * 1000:.3f} mOhm")
        print(f"Power:  {power:.2f} W @ Imax")

    if args.max_power and power > args.max_power:
        print(f"WARNING: exceeds {args.max_power} W limit by {power - args.max_power:.2f} W")

if __name__ == "__main__":
    main()
