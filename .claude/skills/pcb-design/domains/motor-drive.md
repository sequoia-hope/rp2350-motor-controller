# Domain: motor drives (BLDC / brushed / stepper power stages)

Load when the board has a half-bridge, full bridge or three-phase inverter.

## Parts (phase 1)

- **MOSFETs**: Vds ≥ 1.5× max bus including regen spikes; Rds(on) at operating Tj (≈1.5–2× the 25 °C value); total loss = conduction + switching + body-diode/dead-time loss; junction temperature with real thermal path, not datasheet ideal. Check SOA for startup and stall.
- **Gate driver**: drive current from Qg and target rise/fall; bootstrap cap ≥ 10× Qg/ΔV allowed, check minimum on-time and max duty for bootstrap refresh; dead time; desat/OC protection thresholds; undervoltage lockout.
- **Gate resistors**: separate turn-on / turn-off if needed (diode), damp ringing; check driver current limit.
- **Bulk capacitance**: **ripple current is usually the limiting factor, not capacitance.** Estimate RMS ripple current (for three-phase sinusoidal ≈ 0.3–0.5× phase RMS current at worst modulation index; compute for the actual case), split across caps, check each cap's rated ripple at switching frequency and temperature, and its lifetime. Polymer/hybrid or many paralleled electrolytics; plus ceramics across each half-bridge for the high-frequency part.
- **Shunts**: power at peak current, pulse rating, Kelvin footprint (4-terminal or Kelvin pads), inductance; amplifier common mode range including negative phase transients.
- **Bus voltage protection**: regen clamp / brake chopper or TVS sized for regen energy, and what happens if the supply can't sink current.
- **Inrush / pre-charge** into the bulk cap bank on connection.
- **Connectors and wires** rated for continuous phase current with derating.

## Placement and routing

- Commutation loop (high FET, low FET, ceramic caps, return) as small as physically possible. Ceramic decoupling directly across each half-bridge.
- Bus and phase copper as pours on multiple layers, stitched with **via arrays computed from current** (`nets.csv`), not single vias.
- Gate drive loops short, with the return referenced to the FET source (Kelvin source if the package has it).
- Shunt sense traces Kelvin-connected at the shunt pads, routed as a tight pair to the amplifier, away from phase nodes.
- Logic and analog ground: single solid plane, with power stage placed so its return currents don't flow through the analog region. Avoid splitting the plane unless there's a specific reason.
- Thermal: copper area and thermal vias under FET tabs; consider heatsink mounting and its isolation.
- Phase outputs to connectors with the shortest high-current path; high-dV/dt nodes away from edges and sensitive signals.

## Simulation

- SPICE of one half-bridge with gate driver model and estimated layout parasitics (loop inductance from placement): ringing, overshoot on Vds, dead time.
- Bus cap ripple current over the operating range.
- Thermal estimate of FETs and shunts.
