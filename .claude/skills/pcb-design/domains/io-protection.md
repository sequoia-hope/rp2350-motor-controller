# Domain: I/O protection

Load for any board with connectors to the outside world (almost all of them).

## For every external connector (from `intent.md`)

Ask: what's the worst thing that could be plugged in, touched, or shorted here? Then protect against it.

- **ESD**: TVS diode on every signal and power pin that goes off-board, placed **right at the connector**, before anything else, with a short, wide path to ground. Choose by working voltage (standoff ≥ max normal signal), clamp voltage (below downstream abs max at the expected current), and capacitance (low for high-speed lines: USB, Ethernet need ESD arrays rated for that speed).
- **Power input**: reverse polarity (P-FET ideal diode, series Schottky for low current, or diode + fuse), overvoltage/surge TVS (consider load dump for automotive, inductive kicks), fuse or eFuse, inrush limiting for big bulk caps, hot-plug ringing on long cables (bulk electrolytic damping).
- **Signal lines**: series resistor for current limiting into ESD clamps and MCU pins; consider what happens if 12 V or 24 V is accidentally connected to a logic pin (miswired harness).
- **Shorts**: outputs that can be shorted to ground or supply should survive (current limit, PTC, short-circuit protected driver).
- **Communication buses** (CAN, RS-485): bus-specific protection parts, common-mode chokes, split termination.
- **Shield and chassis**: connector shell to ground via the right path (direct, RC, or isolated), decide and log.

## Layout

- Protection parts between the connector pins and everything else, so the surge sees the TVS first.
- Short, low-inductance ground path from TVS to the ground plane (multiple vias).
- Keep unprotected traces from the connector short and away from internal circuits.

## Silkscreen

Each connector's label includes ratings from `intent.md`, and any lack of protection the user should know about.
