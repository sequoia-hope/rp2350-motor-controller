#!/usr/bin/env python3
"""
Live debug inspection of motor firmware via OpenOCD + GDB.

Connects to the CMSIS-DAP debugger, reads firmware state without
reflashing. Uses GDB batch mode for type-aware memory reads.

Usage:
    python3 debug.py              # one-shot dump of all state
    python3 debug.py adc          # raw ADC DMA buffer
    python3 debug.py currents     # current sense offsets + gains
    python3 debug.py motor        # motor state (voltages, currents, target)
    python3 debug.py foc          # full FOC state
    python3 debug.py watch [ms]   # continuous polling (default 500ms)
    python3 debug.py regs         # ADC hardware registers
    python3 debug.py snap         # halt CPU, snapshot full signal chain
    python3 debug.py trap [ms]    # wait ms, halt, dump signal chain, resume
    python3 debug.py resume       # resume after halt
    python3 debug.py openocd      # just start OpenOCD server (background)
    python3 debug.py flash        # build + flash via SWD

    # General debugging
    python3 debug.py status       # halt, backtrace, PC, thread info
    python3 debug.py where        # halt, backtrace (no reset)
    python3 debug.py vars         # read key firmware globals
    python3 debug.py usb          # read USB controller HW registers
    python3 debug.py mem ADDR [N] # read N words at address
    python3 debug.py break FUNC   # breakpoint, reset, run to FUNC, inspect
    python3 debug.py run-to FUNC  # build+flash+run to FUNC
    python3 debug.py reset        # reset and run
    python3 debug.py flash-check  # build+flash+check USB enumeration
"""

import subprocess
import sys
import os
import time
import socket
import struct
import re
from pathlib import Path

# --- Paths ---
OPENOCD = os.path.expanduser(
    "~/.platformio/packages/tool-openocd-rp2040-earlephilhower/bin/openocd"
)
OPENOCD_SCRIPTS = os.path.expanduser(
    "~/.platformio/packages/tool-openocd-rp2040-earlephilhower/share/openocd/scripts"
)
GDB = os.path.expanduser(
    "~/.platformio/packages/toolchain-rp2040-earlephilhower/bin/arm-none-eabi-gdb"
)
NM = os.path.expanduser(
    "~/.platformio/packages/toolchain-rp2040-earlephilhower/bin/arm-none-eabi-nm"
)
ELF = Path(__file__).parent / ".pio/build/motor_controller/firmware.elf"
PROJECT_DIR = Path(__file__).parent

# --- OpenOCD ports ---
OPENOCD_GDB_PORT = 3333
OPENOCD_TELNET_PORT = 4444

# --- RP2350 ADC hardware registers ---
ADC_BASE = 0x400A0000


def check_openocd_running():
    """Check if OpenOCD is already listening on the GDB port."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        s.connect(("localhost", OPENOCD_GDB_PORT))
        s.close()
        return True
    except (ConnectionRefusedError, OSError):
        return False


def start_openocd():
    """Start OpenOCD as a background process. Returns proc or None if already running."""
    if check_openocd_running():
        return None

    print("[openocd] Starting...")
    proc = subprocess.Popen(
        [
            OPENOCD,
            "-s", OPENOCD_SCRIPTS,
            "-f", "interface/cmsis-dap.cfg",
            "-c", "adapter speed 5000",
            "-f", "target/rp2350.cfg",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    for _ in range(30):
        time.sleep(0.2)
        if check_openocd_running():
            print("[openocd] Ready (pid=%d)" % proc.pid)
            return proc
        if proc.poll() is not None:
            err = proc.stderr.read().decode()
            print("[openocd] Failed to start:\n" + err)
            sys.exit(1)

    print("[openocd] Timeout waiting for server")
    proc.kill()
    sys.exit(1)


def ensure_openocd():
    """Ensure OpenOCD is running, start if needed."""
    if not check_openocd_running():
        proc = start_openocd()
        if not proc:
            print("[error] Could not start OpenOCD")
            sys.exit(1)


def get_symbol_addresses():
    """Extract key symbol addresses from the ELF using nm."""
    if not ELF.exists():
        print(f"[error] ELF not found: {ELF}")
        print("        Run: pio run -e motor_controller")
        sys.exit(1)

    result = subprocess.run(
        [NM, str(ELF)], capture_output=True, text=True
    )
    symbols = {}
    for line in result.stdout.split("\n"):
        parts = line.split()
        if len(parts) >= 3:
            addr, typ, name = parts[0], parts[1], parts[2]
            # Demangle the names we care about
            if "_ZL6engine" in name:
                symbols["engine"] = int(addr, 16)
            elif "_ZL5motor" in name and "FOCMotor" not in name:
                symbols["motor"] = int(addr, 16)
            elif "_ZL13current_sense" in name:
                symbols["current_sense"] = int(addr, 16)
            elif "_ZL14hw_initialized" in name:
                symbols["hw_initialized"] = int(addr, 16)
            elif "_ZL9foc_ready" in name:
                symbols["foc_ready"] = int(addr, 16)
            elif "_ZL6driver" in name:
                symbols["driver"] = int(addr, 16)
    return symbols


def gdb_batch(*commands, timeout=15):
    """
    Run GDB in batch mode against the live target.
    Uses a temp script file to support if/else/end and multiline commands.
    Returns stdout as string.
    """
    if not ELF.exists():
        print(f"[error] ELF not found: {ELF}")
        print("        Run: pio run -e motor_controller")
        sys.exit(1)

    import tempfile
    script = "set pagination off\nset print pretty on\n"
    script += f"file {ELF}\n"
    script += f"target remote localhost:{OPENOCD_GDB_PORT}\n"
    for c in commands:
        script += c + "\n"
    script += "detach\nquit\n"

    with tempfile.NamedTemporaryFile(mode='w', suffix='.gdb', delete=False) as f:
        f.write(script)
        script_path = f.name

    try:
        result = subprocess.run(
            [GDB, "--batch", "--nx", "-x", script_path],
            capture_output=True, text=True, timeout=timeout
        )
        return result.stdout
    finally:
        os.unlink(script_path)


def telnet_read_words(addr, count=1):
    """Read 32-bit words via OpenOCD telnet (non-halting)."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2.0)
        s.connect(("localhost", OPENOCD_TELNET_PORT))
        # Consume banner/prompt
        s.recv(4096)
        cmd = f"mdw 0x{addr:08x} {count}\n"
        s.sendall(cmd.encode())
        time.sleep(0.1)
        resp = s.recv(4096).decode()
        s.close()
        # Parse hex values after the colon
        values = []
        for m in re.finditer(r':\s*((?:[0-9a-fA-F]{8}\s*)+)', resp):
            for word in m.group(1).split():
                if len(word) == 8:
                    values.append(int(word, 16))
        return values
    except Exception as e:
        print(f"[telnet error: {e}]")
        return []


def read_u32_telnet(addr):
    """Read single 32-bit word via telnet."""
    vals = telnet_read_words(addr, 1)
    return vals[0] if vals else 0


# --- High-level inspection commands ---


def cmd_adc():
    """Dump ADC engine state: raw DMA samples, channel config."""
    syms = get_symbol_addresses()
    eng_ptr_addr = syms.get("engine")
    if not eng_ptr_addr:
        print("[error] 'engine' symbol not found in ELF")
        return

    print("=== ADC Engine (DMA ring buffer) ===\n")
    output = gdb_batch(
        f"set $eng = *(void**)0x{eng_ptr_addr:x}",
        "if $eng == 0",
        "  printf \"ENGINE NOT INITIALIZED (run H command first)\\n\"",
        "else",
        "  printf \"initialized:  %d\\n\", ((RP2040ADCEngine*)$eng)->initialized",
        "  printf \"channelCount: %d\\n\", ((RP2040ADCEngine*)$eng)->channelCount",
        "  printf \"adc_conv:     %f\\n\", ((RP2040ADCEngine*)$eng)->adc_conv",
        "  printf \"DMA channel:  %d\\n\", ((RP2040ADCEngine*)$eng)->readDMAChannel",
        "  printf \"\\nRaw samples (DMA buffer):\\n\"",
        f"  printf \"  ch0=%d  ch1=%d  ch2=%d  ch3=%d\\n\", "
            "((RP2040ADCEngine*)$eng)->samples[0], "
            "((RP2040ADCEngine*)$eng)->samples[1], "
            "((RP2040ADCEngine*)$eng)->samples[2], "
            "((RP2040ADCEngine*)$eng)->samples[3]",
        f"  printf \"  ch4=%d  ch5=%d  ch6=%d  ch7=%d\\n\", "
            "((RP2040ADCEngine*)$eng)->samples[4], "
            "((RP2040ADCEngine*)$eng)->samples[5], "
            "((RP2040ADCEngine*)$eng)->samples[6], "
            "((RP2040ADCEngine*)$eng)->samples[7]",
        "  printf \"\\nChannel enabled: \"",
        "  output ((RP2040ADCEngine*)$eng)->channelsEnabled",
        "  printf \"\\nChannel slots:  \"",
        "  output ((RP2040ADCEngine*)$eng)->channelSlot",
        "  printf \"\\n\"",
        "end",
    )
    print(clean_output(output))


def cmd_currents():
    """Dump current sense state: offsets, gains, live readings."""
    syms = get_symbol_addresses()
    cs_addr = syms.get("current_sense")
    if not cs_addr:
        print("[error] 'current_sense' symbol not found")
        return

    print("=== Current Sense ===\n")
    output = gdb_batch(
        f"set $cs = *(InlineCurrentSense**)0x{cs_addr:x}",
        "printf \"Offsets (calibration):\\n\"",
        "printf \"  offset_ia = %f\\n\", $cs->offset_ia",
        "printf \"  offset_ib = %f\\n\", $cs->offset_ib",
        "printf \"  offset_ic = %f\\n\", $cs->offset_ic",
        "printf \"\\nGains (A/V, includes shunt+amp):\\n\"",
        "printf \"  gain_a = %f\\n\", $cs->gain_a",
        "printf \"  gain_b = %f\\n\", $cs->gain_b",
        "printf \"  gain_c = %f\\n\", $cs->gain_c",
        "printf \"\\nPin assignments:\\n\"",
        "printf \"  pinA = %d\\n\", $cs->pinA",
        "printf \"  pinB = %d\\n\", $cs->pinB",
        "printf \"  pinC = %d\\n\", $cs->pinC",
        "printf \"\\nDriver-aligned flags:\\n\"",
        "printf \"  skip_align = %d\\n\", $cs->skip_align",
    )
    print(clean_output(output))


def cmd_motor():
    """Dump motor state."""
    syms = get_symbol_addresses()
    motor_addr = syms.get("motor")
    if not motor_addr:
        print("[error] 'motor' symbol not found")
        return

    print("=== Motor State ===\n")
    output = gdb_batch(
        f"set $m = *(BLDCMotor**)0x{motor_addr:x}",
        "printf \"enabled:      %d\\n\", $m->enabled",
        "printf \"controller:   %d\\n\", $m->controller",
        "printf \"torque_ctrl:  %d\\n\", $m->torque_controller",
        "printf \"target:       %f\\n\", $m->target",
        "printf \"shaft_angle:  %f\\n\", $m->shaft_angle",
        "printf \"elec_angle:   %f\\n\", $m->electrical_angle",
        "printf \"velocity:     %f\\n\", $m->shaft_velocity",
        "printf \"\\nDQ Currents:\\n\"",
        "printf \"  Iq = %f\\n\", $m->current.q",
        "printf \"  Id = %f\\n\", $m->current.d",
        "printf \"\\nDQ Voltages:\\n\"",
        "printf \"  Vq = %f\\n\", $m->voltage.q",
        "printf \"  Vd = %f\\n\", $m->voltage.d",
        "printf \"\\nLimits:\\n\"",
        "printf \"  voltage_limit  = %f\\n\", $m->voltage_limit",
        "printf \"  current_limit  = %f\\n\", $m->current_limit",
        "printf \"  velocity_limit = %f\\n\", $m->velocity_limit",
    )
    print(clean_output(output))


def cmd_foc():
    """Full FOC state: motor + currents + PIDs + ADC."""
    syms = get_symbol_addresses()
    motor_addr = syms.get("motor")
    cs_addr = syms.get("current_sense")
    eng_addr = syms.get("engine")

    if not all([motor_addr, cs_addr, eng_addr]):
        print("[error] Missing symbols. Run: pio run -e motor_controller")
        return

    print("=== Full FOC State ===\n")
    output = gdb_batch(
        f"set $m = *(BLDCMotor**)0x{motor_addr:x}",
        f"set $cs = *(InlineCurrentSense**)0x{cs_addr:x}",
        f"set $eng = *(void**)0x{eng_addr:x}",
        # Motor
        "printf \"--- Motor ---\\n\"",
        "printf \"  enabled=%d  target=%f\\n\", $m->enabled, $m->target",
        "printf \"  angle=%f  elec_angle=%f  velocity=%f\\n\", $m->shaft_angle, $m->electrical_angle, $m->shaft_velocity",
        "printf \"  Iq=%f  Id=%f  Vq=%f  Vd=%f\\n\", $m->current.q, $m->current.d, $m->voltage.q, $m->voltage.d",
        # PIDs
        "printf \"\\n--- PID Current Q ---\\n\"",
        "printf \"  P=%f  I=%f  D=%f\\n\", $m->PID_current_q.P, $m->PID_current_q.I, $m->PID_current_q.D",
        "printf \"  output_ramp=%f  limit=%f\\n\", $m->PID_current_q.output_ramp, $m->PID_current_q.limit",
        "printf \"  integral_prev=%f  error_prev=%f  output_prev=%f\\n\", $m->PID_current_q.integral_prev, $m->PID_current_q.error_prev, $m->PID_current_q.output_prev",
        "printf \"\\n--- PID Current D ---\\n\"",
        "printf \"  P=%f  I=%f  D=%f\\n\", $m->PID_current_d.P, $m->PID_current_d.I, $m->PID_current_d.D",
        "printf \"  integral_prev=%f  error_prev=%f  output_prev=%f\\n\", $m->PID_current_d.integral_prev, $m->PID_current_d.error_prev, $m->PID_current_d.output_prev",
        "printf \"\\n--- PID Velocity ---\\n\"",
        "printf \"  P=%f  I=%f  D=%f\\n\", $m->PID_velocity.P, $m->PID_velocity.I, $m->PID_velocity.D",
        "printf \"  output_ramp=%f  limit=%f\\n\", $m->PID_velocity.output_ramp, $m->PID_velocity.limit",
        "printf \"  integral_prev=%f  error_prev=%f  output_prev=%f\\n\", $m->PID_velocity.integral_prev, $m->PID_velocity.error_prev, $m->PID_velocity.output_prev",
        # LPFs
        "printf \"\\n--- LPFs ---\\n\"",
        "printf \"  LPF_Iq: Tf=%f  y_prev=%f\\n\", $m->LPF_current_q.Tf, $m->LPF_current_q.y_prev",
        "printf \"  LPF_Id: Tf=%f  y_prev=%f\\n\", $m->LPF_current_d.Tf, $m->LPF_current_d.y_prev",
        "printf \"  LPF_vel: Tf=%f  y_prev=%f\\n\", $m->LPF_velocity.Tf, $m->LPF_velocity.y_prev",
        # Current sense
        "printf \"\\n--- Current Sense ---\\n\"",
        "printf \"  offsets: a=%f  b=%f  c=%f\\n\", $cs->offset_ia, $cs->offset_ib, $cs->offset_ic",
        "printf \"  gains:   a=%f  b=%f  c=%f\\n\", $cs->gain_a, $cs->gain_b, $cs->gain_c",
        "printf \"  pins:    A=%d  B=%d  C=%d\\n\", $cs->pinA, $cs->pinB, $cs->pinC",
        # ADC raw
        "printf \"\\n--- ADC Raw ---\\n\"",
        "if $eng != 0",
        f"  printf \"  samples: [%d, %d, %d, %d, %d, %d, %d, %d]\\n\", "
            "((RP2040ADCEngine*)$eng)->samples[0], "
            "((RP2040ADCEngine*)$eng)->samples[1], "
            "((RP2040ADCEngine*)$eng)->samples[2], "
            "((RP2040ADCEngine*)$eng)->samples[3], "
            "((RP2040ADCEngine*)$eng)->samples[4], "
            "((RP2040ADCEngine*)$eng)->samples[5], "
            "((RP2040ADCEngine*)$eng)->samples[6], "
            "((RP2040ADCEngine*)$eng)->samples[7]",
        f"  printf \"  channelSlot: [%d, %d, %d, %d, %d, %d, %d, %d]\\n\", "
            "((RP2040ADCEngine*)$eng)->channelSlot[0], "
            "((RP2040ADCEngine*)$eng)->channelSlot[1], "
            "((RP2040ADCEngine*)$eng)->channelSlot[2], "
            "((RP2040ADCEngine*)$eng)->channelSlot[3], "
            "((RP2040ADCEngine*)$eng)->channelSlot[4], "
            "((RP2040ADCEngine*)$eng)->channelSlot[5], "
            "((RP2040ADCEngine*)$eng)->channelSlot[6], "
            "((RP2040ADCEngine*)$eng)->channelSlot[7]",
        "else",
        "  printf \"  [engine not initialized - run H first]\\n\"",
        "end",
    )
    print(clean_output(output))


def cmd_regs():
    """Read ADC hardware registers directly via OpenOCD telnet."""
    print("=== ADC Hardware Registers (0x%08X) ===" % ADC_BASE)

    # Read registers via telnet (doesn't halt CPU)
    vals = telnet_read_words(ADC_BASE, 6)
    if len(vals) < 6:
        print("[error] Could not read ADC registers via telnet")
        print("  Try: python3 debug.py openocd  (ensure server is running)")
        return

    cs, result, fcs, fifo, div_reg, intr = vals[:6]
    print(f"  CS     = 0x{cs:08x}")
    print(f"    EN={cs & 1}, START={cs>>1 & 1}, READY={cs>>8 & 1}")
    print(f"    AINSEL={cs>>12 & 0xF}, RROBIN=0x{cs>>16 & 0xFF:02x}")
    print(f"  RESULT = 0x{result:08x} ({result & 0xFFF} raw, ~{(result & 0xFFF) * 3.3/4096:.3f}V)")
    print(f"  FCS    = 0x{fcs:08x}")
    print(f"    EN={fcs & 1}, DREQ_EN={fcs>>3 & 1}, LEVEL={fcs>>16 & 0xF}")
    print(f"    EMPTY={fcs>>8 & 1}, FULL={fcs>>9 & 1}")
    print(f"  DIV    = 0x{div_reg:08x} (frac={div_reg & 0xFF}/256, int={div_reg >> 8})")
    print(f"  INTR   = 0x{intr:08x}")

    # DMA state
    syms = get_symbol_addresses()
    eng_addr = syms.get("engine")
    if eng_addr:
        output = gdb_batch(
            f"set $eng = *(void**)0x{eng_addr:x}",
            "if $eng != 0",
            "  printf \"DMA_CH=%d\\n\", ((RP2040ADCEngine*)$eng)->readDMAChannel",
            "else",
            "  printf \"DMA_CH=NONE\\n\"",
            "end",
        )
        for line in clean_output(output).split("\n"):
            if "DMA_CH=" in line:
                val = line.split("=")[1]
                if val == "NONE":
                    print("\n  [DMA not configured - engine not initialized]")
                    break
                dma_ch = int(val)
                print(f"\n=== DMA Channel {dma_ch} ===")
                # RP2350 DMA base: 0x50000000
                dma_base = 0x50000000 + dma_ch * 0x40
                dma_vals = telnet_read_words(dma_base, 5)
                if len(dma_vals) >= 5:
                    print(f"  READ_ADDR   = 0x{dma_vals[0]:08x}")
                    print(f"  WRITE_ADDR  = 0x{dma_vals[1]:08x}")
                    print(f"  TRANS_COUNT = 0x{dma_vals[2]:08x}")
                    # ctrl_trig is at offset 0x0c, ctrl is alias at 0x10
                    ctrl = dma_vals[3]
                    print(f"  CTRL_TRIG   = 0x{ctrl:08x}")
                    ring_size = (ctrl >> 6) & 0xF
                    ring_sel = (ctrl >> 10) & 1
                    treq = (ctrl >> 15) & 0x3F
                    busy = (ctrl >> 24) & 1
                    en = ctrl & 1
                    print(f"    EN={en}, RING_SIZE={ring_size} ({1<<ring_size} bytes), RING_SEL={'write' if ring_sel else 'read'}")
                    print(f"    TREQ_SEL=0x{treq:02x}, BUSY={busy}")


def cmd_watch(interval_ms=500):
    """Continuous polling of key values."""
    syms = get_symbol_addresses()
    motor_addr = syms.get("motor")
    eng_addr = syms.get("engine")
    cs_addr = syms.get("current_sense")

    if not motor_addr:
        print("[error] symbols not found")
        return

    print("=== Watch Mode (Ctrl+C to stop, %dms interval) ===" % interval_ms)
    header = f"{'time':>8} | {'raw0':>5} {'raw1':>5} {'raw2':>5} | {'Iq':>7} {'Id':>7} | {'Vq':>7} {'Vd':>7} | {'target':>7} {'vel':>7}"
    print(header)
    print("-" * len(header))

    try:
        while True:
            # Build GDB commands for a single read
            cmds = [
                f"set $m = *(BLDCMotor**)0x{motor_addr:x}",
            ]
            if eng_addr:
                cmds.append(f"set $eng = *(void**)0x{eng_addr:x}")
                cmds += [
                    "if $eng != 0",
                    "  printf \"%d %d %d\\n\", "
                    "((RP2040ADCEngine*)$eng)->samples[0], "
                    "((RP2040ADCEngine*)$eng)->samples[1], "
                    "((RP2040ADCEngine*)$eng)->samples[2]",
                    "else",
                    "  printf \"0 0 0\\n\"",
                    "end",
                ]
            else:
                cmds.append("printf \"0 0 0\\n\"")

            cmds.append(
                "printf \"%.4f %.4f %.4f %.4f %.4f %.4f\\n\", "
                "$m->current.q, $m->current.d, "
                "$m->voltage.q, $m->voltage.d, "
                "$m->target, $m->shaft_velocity"
            )

            output = gdb_batch(*cmds)
            lines = [l for l in clean_output(output).split("\n") if l.strip()]

            # Parse the two printf lines
            try:
                raw_line = None
                float_line = None
                for l in lines:
                    parts = l.split()
                    if len(parts) == 3 and all(p.lstrip('-').isdigit() for p in parts):
                        raw_line = parts
                    elif len(parts) == 6:
                        try:
                            [float(x) for x in parts]
                            float_line = parts
                        except ValueError:
                            pass

                if float_line:
                    ts = time.strftime("%H:%M:%S")
                    r0, r1, r2 = raw_line if raw_line else ["0", "0", "0"]
                    iq, id_, vq, vd, tgt, vel = [float(x) for x in float_line]
                    print(f"{ts:>8} | {r0:>5} {r1:>5} {r2:>5} | "
                          f"{iq:>7.3f} {id_:>7.3f} | "
                          f"{vq:>7.3f} {vd:>7.3f} | "
                          f"{tgt:>7.3f} {vel:>7.3f}")
                else:
                    print(f"  [parse: {lines}]")
            except (ValueError, IndexError) as e:
                print(f"  [parse error: {e}]")

            time.sleep(interval_ms / 1000.0)
    except KeyboardInterrupt:
        print("\n[stopped]")


def kill_openocd():
    """Kill any running OpenOCD processes to release the debugger."""
    subprocess.run(["pkill", "-f", "openocd.*rp2350"], capture_output=True)
    # Wait for port to free up
    for _ in range(10):
        if not check_openocd_running():
            return
        time.sleep(0.2)


def cmd_snap():
    """
    Halt the CPU and snapshot the full current-sense signal chain.
    Useful for catching mid-FOC-loop state to debug phantom readings.
    """
    syms = get_symbol_addresses()
    motor_addr = syms.get("motor")
    cs_addr = syms.get("current_sense")
    eng_addr = syms.get("engine")

    if not all([motor_addr, cs_addr, eng_addr]):
        print("[error] Missing symbols")
        return

    print("=== CPU Snapshot (halted) ===\n")
    output = gdb_batch(
        # Halt the CPU for a consistent read
        "monitor halt",
        f"set $m = *(BLDCMotor**)0x{motor_addr:x}",
        f"set $cs = *(InlineCurrentSense**)0x{cs_addr:x}",
        f"set $eng = *(void**)0x{eng_addr:x}",
        # Signal chain: raw ADC → offset → gain → phase current → Clarke → Park
        "printf \"--- Raw ADC (DMA buffer) ---\\n\"",
        "if $eng != 0",
        f"  printf \"  raw[0]=%d  raw[1]=%d  raw[2]=%d\\n\", "
            "((RP2040ADCEngine*)$eng)->samples[0], "
            "((RP2040ADCEngine*)$eng)->samples[1], "
            "((RP2040ADCEngine*)$eng)->samples[2]",
        f"  printf \"  adc_conv=%f\\n\", ((RP2040ADCEngine*)$eng)->adc_conv",
        "else",
        "  printf \"  [engine NULL]\\n\"",
        "end",
        "printf \"\\n--- Current Sense ---\\n\"",
        "printf \"  offsets:  a=%f  b=%f  c=%f\\n\", $cs->offset_ia, $cs->offset_ib, $cs->offset_ic",
        "printf \"  gains:    a=%f  b=%f  c=%f\\n\", $cs->gain_a, $cs->gain_b, $cs->gain_c",
        # Compute what getPhaseCurrents() would return:
        # phase_current = (raw * adc_conv - offset) * gain
        "if $eng != 0",
        f"  set $raw_a = ((RP2040ADCEngine*)$eng)->samples[((RP2040ADCEngine*)$eng)->channelSlot[2]]",
        f"  set $raw_b = ((RP2040ADCEngine*)$eng)->samples[((RP2040ADCEngine*)$eng)->channelSlot[1]]",
        f"  set $raw_c = ((RP2040ADCEngine*)$eng)->samples[((RP2040ADCEngine*)$eng)->channelSlot[0]]",
        f"  set $conv = ((RP2040ADCEngine*)$eng)->adc_conv",
        "  set $va = (float)$raw_a * $conv",
        "  set $vb = (float)$raw_b * $conv",
        "  set $vc = (float)$raw_c * $conv",
        "  printf \"\\n--- Computed (raw * adc_conv) ---\\n\"",
        "  printf \"  Va=%f  Vb=%f  Vc=%f\\n\", $va, $vb, $vc",
        "  printf \"\\n--- After offset subtraction ---\\n\"",
        "  set $da = $va - $cs->offset_ia",
        "  set $db = $vb - $cs->offset_ib",
        "  set $dc = $vc - $cs->offset_ic",
        "  printf \"  dVa=%f  dVb=%f  dVc=%f\\n\", $da, $db, $dc",
        "  printf \"\\n--- After gain (= phase currents) ---\\n\"",
        "  printf \"  Ia=%f  Ib=%f  Ic=%f\\n\", $da * $cs->gain_a, $db * $cs->gain_b, $dc * $cs->gain_c",
        "end",
        "printf \"\\n--- Motor DQ state ---\\n\"",
        "printf \"  current: Iq=%f  Id=%f\\n\", $m->current.q, $m->current.d",
        "printf \"  voltage: Vq=%f  Vd=%f\\n\", $m->voltage.q, $m->voltage.d",
        "printf \"  elec_angle=%f  target=%f\\n\", $m->electrical_angle, $m->target",
        "printf \"\\n--- LPF state ---\\n\"",
        "printf \"  LPF_Iq.y_prev=%f  LPF_Id.y_prev=%f\\n\", $m->LPF_current_q.y_prev, $m->LPF_current_d.y_prev",
        # Resume
        "monitor resume",
    )
    print(clean_output(output))


def cmd_trap(delay_ms=200):
    """
    Wait, then halt CPU mid-FOC-loop and dump the signal chain.

    Workflow:
      1. In terminal 1: start step test (Sq0.5 via serial or dashboard)
      2. In terminal 2: python3 debug.py trap [delay_ms]
         - Waits delay_ms (default 200) for the FOC loop to be running
         - Halts the CPU via OpenOCD
         - Dumps full signal chain (raw ADC → offset → gain → phase → DQ)
         - Resumes the CPU

    The phantom ~1A appears by t≈25ms so default 200ms catches steady state.
    """
    syms = get_symbol_addresses()
    motor_addr = syms.get("motor")
    cs_addr = syms.get("current_sense")
    eng_addr = syms.get("engine")

    if not all([motor_addr, cs_addr, eng_addr]):
        print("[error] Missing symbols")
        return

    print(f"[trap] Waiting {delay_ms}ms then halting CPU...")
    time.sleep(delay_ms / 1000.0)

    # Halt via OpenOCD telnet (faster than GDB connect)
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2.0)
        s.connect(("localhost", OPENOCD_TELNET_PORT))
        s.recv(4096)
        s.sendall(b"halt\n")
        time.sleep(0.1)
        s.recv(4096)
        s.close()
    except Exception as e:
        print(f"[error] Could not halt via telnet: {e}")
        return

    print("[trap] CPU halted. Reading signal chain...\n")

    # Now read full state via GDB (CPU is already halted)
    output = gdb_batch(
        f"set $m = *(BLDCMotor**)0x{motor_addr:x}",
        f"set $cs = *(InlineCurrentSense**)0x{cs_addr:x}",
        f"set $eng = *(void**)0x{eng_addr:x}",
        # Signal chain
        "printf \"=== Signal Chain Snapshot ===\\n\\n\"",
        "if $eng != 0",
        f"  set $slot0 = ((RP2040ADCEngine*)$eng)->channelSlot[0]",
        f"  set $slot1 = ((RP2040ADCEngine*)$eng)->channelSlot[1]",
        f"  set $slot2 = ((RP2040ADCEngine*)$eng)->channelSlot[2]",
        f"  set $conv = ((RP2040ADCEngine*)$eng)->adc_conv",
        # Raw DMA samples
        "  printf \"1. Raw ADC (DMA buffer):\\n\"",
        f"  printf \"   samples[] = [%d, %d, %d, %d]\\n\", "
            "((RP2040ADCEngine*)$eng)->samples[0], "
            "((RP2040ADCEngine*)$eng)->samples[1], "
            "((RP2040ADCEngine*)$eng)->samples[2], "
            "((RP2040ADCEngine*)$eng)->samples[3]",
        f"  printf \"   channelSlot = [%d, %d, %d]  (ch0→slot, ch1→slot, ch2→slot)\\n\", $slot0, $slot1, $slot2",
        # Which samples map to which phase (pinC=GPIO40=ch0, pinB=GPIO41=ch1, pinA=GPIO42=ch2)
        f"  set $raw_c = (int)((RP2040ADCEngine*)$eng)->samples[$slot0]",
        f"  set $raw_b = (int)((RP2040ADCEngine*)$eng)->samples[$slot1]",
        f"  set $raw_a = (int)((RP2040ADCEngine*)$eng)->samples[$slot2]",
        "  printf \"   Phase mapping: A(pin42/ch2)=%d  B(pin41/ch1)=%d  C(pin40/ch0)=%d\\n\", $raw_a, $raw_b, $raw_c",
        # Convert to voltage
        "  printf \"\\n2. ADC voltage (raw * adc_conv, conv=%f):\\n\", $conv",
        "  set $va = (float)$raw_a * $conv",
        "  set $vb = (float)$raw_b * $conv",
        "  set $vc = (float)$raw_c * $conv",
        "  printf \"   Va=%f  Vb=%f  Vc=%f\\n\", $va, $vb, $vc",
        # Subtract offset
        "  printf \"\\n3. After offset subtraction (offsets: a=%f b=%f c=%f):\\n\", $cs->offset_ia, $cs->offset_ib, $cs->offset_ic",
        "  set $da = $va - $cs->offset_ia",
        "  set $db = $vb - $cs->offset_ib",
        "  set $dc = $vc - $cs->offset_ic",
        "  printf \"   dVa=%f  dVb=%f  dVc=%f\\n\", $da, $db, $dc",
        # Apply gain
        "  printf \"\\n4. After gain (gains: a=%f b=%f c=%f):\\n\", $cs->gain_a, $cs->gain_b, $cs->gain_c",
        "  set $ia = $da * $cs->gain_a",
        "  set $ib = $db * $cs->gain_b",
        "  set $ic = $dc * $cs->gain_c",
        "  printf \"   Ia=%f  Ib=%f  Ic=%f  (phase currents)\\n\", $ia, $ib, $ic",
        "else",
        "  printf \"   [engine NULL - not initialized]\\n\"",
        "end",
        # Motor DQ state (what loopFOC computed)
        "printf \"\\n5. Motor DQ state (after Clarke+Park+LPF):\\n\"",
        "printf \"   current.q=%f  current.d=%f\\n\", $m->current.q, $m->current.d",
        "printf \"   voltage.q=%f  voltage.d=%f\\n\", $m->voltage.q, $m->voltage.d",
        "printf \"   elec_angle=%f  target=%f\\n\", $m->electrical_angle, $m->target",
        "printf \"\\n6. LPF state (y_prev = filtered output):\\n\"",
        "printf \"   LPF_Iq.y_prev=%f  LPF_Id.y_prev=%f\\n\", $m->LPF_current_q.y_prev, $m->LPF_current_d.y_prev",
        "printf \"\\n7. PID integrator state:\\n\"",
        "printf \"   PID_Iq: integral=%f  error=%f  output=%f\\n\", $m->PID_current_q.integral_prev, $m->PID_current_q.error_prev, $m->PID_current_q.output_prev",
        "printf \"   PID_Id: integral=%f  error=%f  output=%f\\n\", $m->PID_current_d.integral_prev, $m->PID_current_d.error_prev, $m->PID_current_d.output_prev",
        # Resume
        "monitor resume",
        "printf \"\\n[CPU resumed]\\n\"",
    )
    print(clean_output(output))


def openocd_oneshot(*cmds):
    """Run OpenOCD one-shot commands (no server needed). Returns filtered output."""
    args = [
        OPENOCD,
        "-s", OPENOCD_SCRIPTS,
        "-f", "interface/cmsis-dap.cfg",
        "-c", "adapter speed 5000",
        "-f", "target/rp2350.cfg",
    ]
    joined = "; ".join(cmds)
    args += ["-c", f"init; {joined}; shutdown"]
    result = subprocess.run(args, capture_output=True, text=True, timeout=15)
    output = result.stdout + result.stderr
    # Filter noise
    lines = []
    for line in output.splitlines():
        if line.startswith(("Info", "Warn", "Open On-Chip", "Licensed",
                           "For bug", "adapter", "cortex_m", "shutdown")):
            continue
        if line.strip():
            lines.append(line.strip())
    return "\n".join(lines)


def cmd_status():
    """Halt target, show PC, backtrace, thread info."""
    ensure_openocd()
    output = gdb_batch(
        "monitor halt",
        "info threads",
        "thread 1",
        "backtrace 15",
        "printf \"PC = 0x%08x\\n\", $pc",
        "printf \"SP = 0x%08x\\n\", $sp",
        "printf \"LR = 0x%08x\\n\", $lr",
    )
    print(clean_output(output))


def cmd_where():
    """Halt and show backtrace."""
    ensure_openocd()
    output = gdb_batch(
        "monitor halt",
        "thread 1",
        "backtrace 20",
    )
    print(clean_output(output))


def cmd_vars():
    """Read key firmware global variables."""
    ensure_openocd()
    output = gdb_batch(
        "monitor halt",
        "printf \"hw_initialized = %d\\n\", hw_initialized",
        "printf \"foc_ready      = %d\\n\", foc_ready",
        "printf \"enc_detected   = %d\\n\", enc_detected",
        "printf \"sine_running   = %d\\n\", sine_running",
        "printf \"driver         = %p\\n\", driver",
        "printf \"motor          = %p\\n\", motor",
        "printf \"encoder        = %p\\n\", encoder",
        "printf \"current_sense  = %p\\n\", current_sense",
        "printf \"commander      = %p\\n\", commander",
        "printf \"led            = %p\\n\", led",
        "#ifdef HAS_USB_PD",
        "printf \"pd_ufp         = %p\\n\", pd_ufp",
        "printf \"pd_initialized = %d\\n\", pd_initialized",
        "printf \"pd_ready       = %d\\n\", pd_ready",
        "printf \"pd_voltage_raw = %d\\n\", pd_voltage_raw",
        "printf \"pd_current_raw = %d\\n\", pd_current_raw",
        "#endif",
        "monitor resume",
    )
    print(clean_output(output))


def cmd_usb():
    """Read RP2350 USB controller hardware registers."""
    # Kill any running OpenOCD to avoid port conflicts
    if check_openocd_running():
        kill_openocd()
        time.sleep(0.5)

    output = openocd_oneshot(
        "halt",
        "mdw 0x50110040",   # MAIN_CTRL
        "mdw 0x50110050",   # SIE_STATUS
        "mdw 0x5011004c",   # SIE_CTRL
        "mdw 0x50110000",   # ADDR_ENDP
        "mdw 0x50110074",   # USB_MUXING
        "mdw 0x50110078",   # USBPHY_DIRECT
        "mdw 0x5011007c",   # USBPHY_DIRECT_OVERRIDE
        "mdw 0x50110090",   # INTE
        "mdw 0x50110098",   # INTS
        "resume",
    )
    # Parse the memory read results
    vals = {}
    reg_names = [
        "MAIN_CTRL", "SIE_STATUS", "SIE_CTRL", "ADDR_ENDP",
        "USB_MUXING", "USBPHY_DIRECT", "USBPHY_DIRECT_OVERRIDE",
        "INTE", "INTS",
    ]
    reg_addrs = [
        0x50110040, 0x50110050, 0x5011004c, 0x50110000,
        0x50110074, 0x50110078, 0x5011007c,
        0x50110090, 0x50110098,
    ]
    for line in output.splitlines():
        for addr, name in zip(reg_addrs, reg_names):
            prefix = f"0x{addr:08x}:"
            if prefix in line:
                val_str = line.split(":")[1].strip().split()[0]
                vals[name] = int(val_str, 16)

    print("=== RP2350 USB Controller Registers ===\n")
    if "MAIN_CTRL" in vals:
        v = vals["MAIN_CTRL"]
        print(f"MAIN_CTRL        = 0x{v:08x}")
        print(f"  CONTROLLER_EN  = {v & 1}")
        print(f"  HOST_NDEVICE   = {(v >> 1) & 1}")
    if "SIE_STATUS" in vals:
        v = vals["SIE_STATUS"]
        print(f"SIE_STATUS       = 0x{v:08x}")
        print(f"  VBUS_DETECTED  = {v & 1}")
        print(f"  SUSPENDED      = {(v >> 4) & 1}")
        print(f"  CONNECTED      = {(v >> 16) & 1}")
        speed = (v >> 8) & 0x3
        speed_names = {0: "disconnected", 1: "low-speed", 2: "full-speed", 3: "reserved"}
        print(f"  SPEED          = {speed} ({speed_names.get(speed, '?')})")
    if "SIE_CTRL" in vals:
        v = vals["SIE_CTRL"]
        print(f"SIE_CTRL         = 0x{v:08x}")
        print(f"  PULLUP_EN      = {(v >> 16) & 1}")
        print(f"  TRANSCEIVER_PD = {(v >> 18) & 1}")
    if "ADDR_ENDP" in vals:
        v = vals["ADDR_ENDP"]
        print(f"ADDR_ENDP        = 0x{v:08x}")
        print(f"  ADDRESS        = {v & 0x7f}")
    if "USB_MUXING" in vals:
        v = vals["USB_MUXING"]
        print(f"USB_MUXING       = 0x{v:08x}")
        print(f"  TO_PHY         = {v & 1}")
        print(f"  TO_EXTPHY      = {(v >> 1) & 1}")
        print(f"  TO_DIGITAL_PAD = {(v >> 2) & 1}")
        print(f"  SOFTCON        = {(v >> 3) & 1}")
    if "USBPHY_DIRECT" in vals:
        v = vals["USBPHY_DIRECT"]
        print(f"USBPHY_DIRECT    = 0x{v:08x}")
    if "USBPHY_DIRECT_OVERRIDE" in vals:
        v = vals["USBPHY_DIRECT_OVERRIDE"]
        print(f"USBPHY_OVERRIDE  = 0x{v:08x}")
    if "INTE" in vals:
        v = vals["INTE"]
        print(f"INTE             = 0x{v:08x}  (enabled interrupts)")
    if "INTS" in vals:
        v = vals["INTS"]
        print(f"INTS             = 0x{v:08x}  (pending interrupts)")

    if not vals:
        print("(failed to read registers)")
        print(output)


def cmd_mem(addr_str, count_str="4"):
    """Read memory words at address."""
    if check_openocd_running():
        kill_openocd()
        time.sleep(0.5)
    addr = int(addr_str, 0)
    count = int(count_str)
    output = openocd_oneshot("halt", f"mdw 0x{addr:08x} {count}", "resume")
    print(output)


def cmd_break_func(func_name):
    """Set breakpoint, reset, run to function, inspect state."""
    ensure_openocd()
    output = gdb_batch(
        "monitor reset halt",
        f"break {func_name}",
        "continue",
        "backtrace 10",
        "info locals",
        timeout=30,
    )
    print(clean_output(output))


def cmd_run_to(func_name):
    """Build, flash, set breakpoint, run to function, inspect."""
    cmd_flash()
    time.sleep(1)
    ensure_openocd()
    output = gdb_batch(
        "monitor reset halt",
        f"break {func_name}",
        "continue",
        "backtrace 10",
        "info locals",
        timeout=30,
    )
    print(clean_output(output))


def cmd_reset():
    """Reset and run the target."""
    if check_openocd_running():
        kill_openocd()
        time.sleep(0.5)
    output = openocd_oneshot("reset run")
    print("Target reset and running.")


def cmd_flash_check():
    """Build, flash, wait for boot, check USB enumeration."""
    cmd_flash()
    print("\n=== Waiting 5s for USB enumeration ===")
    time.sleep(5)

    import glob as globmod
    acms = sorted(globmod.glob("/dev/ttyACM*"))
    print(f"ttyACM devices: {acms if acms else '(none)'}")

    result = subprocess.run(["lsusb"], capture_output=True, text=True)
    rp_devs = [l for l in result.stdout.splitlines() if "2e8a" in l.lower()]
    print(f"Raspberry Pi USB devices: {rp_devs}")

    # Check USB registers
    print()
    cmd_usb()


def cmd_resume():
    """Resume execution after a halt (if snap was used without auto-resume)."""
    print("[resume] Continuing execution...")
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2.0)
        s.connect(("localhost", OPENOCD_TELNET_PORT))
        s.recv(4096)
        s.sendall(b"resume\n")
        time.sleep(0.1)
        s.close()
        print("  CPU resumed.")
    except Exception as e:
        print(f"[error] {e}")


def cmd_flash():
    """Build and flash firmware via SWD."""
    print("[build] pio run -e motor_controller...")
    result = subprocess.run(
        ["pio", "run", "-e", "motor_controller"],
        cwd=PROJECT_DIR, capture_output=True, text=True
    )
    if result.returncode != 0:
        print("[build] FAILED")
        print(result.stderr[-2000:] if result.stderr else result.stdout[-2000:])
        sys.exit(1)
    print("[build] OK")

    # Kill existing OpenOCD to free the debugger
    if check_openocd_running():
        print("[flash] Stopping existing OpenOCD...")
        kill_openocd()

    print("[flash] Programming via SWD...")
    result = subprocess.run(
        [
            OPENOCD,
            "-s", OPENOCD_SCRIPTS,
            "-f", "interface/cmsis-dap.cfg",
            "-c", "adapter speed 5000",
            "-f", "target/rp2350.cfg",
            "-c", f"program {ELF} verify reset exit",
        ],
        capture_output=True, text=True
    )
    if result.returncode != 0:
        print("[flash] FAILED")
        print(result.stderr[-2000:])
        sys.exit(1)
    print("[flash] Done. Firmware is running.")

    # Restart OpenOCD in background for subsequent debug reads
    time.sleep(0.5)
    start_openocd()


# --- Helpers ---


def clean_output(output):
    """Remove GDB connection noise, keep printf output and values."""
    lines = []
    for line in output.split("\n"):
        stripped = line.strip()
        # Skip GDB noise
        if any(x in stripped for x in [
            "Remote debugging", "warning:", "Reading symbols",
            "Loading", "Transfer", "Inferior",
            "vTaskEnterCritical", "spin_lock.h",
            "target halted", "set pagination",
            "set print", "xSemaphore", "mutex",
            "detach", "Detaching", "get_core_num",
            "pico_platform", "platform.h", "__core",
            "variantHooks", "optimized out",
        ]):
            continue
        # Skip empty and prompt lines
        if not stripped or stripped == "(gdb)":
            continue
        # Skip the initial PC display (hex address lines like "0x1000xxxx in ...")
        if re.match(r'^0x[0-9a-f]+ in', stripped):
            continue
        # Skip source-line context from GDB (like "71  xSemaphoreGive(mtx);")
        if re.match(r'^\d+\s+\S', stripped) and not stripped[0:1].isalpha():
            continue
        lines.append(stripped)
    return "\n".join(lines)


# --- Main ---


def main():
    args = sys.argv[1:]
    cmd = args[0] if args else "foc"

    if cmd == "help" or cmd == "-h" or cmd == "--help":
        print(__doc__)
        return

    if cmd == "flash":
        cmd_flash()
        return

    if cmd == "openocd":
        if check_openocd_running():
            print("[openocd] Already running")
        else:
            proc = start_openocd()
            if proc:
                print("[openocd] Running. Ctrl+C to stop.")
                try:
                    proc.wait()
                except KeyboardInterrupt:
                    proc.terminate()
        return

    # All other commands need OpenOCD
    ensure_openocd()

    if cmd == "adc":
        cmd_adc()
    elif cmd == "currents":
        cmd_currents()
    elif cmd == "motor":
        cmd_motor()
    elif cmd == "foc":
        cmd_foc()
    elif cmd == "regs":
        cmd_regs()
    elif cmd == "snap":
        cmd_snap()
    elif cmd == "trap":
        delay = int(args[1]) if len(args) > 1 else 200
        cmd_trap(delay)
    elif cmd == "resume":
        cmd_resume()
    elif cmd == "watch":
        interval = int(args[1]) if len(args) > 1 else 500
        cmd_watch(interval)
    elif cmd == "status":
        cmd_status()
    elif cmd == "where":
        cmd_where()
    elif cmd == "vars":
        cmd_vars()
    elif cmd == "usb":
        cmd_usb()
    elif cmd == "mem":
        if len(args) < 2:
            print("Usage: debug.py mem ADDR [COUNT]")
            sys.exit(1)
        cmd_mem(args[1], args[2] if len(args) > 2 else "4")
    elif cmd == "break":
        if len(args) < 2:
            print("Usage: debug.py break FUNC_NAME")
            sys.exit(1)
        cmd_break_func(args[1])
    elif cmd == "run-to":
        if len(args) < 2:
            print("Usage: debug.py run-to FUNC_NAME")
            sys.exit(1)
        cmd_run_to(args[1])
    elif cmd == "reset":
        cmd_reset()
    elif cmd == "flash-check":
        cmd_flash_check()
    else:
        print(f"Unknown command: {cmd}")
        print(__doc__)
        sys.exit(1)


if __name__ == "__main__":
    main()
