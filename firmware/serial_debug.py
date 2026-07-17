#!/usr/bin/env python3
"""Headless serial debug — send test commands, capture output."""
import serial
import sys
import time
import glob

def find_port():
    """Auto-detect serial port: prefer debug probe UART (2e8a:000c), then RP2350 CDC (2e8a:f00f)."""
    import subprocess
    candidates = {"debugger": None, "rp2350": None}
    for p in sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*")):
        try:
            out = subprocess.check_output(
                ["udevadm", "info", "--query=property", p],
                stderr=subprocess.DEVNULL, text=True)
            props = dict(line.split("=", 1) for line in out.splitlines() if "=" in line)
            vid = props.get("ID_USB_VENDOR_ID", props.get("ID_VENDOR_ID", ""))
            pid = props.get("ID_USB_MODEL_ID", props.get("ID_MODEL_ID", ""))
            if vid == "2e8a" and pid == "000c":
                candidates["debugger"] = p
            elif vid == "2e8a" and pid == "f00f":
                candidates["rp2350"] = p
        except Exception:
            continue
    if candidates["debugger"]:
        return candidates["debugger"]
    if candidates["rp2350"]:
        return candidates["rp2350"]
    ports = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
    return ports[0] if ports else None

def drain(ser, wait=0.5):
    """Read all available data after waiting."""
    time.sleep(wait)
    out = b""
    while ser.in_waiting:
        out += ser.read(ser.in_waiting)
        time.sleep(0.05)
    return out.decode("utf-8", errors="replace")

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if not port:
        print("No serial port found")
        sys.exit(1)

    print(f"Opening {port}...")
    ser = serial.Serial(port, 115200, timeout=1.0)

    # Wait longer for RP2350 USB CDC + boot init
    print("Waiting 8s for boot...")
    boot = drain(ser, wait=8)
    print(f"=== BOOT OUTPUT ({len(boot)} bytes) ===")
    print(boot if boot else "(empty)")
    print("=== END BOOT ===\n")

    # Test commands
    for cmd in ["V", "R"]:
        print(f">>> Sending: {cmd}")
        ser.write((cmd + "\n").encode())
        resp = drain(ser, wait=1.0)
        print(resp if resp else "(no response)")
        print()

    ser.close()

if __name__ == "__main__":
    main()
