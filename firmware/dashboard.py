#!/usr/bin/env python3
"""
RP2350 Debug Dashboard — interactive web page showing all hardware state
captured via SWD debug probe and picotool.

Usage: python3 dashboard.py [port]
"""

import http.server
import json
import os
import re
import subprocess
import sys
import socketserver

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8567
DUMP_FILE = "/tmp/rp2350_full_dump.txt"

# ---------------------------------------------------------------------------
# Data collection helpers
# ---------------------------------------------------------------------------

def parse_dump_file(path):
    """Parse the structured dump file into a dict of register values."""
    regs = {}
    cpu_regs = {}
    sections = {}
    current_section = "unknown"

    if not os.path.exists(path):
        return regs, cpu_regs, sections, "(dump file not found)"

    raw = open(path).read()

    for line in raw.splitlines():
        if line.startswith("====="):
            current_section = line.strip("= \n")
            continue
        m = re.match(r"REG\s+(\S+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)", line)
        if m:
            name, addr, val = m.group(1), int(m.group(2), 16), int(m.group(3), 16)
            regs[name] = {"addr": addr, "val": val, "section": current_section}
            if current_section not in sections:
                sections[current_section] = []
            sections[current_section].append(name)
        m2 = re.match(r"CPUREG\s+(\S+)\s+.*:\s+(0x[0-9a-fA-F]+)", line)
        if m2:
            cpu_regs[m2.group(1)] = int(m2.group(2), 16)

    return regs, cpu_regs, sections, raw


def get_picotool_info():
    """Run picotool info -a and return the output."""
    pt = os.path.expanduser(
        "~/.platformio/packages/tool-picotool-rp2040-earlephilhower/picotool"
    )
    if not os.path.exists(pt):
        return "(picotool not found)"
    try:
        return subprocess.check_output([pt, "info", "-a"],
                                       stderr=subprocess.STDOUT,
                                       timeout=10).decode()
    except Exception as e:
        return f"(picotool error: {e})"


def get_flash_hex():
    """Read flash header hex dump if available."""
    path = "/tmp/flash_head.bin"
    if not os.path.exists(path):
        return "(flash dump not available)"
    try:
        return subprocess.check_output(["xxd", path],
                                       timeout=5).decode()
    except Exception:
        return "(xxd failed)"


# ---------------------------------------------------------------------------
# Bit-field decoders
# ---------------------------------------------------------------------------

def decode_bits(val, fields):
    """Decode a register value given a list of (name, hi_bit, lo_bit, descriptions_dict_or_None)."""
    result = []
    for name, hi, lo, desc_map in fields:
        mask = ((1 << (hi - lo + 1)) - 1) << lo
        field_val = (val & mask) >> lo
        desc = ""
        if desc_map and field_val in desc_map:
            desc = desc_map[field_val]
        elif desc_map:
            desc = f"(unknown: {field_val})"
        result.append({
            "name": name,
            "bits": f"[{hi}:{lo}]" if hi != lo else f"[{lo}]",
            "val": field_val,
            "hex": f"0x{field_val:x}",
            "desc": desc,
        })
    return result


def decode_pad(val):
    return decode_bits(val, [
        ("OD", 7, 7, {0: "Output enabled", 1: "Output disabled"}),
        ("IE", 6, 6, {0: "Input disabled", 1: "Input enabled"}),
        ("DRIVE", 5, 4, {0: "2mA", 1: "4mA", 2: "8mA", 3: "12mA"}),
        ("PUE", 3, 3, {0: "No pull-up", 1: "Pull-up enabled"}),
        ("PDE", 2, 2, {0: "No pull-down", 1: "Pull-down enabled"}),
        ("SCHMITT", 1, 1, {0: "Disabled", 1: "Enabled"}),
        ("SLEWFAST", 0, 0, {0: "Slow slew", 1: "Fast slew"}),
    ])


def decode_gpio_status(val):
    return decode_bits(val, [
        ("IRQTOPROC", 26, 26, {0: "No IRQ", 1: "IRQ to processor"}),
        ("INFROMPAD", 17, 17, {0: "LOW", 1: "HIGH"}),
        ("OETOPAD", 13, 13, {0: "Output disabled", 1: "Output enabled"}),
        ("OUTTOPAD", 9, 9, {0: "Drive LOW", 1: "Drive HIGH"}),
    ])


def decode_gpio_ctrl(val):
    funcsel = val & 0x1f
    funcs = {0: "QSPI function", 5: "SIO", 0x1f: "NULL (disconnected)"}
    return decode_bits(val, [
        ("IRQOVER", 29, 28, {0: "Normal", 1: "Invert", 2: "Force LOW", 3: "Force HIGH"}),
        ("INOVER", 17, 16, {0: "Normal", 1: "Invert", 2: "Force LOW", 3: "Force HIGH"}),
        ("OEOVER", 15, 14, {0: "Normal", 1: "Invert", 2: "Force LOW", 3: "Force HIGH"}),
        ("OUTOVER", 13, 12, {0: "Normal", 1: "Invert", 2: "Force LOW", 3: "Force HIGH"}),
        ("FUNCSEL", 4, 0, funcs),
    ])


def decode_xosc_status(val):
    return decode_bits(val, [
        ("STABLE", 31, 31, {0: "Not stable", 1: "Stable & running"}),
        ("BADWRITE", 24, 24, {0: "OK", 1: "Bad write detected"}),
        ("ENABLED", 12, 12, {0: "Disabled", 1: "Enabled"}),
        ("FREQ_RANGE", 1, 0, {0: "1-15 MHz", 1: "10-30 MHz", 2: "25-60 MHz", 3: "40-100 MHz"}),
    ])


def decode_pll_cs(val):
    return decode_bits(val, [
        ("LOCK", 31, 31, {0: "Not locked", 1: "PLL locked"}),
        ("BYPASS", 8, 8, {0: "Normal", 1: "Bypass"}),
        ("REFDIV", 5, 0, None),
    ])


def decode_pll_prim(val):
    return decode_bits(val, [
        ("POSTDIV1", 18, 16, None),
        ("POSTDIV2", 14, 12, None),
    ])


def decode_watchdog_ctrl(val):
    return decode_bits(val, [
        ("TRIGGER", 31, 31, {0: "No", 1: "Triggered"}),
        ("ENABLE", 30, 30, {0: "Disabled", 1: "Enabled"}),
        ("PAUSE_DBG1", 26, 26, {0: "Run", 1: "Pause on debug1"}),
        ("PAUSE_DBG0", 25, 25, {0: "Run", 1: "Pause on debug0"}),
        ("PAUSE_JTAG", 24, 24, {0: "Run", 1: "Pause on JTAG"}),
        ("TIME", 23, 0, None),
    ])


# ---------------------------------------------------------------------------
# HTML generation
# ---------------------------------------------------------------------------

NAME_MAP = {
    "PADS_SCLK": "PADS_QSPI_SCLK", "PADS_SD0": "PADS_QSPI_SD0",
    "PADS_SD1": "PADS_QSPI_SD1", "PADS_SD2": "PADS_QSPI_SD2",
    "PADS_SD3": "PADS_QSPI_SD3", "PADS_SS": "PADS_QSPI_SS",
    "PADS_VOLTAGE_SELECT": "PADS_QSPI_VOLTAGE_SELECT",
    "WATCHDOG_CTRL": "WDG_CTRL", "WATCHDOG_REASON": "WDG_REASON",
    "WATCHDOG_LOAD": "WDG_LOAD",
    "SCRATCH0": "WDG_SCRATCH0", "SCRATCH1": "WDG_SCRATCH1",
    "SCRATCH2": "WDG_SCRATCH2", "SCRATCH3": "WDG_SCRATCH3",
    "SCRATCH4": "WDG_SCRATCH4", "SCRATCH5": "WDG_SCRATCH5",
    "SCRATCH6": "WDG_SCRATCH6", "SCRATCH7": "WDG_SCRATCH7",
}

def _reg_val(regs, name):
    if name in regs:
        return regs[name]["val"]
    mapped = NAME_MAP.get(name)
    if mapped and mapped in regs:
        return regs[mapped]["val"]
    return None


def generate_html(regs, cpu_regs, sections, raw_dump, picotool_info, flash_hex):
    """Generate the full HTML dashboard."""

    def rv(name):
        v = _reg_val(regs, name)
        if v is not None:
            return v
        # Also try with common prefix variations
        for prefix in ["PADS_QSPI_", "WDG_", "PSM_", "SIO_"]:
            short = name.replace("PADS_", "").replace("WATCHDOG_", "").replace("SCRATCH", "")
            alt = prefix + name.split("_", 1)[-1] if "_" in name else name
            v2 = _reg_val(regs, alt)
            if v2 is not None:
                return v2
        return 0

    def rhex(name):
        v = rv(name)
        return f"0x{v:08x}" if v else "N/A"

    def bit_table(decoded, reg_name="", reg_val_num=0):
        """Render decoded bit fields as an HTML table."""
        rows = ""
        for f in decoded:
            val_class = ""
            if f["name"] in ("INFROMPAD", "STABLE", "LOCK", "IE", "PUE"):
                val_class = ' class="val-good"' if f["val"] else ' class="val-bad"'
            elif f["name"] == "PDE" and f["val"]:
                val_class = ' class="val-warn"'
            elif f["name"] == "ENABLE" and f["val"]:
                val_class = ' class="val-bad"'
            rows += f"""<tr>
                <td class="mono">{f['bits']}</td>
                <td><strong>{f['name']}</strong></td>
                <td class="mono"{val_class}>{f['val']}</td>
                <td>{f['desc']}</td>
            </tr>"""
        return f"""<div class="reg-decode">
            <div class="reg-header">{reg_name} = <span class="mono">0x{reg_val_num:08x}</span>
                <span class="bitbar">{reg_val_num:032b}</span>
            </div>
            <table class="fields"><tr><th>Bits</th><th>Field</th><th>Value</th><th>Description</th></tr>
            {rows}</table></div>"""

    # Compute derived values
    xosc_ok = bool(rv("XOSC_STATUS") & (1 << 31))
    pll_sys_lock = bool(rv("PLL_SYS_CS") & (1 << 31))
    pll_usb_lock = bool(rv("PLL_USB_CS") & (1 << 31))
    ss_infrompad = bool(rv("QSPI_SS_STATUS") & (1 << 17))
    pads_ss = rv("PADS_SS")
    ss_pue = bool(pads_ss & (1 << 3))
    wdt_enabled = bool(rv("WATCHDOG_CTRL") & (1 << 30))

    # PLL frequency calculation
    pll_sys_fbdiv = rv("PLL_SYS_FBDIV_INT") & 0xfff
    pll_sys_refdiv = rv("PLL_SYS_CS") & 0x3f
    pll_sys_pd1 = (rv("PLL_SYS_PRIM") >> 16) & 0x7
    pll_sys_pd2 = (rv("PLL_SYS_PRIM") >> 12) & 0x7
    if pll_sys_refdiv and pll_sys_pd1 and pll_sys_pd2:
        pll_sys_freq = (12.0 / pll_sys_refdiv) * pll_sys_fbdiv / (pll_sys_pd1 * pll_sys_pd2)
    else:
        pll_sys_freq = 0

    pll_usb_fbdiv = rv("PLL_USB_FBDIV_INT") & 0xfff
    pll_usb_refdiv = rv("PLL_USB_CS") & 0x3f
    pll_usb_pd1 = (rv("PLL_USB_PRIM") >> 16) & 0x7
    pll_usb_pd2 = (rv("PLL_USB_PRIM") >> 12) & 0x7
    if pll_usb_refdiv and pll_usb_pd1 and pll_usb_pd2:
        pll_usb_freq = (12.0 / pll_usb_refdiv) * pll_usb_fbdiv / (pll_usb_pd1 * pll_usb_pd2)
    else:
        pll_usb_freq = 0

    # CPU regs
    pc = cpu_regs.get("pc", 0)

    # Status indicators
    status_items = [
        ("QSPI_SS Pin", "LOW (STUCK)" if not ss_infrompad else "HIGH (OK)",
         "critical" if not ss_infrompad else "ok"),
        ("Pull-up", "Enabled" if ss_pue else "Disabled",
         "ok" if ss_pue else "warn"),
        ("Crystal (XOSC)", "Stable" if xosc_ok else "NOT Stable",
         "ok" if xosc_ok else "critical"),
        ("PLL SYS", f"Locked ({pll_sys_freq:.0f} MHz)" if pll_sys_lock else "Unlocked",
         "ok" if pll_sys_lock else "warn"),
        ("PLL USB", f"Locked ({pll_usb_freq:.0f} MHz)" if pll_usb_lock else "Unlocked",
         "ok" if pll_usb_lock else "warn"),
        ("Watchdog", "Enabled" if wdt_enabled else "Disabled",
         "warn" if wdt_enabled else "ok"),
        ("Boot Mode", "BOOTSEL", "critical"),
        ("Core 0 PC", f"0x{pc:08x} (bootrom)" if pc < 0x00020000 else f"0x{pc:08x}", "info"),
    ]

    status_html = ""
    for label, value, severity in status_items:
        status_html += f'<div class="status-card {severity}"><div class="status-label">{label}</div><div class="status-value">{value}</div></div>'

    # QSPI section
    qspi_pins = ["SCLK", "SS", "SD0", "SD1", "SD2", "SD3"]
    qspi_html = ""
    for pin in qspi_pins:
        st_name = f"QSPI_{pin}_STATUS"
        ct_name = f"QSPI_{pin}_CTRL"
        pad_name = f"PADS_{pin}"
        st_val = rv(st_name)
        ct_val = rv(ct_name)
        pad_val = rv(pad_name)
        infrom = (st_val >> 17) & 1
        oetopad = (st_val >> 13) & 1
        outtopad = (st_val >> 9) & 1
        funcsel = ct_val & 0x1f
        ie = (pad_val >> 6) & 1
        pue = (pad_val >> 3) & 1
        pde = (pad_val >> 2) & 1

        pin_class = "pin-high" if infrom else "pin-low"
        if pin == "SS" and not infrom:
            pin_class = "pin-stuck"

        qspi_html += f"""<div class="pin-card {pin_class}">
            <div class="pin-name">QSPI_{pin}</div>
            <div class="pin-grid">
                <span class="pin-field">Input: <strong>{'HIGH' if infrom else 'LOW'}</strong></span>
                <span class="pin-field">OE: <strong>{'ON' if oetopad else 'OFF'}</strong></span>
                <span class="pin-field">Out: <strong>{'HIGH' if outtopad else 'LOW'}</strong></span>
                <span class="pin-field">Func: <strong>{'NULL' if funcsel == 0x1f else ('QSPI' if funcsel == 0 else f'F{funcsel}')}</strong></span>
                <span class="pin-field">IE: <strong>{ie}</strong></span>
                <span class="pin-field">PU: <strong>{pue}</strong> PD: <strong>{pde}</strong></span>
            </div>
            <div class="pin-raw">STATUS={rhex(st_name)} CTRL={rhex(ct_name)} PAD={rhex(pad_name)}</div>
        </div>"""

    # Clock tree
    clk_ref_src = rv("CLK_REF_CTRL") & 0x3
    clk_sys_src = rv("CLK_SYS_CTRL") & 0x1
    clk_sys_auxsrc = (rv("CLK_SYS_CTRL") >> 5) & 0x7
    ref_sources = {0: "ROSC", 1: "AUX", 2: "XOSC"}
    sys_aux_sources = {0: "PLL_SYS", 1: "PLL_USB", 2: "ROSC", 3: "XOSC"}

    # Decode register groups for detailed view
    detailed_sections = []

    # QSPI_SS detailed
    if "QSPI_SS_STATUS" in regs:
        detailed_sections.append(("QSPI_SS (BOOTSEL Pin)", [
            bit_table(decode_gpio_status(rv("QSPI_SS_STATUS")), "QSPI_SS_STATUS", rv("QSPI_SS_STATUS")),
            bit_table(decode_gpio_ctrl(rv("QSPI_SS_CTRL")), "QSPI_SS_CTRL", rv("QSPI_SS_CTRL")),
            bit_table(decode_pad(rv("PADS_SS")), "PADS_QSPI_SS", rv("PADS_SS")),
        ]))

    # XOSC
    if "XOSC_STATUS" in regs:
        detailed_sections.append(("Crystal Oscillator (XOSC)", [
            bit_table(decode_xosc_status(rv("XOSC_STATUS")), "XOSC_STATUS", rv("XOSC_STATUS")),
            f'<div class="reg-decode"><div class="reg-header">XOSC_CTRL = <span class="mono">{rhex("XOSC_CTRL")}</span></div>'
            f'<p>ENABLE=0xfab (enabled), FREQ_RANGE configured for 12 MHz crystal</p></div>',
            f'<div class="reg-decode"><div class="reg-header">XOSC_STARTUP = <span class="mono">{rhex("XOSC_STARTUP")}</span></div>'
            f'<p>Startup delay: {rv("XOSC_STARTUP") & 0x3fff} &times; 256 XOSC cycles</p></div>',
        ]))

    # PLLs
    if "PLL_SYS_CS" in regs:
        detailed_sections.append(("PLL System", [
            bit_table(decode_pll_cs(rv("PLL_SYS_CS")), "PLL_SYS_CS", rv("PLL_SYS_CS")),
            bit_table(decode_pll_prim(rv("PLL_SYS_PRIM")), "PLL_SYS_PRIM", rv("PLL_SYS_PRIM")),
            f'<div class="reg-decode"><div class="reg-header">PLL_SYS_FBDIV_INT = <span class="mono">{rhex("PLL_SYS_FBDIV_INT")}</span></div>'
            f'<p>Feedback divisor: {pll_sys_fbdiv}</p>'
            f'<p>Output: (12 MHz / {pll_sys_refdiv}) &times; {pll_sys_fbdiv} / ({pll_sys_pd1} &times; {pll_sys_pd2}) = <strong>{pll_sys_freq:.1f} MHz</strong></p></div>',
        ]))

    if "PLL_USB_CS" in regs:
        detailed_sections.append(("PLL USB", [
            bit_table(decode_pll_cs(rv("PLL_USB_CS")), "PLL_USB_CS", rv("PLL_USB_CS")),
            bit_table(decode_pll_prim(rv("PLL_USB_PRIM")), "PLL_USB_PRIM", rv("PLL_USB_PRIM")),
            f'<div class="reg-decode"><div class="reg-header">PLL_USB_FBDIV_INT = <span class="mono">{rhex("PLL_USB_FBDIV_INT")}</span></div>'
            f'<p>Feedback divisor: {pll_usb_fbdiv}</p>'
            f'<p>Output: (12 MHz / {pll_usb_refdiv}) &times; {pll_usb_fbdiv} / ({pll_usb_pd1} &times; {pll_usb_pd2}) = <strong>{pll_usb_freq:.1f} MHz</strong></p></div>',
        ]))

    # Watchdog
    if "WATCHDOG_CTRL" in regs:
        wdt_reason = rv("WATCHDOG_REASON")
        scratches = "<table class='fields'><tr><th>Register</th><th>Value</th></tr>"
        for i in range(8):
            sn = f"SCRATCH{i}"
            scratches += f"<tr><td>{sn}</td><td class='mono'>{rhex(sn)}</td></tr>"
        scratches += "</table>"
        detailed_sections.append(("Watchdog", [
            bit_table(decode_watchdog_ctrl(rv("WATCHDOG_CTRL")), "WATCHDOG_CTRL", rv("WATCHDOG_CTRL")),
            f'<div class="reg-decode"><div class="reg-header">WATCHDOG_REASON = <span class="mono">0x{wdt_reason:08x}</span></div>'
            f'<p>FORCE: {(wdt_reason>>1)&1}, TIMER: {wdt_reason&1}</p></div>',
            f'<div class="reg-decode"><div class="reg-header">Scratch Registers</div>{scratches}</div>',
        ]))

    # SIO
    if "SIO_CPUID" in regs:
        detailed_sections.append(("SIO (Single-cycle IO)", [
            f'<div class="reg-decode"><div class="reg-header">CPUID = <span class="mono">{rhex("SIO_CPUID")}</span></div>'
            f'<p>Currently executing on Core {rv("SIO_CPUID")}</p></div>',
            f'<div class="reg-decode"><div class="reg-header">GPIO_IN = <span class="mono">{rhex("SIO_GPIO_IN")}</span></div>'
            f'<p class="bitbar">{rv("SIO_GPIO_IN"):032b}</p></div>',
            f'<div class="reg-decode"><div class="reg-header">GPIO_OUT = <span class="mono">{rhex("SIO_GPIO_OUT")}</span></div>'
            f'<p class="bitbar">{rv("SIO_GPIO_OUT"):032b}</p></div>',
            f'<div class="reg-decode"><div class="reg-header">GPIO_OE = <span class="mono">{rhex("SIO_GPIO_OE")}</span></div>'
            f'<p class="bitbar">{rv("SIO_GPIO_OE"):032b}</p></div>',
        ]))

    # System info
    if "SYSINFO_CHIP_ID" in regs:
        chip_id = rv("SYSINFO_CHIP_ID")
        mfr = chip_id & 0xfff
        part = (chip_id >> 12) & 0xffff
        rev = (chip_id >> 28) & 0xf
        detailed_sections.append(("System Identification", [
            f'<div class="reg-decode"><div class="reg-header">CHIP_ID = <span class="mono">0x{chip_id:08x}</span></div>'
            f'<p>Manufacturer: 0x{mfr:03x}, Part: 0x{part:04x}, Revision: {rev}</p></div>',
            f'<div class="reg-decode"><div class="reg-header">PLATFORM = <span class="mono">{rhex("SYSINFO_PLATFORM")}</span></div>'
            f'<p>Bit 0 (FPGA): {rv("SYSINFO_PLATFORM")&1}, Bit 1 (ASIC): {(rv("SYSINFO_PLATFORM")>>1)&1}</p></div>',
            f'<div class="reg-decode"><div class="reg-header">ROM GITREF = <span class="mono">{rhex("SYSINFO_GITREF_RP2350")}</span></div></div>',
        ]))

    # Resets
    if "RESET" in regs:
        reset_val = rv("RESET")
        reset_done = rv("RESET_DONE")
        held = reset_val & ~reset_done
        periph_names = [
            "ADC","BUSCTRL","DMA","HSTX","I2C0","I2C1","IO_BANK0","IO_QSPI",
            "JTAG","PADS_BANK0","PADS_QSPI","PIO0","PIO1","PIO2","PLL_SYS","PLL_USB",
            "PWM","SHA256","SPI0","SPI1","SYSCFG","SYSINFO","TBMAN","TIMER0",
            "TIMER1","TRNG","UART0","UART1","USBCTRL"
        ]
        reset_rows = ""
        for i, pname in enumerate(periph_names):
            if i >= 32:
                break
            in_reset = (reset_val >> i) & 1
            done = (reset_done >> i) & 1
            st = "In reset" if in_reset else ("Running" if done else "Unknown")
            cls = "val-bad" if in_reset else "val-good"
            reset_rows += f'<tr><td>{i}</td><td>{pname}</td><td class="{cls}">{st}</td></tr>'
        detailed_sections.append(("Peripheral Resets", [
            f'<div class="reg-decode"><div class="reg-header">RESET = <span class="mono">0x{reset_val:08x}</span> &nbsp; RESET_DONE = <span class="mono">0x{reset_done:08x}</span></div>'
            f'<table class="fields"><tr><th>Bit</th><th>Peripheral</th><th>State</th></tr>{reset_rows}</table></div>',
        ]))

    # CPU Registers
    if cpu_regs:
        cpu_rows = ""
        for rn, rv_val in cpu_regs.items():
            extra = ""
            if rn == "pc" and rv_val < 0x20000:
                extra = " (bootrom)"
            elif rn == "xpsr":
                mode = rv_val & 0x1ff
                if mode == 0:
                    extra = " Thread mode"
                else:
                    extra = f" Exception #{mode}"
            cpu_rows += f'<tr><td><strong>{rn}</strong></td><td class="mono">0x{rv_val:08x}</td><td>{extra}</td></tr>'
        detailed_sections.append(("ARM Cortex-M33 Core 0 Registers", [
            f'<table class="fields"><tr><th>Register</th><th>Value</th><th>Notes</th></tr>{cpu_rows}</table>',
        ]))

    # All registers raw table
    all_reg_rows = ""
    for name, info in sorted(regs.items(), key=lambda x: x[1]["addr"]):
        all_reg_rows += f'<tr><td class="mono">0x{info["addr"]:08x}</td><td>{name}</td><td class="mono">0x{info["val"]:08x}</td><td class="bitbar-small">{info["val"]:032b}</td></tr>'

    # Build detailed sections HTML
    detailed_html = ""
    for title, parts in detailed_sections:
        inner = "".join(parts)
        detailed_html += f'<details class="section-detail"><summary><h3>{title}</h3></summary>{inner}</details>'

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>RP2350 Debug Dashboard</title>
<style>
:root {{
    --bg: #0d1117; --surface: #161b22; --surface2: #21262d;
    --border: #30363d; --text: #e6edf3; --text2: #8b949e;
    --accent: #58a6ff; --green: #3fb950; --red: #f85149;
    --yellow: #d29922; --purple: #bc8cff; --mono: 'JetBrains Mono', 'Fira Code', 'Cascadia Code', monospace;
}}
* {{ margin: 0; padding: 0; box-sizing: border-box; }}
body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
    background: var(--bg); color: var(--text); line-height: 1.5; }}
.container {{ max-width: 1400px; margin: 0 auto; padding: 20px; }}
h1 {{ font-size: 1.8em; margin-bottom: 4px; }}
h1 span {{ color: var(--accent); }}
.subtitle {{ color: var(--text2); margin-bottom: 24px; font-size: 0.95em; }}
h2 {{ color: var(--accent); margin: 32px 0 16px; font-size: 1.3em;
    border-bottom: 1px solid var(--border); padding-bottom: 8px; }}
h3 {{ color: var(--purple); margin: 0; font-size: 1.1em; display: inline; }}

/* Status cards */
.status-grid {{ display: grid; grid-template-columns: repeat(auto-fill, minmax(200px, 1fr));
    gap: 12px; margin-bottom: 24px; }}
.status-card {{ background: var(--surface); border: 1px solid var(--border);
    border-radius: 8px; padding: 14px; border-left: 4px solid var(--border); }}
.status-card.ok {{ border-left-color: var(--green); }}
.status-card.critical {{ border-left-color: var(--red); background: #1a0a0a; }}
.status-card.warn {{ border-left-color: var(--yellow); }}
.status-card.info {{ border-left-color: var(--accent); }}
.status-label {{ font-size: 0.8em; color: var(--text2); text-transform: uppercase;
    letter-spacing: 0.05em; }}
.status-value {{ font-size: 1.1em; font-weight: 600; margin-top: 4px; }}
.status-card.critical .status-value {{ color: var(--red); }}
.status-card.ok .status-value {{ color: var(--green); }}
.status-card.warn .status-value {{ color: var(--yellow); }}

/* Pin cards */
.pin-grid-container {{ display: grid; grid-template-columns: repeat(auto-fill, minmax(280px, 1fr));
    gap: 12px; margin-bottom: 24px; }}
.pin-card {{ background: var(--surface); border: 1px solid var(--border);
    border-radius: 8px; padding: 14px; }}
.pin-card.pin-high {{ border-left: 4px solid var(--green); }}
.pin-card.pin-low {{ border-left: 4px solid var(--text2); }}
.pin-card.pin-stuck {{ border-left: 4px solid var(--red); background: #1a0a0a; }}
.pin-name {{ font-weight: 700; font-size: 1.05em; margin-bottom: 8px; font-family: var(--mono); }}
.pin-grid {{ display: grid; grid-template-columns: 1fr 1fr; gap: 4px 12px; font-size: 0.9em; }}
.pin-field {{ color: var(--text2); }}
.pin-field strong {{ color: var(--text); }}
.pin-raw {{ margin-top: 8px; font-family: var(--mono); font-size: 0.7em; color: var(--text2);
    word-break: break-all; }}

/* Clock tree */
.clock-tree {{ background: var(--surface); border: 1px solid var(--border);
    border-radius: 8px; padding: 20px; margin-bottom: 24px; font-family: var(--mono);
    font-size: 0.85em; overflow-x: auto; }}
.clock-tree pre {{ color: var(--text); white-space: pre; }}
.clk-active {{ color: var(--green); font-weight: bold; }}
.clk-inactive {{ color: var(--text2); }}

/* Register decode */
.reg-decode {{ background: var(--surface); border: 1px solid var(--border);
    border-radius: 6px; padding: 12px; margin: 10px 0; }}
.reg-header {{ font-family: var(--mono); font-size: 0.95em; margin-bottom: 8px;
    color: var(--accent); font-weight: 600; }}
.fields {{ width: 100%; border-collapse: collapse; font-size: 0.85em; }}
.fields th {{ text-align: left; color: var(--text2); padding: 4px 12px 4px 0;
    border-bottom: 1px solid var(--border); font-weight: 500; }}
.fields td {{ padding: 3px 12px 3px 0; border-bottom: 1px solid var(--surface2); }}
.val-good {{ color: var(--green); font-weight: 600; }}
.val-bad {{ color: var(--red); font-weight: 600; }}
.val-warn {{ color: var(--yellow); }}

.mono {{ font-family: var(--mono); font-size: 0.92em; }}
.bitbar {{ font-family: var(--mono); font-size: 0.65em; color: var(--text2);
    letter-spacing: 0.5px; margin-left: 8px; word-break: break-all; }}
.bitbar-small {{ font-family: var(--mono); font-size: 0.6em; color: var(--text2);
    letter-spacing: 0.3px; }}

/* Details / collapsible */
details.section-detail {{ background: var(--surface); border: 1px solid var(--border);
    border-radius: 8px; margin: 12px 0; }}
details.section-detail > summary {{ padding: 14px; cursor: pointer; list-style: none; }}
details.section-detail > summary::-webkit-details-marker {{ display: none; }}
details.section-detail > summary::before {{ content: "\\25B6\\FE0E "; color: var(--accent); }}
details.section-detail[open] > summary::before {{ content: "\\25BC\\FE0E "; }}
details.section-detail > summary:hover {{ background: var(--surface2); }}
details.section-detail > :not(summary) {{ padding: 0 14px 14px; }}

/* Flash hex */
.hex-dump {{ background: var(--surface); border: 1px solid var(--border);
    border-radius: 6px; padding: 14px; font-family: var(--mono); font-size: 0.78em;
    overflow-x: auto; white-space: pre; color: var(--text2); max-height: 400px; overflow-y: auto; }}

/* Picotool output */
.pre-block {{ background: var(--surface); border: 1px solid var(--border);
    border-radius: 6px; padding: 14px; font-family: var(--mono); font-size: 0.82em;
    overflow-x: auto; white-space: pre-wrap; color: var(--text); max-height: 500px;
    overflow-y: auto; }}

/* All registers table */
.all-regs {{ max-height: 600px; overflow-y: auto; }}
.all-regs table {{ width: 100%; }}

/* Diagnosis box */
.diagnosis {{ background: #1a0a0a; border: 2px solid var(--red); border-radius: 8px;
    padding: 20px; margin: 24px 0; }}
.diagnosis h3 {{ color: var(--red); margin-bottom: 12px; display: block; }}
.diagnosis ul {{ margin-left: 20px; margin-top: 8px; }}
.diagnosis li {{ margin: 4px 0; }}

/* Tabs */
.tab-bar {{ display: flex; gap: 4px; border-bottom: 1px solid var(--border); margin-bottom: 16px; }}
.tab-btn {{ background: none; border: none; color: var(--text2); padding: 10px 18px;
    cursor: pointer; font-size: 0.95em; border-bottom: 2px solid transparent;
    font-family: inherit; }}
.tab-btn:hover {{ color: var(--text); background: var(--surface); }}
.tab-btn.active {{ color: var(--accent); border-bottom-color: var(--accent); }}
.tab-content {{ display: none; }}
.tab-content.active {{ display: block; }}
</style>
</head>
<body>
<div class="container">

<h1><span>RP2350</span> Debug Dashboard</h1>
<p class="subtitle">Live hardware state captured via SWD debug probe and picotool &mdash; Sequoia Motor Controller Board (QFN-80)</p>

<div class="diagnosis">
<h3>Diagnosis: QSPI_SS (BOOTSEL) Pin Stuck LOW</h3>
<p>The SWD debug probe confirms that <strong>QSPI_SS reads LOW</strong> despite the internal pull-up being enabled.
Something external is pulling the pin to ground, causing the bootrom to enter BOOTSEL mode on every reset.
The crystal, PLLs, flash, and firmware are all fine.</p>
<ul>
<li>Measure QSPI_SS to GND with a multimeter &mdash; should be open when BOOTSEL button is not pressed</li>
<li>Inspect for solder bridges near the QFN-80 QSPI_SS pad and flash chip CS pin</li>
<li>If no BOOTSEL button exists on this board, add a 10k&Omega; pull-up from QSPI_SS to IOVDD</li>
</ul>
</div>

<h2>System Status</h2>
<div class="status-grid">{status_html}</div>

<div class="tab-bar">
    <button class="tab-btn active" onclick="showTab('qspi')">QSPI Pins</button>
    <button class="tab-btn" onclick="showTab('clocks')">Clock Tree</button>
    <button class="tab-btn" onclick="showTab('details')">Register Details</button>
    <button class="tab-btn" onclick="showTab('flash')">Flash</button>
    <button class="tab-btn" onclick="showTab('picotool')">Picotool Info</button>
    <button class="tab-btn" onclick="showTab('allregs')">All Registers</button>
    <button class="tab-btn" onclick="showTab('raw')">Raw Dump</button>
</div>

<div id="tab-qspi" class="tab-content active">
<h2>QSPI Pin States</h2>
<div class="pin-grid-container">{qspi_html}</div>
</div>

<div id="tab-clocks" class="tab-content">
<h2>Clock Tree</h2>
<div class="clock-tree"><pre>
  ┌─────────────┐
  │  12 MHz XTAL │──┐  XOSC_STATUS: <span class="{'clk-active' if xosc_ok else 'clk-inactive'}">{'STABLE' if xosc_ok else 'NOT STABLE'}</span>
  └─────────────┘  │
                    ▼
              ┌──────────┐      ┌───────────────┐
              │   XOSC   │─────▶│   PLL_SYS     │  {'<span class="clk-active">LOCKED</span>' if pll_sys_lock else '<span class="clk-inactive">UNLOCKED</span>'}
              │          │      │ FBDIV={pll_sys_fbdiv:3d}     │  REFDIV={pll_sys_refdiv} POSTDIV={pll_sys_pd1}x{pll_sys_pd2}
              │          │      │ <span class="clk-active">{pll_sys_freq:6.1f} MHz</span>   │
              └──────────┘      └───────┬───────┘
                    │                   │
                    │                   ▼
  ┌─────────┐      │           ┌───────────────┐
  │  ROSC   │      │           │   CLK_SYS     │  SRC={'AUX' if clk_sys_src else 'REF'}  AUXSRC={sys_aux_sources.get(clk_sys_auxsrc, '?')}
  │ (~6 MHz)│      │           └───────┬───────┘
  └────┬────┘      │                   │
       │           │                   ▼
       │           │            Cortex-M33 cores, bus fabric, memory
       │           │
       │           │
       │      ┌────┴──────┐    ┌───────────────┐
       │      │   XOSC    │───▶│   PLL_USB     │  {'<span class="clk-active">LOCKED</span>' if pll_usb_lock else '<span class="clk-inactive">UNLOCKED</span>'}
       │      └───────────┘    │ FBDIV={pll_usb_fbdiv:3d}     │  REFDIV={pll_usb_refdiv} POSTDIV={pll_usb_pd1}x{pll_usb_pd2}
       │                       │ <span class="clk-active">{pll_usb_freq:6.1f} MHz</span>   │
       │                       └───────┬───────┘
       │                               │
       ▼                               ▼
  ┌──────────┐                 ┌───────────────┐
  │ CLK_REF  │                 │   CLK_USB     │  USB controller (48 MHz for BOOTSEL)
  │ SRC={ref_sources.get(clk_ref_src, '?'):4s}  │                 └───────────────┘
  └──────────┘
</pre></div>
</div>

<div id="tab-details" class="tab-content">
<h2>Detailed Register Decode</h2>
{detailed_html}
</div>

<div id="tab-flash" class="tab-content">
<h2>Flash Header (first 256 bytes at 0x10000000)</h2>
<div class="hex-dump">{flash_hex}</div>
</div>

<div id="tab-picotool" class="tab-content">
<h2>Picotool Device Info</h2>
<div class="pre-block">{picotool_info}</div>
</div>

<div id="tab-allregs" class="tab-content">
<h2>All Captured Registers</h2>
<div class="all-regs">
<table class="fields">
<tr><th>Address</th><th>Name</th><th>Value</th><th>Binary</th></tr>
{all_reg_rows}
</table>
</div>
</div>

<div id="tab-raw" class="tab-content">
<h2>Raw Dump Output</h2>
<div class="pre-block">{raw_dump[:30000] if raw_dump else '(no raw dump available)'}</div>
</div>

</div>
<script>
function showTab(id) {{
    document.querySelectorAll('.tab-content').forEach(t => t.classList.remove('active'));
    document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
    document.getElementById('tab-' + id).classList.add('active');
    event.target.classList.add('active');
}}
</script>
</body>
</html>"""
    return html


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print(f"Loading data from {DUMP_FILE}...")
    regs, cpu_regs, sections, raw_dump = parse_dump_file(DUMP_FILE)
    print(f"  Loaded {len(regs)} registers, {len(cpu_regs)} CPU regs")

    print("Getting picotool info...")
    picotool_info = get_picotool_info()

    print("Getting flash hex dump...")
    flash_hex = get_flash_hex()

    print("Generating HTML...")
    html = generate_html(regs, cpu_regs, sections, raw_dump, picotool_info, flash_hex)

    # Write HTML to file for reference
    out_path = "/home/sequoia/Software/motor-firmware/dashboard.html"
    with open(out_path, "w") as f:
        f.write(html)
    print(f"  Saved to {out_path}")

    # Serve
    class Handler(http.server.SimpleHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(html.encode())

        def log_message(self, format, *args):
            pass  # quiet

    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        print(f"\n  Serving on http://localhost:{PORT}")
        print(f"  Press Ctrl+C to stop\n")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
