"""
Pre-build script: patches libraries for RP2350B motor controller.

SimpleFOC patches:
  - ADC pin offset: RP2350B uses GPIO 40-47 (not 26-29)
  - 12-bit ADC resolution (upstream uses 8-bit)
  - Max ADC sample rate (upstream uses 20kHz)
  - LowPassFilter y_prev made public (allows PID/LPF reset after NaN)

FUSB302 patches:
  - Replace Wire (I2C0) with Wire1 (I2C1) — GPIO22/23 are I2C1 only on RP2350
  - Define SERIAL_TX_BUFFER_SIZE fallback (not provided by arduino-pico)
"""
import shutil
from pathlib import Path

Import("env")

patches = Path(env["PROJECT_DIR"]) / "patches"
libdeps = Path(env["PROJECT_LIBDEPS_DIR"]) / env["PIOENV"]
lib_src = libdeps / "Simple FOC" / "src"

# Current sense ADC patches (pin offset, 12-bit, max sample rate)
cs_dir = lib_src / "current_sense" / "hardware_specific" / "rp2040"
for name in ["simplefoc_rp2040_current_sense.cpp", "simplefoc_rp2040_current_sense.h"]:
    src = patches / name
    dst = cs_dir / name.replace("simplefoc_rp2040_current_sense", "rp2040_mcu")
    if src.exists() and dst.exists():
        shutil.copy2(src, dst)
        print(f"  Patched SimpleFOC current sense: {dst.name}")

# LowPassFilter patch (public y_prev for reset)
src = patches / "simplefoc_lowpass_filter.h"
dst = lib_src / "common" / "lowpass_filter.h"
if src.exists() and dst.exists():
    shutil.copy2(src, dst)
    print(f"  Patched SimpleFOC: {dst.name}")

# --- FUSB302 PD UFP sink: replace Wire (I2C0) with Wire1 (I2C1) ---
fusb_src = libdeps / "FUSB302 PD UFP sink" / "src" / "PD_UFP.cpp"
if fusb_src.exists():
    text = fusb_src.read_text()
    original = text
    text = text.replace("Wire.", "Wire1.")
    text = text.replace("Wire1.h", "Wire.h")  # undo header name mangling
    if text != original:
        fusb_src.write_text(text)
        print(f"  Patched FUSB302: Wire -> Wire1 in {fusb_src.name}")

# FUSB302 PD_UFP_Log.cpp: SERIAL_TX_BUFFER_SIZE not defined on arduino-pico
fusb_log = libdeps / "FUSB302 PD UFP sink" / "src" / "PD_UFP_Log.cpp"
if fusb_log.exists():
    text = fusb_log.read_text()
    if "SERIAL_TX_BUFFER_SIZE" in text and "#ifndef SERIAL_TX_BUFFER_SIZE" not in text:
        # Add fallback define after the includes
        text = text.replace(
            '#include "PD_UFP.h"',
            '#include "PD_UFP.h"\n\n'
            '#ifndef SERIAL_TX_BUFFER_SIZE\n'
            '#define SERIAL_TX_BUFFER_SIZE 256\n'
            '#endif',
        )
        fusb_log.write_text(text)
        print(f"  Patched FUSB302: SERIAL_TX_BUFFER_SIZE fallback in {fusb_log.name}")
