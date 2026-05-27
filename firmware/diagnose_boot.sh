#!/usr/bin/env bash
#
# RP2350 Boot Diagnostic Script
#
# Runs a sequence of checks against an attached RP2350 board that is stuck
# in BOOTSEL mode. Covers USB enumeration, picotool device info, OTP
# configuration, flash contents, firmware verification, and a reboot test.
#
# Usage: ./diagnose_boot.sh [path-to-firmware.uf2]

set -uo pipefail

RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[1;33m'
CYN='\033[0;36m'
RST='\033[0m'

PASS=0
FAIL=0
WARN=0

pass()  { PASS=$((PASS+1)); printf "${GRN}[PASS]${RST} %s\n" "$1"; }
fail()  { FAIL=$((FAIL+1)); printf "${RED}[FAIL]${RST} %s\n" "$1"; }
warn()  { WARN=$((WARN+1)); printf "${YLW}[WARN]${RST} %s\n" "$1"; }
info()  { printf "${CYN}[INFO]${RST} %s\n" "$1"; }
header(){ printf "\n${CYN}=== %s ===${RST}\n" "$1"; }

FIRMWARE="${1:-}"
TMPDIR=$(mktemp -d)
trap 'rm -rf "$TMPDIR"' EXIT

# ---------------------------------------------------------------------------
# Locate picotool
# ---------------------------------------------------------------------------
header "Locating picotool"

PICOTOOL=""
for candidate in \
    "$(command -v picotool 2>/dev/null || true)" \
    "$HOME/.platformio/packages/tool-picotool-rp2040-earlephilhower/picotool" \
    "$HOME/.platformio/packages/tool-rp2040tools/picotool" \
    "$HOME/.arduino15/packages/rp2040/tools/pqt-picotool/4.1.0-1aec55e/picotool"; do
    if [[ -n "$candidate" && -x "$candidate" ]]; then
        PICOTOOL="$candidate"
        break
    fi
done

if [[ -z "$PICOTOOL" ]]; then
    fail "picotool not found. Install it or pass its path via PATH."
    echo "  Try: sudo apt install picotool"
    echo "  Or build from https://github.com/raspberrypi/picotool"
    exit 1
fi

PICOTOOL_VER=$("$PICOTOOL" version 2>&1 || true)
info "Using: $PICOTOOL ($PICOTOOL_VER)"

# ---------------------------------------------------------------------------
# 1. USB Enumeration
# ---------------------------------------------------------------------------
header "1. USB Enumeration"

# Show all RP2xxx-related USB devices (VID 2e8a)
ALL_RP_USB=$(lsusb 2>/dev/null | grep -i "2e8a" || true)
if [[ -n "$ALL_RP_USB" ]]; then
    info "All Raspberry Pi USB devices:"
    echo "$ALL_RP_USB" | while read -r line; do info "  $line"; done
fi

# Look specifically for RP2350/RP2040 boot or application devices (exclude debug probes)
USB_LINE=$(lsusb 2>/dev/null | grep -i "2e8a" | grep -v "CMSIS-DAP\|debugprobe\|Picoprobe" || true)
if [[ -z "$USB_LINE" ]]; then
    fail "No RP2xxx target device found on USB (debug probes excluded)"
    echo "  Make sure the board is connected and powered."
    echo "  If OpenOCD left it halted, try unplugging and re-plugging the board."
fi

if echo "$USB_LINE" | grep -q "000f"; then
    warn "Device is in BOOTSEL mode (PID 000f = RP2350 Boot)"
elif echo "$USB_LINE" | grep -q "0003"; then
    warn "Device is in BOOTSEL mode (PID 0003 = RP2040 Boot)"
elif [[ -n "$USB_LINE" ]]; then
    pass "Device appears to be running application firmware"
fi

# ---------------------------------------------------------------------------
# 2. Device Information (picotool info -a)
# ---------------------------------------------------------------------------
header "2. Device Information"

DEVINFO=$("$PICOTOOL" info -a 2>&1) || {
    fail "picotool info failed — device may not be in BOOTSEL mode or is disconnected"
    echo "$DEVINFO"
    info "Skipping picotool-based checks. SWD checks may still work (see below)."
    DEVINFO=""
}

if [[ -z "$DEVINFO" ]]; then
    # Jump past all picotool-dependent sections
    BOOT_TYPE=""
    CHIP_TYPE=""
    USB_AFTER=""
    # Skip to section 8
else

echo "$DEVINFO"
echo ""

# Parse key fields
parse_field() { echo "$DEVINFO" | sed -n "s/^.*$1:[[:space:]]*//p" | head -1 || true; }

# Parse "type" carefully — avoid matching "image type" or "boot type"
CHIP_TYPE=$(echo "$DEVINFO" | sed -n 's/^ type:[[:space:]]*//p' | head -1 || true)
CHIP_REV=$(parse_field "revision")
CHIP_PKG=$(parse_field "package")
FLASH_SIZE=$(parse_field "flash size")
FLASH_DEVINFO=$(parse_field "flash devinfo")
BOOT_TYPE=$(parse_field "boot type")
LAST_PART=$(parse_field "last booted partition")
SECURE_BOOT=$(parse_field "secure boot")
IMG_TYPE=$(echo "$DEVINFO" | grep "image type" | head -1 | sed 's/.*image type:\s*//' || true)
REBOOT0=$(parse_field "reboot param 0")
REBOOT1=$(parse_field "reboot param 1")

# Chip type
if [[ "$CHIP_TYPE" == "RP2350" ]]; then
    pass "Chip type: RP2350"
else
    warn "Chip type: $CHIP_TYPE (expected RP2350)"
fi

# Revision
if [[ "$CHIP_REV" == "Unknown" ]]; then
    warn "Silicon revision: Unknown — may be engineering sample or unsupported stepping"
else
    pass "Silicon revision: $CHIP_REV"
fi

# Flash size
if [[ -n "$FLASH_SIZE" ]]; then
    info "Flash size reported: $FLASH_SIZE"
else
    fail "No flash size reported"
fi

# Flash devinfo
info "Flash devinfo register: $FLASH_DEVINFO"

# Boot type
if [[ "$BOOT_TYPE" == "bootsel" ]]; then
    fail "Boot type: bootsel — device did NOT boot from flash"
else
    pass "Boot type: $BOOT_TYPE"
fi

# Last booted partition
if [[ "$LAST_PART" == "none" ]]; then
    fail "Last booted partition: none — flash has NEVER successfully booted"
else
    pass "Last booted partition: $LAST_PART"
fi

# Secure boot
if [[ "$SECURE_BOOT" == "0" ]]; then
    pass "Secure boot: disabled"
else
    warn "Secure boot: enabled ($SECURE_BOOT) — image must be signed"
fi

# Image type
if [[ -n "$IMG_TYPE" ]]; then
    info "Image type in flash: $IMG_TYPE"
else
    warn "No image found in flash"
fi

# Reboot params (informational)
info "Reboot param 0: $REBOOT0"
info "Reboot param 1: $REBOOT1"

# ---------------------------------------------------------------------------
# 3. OTP Configuration
# ---------------------------------------------------------------------------
header "3. OTP Boot Configuration"

otp_get() {
    "$PICOTOOL" otp get "$1" 2>&1
}

# BOOT_FLAGS0
BF0=$(otp_get BOOT_FLAGS0)
BF0_VAL=$(echo "$BF0" | grep -oP 'VALUE\s+\K0x[0-9a-fA-F]+' || echo "")
info "BOOT_FLAGS0 raw: $BF0_VAL"

check_otp_bit() {
    local label="$1" pattern="$2" good_val="$3" bad_msg="$4" good_msg="$5"
    local val
    val=$(echo "$BF0" | grep -oP "(?<=$pattern\s).*=\s*\K[0-9]+" || echo "")
    if [[ "$val" == "$good_val" ]]; then
        pass "$good_msg"
    else
        fail "$bad_msg (value=$val)"
    fi
}

# Check critical BOOT_FLAGS0 bits
if echo "$BF0" | grep -q "DISABLE_FLASH_BOOT.*= 0"; then
    pass "Flash boot: enabled (DISABLE_FLASH_BOOT=0)"
else
    fail "Flash boot: DISABLED in OTP — this prevents booting from flash"
fi

if echo "$BF0" | grep -q "FLASH_IO_VOLTAGE_1V8.*= 0"; then
    pass "QSPI voltage: 3.3V mode (FLASH_IO_VOLTAGE_1V8=0)"
else
    warn "QSPI voltage: 1.8V mode is set — verify flash chip supports 1.8V"
fi

if echo "$BF0" | grep -q "FLASH_DEVINFO_ENABLE.*= 0"; then
    info "FLASH_DEVINFO_ENABLE=0 — bootrom uses default flash config (CS0=16MiB)"
else
    info "FLASH_DEVINFO_ENABLE=1 — bootrom uses OTP flash device info"
fi

if echo "$BF0" | grep -q "DISABLE_BOOTSEL_USB_PICOBOOT_IFC.*= 0"; then
    pass "PICOBOOT USB interface: enabled"
else
    warn "PICOBOOT USB interface: disabled in OTP"
fi

# BOOT_FLAGS1
BF1=$(otp_get BOOT_FLAGS1)
if echo "$BF1" | grep -q "DOUBLE_TAP.*= 0"; then
    info "Double-tap BOOTSEL: disabled"
else
    info "Double-tap BOOTSEL: enabled"
fi

# CRIT0
CRIT0=$(otp_get CRIT0)
if echo "$CRIT0" | grep -q "ARM_DISABLE.*= 0"; then
    pass "ARM cores: enabled"
else
    fail "ARM cores: DISABLED in OTP"
fi

# CRIT1
CRIT1=$(otp_get CRIT1)
if echo "$CRIT1" | grep -q "SECURE_BOOT_ENABLE.*= 0"; then
    pass "Secure boot (CRIT1): not enforced"
else
    fail "Secure boot enforced via CRIT1 — unsigned images will be rejected"
fi

if echo "$CRIT1" | grep -q "BOOT_ARCH.*= 0"; then
    pass "Default boot arch: ARM (BOOT_ARCH=0)"
else
    warn "Default boot arch: RISC-V (BOOT_ARCH=1)"
fi

# FLASH_DEVINFO
FDI=$(otp_get FLASH_DEVINFO)
info "FLASH_DEVINFO OTP contents:"
echo "$FDI" | grep -E "VALUE|CS0_SIZE|CS1_SIZE|D8H" | while read -r line; do
    info "  $line"
done

# ---------------------------------------------------------------------------
# 4. Partition Table
# ---------------------------------------------------------------------------
header "4. Partition Table"

PTINFO=$("$PICOTOOL" partition info -m rp2350-arm-s 2>&1 || true)
echo "$PTINFO"

if echo "$PTINFO" | grep -qi "no partition table"; then
    info "No partition table — using un-partitioned flash (normal for simple images)"
fi

if echo "$PTINFO" | grep -q "rp2350-arm-s"; then
    pass "Family rp2350-arm-s is accepted for download"
else
    fail "Family rp2350-arm-s is NOT accepted"
fi

# ---------------------------------------------------------------------------
# 5. Flash Header Inspection
# ---------------------------------------------------------------------------
header "5. Flash Header / Vector Table"

FLASH_HDR="$TMPDIR/flash_header.bin"
"$PICOTOOL" save -r 0x10000000 0x10000200 "$FLASH_HDR" 2>/dev/null || {
    fail "Could not read flash at 0x10000000"
}

if [[ -f "$FLASH_HDR" ]]; then
    # Read first two 32-bit words as little-endian using od
    SP_HEX=$(od -A n -t x4 -N 4 "$FLASH_HDR" 2>/dev/null | tr -d ' ' || true)
    RST_HEX=$(od -A n -t x4 -N 4 -j 4 "$FLASH_HDR" 2>/dev/null | tr -d ' ' || true)

    info "Initial SP:    0x$SP_HEX"
    info "Reset vector:  0x$RST_HEX"

    if [[ -n "$SP_HEX" ]]; then
        SP_DEC=$((16#${SP_HEX}))
        if (( SP_DEC >= 0x20000000 && SP_DEC <= 0x20082000 )); then
            pass "Initial SP (0x$SP_HEX) points to valid RAM region"
        else
            fail "Initial SP (0x$SP_HEX) does NOT point to RAM"
        fi
    fi

    if [[ -n "$RST_HEX" ]]; then
        RST_DEC=$((16#${RST_HEX}))
        if (( RST_DEC >= 0x10000000 && RST_DEC <= 0x11000000 && (RST_DEC & 1) == 1 )); then
            pass "Reset vector (0x$RST_HEX) points to flash with Thumb bit set"
        else
            fail "Reset vector (0x$RST_HEX) looks invalid"
        fi
    fi

    # Check for IMAGE_DEF block marker (0xffffded3) in first 512 bytes
    if xxd "$FLASH_HDR" | grep -qi "d3de ffff"; then
        pass "IMAGE_DEF block marker (0xffffded3) found in first 512 bytes"
    else
        fail "No IMAGE_DEF block marker found — bootrom cannot identify the image"
    fi
fi

# ---------------------------------------------------------------------------
# 6. Firmware Verification (if UF2 provided)
# ---------------------------------------------------------------------------
header "6. Firmware Verification"

if [[ -n "$FIRMWARE" && -f "$FIRMWARE" ]]; then
    info "Verifying flash against: $FIRMWARE"
    VERIFY_OUT=$("$PICOTOOL" verify "$FIRMWARE" 2>&1) || true
    echo "$VERIFY_OUT"

    if echo "$VERIFY_OUT" | grep -q "OK"; then
        pass "Flash contents match provided firmware"
    else
        fail "Flash contents do NOT match provided firmware"
    fi

    # Also show binary info
    info "Firmware binary info:"
    "$PICOTOOL" info -b "$FIRMWARE" 2>&1 | head -10
else
    # Check if there's a built firmware in the PIO build dir
    PIO_FW="$(dirname "$0")/.pio/build/motor_controller/firmware.uf2"
    if [[ -f "$PIO_FW" ]]; then
        info "Found PlatformIO firmware at $PIO_FW — verifying"
        VERIFY_OUT=$("$PICOTOOL" verify "$PIO_FW" 2>&1) || true
        if echo "$VERIFY_OUT" | grep -q "OK"; then
            pass "Flash contents match PlatformIO firmware"
        else
            warn "Flash contents do NOT match PlatformIO firmware (may have been overwritten)"
        fi
    else
        info "No firmware file specified and no PIO build found. Skipping verify."
        info "  Usage: $0 path/to/firmware.uf2"
    fi
fi

# ---------------------------------------------------------------------------
# 7. Reboot-to-Application Test
# ---------------------------------------------------------------------------
header "7. Reboot-to-Application Test"

info "Attempting picotool reboot -a (boot into application)..."
"$PICOTOOL" reboot -a 2>&1 || true

info "Waiting 4 seconds for device to re-enumerate..."
sleep 4

USB_AFTER=$(lsusb 2>/dev/null | grep -i "2e8a" | grep -v "CMSIS-DAP\|debugprobe\|Picoprobe" || true)
info "USB after reboot: $USB_AFTER"

if [[ -z "$USB_AFTER" ]]; then
    # Device disappeared — might be running (no USB in firmware) or crashed
    warn "Device not found on USB after reboot"
    info "  If firmware has no USB support, this may be normal."
    info "  Check UART (TX=GP0 at 115200 baud) for serial output."
elif echo "$USB_AFTER" | grep -q "000f\|0003"; then
    fail "Device returned to BOOTSEL mode after reboot-to-application"
    info "  The bootrom is refusing to boot from flash."
else
    pass "Device appears to be running application firmware"
fi

# Re-check boot type if still in BOOTSEL
if echo "$USB_AFTER" | grep -q "000f\|0003"; then
    sleep 1
    POST_INFO=$("$PICOTOOL" info -a 2>&1 || true)
    POST_BOOT=$(echo "$POST_INFO" | sed -n 's/^.*boot type:[[:space:]]*//p' | head -1)
    POST_PART=$(echo "$POST_INFO" | sed -n 's/^.*last booted partition:[[:space:]]*//p' | head -1)
    POST_R0=$(echo "$POST_INFO" | sed -n 's/^.*reboot param 0:[[:space:]]*//p' | head -1)
    POST_R1=$(echo "$POST_INFO" | sed -n 's/^.*reboot param 1:[[:space:]]*//p' | head -1)
    POST_BOOT="${POST_BOOT:-unknown}"
    POST_PART="${POST_PART:-unknown}"
    POST_R0="${POST_R0:-unknown}"
    POST_R1="${POST_R1:-unknown}"

    info "Post-reboot diagnostics:"
    info "  boot type:            $POST_BOOT"
    info "  last booted partition: $POST_PART"
    info "  reboot param 0:       $POST_R0"
    info "  reboot param 1:       $POST_R1"
fi

fi  # end of picotool-dependent sections (if DEVINFO was non-empty)

# ---------------------------------------------------------------------------
# 8. Debug Probe Detection
# ---------------------------------------------------------------------------
header "8. Debug Probe Detection"

PROBES=$(lsusb 2>/dev/null | grep -iE "cmsis-dap|debugprobe|picoprobe|FTDI|jlink|segger|st-link|stlink" || true)
if [[ -n "$PROBES" ]]; then
    pass "Debug probe(s) found:"
    echo "$PROBES"
else
    info "No SWD/JTAG debug probe detected on USB"
    info "  Connect a Raspberry Pi Debug Probe or Picoprobe to read QSPI_SS GPIO state"
fi

# ---------------------------------------------------------------------------
# 9. Serial Port Check
# ---------------------------------------------------------------------------
header "9. Serial Port Check"

SERIAL_DEVS=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true)
if [[ -n "$SERIAL_DEVS" ]]; then
    info "Serial devices found:"
    for dev in $SERIAL_DEVS; do
        DEV_INFO=$(udevadm info --query=property "$dev" 2>/dev/null | grep -E "ID_MODEL=|ID_VENDOR=" || echo "  (unknown)")
        echo "  $dev  $DEV_INFO"
    done
else
    info "No serial ports found (/dev/ttyACM* or /dev/ttyUSB*)"
fi

# ---------------------------------------------------------------------------
# 10. SWD Register Probe (requires debug probe + OpenOCD with RP2350 support)
# ---------------------------------------------------------------------------
header "10. SWD QSPI Pin State (via debug probe)"

OPENOCD=""
OPENOCD_SCRIPTS=""
for ocd_bin in \
    "$HOME/.platformio/packages/tool-openocd-rp2040-earlephilhower/bin/openocd" \
    "$(command -v openocd 2>/dev/null || true)"; do
    if [[ -n "$ocd_bin" && -x "$ocd_bin" ]]; then
        OPENOCD="$ocd_bin"
        # Derive scripts dir
        OPENOCD_SCRIPTS="$(dirname "$(dirname "$ocd_bin")")/share/openocd/scripts"
        [[ -d "$OPENOCD_SCRIPTS" ]] || OPENOCD_SCRIPTS="/usr/share/openocd/scripts"
        break
    fi
done

# Check if a CMSIS-DAP probe is actually connected
HAS_DAP_PROBE=$(lsusb 2>/dev/null | grep -iE "cmsis-dap" || true)

if [[ -z "$OPENOCD" ]]; then
    info "OpenOCD not found — skipping SWD register probe"
elif [[ -z "$HAS_DAP_PROBE" ]]; then
    info "No CMSIS-DAP debug probe detected — skipping SWD register probe"
elif [[ ! -f "$OPENOCD_SCRIPTS/target/rp2350.cfg" ]]; then
    info "OpenOCD lacks RP2350 target config — skipping SWD register probe"
else
    info "Debug probe detected, attempting SWD connection..."

    # Write a small TCL script for register reads
    SWD_TCL="$TMPDIR/swd_diag.tcl"
    cat > "$SWD_TCL" << 'TCLEOF'
transport select swd
source [find target/swj-dp.tcl]
swj_newdap rp2350 swd -expected-id 0x00040927
dap create rp2350.dap -chain-position rp2350.swd -adiv6
target create rp2350.core0 cortex_m -dap rp2350.dap -ap-num 0x2000 -coreid 0
cortex_m reset_config sysresetreq

proc rd {label addr} {
    set val [read_memory $addr 32 1]
    echo [format "SWD_REG %s 0x%08x 0x%08x" $label $addr $val]
}

init
halt
sleep 500

rd QSPI_SS_STATUS   0x40030008
rd QSPI_SS_CTRL     0x4003000c
rd PADS_QSPI_SS     0x40040018
rd XOSC_STATUS      0x40048004
rd WATCHDOG_REASON   0x400d8008

resume
shutdown
TCLEOF

    SWD_OUT=$(timeout 15 "$OPENOCD" -s "$OPENOCD_SCRIPTS" \
        -f interface/cmsis-dap.cfg \
        -c "adapter speed 1000" \
        -f "$SWD_TCL" 2>&1 || true)

    if echo "$SWD_OUT" | grep -q "SWD_REG"; then
        pass "SWD connection succeeded"
        echo ""

        # Parse QSPI_SS_STATUS
        SS_STATUS=$(echo "$SWD_OUT" | grep "SWD_REG QSPI_SS_STATUS" | awk '{print $4}')
        SS_CTRL=$(echo "$SWD_OUT" | grep "SWD_REG QSPI_SS_CTRL" | awk '{print $4}')
        PADS_SS=$(echo "$SWD_OUT" | grep "SWD_REG PADS_QSPI_SS" | awk '{print $4}')
        XOSC_ST=$(echo "$SWD_OUT" | grep "SWD_REG XOSC_STATUS" | awk '{print $4}')
        WDG_REASON=$(echo "$SWD_OUT" | grep "SWD_REG WATCHDOG_REASON" | awk '{print $4}')

        info "QSPI_SS_STATUS:  $SS_STATUS"
        info "QSPI_SS_CTRL:    $SS_CTRL"
        info "PADS_QSPI_SS:    $PADS_SS"
        info "XOSC_STATUS:     $XOSC_ST"
        info "WATCHDOG_REASON: $WDG_REASON"
        echo ""

        # Decode PADS_QSPI_SS
        if [[ -n "$PADS_SS" ]]; then
            PADS_VAL=$((16#${PADS_SS#0x}))
            PAD_IE=$(( (PADS_VAL >> 6) & 1 ))
            PAD_PUE=$(( (PADS_VAL >> 3) & 1 ))
            PAD_PDE=$(( (PADS_VAL >> 2) & 1 ))
            info "  SS pad: IE=$PAD_IE (input enable), PUE=$PAD_PUE (pull-up), PDE=$PAD_PDE (pull-down)"
        fi

        # Decode QSPI_SS_STATUS
        if [[ -n "$SS_STATUS" ]]; then
            SS_VAL=$((16#${SS_STATUS#0x}))
            INFROMPAD=$(( (SS_VAL >> 17) & 1 ))
            OUTTOPAD=$(( (SS_VAL >> 9) & 1 ))
            OETOPAD=$(( (SS_VAL >> 13) & 1 ))

            if (( INFROMPAD == 0 )); then
                fail "QSPI_SS reads LOW (INFROMPAD=0) — pin is being held to ground"
                if [[ -n "$PADS_SS" ]] && (( PAD_PUE == 1 )); then
                    fail "  Pull-up is ENABLED but pin still reads LOW — external short to GND"
                fi
            else
                pass "QSPI_SS reads HIGH (INFROMPAD=1) — pin is not stuck"
            fi
        fi

        # Decode XOSC_STATUS
        if [[ -n "$XOSC_ST" ]]; then
            XOSC_VAL=$((16#${XOSC_ST#0x}))
            XOSC_STABLE=$(( (XOSC_VAL >> 31) & 1 ))
            if (( XOSC_STABLE == 1 )); then
                pass "Crystal oscillator: stable and running"
            else
                fail "Crystal oscillator: NOT stable"
            fi
        fi
    else
        warn "SWD register read failed or timed out"
        # Show any error info
        echo "$SWD_OUT" | grep -iE "error|fail|timeout" | head -5
    fi
fi

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
header "SUMMARY"

printf "  ${GRN}PASS: %d${RST}  ${RED}FAIL: %d${RST}  ${YLW}WARN: %d${RST}\n\n" "$PASS" "$FAIL" "$WARN"

if (( FAIL > 0 )); then
    echo "Diagnosis:"
    echo ""

    # Determine likely root cause
    if echo "$USB_AFTER" | grep -q "000f\|0003"; then
        echo "  The board always returns to BOOTSEL mode. Given that:"
        echo "    - Flash is accessible and firmware verifies OK"
        echo "    - OTP has no boot restrictions"
        echo "    - Multiple different firmwares all fail to boot"
        echo ""
        echo "  MOST LIKELY CAUSE: QSPI_SS (BOOTSEL) pin is stuck LOW"
        echo ""
        echo "  The RP2350 bootrom samples QSPI_SS at reset. If it reads"
        echo "  low, the bootrom enters BOOTSEL immediately — it never"
        echo "  attempts flash boot."
        echo ""
        echo "  Recommended actions:"
        echo "    1. Measure resistance between QSPI_SS and GND with a"
        echo "       multimeter. Should be OPEN when BOOTSEL button is"
        echo "       not pressed."
        echo "    2. Visually inspect the BOOTSEL button and QSPI_SS"
        echo "       trace for solder bridges or shorts to ground."
        echo "    3. If this is a custom board without a BOOTSEL button,"
        echo "       ensure QSPI_SS has a pull-up (10k to IOVDD)."
        echo "    4. Connect an SWD debug probe and read IO_QSPI"
        echo "       GPIO_QSPI_SS_STATUS register at 0x40030008."
    fi
fi
