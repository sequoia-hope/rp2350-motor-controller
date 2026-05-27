#!/usr/bin/env bash
#
# Install the Raspberry Pi RP2350 RISC-V GCC toolchain (GCC 15)
# from the official pico-sdk-tools releases.
#
# Source: https://github.com/raspberrypi/pico-sdk-tools/releases
#
set -euo pipefail

TOOLCHAIN_VERSION="v2.2.0-3"
TOOLCHAIN_FILE="riscv-toolchain-15-x86_64-lin.tar.gz"
DOWNLOAD_URL="https://github.com/raspberrypi/pico-sdk-tools/releases/download/${TOOLCHAIN_VERSION}/${TOOLCHAIN_FILE}"
INSTALL_DIR="${HOME}/.local/riscv-toolchain"

echo "=== RP2350 RISC-V GCC Toolchain Installer ==="
echo ""
echo "  Version:  ${TOOLCHAIN_VERSION} (GCC 15)"
echo "  Source:   ${DOWNLOAD_URL}"
echo "  Install:  ${INSTALL_DIR}"
echo ""

# Check architecture
ARCH=$(uname -m)
if [ "$ARCH" != "x86_64" ]; then
    echo "Error: This script is for x86_64 Linux. Detected: $ARCH"
    echo "Check https://github.com/raspberrypi/pico-sdk-tools/releases for your platform."
    exit 1
fi

# Create a temp directory for the download
TMPDIR=$(mktemp -d)
trap 'rm -rf "$TMPDIR"' EXIT

echo "Downloading toolchain..."
wget -q --show-progress -O "${TMPDIR}/${TOOLCHAIN_FILE}" "$DOWNLOAD_URL"

echo ""
echo "Installing to ${INSTALL_DIR}..."
mkdir -p "$INSTALL_DIR"
tar xzf "${TMPDIR}/${TOOLCHAIN_FILE}" -C "$INSTALL_DIR" --strip-components=1

# Verify the installation
RISCV_GCC=$(find "$INSTALL_DIR" -name 'riscv32-*-gcc' -o -name 'riscv*-elf-gcc' 2>/dev/null | head -1)
if [ -z "$RISCV_GCC" ]; then
    # Try without strip-components in case the archive layout differs
    echo "Warning: Could not find gcc binary with strip-components=1, retrying..."
    rm -rf "${INSTALL_DIR:?}"/*
    tar xzf "${TMPDIR}/${TOOLCHAIN_FILE}" -C "$INSTALL_DIR"
    RISCV_GCC=$(find "$INSTALL_DIR" -name 'riscv32-*-gcc' -o -name 'riscv*-elf-gcc' 2>/dev/null | head -1)
fi

if [ -n "$RISCV_GCC" ]; then
    GCC_DIR=$(dirname "$RISCV_GCC")
    GCC_VERSION=$("$RISCV_GCC" --version 2>/dev/null | head -1 || echo "unknown")
    echo ""
    echo "Installation successful!"
    echo "  GCC binary: ${RISCV_GCC}"
    echo "  Version:    ${GCC_VERSION}"
    echo ""
    echo "Add the following to your shell profile (~/.bashrc or ~/.zshrc):"
    echo ""
    echo "  export PICO_TOOLCHAIN_PATH=\"${INSTALL_DIR}\""
    echo "  export PATH=\"${GCC_DIR}:\$PATH\""
    echo ""
    echo "Or for a Pico SDK CMake build:"
    echo ""
    echo "  cmake -DPICO_PLATFORM=rp2350-riscv -DPICO_TOOLCHAIN_PATH=${INSTALL_DIR} .."
    echo ""
else
    echo ""
    echo "Warning: Installation completed but could not locate a riscv gcc binary."
    echo "Check the contents of ${INSTALL_DIR} manually:"
    echo "  ls -R ${INSTALL_DIR}/bin/"
    exit 1
fi
