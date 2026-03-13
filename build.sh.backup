#!/usr/bin/env bash
# build.sh — Compile control_node.cpp for Raspberry Pi 4/5
#
# Prerequisites (run once):
#   sudo apt update
#   sudo apt install -y pigpio libpigpio-dev
#
# Run the binary as root (pigpio requires /dev/mem access):
#   sudo ./control_node
#
# Alternatively, start the pigpiod daemon and adapt control_node.cpp
# to use the pigpiod client API (pigpiod_if2.h) to avoid running as root.

set -euo pipefail

# ── Dependency check ──────────────────────────────────────────────────────────
if ! dpkg -l libpigpio-dev 2>/dev/null | grep -q '^ii'; then
    echo "ERROR: libpigpio-dev is not installed."
    echo "       Run: sudo apt install pigpio libpigpio-dev"
    exit 1
fi

TARGET="control_node"
SOURCE="control_node.cpp"

if [[ ! -f "$SOURCE" ]]; then
    echo "ERROR: $SOURCE not found in current directory."
    exit 1
fi

echo "[BUILD] Compiling $SOURCE → $TARGET ..."

g++ \
    -O3 \               \
    -std=c++17 \        \
    -march=native \     \
    -Wall -Wextra \     \
    -Wno-unused-result  \
    -o "$TARGET"        \
    "$SOURCE"           \
    -lpigpio            \
    -lpthread           \
    -lrt

# Compiler flags explained:
#   -O3            Full optimization; critical for the 1kHz busy-spin loop.
#   -std=c++17     Required for std::clamp, std::atomic, structured init.
#   -march=native  Emit Cortex-A72 (Pi 4) or Cortex-A76 (Pi 5) instructions.
#                  Do NOT use this flag if cross-compiling on x86.
#   -Wall -Wextra  Surface warnings during development; safe to remove for prod.
#   -lpigpio       pigpio library: hardware PWM, GPIO, gpioDelay.
#   -lpthread      std::thread, std::atomic (pthreads backend).
#   -lrt           clock_gettime(), POSIX real-time extensions.

echo ""
echo "[BUILD] Success: ./$TARGET"
echo ""
echo "Usage:"
echo "  sudo ./$TARGET"
echo ""
echo "Verify pigpio daemon is NOT running (conflicts with direct pigpio):"
echo "  sudo systemctl stop pigpiod"
echo "  sudo systemctl disable pigpiod"
