# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Real-time computer vision servo tracking system split across two hardware nodes:

- **Vision Node** (`vision_node.py`) — runs on Jetson Nano; captures camera frames, detects cyan/teal targets via HSV color thresholding, transmits target position over UDP
- **Control Node** (`control_node.cpp`) — runs on Raspberry Pi 4/5; receives UDP packets, runs 1kHz PID control loop, drives ST3215 servo via Feetech SCS/STS serial protocol and a solenoid trigger via pigpio

## Environment

Use the `machine-learning` conda environment for all Python commands:
```bash
conda run -n machine-learning python vision_node.py
conda run -n machine-learning pip install <package>
```

## Build & Run

**Python vision node** (Jetson Nano):
```bash
conda run -n machine-learning python vision_node.py
```

**C++ control node** (Raspberry Pi — requires `libpigpio-dev`):
```bash
bash build.sh
sudo ./control_node
```
Must run as root (`sudo`) because pigpio requires direct hardware access.

**Camera diagnostics:**
```bash
conda run -n machine-learning python test_arducam_cv2_mac.py   # macOS
conda run -n machine-learning python test_arducam_cv2.py        # Linux/Windows
```

## Dependencies

Python: `opencv-python>=4.8.0`, `numpy>=1.24.0` (see `requirements.txt`)

C++: pigpio, pthreads, librt — compiled with `-O3 -march=native -std=c++17`

Servo serial: uses POSIX `termios` + `write()` — no additional library needed.

## Key Architecture Decisions

### Servo Control — Feetech SCS/STS Serial Protocol
The ST3215 servo is driven via half-duplex UART at 1 Mbps using the Feetech SCS/STS protocol (ported from the NeuromuscularAimAssist FPGA implementation). This replaces the previous pigpio PWM approach.

- **Position write packet** (9 bytes): `0xFF 0xFF ID 0x05 0x03 0x2A POS_L POS_H CSUM`
- **Torque enable packet** (8 bytes): `0xFF 0xFF ID 0x04 0x03 0x28 0x01 CSUM`
- Serial port: `/dev/ttyS0` at 1 Mbps 8N1
- Default servo ID: `0x01` (change to `0xFE` for broadcast during bring-up)
- 12-bit position range: 0-4095, operating window clamped to [1024, 3072], center at 2048
- First-order position smoothing: moves 1/4 of remaining error each 1ms update cycle

### Communication Protocol
Vision node → Control node: 16-byte big-endian UDP packets containing timestamp, target X, and crosshair reference X. Default: `10.0.0.2:5005`.

### PID State Machine (control_node.cpp)
Two-state controller based on pixel distance to target:
- **FLICK** (>30px): PD only — fast ballistic movement to target
- **SETTLE** (<30px): Full PID — precision lock, integral clamped to ±200px·s
- Target jump >50px resets integral to prevent windup

### Latency Optimizations
- Vision: Dedicated `FrameGrabber` thread + shallow queue (maxsize=2) decouples capture from processing; MJPEG over V4L2 reduces USB bandwidth
- Control: Busy-spin 1kHz loop (<5µs jitter), non-blocking UDP recv, no heap allocation in hot path, detached solenoid thread

### Key Tuning Parameters
| Parameter | Location | Value |
|-----------|----------|-------|
| Target strategy | `vision_node.py` | `'weighted'` (area-weighted centroid across all contours; alternatives: `'closest'`, `'largest'`) |
| HSV target bounds | `vision_node.py` | H[85,100], S[120,255], V[120,255] |
| Crosshair reference | `vision_node.py` | (320, 240) — adjust per monitor |
| UDP target host | `vision_node.py` | 10.0.0.2:5005 |
| Servo serial port | `control_node.cpp` | /dev/ttyS0 @ 1 Mbps |
| Servo ID | `control_node.cpp` | 0x01 (0xFE for broadcast) |
| Servo position center | `control_node.cpp` | 2048 (range: 1024–3072) |
| PID→Position scale | `control_node.cpp` | 1.0 (tune up if servo barely moves) |
| Position smoothing | `control_node.cpp` | 1/4 step per update (first-order) |
| Torque refresh interval | `control_node.cpp` | Every 512 position writes |
| Solenoid enabled (Pi) | `config.json` / `control_node.cpp` | `false` (solenoid fires from Jetson by default) |
| Solenoid enabled (Jetson) | `vision_node.py` | `SOLENOID_ENABLED = True` |
| Solenoid GPIO (Jetson) | `vision_node.py` | BOARD pin 18 |
| Trigger GPIO (Pi) | `control_node.cpp` | GPIO17 |
| FLICK gains | `control_node.cpp` | Kp=8.0, Kd=4.0 |
| SETTLE gains | `control_node.cpp` | Kp=5.0, Ki=0.15, Kd=3.0 |
| Fire threshold | `control_node.cpp` | <5px distance |
