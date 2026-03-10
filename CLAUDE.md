# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Real-time computer vision servo tracking system split across two hardware nodes:

- **Vision Node** (`vision_node.py`) — runs on Jetson Nano; captures camera frames, detects cyan/teal targets via HSV color thresholding, transmits target position over UDP
- **Control Node** (`control_node.cpp`) — runs on Raspberry Pi 4/5; receives UDP packets, runs 1kHz PID control loop, drives two servos and a solenoid trigger via pigpio

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

## Key Architecture Decisions

### Communication Protocol
Vision node → Control node: 24-byte big-endian UDP packets containing timestamp, target (x,y), and crosshair reference (x,y). Default: `192.168.1.50:5005`.

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
| HSV target bounds | `vision_node.py` | H[85,100], S[120,255], V[120,255] |
| Crosshair reference | `vision_node.py` | (320, 240) — adjust per monitor |
| UDP target host | `vision_node.py` | 192.168.1.50:5005 |
| Servo GPIO pins | `control_node.cpp` | X=GPIO12, Y=GPIO13, Trigger=GPIO17 |
| FLICK gains | `control_node.cpp` | Kp=8.0, Kd=4.0 |
| SETTLE gains | `control_node.cpp` | Kp=5.0, Ki=0.15, Kd=3.0 |
| Fire threshold | `control_node.cpp` | <5px distance |
