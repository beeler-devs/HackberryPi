#!/usr/bin/env python3
"""Servo sweep test — verifies wiring by driving GPIO 12 between -45° and +45°.

Usage:
    sudo pigpiod          # start daemon if not running
    sudo python3 test_servo.py
"""

import time
import pigpio

PIN_SERVO_X = 12
PULSE_NEG45 = 1000  # µs → -45°
PULSE_POS45 = 2000  # µs → +45°

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Cannot connect to pigpiod — run 'sudo pigpiod' first")

try:
    pi.set_mode(PIN_SERVO_X, pigpio.OUTPUT)
    print(f"Sweeping servo on GPIO {PIN_SERVO_X} between -45° and +45° every 3s")
    print("Press Ctrl+C to stop\n")

    pos = PULSE_NEG45
    while True:
        pi.set_servo_pulsewidth(PIN_SERVO_X, pos)
        label = "-45°" if pos == PULSE_NEG45 else "+45°"
        print(f"  → {label}  ({pos} µs)")
        pos = PULSE_POS45 if pos == PULSE_NEG45 else PULSE_NEG45
        time.sleep(3)

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    pi.set_servo_pulsewidth(PIN_SERVO_X, 0)
    pi.stop()
    print("Cleanup done.")
