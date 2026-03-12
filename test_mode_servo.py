import time
import pigpio
from st3215 import ST3215

SERVO_ID = 1
PIN_SERVO_X = 12
PULSE_NEG45 = 1000
PULSE_POS45 = 2000

# Step 1: send serial command to switch to PWM mode
servo = ST3215('/dev/ttyS0')

res = servo.UnLockEprom(SERVO_ID)
print(f"UnLockEprom: {res}")

comm, error = servo.SetMode(SERVO_ID, 2)  # mode 2 = PWM open-loop
print(f"SetMode(2): comm={comm}, error={error}")
if comm != 0 or error != 0:
    servo.portHandler.closePort()
    raise RuntimeError(f"SetMode failed — comm={comm}, error={error}. Check servo ID and wiring.")

res = servo.LockEprom(SERVO_ID)
print(f"LockEprom: {res}")

servo.portHandler.closePort()
time.sleep(0.1)                 # give it a moment to switch

# Step 2: now drive PWM — line is no longer being driven by serial bus logic
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Cannot connect to pigpiod")

try:
    pi.set_mode(PIN_SERVO_X, pigpio.OUTPUT)
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