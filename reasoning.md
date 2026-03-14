# Config Change Log

This file tracks all `config.json` changes — what was changed, why, and whether it helped.

---

<!-- Template for new entries:

## YYYY-MM-DD — Brief description

**Changed:** `param_name`: old_value → new_value

**Problem:** What wasn't working / what prompted the change.

**Reasoning:** Why these specific values were chosen.

**Result:** _(Fill in after testing)_ Did it help? What improved or got worse?

---
-->

## 2026-03-14 — Keep servo tracking during TENS_ACTIVE state

**Changed:** `control_node.cpp` logic (not config.json) — servo PID now runs during TENS_ACTIVE state alongside TENS stimulation, instead of centering the servo and relying on TENS alone.

**Problem:** When error exceeded ~165px (tens_activation_px 150 + hysteresis 15), the system transitioned from SERVO_ACTIVE to TENS_ACTIVE, which centered the servo and only ran TENS. If TENS wasn't effectively moving the aim, the servo sat idle for seconds while needing to turn left to reach the target.

**Reasoning:** TENS should supplement the servo, not replace it. The servo can handle errors well beyond 165px (servo_range=398, pid_to_pos_scale=2.0). Centering the servo during TENS was throwing away useful tracking ability.

**Result:** Confirmed working — servo tracks during TENS_ACTIVE in both subsequent runs.

---

## 2026-03-14 — Slash derivative gains to stop wrong-direction servo slams

**Changed:**
- `kd_flick`: 0.3 → 0.0
- `kd_settle`: 0.8 → 0.05
- `deriv_filter_alpha`: 0.3 → 0.5

**Problem:** The derivative term was routinely 10-50x larger than P, often flipping the servo to the WRONG direction. Examples from logs:
- Tick 400: err=+25.7px (target RIGHT), D=-561 → total output sends servo LEFT
- Tick 7400: err=-25.9px (target LEFT), D=+652 → total output sends servo RIGHT
- Tick 9400: err=-107.7px, D=+2409 → output goes POSITIVE despite large negative error

The clamp (±284) was saving us by limiting the damage, and the system was only hitting targets BECAUSE of the clamp, not because of the PID. The P term alone (err * Kp * scale) already maps well to the servo range — 50px error → offset 300, nearly saturating ±284.

**Reasoning:**
- FLICK Kd → 0.0: For large errors, pure proportional is correct. The servo should just go to the target fast. D was adding noise, not damping.
- SETTLE Kd: 0.8 → 0.05: Keep a tiny amount for fine settling to reduce overshoot, but 0.05 keeps it proportional to P rather than dominating it.
- deriv_filter_alpha: 0.3 → 0.5: More smoothing on whatever derivative remains, to prevent spikes from packet arrival.

**Result:** Major improvement — ~100 fires in 27s (~3.7/s). FLICK Kd=0 is clean. But kd_settle=0.05 still causes wrong-direction outputs on packet arrival (raw_deriv 17k-32k → D of 400-800, overpowering P).

---

## 2026-03-14 — Zero out settle derivative — pure PI control

**Changed:** `kd_settle`: 0.05 → 0.0

**Problem:** Even at Kd=0.05, the derivative term still causes wrong-direction servo commands on packet arrival. Raw derivatives of 17,000-32,000 px/s produce D terms of 400-800, overpowering the P term:
- Tick 7300: err=+42px, P=+126, D=-438 → output=-310 (WRONG WAY)
- Tick 12700: err=+35px, P=+105, D=-219 → output=-114 (WRONG WAY)
- Tick 12100: err=+39px, P=+118, D=-128 → output=-10 (nearly cancelled)

The root cause is the 1kHz loop vs ~10Hz vision packets: error is constant for ~100 ticks, then jumps 30-50px in one tick, creating artificial derivatives of 30,000+ px/s. No Kd value can handle this without wrong-direction outputs.

**Reasoning:** The overshoot damping system (gain_mult=0.5 for 10 frames on sign flip) already handles oscillation control — verified working in logs (DAMPED entries appear correctly). D is redundant and harmful. Pure PI gives clean, proportional response.

**Result:** _(Fill in after testing)_

---

## 2026-03-14 — Refactor TENS to pulsed model with atomic mutual exclusion

**Changed:**
- `tens_max_intensity`: removed → replaced by `tens_intensity`: 31
- `tens_min_intensity`: removed
- `tens_activation_px`: 150.0 → 200.0
- New: `tens_pulse_ramp_up_ms`: 25
- New: `tens_pulse_hold_ms`: 50
- New: `tens_pulse_ramp_down_ms`: 25
- New: `tens_servo_gain_multiplier`: 0.5

**Problem:** TENS was running as continuous intensity modulation — pot wiper set and left on. This doesn't match how TENS actually works (pulsed stimulation). Also lacked hard atomic mutual exclusion between channels (used sequential off/on which could glitch), and servo ran at full gain during TENS pulses (fighting the muscle stimulation).

**Reasoning:**
- Pulsed model (25ms ramp up → 50ms hold → 25ms ramp down = 100ms total) mimics actual TENS pulse behavior
- Fixed intensity of 31/127 instead of distance-proportional — simpler, safer starting point
- `std::atomic<bool>` CAS gate on `tens_pulse_in_progress` prevents any concurrent pulse (like solenoid pattern)
- `std::atomic<int>` `tens_active_channel` tracks which channel is live, force-kills other channel before activation
- Servo gain multiplier (0.5) during TENS pulse reduces servo fighting the muscle stimulation
- Activation threshold raised to 200px — TENS should only fire for large errors where muscle assist matters
- 200ms cooldown between pulses prevents overstimulation
- Detached thread (matches solenoid pattern) — doesn't block 1kHz loop

**Result:** Software is working correctly — pulses fire with clean 100ms lifecycle, SPI writes succeed (rc=2), mutual exclusion holds, cooldown enforced. But person never felt the shock. Intensity 31/127 (~2.4kΩ) is below perceptible threshold. Raising to 80.

---

## 2026-03-14 — Raise TENS intensity from 31 to 80

**Changed:** `tens_intensity`: 31 → 80 (then user set to 60)

**Problem:** TENS pulses fired correctly (confirmed in logs: clean START/END cycles, SPI rc=2, proper ramp up/hold/down), but the person never felt any stimulation. At wiper=31/127 on a 10kΩ MCP4131, the resistance is ~2.4kΩ — too low to produce perceptible TENS output through skin electrodes.

**Reasoning:** MCP4131 wiper=80/127 gives ~6.3kΩ, which should produce noticeable stimulation. The previous continuous config also used 63 with no reported sensation, suggesting the threshold is above 50%. Starting at 80 (~63%) as a reasonable first perceptible level. Hard safety cap in code is 127 max.

**Result:** Still no sensation at intensity=60. Root cause found: not hardware — it was the TCON register manipulation added in commit c4c74b2. See next entry.

---

## 2026-03-14 — Remove TCON shutdown/wakeup — root cause of TENS not working

**Changed:** Removed `tens_shutdown()` (TCON=0x00) and `tens_wakeup()` (TCON=0xFF) functions entirely. Reverted `tens_off()` to simply set wiper=0, matching the original working code from commits 44ece17/8b0ea81.

**Problem:** TENS never produced perceptible output despite software pulses firing correctly. Compared git history: the original "Add TENS" commit (44ece17, 1:02 PM Mar 13) and "Ensure TENS channels can't activate same time" (8b0ea81, 1:22 PM) both worked — they simply set wiper=0 for off and wiper=value for on. Then "Troubleshoot TENS always on" (c4c74b2, 2:14 PM) added TCON register manipulation to "harder off" the pot by disconnecting all terminals. After that commit, TENS never worked again.

**Reasoning:** The TCON register write (address 0x04, command byte 0x40) disconnects all resistor network terminals. The corresponding wakeup (TCON=0xFF) was supposed to reconnect them, but either: (1) the command byte encoding was wrong for this chip variant, (2) TCON writes aren't reliable at 1MHz SPI, or (3) there's a timing issue where the wakeup doesn't take effect before the wiper write. Regardless, the original approach (wiper=0 for off) was working and is sufficient — ~75Ω wiper resistance at position 0 is effectively off for TENS purposes.

**Result:** _(Fill in after testing)_

---
