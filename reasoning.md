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
