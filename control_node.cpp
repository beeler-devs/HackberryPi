/*
 * control_node.cpp — Raspberry Pi 4/5 Control Node
 *
 * Receives 16-byte UDP packets from the Jetson vision node, runs a state-driven
 * PID controller for X-axis, and drives a servo + solenoid trigger via pigpio.
 * Y-axis is user-controlled.
 *
 * Build:  see build.sh
 * Run:    sudo ./control_node   (pigpio requires /dev/mem access → root)
 *
 * Latency priorities:
 *   - Non-blocking UDP recv keeps control loop at 1kHz regardless of packet rate
 *   - Zero heap allocation in the hot path (all stack)
 *   - Busy-spin wait for sub-millisecond loop timing (lower jitter than nanosleep)
 *   - Detached trigger thread with atomic guard prevents solenoid blocking control
 */

#include <pigpio.h>

#include <sys/socket.h>
#include <sys/fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <csignal>
#include <algorithm>
#include <atomic>
#include <thread>

// ── GPIO Pin Assignments (BCM numbering) ─────────────────────────────────────
// GPIO 12 and 13 are hardware PWM-capable (ALT0 on Raspberry Pi 4/5).
// pigpio's gpioServo() manages the 50Hz PWM signal in the kernel; no busy-wait needed.
static constexpr int PIN_SERVO_X  = 12;   // Header pin 32
static constexpr int PIN_TRIGGER  = 17;   // Header pin 11 → MOSFET gate → solenoid

// ── Servo PWM Parameters ─────────────────────────────────────────────────────
// pigpio gpioServo() accepts pulse width in microseconds [500, 2500].
// Standard hobby servo: 1500µs = center, 500µs = -90°, 2500µs = +90°.
static constexpr float SERVO_MIN_US    = 500.0f;
static constexpr float SERVO_CENTER_US = 1500.0f;
static constexpr float SERVO_MAX_US    = 2500.0f;

// Mapping scalar: PID output (in pixel-error-weighted units) → µs offset from center.
// Formula: pulse_us = clamp(1500 + pid_output * PID_TO_US_SCALE, 500, 2500)
//
// Example with defaults:
//   error=15px, Kp_settle=5 → P_term=75 → output_x=75
//   75 * 2.0 = 150µs → servo at 1650µs (slightly right of center)
//
// TUNE: if servo barely moves, lower this value.
//       if servo immediately slams to endpoint, raise this value.
static constexpr float PID_TO_US_SCALE = 2.0f;

// ── PID Tuning Parameters — FLICK state ──────────────────────────────────────
// Active when Euclidean distance to target > FLICK_THRESHOLD_PX.
// Uses PD only (no integral) to achieve fast ballistic slew to target.
//
// Kp_flick: proportional gain. Raise for faster snap; lower if servo overshoots badly.
//           Start at 2.0, double until reaching target in ~100ms, then back off 20%.
static constexpr float Kp_flick = 8.0f;

// Kd_flick: derivative gain. Raise to dampen oscillation at end of flick.
//           If servo chatters or bounces past target, increase by 0.5 until stable.
static constexpr float Kd_flick = 4.0f;

// ── PID Tuning Parameters — SETTLE state ─────────────────────────────────────
// Active when Euclidean distance to target < FLICK_THRESHOLD_PX.
// Full PID for precision lock; integral eliminates steady-state offset.
//
// Kp_settle: proportional gain. Lower than Kp_flick for stability at small error.
//            Start at 2.0; raise until position holds tightly without oscillation.
static constexpr float Kp_settle = 5.0f;

// Ki_settle: integral gain. Eliminates steady-state offset (servo resting slightly off center).
//            KEEP SMALL. Increase in steps of 0.05. Watch for oscillation (integral windup).
//            INTEGRAL_CLAMP provides a hard cap; Ki works within that bound.
static constexpr float Ki_settle = 0.15f;

// Kd_settle: derivative gain for settle state. Dampen oscillation during fine settling.
//            If the servo oscillates in place, increase by 0.5.
static constexpr float Kd_settle = 3.0f;

// Hard clamp on integral accumulator (in pixel·seconds).
// Prevents integral windup when target is absent for a long time.
// At Ki_settle=0.15: max integral contribution = 200 * 0.15 = 30 pixel-units.
static constexpr float INTEGRAL_CLAMP = 200.0f;

// ── State Machine Thresholds ──────────────────────────────────────────────────
// Distance (pixels) above which FLICK state is used instead of SETTLE.
// Increase if the settle state isn't reached; decrease if flick overshoots.
static constexpr float FLICK_THRESHOLD_PX = 30.0f;

// Distance (pixels) below which the solenoid trigger fires.
// Decrease for more accurate clicks (requires mechanical rig to be precise enough).
// Increase if shots are being missed because the servo hasn't fully settled.
static constexpr float FIRE_THRESHOLD_PX  = 5.0f;

// Error delta (pixels) that indicates a new target has spawned vs. same target moved.
// If the error vector jumps by more than this between consecutive frames, integrals
// are reset to avoid cross-contamination from the previous target's history.
// Tune: lower if target transitions aren't being detected; raise if false resets occur.
static constexpr float TARGET_SWITCH_PX   = 50.0f;

// ── Trigger Parameters ────────────────────────────────────────────────────────
// Solenoid/TENS dwell time: uniform random in [TRIGGER_MIN_MS, TRIGGER_MIN_MS + TRIGGER_RANGE_MS].
// Randomization avoids a fixed-period signature; adjust to match your solenoid's actuation speed.
static constexpr int TRIGGER_MIN_MS   = 30;
static constexpr int TRIGGER_RANGE_MS = 30;   // → total range [30, 60] ms

// ── Network ───────────────────────────────────────────────────────────────────
static constexpr int   UDP_PORT       = 5005;
static constexpr int   PACKET_SIZE    = 16;    // must match vision_node.py struct '!dff'

// Staleness threshold: discard packets older than this many seconds.
// NOTE: Jetson and Pi monotonic clocks are independent. Sync with chrony+PTP for
// accurate staleness. Alternatively, replace with sequence-number gap detection.
static constexpr double PACKET_STALE_S = 0.050;  // 50ms

// ── Control Loop ──────────────────────────────────────────────────────────────
// Target loop period in nanoseconds. 1,000,000 ns = 1ms = 1kHz.
// Implemented as a busy-spin (not nanosleep) to achieve <100µs jitter on Pi.
static constexpr long LOOP_PERIOD_NS = 1000000L;

// ─────────────────────────────────────────────────────────────────────────────
// Data structures
// ─────────────────────────────────────────────────────────────────────────────

// Binary layout of incoming UDP packet from vision_node.py struct.pack('!dff').
// All fields are big-endian; must convert to host order after recv().
#pragma pack(push, 1)
struct VisionPacket {
    double timestamp_s;   // bytes  0-7:  time.monotonic() on Jetson (big-endian double)
    float  cx;            // bytes  8-11: target centroid X (big-endian float)
    float  tx;            // bytes 12-15: crosshair X (big-endian float, static reference)
};
#pragma pack(pop)

enum class ControlState : uint8_t { FLICK, SETTLE };

struct PIDState {
    float integral_x    = 0.0f;
    float prev_error_x  = 0.0f;
    ControlState state  = ControlState::FLICK;
};

struct ControlOutput {
    float servo_x_us;   // servo X pulse width in µs [500, 2500]
    bool  fire;         // true = trigger solenoid
};

// ─────────────────────────────────────────────────────────────────────────────
// Signal handling
// ─────────────────────────────────────────────────────────────────────────────
static volatile sig_atomic_t g_running = 1;

static void handle_signal(int /*sig*/) {
    g_running = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Timing utilities
// ─────────────────────────────────────────────────────────────────────────────

// Returns CLOCK_MONOTONIC time as seconds (double).
// This is a vDSO call on Linux — effectively O(1), no syscall overhead.
static inline double mono_seconds() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

// Returns CLOCK_MONOTONIC as a raw timespec for loop timing.
static inline struct timespec mono_ts() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts;
}

// Busy-spin until the wall clock reaches target_ts.
// Preferred over nanosleep() for sub-millisecond sleeps: nanosleep has ~100µs
// kernel scheduling jitter; spinning on clock_gettime achieves <5µs jitter.
static void spin_until(const struct timespec& target_ts) {
    while (true) {
        struct timespec now = mono_ts();
        if (now.tv_sec > target_ts.tv_sec ||
            (now.tv_sec == target_ts.tv_sec && now.tv_nsec >= target_ts.tv_nsec)) {
            return;
        }
    }
}

// Add nanoseconds to a timespec, normalizing carry into tv_sec.
static struct timespec ts_add_ns(struct timespec ts, long ns) {
    ts.tv_nsec += ns;
    if (ts.tv_nsec >= 1000000000L) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000L;
    }
    return ts;
}

// ─────────────────────────────────────────────────────────────────────────────
// Packet deserialization
// ─────────────────────────────────────────────────────────────────────────────

// Convert a raw 24-byte big-endian buffer into host-order VisionPacket fields.
// Returns false if length != PACKET_SIZE.
// All operations are stack-allocated — zero heap usage.
static bool deserialize_packet(const uint8_t* buf, ssize_t len, VisionPacket& out) {
    if (len != PACKET_SIZE) return false;

    // Double (8 bytes): memcpy into uint64_t, byte-swap, memcpy back.
    {
        uint64_t raw;
        std::memcpy(&raw, buf, 8);
        raw = be64toh(raw);
        std::memcpy(&out.timestamp_s, &raw, 8);
    }

    // Float (4 bytes each): same pattern with uint32_t.
    auto read_f32 = [](const uint8_t* p) -> float {
        uint32_t raw;
        std::memcpy(&raw, p, 4);
        raw = ntohl(raw);
        float f;
        std::memcpy(&f, &raw, 4);
        return f;
    };

    out.cx = read_f32(buf +  8);
    out.tx = read_f32(buf + 12);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// PID state machine
// ─────────────────────────────────────────────────────────────────────────────

/*
 * compute_pid — core X-axis control law.
 *
 * Parameters:
 *   pid     — mutable state (integral, previous error, current state enum)
 *   error_x — signed pixel offset: target_x - crosshair_x (positive = target right)
 *   dt_s    — elapsed time since last call, in seconds (from CLOCK_MONOTONIC)
 *
 * Returns ControlOutput with servo pulse width and fire flag.
 *
 * State machine summary:
 *
 *   ┌─────────────────────────────────────────────────────────────┐
 *   │ Target Switch Detection                                      │
 *   │ If |Δerror_x| > 50px:                                       │
 *   │   → Reset integral to 0                                      │
 *   │   → Set prev_error = current (prevents derivative kick)      │
 *   │   → Force FLICK state                                        │
 *   └──────────────────────┬──────────────────────────────────────┘
 *                          │
 *        |err| > 30px ─────┴───── |err| < 30px
 *               │                      │
 *           FLICK state           SETTLE state
 *        (PD control only)       (full PID control)
 *        High Kp, High Kd        Lower Kp, small Ki, Kd
 *        No integral             Integral with clamp
 *               │                      │
 *               └──────────┬───────────┘
 *                          │
 *            |err| < 5px → fire = true
 */
static ControlOutput compute_pid(PIDState& pid,
                                  float error_x,
                                  float dt_s) {
    float dist = std::abs(error_x);

    // ── Target Switch Detection ───────────────────────────────────────────────
    float delta_x = std::abs(error_x - pid.prev_error_x);
    if (delta_x > TARGET_SWITCH_PX) {
        pid.integral_x   = 0.0f;
        pid.prev_error_x = error_x;
        pid.state = ControlState::FLICK;
    }

    // ── State Transition ──────────────────────────────────────────────────────
    if (dist > FLICK_THRESHOLD_PX) {
        pid.state = ControlState::FLICK;
    } else {
        pid.state = ControlState::SETTLE;
    }

    float output_x;
    float safe_dt = (dt_s > 1e-6f) ? dt_s : 1e-6f;

    if (pid.state == ControlState::FLICK) {
        float deriv_x = (error_x - pid.prev_error_x) / safe_dt;
        output_x = Kp_flick * error_x + Kd_flick * deriv_x;
    } else {
        pid.integral_x = std::clamp(pid.integral_x + error_x * dt_s,
                                    -INTEGRAL_CLAMP, INTEGRAL_CLAMP);
        float deriv_x = (error_x - pid.prev_error_x) / safe_dt;
        output_x = Kp_settle * error_x
                 + Ki_settle * pid.integral_x
                 + Kd_settle * deriv_x;
    }

    pid.prev_error_x = error_x;

    // ── Map PID Output to Servo Pulse Width ───────────────────────────────────
    float servo_x_us = std::clamp(SERVO_CENTER_US + output_x * PID_TO_US_SCALE,
                                  SERVO_MIN_US, SERVO_MAX_US);

    return { servo_x_us, dist < FIRE_THRESHOLD_PX };
}

// ─────────────────────────────────────────────────────────────────────────────
// Solenoid trigger
// ─────────────────────────────────────────────────────────────────────────────

// Atomic flag: prevents concurrent trigger threads from piling up.
// trigger_active == true means a fire thread is currently sleeping.
static std::atomic<bool> trigger_active{false};

/*
 * maybe_fire — asynchronously pulse the solenoid if not already firing.
 *
 * Spawns a detached thread that:
 *   1. Sets GPIO high (energizes solenoid/TENS)
 *   2. Waits a random duration [TRIGGER_MIN_MS, TRIGGER_MIN_MS+TRIGGER_RANGE_MS]
 *   3. Sets GPIO low (releases solenoid)
 *   4. Clears the atomic flag
 *
 * The detached thread does NOT block the 1kHz control loop.
 * The atomic exchange ensures only one fire is in flight at a time.
 */
static void maybe_fire() {
    // exchange returns the old value. If it was already true, another thread is running.
    if (trigger_active.exchange(true)) return;

    std::thread([]() {
        gpioWrite(PIN_TRIGGER, 1);
        // Randomized dwell: uniform in [TRIGGER_MIN_MS, TRIGGER_MIN_MS + TRIGGER_RANGE_MS].
        // gpioDelay takes microseconds; multiply ms by 1000.
        unsigned dwell_us = static_cast<unsigned>(
            (TRIGGER_MIN_MS + (std::rand() % (TRIGGER_RANGE_MS + 1))) * 1000
        );
        gpioDelay(dwell_us);
        gpioWrite(PIN_TRIGGER, 0);
        trigger_active.store(false);   // allow next fire
    }).detach();
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    // ── Signal handling ───────────────────────────────────────────────────────
    std::signal(SIGINT,  handle_signal);
    std::signal(SIGTERM, handle_signal);

    // ── pigpio init ───────────────────────────────────────────────────────────
    if (gpioInitialise() < 0) {
        std::fprintf(stderr, "[ERROR] gpioInitialise() failed. Run as root?\n");
        return 1;
    }

    gpioSetMode(PIN_SERVO_X, PI_OUTPUT);
    gpioSetMode(PIN_TRIGGER, PI_OUTPUT);

    // Safe initial state: servo centered, trigger off.
    gpioServo(PIN_SERVO_X, static_cast<unsigned>(SERVO_CENTER_US));
    gpioWrite(PIN_TRIGGER, 0);

    std::srand(static_cast<unsigned>(time(nullptr)));

    // ── UDP socket ────────────────────────────────────────────────────────────
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::perror("[ERROR] socket()");
        gpioTerminate();
        return 1;
    }

    // Non-blocking: recv returns EAGAIN immediately if no packet is available.
    // This keeps the 1kHz loop running even when the vision node is silent.
    if (fcntl(sock, F_SETFL, O_NONBLOCK) < 0) {
        std::perror("[ERROR] fcntl(O_NONBLOCK)");
        gpioTerminate();
        return 1;
    }

    // Optional: SO_RCVBUF — default is usually 212992 bytes on Linux, sufficient
    // for buffering a few 16-byte packets. Not tuned further here.

    struct sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(static_cast<uint16_t>(UDP_PORT));
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("[ERROR] bind()");
        gpioTerminate();
        return 1;
    }

    std::printf("[INFO] Control node listening on UDP port %d\n", UDP_PORT);
    std::printf("[INFO] Servo X: GPIO%d | Trigger: GPIO%d\n",
                PIN_SERVO_X, PIN_TRIGGER);
    std::printf("[INFO] PID Flick  — Kp=%.2f Kd=%.2f\n", Kp_flick, Kd_flick);
    std::printf("[INFO] PID Settle — Kp=%.2f Ki=%.3f Kd=%.2f\n",
                Kp_settle, Ki_settle, Kd_settle);

    // ── State ─────────────────────────────────────────────────────────────────
    PIDState pid{};
    uint8_t  recv_buf[PACKET_SIZE];

    // Last known target X position (used to hold servo position between packets).
    float    last_tx = 0.0f;   // target centroid X
    float    last_cx = 0.0f;   // crosshair reference X
    bool     have_target = false;

    double   prev_time = mono_seconds();

    // ── 1kHz control loop ─────────────────────────────────────────────────────
    while (g_running) {
        struct timespec loop_start = mono_ts();

        double now = mono_seconds();
        float dt = static_cast<float>(now - prev_time);
        prev_time = now;

        // ── Receive new packet (non-blocking) ─────────────────────────────────
        ssize_t n = recv(sock, recv_buf, sizeof(recv_buf), 0);
        if (n == PACKET_SIZE) {
            VisionPacket pkt;
            if (deserialize_packet(recv_buf, n, pkt)) {
                // Staleness check: discard packets older than threshold.
                // See note in constants section about clock sync requirements.
                double age = now - pkt.timestamp_s;
                if (age < PACKET_STALE_S && age > -1.0) {   // -1.0 allows minor clock skew
                    last_tx = pkt.cx;
                    last_cx = pkt.tx;
                    have_target = true;
                }
            }
        }
        // errno == EAGAIN/EWOULDBLOCK is normal (no packet this iteration) — ignore.

        // ── Control law ───────────────────────────────────────────────────────
        if (have_target) {
            // error: positive X = target is to the right of crosshair
            float error_x = last_tx - last_cx;

            ControlOutput out = compute_pid(pid, error_x, dt);

            // gpioServo truncates to int µs internally; cast is safe within [500,2500].
            gpioServo(PIN_SERVO_X, static_cast<unsigned>(out.servo_x_us));

            if (out.fire) {
                maybe_fire();
            }
        }

        // ── Busy-spin to maintain 1kHz loop rate ──────────────────────────────
        // Compute the absolute target wakeup time and spin until reached.
        // This gives <5µs jitter vs ~100µs for nanosleep on a loaded Pi kernel.
        struct timespec wake = ts_add_ns(loop_start, LOOP_PERIOD_NS);
        spin_until(wake);
    }

    // ── Cleanup ───────────────────────────────────────────────────────────────
    std::printf("\n[INFO] Shutting down...\n");

    // Safe stop: center servo, ensure trigger is off.
    gpioServo(PIN_SERVO_X, static_cast<unsigned>(SERVO_CENTER_US));
    gpioWrite(PIN_TRIGGER, 0);

    // Brief delay to allow any in-flight trigger thread to complete.
    gpioDelay(100000);   // 100ms

    close(sock);
    gpioTerminate();
    std::printf("[INFO] Control node stopped.\n");
    return 0;
}
