/*
 * control_node.cpp — Raspberry Pi 4/5 Control Node
 *
 * Receives 16-byte UDP packets from the Jetson vision node, runs a state-driven
 * PID controller for X-axis, and drives a Feetech ST3215 servo via SCS/STS
 * serial protocol + solenoid trigger via pigpio.
 * Y-axis is user-controlled.
 *
 * Servo control: Feetech SCS/STS half-duplex UART at 1 Mbps.
 *   Position write packet (9 bytes):
 *     0xFF 0xFF ID LEN INST ADDR POS_L POS_H CSUM
 *   Torque enable packet (8 bytes):
 *     0xFF 0xFF ID LEN INST ADDR 0x01 CSUM
 *   Protocol ported from NeuromuscularAimAssist FPGA implementation.
 *
 * Configuration: all tunable parameters are loaded from config.json at startup.
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
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <csignal>
#include <algorithm>
#include <atomic>
#include <thread>
#include <string>
#include <fstream>
#include <sstream>
#include <map>

// ── Feetech SCS/STS protocol constants (fixed, not configurable) ────────────
static constexpr uint8_t SCS_HEADER     = 0xFF;
static constexpr uint8_t SCS_INST_WRITE = 0x03;
static constexpr uint8_t SCS_ADDR_TORQUE = 0x28;
static constexpr uint8_t SCS_ADDR_GOAL   = 0x2A;
static constexpr uint8_t SCS_ADDR_SPEED  = 0x2E;
static constexpr int     PACKET_SIZE     = 20;

// ─────────────────────────────────────────────────────────────────────────────
// Configuration — loaded from config.json at startup
// ─────────────────────────────────────────────────────────────────────────────

static constexpr const char* CONFIG_PATH = "config.json";

struct Config {
    // Solenoid
    bool solenoid_enabled = false;  // set true to fire solenoid from the Pi

    // GPIO
    int pin_trigger = 17;

    // Servo serial
    std::string servo_serial_port = "/dev/ttyS0";
    int servo_id = 1;

    // Servo position
    int servo_center = 2048;
    int servo_min    = 1536;
    int servo_max    = 2560;
    int servo_range  = 512;

    // PID → position scaling
    float pid_to_pos_scale = 2.0f;

    // Position smoothing
    bool smooth_enabled = true;
    int  smooth_divisor = 4;

    // Torque refresh
    int torque_refresh_every = 512;

    // PID — FLICK state
    float kp_flick = 4.0f;
    float kd_flick = 6.0f;

    // PID — SETTLE state
    float kp_settle = 2.5f;
    float ki_settle = 0.10f;
    float kd_settle = 4.0f;

    // Integral clamp
    float integral_clamp = 150.0f;

    // Derivative low-pass filter (0.0 = use raw derivative, 1.0 = ignore new samples)
    float deriv_filter_alpha = 0.85f;

    // State machine thresholds
    float flick_threshold_px  = 30.0f;
    float activation_range_px = 1000.0f;
    float fire_threshold_px   = 5.0f;
    float target_switch_px    = 50.0f;

    // Dynamic blob-width scaling
    float fire_threshold_width_scale  = 0.15f;   // fire_thresh = base + blob_w * scale
    float flick_threshold_width_scale = 0.1f;    // flick_thresh = base + blob_w * scale
    float settle_gain_reference_width = 60.0f;   // blob_w at which settle gains = 100%
    float settle_gain_min_scale       = 0.3f;    // floor for gain scaling (30%)

    // Overshoot damping — reduces gains when error crosses zero
    bool  overshoot_damping_enabled = true;
    float overshoot_gain_multiplier = 0.5f;   // multiply Kp by this on zero-cross
    int   overshoot_damping_frames  = 10;     // how many 1ms frames to stay damped

    // Backlash compensation
    bool  backlash_enabled      = true;
    float backlash_gain         = 1.0f;   // multiplier on dead zone (1.0 = exact physical model)
    int   backlash_ramp_frames  = 5;      // ramp in compensation over N frames to prevent step input

    // Trigger
    int trigger_min_ms   = 30;
    int trigger_range_ms = 30;
    int trigger_cooldown_ms = 500;  // minimum time between successive fires

    // Network
    int udp_port = 5005;

    // Control loop
    long loop_period_ns = 1000000L;

    // TENS
    bool  tens_enabled        = false;
    int   tens_max_intensity  = 80;
    int   tens_min_intensity  = 20;
    float tens_activation_px  = 150.0f;
    float tens_hysteresis_px  = 15.0f;
    int   tens_cooldown_ms    = 200;
    int   tens_spi_channel    = 0;
    int   tens_spi_speed      = 1000000;
};

// Global config instance — populated once at startup, then read-only.
static Config cfg;

// ─────────────────────────────────────────────────────────────────────────────
// Minimal JSON config loader
// Handles flat JSON objects with string, number, and boolean values.
// No external dependencies.
// ─────────────────────────────────────────────────────────────────────────────

static std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

static bool load_config(const char* path, Config& c) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::fprintf(stderr, "[WARN] Cannot open %s — using defaults\n", path);
        return false;
    }

    std::string content((std::istreambuf_iterator<char>(f)),
                         std::istreambuf_iterator<char>());

    // Simple flat JSON parser: extract "key": value pairs
    std::map<std::string, std::string> kv;

    size_t pos = 0;
    while (pos < content.size()) {
        // Find next key (quoted string)
        size_t q1 = content.find('"', pos);
        if (q1 == std::string::npos) break;
        size_t q2 = content.find('"', q1 + 1);
        if (q2 == std::string::npos) break;

        std::string key = content.substr(q1 + 1, q2 - q1 - 1);

        // Find colon after key
        size_t colon = content.find(':', q2 + 1);
        if (colon == std::string::npos) break;

        // Find value: could be string (quoted), number, or bool
        size_t vstart = content.find_first_not_of(" \t\r\n", colon + 1);
        if (vstart == std::string::npos) break;

        std::string value;
        if (content[vstart] == '"') {
            // String value
            size_t vend = content.find('"', vstart + 1);
            if (vend == std::string::npos) break;
            value = content.substr(vstart + 1, vend - vstart - 1);
            pos = vend + 1;
        } else {
            // Number or boolean — read until comma, closing brace, or newline
            size_t vend = content.find_first_of(",}\r\n", vstart);
            if (vend == std::string::npos) vend = content.size();
            value = trim(content.substr(vstart, vend - vstart));
            pos = vend + 1;
        }

        kv[key] = value;
    }

    // Map parsed values into Config struct
    auto get_int = [&](const char* k, int& dst) {
        auto it = kv.find(k);
        if (it != kv.end()) dst = std::atoi(it->second.c_str());
    };
    auto get_long = [&](const char* k, long& dst) {
        auto it = kv.find(k);
        if (it != kv.end()) dst = std::atol(it->second.c_str());
    };
    auto get_float = [&](const char* k, float& dst) {
        auto it = kv.find(k);
        if (it != kv.end()) dst = std::strtof(it->second.c_str(), nullptr);
    };
    auto get_bool = [&](const char* k, bool& dst) {
        auto it = kv.find(k);
        if (it != kv.end()) dst = (it->second == "true");
    };
    auto get_str = [&](const char* k, std::string& dst) {
        auto it = kv.find(k);
        if (it != kv.end()) dst = it->second;
    };

    get_bool ("solenoid_enabled",    c.solenoid_enabled);
    get_int  ("pin_trigger",         c.pin_trigger);
    get_str  ("servo_serial_port",   c.servo_serial_port);
    get_int  ("servo_id",            c.servo_id);
    get_int  ("servo_center",        c.servo_center);
    get_int  ("servo_min",           c.servo_min);
    get_int  ("servo_max",           c.servo_max);
    get_int  ("servo_range",         c.servo_range);
    get_float("pid_to_pos_scale",    c.pid_to_pos_scale);
    get_bool ("smooth_enabled",      c.smooth_enabled);
    get_int  ("smooth_divisor",      c.smooth_divisor);
    get_int  ("torque_refresh_every",c.torque_refresh_every);
    get_float("kp_flick",           c.kp_flick);
    get_float("kd_flick",           c.kd_flick);
    get_float("kp_settle",          c.kp_settle);
    get_float("ki_settle",          c.ki_settle);
    get_float("kd_settle",          c.kd_settle);
    get_float("integral_clamp",     c.integral_clamp);
    get_float("deriv_filter_alpha", c.deriv_filter_alpha);
    get_float("flick_threshold_px", c.flick_threshold_px);
    get_float("activation_range_px",c.activation_range_px);
    get_float("fire_threshold_px",  c.fire_threshold_px);
    get_float("target_switch_px",   c.target_switch_px);
    get_float("fire_threshold_width_scale",  c.fire_threshold_width_scale);
    get_float("flick_threshold_width_scale", c.flick_threshold_width_scale);
    get_float("settle_gain_reference_width", c.settle_gain_reference_width);
    get_float("settle_gain_min_scale",       c.settle_gain_min_scale);
    get_bool ("overshoot_damping_enabled", c.overshoot_damping_enabled);
    get_float("overshoot_gain_multiplier",c.overshoot_gain_multiplier);
    get_int  ("overshoot_damping_frames", c.overshoot_damping_frames);
    get_bool ("backlash_enabled",       c.backlash_enabled);
    get_float("backlash_gain",          c.backlash_gain);
    get_int  ("backlash_ramp_frames",   c.backlash_ramp_frames);
    get_int  ("trigger_min_ms",     c.trigger_min_ms);
    get_int  ("trigger_range_ms",   c.trigger_range_ms);
    get_int  ("trigger_cooldown_ms", c.trigger_cooldown_ms);
    get_int  ("udp_port",           c.udp_port);
    get_long ("loop_period_ns",     c.loop_period_ns);

    get_bool ("tens_enabled",        c.tens_enabled);
    get_int  ("tens_max_intensity",  c.tens_max_intensity);
    get_int  ("tens_min_intensity",  c.tens_min_intensity);
    get_float("tens_activation_px",  c.tens_activation_px);
    get_float("tens_hysteresis_px",  c.tens_hysteresis_px);
    get_int  ("tens_cooldown_ms",    c.tens_cooldown_ms);
    get_int  ("tens_spi_channel",    c.tens_spi_channel);
    get_int  ("tens_spi_speed",      c.tens_spi_speed);

    // Safety: hard-cap TENS intensity at 100 regardless of config
    c.tens_max_intensity = std::min(c.tens_max_intensity, 100);
    c.tens_min_intensity = std::clamp(c.tens_min_intensity, 0, c.tens_max_intensity);

    // Safety: hard-cap servo rotation to ±45° from center (512 steps on 4096-step/360° servo)
    static constexpr int MAX_SERVO_RANGE = 512;  // 45° = 45 * (4096/360) ≈ 512
    c.servo_range = std::min(c.servo_range, MAX_SERVO_RANGE);
    c.servo_min   = std::max(c.servo_min, c.servo_center - MAX_SERVO_RANGE);
    c.servo_max   = std::min(c.servo_max, c.servo_center + MAX_SERVO_RANGE);

    std::printf("[INFO] Config loaded from %s (%zu keys)\n", path, kv.size());
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Data structures
// ─────────────────────────────────────────────────────────────────────────────

#pragma pack(push, 1)
struct VisionPacket {
    double timestamp_s;
    float  tx;
    float  cx;
    float  blob_w;
};
#pragma pack(pop)

enum class ControlState : uint8_t { FLICK, SETTLE };
enum class SystemState  : uint8_t { SERVO_ACTIVE, TENS_ACTIVE, IDLE };

struct PIDState {
    float integral_x    = 0.0f;
    float prev_error_x  = 0.0f;
    float filtered_deriv = 0.0f;  // low-pass filtered derivative to tame packet spikes
    ControlState state  = ControlState::FLICK;
    int   damping_countdown = 0;  // frames remaining with reduced gains after overshoot
    // Backlash tracking
    int   last_push_sign  = 0;      // +1 = servo pushing right, -1 = left, 0 = neutral
    float backlash_offset = 0.0f;   // current compensation being applied
    int   backlash_ramp   = 0;      // frames since last reversal
};

struct ServoState {
    int   current_pos       = 0;   // initialized from cfg.servo_center in main()
    int   torque_refresh_cnt = 0;
};

struct TensState {
    int    active_channel   = -1;   // 0 or 1, -1 = none
    double last_update_time = 0.0;
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

static inline double mono_seconds() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

static inline struct timespec mono_ts() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts;
}

static void spin_until(const struct timespec& target_ts) {
    while (g_running) {
        struct timespec now = mono_ts();
        if (now.tv_sec > target_ts.tv_sec ||
            (now.tv_sec == target_ts.tv_sec && now.tv_nsec >= target_ts.tv_nsec)) {
            return;
        }
    }
}

static struct timespec ts_add_ns(struct timespec ts, long ns) {
    ts.tv_nsec += ns;
    if (ts.tv_nsec >= 1000000000L) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000L;
    }
    return ts;
}

// ─────────────────────────────────────────────────────────────────────────────
// Feetech SCS/STS serial servo driver
// ─────────────────────────────────────────────────────────────────────────────

static int feetech_open(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        std::perror("[ERROR] feetech_open: open()");
        return -1;
    }

    struct termios tio;
    std::memset(&tio, 0, sizeof(tio));

    cfsetispeed(&tio, B1000000);
    cfsetospeed(&tio, B1000000);

    tio.c_cflag |= CS8 | CLOCAL | CREAD;
    tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tio.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNCR);
    tio.c_oflag &= ~OPOST;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        std::perror("[ERROR] feetech_open: tcsetattr()");
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

static void feetech_send_torque_enable(int fd, uint8_t id) {
    uint8_t len  = 0x04;
    uint8_t data = 0x01;
    uint8_t csum = ~(id + len + SCS_INST_WRITE + SCS_ADDR_TORQUE + data) & 0xFF;

    uint8_t pkt[8] = {
        SCS_HEADER, SCS_HEADER,
        id, len, SCS_INST_WRITE, SCS_ADDR_TORQUE, data, csum
    };

    write(fd, pkt, sizeof(pkt));
    tcdrain(fd);
}

static void feetech_send_position(int fd, uint8_t id, int position) {
    position = std::clamp(position, 0, 4095);

    uint8_t pos_l = static_cast<uint8_t>(position & 0xFF);
    uint8_t pos_h = static_cast<uint8_t>((position >> 8) & 0xFF);
    uint8_t len   = 0x05;
    uint8_t csum  = ~(id + len + SCS_INST_WRITE + SCS_ADDR_GOAL + pos_l + pos_h) & 0xFF;

    uint8_t pkt[9] = {
        SCS_HEADER, SCS_HEADER,
        id, len, SCS_INST_WRITE, SCS_ADDR_GOAL, pos_l, pos_h, csum
    };

    write(fd, pkt, sizeof(pkt));
}

static void feetech_send_max_speed(int fd, uint8_t id) {
    // Write 0x0000 to speed register (0x2E) — 0 means no speed limit (maximum)
    uint8_t spd_l = 0x00;
    uint8_t spd_h = 0x00;
    uint8_t len   = 0x05;
    uint8_t csum  = ~(id + len + SCS_INST_WRITE + SCS_ADDR_SPEED + spd_l + spd_h) & 0xFF;

    uint8_t pkt[9] = {
        SCS_HEADER, SCS_HEADER,
        id, len, SCS_INST_WRITE, SCS_ADDR_SPEED, spd_l, spd_h, csum
    };

    write(fd, pkt, sizeof(pkt));
    tcdrain(fd);
}

// ─────────────────────────────────────────────────────────────────────────────
// MCP4131 digital potentiometer SPI driver (TENS control)
//
// Two MCP4131 chips on SPI0: CE0 (channel 0) and CE1 (channel 1).
// Write command: 2 bytes — [0x00 (wiper 0, write), value (0-127)]
// ─────────────────────────────────────────────────────────────────────────────

static int tens_spi_handle[2] = {-1, -1};

static void tens_set_wiper(int handle, uint8_t value, bool verbose = false) {
    if (handle < 0) {
        if (verbose) std::printf("[TENS] set_wiper skipped: handle=%d\n", handle);
        return;
    }
    value = std::min(value, static_cast<uint8_t>(127));
    char buf[2] = { 0x00, static_cast<char>(value) };
    int rc = spiWrite(static_cast<unsigned>(handle), buf, 2);
    if (verbose) {
        std::printf("[TENS] set_wiper handle=%d value=%d rc=%d\n", handle, value, rc);
        std::fflush(stdout);
    }
}

// tens_off: just zero the wiper. No TCON manipulation — TCON shutdown/wakeup
// was added in c4c74b2 to fix "always on" but actually broke TENS entirely
// by disconnecting terminals and failing to reliably reconnect them.
static void tens_off(int ch) {
    if (ch >= 0 && ch < 2) {
        tens_set_wiper(tens_spi_handle[ch], 0);
    }
}

static void tens_all_off() {
    tens_off(0);
    tens_off(1);
}

// ─────────────────────────────────────────────────────────────────────────────
// Packet deserialization
// ─────────────────────────────────────────────────────────────────────────────

static bool deserialize_packet(const uint8_t* buf, ssize_t len, VisionPacket& out) {
    if (len != PACKET_SIZE) return false;

    {
        uint64_t raw;
        std::memcpy(&raw, buf, 8);
        raw = be64toh(raw);
        std::memcpy(&out.timestamp_s, &raw, 8);
    }

    auto read_f32 = [](const uint8_t* p) -> float {
        uint32_t raw;
        std::memcpy(&raw, p, 4);
        raw = ntohl(raw);
        float f;
        std::memcpy(&f, &raw, 4);
        return f;
    };

    out.tx     = read_f32(buf +  8);
    out.cx     = read_f32(buf + 12);
    out.blob_w = read_f32(buf + 16);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// PID state machine
// ─────────────────────────────────────────────────────────────────────────────

struct ControlOutput {
    int          servo_target_pos;
    bool         fire;
    float        dist;
    float        servo_offset;
    ControlState mode;
    bool         backlash_applied;
    float        backlash_amount;
    bool         damped;
};

static ControlOutput compute_pid(PIDState& pid,
                                  float error_x,
                                  float dt_s,
                                  int servo_current_pos,
                                  float eff_fire_threshold,
                                  float eff_flick_threshold,
                                  float settle_gain_scale,
                                  bool verbose = false) {
    float dist = std::abs(error_x);

    if (verbose) {
        std::printf("  [PID:input] err=%.2f dist=%.2f prev_err=%.2f dt=%.6fs integral=%.2f damp_cd=%d\n",
                    error_x, dist, pid.prev_error_x, dt_s, pid.integral_x, pid.damping_countdown);
    }

    if (dist > cfg.activation_range_px) {
        if (verbose) {
            std::printf("  [PID:skip] dist %.1f > activation_range %.1f → center, reset\n",
                        dist, cfg.activation_range_px);
        }
        pid.integral_x   = 0.0f;
        pid.prev_error_x = error_x;
        pid.last_push_sign = 0;
        pid.backlash_offset = 0.0f;
        pid.backlash_ramp = 0;
        return { cfg.servo_center, false, dist, 0.0f, ControlState::FLICK, false, 0.0f, false };
    }

    // ── Target switch detection ──
    float delta_x = std::abs(error_x - pid.prev_error_x);
    bool target_switched = (delta_x > cfg.target_switch_px);
    if (target_switched) {
        if (verbose) {
            std::printf("  [PID:target_switch] delta=%.1f > threshold=%.1f → reset integral, prev_err, deriv filter\n",
                        delta_x, cfg.target_switch_px);
        }
        pid.integral_x   = 0.0f;
        pid.prev_error_x = error_x;
        pid.filtered_deriv = 0.0f;
        pid.state = ControlState::FLICK;
    }

    // ── Overshoot detection: error sign flipped → we overshot ──
    // Suppress during backlash ramp — the large position change is intentional
    bool in_backlash_ramp = (pid.backlash_ramp > 0 && pid.backlash_ramp <= cfg.backlash_ramp_frames);
    bool sign_flipped = (pid.prev_error_x != 0.0f &&
                         ((error_x > 0.0f) != (pid.prev_error_x > 0.0f)));
    if (cfg.overshoot_damping_enabled && sign_flipped && !in_backlash_ramp) {
        if (verbose) {
            std::printf("  [PID:overshoot] sign flip! err=%.2f prev=%.2f → damping for %d frames, integral zeroed\n",
                        error_x, pid.prev_error_x, cfg.overshoot_damping_frames);
        }
        pid.damping_countdown = cfg.overshoot_damping_frames;
        pid.integral_x = 0.0f;
    }

    bool damped = (pid.damping_countdown > 0);
    if (damped) pid.damping_countdown--;
    float gain_mult = damped ? cfg.overshoot_gain_multiplier : 1.0f;

    // ── Mode selection ──
    ControlState prev_state = pid.state;
    if (dist > eff_flick_threshold) {
        pid.state = ControlState::FLICK;
    } else {
        pid.state = ControlState::SETTLE;
    }
    if (verbose && pid.state != prev_state) {
        std::printf("  [PID:mode_change] %s → %s (dist=%.1f threshold=%.1f)\n",
                    prev_state == ControlState::FLICK ? "FLICK" : "SETTLE",
                    pid.state == ControlState::FLICK ? "FLICK" : "SETTLE",
                    dist, eff_flick_threshold);
    }

    float output_x;
    float safe_dt = (dt_s > 1e-6f) ? dt_s : 1e-6f;
    float P_term, I_term = 0.0f, D_term, deriv_x;

    // Compute raw derivative, then low-pass filter to tame packet-arrival spikes
    float raw_deriv = (error_x - pid.prev_error_x) / safe_dt;
    float alpha = cfg.deriv_filter_alpha;
    pid.filtered_deriv = alpha * pid.filtered_deriv + (1.0f - alpha) * raw_deriv;
    deriv_x = pid.filtered_deriv;

    if (pid.state == ControlState::FLICK) {
        P_term = (cfg.kp_flick * gain_mult) * error_x;
        D_term = cfg.kd_flick * deriv_x;
        output_x = P_term + D_term;
        if (verbose) {
            std::printf("  [PID:FLICK] Kp=%.2f*%.2f=%.2f Kd=%.2f raw_deriv=%.1f filt_deriv=%.1f | P=%.2f D=%.2f → out=%.2f\n",
                        cfg.kp_flick, gain_mult, cfg.kp_flick * gain_mult,
                        cfg.kd_flick, raw_deriv, deriv_x, P_term, D_term, output_x);
        }
    } else {
        float integral_before = pid.integral_x;
        float integral_add = error_x * dt_s;
        pid.integral_x = std::clamp(pid.integral_x + integral_add,
                                    -cfg.integral_clamp, cfg.integral_clamp);
        P_term = (cfg.kp_settle * gain_mult * settle_gain_scale) * error_x;
        I_term = cfg.ki_settle * pid.integral_x;
        D_term = (cfg.kd_settle * settle_gain_scale) * deriv_x;
        output_x = P_term + I_term + D_term;
        if (verbose) {
            bool clamped = (std::abs(pid.integral_x) >= cfg.integral_clamp - 0.01f);
            std::printf("  [PID:SETTLE] Kp=%.2f*%.2f=%.2f Ki=%.3f Kd=%.2f raw_deriv=%.1f filt_deriv=%.1f\n",
                        cfg.kp_settle, gain_mult, cfg.kp_settle * gain_mult,
                        cfg.ki_settle, cfg.kd_settle, raw_deriv, deriv_x);
            std::printf("  [PID:SETTLE]   P=%.2f I=%.2f(integral %.4f+%.4f=%.4f%s) D=%.2f → out=%.2f\n",
                        P_term, I_term, integral_before, integral_add, pid.integral_x,
                        clamped ? " CLAMPED" : "", D_term, output_x);
        }
    }

    if (verbose && damped) {
        std::printf("  [PID:damping] gain_mult=%.2f countdown=%d\n", gain_mult, pid.damping_countdown);
    }

    pid.prev_error_x = error_x;

    float raw_servo_offset = output_x * cfg.pid_to_pos_scale;
    float servo_offset = raw_servo_offset;

    // Backlash compensation: direction-dependent dead zone model
    bool backlash_applied = false;
    float backlash_amount = 0.0f;
    if (cfg.backlash_enabled) {
        int current_offset_from_center = servo_current_pos - cfg.servo_center;
        int desired_sign = (output_x > 0.0f) ? 1 : ((output_x < 0.0f) ? -1 : 0);
        int current_push_sign = (current_offset_from_center > 0) ? 1 :
                                ((current_offset_from_center < 0) ? -1 : 0);

        // Reversal: desired direction differs from where servo currently is
        if (desired_sign != 0 && current_push_sign != 0 && desired_sign != current_push_sign) {
            if (pid.last_push_sign != desired_sign) {
                // New reversal detected — start ramp
                pid.backlash_ramp = 1;
                pid.last_push_sign = desired_sign;
            } else {
                pid.backlash_ramp++;
            }

            float dead_zone = static_cast<float>(std::abs(current_offset_from_center));
            float ramp_factor = std::min(static_cast<float>(pid.backlash_ramp) /
                                         static_cast<float>(cfg.backlash_ramp_frames), 1.0f);
            backlash_amount = static_cast<float>(desired_sign) * dead_zone * cfg.backlash_gain * ramp_factor;
            servo_offset += backlash_amount;
            backlash_applied = true;

            if (verbose) {
                std::printf("  [PID:backlash] servo_pos=%d center=%d offset_from_center=%d dead_zone=%.0f desired_sign=%d ramp=%d/%d backlash=%.1f\n",
                            servo_current_pos, cfg.servo_center, current_offset_from_center,
                            dead_zone, desired_sign, pid.backlash_ramp, cfg.backlash_ramp_frames, backlash_amount);
            }
        } else {
            // No reversal — reset backlash tracking
            if (desired_sign != 0) pid.last_push_sign = desired_sign;
            pid.backlash_offset = 0.0f;
            pid.backlash_ramp = 0;
        }
    }

    float pre_clamp_offset = servo_offset;
    servo_offset = std::clamp(servo_offset,
                              static_cast<float>(-cfg.servo_range),
                              static_cast<float>(cfg.servo_range));
    bool was_clamped = (pre_clamp_offset != servo_offset);

    int raw_target_pos = cfg.servo_center + static_cast<int>(servo_offset);
    int target_pos = std::clamp(raw_target_pos, cfg.servo_min, cfg.servo_max);
    bool pos_clamped = (raw_target_pos != target_pos);

    if (verbose) {
        std::printf("  [PID:output] pid_out=%.2f *scale=%.2f → raw_offset=%.2f",
                    output_x, cfg.pid_to_pos_scale, raw_servo_offset);
        if (backlash_applied) std::printf(" +backlash=%.1f", backlash_amount);
        std::printf(" = %.2f", pre_clamp_offset);
        if (was_clamped) std::printf(" → CLAMPED to %.1f (range=±%d)", servo_offset, cfg.servo_range);
        std::printf("\n");
        std::printf("  [PID:pos] center(%d) + offset(%d) = %d",
                    cfg.servo_center, static_cast<int>(servo_offset), raw_target_pos);
        if (pos_clamped) std::printf(" → CLAMPED to %d [%d,%d]", target_pos, cfg.servo_min, cfg.servo_max);
        std::printf(" | fire=%s (dist=%.1f threshold=%.1f)\n",
                    dist < eff_fire_threshold ? "YES" : "no", dist, eff_fire_threshold);
    }

    return { target_pos, dist < eff_fire_threshold, dist, servo_offset, pid.state, backlash_applied, backlash_amount, damped };
}

static int smooth_servo_position(ServoState& ss, int target_pos, bool verbose = false) {
    if (!cfg.smooth_enabled) {
        int prev = ss.current_pos;
        ss.current_pos = std::clamp(target_pos, cfg.servo_min, cfg.servo_max);
        if (verbose) {
            std::printf("  [SMOOTH:off] target=%d → pos=%d (was %d)\n",
                        target_pos, ss.current_pos, prev);
        }
        return ss.current_pos;
    }

    int prev = ss.current_pos;
    int delta = target_pos - ss.current_pos;
    int step  = delta / cfg.smooth_divisor;

    if (step == 0 && delta != 0) {
        step = (delta > 0) ? 1 : -1;
    }

    ss.current_pos = std::clamp(ss.current_pos + step, cfg.servo_min, cfg.servo_max);

    if (verbose) {
        std::printf("  [SMOOTH] cur=%d target=%d delta=%d step=%d/%d=%d → new_pos=%d\n",
                    prev, target_pos, delta, delta, cfg.smooth_divisor, step, ss.current_pos);
    }
    return ss.current_pos;
}

// ─────────────────────────────────────────────────────────────────────────────
// Solenoid trigger
// ─────────────────────────────────────────────────────────────────────────────

static std::atomic<bool> trigger_active{false};
static std::atomic<uint64_t> last_fire_time_us{0};

static void maybe_fire() {
    // Enforce cooldown between fires to prevent continuous re-energizing
    uint64_t now_us = static_cast<uint64_t>(gpioTick());
    uint64_t cooldown_us = static_cast<uint64_t>(cfg.trigger_cooldown_ms) * 1000;
    uint64_t last = last_fire_time_us.load(std::memory_order_relaxed);
    if (now_us - last < cooldown_us && last != 0) return;

    if (trigger_active.exchange(true)) return;

    last_fire_time_us.store(now_us, std::memory_order_relaxed);

    std::thread([]() {
        unsigned dwell_us = static_cast<unsigned>(
            (cfg.trigger_min_ms + (std::rand() % (cfg.trigger_range_ms + 1))) * 1000
        );
        gpioWrite(cfg.pin_trigger, 1);
        gpioDelay(dwell_us);
        gpioWrite(cfg.pin_trigger, 0);
        trigger_active.store(false, std::memory_order_release);
    }).detach();
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    // ── Load config ──────────────────────────────────────────────────────────
    load_config(CONFIG_PATH, cfg);

    // ── Signal handling ──────────────────────────────────────────────────────
    std::signal(SIGINT,  handle_signal);
    std::signal(SIGTERM, handle_signal);

    // ── pigpio init (solenoid GPIO + TENS SPI) ─────────────────────────────
    bool need_pigpio = cfg.solenoid_enabled || cfg.tens_enabled;
    if (need_pigpio) {
        // Disable pigpio's internal signal handling so our SIGINT handler runs cleanup
        gpioCfgSetInternals(gpioCfgGetInternals() | PI_CFG_NOSIGHANDLER);
        if (gpioInitialise() < 0) {
            std::fprintf(stderr, "[ERROR] gpioInitialise() failed. Run as root?\n");
            return 1;
        }
    }

    if (cfg.solenoid_enabled) {
        gpioSetMode(cfg.pin_trigger, PI_OUTPUT);
        gpioWrite(cfg.pin_trigger, 0);
        std::srand(static_cast<unsigned>(time(nullptr)));
    }

    // ── TENS SPI init ─────────────────────────────────────────────────────────
    if (cfg.tens_enabled) {
        tens_spi_handle[0] = spiOpen(0, static_cast<unsigned>(cfg.tens_spi_speed), 0);
        tens_spi_handle[1] = spiOpen(1, static_cast<unsigned>(cfg.tens_spi_speed), 0);
        if (tens_spi_handle[0] < 0 || tens_spi_handle[1] < 0) {
            std::fprintf(stderr, "[WARN] TENS SPI open failed (h0=%d h1=%d), disabling TENS\n",
                         tens_spi_handle[0], tens_spi_handle[1]);
            if (tens_spi_handle[0] >= 0) spiClose(static_cast<unsigned>(tens_spi_handle[0]));
            if (tens_spi_handle[1] >= 0) spiClose(static_cast<unsigned>(tens_spi_handle[1]));
            tens_spi_handle[0] = tens_spi_handle[1] = -1;
            cfg.tens_enabled = false;
        } else {
            tens_all_off();
            std::printf("[INFO] TENS SPI initialized (CE0 + CE1, %d Hz)\n",
                        cfg.tens_spi_speed);
        }
    }

    // ── Feetech servo serial port ────────────────────────────────────────────
    int servo_fd = feetech_open(cfg.servo_serial_port.c_str());
    if (servo_fd < 0) {
        std::fprintf(stderr, "[ERROR] Failed to open servo serial port %s\n",
                     cfg.servo_serial_port.c_str());
        if (need_pigpio) gpioTerminate();
        return 1;
    }

    uint8_t servo_id = static_cast<uint8_t>(cfg.servo_id);

    feetech_send_torque_enable(servo_fd, servo_id);
    feetech_send_max_speed(servo_fd, servo_id);
    std::printf("[INFO] Servo torque enabled, speed unlimited (ID=0x%02X, port=%s)\n",
                servo_id, cfg.servo_serial_port.c_str());

    feetech_send_position(servo_fd, servo_id, cfg.servo_center);

    // ── UDP socket ───────────────────────────────────────────────────────────
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::perror("[ERROR] socket()");
        close(servo_fd);
        if (need_pigpio) gpioTerminate();
        return 1;
    }

    if (fcntl(sock, F_SETFL, O_NONBLOCK) < 0) {
        std::perror("[ERROR] fcntl(O_NONBLOCK)");
        close(servo_fd);
        if (need_pigpio) gpioTerminate();
        return 1;
    }

    struct sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(static_cast<uint16_t>(cfg.udp_port));
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("[ERROR] bind()");
        close(servo_fd);
        if (need_pigpio) gpioTerminate();
        return 1;
    }

    std::printf("[INFO] Control node listening on UDP port %d\n", cfg.udp_port);
    std::printf("[INFO] Servo: Feetech SCS/STS ID=0x%02X @ %s (1 Mbps)\n",
                servo_id, cfg.servo_serial_port.c_str());
    std::printf("[INFO] Servo position: center=%d range=[%d, %d]\n",
                cfg.servo_center, cfg.servo_min, cfg.servo_max);
    std::printf("[INFO] Trigger: GPIO%d (%s)\n", cfg.pin_trigger,
                cfg.solenoid_enabled ? "enabled" : "disabled — solenoid on Jetson");
    std::printf("[INFO] PID Flick  — Kp=%.2f Kd=%.2f\n", cfg.kp_flick, cfg.kd_flick);
    std::printf("[INFO] PID Settle — Kp=%.2f Ki=%.3f Kd=%.2f\n",
                cfg.kp_settle, cfg.ki_settle, cfg.kd_settle);
    std::printf("[INFO] Derivative filter alpha=%.2f (0=raw, higher=smoother)\n",
                cfg.deriv_filter_alpha);
    std::printf("[INFO] Backlash compensation: %s (gain=%.2f ramp_frames=%d)\n",
                cfg.backlash_enabled ? "on" : "off", cfg.backlash_gain, cfg.backlash_ramp_frames);
    std::printf("[INFO] Overshoot damping: %s (mult=%.2f frames=%d)\n",
                cfg.overshoot_damping_enabled ? "on" : "off",
                cfg.overshoot_gain_multiplier, cfg.overshoot_damping_frames);
    std::printf("[INFO] PID→Pos scale=%.2f  Smoothing=%s (divisor=%d)\n",
                cfg.pid_to_pos_scale,
                cfg.smooth_enabled ? "on" : "off",
                cfg.smooth_divisor);
    std::printf("[INFO] TENS: %s", cfg.tens_enabled ? "enabled" : "disabled");
    if (cfg.tens_enabled) {
        std::printf(" — intensity=[%d, %d], activation=%.0fpx, cooldown=%dms\n",
                    cfg.tens_min_intensity, cfg.tens_max_intensity,
                    cfg.tens_activation_px, cfg.tens_cooldown_ms);
    } else {
        std::printf("\n");
    }

    // ── State ────────────────────────────────────────────────────────────────
    PIDState    pid{};
    ServoState  servo{};
    TensState   tens{};
    servo.current_pos = cfg.servo_center;
    uint8_t     recv_buf[PACKET_SIZE];

    float       last_tx = 0.0f;
    float       last_cx = 0.0f;
    float       last_blob_w = 0.0f;
    bool        have_target = false;

    double      prev_time = mono_seconds();
    double      last_packet_time = prev_time;
    uint64_t    recv_count = 0;
    SystemState sys_state = SystemState::IDLE;
    SystemState prev_sys_state = SystemState::IDLE;

    // ── 1kHz control loop ────────────────────────────────────────────────────
    while (g_running) {
        struct timespec loop_start = mono_ts();

        double now = mono_seconds();
        float dt = static_cast<float>(now - prev_time);
        prev_time = now;

        // ── Receive new packet (non-blocking) ────────────────────────────────
        ssize_t n = recv(sock, recv_buf, sizeof(recv_buf), 0);
        bool new_packet = false;
        if (n == PACKET_SIZE) {
            VisionPacket pkt;
            if (deserialize_packet(recv_buf, n, pkt)) {
                ++recv_count;
                last_packet_time = now;
                float prev_tx = last_tx, prev_cx = last_cx;
                last_tx = pkt.tx;
                last_cx = pkt.cx;
                last_blob_w = pkt.blob_w;
                have_target = true;
                new_packet = true;
                // Log every packet arrival at ~10Hz
                static uint64_t pkt_log_tick = 0;
                if (++pkt_log_tick % 100 == 0) {
                    std::printf("[PKT] #%lu tx=%.1f cx=%.1f blob_w=%.1f err=%.1f (prev_tx=%.1f prev_cx=%.1f delta_err=%.1f) age=%.1fms\n",
                                recv_count, last_tx, last_cx, last_blob_w, last_tx - last_cx,
                                prev_tx, prev_cx, (last_tx - last_cx) - (prev_tx - prev_cx),
                                (now - pkt.timestamp_s) * 1000.0);
                }
            }
        }

        // ── UDP watchdog: force IDLE if no packets for >500ms ────────────────
        if (have_target && (now - last_packet_time) > 0.5) {
            std::printf("[WATCHDOG] No packets for %.0fms → forcing IDLE\n",
                        (now - last_packet_time) * 1000.0);
            have_target = false;
        }

        // ── Determine system state ──────────────────────────────────────────
        prev_sys_state = sys_state;

        if (!have_target) {
            sys_state = SystemState::IDLE;
        } else {
            float error_x = last_tx - last_cx;
            float dist = std::abs(error_x);

            switch (sys_state) {
                case SystemState::SERVO_ACTIVE:
                    if (cfg.tens_enabled &&
                        dist > cfg.tens_activation_px + cfg.tens_hysteresis_px)
                        sys_state = SystemState::TENS_ACTIVE;
                    else if (dist > cfg.activation_range_px)
                        sys_state = SystemState::IDLE;
                    break;
                case SystemState::TENS_ACTIVE:
                    if (dist < cfg.tens_activation_px - cfg.tens_hysteresis_px)
                        sys_state = SystemState::SERVO_ACTIVE;
                    else if (dist > cfg.activation_range_px + cfg.tens_hysteresis_px)
                        sys_state = SystemState::IDLE;
                    break;
                case SystemState::IDLE:
                    if (dist < cfg.tens_activation_px - cfg.tens_hysteresis_px)
                        sys_state = SystemState::SERVO_ACTIVE;
                    else if (cfg.tens_enabled &&
                             dist <= cfg.activation_range_px - cfg.tens_hysteresis_px)
                        sys_state = SystemState::TENS_ACTIVE;
                    else if (!cfg.tens_enabled &&
                             dist <= cfg.activation_range_px)
                        sys_state = SystemState::SERVO_ACTIVE;
                    break;
            }
        }

        // ── State transition actions ─────────────────────────────────────────
        if (sys_state != prev_sys_state) {
            const char* state_names[] = { "SERVO_ACTIVE", "TENS_ACTIVE", "IDLE" };
            float trans_err = last_tx - last_cx;
            float trans_dist = std::abs(trans_err);
            std::printf("[STATE] %s → %s (err=%.1f dist=%.1f have_target=%d)\n",
                        state_names[static_cast<int>(prev_sys_state)],
                        state_names[static_cast<int>(sys_state)],
                        trans_err, trans_dist, have_target ? 1 : 0);
            float eff_fire_t = cfg.fire_threshold_px + last_blob_w * cfg.fire_threshold_width_scale;
            float eff_flick_t = cfg.flick_threshold_px + last_blob_w * cfg.flick_threshold_width_scale;
            std::printf("  [STATE:thresholds] tens_act=%.0f±%.0f servo_act=%.0f flick=%.0f(eff=%.0f) fire=%.0f(eff=%.0f) blob_w=%.1f\n",
                        cfg.tens_activation_px, cfg.tens_hysteresis_px,
                        cfg.activation_range_px, cfg.flick_threshold_px, eff_flick_t,
                        cfg.fire_threshold_px, eff_fire_t, last_blob_w);
            std::fflush(stdout);

            // Leaving TENS_ACTIVE: zero pots immediately
            if (prev_sys_state == SystemState::TENS_ACTIVE) {
                tens_all_off();
                tens.active_channel = -1;
            }

            // Entering any state: reset PID
            pid.integral_x   = 0.0f;
            pid.prev_error_x = last_tx - last_cx;
            pid.filtered_deriv = 0.0f;
            pid.last_push_sign = 0;
            pid.backlash_offset = 0.0f;
            pid.backlash_ramp = 0;

            // Entering TENS_ACTIVE or IDLE: center servo
            if (sys_state == SystemState::TENS_ACTIVE ||
                sys_state == SystemState::IDLE) {
                feetech_send_position(servo_fd, servo_id, cfg.servo_center);
                servo.current_pos = cfg.servo_center;
            }
        }

        // ── Execute current state ────────────────────────────────────────────
        if (sys_state == SystemState::SERVO_ACTIVE && have_target) {
            float error_x = last_tx - last_cx;

            // Verbose logging at ~10Hz
            static uint64_t servo_log_tick = 0;
            bool verbose = (++servo_log_tick % 100 == 0);

            // Compute dynamic thresholds/gains based on blob width
            float eff_fire_threshold = cfg.fire_threshold_px + last_blob_w * cfg.fire_threshold_width_scale;
            float eff_flick_threshold = cfg.flick_threshold_px + last_blob_w * cfg.flick_threshold_width_scale;
            float settle_gain_scale = (cfg.settle_gain_reference_width > 0.0f)
                ? std::clamp(last_blob_w / cfg.settle_gain_reference_width, cfg.settle_gain_min_scale, 1.0f)
                : 1.0f;

            if (verbose) {
                std::printf("──────────────── tick %lu ────────────────\n", servo_log_tick);
                std::printf("[INPUT] tx=%.1f cx=%.1f blob_w=%.1f → err=%.1f dt=%.6fs new_pkt=%d\n",
                            last_tx, last_cx, last_blob_w, error_x, dt, new_packet ? 1 : 0);
                std::printf("[DYNAMIC] eff_fire=%.1f eff_flick=%.1f settle_gain_scale=%.2f\n",
                            eff_fire_threshold, eff_flick_threshold, settle_gain_scale);
            }

            ControlOutput out = compute_pid(pid, error_x, dt, servo.current_pos,
                                            eff_fire_threshold, eff_flick_threshold, settle_gain_scale, verbose);

            int smoothed_pos = smooth_servo_position(servo, out.servo_target_pos, verbose);

            if (servo.torque_refresh_cnt <= 0) {
                feetech_send_torque_enable(servo_fd, servo_id);
                if (verbose) std::printf("  [TORQUE] refreshed\n");
                servo.torque_refresh_cnt = cfg.torque_refresh_every;
            } else {
                servo.torque_refresh_cnt--;
            }

            feetech_send_position(servo_fd, servo_id, smoothed_pos);

            if (verbose) {
                const char* mode = (out.mode == ControlState::FLICK) ? "FLICK" : "SETTLE";
                char backlash_str[32] = "";
                if (out.backlash_applied) std::snprintf(backlash_str, sizeof(backlash_str), " +backlash(%.0f)", out.backlash_amount);
                std::printf("[SUMMARY] %s err=%.1f dist=%.1f offset=%.1f target=%d smoothed=%d%s%s%s\n",
                            mode, error_x, out.dist, out.servo_offset,
                            out.servo_target_pos, smoothed_pos,
                            backlash_str,
                            out.damped ? " DAMPED" : "",
                            out.fire ? " FIRE" : "");
                std::fflush(stdout);
            }

            if (cfg.solenoid_enabled && out.fire) {
                if (verbose) std::printf("  [TRIGGER] firing solenoid!\n");
                maybe_fire();
            }

        } else if (sys_state == SystemState::TENS_ACTIVE && have_target) {
            float error_x = last_tx - last_cx;
            float dist = std::abs(error_x);

            // Channel selection: positive error → ch0, negative → ch1
            int target_ch = (error_x > 0.0f) ? 0 : 1;
            int other_ch  = 1 - target_ch;

            // Intensity: linear map from tens_activation_px to activation_range_px
            float beyond = dist - cfg.tens_activation_px;
            float max_beyond = cfg.activation_range_px - cfg.tens_activation_px;
            float frac = std::clamp(beyond / max_beyond, 0.0f, 1.0f);
            int intensity = cfg.tens_min_intensity +
                static_cast<int>(frac * (cfg.tens_max_intensity - cfg.tens_min_intensity));
            intensity = std::min(intensity, 100);  // hard safety cap

            // Update wiper if cooldown elapsed
            double cooldown_s = cfg.tens_cooldown_ms / 1000.0;
            if ((now - tens.last_update_time) >= cooldown_s) {
                tens_set_wiper(tens_spi_handle[target_ch],
                               static_cast<uint8_t>(intensity));
                tens_off(other_ch);

                // Log channel changes and periodic intensity updates
                static uint64_t tens_log_tick = 0;
                bool tens_verbose = (++tens_log_tick % 100 == 0);
                if (tens.active_channel != target_ch) {
                    std::printf("[TENS] ch%d → ch%d err=%.1f dist=%.1f intensity=%d frac=%.2f\n",
                                tens.active_channel, target_ch, error_x, dist, intensity, frac);
                    std::fflush(stdout);
                } else if (tens_verbose) {
                    std::printf("[TENS] ch%d err=%.1f dist=%.1f intensity=%d frac=%.2f\n",
                                target_ch, error_x, dist, intensity, frac);
                    std::fflush(stdout);
                }

                tens.active_channel = target_ch;
                tens.last_update_time = now;
            }
        }
        // IDLE: nothing to do (servo centered, TENS off from transition)

        // ── Busy-spin to maintain loop rate ──────────────────────────────────
        struct timespec wake = ts_add_ns(loop_start, cfg.loop_period_ns);
        spin_until(wake);
    }

    // ── Cleanup ──────────────────────────────────────────────────────────────
    // Ignore further signals so a second Ctrl+C doesn't kill us mid-cleanup
    std::signal(SIGINT,  SIG_IGN);
    std::signal(SIGTERM, SIG_IGN);
    std::printf("\n[INFO] Shutting down...\n");

    feetech_send_position(servo_fd, servo_id, cfg.servo_center);

    // TENS: zero pots and close SPI
    if (cfg.tens_enabled) {
        tens_all_off();
        if (tens_spi_handle[0] >= 0) spiClose(static_cast<unsigned>(tens_spi_handle[0]));
        if (tens_spi_handle[1] >= 0) spiClose(static_cast<unsigned>(tens_spi_handle[1]));
    }

    if (cfg.solenoid_enabled) {
        gpioWrite(cfg.pin_trigger, 0);
    }

    if (need_pigpio) {
        gpioDelay(100000);
        gpioTerminate();
    }

    close(servo_fd);
    close(sock);
    std::printf("[INFO] Control node stopped.\n");
    return 0;
}
