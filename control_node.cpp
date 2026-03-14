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
static constexpr int     PACKET_SIZE     = 16;

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
    int servo_min    = 1365;
    int servo_max    = 2731;
    int servo_range  = 683;

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

    // State machine thresholds
    float flick_threshold_px  = 30.0f;
    float activation_range_px = 1000.0f;
    float fire_threshold_px   = 5.0f;
    float target_switch_px    = 50.0f;

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
    get_float("flick_threshold_px", c.flick_threshold_px);
    get_float("activation_range_px",c.activation_range_px);
    get_float("fire_threshold_px",  c.fire_threshold_px);
    get_float("target_switch_px",   c.target_switch_px);
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

    // Safety: hard-cap servo rotation to ±60° from center (683 steps on 4096-step/360° servo)
    static constexpr int MAX_SERVO_RANGE = 683;  // 60° = 60 * (4096/360) ≈ 683
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
};
#pragma pack(pop)

enum class ControlState : uint8_t { FLICK, SETTLE };
enum class SystemState  : uint8_t { SERVO_ACTIVE, TENS_ACTIVE, IDLE };

struct PIDState {
    float integral_x    = 0.0f;
    float prev_error_x  = 0.0f;
    ControlState state  = ControlState::FLICK;
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

// MCP4131 TCON register shutdown: writing 0x00 to TCON (address 0x04) clears
// R0HW, R0A, R0W, R0B — disconnecting all terminals from the resistor network.
// Command byte: (addr << 4) | (cmd << 2) = (0x04 << 4) | (0x00 << 2) = 0x40
// This is a harder "off" than wiper=0, which still leaves ~75Ω wiper resistance.
static void tens_shutdown(int handle, bool verbose = false) {
    if (handle < 0) return;
    char buf[2] = { 0x40, 0x00 };
    int rc = spiWrite(static_cast<unsigned>(handle), buf, 2);
    if (verbose) {
        std::printf("[TENS] shutdown (TCON=0x00) handle=%d rc=%d\n", handle, rc);
        std::fflush(stdout);
    }
}

// Re-enable all TCON terminals after a shutdown (TCON = 0xFF)
static void tens_wakeup(int handle) {
    if (handle < 0) return;
    char buf[2] = { 0x40, static_cast<char>(0xFF) };
    spiWrite(static_cast<unsigned>(handle), buf, 2);
}

static void tens_off(int ch) {
    if (ch >= 0 && ch < 2) {
        tens_set_wiper(tens_spi_handle[ch], 0);
        tens_shutdown(tens_spi_handle[ch]);
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

    out.tx = read_f32(buf +  8);
    out.cx = read_f32(buf + 12);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// PID state machine
// ─────────────────────────────────────────────────────────────────────────────

struct ControlOutput {
    int   servo_target_pos;
    bool  fire;
};

static ControlOutput compute_pid(PIDState& pid,
                                  float error_x,
                                  float dt_s) {
    float dist = std::abs(error_x);

    if (dist > cfg.activation_range_px) {
        pid.integral_x   = 0.0f;
        pid.prev_error_x = error_x;
        return { cfg.servo_center, false };
    }

    float delta_x = std::abs(error_x - pid.prev_error_x);
    if (delta_x > cfg.target_switch_px) {
        pid.integral_x   = 0.0f;
        pid.prev_error_x = error_x;
        pid.state = ControlState::FLICK;
    }

    if (dist > cfg.flick_threshold_px) {
        pid.state = ControlState::FLICK;
    } else {
        pid.state = ControlState::SETTLE;
    }

    float output_x;
    float safe_dt = (dt_s > 1e-6f) ? dt_s : 1e-6f;

    if (pid.state == ControlState::FLICK) {
        float deriv_x = (error_x - pid.prev_error_x) / safe_dt;
        output_x = cfg.kp_flick * error_x + cfg.kd_flick * deriv_x;
    } else {
        pid.integral_x = std::clamp(pid.integral_x + error_x * dt_s,
                                    -cfg.integral_clamp, cfg.integral_clamp);
        float deriv_x = (error_x - pid.prev_error_x) / safe_dt;
        output_x = cfg.kp_settle * error_x
                 + cfg.ki_settle * pid.integral_x
                 + cfg.kd_settle * deriv_x;
    }

    pid.prev_error_x = error_x;

    float servo_offset = output_x * cfg.pid_to_pos_scale;
    servo_offset = std::clamp(servo_offset,
                              static_cast<float>(-cfg.servo_range),
                              static_cast<float>(cfg.servo_range));
    int target_pos = std::clamp(cfg.servo_center + static_cast<int>(servo_offset),
                                cfg.servo_min, cfg.servo_max);

    return { target_pos, dist < cfg.fire_threshold_px };
}

static int smooth_servo_position(ServoState& ss, int target_pos) {
    if (!cfg.smooth_enabled) {
        ss.current_pos = std::clamp(target_pos, cfg.servo_min, cfg.servo_max);
        return ss.current_pos;
    }

    int delta = target_pos - ss.current_pos;
    int step  = delta / cfg.smooth_divisor;

    if (step == 0 && delta != 0) {
        step = (delta > 0) ? 1 : -1;
    }

    ss.current_pos = std::clamp(ss.current_pos + step, cfg.servo_min, cfg.servo_max);
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
    std::printf("[DEBUG] Solenoid firing\n");
    std::fflush(stdout);

    std::thread([]() {
        unsigned dwell_us = static_cast<unsigned>(
            (cfg.trigger_min_ms + (std::rand() % (cfg.trigger_range_ms + 1))) * 1000
        );
        gpioWrite(cfg.pin_trigger, 1);
        gpioDelay(dwell_us);
        gpioWrite(cfg.pin_trigger, 0);
        std::printf("[DEBUG] Solenoid released (dwell=%u ms)\n", dwell_us / 1000);
        std::fflush(stdout);
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
        if (n == PACKET_SIZE) {
            VisionPacket pkt;
            if (deserialize_packet(recv_buf, n, pkt)) {
                ++recv_count;
                last_packet_time = now;
                if (recv_count <= 5 || recv_count % 100 == 0) {
                    std::printf("[UDP] pkt #%lu: tx=%.1f cx=%.1f\n",
                                recv_count, pkt.tx, pkt.cx);
                    std::fflush(stdout);
                }
                last_tx = pkt.tx;
                last_cx = pkt.cx;
                have_target = true;
            }
        }

        // ── UDP watchdog: force IDLE if no packets for >500ms ────────────────
        if (have_target && (now - last_packet_time) > 0.5) {
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
            std::printf("[STATE] %s → %s\n",
                        state_names[static_cast<int>(prev_sys_state)],
                        state_names[static_cast<int>(sys_state)]);
            std::fflush(stdout);

            // Leaving TENS_ACTIVE: zero pots immediately
            if (prev_sys_state == SystemState::TENS_ACTIVE) {
                tens_all_off();
                tens.active_channel = -1;
            }

            // Entering any state: reset PID
            pid.integral_x   = 0.0f;
            pid.prev_error_x = last_tx - last_cx;

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

            ControlOutput out = compute_pid(pid, error_x, dt);

            int smoothed_pos = smooth_servo_position(servo, out.servo_target_pos);

            if (servo.torque_refresh_cnt <= 0) {
                feetech_send_torque_enable(servo_fd, servo_id);
                servo.torque_refresh_cnt = cfg.torque_refresh_every;
            } else {
                servo.torque_refresh_cnt--;
            }

            feetech_send_position(servo_fd, servo_id, smoothed_pos);

            if (cfg.solenoid_enabled && out.fire) {
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
            // Mutual exclusion: zero inactive channel FIRST, then set active — never both on
            double cooldown_s = cfg.tens_cooldown_ms / 1000.0;
            if ((now - tens.last_update_time) >= cooldown_s) {
                tens_off(other_ch);
                tens_wakeup(tens_spi_handle[target_ch]);
                tens_set_wiper(tens_spi_handle[target_ch],
                               static_cast<uint8_t>(intensity));
                tens.active_channel = target_ch;
                tens.last_update_time = now;

                static uint64_t tens_log_count = 0;
                if (++tens_log_count <= 5 || tens_log_count % 100 == 0) {
                    std::printf("[TENS] #%lu ch=%d intensity=%d dist=%.1f\n",
                                tens_log_count, target_ch, intensity, dist);
                    std::fflush(stdout);
                }
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

    // TENS: zero + shutdown both pots, then close SPI
    if (cfg.tens_enabled) {
        std::printf("[TENS] Shutting down both channels (handles: %d, %d)...\n",
                    tens_spi_handle[0], tens_spi_handle[1]);
        std::fflush(stdout);
        tens_set_wiper(tens_spi_handle[0], 0, true);
        tens_shutdown(tens_spi_handle[0], true);
        tens_set_wiper(tens_spi_handle[1], 0, true);
        tens_shutdown(tens_spi_handle[1], true);
        gpioDelay(10000);
        if (tens_spi_handle[0] >= 0) spiClose(static_cast<unsigned>(tens_spi_handle[0]));
        if (tens_spi_handle[1] >= 0) spiClose(static_cast<unsigned>(tens_spi_handle[1]));
        std::printf("[TENS] Both channels shut down and SPI closed\n");
        std::fflush(stdout);
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
