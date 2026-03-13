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
static constexpr int     PACKET_SIZE     = 16;

// ─────────────────────────────────────────────────────────────────────────────
// Configuration — loaded from config.json at startup
// ─────────────────────────────────────────────────────────────────────────────

static constexpr const char* CONFIG_PATH = "config.json";

struct Config {
    // GPIO
    int pin_trigger = 17;

    // Servo serial
    std::string servo_serial_port = "/dev/ttyS0";
    int servo_id = 1;

    // Servo position
    int servo_center = 2048;
    int servo_min    = 1024;
    int servo_max    = 3072;
    int servo_range  = 1024;

    // PID → position scaling
    float pid_to_pos_scale = 5.0f;

    // Position smoothing
    bool smooth_enabled = true;
    int  smooth_divisor = 2;

    // Torque refresh
    int torque_refresh_every = 512;

    // PID — FLICK state
    float kp_flick = 8.0f;
    float kd_flick = 4.0f;

    // PID — SETTLE state
    float kp_settle = 5.0f;
    float ki_settle = 0.15f;
    float kd_settle = 3.0f;

    // Integral clamp
    float integral_clamp = 200.0f;

    // State machine thresholds
    float flick_threshold_px  = 30.0f;
    float activation_range_px = 1000.0f;
    float fire_threshold_px   = 5.0f;
    float target_switch_px    = 50.0f;

    // Trigger
    int trigger_min_ms   = 30;
    int trigger_range_ms = 30;

    // Network
    int udp_port = 5005;

    // Control loop
    long loop_period_ns = 1000000L;
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
    get_int  ("udp_port",           c.udp_port);
    get_long ("loop_period_ns",     c.loop_period_ns);

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

struct PIDState {
    float integral_x    = 0.0f;
    float prev_error_x  = 0.0f;
    ControlState state  = ControlState::FLICK;
};

struct ServoState {
    int   current_pos       = 0;   // initialized from cfg.servo_center in main()
    int   torque_refresh_cnt = 0;
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
    while (true) {
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

static void maybe_fire() {
    if (trigger_active.exchange(true)) return;

    std::printf("[DEBUG] Solenoid firing\n");
    std::fflush(stdout);

    std::thread([]() {
        gpioWrite(cfg.pin_trigger, 1);
        unsigned dwell_us = static_cast<unsigned>(
            (cfg.trigger_min_ms + (std::rand() % (cfg.trigger_range_ms + 1))) * 1000
        );
        gpioDelay(dwell_us);
        gpioWrite(cfg.pin_trigger, 0);
        std::printf("[DEBUG] Solenoid released (dwell=%u ms)\n", dwell_us / 1000);
        std::fflush(stdout);
        trigger_active.store(false);
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

    // ── pigpio init (for solenoid trigger only) ──────────────────────────────
    if (gpioInitialise() < 0) {
        std::fprintf(stderr, "[ERROR] gpioInitialise() failed. Run as root?\n");
        return 1;
    }

    gpioSetMode(cfg.pin_trigger, PI_OUTPUT);
    gpioWrite(cfg.pin_trigger, 0);

    std::srand(static_cast<unsigned>(time(nullptr)));

    // ── Feetech servo serial port ────────────────────────────────────────────
    int servo_fd = feetech_open(cfg.servo_serial_port.c_str());
    if (servo_fd < 0) {
        std::fprintf(stderr, "[ERROR] Failed to open servo serial port %s\n",
                     cfg.servo_serial_port.c_str());
        gpioTerminate();
        return 1;
    }

    uint8_t servo_id = static_cast<uint8_t>(cfg.servo_id);

    feetech_send_torque_enable(servo_fd, servo_id);
    std::printf("[INFO] Servo torque enabled (ID=0x%02X, port=%s)\n",
                servo_id, cfg.servo_serial_port.c_str());

    feetech_send_position(servo_fd, servo_id, cfg.servo_center);

    // ── UDP socket ───────────────────────────────────────────────────────────
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::perror("[ERROR] socket()");
        close(servo_fd);
        gpioTerminate();
        return 1;
    }

    if (fcntl(sock, F_SETFL, O_NONBLOCK) < 0) {
        std::perror("[ERROR] fcntl(O_NONBLOCK)");
        close(servo_fd);
        gpioTerminate();
        return 1;
    }

    struct sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(static_cast<uint16_t>(cfg.udp_port));
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("[ERROR] bind()");
        close(servo_fd);
        gpioTerminate();
        return 1;
    }

    std::printf("[INFO] Control node listening on UDP port %d\n", cfg.udp_port);
    std::printf("[INFO] Servo: Feetech SCS/STS ID=0x%02X @ %s (1 Mbps)\n",
                servo_id, cfg.servo_serial_port.c_str());
    std::printf("[INFO] Servo position: center=%d range=[%d, %d]\n",
                cfg.servo_center, cfg.servo_min, cfg.servo_max);
    std::printf("[INFO] Trigger: GPIO%d\n", cfg.pin_trigger);
    std::printf("[INFO] PID Flick  — Kp=%.2f Kd=%.2f\n", cfg.kp_flick, cfg.kd_flick);
    std::printf("[INFO] PID Settle — Kp=%.2f Ki=%.3f Kd=%.2f\n",
                cfg.kp_settle, cfg.ki_settle, cfg.kd_settle);
    std::printf("[INFO] PID→Pos scale=%.2f  Smoothing=%s (divisor=%d)\n",
                cfg.pid_to_pos_scale,
                cfg.smooth_enabled ? "on" : "off",
                cfg.smooth_divisor);

    // ── State ────────────────────────────────────────────────────────────────
    PIDState   pid{};
    ServoState servo{};
    servo.current_pos = cfg.servo_center;
    uint8_t    recv_buf[PACKET_SIZE];

    float    last_tx = 0.0f;
    float    last_cx = 0.0f;
    bool     have_target = false;

    double   prev_time = mono_seconds();
    uint64_t recv_count = 0;
    bool     servo_was_active = false;

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

        // ── Control law ──────────────────────────────────────────────────────
        if (have_target) {
            float error_x = last_tx - last_cx;
            float dist = std::abs(error_x);
            bool servo_active = (dist <= cfg.activation_range_px);

            if (servo_active && !servo_was_active) {
                std::printf("[DEBUG] Servo activated (target in range, dist=%.1f px)\n",
                            dist);
                std::fflush(stdout);
            } else if (!servo_active && servo_was_active) {
                std::printf("[DEBUG] Servo deactivated (target out of range, dist=%.1f px)\n",
                            dist);
                std::fflush(stdout);
            }
            servo_was_active = servo_active;

            ControlOutput out = compute_pid(pid, error_x, dt);

            int smoothed_pos = smooth_servo_position(servo, out.servo_target_pos);

            if (servo.torque_refresh_cnt <= 0) {
                feetech_send_torque_enable(servo_fd, servo_id);
                servo.torque_refresh_cnt = cfg.torque_refresh_every;
            } else {
                servo.torque_refresh_cnt--;
            }

            feetech_send_position(servo_fd, servo_id, smoothed_pos);

            if (out.fire) {
                maybe_fire();
            }
        }

        // ── Busy-spin to maintain loop rate ──────────────────────────────────
        struct timespec wake = ts_add_ns(loop_start, cfg.loop_period_ns);
        spin_until(wake);
    }

    // ── Cleanup ──────────────────────────────────────────────────────────────
    std::printf("\n[INFO] Shutting down...\n");

    feetech_send_position(servo_fd, servo_id, cfg.servo_center);
    gpioWrite(cfg.pin_trigger, 0);

    gpioDelay(100000);

    close(servo_fd);
    close(sock);
    gpioTerminate();
    std::printf("[INFO] Control node stopped.\n");
    return 0;
}
