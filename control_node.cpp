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
 
 // ── GPIO Pin Assignments (BCM numbering) ─────────────────────────────────────
 // Trigger pin only — servo is now driven over serial, not PWM.
 static constexpr int PIN_TRIGGER  = 17;   // Header pin 11 → MOSFET gate → solenoid
 
 // ── Feetech SCS/STS Servo Serial Parameters ─────────────────────────────────
 // ST3215 servo communicates via half-duplex UART at 1 Mbps.
 // Protocol ported from NeuromuscularAimAssist FPGA feetech_servo module.
 static constexpr const char* SERVO_SERIAL_PORT = "/dev/ttyS0";
 static constexpr speed_t     SERVO_BAUD        = B1000000;   // 1 Mbps
 static constexpr uint8_t     SERVO_ID          = 0x01;       // unicast; use 0xFE for broadcast during bring-up
 
 // Feetech SCS/STS protocol constants
 static constexpr uint8_t SCS_HEADER    = 0xFF;
 static constexpr uint8_t SCS_INST_WRITE = 0x03;   // WRITE_DATA instruction
 static constexpr uint8_t SCS_ADDR_TORQUE = 0x28;  // Torque Enable register
 static constexpr uint8_t SCS_ADDR_GOAL   = 0x2A;  // Goal Position register (2 bytes)
 
 // ── Servo Position Parameters ────────────────────────────────────────────────
 // ST3215 uses 12-bit position (0-4095). Center at 2048.
 // Operating range clamped to [SERVO_MIN, SERVO_MAX] to protect mechanical limits.
 static constexpr int SERVO_CENTER = 2048;
 static constexpr int SERVO_MIN    = 1024;
 static constexpr int SERVO_MAX    = 3072;
 static constexpr int SERVO_RANGE  = 1024;   // max offset from center
 
 // PID output → servo position scaling.
 // pid_output is in pixel-error-weighted units; this maps to servo position offset.
 // servo_offset = pid_output * PID_TO_POS_SCALE
 // Example: error=15px, Kp_settle=5 → pid_output=75 → 75*1.0 = 75 steps offset.
 //
 // Start low and tune up — the FPGA's 512/50 ratio is not directly portable to this PID gain set.
 // TUNE: increase if servo barely moves; decrease if it overshoots.
 static constexpr float PID_TO_POS_SCALE = 1.0f;
 
 // Number of position writes between torque-enable refreshes.
 // Keeps the servo's torque-enable register active in case of power glitches.
 static constexpr int TORQUE_REFRESH_EVERY = 512;
 
 // ── PID Tuning Parameters — FLICK state ──────────────────────────────────────
 // Active when Euclidean distance to target > FLICK_THRESHOLD_PX.
 // Uses PD only (no integral) to achieve fast ballistic slew to target.
 static constexpr float Kp_flick = 8.0f;
 static constexpr float Kd_flick = 4.0f;
 
 // ── PID Tuning Parameters — SETTLE state ─────────────────────────────────────
 // Active when Euclidean distance to target < FLICK_THRESHOLD_PX.
 // Full PID for precision lock; integral eliminates steady-state offset.
 static constexpr float Kp_settle = 5.0f;
 static constexpr float Ki_settle = 0.15f;
 static constexpr float Kd_settle = 3.0f;
 
 // Hard clamp on integral accumulator (in pixel·seconds).
 static constexpr float INTEGRAL_CLAMP = 200.0f;
 
 // ── State Machine Thresholds ──────────────────────────────────────────────────
 static constexpr float FLICK_THRESHOLD_PX = 30.0f;
 static constexpr float ACTIVATION_RANGE_PX = 50.0f;
 static constexpr float FIRE_THRESHOLD_PX  = 5.0f;
 static constexpr float TARGET_SWITCH_PX   = 50.0f;
 
 // ── Trigger Parameters ────────────────────────────────────────────────────────
 static constexpr int TRIGGER_MIN_MS   = 30;
 static constexpr int TRIGGER_RANGE_MS = 30;
 
 // ── Network ───────────────────────────────────────────────────────────────────
 static constexpr int   UDP_PORT       = 5005;
 static constexpr int   PACKET_SIZE    = 16;
 
 // ── Control Loop ──────────────────────────────────────────────────────────────
 static constexpr long LOOP_PERIOD_NS = 1000000L;   // 1ms = 1kHz
 
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
 
 // ── Servo position smoothing state ───────────────────────────────────────────
 // First-order exponential smoother ported from FPGA control_splitter.
 // Each update moves 1/4 of the remaining error to prevent slamming.
 struct ServoState {
     int   current_pos       = SERVO_CENTER;   // smoothed position sent to servo
     int   torque_refresh_cnt = 0;             // counts down to next torque-enable refresh
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
 
 /*
  * feetech_open — open and configure the servo serial port.
  *
  * Configures the UART for 1 Mbps 8N1 with no flow control.
  * Returns file descriptor on success, -1 on failure.
  */
 static int feetech_open(const char* port) {
     int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
     if (fd < 0) {
         std::perror("[ERROR] feetech_open: open()");
         return -1;
     }
 
     struct termios tio;
     std::memset(&tio, 0, sizeof(tio));
 
     // Input/output baud rate
     cfsetispeed(&tio, SERVO_BAUD);
     cfsetospeed(&tio, SERVO_BAUD);
 
     // 8 data bits, no parity, 1 stop bit
     tio.c_cflag |= CS8 | CLOCAL | CREAD;
     tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
 
     // Raw mode: no echo, no canonical processing, no signals
     tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
 
     // No software flow control, no input processing
     tio.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNCR);
 
     // Raw output
     tio.c_oflag &= ~OPOST;
 
     // Non-blocking: return immediately from read()
     tio.c_cc[VMIN]  = 0;
     tio.c_cc[VTIME] = 0;
 
     if (tcsetattr(fd, TCSANOW, &tio) != 0) {
         std::perror("[ERROR] feetech_open: tcsetattr()");
         close(fd);
         return -1;
     }
 
     // Flush any stale data in the buffers
     tcflush(fd, TCIOFLUSH);
 
     return fd;
 }
 
 /*
  * feetech_send_torque_enable — send a torque-enable packet to the servo.
  *
  * Packet (8 bytes):
  *   0xFF 0xFF ID 0x04 0x03 0x28 0x01 CSUM
  *   CSUM = ~(ID + 0x04 + 0x03 + 0x28 + 0x01) & 0xFF
  */
 static void feetech_send_torque_enable(int fd, uint8_t id) {
     uint8_t len  = 0x04;
     uint8_t data = 0x01;   // torque on
     uint8_t csum = ~(id + len + SCS_INST_WRITE + SCS_ADDR_TORQUE + data) & 0xFF;
 
     uint8_t pkt[8] = {
         SCS_HEADER, SCS_HEADER,
         id, len, SCS_INST_WRITE, SCS_ADDR_TORQUE, data, csum
     };
 
     write(fd, pkt, sizeof(pkt));
     tcdrain(fd);   // wait for transmission to complete (half-duplex safety)
 }
 
 /*
  * feetech_send_position — send a goal-position write packet to the servo.
  *
  * Packet (9 bytes):
  *   0xFF 0xFF ID 0x05 0x03 0x2A POS_L POS_H CSUM
  *   CSUM = ~(ID + 0x05 + 0x03 + 0x2A + POS_L + POS_H) & 0xFF
  *
  * position: 12-bit value (0-4095), clamped internally.
  */
 static void feetech_send_position(int fd, uint8_t id, int position) {
     // Clamp to valid range
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
     // Note: no tcdrain() here — position writes happen at 1kHz in the hot loop.
     // The 1 Mbps link transmits 9 bytes in ~90µs, well within the 1ms loop period.
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
 
 /*
  * ControlOutput now produces a 12-bit servo target position instead of µs.
  */
 struct ControlOutput {
     int   servo_target_pos;   // target servo position [SERVO_MIN, SERVO_MAX]
     bool  fire;               // true = trigger solenoid
 };
 
 /*
  * compute_pid — core X-axis control law.
  *
  * Returns ControlOutput with target servo position (before smoothing) and fire flag.
  * The caller applies first-order smoothing before sending to the servo.
  */
 static ControlOutput compute_pid(PIDState& pid,
                                   float error_x,
                                   float dt_s) {
     float dist = std::abs(error_x);
 
     // ── Activation Range Check ───────────────────────────────────────────────
     if (dist > ACTIVATION_RANGE_PX) {
         pid.integral_x   = 0.0f;
         pid.prev_error_x = error_x;
         return { SERVO_CENTER, false };
     }
 
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
 
     // ── Map PID Output to Servo Target Position ──────────────────────────────
     float servo_offset = output_x * PID_TO_POS_SCALE;
     // Clamp offset to servo range
     servo_offset = std::clamp(servo_offset,
                               static_cast<float>(-SERVO_RANGE),
                               static_cast<float>(SERVO_RANGE));
     int target_pos = std::clamp(SERVO_CENTER + static_cast<int>(servo_offset),
                                 SERVO_MIN, SERVO_MAX);
 
     return { target_pos, dist < FIRE_THRESHOLD_PX };
 }
 
 /*
  * smooth_servo_position — first-order exponential smoother.
  *
  * Moves the current servo position 1/4 of the way toward the target each call.
  * Ensures at least ±1 step when there's any remaining error (no dead zone).
  * Ported from NeuromuscularAimAssist FPGA control_splitter module.
  */
 static int smooth_servo_position(ServoState& ss, int target_pos) {
     int delta = target_pos - ss.current_pos;
     int step  = delta / 4;   // integer division: move 25% of remaining error
 
     // Avoid getting stuck: if there's error but step rounded to 0, nudge by 1.
     if (step == 0 && delta != 0) {
         step = (delta > 0) ? 1 : -1;
     }
 
     ss.current_pos = std::clamp(ss.current_pos + step, SERVO_MIN, SERVO_MAX);
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
         gpioWrite(PIN_TRIGGER, 1);
         unsigned dwell_us = static_cast<unsigned>(
             (TRIGGER_MIN_MS + (std::rand() % (TRIGGER_RANGE_MS + 1))) * 1000
         );
         gpioDelay(dwell_us);
         gpioWrite(PIN_TRIGGER, 0);
         std::printf("[DEBUG] Solenoid released (dwell=%u ms)\n", dwell_us / 1000);
         std::fflush(stdout);
         trigger_active.store(false);
     }).detach();
 }
 
 // ─────────────────────────────────────────────────────────────────────────────
 // main
 // ─────────────────────────────────────────────────────────────────────────────
 int main() {
     // ── Signal handling ───────────────────────────────────────────────────────
     std::signal(SIGINT,  handle_signal);
     std::signal(SIGTERM, handle_signal);
 
     // ── pigpio init (for solenoid trigger only) ───────────────────────────────
     if (gpioInitialise() < 0) {
         std::fprintf(stderr, "[ERROR] gpioInitialise() failed. Run as root?\n");
         return 1;
     }
 
     gpioSetMode(PIN_TRIGGER, PI_OUTPUT);
     gpioWrite(PIN_TRIGGER, 0);
 
     std::srand(static_cast<unsigned>(time(nullptr)));
 
     // ── Feetech servo serial port ─────────────────────────────────────────────
     int servo_fd = feetech_open(SERVO_SERIAL_PORT);
     if (servo_fd < 0) {
         std::fprintf(stderr, "[ERROR] Failed to open servo serial port %s\n",
                      SERVO_SERIAL_PORT);
         gpioTerminate();
         return 1;
     }
 
     // Send initial torque-enable command
     feetech_send_torque_enable(servo_fd, SERVO_ID);
     std::printf("[INFO] Servo torque enabled (ID=0x%02X, port=%s)\n",
                 SERVO_ID, SERVO_SERIAL_PORT);
 
     // Center the servo on startup
     feetech_send_position(servo_fd, SERVO_ID, SERVO_CENTER);
 
     // ── UDP socket ────────────────────────────────────────────────────────────
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
     addr.sin_port        = htons(static_cast<uint16_t>(UDP_PORT));
     addr.sin_addr.s_addr = INADDR_ANY;
 
     if (bind(sock, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0) {
         std::perror("[ERROR] bind()");
         close(servo_fd);
         gpioTerminate();
         return 1;
     }
 
     std::printf("[INFO] Control node listening on UDP port %d\n", UDP_PORT);
     std::printf("[INFO] Servo: Feetech SCS/STS ID=0x%02X @ %s (1 Mbps)\n",
                 SERVO_ID, SERVO_SERIAL_PORT);
     std::printf("[INFO] Servo position: center=%d range=[%d, %d]\n",
                 SERVO_CENTER, SERVO_MIN, SERVO_MAX);
     std::printf("[INFO] Trigger: GPIO%d\n", PIN_TRIGGER);
     std::printf("[INFO] PID Flick  — Kp=%.2f Kd=%.2f\n", Kp_flick, Kd_flick);
     std::printf("[INFO] PID Settle — Kp=%.2f Ki=%.3f Kd=%.2f\n",
                 Kp_settle, Ki_settle, Kd_settle);
     std::printf("[INFO] PID→Pos scale=%.2f  Smoothing=1/4 per step\n",
                 PID_TO_POS_SCALE);
 
     // ── State ─────────────────────────────────────────────────────────────────
     PIDState   pid{};
     ServoState servo{};
     uint8_t    recv_buf[PACKET_SIZE];
 
     float    last_tx = 0.0f;
     float    last_cx = 0.0f;
     bool     have_target = false;
 
     double   prev_time = mono_seconds();
     uint64_t recv_count = 0;
     bool     servo_was_active = false;
 
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
 
         // ── Control law ───────────────────────────────────────────────────────
         if (have_target) {
             float error_x = last_tx - last_cx;
             float dist = std::abs(error_x);
             bool servo_active = (dist <= ACTIVATION_RANGE_PX);
 
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
 
             // Apply first-order smoothing (move 1/4 of error per update)
             int smoothed_pos = smooth_servo_position(servo, out.servo_target_pos);
 
             // Periodic torque-enable refresh
             if (servo.torque_refresh_cnt <= 0) {
                 feetech_send_torque_enable(servo_fd, SERVO_ID);
                 servo.torque_refresh_cnt = TORQUE_REFRESH_EVERY;
             } else {
                 servo.torque_refresh_cnt--;
             }
 
             // Send position command to servo over serial
             feetech_send_position(servo_fd, SERVO_ID, smoothed_pos);
 
             if (out.fire) {
                 maybe_fire();
             }
         }
 
         // ── Busy-spin to maintain 1kHz loop rate ──────────────────────────────
         struct timespec wake = ts_add_ns(loop_start, LOOP_PERIOD_NS);
         spin_until(wake);
     }
 
     // ── Cleanup ───────────────────────────────────────────────────────────────
     std::printf("\n[INFO] Shutting down...\n");
 
     // Safe stop: center servo, ensure trigger is off.
     feetech_send_position(servo_fd, SERVO_ID, SERVO_CENTER);
     gpioWrite(PIN_TRIGGER, 0);
 
     // Brief delay to allow any in-flight trigger thread to complete.
     gpioDelay(100000);   // 100ms
 
     close(servo_fd);
     close(sock);
     gpioTerminate();
     std::printf("[INFO] Control node stopped.\n");
     return 0;
 }
 