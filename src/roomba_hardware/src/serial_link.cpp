#include "roomba_hardware/serial_link.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <regex>

namespace roomba_hardware
{

namespace
{
// Telemetry line, e.g.:
//   POS A=12.50deg  VEL A=3.10deg/s   |   POS B=12.40deg  VEL B=3.05deg/s
// Whitespace on the wire is not guaranteed to match this exactly, so match
// loosely on whitespace and strictly on the literal tokens around it.
const std::regex kTelemetryRe(
  R"(POS\s*A=([-+]?[0-9]*\.?[0-9]+)deg\s*VEL\s*A=([-+]?[0-9]*\.?[0-9]+)deg/s\s*\|\s*)"
  R"(POS\s*B=([-+]?[0-9]*\.?[0-9]+)deg\s*VEL\s*B=([-+]?[0-9]*\.?[0-9]+)deg/s)");

// --- Command rate-limiting rule (write side) ---------------------------
// controller_manager calls write() at the controller_manager `update_rate`
// (100 Hz / 10ms in controllers.yaml) but the ESP32 only needs a new V1/V2
// when the commanded speed actually changes, plus a keepalive well inside
// the firmware's own command watchdog. dual_motor_controller_ros2_compatible
// confirms: a motor in VELOCITY mode with no V1/V2/VV refresh within
// VEL_CMD_TIMEOUT_MS (500ms there) gets force-stopped by the firmware
// itself, independent of us. So we send when either:
//   - the commanded speed on a motor has moved by more than
//     kCommandChangeThresholdDegS since the last value we actually sent, or
//   - kCommandKeepaliveInterval has elapsed since the last send, even if
//     unchanged.
// 1.0 deg/s is well under what's perceptible in wheel motion. 100ms
// keepalive gives a 5x margin under the firmware's 500ms watchdog (so a
// missed cycle or two from reconnect/scheduling jitter still won't trip
// it), while at idle (cmd == 0, unchanging) this caps traffic to 10 Hz per
// motor instead of 100 Hz. If VEL_CMD_TIMEOUT_MS ever changes in firmware,
// keep this comfortably below half of it.
constexpr double kCommandChangeThresholdDegS = 1.0;
constexpr std::chrono::milliseconds kCommandKeepaliveInterval{100};

// --- Idle-telemetry poll rule (read side) -------------------------------
// dual_motor_controller_ros2_compatible's loop() only calls printStatus()
// when `active` (i.e. at least one motor is out of MODE_IDLE) -- so once
// both wheels settle to a stop, telemetry stops arriving *at all*, and
// read() would otherwise hold whatever position/velocity was last measured
// forever (velocity in particular isn't guaranteed to be exactly 0 at the
// moment a motor goes idle, just under the firmware's own settle
// threshold). "P" always calls printStatus() unconditionally, idle or not,
// so we use it purely as a keep-fresh poll when nothing has arrived
// recently. 200ms is comfortably above the firmware's ~150ms active-
// streaming cadence (so a moving robot's real telemetry always beats this
// timer and no redundant "P" gets sent) but still tight enough that a
// stationary robot's read() state is never more than ~200ms stale.
constexpr std::chrono::milliseconds kIdleTelemetryPollInterval{200};

// How often to retry opening the port while it's absent/disconnected.
constexpr std::chrono::milliseconds kReconnectInterval{1000};

// poll() timeout for the background loop. Short enough that queued one-shot
// commands (Z/S) and rate-limited V1/V2 sends go out promptly, long enough
// to not busy-spin.
constexpr int kPollTimeoutMs = 20;

speed_t baud_to_speed_t(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
  }
}
}  // namespace

SerialLink::SerialLink(std::string port, int baud_rate, rclcpp::Logger logger)
: port_(std::move(port)), baud_rate_(baud_rate), logger_(logger)
{
}

SerialLink::~SerialLink() { stop(); }

void SerialLink::start()
{
  if (running_.exchange(true)) {
    return;  // already running
  }
  thread_ = std::thread(&SerialLink::thread_main, this);
}

void SerialLink::stop()
{
  if (!running_.exchange(false)) {
    return;  // wasn't running
  }
  if (thread_.joinable()) {
    thread_.join();
  }
  close_port();
}

void SerialLink::set_command_deg_s(double motor_a_deg_s, double motor_b_deg_s)
{
  cmd_a_deg_s_.store(motor_a_deg_s, std::memory_order_relaxed);
  cmd_b_deg_s_.store(motor_b_deg_s, std::memory_order_relaxed);
}

void SerialLink::request_zero() { zero_requested_.store(true, std::memory_order_relaxed); }

void SerialLink::request_stop()
{
  cmd_a_deg_s_.store(0.0, std::memory_order_relaxed);
  cmd_b_deg_s_.store(0.0, std::memory_order_relaxed);
  stop_requested_.store(true, std::memory_order_relaxed);
}

SerialLink::Telemetry SerialLink::get_telemetry() const
{
  std::lock_guard<std::mutex> lock(telemetry_mutex_);
  return telemetry_;
}

bool SerialLink::try_open()
{
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    RCLCPP_ERROR(logger_, "tcgetattr('%s') failed: %s", port_.c_str(), std::strerror(errno));
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  cfmakeraw(&tty);
  speed_t speed = baud_to_speed_t(baud_rate_);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(logger_, "tcsetattr('%s') failed: %s", port_.c_str(), std::strerror(errno));
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  rx_buffer_.clear();
  connected_.store(true, std::memory_order_relaxed);
  last_telemetry_request_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(logger_, "Opened ESP32 serial link on '%s' @ %d baud", port_.c_str(), baud_rate_);
  return true;
}

void SerialLink::close_port()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  connected_.store(false, std::memory_order_relaxed);
}

bool SerialLink::write_line(const std::string & line)
{
  if (fd_ < 0) {
    return false;
  }
  const std::string out = line + "\n";
  ssize_t written = ::write(fd_, out.data(), out.size());
  if (written < 0) {
    RCLCPP_WARN(
      logger_, "Serial write to '%s' failed: %s -- treating link as dropped", port_.c_str(),
      std::strerror(errno));
    close_port();
    return false;
  }
  return true;
}

void SerialLink::parse_buffered_lines()
{
  size_t pos;
  while ((pos = rx_buffer_.find('\n')) != std::string::npos) {
    std::string line = rx_buffer_.substr(0, pos);
    rx_buffer_.erase(0, pos + 1);
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    std::smatch m;
    if (std::regex_search(line, m, kTelemetryRe)) {
      Telemetry t;
      t.pos_a_deg = std::stod(m[1]);
      t.vel_a_deg_s = std::stod(m[2]);
      t.pos_b_deg = std::stod(m[3]);
      t.vel_b_deg_s = std::stod(m[4]);
      t.stamp = std::chrono::steady_clock::now();
      t.ever_received = true;
      {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        telemetry_ = t;
      }
      last_telemetry_request_ = t.stamp;
    }
    // Anything else on the wire (command acks, help text from '?', partial
    // noise on connect) is intentionally ignored -- not every line is
    // telemetry, and that's expected per the protocol.
  }

  // Guard against a stuck line with no '\n' ever arriving (e.g. noise on a
  // half-connected cable) from growing rx_buffer_ unbounded.
  constexpr size_t kMaxBufferedBytes = 4096;
  if (rx_buffer_.size() > kMaxBufferedBytes) {
    RCLCPP_WARN(
      logger_, "Discarding %zu bytes of unterminated serial data from '%s'", rx_buffer_.size(),
      port_.c_str());
    rx_buffer_.clear();
  }
}

void SerialLink::process_incoming()
{
  char buf[256];
  while (true) {
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n > 0) {
      rx_buffer_.append(buf, static_cast<size_t>(n));
      continue;
    }
    if (n == 0) {
      break;  // nothing available right now
    }
    // n < 0
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      break;  // no more data buffered, normal for non-blocking fd
    }
    RCLCPP_WARN(
      logger_, "Serial read from '%s' failed: %s -- treating link as dropped", port_.c_str(),
      std::strerror(errno));
    close_port();
    return;
  }
  parse_buffered_lines();
}

void SerialLink::service_one_shot_requests()
{
  if (stop_requested_.exchange(false)) {
    write_line("S");
    last_sent_a_deg_s_ = 0.0;
    last_sent_b_deg_s_ = 0.0;
    last_send_time_ = std::chrono::steady_clock::now();
  }
  if (zero_requested_.exchange(false)) {
    write_line("Z");
  }
}

void SerialLink::maybe_send_commands(std::chrono::steady_clock::time_point now)
{
  const double a = cmd_a_deg_s_.load(std::memory_order_relaxed);
  const double b = cmd_b_deg_s_.load(std::memory_order_relaxed);

  const bool changed = std::abs(a - last_sent_a_deg_s_) > kCommandChangeThresholdDegS ||
    std::abs(b - last_sent_b_deg_s_) > kCommandChangeThresholdDegS;
  const bool keepalive_due = (now - last_send_time_) >= kCommandKeepaliveInterval;

  if (!changed && !keepalive_due) {
    return;
  }

  char line[64];
  std::snprintf(line, sizeof(line), "V1 %.3f", a);
  write_line(line);
  std::snprintf(line, sizeof(line), "V2 %.3f", b);
  write_line(line);

  last_sent_a_deg_s_ = a;
  last_sent_b_deg_s_ = b;
  last_send_time_ = now;
}

void SerialLink::maybe_poll_telemetry(std::chrono::steady_clock::time_point now)
{
  // Only a fallback for "both motors idle, firmware isn't streaming" (see
  // the rationale above kIdleTelemetryPollInterval) -- while moving, real
  // telemetry arrives faster than this timer and keeps resetting it, so
  // this branch is a no-op during normal driving.
  if (now - last_telemetry_request_ >= kIdleTelemetryPollInterval) {
    write_line("P");
    last_telemetry_request_ = now;
  }
}

void SerialLink::thread_main()
{
  while (running_.load(std::memory_order_relaxed)) {
    if (fd_ < 0) {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_open_attempt_ >= kReconnectInterval) {
        last_open_attempt_ = now;
        if (!try_open()) {
          // Stay disconnected; read()/write() on the hardware interface keep
          // reporting last-known state rather than crashing ros2_control_node.
          std::this_thread::sleep_for(std::chrono::milliseconds(kPollTimeoutMs));
          continue;
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollTimeoutMs));
        continue;
      }
    }

    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;
    int ret = ::poll(&pfd, 1, kPollTimeoutMs);
    if (ret < 0) {
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_WARN(logger_, "poll() on '%s' failed: %s", port_.c_str(), std::strerror(errno));
      close_port();
      continue;
    }
    if (ret > 0 && (pfd.revents & (POLLIN | POLLERR | POLLHUP))) {
      if (pfd.revents & (POLLERR | POLLHUP)) {
        RCLCPP_WARN(logger_, "Serial link '%s' reported HUP/ERR -- reconnecting", port_.c_str());
        close_port();
        continue;
      }
      process_incoming();
      if (fd_ < 0) {
        continue;  // process_incoming() dropped the link
      }
    }

    service_one_shot_requests();
    if (fd_ < 0) {
      continue;
    }
    const auto now = std::chrono::steady_clock::now();
    maybe_send_commands(now);
    maybe_poll_telemetry(now);
  }

  // Best-effort ramped stop on the way out (deactivate/shutdown already
  // calls request_stop() itself, but this covers destruction without an
  // explicit deactivate).
  if (fd_ >= 0) {
    write_line("S");
  }
  close_port();
}

}  // namespace roomba_hardware
