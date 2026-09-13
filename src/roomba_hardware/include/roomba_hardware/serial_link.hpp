#ifndef ROOMBA_HARDWARE__SERIAL_LINK_HPP_
#define ROOMBA_HARDWARE__SERIAL_LINK_HPP_

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace roomba_hardware
{

/// Owns the ESP32 serial connection and does all blocking I/O on a private
/// background thread, so the ros2_control read()/write() calls (invoked at
/// 100 Hz by controller_manager) never block on serial syscalls or on a
/// device that is absent/reconnecting.
///
/// Wire protocol (verified against dual_motor_controller_ros2_compatible.ino,
/// unit: degrees / degrees-per-second):
///   out: "V1 <deg_s>\n", "V2 <deg_s>\n", "S\n", "Z\n", "P\n"
///   in : "POS A=<f>deg  VEL A=<f>deg/s   |   POS B=<f>deg  VEL B=<f>deg/s\n"
///        streamed unprompted ~every 150ms *only while at least one motor is
///        non-idle* (confirmed from firmware's `loop()`: the telemetry print
///        is gated on `active`), and once in reply to "P" -- which works
///        regardless of idle/active state. This class uses "P" internally
///        (see maybe_poll_telemetry) purely to keep telemetry fresh while
///        both motors are at rest, since otherwise nothing would ever refresh
///        read() state at rest. Anything else on the wire (command acks,
///        help text) is ignored by the parser.
class SerialLink
{
public:
  struct Telemetry
  {
    double pos_a_deg = 0.0;
    double vel_a_deg_s = 0.0;
    double pos_b_deg = 0.0;
    double vel_b_deg_s = 0.0;
    // Wall-clock time this Telemetry was last refreshed from a parsed line.
    // Not the same as "data is stale" -- callers decide what staleness means.
    std::chrono::steady_clock::time_point stamp{};
    bool ever_received = false;
  };

  SerialLink(std::string port, int baud_rate, rclcpp::Logger logger);
  ~SerialLink();

  SerialLink(const SerialLink &) = delete;
  SerialLink & operator=(const SerialLink &) = delete;

  /// Spawn the background I/O thread. Safe to call even if the device isn't
  /// plugged in yet -- the thread will keep retrying to open the port.
  void start();

  /// Signal the background thread to stop and join it. Blocking, but bounded
  /// (the thread polls with a short timeout), so this is only called from
  /// on_deactivate/on_cleanup/destruction, never from read()/write().
  void stop();

  /// Non-blocking: just latches the desired per-motor speed. The background
  /// thread decides when to actually put it on the wire (see maybe_send_commands
  /// in the .cpp for the rate-limiting rule and rationale).
  void set_command_deg_s(double motor_a_deg_s, double motor_b_deg_s);

  /// Non-blocking: queues a one-shot "Z" (zero encoders) to be sent as soon
  /// as the background thread's next loop iteration runs.
  void request_zero();

  /// Non-blocking: queues a one-shot "S" (ramped stop) and clears any
  /// pending velocity command so we don't immediately re-send a nonzero
  /// V1/V2 behind the stop.
  void request_stop();

  /// Thread-safe snapshot of the latest parsed telemetry. If ever_received
  /// is false, no valid telemetry line has been parsed yet since connecting.
  Telemetry get_telemetry() const;

  bool is_connected() const { return connected_.load(std::memory_order_relaxed); }

private:
  void thread_main();
  bool try_open();
  void close_port();
  void process_incoming();
  void maybe_send_commands(std::chrono::steady_clock::time_point now);
  void maybe_poll_telemetry(std::chrono::steady_clock::time_point now);
  void service_one_shot_requests();
  bool write_line(const std::string & line);
  void parse_buffered_lines();

  const std::string port_;
  const int baud_rate_;
  rclcpp::Logger logger_;

  int fd_ = -1;
  std::atomic<bool> connected_{false};
  std::atomic<bool> running_{false};
  std::thread thread_;

  // Growing receive buffer; complete lines are extracted and erased as they
  // are found, so a malformed/partial line never permanently desyncs parsing
  // -- we just resume scanning after the next '\n'.
  std::string rx_buffer_;

  mutable std::mutex telemetry_mutex_;
  Telemetry telemetry_;

  std::atomic<double> cmd_a_deg_s_{0.0};
  std::atomic<double> cmd_b_deg_s_{0.0};
  std::atomic<bool> zero_requested_{false};
  std::atomic<bool> stop_requested_{false};

  // Background-thread-only state (no synchronization needed, never touched
  // from outside thread_main).
  double last_sent_a_deg_s_ = 0.0;
  double last_sent_b_deg_s_ = 0.0;
  std::chrono::steady_clock::time_point last_send_time_{};
  std::chrono::steady_clock::time_point last_open_attempt_{};
  // Last time we either received a real telemetry line or sent a "P" asking
  // for one -- drives maybe_poll_telemetry's idle fallback (see .cpp).
  std::chrono::steady_clock::time_point last_telemetry_request_{};
};

}  // namespace roomba_hardware

#endif  // ROOMBA_HARDWARE__SERIAL_LINK_HPP_
