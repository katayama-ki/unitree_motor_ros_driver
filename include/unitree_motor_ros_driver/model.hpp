#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace unitree_driver {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
enum class MotorType { A1, B1, GO_M8010_6 };
enum class Mode : uint8_t { BRAKE = 0, FOC = 1, CALIBRATE = 2, UNKNOWN = 255 };
enum class ControlState { IDLE, ACTIVE, CMD_TIMED_OUT, FAULT_LATCHED };
enum class TimeoutAction { ZERO_TORQUE, DAMPING, BRAKE };
enum class Severity { INFO, WARN, ERROR };

struct MotorProfile {
  double sdk_gear_ratio;
  double velocity_limit;
  double torque_limit;
};

// Values used by libUnitreeMotorSDK_Linux64.so, Git blob
// ccd5cee41a3c32bb810e556327e1400bea3fd3de.
constexpr MotorProfile kA1Profile{9.1, 21.0, 33.5};
// Unitree's B1-16 product page does not publish a max speed. 15.55 rad/s is the calf
// velocity limit in Unitree's official b1_description; it is independently consistent
// with ~149.3 RPM reported for B1-16 by a third-party database.
constexpr MotorProfile kB1Profile{8.66, 15.55, 140.0};
constexpr MotorProfile kGoProfile{6.33, 30.0, 23.7};
MotorProfile profile(MotorType type);
MotorType parseMotorType(const std::string& value);
TimeoutAction parseTimeoutAction(const std::string& value);
bool isBroadcast(MotorType type, uint8_t id);
std::vector<uint8_t> scanIds(MotorType type, int maximum);

struct Command {
  uint8_t id = 0;
  float position = 0;
  float velocity = 0;
  float torque = 0;
  float kp = 0;
  float kd = 0;
};

// Physical fields are on the rotor side; kp/kd include the SDK's A1/B1 factors.
struct WireCommand {
  Command values;
  Mode mode = Mode::BRAKE;
};

struct Feedback {
  uint8_t id = 0;
  Mode mode = Mode::UNKNOWN;
  float position = 0;
  float velocity = 0;
  float torque = 0;
  int8_t temperature = 0;
  uint8_t error = 0;
  // Supplied at successful receive by the middleware clock callback.
  uint64_t stamp_ns = 0;
};

struct Config {
  std::string serial_port;
  MotorType motor_type = MotorType::GO_M8010_6;
  int motor_id_scan_max = 14;
  double command_timeout = 0.5;
  double active_io_rate = 100;
  double idle_io_rate = 10;
  double io_rate_warn_ratio = 0.9;
  double io_retry_rate = 0;
  double active_state_pub_rate = 100;
  double idle_state_pub_rate = 10;
  TimeoutAction timeout_action = TimeoutAction::ZERO_TORQUE;
  double timeout_damping_kd = 1;
  double velocity_limit = 0;
  double torque_limit = 0;
  double kp_limit = 200;
  double kd_limit = 10;
  int warn_temperature = 70;
  int stop_temperature = 80;
  bool stop_on_motor_error = true;

  // Validate before opening the serial port; resolve hardware limits and scan cap.
  std::vector<std::string> normalize();
};

WireCommand toWire(MotorType type, const Command& output, Mode mode = Mode::FOC);
Feedback toOutput(MotorType type, const Feedback& rotor);
bool validateCommand(const Config& config, const Command& command, std::string& reason);
std::string motorErrorText(MotorType type, uint8_t error);
Clock::duration period(double rate);
TimePoint advanceDeadline(TimePoint deadline, Clock::duration interval, TimePoint now);

// State and deadlines are owned by BusDriver's single context mutex.
struct MotorContext {
  uint8_t id = 0;
  ControlState state = ControlState::IDLE;
  bool io_error = false;
  int consecutive_io_failures = 0;
  Command latest;
  TimePoint command_received{};
  TimePoint next_due{};
  uint64_t schedule_revision = 0;
  TimePoint last_published{};
  uint64_t last_published_stamp = 0;
  bool has_published = false;
  int previous_error = 0;
  TimePoint rate_window{};
  unsigned attempts = 0;

  bool acceptCommand(const Command& command, TimePoint now);
  bool expire(const Config& config, TimePoint now);
  bool latchFault(TimePoint now);
  bool recordIo(bool success, TimePoint now);
  double ioRate(const Config& config) const;
  double pubRate(const Config& config) const;
  WireCommand wireCommand(const Config& config) const;
  bool shouldPublish(const Config& config, TimePoint now, uint64_t stamp_ns);
  void makeDue(TimePoint now);
};

}  // namespace unitree_driver
