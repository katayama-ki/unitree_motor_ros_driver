#include "unitree_motor_ros_driver/model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace unitree_driver {
namespace {
void require(bool valid, const std::string& name) {
  if (!valid) throw std::invalid_argument("Invalid parameter: " + name);
}
bool finiteNonnegative(double value) { return std::isfinite(value) && value >= 0; }
bool positiveRate(double value) {
  return std::isfinite(value) && value >= 1e-9 && value <= 1e9;
}
bool signedField(double encoded, double minimum, double maximum) {
  return std::isfinite(encoded) && encoded >= minimum && encoded <= maximum;
}
bool protocolFits(MotorType type, const Command& c) {
  if (type == MotorType::GO_M8010_6) {
    // The bundled SDK saturates at these values (not the misleading q7/q15 labels
    // alone in motor_msg_GO-M8010-6.h). Reject before it can mutate the command.
    return std::abs(c.torque) <= 127.99f && std::abs(c.velocity) <= 804.0f &&
           std::abs(c.position) <= 411774.0f && c.kp >= 0 && c.kp <= 25.599f &&
           c.kd >= 0 && c.kd <= 25.599f;
  }
  // Mirror SDK arithmetic, including float products and B1's kd encoding x512.
  return signedField(c.torque * 256.0f, -32768, 32767) &&
         signedField(c.velocity * 128.0f, -32768, 32767) &&
         signedField(static_cast<double>(c.position) / 6.2832 * 16384,
                     -2147483648.0, 2147483647.0) &&
         signedField(c.kp * 2048.0f, 0, 32767) &&
         signedField(c.kd * (type == MotorType::A1 ? 1024.0f : 512.0f), 0, 32767);
}
}  // namespace

MotorProfile profile(MotorType type) {
  switch (type) {
    case MotorType::A1: return kA1Profile;
    case MotorType::B1: return kB1Profile;
    case MotorType::GO_M8010_6: return kGoProfile;
  }
  throw std::invalid_argument("Unknown motor type");
}
MotorType parseMotorType(const std::string& value) {
  if (value == "A1") return MotorType::A1;
  if (value == "B1") return MotorType::B1;
  if (value == "GO-M8010-6") return MotorType::GO_M8010_6;
  throw std::invalid_argument("motor_type must be A1, B1, or GO-M8010-6");
}
TimeoutAction parseTimeoutAction(const std::string& value) {
  if (value == "zero_torque") return TimeoutAction::ZERO_TORQUE;
  if (value == "damping") return TimeoutAction::DAMPING;
  if (value == "brake") return TimeoutAction::BRAKE;
  throw std::invalid_argument("timeout_action must be zero_torque, damping, or brake");
}
bool isBroadcast(MotorType type, uint8_t id) {
  return id == (type == MotorType::GO_M8010_6 ? 15 : 0xBB);
}
std::vector<uint8_t> scanIds(MotorType type, int maximum) {
  require(maximum >= 0 && maximum <= 255, "motor_id_scan_max (0..255)");
  if (type == MotorType::GO_M8010_6) maximum = std::min(maximum, 14);
  std::vector<uint8_t> ids;
  for (int id = 0; id <= maximum; ++id) {
    if (!isBroadcast(type, static_cast<uint8_t>(id))) ids.push_back(id);
  }
  return ids;
}
std::vector<std::string> Config::normalize() {
  std::vector<std::string> warnings;
  require(!serial_port.empty(), "serial_port (required)");
  require(motor_id_scan_max >= 0, "motor_id_scan_max");
  if (motor_type == MotorType::GO_M8010_6 && motor_id_scan_max > 14) {
    warnings.push_back("GO-M8010-6 scan maximum capped at 14; ID 15 is broadcast");
    motor_id_scan_max = 14;
  }
  require(motor_id_scan_max <= 255, "motor_id_scan_max (uint8 ID)");
  require(std::isfinite(command_timeout) && command_timeout >= 1e-9 &&
          command_timeout <= 1e9, "command_timeout");
  require(positiveRate(active_io_rate), "active_io_rate");
  require(positiveRate(idle_io_rate), "idle_io_rate");
  require(io_retry_rate == 0 || positiveRate(io_retry_rate), "io_retry_rate");
  require(positiveRate(active_state_pub_rate), "active_state_pub_rate");
  require(positiveRate(idle_state_pub_rate), "idle_state_pub_rate");
  require(std::isfinite(io_rate_warn_ratio) && io_rate_warn_ratio > 0 &&
          io_rate_warn_ratio <= 1, "io_rate_warn_ratio (0..1]");
  require(finiteNonnegative(velocity_limit), "velocity_limit");
  require(finiteNonnegative(torque_limit), "torque_limit");
  require(finiteNonnegative(kp_limit), "kp_limit");
  require(finiteNonnegative(kd_limit), "kd_limit");
  require(finiteNonnegative(timeout_damping_kd), "timeout_damping_kd");
  require(warn_temperature >= -128 && stop_temperature <= 127 &&
          warn_temperature < stop_temperature, "warn_temperature < stop_temperature (-128..127)");
  const auto hardware = profile(motor_type);
  if (velocity_limit > hardware.velocity_limit)
    warnings.push_back("velocity_limit exceeds hardware limit; using hardware limit");
  if (torque_limit > hardware.torque_limit)
    warnings.push_back("torque_limit exceeds hardware limit; using hardware limit");
  if (velocity_limit == 0 || velocity_limit > hardware.velocity_limit)
    velocity_limit = hardware.velocity_limit;
  if (torque_limit == 0 || torque_limit > hardware.torque_limit)
    torque_limit = hardware.torque_limit;
  if (active_state_pub_rate > active_io_rate)
    warnings.push_back("active_state_pub_rate exceeds active_io_rate; feedback is limited by IO");
  if (idle_state_pub_rate > idle_io_rate)
    warnings.push_back("idle_state_pub_rate exceeds idle_io_rate; feedback is limited by IO");
  if (timeout_action == TimeoutAction::DAMPING) {
    require(timeout_damping_kd > 0, "timeout_damping_kd (damping requires > 0)");
    Command damping;
    damping.kd = static_cast<float>(timeout_damping_kd);
    std::string reason;
    const bool valid = validateCommand(*this, damping, reason);
    require(valid, "timeout_damping_kd: " + reason);
  }
  return warnings;
}
WireCommand toWire(MotorType type, const Command& output, Mode mode) {
  WireCommand result;
  result.mode = mode;
  result.values.id = output.id;
  if (mode == Mode::BRAKE) return result;
  const double r = profile(type).sdk_gear_ratio;
  result.values.position = output.position * r;
  result.values.velocity = output.velocity * r;
  result.values.torque = output.torque / r;
  result.values.kp = output.kp / (r * r);
  result.values.kd = output.kd / (r * r);
  if (type != MotorType::GO_M8010_6) {
    result.values.kp = output.kp / (r * r) / 26.07;
    result.values.kd = output.kd / (r * r) * 100.0;
  }
  return result;
}
Feedback toOutput(MotorType type, const Feedback& rotor) {
  Feedback result = rotor;
  const double r = profile(type).sdk_gear_ratio;
  result.position /= r;
  result.velocity /= r;
  result.torque *= r;
  return result;
}
bool validateCommand(const Config& config, const Command& c, std::string& reason) {
  if (!std::isfinite(c.position) || !std::isfinite(c.velocity) ||
      !std::isfinite(c.torque) || !std::isfinite(c.kp) || !std::isfinite(c.kd)) {
    reason = "NaN or Inf";
  } else if (isBroadcast(config.motor_type, c.id) ||
             (config.motor_type == MotorType::GO_M8010_6 && c.id > 14)) {
    reason = "broadcast or out-of-range ID";
  } else if (std::abs(c.velocity) > config.velocity_limit ||
             std::abs(c.torque) > config.torque_limit ||
             c.kp < 0 || c.kp > config.kp_limit || c.kd < 0 || c.kd > config.kd_limit) {
    reason = "configured output-side limit exceeded";
  } else if (!protocolFits(config.motor_type, toWire(config.motor_type, c).values)) {
    reason = "rotor-side value exceeds SDK protocol range";
  } else {
    reason.clear();
    return true;
  }
  return false;
}
std::string motorErrorText(MotorType type, uint8_t error) {
  if (error == 0) return "normal";
  if (type != MotorType::GO_M8010_6) return "meaning undocumented";
  switch (error) {
    case 1: return "over-temperature";
    case 2: return "over-current";
    case 3: return "over-voltage";
    case 4: return "encoder fault";
    default: return "unknown";
  }
}
Clock::duration period(double rate) {
  return std::max(Clock::duration(1),
    std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / rate)));
}
TimePoint advanceDeadline(TimePoint deadline, Clock::duration interval, TimePoint now) {
  if (deadline > now) return deadline;
  return deadline + interval * ((now - deadline) / interval + 1);
}
void MotorContext::makeDue(TimePoint now) {
  next_due = now;
  ++schedule_revision;
  rate_window = now;
  attempts = 0;
}
bool MotorContext::acceptCommand(const Command& command, TimePoint now) {
  latest = command;
  command_received = now;
  if (state == ControlState::FAULT_LATCHED || state == ControlState::ACTIVE) return false;
  state = ControlState::ACTIVE;
  makeDue(now);
  return true;
}
bool MotorContext::expire(const Config& config, TimePoint now) {
  if (state != ControlState::ACTIVE ||
      std::chrono::duration<double>(now - command_received).count() < config.command_timeout)
    return false;
  state = ControlState::CMD_TIMED_OUT;
  makeDue(now);
  return true;
}
bool MotorContext::latchFault(TimePoint now) {
  if (state == ControlState::FAULT_LATCHED) return false;
  state = ControlState::FAULT_LATCHED;
  makeDue(now);
  return true;
}
bool MotorContext::recordIo(bool success, TimePoint now) {
  if (success) {
    consecutive_io_failures = 0;
    if (!io_error) return false;
    io_error = false;
  } else {
    if (consecutive_io_failures < 2) ++consecutive_io_failures;
    if (io_error || consecutive_io_failures < 2) return false;
    io_error = true;
  }
  ++schedule_revision;
  next_due = now;
  rate_window = now;
  attempts = 0;
  return true;
}
double MotorContext::ioRate(const Config& config) const {
  if (io_error) return config.io_retry_rate;
  return state == ControlState::ACTIVE ? config.active_io_rate : config.idle_io_rate;
}
double MotorContext::pubRate(const Config& config) const {
  return state == ControlState::ACTIVE ? config.active_state_pub_rate : config.idle_state_pub_rate;
}
WireCommand MotorContext::wireCommand(const Config& config) const {
  if (state == ControlState::ACTIVE) return toWire(config.motor_type, latest);
  Command zero;
  zero.id = id;
  if (state == ControlState::CMD_TIMED_OUT && config.timeout_action != TimeoutAction::BRAKE) {
    if (config.timeout_action == TimeoutAction::DAMPING) zero.kd = config.timeout_damping_kd;
    return toWire(config.motor_type, zero);
  }
  return toWire(config.motor_type, zero, Mode::BRAKE);
}
bool MotorContext::shouldPublish(const Config& config, TimePoint now, uint64_t stamp_ns) {
  if (has_published && stamp_ns == last_published_stamp) return false;
  const double target = pubRate(config);
  if (!has_published || target >= ioRate(config) || now - last_published >= period(target)) {
    has_published = true;
    last_published = now;
    last_published_stamp = stamp_ns;
    return true;
  }
  return false;
}
}  // namespace unitree_driver
