#include "unitree_motor_ros_driver/sdk_transport.hpp"

#include <serialPort/SerialPort.h>
#include <unitreeMotor/unitreeMotor.h>

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace unitree_driver {
namespace {
::MotorType sdkType(MotorType type) {
  switch (type) {
    case MotorType::A1: return ::MotorType::A1;
    case MotorType::B1: return ::MotorType::B1;
    case MotorType::GO_M8010_6: return ::MotorType::GO_M8010_6;
  }
  throw std::invalid_argument("Unknown motor type");
}
::MotorMode sdkMode(Mode mode) {
  switch (mode) {
    case Mode::BRAKE: return ::MotorMode::BRAKE;
    case Mode::FOC: return ::MotorMode::FOC;
    case Mode::CALIBRATE: return ::MotorMode::CALIBRATE;
    default: throw std::invalid_argument("Cannot send UNKNOWN mode");
  }
}
class SdkTransport : public Transport {
 public:
  SdkTransport(const std::string& port, MotorType type, std::function<uint64_t()> stamp)
    : type_(sdkType(type)), stamp_(std::move(stamp)), serial_(port) {
    if (std::abs(queryGearRatio(type_) - profile(type).sdk_gear_ratio) > 1e-5)
      throw std::runtime_error("Bundled SDK gear ratio does not match motor profile");
  }
  bool transact(const WireCommand& command, Feedback& feedback) override {
    static_assert(std::is_trivially_copyable<MotorCmd>::value, "SDK command layout changed");
    static_assert(std::is_trivially_copyable<MotorData>::value, "SDK feedback layout changed");
    MotorCmd cmd;
    MotorData data;
    // SDK constructors leave fields/private packet storage uninitialized. Zero all
    // reserved bytes as well as public fields before modify_data builds a packet.
    std::memset(static_cast<void*>(&cmd), 0, sizeof(cmd));
    std::memset(static_cast<void*>(&data), 0, sizeof(data));
    cmd.motorType = type_;
    data.motorType = type_;
    cmd.id = command.values.id;
    cmd.mode = queryMotorMode(type_, sdkMode(command.mode));
    cmd.q = command.values.position;
    cmd.dq = command.values.velocity;
    cmd.tau = command.values.torque;
    cmd.kp = command.values.kp;
    cmd.kd = command.values.kd;
    if (!serial_.sendRecv(&cmd, &data)) return false;
    const uint64_t stamp = stamp_();
    feedback.id = data.motor_id;
    feedback.mode = Mode::UNKNOWN;
    for (const auto mode : {Mode::BRAKE, Mode::FOC, Mode::CALIBRATE}) {
      if (data.mode == queryMotorMode(type_, sdkMode(mode))) feedback.mode = mode;
    }
    feedback.position = data.q;
    feedback.velocity = data.dq;
    feedback.torque = data.tau;
    feedback.temperature = static_cast<int8_t>(data.temp);
    feedback.error = static_cast<uint8_t>(data.merror);
    feedback.stamp_ns = stamp;
    return true;
  }
 private:
  ::MotorType type_;
  std::function<uint64_t()> stamp_;
  SerialPort serial_;
};
}  // namespace

std::unique_ptr<Transport> makeSdkTransport(const std::string& port, MotorType type,
                                           std::function<uint64_t()> receive_stamp) {
  return std::unique_ptr<Transport>(new SdkTransport(port, type, std::move(receive_stamp)));
}
}  // namespace unitree_driver
