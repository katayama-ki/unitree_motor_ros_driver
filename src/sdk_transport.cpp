#include "unitree_motor_ros_driver/sdk_transport.hpp"

#include <serialPort/SerialPort.h>
#include <unitreeMotor/unitreeMotor.h>

#include <fcntl.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
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
// The SDK's fd is private, so exclusivity is applied through a second fd on the
// same tty. flock is kept alongside TIOCEXCL because root bypasses TIOCEXCL.
class PortLock {
 public:
  explicit PortLock(const std::string& port)
    : fd_(::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC)) {
    if (fd_ < 0) fail("Cannot open serial port " + port);
    if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      const int error = errno;
      ::close(fd_);
      if (error == EWOULDBLOCK)
        throw std::runtime_error("Serial port " + port + " is already in use");
      errno = error;
      fail("Cannot lock serial port " + port);
    }
  }
  ~PortLock() {
    // The flag survives while any other fd keeps the tty open.
    if (exclusive_) ::ioctl(fd_, TIOCNXCL);
    ::close(fd_);
  }
  PortLock(const PortLock&) = delete;
  PortLock& operator=(const PortLock&) = delete;
  void makeExclusive(const std::string& port) {
    if (::ioctl(fd_, TIOCEXCL) != 0) fail("Cannot set exclusive mode on serial port " + port);
    exclusive_ = true;
  }
 private:
  [[noreturn]] static void fail(const std::string& what) {
    throw std::runtime_error(what + ": " + std::strerror(errno));
  }
  int fd_;
  bool exclusive_ = false;
};
class SdkTransport : public Transport {
 public:
  // Keep SerialPort's constructor defaults: the bundled SDK's typed sendRecv
  // overload reconfigures baud rate and receive length according to the motor type.
  SdkTransport(const std::string& port, MotorType type, std::function<uint64_t()> stamp)
    : type_(sdkType(type)), stamp_(std::move(stamp)), lock_(port), serial_(port) {
    lock_.makeExclusive(port);  // Only after serial_ opens: TIOCEXCL would reject the SDK too.
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
  PortLock lock_;  // Before serial_ so the lock is held for the SDK fd's whole lifetime.
  SerialPort serial_;
};
}  // namespace

std::unique_ptr<Transport> makeSdkTransport(const std::string& port, MotorType type,
                                           std::function<uint64_t()> receive_stamp) {
  return std::unique_ptr<Transport>(new SdkTransport(port, type, std::move(receive_stamp)));
}
}  // namespace unitree_driver
