#include "unitree_motor_ros_driver/bus_driver.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <future>
#include <limits>
#include <set>

namespace ud = unitree_driver;
namespace {
ud::Config config(ud::MotorType type = ud::MotorType::GO_M8010_6) {
  ud::Config c;
  c.serial_port = "fake";
  c.motor_type = type;
  c.motor_id_scan_max = 1;
  return c;
}
ud::TimePoint at(double seconds) {
  return ud::TimePoint(std::chrono::duration_cast<ud::Clock::duration>(
    std::chrono::duration<double>(seconds)));
}
TEST(Configuration, HardwareLimitsAndScanBroadcastExclusion) {
  for (const auto type : {ud::MotorType::A1, ud::MotorType::B1, ud::MotorType::GO_M8010_6}) {
    auto c = config(type);
    c.velocity_limit = 1000;
    c.torque_limit = 1000;
    EXPECT_EQ(2u, c.normalize().size());
    EXPECT_DOUBLE_EQ(ud::profile(type).velocity_limit, c.velocity_limit);
    EXPECT_DOUBLE_EQ(ud::profile(type).torque_limit, c.torque_limit);
  }
  const auto a1 = ud::scanIds(ud::MotorType::A1, 255);
  EXPECT_EQ(255u, a1.size());
  EXPECT_EQ(a1.end(), std::find(a1.begin(), a1.end(), 0xBB));
  EXPECT_NE(a1.end(), std::find(a1.begin(), a1.end(), 255));
  const auto go = ud::scanIds(ud::MotorType::GO_M8010_6, 255);
  ASSERT_EQ(15u, go.size());
  EXPECT_EQ(14, go.back());
  auto c = config();
  c.motor_id_scan_max = 1000;
  EXPECT_EQ(1u, c.normalize().size());
  EXPECT_EQ(14, c.motor_id_scan_max);
}
TEST(Configuration, InvalidParametersFailBeforeOpeningBus) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (double ud::Config::* field : {&ud::Config::velocity_limit, &ud::Config::torque_limit,
      &ud::Config::kp_limit, &ud::Config::kd_limit, &ud::Config::timeout_damping_kd}) {
    for (const double value : {nan, inf, -1.0}) {
      auto c = config();
      c.*field = value;
      EXPECT_THROW(c.normalize(), std::invalid_argument);
    }
  }
  for (double ud::Config::* field : {&ud::Config::command_timeout, &ud::Config::active_io_rate,
      &ud::Config::idle_io_rate, &ud::Config::active_state_pub_rate,
      &ud::Config::idle_state_pub_rate}) {
    for (const double value : {nan, inf, -1.0, 0.0, 1e-300}) {
      auto c = config();
      c.*field = value;
      EXPECT_THROW(c.normalize(), std::invalid_argument);
    }
  }
  auto c = config();
  c.timeout_action = ud::TimeoutAction::DAMPING;
  c.timeout_damping_kd = 0;
  EXPECT_THROW(c.normalize(), std::invalid_argument);
  c.timeout_damping_kd = 11;
  EXPECT_THROW(c.normalize(), std::invalid_argument);
  c = config(ud::MotorType::A1);
  c.motor_id_scan_max = 256;
  EXPECT_THROW(c.normalize(), std::invalid_argument);
  EXPECT_THROW(ud::parseMotorType("Go"), std::invalid_argument);
  EXPECT_THROW(ud::parseTimeoutAction("coast"), std::invalid_argument);
}
TEST(Conversion, OutputRotorRoundTripAndSpecialGains) {
  for (const auto type : {ud::MotorType::A1, ud::MotorType::B1, ud::MotorType::GO_M8010_6}) {
    ud::Command cmd;
    cmd.id = 1;
    cmd.position = -1.5;
    cmd.velocity = 2;
    cmd.torque = -3;
    cmd.kp = 25;
    cmd.kd = 0.6;
    const auto wire = ud::toWire(type, cmd);
    const double r = ud::profile(type).sdk_gear_ratio;
    EXPECT_NEAR(cmd.kp / (r*r) / (type == ud::MotorType::GO_M8010_6 ? 1 : 26.07),
                wire.values.kp, 1e-6);
    EXPECT_NEAR(cmd.kd / (r*r) * (type == ud::MotorType::GO_M8010_6 ? 1 : 100),
                wire.values.kd, 1e-6);
    ud::Feedback feedback;
    feedback.position = wire.values.position;
    feedback.velocity = wire.values.velocity;
    feedback.torque = wire.values.torque;
    const auto output = ud::toOutput(type, feedback);
    EXPECT_NEAR(cmd.position, output.position, 1e-6);
    EXPECT_NEAR(cmd.velocity, output.velocity, 1e-6);
    EXPECT_NEAR(cmd.torque, output.torque, 1e-6);
    const auto brake = ud::toWire(type, cmd, ud::Mode::BRAKE);
    EXPECT_EQ(1, brake.values.id);
    EXPECT_EQ(0, brake.values.position);
    EXPECT_EQ(0, brake.values.velocity);
    EXPECT_EQ(0, brake.values.torque);
    EXPECT_EQ(0, brake.values.kp);
    EXPECT_EQ(0, brake.values.kd);
  }
}
TEST(CommandValidation, RejectsNonfiniteLimitsAndProtocolOverflow) {
  auto c = config();
  c.normalize();
  ud::Command cmd;
  std::string reason;
  EXPECT_TRUE(ud::validateCommand(c, cmd, reason));
  for (float ud::Command::* field : {&ud::Command::position, &ud::Command::velocity,
      &ud::Command::torque, &ud::Command::kp, &ud::Command::kd}) {
    for (const float value : {std::numeric_limits<float>::quiet_NaN(),
                             std::numeric_limits<float>::infinity()}) {
      auto bad = cmd;
      bad.*field = value;
      EXPECT_FALSE(ud::validateCommand(c, bad, reason));
    }
  }
  cmd.velocity = -30;
  cmd.torque = -23;
  EXPECT_TRUE(ud::validateCommand(c, cmd, reason));
  cmd.velocity = -31;
  EXPECT_FALSE(ud::validateCommand(c, cmd, reason));
  cmd = ud::Command();
  cmd.position = 100000;
  EXPECT_FALSE(ud::validateCommand(c, cmd, reason));
  cmd.position = 0;
  cmd.id = 15;
  EXPECT_FALSE(ud::validateCommand(c, cmd, reason));
  c = config(ud::MotorType::A1);
  c.kd_limit = 100;
  c.normalize();
  cmd.id = 0;
  cmd.kd = 27;
  EXPECT_FALSE(ud::validateCommand(c, cmd, reason));
  EXPECT_NE(std::string::npos, reason.find("protocol"));
  c = config(ud::MotorType::B1);
  c.kd_limit = 100;
  c.normalize();
  EXPECT_TRUE(ud::validateCommand(c, cmd, reason));  // B1 x512, not A1 x1024.
  cmd.kp = -1;
  EXPECT_FALSE(ud::validateCommand(c, cmd, reason));
}
TEST(ControlState, WatchdogTimeoutActionsRecoveryAndLatchedFault) {
  auto c = config();
  c.normalize();
  ud::MotorContext motor;
  motor.id = 1;
  EXPECT_EQ(ud::Mode::BRAKE, motor.wireCommand(c).mode);
  EXPECT_FALSE(motor.expire(c, at(10)));
  ud::Command cmd;
  cmd.id = 1;
  cmd.torque = 2;
  EXPECT_TRUE(motor.acceptCommand(cmd, at(10)));
  EXPECT_FALSE(motor.expire(c, at(10.49)));
  EXPECT_TRUE(motor.expire(c, at(10.5)));
  EXPECT_EQ(ud::ControlState::CMD_TIMED_OUT, motor.state);
  EXPECT_EQ(c.idle_io_rate, motor.ioRate(c));
  auto wire = motor.wireCommand(c);
  EXPECT_EQ(ud::Mode::FOC, wire.mode);
  EXPECT_EQ(0, wire.values.torque);
  EXPECT_EQ(0, wire.values.kd);
  c.timeout_action = ud::TimeoutAction::DAMPING;
  EXPECT_GT(motor.wireCommand(c).values.kd, 0);
  c.timeout_action = ud::TimeoutAction::BRAKE;
  EXPECT_EQ(ud::Mode::BRAKE, motor.wireCommand(c).mode);
  EXPECT_TRUE(motor.acceptCommand(cmd, at(11)));
  EXPECT_TRUE(motor.latchFault(at(11.1)));
  cmd.torque = 3;
  EXPECT_FALSE(motor.acceptCommand(cmd, at(12)));
  EXPECT_EQ(3, motor.latest.torque);
  EXPECT_FALSE(motor.expire(c, at(20)));
  EXPECT_EQ(ud::ControlState::FAULT_LATCHED, motor.state);
  EXPECT_EQ(ud::Mode::BRAKE, motor.wireCommand(c).mode);
}
TEST(IoState, ConsecutiveFailuresRetryOverrideAndSingleSuccessRecovery) {
  auto c = config();
  c.io_retry_rate = 2;
  c.normalize();
  ud::MotorContext motor;
  EXPECT_FALSE(motor.recordIo(false, at(1)));
  EXPECT_FALSE(motor.recordIo(true, at(2)));
  EXPECT_EQ(0, motor.consecutive_io_failures);
  EXPECT_FALSE(motor.recordIo(false, at(3)));
  EXPECT_TRUE(motor.recordIo(false, at(4)));
  EXPECT_TRUE(motor.io_error);
  EXPECT_EQ(2, motor.ioRate(c));
  ud::Command cmd;
  cmd.torque = 2;
  motor.acceptCommand(cmd, at(4.1));
  EXPECT_EQ(ud::Mode::FOC, motor.wireCommand(c).mode);
  EXPECT_TRUE(motor.expire(c, at(5)));
  EXPECT_EQ(0, motor.wireCommand(c).values.torque);
  EXPECT_TRUE(motor.recordIo(true, at(6)));
  EXPECT_FALSE(motor.io_error);
  EXPECT_EQ(0, motor.consecutive_io_failures);
  EXPECT_EQ(c.idle_io_rate, motor.ioRate(c));
}
TEST(Scheduler, DropsMissedCyclesAndPublishesPerMotor) {
  EXPECT_EQ(at(1.04), ud::advanceDeadline(at(1), ud::period(100), at(1.035)));
  EXPECT_EQ(at(1.05), ud::advanceDeadline(at(1), ud::period(100), at(1.04)));
  auto c = config();
  c.active_state_pub_rate = 10;
  c.normalize();
  ud::MotorContext one, two;
  ud::Command cmd;
  one.acceptCommand(cmd, at(1));
  two.acceptCommand(cmd, at(1));
  EXPECT_TRUE(one.shouldPublish(c, at(1), 1));
  EXPECT_FALSE(one.shouldPublish(c, at(1.05), 2));
  EXPECT_TRUE(two.shouldPublish(c, at(1.05), 2));
  EXPECT_TRUE(one.shouldPublish(c, at(1.1), 3));
  c.active_state_pub_rate = 1000;
  EXPECT_FALSE(one.shouldPublish(c, at(1.11), 3));
  EXPECT_TRUE(one.shouldPublish(c, at(1.11), 4));
}

struct Record {
  ud::WireCommand command;
  ud::TimePoint time;
};
struct FakeBus {
  std::mutex mutex;
  std::condition_variable cv;
  std::map<uint8_t, ud::Feedback> feedback;
  std::map<uint8_t, std::deque<int>> outcomes;  // 0=success, -1=failure, -2=ID mismatch
  std::vector<Record> transactions;
  std::vector<ud::Feedback> published;
  std::vector<std::string> logs;
  std::set<std::thread::id> owners;
  bool hold_foc = false;
  bool held = false;
  uint64_t stamp = 0;
  bool closed = false;

  template <typename Predicate>
  bool wait(Predicate predicate, double timeout = 2) {
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, std::chrono::duration<double>(timeout), predicate);
  }
  bool hasLog(const std::string& text) const {
    for (const auto& log : logs) if (log.find(text) != std::string::npos) return true;
    return false;
  }
  bool hasCommand(uint8_t id, ud::Mode mode) const {
    for (const auto& r : transactions)
      if (r.command.values.id == id && r.command.mode == mode) return true;
    return false;
  }
};
class FakeTransport : public ud::Transport {
 public:
  explicit FakeTransport(std::shared_ptr<FakeBus> bus) : bus_(std::move(bus)) {
    std::lock_guard<std::mutex> lock(bus_->mutex);
    bus_->owners.insert(std::this_thread::get_id());
  }
  ~FakeTransport() override {
    std::lock_guard<std::mutex> lock(bus_->mutex);
    bus_->owners.insert(std::this_thread::get_id());
    bus_->closed = true;
    bus_->cv.notify_all();
  }
  bool transact(const ud::WireCommand& command, ud::Feedback& feedback) override {
    std::unique_lock<std::mutex> lock(bus_->mutex);
    bus_->owners.insert(std::this_thread::get_id());
    bus_->transactions.push_back(Record{command, ud::Clock::now()});
    bus_->cv.notify_all();
    if (command.mode == ud::Mode::FOC && bus_->hold_foc) {
      bus_->held = true;
      bus_->cv.notify_all();
      bus_->cv.wait(lock, [this] { return !bus_->hold_foc; });
    }
    const auto id = command.values.id;
    if (!bus_->feedback.count(id)) return false;
    int outcome = 0;
    auto& outcomes = bus_->outcomes[id];
    if (!outcomes.empty()) { outcome = outcomes.front(); outcomes.pop_front(); }
    if (outcome == -1) return false;
    feedback = bus_->feedback[id];
    feedback.id = outcome == -2 ? 14 : id;
    feedback.stamp_ns = ++bus_->stamp;
    return true;
  }
 private:
  std::shared_ptr<FakeBus> bus_;
};
struct Harness {
  std::shared_ptr<FakeBus> bus{new FakeBus};
  std::unique_ptr<ud::BusDriver> driver;
  explicit Harness(ud::Config c, bool add_motors = true) {
    if (add_motors) {
      bus->feedback[0] = ud::Feedback();
      bus->feedback[1] = ud::Feedback();
      bus->feedback[0].mode = bus->feedback[1].mode = ud::Mode::BRAKE;
    }
    const auto shared = bus;
    driver.reset(new ud::BusDriver(c,
      [shared] { return std::unique_ptr<ud::Transport>(new FakeTransport(shared)); },
      [shared](const ud::Feedback& f) {
        std::lock_guard<std::mutex> lock(shared->mutex);
        shared->published.push_back(f);
        shared->cv.notify_all();
      },
      [shared](ud::Severity, const std::string& text) {
        std::lock_guard<std::mutex> lock(shared->mutex);
        shared->logs.push_back(text);
        shared->cv.notify_all();
      }));
  }
  ~Harness() {
    { std::lock_guard<std::mutex> lock(bus->mutex); bus->hold_foc = false; }
    bus->cv.notify_all();
    driver->stop();
  }
};
TEST(BusDriver, ScanRetriesAndRejectsUnknownIdWithoutFeedingWatchdog) {
  auto c = config();
  c.command_timeout = 0.05;
  c.timeout_action = ud::TimeoutAction::BRAKE;
  Harness h(c);
  h.bus->outcomes[0] = {-2, 0};
  h.driver->start();
  EXPECT_EQ((std::vector<uint8_t>{0, 1}), h.driver->ids());
  ud::Command cmd;
  cmd.id = 2;
  EXPECT_FALSE(h.driver->submit(cmd));
  cmd.id = 0;
  cmd.torque = 2;
  ASSERT_TRUE(h.driver->submit(cmd));
  ASSERT_TRUE(h.bus->wait([&] { return h.bus->hasCommand(0, ud::Mode::FOC); }));
  cmd.torque = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(h.driver->submit(cmd));
  ASSERT_TRUE(h.bus->wait([&] { return h.bus->hasLog("watchdog expired"); }));
  h.driver->stop();
  std::lock_guard<std::mutex> lock(h.bus->mutex);
  EXPECT_EQ(1u, h.bus->owners.size());
  EXPECT_TRUE(h.bus->closed);
  ASSERT_GE(h.bus->transactions.size(), 3u);
  EXPECT_EQ(0, h.bus->transactions[0].command.values.id);
  EXPECT_EQ(0, h.bus->transactions[1].command.values.id);
  EXPECT_TRUE(h.bus->hasLog("mismatched ID"));
  for (size_t i = h.bus->transactions.size() - 2; i < h.bus->transactions.size(); ++i) {
    const auto& wire = h.bus->transactions[i].command;
    EXPECT_EQ(ud::Mode::BRAKE, wire.mode);
    EXPECT_EQ(0, wire.values.torque);
    EXPECT_EQ(0, wire.values.position);
    EXPECT_EQ(0, wire.values.velocity);
    EXPECT_EQ(0, wire.values.kp);
    EXPECT_EQ(0, wire.values.kd);
  }
}
TEST(BusDriver, NoDetectedMotorIsStartupFailure) {
  Harness h(config(), false);
  EXPECT_THROW(h.driver->start(), std::runtime_error);
  EXPECT_FALSE(h.driver->running());
  EXPECT_TRUE(h.driver->ids().empty());
  std::lock_guard<std::mutex> lock(h.bus->mutex);
  EXPECT_EQ(4u, h.bus->transactions.size());
  EXPECT_TRUE(h.bus->published.empty());
  EXPECT_TRUE(h.bus->closed);
}
TEST(BusDriver, ScanFaultLatchesOnlyAffectedMotor) {
  Harness h(config());
  h.bus->feedback[0].error = 2;
  h.driver->start();
  ud::Command cmd;
  cmd.torque = 1;
  EXPECT_TRUE(h.driver->submit(cmd));
  cmd.id = 1;
  EXPECT_TRUE(h.driver->submit(cmd));
  ASSERT_TRUE(h.bus->wait([&] { return h.bus->hasCommand(1, ud::Mode::FOC); }));
  h.driver->stop();
  std::lock_guard<std::mutex> lock(h.bus->mutex);
  EXPECT_FALSE(h.bus->hasCommand(0, ud::Mode::FOC));
  EXPECT_TRUE(h.bus->hasLog("over-current"));
  EXPECT_TRUE(h.bus->hasLog("FAULT_LATCHED"));
}
TEST(BusDriver, IoFailureDisablesRetryAndNeverPublishesMismatchedFeedback) {
  auto c = config();
  c.idle_io_rate = 100;
  c.idle_state_pub_rate = 100;
  Harness h(c);
  h.driver->start();
  {
    std::lock_guard<std::mutex> lock(h.bus->mutex);
    h.bus->outcomes[0] = {-2, -1};
  }
  ASSERT_TRUE(h.bus->wait([&] { return h.bus->hasLog("io_error set"); }));
  size_t count;
  {
    std::lock_guard<std::mutex> lock(h.bus->mutex);
    count = std::count_if(h.bus->transactions.begin(), h.bus->transactions.end(),
      [](const Record& r) { return r.command.values.id == 0; });
  }
  ud::Command cmd;
  cmd.torque = 2;
  EXPECT_TRUE(h.driver->submit(cmd));
  ASSERT_TRUE(h.bus->wait([&] { return h.bus->transactions.size() > count + 5; }));
  {
    std::lock_guard<std::mutex> lock(h.bus->mutex);
    EXPECT_EQ(count, static_cast<size_t>(std::count_if(
      h.bus->transactions.begin(), h.bus->transactions.end(),
      [](const Record& r) { return r.command.values.id == 0; })));
    for (const auto& f : h.bus->published) EXPECT_NE(14, f.id);
  }
}
TEST(BusDriver, RetryUsesFreshCommandAndOneSuccessRecovers) {
  auto c = config();
  c.idle_io_rate = 100;
  c.io_retry_rate = 10;
  Harness h(c);
  h.driver->start();
  {
    std::lock_guard<std::mutex> lock(h.bus->mutex);
    h.bus->outcomes[0] = {-1, -2};
  }
  ASSERT_TRUE(h.bus->wait([&] { return h.bus->hasLog("io_error set"); }));
  ud::Command cmd;
  cmd.torque = 3;
  ASSERT_TRUE(h.driver->submit(cmd));
  ASSERT_TRUE(h.bus->wait([&] { return h.bus->hasLog("IO recovered"); }));
  std::lock_guard<std::mutex> lock(h.bus->mutex);
  EXPECT_TRUE(h.bus->hasCommand(0, ud::Mode::FOC));
}
TEST(BusDriver, CommandSubmissionDoesNotBlockBehindTransactionAndSafetyOverridesIt) {
  auto c = config();
  c.command_timeout = 5;
  c.active_state_pub_rate = 0.1;  // Safety is inspected even when state publication is suppressed.
  Harness h(c);
  h.driver->start();
  {
    std::lock_guard<std::mutex> lock(h.bus->mutex);
    h.bus->hold_foc = true;
  }
  ud::Command cmd;
  cmd.torque = 1;
  ASSERT_TRUE(h.driver->submit(cmd));
  ASSERT_TRUE(h.bus->wait([&] { return h.bus->held; }));
  cmd.torque = 2;
  auto submitted = std::async(std::launch::async, [&] { return h.driver->submit(cmd); });
  EXPECT_EQ(std::future_status::ready, submitted.wait_for(std::chrono::milliseconds(100)));
  {
    std::lock_guard<std::mutex> lock(h.bus->mutex);
    h.bus->feedback[0].temperature = 81;
    h.bus->hold_foc = false;
  }
  h.bus->cv.notify_all();
  EXPECT_TRUE(submitted.get());
  ASSERT_TRUE(h.bus->wait([&] { return h.bus->hasLog("stop temperature exceeded"); }));
  {
    std::lock_guard<std::mutex> lock(h.bus->mutex);
    h.bus->feedback[0].temperature = 20;
    h.bus->transactions.clear();
  }
  EXPECT_TRUE(h.driver->submit(cmd));
  ASSERT_TRUE(h.bus->wait([&] { return h.bus->hasCommand(0, ud::Mode::BRAKE); }));
  std::lock_guard<std::mutex> lock(h.bus->mutex);
  EXPECT_FALSE(h.bus->hasCommand(0, ud::Mode::FOC));
}
TEST(BusDriver, DisabledErrorStopStillLogsErrorEdges) {
  auto c = config();
  c.stop_on_motor_error = false;
  c.active_io_rate = 100;
  Harness h(c);
  h.bus->feedback[0].error = 4;
  h.driver->start();
  ud::Command cmd;
  cmd.torque = 1;
  EXPECT_TRUE(h.driver->submit(cmd));
  ASSERT_TRUE(h.bus->wait([&] { return h.bus->hasCommand(0, ud::Mode::FOC); }));
  {
    std::lock_guard<std::mutex> lock(h.bus->mutex);
    h.bus->feedback[0].error = 0;
  }
  ASSERT_TRUE(h.bus->wait([&] { return h.bus->hasLog("returned to normal"); }));
  std::lock_guard<std::mutex> lock(h.bus->mutex);
  EXPECT_FALSE(h.bus->hasLog("FAULT_LATCHED"));
  EXPECT_TRUE(h.bus->hasLog("encoder fault"));
}
TEST(BusDriver, RateMeasurementCountsAttemptsIncludingAlternatingFailures) {
  auto c = config();
  c.motor_id_scan_max = 0;
  c.idle_io_rate = 100;
  c.idle_state_pub_rate = 100;
  Harness h(c);
  for (int i = 0; i < 200; ++i) {
    h.bus->outcomes[0].push_back(-1);
    h.bus->outcomes[0].push_back(0);
  }
  h.driver->start();
  ASSERT_TRUE(h.bus->wait([&] { return h.bus->transactions.size() >= 125; }));
  std::lock_guard<std::mutex> lock(h.bus->mutex);
  // A successful-reply counter would report ~50 Hz and falsely warn here.
  EXPECT_FALSE(h.bus->hasLog("IO attempt rate"));
  EXPECT_FALSE(h.bus->hasLog("io_error set"));
}
TEST(BusDriver, WarningThrottleIsIndependentForEachMotorAndLogType) {
  auto c = config();
  c.idle_io_rate = 100;
  Harness h(c);
  for (auto& entry : h.bus->feedback) {
    entry.second.mode = ud::Mode::UNKNOWN;
    entry.second.temperature = 71;
    entry.second.velocity = 31 * ud::profile(c.motor_type).sdk_gear_ratio;
    entry.second.torque = 25 / ud::profile(c.motor_type).sdk_gear_ratio;
  }
  h.driver->start();
  ASSERT_TRUE(h.bus->wait([&] { return h.bus->transactions.size() > 30; }));
  h.driver->stop();
  std::lock_guard<std::mutex> lock(h.bus->mutex);
  for (uint8_t id : {0, 1}) {
    for (const auto& text : {"reported UNKNOWN mode", "feedback velocity exceeds limit",
                            "feedback torque exceeds limit", "temperature 71"}) {
      const std::string prefix = "Motor id " + std::to_string(id) + " " + text;
      EXPECT_EQ(1, std::count_if(h.bus->logs.begin(), h.bus->logs.end(),
        [&prefix](const std::string& log) { return log.find(prefix) != std::string::npos; }));
    }
  }
}
}  // namespace
