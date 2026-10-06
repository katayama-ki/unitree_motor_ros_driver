#include "unitree_motor_ros_driver/bus_driver.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace unitree_driver {
BusDriver::BusDriver(Config config, Factory factory, Publish publish, Log log)
  : config_(std::move(config)), factory_(std::move(factory)),
    publish_(std::move(publish)), log_(std::move(log)) {
  for (const auto& warning : config_.normalize()) log_(Severity::WARN, warning);
}
BusDriver::~BusDriver() { stop(); }

void BusDriver::start() {
  std::unique_lock<std::mutex> lock(mutex_);
  if (started_) throw std::logic_error("BusDriver can only be started once");
  started_ = true;
  thread_ = std::thread(&BusDriver::worker, this);
  cv_.wait(lock, [this] { return ready_; });
  if (startup_error_) {
    const auto error = startup_error_;
    lock.unlock();
    stop();
    std::rethrow_exception(error);
  }
}
void BusDriver::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}
bool BusDriver::running() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return running_ && !stopping_;
}
std::vector<uint8_t> BusDriver::ids() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> result;
  for (const auto& motor : motors_) result.push_back(motor.first);
  return result;
}
bool BusDriver::submit(const Command& command) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!running_ || stopping_) return false;
  const auto now = Clock::now();
  std::string reason;
  auto found = motors_.find(command.id);
  if (!validateCommand(config_, command, reason) || found == motors_.end()) {
    if (reason.empty()) reason = "ID was not detected during startup scan";
    throttled(Severity::WARN, command.id, "invalid_command", "ignored command: " + reason, now);
    return false;
  }
  found->second.acceptCommand(command, now);
  cv_.notify_one();
  return true;
}
void BusDriver::logMotor(Severity severity, uint8_t id, const std::string& text) {
  log_(severity, "Motor id " + std::to_string(id) + " " + text);
}
void BusDriver::throttled(Severity severity, uint8_t id, const std::string& kind,
                          const std::string& text, TimePoint now) {
  const auto key = std::make_pair(id, kind);
  const auto found = throttle_.find(key);
  if (found == throttle_.end() || now - found->second >= std::chrono::seconds(2)) {
    throttle_[key] = now;
    logMotor(severity, id, text);
  }
}
bool BusDriver::transact(Transport& transport, const WireCommand& command, Feedback& feedback,
                         bool warn_on_send_recv_failure) {
  bool success = false;
  std::string failure;
  try {
    success = transport.transact(command, feedback);
    if (success && feedback.id != command.values.id) {
      success = false;
      failure = "received mismatched ID " + std::to_string(feedback.id);
    }
  } catch (const std::exception& error) {
    failure = std::string("transaction exception: ") + error.what();
  }
  if (!success && (warn_on_send_recv_failure || !failure.empty())) {
    std::lock_guard<std::mutex> lock(mutex_);
    throttled(Severity::WARN, command.values.id, "io_failure",
              failure.empty() ? "sendRecv failed" : failure, Clock::now());
  }
  return success;
}
void BusDriver::inspectFeedback(MotorContext& motor, const Feedback& f, TimePoint now) {
  if (f.mode == Mode::UNKNOWN)
    throttled(Severity::WARN, motor.id, "unknown_mode", "reported UNKNOWN mode", now);
  if (std::abs(f.velocity) > config_.velocity_limit)
    throttled(Severity::WARN, motor.id, "velocity_limit", "feedback velocity exceeds limit", now);
  if (std::abs(f.torque) > config_.torque_limit)
    throttled(Severity::WARN, motor.id, "torque_limit", "feedback torque exceeds limit", now);
  if (f.temperature > config_.warn_temperature)
    throttled(Severity::WARN, motor.id, "temperature",
              "temperature " + std::to_string(f.temperature) + " exceeds warning threshold", now);
  if (f.error != motor.previous_error) {
    if (f.error == 0) logMotor(Severity::INFO, motor.id, "error code returned to normal (0)");
    else logMotor(Severity::ERROR, motor.id, "reported error code " + std::to_string(f.error) +
                  " (" + motorErrorText(config_.motor_type, f.error) + ")");
    motor.previous_error = f.error;
  }
  const bool hot = f.temperature > config_.stop_temperature;
  if ((hot || (config_.stop_on_motor_error && f.error != 0)) && motor.latchFault(now)) {
    logMotor(Severity::ERROR, motor.id,
      hot ? "stop temperature exceeded; FAULT_LATCHED, sending BRAKE until restart"
          : "motor error; FAULT_LATCHED, sending BRAKE until restart");
  }
}
void BusDriver::scan(Transport& transport) {
  for (const uint8_t id : scanIds(config_.motor_type, config_.motor_id_scan_max)) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_) throw std::runtime_error("Stopped during startup scan");
    }
    Command zero;
    zero.id = id;
    const auto brake = toWire(config_.motor_type, zero, Mode::BRAKE);
    Feedback rotor;
    // A failed sendRecv is expected for an absent ID during discovery. Keep
    // mismatched replies and transaction exceptions visible even during the scan.
    if (!transact(transport, brake, rotor, false) &&
        !transact(transport, brake, rotor, false)) continue;
    const auto feedback = toOutput(config_.motor_type, rotor);
    const auto now = Clock::now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto& motor = motors_[id];
      motor.id = id;
      motor.latest.id = id;
      motor.makeDue(now);
      inspectFeedback(motor, feedback, now);
      motor.shouldPublish(config_, now, feedback.stamp_ns);  // Called only to record this scan feedback as published.
    }
    publish_(feedback);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (motors_.empty()) throw std::runtime_error("No motors detected on serial bus");
  std::ostringstream text;
  text << "Detected motor IDs:";
  for (const auto& motor : motors_) text << " " << static_cast<unsigned>(motor.first);
  log_(Severity::INFO, text.str());
  // Start regular scheduling after the entire scan, without accounting scan delays
  // as missed runtime IO. Preserve faults detected during scanning.
  for (auto& motor : motors_) motor.second.makeDue(Clock::now());
}
void BusDriver::checkRate(MotorContext& motor, TimePoint now) {
  const double elapsed = std::chrono::duration<double>(now - motor.rate_window).count();
  if (elapsed < 1.0) return;
  if (!motor.io_error) {
    const double actual = motor.attempts / elapsed;
    const double target = motor.ioRate(config_);
    if (actual < config_.io_rate_warn_ratio * target) {
      std::ostringstream text;
      text << "IO attempt rate " << actual << " Hz is below target " << target << " Hz";
      throttled(Severity::WARN, motor.id, "io_rate", text.str(), now);
    }
  }
  motor.rate_window = now;
  motor.attempts = 0;
}
void BusDriver::loop(Transport& transport) {
  std::unique_lock<std::mutex> lock(mutex_);
  while (!stopping_) {
    const auto now = Clock::now();
    auto selected = motors_.end();
    TimePoint wake = TimePoint::max();
    for (auto it = motors_.begin(); it != motors_.end(); ++it) {
      auto& motor = it->second;
      if (motor.expire(config_, now))
        logMotor(Severity::WARN, motor.id, "command watchdog expired");
      checkRate(motor, now);
      if (motor.state == ControlState::ACTIVE) {
        const auto expiry = motor.command_received +
          std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(config_.command_timeout));
        if (expiry < wake) wake = expiry;
      }
      if (motor.ioRate(config_) == 0) continue;
      if (motor.next_due < wake) wake = motor.next_due;
      if (!motor.io_error && motor.rate_window + std::chrono::seconds(1) < wake)
        wake = motor.rate_window + std::chrono::seconds(1);
      if (motor.next_due <= now &&
          (selected == motors_.end() || motor.next_due < selected->second.next_due)) selected = it;
    }
    if (selected == motors_.end()) {
      if (wake == TimePoint::max()) cv_.wait(lock);
      else cv_.wait_until(lock, wake);
      continue;
    }
    auto& motor = selected->second;
    const auto command = motor.wireCommand(config_);
    const auto deadline = motor.next_due;
    const auto revision = motor.schedule_revision;
    ++motor.attempts;  // Count attempts, including failed/mismatched transactions.
    lock.unlock();
    Feedback rotor;
    const bool success = transact(transport, command, rotor);
    const auto completed = Clock::now();
    lock.lock();
    const bool rate_changed = motor.recordIo(success, completed);
    if (rate_changed) {
      logMotor(motor.io_error ? Severity::ERROR : Severity::INFO, motor.id,
        motor.io_error ? "two consecutive IO failures; io_error set" : "IO recovered; io_error cleared");
      if (motor.io_error && config_.io_retry_rate > 0)
        motor.next_due = completed + period(config_.io_retry_rate);
    }
    Feedback output;
    bool publish = false;
    if (motor.expire(config_, completed))
      logMotor(Severity::WARN, motor.id, "command watchdog expired");
    if (success) {
      output = toOutput(config_.motor_type, rotor);
      inspectFeedback(motor, output, completed);
      publish = motor.shouldPublish(config_, completed, output.stamp_ns);
    }
    // A command/fault transition during sendRecv must retain its immediate deadline.
    // A regular late transaction drops missed cycles and moves to a future deadline.
    if (motor.schedule_revision == revision && motor.ioRate(config_) > 0)
      motor.next_due = advanceDeadline(deadline, period(motor.ioRate(config_)), completed);
    lock.unlock();
    if (publish) publish_(output);
    lock.lock();
  }
}
void BusDriver::shutdown(Transport& transport) {
  for (const auto id : ids()) {
    Command zero;
    zero.id = id;
    Feedback rotor;
    const bool ok = transact(transport, toWire(config_.motor_type, zero, Mode::BRAKE), rotor);
    std::lock_guard<std::mutex> lock(mutex_);
    if (ok) {
      inspectFeedback(motors_.at(id), toOutput(config_.motor_type, rotor), Clock::now());
      logMotor(Severity::INFO, id, "shutdown BRAKE acknowledged");
    } else {
      logMotor(Severity::ERROR, id, "shutdown BRAKE failed or received mismatched reply");
    }
  }
}
void BusDriver::worker() {
  std::unique_ptr<Transport> transport;
  try {
    transport = factory_();
    if (!transport) throw std::runtime_error("Transport factory returned null");
    scan(*transport);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      running_ = true;
      ready_ = true;
    }
    cv_.notify_all();
    loop(*transport);
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ready_) startup_error_ = std::current_exception();
    else log_(Severity::ERROR, "Communication thread terminated unexpectedly");
    ready_ = true;
    running_ = false;
    cv_.notify_all();
  }
  if (transport) {
    try { shutdown(*transport); }
    catch (const std::exception& error) {
      log_(Severity::ERROR, std::string("Shutdown error: ") + error.what());
    }
    transport.reset();  // SerialPort destructor closes the port on the bus thread.
    log_(Severity::INFO, "Serial bus closed");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  running_ = false;
}
}  // namespace unitree_driver
