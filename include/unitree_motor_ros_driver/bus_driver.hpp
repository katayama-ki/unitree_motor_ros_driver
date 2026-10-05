#pragma once

#include "unitree_motor_ros_driver/model.hpp"

#include <condition_variable>
#include <exception>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <thread>

namespace unitree_driver {

class Transport {
 public:
  virtual ~Transport() = default;
  virtual bool transact(const WireCommand& command, Feedback& rotor_feedback) = 0;
};

// Callbacks must not re-enter BusDriver. Logging may run with the context mutex held.
// The factory, all transactions, and transport destruction run on one bus thread.
class BusDriver {
 public:
  using Factory = std::function<std::unique_ptr<Transport>()>;
  using Publish = std::function<void(const Feedback&)>;
  using Log = std::function<void(Severity, const std::string&)>;

  BusDriver(Config config, Factory factory, Publish publish, Log log);
  ~BusDriver();
  void start();  // Waits for the startup scan; throws if no motor was detected.
  void stop();
  bool submit(const Command& command);
  bool running() const;
  std::vector<uint8_t> ids() const;

 private:
  void worker();
  void scan(Transport& transport);
  void loop(Transport& transport);
  void shutdown(Transport& transport);
  bool transact(Transport& transport, const WireCommand& command, Feedback& feedback);
  void inspectFeedback(MotorContext& motor, const Feedback& feedback, TimePoint now);
  void logMotor(Severity severity, uint8_t id, const std::string& text);
  void throttled(Severity severity, uint8_t id, const std::string& kind,
                 const std::string& text, TimePoint now);
  void checkRate(MotorContext& motor, TimePoint now);

  Config config_;
  Factory factory_;
  Publish publish_;
  Log log_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::map<uint8_t, MotorContext> motors_;
  std::map<std::pair<uint8_t, std::string>, TimePoint> throttle_;
  std::thread thread_;
  bool stopping_ = false;
  bool ready_ = false;
  bool running_ = false;
  bool started_ = false;
  std::exception_ptr startup_error_;
};

}  // namespace unitree_driver
