#include "unitree_motor_ros_driver/MotorCommand.h"
#include "unitree_motor_ros_driver/MotorState.h"
#include "unitree_motor_ros_driver/sdk_transport.hpp"

#include <ros/ros.h>
#include <XmlRpcValue.h>

#include <cstdio>
#include <stdexcept>
#include <string>

namespace {
template <typename T>
void parameter(const ros::NodeHandle& node, const std::string& name, T& value) {
  if (node.hasParam(name) && !node.getParam(name, value))
    throw std::invalid_argument("Wrong parameter type: " + name);
}
// ros::getParam(double) may coerce booleans; reject them and nonnumeric XML-RPC types.
void parameter(const ros::NodeHandle& node, const std::string& name, double& value) {
  if (!node.hasParam(name)) return;
  XmlRpc::XmlRpcValue raw;
  if (!node.getParam(name, raw) ||
      (raw.getType() != XmlRpc::XmlRpcValue::TypeInt &&
       raw.getType() != XmlRpc::XmlRpcValue::TypeDouble))
    throw std::invalid_argument("Wrong parameter type: " + name);
  value = raw.getType() == XmlRpc::XmlRpcValue::TypeInt
    ? static_cast<int>(raw) : static_cast<double>(raw);
}
unitree_driver::Config readConfig(const ros::NodeHandle& node) {
  unitree_driver::Config c;
  std::string type;
  std::string action = "zero_torque";
  if (!node.getParam("serial_port", c.serial_port) || !node.getParam("motor_type", type))
    throw std::invalid_argument("serial_port and motor_type are required private parameters");
  c.motor_type = unitree_driver::parseMotorType(type);
  parameter(node, "motor_id_scan_max", c.motor_id_scan_max);
  parameter(node, "command_timeout", c.command_timeout);
  parameter(node, "active_io_rate", c.active_io_rate);
  parameter(node, "idle_io_rate", c.idle_io_rate);
  parameter(node, "io_rate_warn_ratio", c.io_rate_warn_ratio);
  parameter(node, "io_retry_rate", c.io_retry_rate);
  parameter(node, "active_state_pub_rate", c.active_state_pub_rate);
  parameter(node, "idle_state_pub_rate", c.idle_state_pub_rate);
  parameter(node, "timeout_action", action);
  c.timeout_action = unitree_driver::parseTimeoutAction(action);
  parameter(node, "timeout_damping_kd", c.timeout_damping_kd);
  parameter(node, "velocity_limit", c.velocity_limit);
  parameter(node, "torque_limit", c.torque_limit);
  parameter(node, "kp_limit", c.kp_limit);
  parameter(node, "kd_limit", c.kd_limit);
  parameter(node, "warn_temperature", c.warn_temperature);
  parameter(node, "stop_temperature", c.stop_temperature);
  parameter(node, "stop_on_motor_error", c.stop_on_motor_error);
  return c;
}
void logRos(unitree_driver::Severity severity, const std::string& text) {
  // rosconsole is torn down by SIGINT/rosnode kill before the bus sends its final
  // BRAKEs. Keep their acknowledgments/failures visible after ROS shutdown.
  if (ros::isShuttingDown() || !ros::ok()) {
    const char* level = severity == unitree_driver::Severity::ERROR ? "ERROR" :
      severity == unitree_driver::Severity::WARN ? "WARN" : "INFO";
    std::fprintf(stderr, "[%s] %s\n", level, text.c_str());
    return;
  }
  switch (severity) {
    case unitree_driver::Severity::INFO: ROS_INFO_STREAM(text); break;
    case unitree_driver::Severity::WARN: ROS_WARN_STREAM(text); break;
    case unitree_driver::Severity::ERROR: ROS_ERROR_STREAM(text); break;
  }
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "unitree_motor_driver");
  ros::NodeHandle node("~");
  try {
    const auto config = readConfig(node);
    auto publisher = node.advertise<unitree_motor_ros_driver::MotorState>("state", 100);
    unitree_driver::BusDriver driver(
      config,
      [config] {
        return unitree_driver::makeSdkTransport(config.serial_port, config.motor_type,
                                                [] { return ros::Time::now().toNSec(); });
      },
      [&publisher](const unitree_driver::Feedback& f) {
        unitree_motor_ros_driver::MotorState msg;
        msg.header.stamp.fromNSec(f.stamp_ns);
        msg.id = f.id;
        msg.mode = static_cast<uint8_t>(f.mode);
        msg.position = f.position;
        msg.velocity = f.velocity;
        msg.torque = f.torque;
        msg.temperature = f.temperature;
        msg.error = f.error;
        publisher.publish(msg);
      }, logRos);
    driver.start();
    auto subscriber = node.subscribe<unitree_motor_ros_driver::MotorCommand>(
      "command", 100, [&driver](const unitree_motor_ros_driver::MotorCommand::ConstPtr& msg) {
        unitree_driver::Command command;
        command.id = msg->id;
        command.position = msg->position;
        command.velocity = msg->velocity;
        command.torque = msg->torque;
        command.kp = msg->kp;
        command.kd = msg->kd;
        driver.submit(command);
      });
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // Wall time keeps shutdown responsive when /use_sim_time is paused.
    while (ros::ok() && driver.running()) ros::WallDuration(0.05).sleep();
    const bool unexpected_stop = ros::ok();
    subscriber.shutdown();
    spinner.stop();
    driver.stop();
    if (unexpected_stop) {
      ROS_FATAL("Communication thread stopped unexpectedly");
      return 1;
    }
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Unitree motor driver startup failed: " << error.what());
    return 1;
  }
  return 0;
}
