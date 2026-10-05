#pragma once

#include "unitree_motor_ros_driver/bus_driver.hpp"

namespace unitree_driver {

// No ROS headers: the ROS adapter injects ros::Time at the receive boundary.
std::unique_ptr<Transport> makeSdkTransport(const std::string& port, MotorType type,
                                           std::function<uint64_t()> receive_stamp);

}  // namespace unitree_driver
