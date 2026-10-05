// SPDX-FileCopyrightText: Copyright katayama-ki
// SPDX-License-Identifier: MIT

#ifndef UNITREE_MOTOR_ROS_DRIVER__SDK_TRANSPORT_HPP_
#define UNITREE_MOTOR_ROS_DRIVER__SDK_TRANSPORT_HPP_

#include <memory>
#include <string>

#include "unitree_motor_ros_driver/bus_driver.hpp"

namespace unitree_driver
{

// No ROS headers: the ROS adapter injects ros::Time at the receive boundary.
std::unique_ptr<Transport> makeSdkTransport(
  const std::string & port, MotorType type, std::function<uint64_t()> receive_stamp);

}  // namespace unitree_driver

#endif  // UNITREE_MOTOR_ROS_DRIVER__SDK_TRANSPORT_HPP_
