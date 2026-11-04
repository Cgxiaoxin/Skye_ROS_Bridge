#pragma once

#include <array>
#include <string>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "skye_robot_driver/driver_core.hpp"

namespace skye_robot_driver {

inline sensor_msgs::msg::JointState make_arm_joint_state(
    const rclcpp::Time &stamp, const std::array<std::string, 7> &names,
    const DriverCore::JointArray &position,
    const DriverCore::JointArray &velocity,
    const DriverCore::JointArray *sensor_torque = nullptr) {
  sensor_msgs::msg::JointState message;
  message.header.stamp = stamp;
  message.name.assign(names.begin(), names.end());
  message.position.assign(position.begin(), position.end());
  message.velocity.assign(velocity.begin(), velocity.end());
  if (sensor_torque) {
    message.effort.assign(sensor_torque->begin(), sensor_torque->end());
  }
  return message;
}

inline sensor_msgs::msg::JointState make_external_torque_state(
    const rclcpp::Time &stamp, const std::array<std::string, 7> &names,
    const DriverCore::JointArray &external_torque) {
  sensor_msgs::msg::JointState message;
  message.header.stamp = stamp;
  message.name.assign(names.begin(), names.end());
  message.effort.assign(external_torque.begin(), external_torque.end());
  return message;
}

inline geometry_msgs::msg::WrenchStamped make_flange_wrench(
    const rclcpp::Time &stamp, const std::string &frame_id,
    const DriverCore::WrenchArray &values) {
  geometry_msgs::msg::WrenchStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  // Passthrough convention; SDK headers do not specify axes or units.
  message.wrench.force.x = values[0];
  message.wrench.force.y = values[1];
  message.wrench.force.z = values[2];
  message.wrench.torque.x = values[3];
  message.wrench.torque.y = values[4];
  message.wrench.torque.z = values[5];
  return message;
}

}  // namespace skye_robot_driver
