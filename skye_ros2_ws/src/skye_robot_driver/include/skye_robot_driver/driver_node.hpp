#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "skye_robot_driver/driver_core.hpp"

namespace skye_robot_driver {

// Skeleton node: connect params only. Teleop I/O comes next.
class DriverNode : public rclcpp::Node {
 public:
  DriverNode();
  ~DriverNode() override;

 private:
  DriverCore core_;
};

}  // namespace skye_robot_driver
