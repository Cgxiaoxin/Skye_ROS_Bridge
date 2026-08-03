#include "skye_robot_driver/driver_node.hpp"

#include <cstdio>

namespace skye_robot_driver {

DriverNode::DriverNode() : Node("skye_robot_driver") {
  declare_parameter<std::string>("robot_ip", "6.6.7.190");
  const auto ip_str = get_parameter("robot_ip").as_string();

  std::array<unsigned char, 4> ip{6, 6, 7, 190};
  unsigned int a = 0, b = 0, c = 0, d = 0;
  if (sscanf(ip_str.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
    ip = {static_cast<unsigned char>(a), static_cast<unsigned char>(b),
          static_cast<unsigned char>(c), static_cast<unsigned char>(d)};
  }

  if (!core_.connect(ip)) {
    RCLCPP_ERROR(get_logger(), "FX_L1_System_Link failed (%s)", ip_str.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "Linked to controller %s", ip_str.c_str());
  }
}

DriverNode::~DriverNode() { core_.shutdown(); }

}  // namespace skye_robot_driver
