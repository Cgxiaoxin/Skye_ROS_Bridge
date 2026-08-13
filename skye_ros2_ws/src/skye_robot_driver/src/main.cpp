#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "skye_robot_driver/driver_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<skye_robot_driver::DriverNode>());
  rclcpp::shutdown();
  return 0;
}
