#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "gento_robot_driver/gento_robot_driver_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
        std::make_shared<gento_robot_driver::GentoRobotDriverNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("gento_robot_driver"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
