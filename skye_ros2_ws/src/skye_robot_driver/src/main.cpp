#include <memory>

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "skye_robot_driver/driver_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<skye_robot_driver::DriverNode>();
  // 2 threads: control (joints/state/timeout) + gripper. SDK mutex still
  // serializes FX calls; gripper I/O must use a short terminal timeout.
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 2U);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
