#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "skye_robot_driver/srv/set_mode.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "skye_robot_driver/driver_core.hpp"

namespace skye_robot_driver {

class DriverNode : public rclcpp::Node {
 public:
  explicit DriverNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~DriverNode() override;

 private:
  using JointState = sensor_msgs::msg::JointState;
  using JointArray = DriverCore::JointArray;
  using Trigger = std_srvs::srv::Trigger;
  using SetMode = skye_robot_driver::srv::SetMode;

  static std::array<unsigned char, 4> parse_ipv4(const std::string &value);
  static JointArray load_joint_array(
      rclcpp::Node &node, const std::string &parameter_name,
      const JointArray &defaults);
  static std::array<int, 7> load_joint_order(
      rclcpp::Node &node, const std::string &parameter_name,
      const std::array<int, 7> &defaults);
  static DriverCore::ControlMode parse_control_mode(const std::string &value);

  void handle_command(DriverCore::Arm arm, const JointState::SharedPtr message);
  void handle_set_mode(
      const std::shared_ptr<SetMode::Request> request,
      std::shared_ptr<SetMode::Response> response);
  void publish_state();
  void check_command_timeout();
  void handle_hold_current(
      const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response);
  void handle_stop_motion(
      const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response);
  void handle_emergency_stop(
      const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response);

  DriverCore core_;
  std::array<int, 7> left_joint_order_{0, 1, 2, 3, 4, 5, 6};
  std::array<int, 7> right_joint_order_{0, 1, 2, 3, 4, 5, 6};
  JointArray left_signs_{};
  JointArray right_signs_{};
  JointArray left_offsets_{};
  JointArray right_offsets_{};
  JointArray left_minimum_{};
  JointArray left_maximum_{};
  JointArray right_minimum_{};
  JointArray right_maximum_{};
  double max_delta_per_cycle_{0.05};
  double command_timeout_s_{0.20};
  std::optional<JointArray> left_last_command_;
  std::optional<JointArray> right_last_command_;
  rclcpp::Time left_last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time right_last_command_time_{0, 0, RCL_ROS_TIME};
  bool left_streaming_{false};
  bool right_streaming_{false};
  rclcpp::Publisher<JointState>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr
      robot_state_publisher_;
  rclcpp::Subscription<JointState>::SharedPtr left_command_subscription_;
  rclcpp::Subscription<JointState>::SharedPtr right_command_subscription_;
  rclcpp::Service<SetMode>::SharedPtr set_mode_service_;
  rclcpp::Service<Trigger>::SharedPtr hold_current_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_motion_service_;
  rclcpp::Service<Trigger>::SharedPtr emergency_stop_service_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

}  // namespace skye_robot_driver
