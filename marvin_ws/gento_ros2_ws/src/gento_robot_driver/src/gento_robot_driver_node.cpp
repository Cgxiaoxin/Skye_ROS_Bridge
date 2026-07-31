#include "gento_robot_driver/gento_robot_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace gento_robot_driver {
namespace {

constexpr DriverCore::JointArray kDefaultMinimum{
    -3.1067, -2.01, -3.1067, -1.0472, -3.1067, -1.0472, -1.5708};
constexpr DriverCore::JointArray kDefaultMaximum{
    3.1067, 2.01, 3.1067, 2.53, 3.1067, 1.0472, 1.5708};
constexpr DriverCore::JointArray kDefaultSigns{1.0, 1.0, 1.0, -1.0, 1.0, -1.0, -1.0};
constexpr DriverCore::JointArray kDefaultOffsets{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
constexpr std::array<int, 7> kDefaultOrder{0, 1, 2, 3, 4, 5, 6};

const std::array<std::string, 14> kJointNames{
    "l_j1", "l_j2", "l_j3", "l_j4", "l_j5", "l_j6", "l_j7",
    "r_j1", "r_j2", "r_j3", "r_j4", "r_j5", "r_j6", "r_j7"};

}  // namespace

GentoRobotDriverNode::GentoRobotDriverNode(const rclcpp::NodeOptions& options)
    : Node("gento_robot_driver", options) {
  const auto robot_ip = declare_parameter<std::string>("robot_ip", "6.6.7.190");
  const auto left_ratio_parameter = declare_parameter<int>("left_velocity_ratio", 10);
  const auto right_ratio_parameter = declare_parameter<int>("right_velocity_ratio", 10);
  const auto state_publish_hz = declare_parameter<double>("state_publish_hz", 100.0);
  const auto connect_on_startup = declare_parameter<bool>("connect_on_startup", true);
  max_delta_per_cycle_ = declare_parameter<double>("max_delta_per_cycle", 0.05);
  command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.20);

  left_joint_order_ = load_joint_order(*this, "left_joint_order", kDefaultOrder);
  right_joint_order_ = load_joint_order(*this, "right_joint_order", kDefaultOrder);
  left_signs_ = load_joint_array(*this, "left_joint_signs", kDefaultSigns);
  right_signs_ = load_joint_array(*this, "right_joint_signs", kDefaultSigns);
  left_offsets_ = load_joint_array(*this, "left_joint_offsets", kDefaultOffsets);
  right_offsets_ = load_joint_array(*this, "right_joint_offsets", kDefaultOffsets);
  left_minimum_ = load_joint_array(*this, "left_joint_limits_min", kDefaultMinimum);
  left_maximum_ = load_joint_array(*this, "left_joint_limits_max", kDefaultMaximum);
  right_minimum_ = load_joint_array(*this, "right_joint_limits_min", kDefaultMinimum);
  right_maximum_ = load_joint_array(*this, "right_joint_limits_max", kDefaultMaximum);

  if (state_publish_hz <= 0.0 || !std::isfinite(state_publish_hz)) {
    throw std::invalid_argument("state_publish_hz must be finite and greater than zero");
  }
  if (!std::isfinite(max_delta_per_cycle_) || max_delta_per_cycle_ <= 0.0) {
    throw std::invalid_argument("max_delta_per_cycle must be finite and greater than zero");
  }
  if (!std::isfinite(command_timeout_s_) || command_timeout_s_ <= 0.0) {
    throw std::invalid_argument("command_timeout_s must be finite and greater than zero");
  }
  if (left_ratio_parameter < 1 || left_ratio_parameter > 100 ||
      right_ratio_parameter < 1 || right_ratio_parameter > 100) {
    throw std::invalid_argument("velocity ratios must be in the range 1..100");
  }
  const int left_ratio = static_cast<int>(left_ratio_parameter);
  const int right_ratio = static_cast<int>(right_ratio_parameter);
  for (std::size_t i = 0; i < DriverCore::JointArray{}.size(); ++i) {
    if (left_minimum_[i] > left_maximum_[i] || right_minimum_[i] > right_maximum_[i]) {
      throw std::invalid_argument("joint limit minimum cannot exceed maximum");
    }
  }

  state_publisher_ = create_publisher<JointState>("/joint_states", 10);
  left_command_subscription_ = create_subscription<JointState>(
      "/left_joint_control", 10,
      [this](JointState::SharedPtr message) {
        handle_command(DriverCore::Arm::kLeft, std::move(message));
      });
  right_command_subscription_ = create_subscription<JointState>(
      "/right_joint_control", 10,
      [this](JointState::SharedPtr message) {
        handle_command(DriverCore::Arm::kRight, std::move(message));
      });

  hold_current_service_ = create_service<Trigger>(
      "/hold_current",
      [this](const std::shared_ptr<Trigger::Request> request,
             std::shared_ptr<Trigger::Response> response) {
        handle_hold_current(request, response);
      });
  stop_motion_service_ = create_service<Trigger>(
      "/stop_motion",
      [this](const std::shared_ptr<Trigger::Request> request,
             std::shared_ptr<Trigger::Response> response) {
        handle_stop_motion(request, response);
      });

  const auto period = std::chrono::duration<double>(1.0 / state_publish_hz);
  state_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publish_state(); });
  timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() { check_command_timeout(); });

  if (connect_on_startup) {
    const auto ip = parse_ipv4(robot_ip);
    if (!core_.connect_and_enable(ip, left_ratio, right_ratio)) {
      throw std::runtime_error("Gento SDK failed to connect or enter position mode");
    }
    RCLCPP_INFO(
        get_logger(),
        "Connected to Gento controller %s; left/right speed ratios: %d%%/%d%%",
        robot_ip.c_str(), left_ratio, right_ratio);
  } else {
    RCLCPP_INFO(get_logger(), "Hardware connection disabled by connect_on_startup=false");
  }
}

GentoRobotDriverNode::~GentoRobotDriverNode() {
  core_.shutdown();
}

std::array<unsigned char, 4> GentoRobotDriverNode::parse_ipv4(
    const std::string& value) {
  std::array<unsigned char, 4> result{};
  std::istringstream stream(value);
  std::string component;
  for (std::size_t i = 0; i < result.size(); ++i) {
    if (!std::getline(stream, component, '.') || component.empty()) {
      throw std::invalid_argument("robot_ip must be a dotted IPv4 address");
    }
    std::size_t parsed_characters = 0;
    const int octet = std::stoi(component, &parsed_characters);
    if (parsed_characters != component.size() || octet < 0 || octet > 255) {
      throw std::invalid_argument("robot_ip contains an invalid IPv4 octet");
    }
    result[i] = static_cast<unsigned char>(octet);
  }
  if (std::getline(stream, component, '.')) {
    throw std::invalid_argument("robot_ip must contain exactly four octets");
  }
  return result;
}

GentoRobotDriverNode::JointArray GentoRobotDriverNode::load_joint_array(
    rclcpp::Node& node,
    const std::string& parameter_name,
    const JointArray& defaults) {
  const std::vector<double> default_values(defaults.begin(), defaults.end());
  const auto values = node.declare_parameter<std::vector<double>>(
      parameter_name, default_values);
  if (values.size() != defaults.size() ||
      !std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
      })) {
    throw std::invalid_argument(parameter_name + " must contain seven finite values");
  }

  JointArray result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

std::array<int, 7> GentoRobotDriverNode::load_joint_order(
    rclcpp::Node& node,
    const std::string& parameter_name,
    const std::array<int, 7>& defaults) {
  const std::vector<int64_t> default_values(defaults.begin(), defaults.end());
  const auto values = node.declare_parameter<std::vector<int64_t>>(
      parameter_name, default_values);
  if (values.size() != defaults.size()) {
    throw std::invalid_argument(parameter_name + " must contain seven indices");
  }

  std::array<int, 7> result{};
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (values[i] < 0 || values[i] > 6) {
      throw std::invalid_argument(parameter_name + " indices must be in 0..6");
    }
    result[i] = static_cast<int>(values[i]);
  }
  return result;
}

void GentoRobotDriverNode::handle_command(
    DriverCore::Arm arm,
    const JointState::SharedPtr message) {
  const char* arm_name = arm == DriverCore::Arm::kLeft ? "left" : "right";
  if (message->position.size() != DriverCore::JointArray{}.size()) {
    RCLCPP_ERROR(
        get_logger(), "%s command rejected: expected exactly 7 positions, received %zu",
        arm_name, message->position.size());
    return;
  }

  JointArray leader{};
  std::copy(message->position.begin(), message->position.end(), leader.begin());

  const auto& order = arm == DriverCore::Arm::kLeft ? left_joint_order_ : right_joint_order_;
  const auto& signs = arm == DriverCore::Arm::kLeft ? left_signs_ : right_signs_;
  const auto& offsets = arm == DriverCore::Arm::kLeft ? left_offsets_ : right_offsets_;
  const auto& minimum = arm == DriverCore::Arm::kLeft ? left_minimum_ : right_minimum_;
  const auto& maximum = arm == DriverCore::Arm::kLeft ? left_maximum_ : right_maximum_;
  auto& last_command =
      arm == DriverCore::Arm::kLeft ? left_last_command_ : right_last_command_;
  auto& last_command_time =
      arm == DriverCore::Arm::kLeft ? left_last_command_time_ : right_last_command_time_;
  auto& streaming = arm == DriverCore::Arm::kLeft ? left_streaming_ : right_streaming_;

  auto mapped = DriverCore::apply_joint_mapping(leader, order, signs, offsets);
  if (!DriverCore::validate_target(mapped, minimum, maximum)) {
    RCLCPP_ERROR(
        get_logger(), "%s command rejected: mapped target non-finite or outside limits",
        arm_name);
    return;
  }

  if (!last_command) {
    last_command = mapped;
  } else {
    mapped = DriverCore::limit_delta(mapped, *last_command, max_delta_per_cycle_);
  }

  if (!core_.send_position(arm, mapped)) {
    RCLCPP_ERROR(get_logger(), "%s command failed: SDK is not ready or returned an error", arm_name);
    return;
  }

  last_command = mapped;
  last_command_time = now();
  streaming = true;
}

void GentoRobotDriverNode::check_command_timeout() {
  const auto stamp = now();
  auto maybe_hold = [this, &stamp](
                        bool& streaming,
                        const rclcpp::Time& last_command_time,
                        const char* arm_name) {
    if (!streaming) {
      return;
    }
    if ((stamp - last_command_time).seconds() <= command_timeout_s_) {
      return;
    }
    RCLCPP_WARN(
        get_logger(),
        "%s command timeout (%.3f s); calling hold_current",
        arm_name, command_timeout_s_);
    if (!core_.hold_current()) {
      RCLCPP_ERROR(get_logger(), "hold_current after %s timeout failed", arm_name);
    }
    left_streaming_ = false;
    right_streaming_ = false;
  };

  maybe_hold(left_streaming_, left_last_command_time_, "left");
  maybe_hold(right_streaming_, right_last_command_time_, "right");
}

void GentoRobotDriverNode::handle_hold_current(
    const std::shared_ptr<Trigger::Request> /*request*/,
    std::shared_ptr<Trigger::Response> response) {
  const bool ok = core_.hold_current();
  response->success = ok;
  response->message = ok ? "holding current joint positions" : "hold_current failed";
  if (ok) {
    left_streaming_ = false;
    right_streaming_ = false;
  }
}

void GentoRobotDriverNode::handle_stop_motion(
    const std::shared_ptr<Trigger::Request> /*request*/,
    std::shared_ptr<Trigger::Response> response) {
  const bool ok = core_.stop_motion();
  response->success = ok;
  response->message = ok ? "motion stopped; commands rejected until hold_current" : "stop_motion failed";
  left_streaming_ = false;
  right_streaming_ = false;
  left_last_command_.reset();
  right_last_command_.reset();
}

void GentoRobotDriverNode::publish_state() {
  const auto state = core_.read_state();
  if (!state) {
    return;
  }

  JointState message;
  message.header.stamp = now();
  message.name.assign(kJointNames.begin(), kJointNames.end());
  message.position.reserve(kJointNames.size());
  message.velocity.reserve(kJointNames.size());
  message.position.insert(
      message.position.end(), state->left_position.begin(), state->left_position.end());
  message.position.insert(
      message.position.end(), state->right_position.begin(), state->right_position.end());
  message.velocity.insert(
      message.velocity.end(), state->left_velocity.begin(), state->left_velocity.end());
  message.velocity.insert(
      message.velocity.end(), state->right_velocity.begin(), state->right_velocity.end());
  state_publisher_->publish(message);
}

}  // namespace gento_robot_driver
