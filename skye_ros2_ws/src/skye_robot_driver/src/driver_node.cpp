#include "skye_robot_driver/driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace skye_robot_driver {
namespace {

constexpr DriverCore::JointArray kDefaultMinimum{
    -3.1067, -2.01, -3.1067, -1.0472, -3.1067, -1.0472, -1.5708};
constexpr DriverCore::JointArray kDefaultMaximum{
    3.1067, 2.01, 3.1067, 2.53, 3.1067, 1.0472, 1.5708};
constexpr DriverCore::JointArray kDefaultSigns{
    1.0, 1.0, 1.0, -1.0, 1.0, -1.0, -1.0};
constexpr DriverCore::JointArray kDefaultOffsets{
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
constexpr std::array<int, 7> kDefaultOrder{0, 1, 2, 3, 4, 5, 6};
constexpr DriverCore::JointArray kDefaultJointK{
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
constexpr DriverCore::JointArray kDefaultJointD{
    10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
constexpr DriverCore::JointArray kDefaultCartK{
    2000.0, 2000.0, 2000.0, 100.0, 100.0, 100.0, 50.0};
constexpr DriverCore::JointArray kDefaultCartD{
    0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1.0};

const std::array<std::string, 14> kJointNames{
    "l_j1", "l_j2", "l_j3", "l_j4", "l_j5", "l_j6", "l_j7",
    "r_j1", "r_j2", "r_j3", "r_j4", "r_j5", "r_j6", "r_j7"};

rclcpp::QoS control_qos() {
  // Accepts FACTR (typically RELIABLE) publishers; drops old cmds.
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

rclcpp::QoS state_qos() {
  // FACTR sync subscribes with default RELIABLE; BEST_EFFORT state would not match.
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

}  // namespace

DriverNode::DriverNode(const rclcpp::NodeOptions &options)
    : Node("skye_robot_driver", options) {
  const auto robot_ip = declare_parameter<std::string>("robot_ip", "6.6.7.190");
  const auto left_vel = declare_parameter<int>("left_velocity_ratio", 10);
  const auto right_vel = declare_parameter<int>("right_velocity_ratio", 10);
  const auto left_acc =
      declare_parameter<int>("left_acceleration_ratio", left_vel);
  const auto right_acc =
      declare_parameter<int>("right_acceleration_ratio", right_vel);
  const auto state_publish_hz =
      declare_parameter<double>("state_publish_hz", 250.0);
  const auto connect_on_startup =
      declare_parameter<bool>("connect_on_startup", true);
  const auto control_mode_str =
      declare_parameter<std::string>("control_mode", "imp_joint");
  const auto cmd_cycle_time_ms = declare_parameter<int>("cmd_cycle_time_ms", 4);
  max_delta_per_cycle_ = declare_parameter<double>("max_delta_per_cycle", 0.05);
  command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.20);

  left_joint_order_ = load_joint_order(*this, "left_joint_order", kDefaultOrder);
  right_joint_order_ =
      load_joint_order(*this, "right_joint_order", kDefaultOrder);
  left_signs_ = load_joint_array(*this, "left_joint_signs", kDefaultSigns);
  right_signs_ = load_joint_array(*this, "right_joint_signs", kDefaultSigns);
  left_offsets_ =
      load_joint_array(*this, "left_joint_offsets", kDefaultOffsets);
  right_offsets_ =
      load_joint_array(*this, "right_joint_offsets", kDefaultOffsets);
  left_minimum_ =
      load_joint_array(*this, "left_joint_limits_min", kDefaultMinimum);
  left_maximum_ =
      load_joint_array(*this, "left_joint_limits_max", kDefaultMaximum);
  right_minimum_ =
      load_joint_array(*this, "right_joint_limits_min", kDefaultMinimum);
  right_maximum_ =
      load_joint_array(*this, "right_joint_limits_max", kDefaultMaximum);

  DriverCore::ConnectConfig connect_config;
  connect_config.mode = parse_control_mode(control_mode_str);
  connect_config.left_vel_ratio = left_vel;
  connect_config.right_vel_ratio = right_vel;
  connect_config.left_acc_ratio = left_acc;
  connect_config.right_acc_ratio = right_acc;
  connect_config.cmd_cycle_time_ms = cmd_cycle_time_ms;
  connect_config.joint_gains.k =
      load_joint_array(*this, "impedance_stiffness", kDefaultJointK);
  connect_config.joint_gains.d =
      load_joint_array(*this, "impedance_damping", kDefaultJointD);
  connect_config.cart_gains.k =
      load_joint_array(*this, "cartesian_stiffness", kDefaultCartK);
  connect_config.cart_gains.d =
      load_joint_array(*this, "cartesian_damping", kDefaultCartD);

  if (state_publish_hz <= 0.0 || !std::isfinite(state_publish_hz)) {
    throw std::invalid_argument(
        "state_publish_hz must be finite and greater than zero");
  }
  if (!std::isfinite(max_delta_per_cycle_) || max_delta_per_cycle_ <= 0.0) {
    throw std::invalid_argument(
        "max_delta_per_cycle must be finite and greater than zero");
  }
  if (!std::isfinite(command_timeout_s_) || command_timeout_s_ <= 0.0) {
    throw std::invalid_argument(
        "command_timeout_s must be finite and greater than zero");
  }
  if (cmd_cycle_time_ms <= 0) {
    throw std::invalid_argument("cmd_cycle_time_ms must be > 0");
  }
  auto ratio_ok = [](int value) { return value >= 1 && value <= 100; };
  if (!ratio_ok(left_vel) || !ratio_ok(right_vel) || !ratio_ok(left_acc) ||
      !ratio_ok(right_acc)) {
    throw std::invalid_argument("velocity/acceleration ratios must be 1..100");
  }
  for (std::size_t i = 0; i < DriverCore::JointArray{}.size(); ++i) {
    if (left_minimum_[i] > left_maximum_[i] ||
        right_minimum_[i] > right_maximum_[i]) {
      throw std::invalid_argument("joint limit minimum cannot exceed maximum");
    }
  }

  const auto cmd_qos = control_qos();
  const auto st_qos = state_qos();
  state_publisher_ = create_publisher<JointState>("/joint_states", st_qos);
  robot_state_publisher_ =
      create_publisher<std_msgs::msg::Int16MultiArray>("/robot_state", st_qos);
  left_command_subscription_ = create_subscription<JointState>(
      "/left_joint_control", cmd_qos,
      [this](JointState::SharedPtr message) {
        handle_command(DriverCore::Arm::kLeft, std::move(message));
      });
  right_command_subscription_ = create_subscription<JointState>(
      "/right_joint_control", cmd_qos,
      [this](JointState::SharedPtr message) {
        handle_command(DriverCore::Arm::kRight, std::move(message));
      });

  set_mode_service_ = create_service<SetMode>(
      "/set_mode",
      [this](
          const std::shared_ptr<SetMode::Request> request,
          std::shared_ptr<SetMode::Response> response) {
        handle_set_mode(request, response);
      });
  hold_current_service_ = create_service<Trigger>(
      "/hold_current",
      [this](
          const std::shared_ptr<Trigger::Request> request,
          std::shared_ptr<Trigger::Response> response) {
        handle_hold_current(request, response);
      });
  stop_motion_service_ = create_service<Trigger>(
      "/stop_motion",
      [this](
          const std::shared_ptr<Trigger::Request> request,
          std::shared_ptr<Trigger::Response> response) {
        handle_stop_motion(request, response);
      });
  emergency_stop_service_ = create_service<Trigger>(
      "/emergency_stop",
      [this](
          const std::shared_ptr<Trigger::Request> request,
          std::shared_ptr<Trigger::Response> response) {
        handle_emergency_stop(request, response);
      });

  const auto period = std::chrono::duration<double>(1.0 / state_publish_hz);
  state_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publish_state(); });
  timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() { check_command_timeout(); });

  if (connect_on_startup) {
    const auto ip = parse_ipv4(robot_ip);
    if (!core_.connect_and_enable(ip, connect_config)) {
      throw std::runtime_error(
          "Gento SDK failed to connect or enter control mode");
    }
    RCLCPP_INFO(
        get_logger(),
        "Connected to controller %s; mode=%s(%d); vel L/R=%d/%d; acc L/R=%d/%d; "
        "cmd_cycle=%dms",
        robot_ip.c_str(), DriverCore::mode_name(connect_config.mode),
        static_cast<int>(connect_config.mode), connect_config.left_vel_ratio,
        connect_config.right_vel_ratio, connect_config.left_acc_ratio,
        connect_config.right_acc_ratio, connect_config.cmd_cycle_time_ms);
  } else {
    RCLCPP_INFO(
        get_logger(),
        "Hardware connection disabled by connect_on_startup=false");
  }
}

DriverNode::~DriverNode() { core_.shutdown(); }

DriverCore::ControlMode DriverNode::parse_control_mode(
    const std::string &value) {
  if (value == "idle" || value == "0") {
    return DriverCore::ControlMode::kIdle;
  }
  if (value == "position" || value == "1") {
    return DriverCore::ControlMode::kPosition;
  }
  if (value == "imp_joint" || value == "impedance" || value == "2") {
    return DriverCore::ControlMode::kImpJoint;
  }
  if (value == "imp_cart" || value == "3") {
    return DriverCore::ControlMode::kImpCart;
  }
  if (value == "pd" || value == "11") {
    return DriverCore::ControlMode::kPd;
  }
  throw std::invalid_argument(
      "control_mode must be idle|position|imp_joint|imp_cart|pd (or 0/1/2/3/11)");
}

std::array<unsigned char, 4> DriverNode::parse_ipv4(const std::string &value) {
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

DriverNode::JointArray DriverNode::load_joint_array(
    rclcpp::Node &node, const std::string &parameter_name,
    const JointArray &defaults) {
  const std::vector<double> default_values(defaults.begin(), defaults.end());
  const auto values =
      node.declare_parameter<std::vector<double>>(parameter_name, default_values);
  if (values.size() != defaults.size() ||
      !std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
      })) {
    throw std::invalid_argument(
        parameter_name + " must contain seven finite values");
  }

  JointArray result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

std::array<int, 7> DriverNode::load_joint_order(
    rclcpp::Node &node, const std::string &parameter_name,
    const std::array<int, 7> &defaults) {
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

void DriverNode::handle_set_mode(
    const std::shared_ptr<SetMode::Request> request,
    std::shared_ptr<SetMode::Response> response) {
  const auto mapped = DriverCore::mode_from_int(request->mode);
  response->left_state =
      static_cast<int16_t>(core_.current_state(DriverCore::Arm::kLeft));
  response->right_state =
      static_cast<int16_t>(core_.current_state(DriverCore::Arm::kRight));

  if (!mapped) {
    response->success = false;
    response->message =
        "unsupported mode (use 0=idle, 1=position, 2=imp_joint, 3=imp_cart, 11=pd)";
    return;
  }

  if (!core_.switch_control_mode(*mapped)) {
    response->success = false;
    response->message =
        std::string("failed to switch to ") + DriverCore::mode_name(*mapped);
    response->left_state =
        static_cast<int16_t>(core_.current_state(DriverCore::Arm::kLeft));
    response->right_state =
        static_cast<int16_t>(core_.current_state(DriverCore::Arm::kRight));
    return;
  }

  response->success = true;
  response->message =
      std::string("switched to ") + DriverCore::mode_name(*mapped);
  response->left_state =
      static_cast<int16_t>(core_.current_state(DriverCore::Arm::kLeft));
  response->right_state =
      static_cast<int16_t>(core_.current_state(DriverCore::Arm::kRight));
  left_streaming_ = false;
  right_streaming_ = false;
  left_last_command_.reset();
  right_last_command_.reset();
  RCLCPP_INFO(
      get_logger(), "set_mode -> %s (%d); feedback L/R=%d/%d",
      DriverCore::mode_name(*mapped), static_cast<int>(*mapped),
      response->left_state, response->right_state);
}

void DriverNode::handle_command(
    DriverCore::Arm arm, const JointState::SharedPtr message) {
  const char *arm_name = arm == DriverCore::Arm::kLeft ? "left" : "right";
  if (message->position.size() != DriverCore::JointArray{}.size()) {
    RCLCPP_ERROR(
        get_logger(),
        "%s command rejected: expected exactly 7 positions, received %zu",
        arm_name, message->position.size());
    return;
  }

  JointArray leader{};
  std::copy(message->position.begin(), message->position.end(), leader.begin());

  const auto &order =
      arm == DriverCore::Arm::kLeft ? left_joint_order_ : right_joint_order_;
  const auto &signs =
      arm == DriverCore::Arm::kLeft ? left_signs_ : right_signs_;
  const auto &offsets =
      arm == DriverCore::Arm::kLeft ? left_offsets_ : right_offsets_;
  const auto &minimum =
      arm == DriverCore::Arm::kLeft ? left_minimum_ : right_minimum_;
  const auto &maximum =
      arm == DriverCore::Arm::kLeft ? left_maximum_ : right_maximum_;
  auto &last_command =
      arm == DriverCore::Arm::kLeft ? left_last_command_ : right_last_command_;
  auto &last_command_time = arm == DriverCore::Arm::kLeft
                                ? left_last_command_time_
                                : right_last_command_time_;
  auto &streaming =
      arm == DriverCore::Arm::kLeft ? left_streaming_ : right_streaming_;

  auto mapped =
      DriverCore::apply_joint_mapping(leader, order, signs, offsets);
  if (!DriverCore::validate_target(mapped, minimum, maximum)) {
    RCLCPP_ERROR(
        get_logger(),
        "%s command rejected: mapped target non-finite or outside limits",
        arm_name);
    return;
  }

  if (!last_command) {
    last_command = mapped;
  } else {
    mapped =
        DriverCore::limit_delta(mapped, *last_command, max_delta_per_cycle_);
  }

  if (!core_.send_position(arm, mapped)) {
    RCLCPP_ERROR(
        get_logger(),
        "%s command failed: SDK not ready / idle / mode rejected command",
        arm_name);
    return;
  }

  last_command = mapped;
  last_command_time = now();
  streaming = true;
}

void DriverNode::check_command_timeout() {
  const auto stamp = now();
  auto maybe_hold = [this, &stamp](
                        bool &streaming, const rclcpp::Time &last_command_time,
                        const char *arm_name) {
    if (!streaming) {
      return;
    }
    if ((stamp - last_command_time).seconds() <= command_timeout_s_) {
      return;
    }
    RCLCPP_WARN(
        get_logger(), "%s command timeout (%.3f s); calling hold_current",
        arm_name, command_timeout_s_);
    if (!core_.hold_current()) {
      RCLCPP_ERROR(
          get_logger(), "hold_current after %s timeout failed", arm_name);
    }
    left_streaming_ = false;
    right_streaming_ = false;
  };

  maybe_hold(left_streaming_, left_last_command_time_, "left");
  maybe_hold(right_streaming_, right_last_command_time_, "right");
}

void DriverNode::handle_hold_current(
    const std::shared_ptr<Trigger::Request> /*request*/,
    std::shared_ptr<Trigger::Response> response) {
  const bool ok = core_.hold_current();
  response->success = ok;
  response->message =
      ok ? "holding current joint positions" : "hold_current failed";
  if (ok) {
    left_streaming_ = false;
    right_streaming_ = false;
  }
}

void DriverNode::handle_stop_motion(
    const std::shared_ptr<Trigger::Request> /*request*/,
    std::shared_ptr<Trigger::Response> response) {
  const bool ok = core_.stop_motion();
  response->success = ok;
  response->message = ok ? "motion stopped; now IDLE" : "stop_motion failed";
  left_streaming_ = false;
  right_streaming_ = false;
  left_last_command_.reset();
  right_last_command_.reset();
}

void DriverNode::handle_emergency_stop(
    const std::shared_ptr<Trigger::Request> /*request*/,
    std::shared_ptr<Trigger::Response> response) {
  const bool ok = core_.emergency_stop();
  response->success = ok;
  response->message = ok ? "emergency stop issued; now IDLE" : "emergency_stop failed";
  left_streaming_ = false;
  right_streaming_ = false;
  left_last_command_.reset();
  right_last_command_.reset();
}

void DriverNode::publish_state() {
  const auto state = core_.read_state();
  if (state) {
    JointState message;
    message.header.stamp = now();
    message.name.assign(kJointNames.begin(), kJointNames.end());
    message.position.reserve(kJointNames.size());
    message.velocity.reserve(kJointNames.size());
    message.position.insert(
        message.position.end(), state->left_position.begin(),
        state->left_position.end());
    message.position.insert(
        message.position.end(), state->right_position.begin(),
        state->right_position.end());
    message.velocity.insert(
        message.velocity.end(), state->left_velocity.begin(),
        state->left_velocity.end());
    message.velocity.insert(
        message.velocity.end(), state->right_velocity.begin(),
        state->right_velocity.end());
    state_publisher_->publish(message);
  }

  std_msgs::msg::Int16MultiArray robot_state;
  robot_state.data = {
      static_cast<int16_t>(core_.current_state(DriverCore::Arm::kLeft)),
      static_cast<int16_t>(core_.current_state(DriverCore::Arm::kRight))};
  robot_state_publisher_->publish(robot_state);
}

}  // namespace skye_robot_driver
