#include "skye_robot_driver/driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "skye_robot_driver/gripper_common.hpp"

namespace skye_robot_driver {
namespace {

constexpr DriverCore::JointArray kDefaultMinimum{
    -3.1067, -2.0944, -3.1067, -2.5307, -3.1067, -1.0472, -1.5708};
constexpr DriverCore::JointArray kDefaultMaximum{
    3.1067, 2.0944, 3.1067, 1.0472, 3.1067, 1.0472, 1.5708};
constexpr DriverCore::JointArray kDefaultSigns{
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
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

constexpr std::size_t kArmDof = 7;

const std::array<std::string, 14> kJointNames{
    "l_j1", "l_j2", "l_j3", "l_j4", "l_j5", "l_j6", "l_j7",
    "r_j1", "r_j2", "r_j3", "r_j4", "r_j5", "r_j6", "r_j7"};

const std::array<std::string, kArmDof> kLeftJointNames{
    "l_j1", "l_j2", "l_j3", "l_j4", "l_j5", "l_j6", "l_j7"};
const std::array<std::string, kArmDof> kRightJointNames{
    "r_j1", "r_j2", "r_j3", "r_j4", "r_j5", "r_j6", "r_j7"};

sensor_msgs::msg::JointState make_arm_joint_state(
    const rclcpp::Time &stamp,
    const std::array<std::string, kArmDof> &names,
    const DriverCore::JointArray &position,
    const DriverCore::JointArray &velocity) {
  sensor_msgs::msg::JointState message;
  message.header.stamp = stamp;
  message.name.assign(names.begin(), names.end());
  message.position.assign(position.begin(), position.end());
  message.velocity.assign(velocity.begin(), velocity.end());
  return message;
}

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

FXChnType parse_robotiq_485_channel(const std::string &value) {
  std::string key = value;
  std::transform(key.begin(), key.end(), key.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (key == "485b" || key == "b") {
    return FX_CHN_485B;
  }
  return FX_CHN_485A;
}

GripperDriverType load_gripper_type(rclcpp::Node &node, const std::string &param) {
  const auto raw = node.declare_parameter<std::string>(param, "dm4310");
  const auto parsed = parse_gripper_type(raw);
  if (!parsed) {
    throw std::invalid_argument("invalid " + param + ": " + raw);
  }
  return *parsed;
}

void validate_dm4310_arm(const Dm4310GripperArm::Config &cfg) {
  if (!std::isfinite(cfg.kp) || !std::isfinite(cfg.kd) ||
      !std::isfinite(cfg.pos_min) || !std::isfinite(cfg.pos_max) ||
      !std::isfinite(cfg.close_limit) || cfg.pos_max <= cfg.pos_min ||
      cfg.close_limit <= 0.0 || cfg.close_limit > 1.0 ||
      cfg.feedback_timeout_ms == 0 ||
      (cfg.terminal != 0 && cfg.terminal != 1)) {
    throw std::invalid_argument("invalid dm4310 gripper params");
  }
}

void validate_robotiq_arm(const RobotiqGripperArm::Config &cfg) {
  if (cfg.slave_id <= 0 || cfg.slave_id > 247 ||
      cfg.pos_max_mm <= cfg.pos_min_mm || cfg.close_limit <= 0.0 ||
      cfg.close_limit > 1.0 || cfg.modbus_timeout_ms == 0 ||
      (cfg.terminal != 0 && cfg.terminal != 1)) {
    throw std::invalid_argument("invalid robotiq gripper params");
  }
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
  max_delta_per_cycle_ = declare_parameter<double>("max_delta_per_cycle", 0.25);
  command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.50);
  teleop_mapping_mode_ = parse_teleop_mapping_mode(
      declare_parameter<std::string>("teleop_mapping_mode", "relative"));
  const auto enable_gripper = declare_parameter<bool>("enable_gripper", true);
  const auto gripper_rate_hz =
      declare_parameter<double>("gripper_rate_hz", 100.0);
  const auto gripper_invert_default =
      declare_parameter<bool>("gripper_invert", true);
  gripper_invert_left_ = declare_parameter<bool>(
      "gripper_left_invert", gripper_invert_default);
  gripper_invert_right_ = declare_parameter<bool>(
      "gripper_right_invert", gripper_invert_default);

  GripperBridge::Config gripper_config;
  gripper_config.left.type =
      load_gripper_type(*this, "gripper_left_type");
  gripper_config.right.type =
      load_gripper_type(*this, "gripper_right_type");

  const auto gripper_kp = declare_parameter<double>("gripper_kp", 3.0);
  const auto gripper_kd = declare_parameter<double>("gripper_kd", 0.12);
  const auto gripper_pos_min = declare_parameter<double>("gripper_pos_min", 0.0);
  const auto gripper_pos_max = declare_parameter<double>("gripper_pos_max", 1.6);
  const auto gripper_close_limit =
      declare_parameter<double>("gripper_close_limit", 0.93);
  const auto gripper_feedback_timeout_ms = static_cast<unsigned int>(
      declare_parameter<int>("gripper_feedback_timeout_ms", 1));
  const auto gripper_left_motor_id =
      declare_parameter<int>("gripper_left_motor_id", 1);
  const auto gripper_right_motor_id =
      declare_parameter<int>("gripper_right_motor_id", 2);
  const auto gripper_right_terminal =
      declare_parameter<int>("gripper_right_terminal", 1);

  gripper_config.left.dm = {
      gripper_left_motor_id, 0, gripper_kp, gripper_kd, gripper_pos_min,
      gripper_pos_max, gripper_close_limit, gripper_feedback_timeout_ms};
  gripper_config.right.dm = {
      gripper_right_motor_id, gripper_right_terminal, gripper_kp, gripper_kd,
      gripper_pos_min, gripper_pos_max, gripper_close_limit,
      gripper_feedback_timeout_ms};

  const auto robotiq_speed = declare_parameter<int>("gripper_robotiq_speed", 136);
  const auto robotiq_force = declare_parameter<int>("gripper_robotiq_force", 16);
  const auto robotiq_pos_min_mm =
      declare_parameter<double>("gripper_robotiq_pos_min_mm", 0.0);
  const auto left_robotiq_pos_min_mm = declare_parameter<double>(
      "gripper_left_robotiq_pos_min_mm", robotiq_pos_min_mm);
  const auto right_robotiq_pos_min_mm = declare_parameter<double>(
      "gripper_right_robotiq_pos_min_mm", robotiq_pos_min_mm);
  const auto robotiq_pos_max_mm =
      declare_parameter<double>("gripper_robotiq_pos_max_mm", 50.0);
  const auto left_robotiq_pos_max_mm = declare_parameter<double>(
      "gripper_left_robotiq_pos_max_mm", robotiq_pos_max_mm);
  const auto right_robotiq_pos_max_mm = declare_parameter<double>(
      "gripper_right_robotiq_pos_max_mm", robotiq_pos_max_mm);
  const auto robotiq_modbus_timeout_ms = static_cast<unsigned int>(
      declare_parameter<int>("gripper_robotiq_modbus_timeout_ms", 150));

  gripper_config.left.robotiq = {
      declare_parameter<int>("gripper_left_robotiq_slave_id", 9), robotiq_speed,
      robotiq_force, left_robotiq_pos_min_mm, left_robotiq_pos_max_mm,
      gripper_close_limit,
      parse_robotiq_485_channel(declare_parameter<std::string>(
          "gripper_left_robotiq_485_channel", "485A")),
      declare_parameter<int>("gripper_left_robotiq_terminal", 0),
      robotiq_modbus_timeout_ms};
  gripper_config.right.robotiq = {
      declare_parameter<int>("gripper_right_robotiq_slave_id", 9), robotiq_speed,
      robotiq_force, right_robotiq_pos_min_mm, right_robotiq_pos_max_mm,
      gripper_close_limit,
      parse_robotiq_485_channel(declare_parameter<std::string>(
          "gripper_right_robotiq_485_channel", "485A")),
      declare_parameter<int>("gripper_right_robotiq_terminal", 1),
      robotiq_modbus_timeout_ms};

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
  if (enable_gripper) {
    if (!std::isfinite(gripper_rate_hz) || gripper_rate_hz <= 0.0) {
      throw std::invalid_argument(
          "gripper_rate_hz must be finite and greater than zero");
    }
    if (gripper_config.left.type == GripperDriverType::kDm4310) {
      validate_dm4310_arm(gripper_config.left.dm);
    } else {
      validate_robotiq_arm(gripper_config.left.robotiq);
    }
    if (gripper_config.right.type == GripperDriverType::kDm4310) {
      validate_dm4310_arm(gripper_config.right.dm);
    } else {
      validate_robotiq_arm(gripper_config.right.robotiq);
    }
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

  control_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  gripper_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions control_sub_opts;
  control_sub_opts.callback_group = control_callback_group_;
  rclcpp::SubscriptionOptions gripper_sub_opts;
  gripper_sub_opts.callback_group = gripper_callback_group_;

  const auto cmd_qos = control_qos();
  const auto st_qos = state_qos();
  state_publisher_ = create_publisher<JointState>("/joint_states", st_qos);
  left_state_publisher_ =
      create_publisher<JointState>("/left_joint_states", st_qos);
  right_state_publisher_ =
      create_publisher<JointState>("/right_joint_states", st_qos);
  robot_state_publisher_ =
      create_publisher<std_msgs::msg::Int16MultiArray>("/robot_state", st_qos);
  left_command_subscription_ = create_subscription<JointState>(
      "/left_joint_control", cmd_qos,
      [this](JointState::SharedPtr message) {
        handle_command(DriverCore::Arm::kLeft, std::move(message));
      },
      control_sub_opts);
  right_command_subscription_ = create_subscription<JointState>(
      "/right_joint_control", cmd_qos,
      [this](JointState::SharedPtr message) {
        handle_command(DriverCore::Arm::kRight, std::move(message));
      },
      control_sub_opts);
  left_abs_command_subscription_ = create_subscription<JointState>(
      "/left_joint_control_abs", cmd_qos,
      [this](JointState::SharedPtr message) {
        handle_absolute_command(DriverCore::Arm::kLeft, std::move(message));
      },
      control_sub_opts);
  right_abs_command_subscription_ = create_subscription<JointState>(
      "/right_joint_control_abs", cmd_qos,
      [this](JointState::SharedPtr message) {
        handle_absolute_command(DriverCore::Arm::kRight, std::move(message));
      },
      control_sub_opts);

  left_gripper_subscription_ = create_subscription<JointState>(
      "/left_teleop_gripper/ctrl", cmd_qos,
      [this](JointState::SharedPtr message) {
        handle_gripper_command(DriverCore::Arm::kLeft, std::move(message));
      },
      gripper_sub_opts);
  right_gripper_subscription_ = create_subscription<JointState>(
      "/right_teleop_gripper/ctrl", cmd_qos,
      [this](JointState::SharedPtr message) {
        handle_gripper_command(DriverCore::Arm::kRight, std::move(message));
      },
      gripper_sub_opts);
  left_gripper_state_publisher_ =
      create_publisher<JointState>("/left_gripper/state", st_qos);
  right_gripper_state_publisher_ =
      create_publisher<JointState>("/right_gripper/state", st_qos);

  set_mode_service_ = create_service<SetMode>(
      "/set_mode",
      [this](
          const std::shared_ptr<SetMode::Request> request,
          std::shared_ptr<SetMode::Response> response) {
        handle_set_mode(request, response);
      },
      rmw_qos_profile_services_default, control_callback_group_);
  hold_current_service_ = create_service<Trigger>(
      "/hold_current",
      [this](
          const std::shared_ptr<Trigger::Request> request,
          std::shared_ptr<Trigger::Response> response) {
        handle_hold_current(request, response);
      },
      rmw_qos_profile_services_default, control_callback_group_);
  stop_motion_service_ = create_service<Trigger>(
      "/stop_motion",
      [this](
          const std::shared_ptr<Trigger::Request> request,
          std::shared_ptr<Trigger::Response> response) {
        handle_stop_motion(request, response);
      },
      rmw_qos_profile_services_default, control_callback_group_);
  emergency_stop_service_ = create_service<Trigger>(
      "/emergency_stop",
      [this](
          const std::shared_ptr<Trigger::Request> request,
          std::shared_ptr<Trigger::Response> response) {
        handle_emergency_stop(request, response);
      },
      rmw_qos_profile_services_default, control_callback_group_);

  const auto period = std::chrono::duration<double>(1.0 / state_publish_hz);
  state_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publish_state(); }, control_callback_group_);
  timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() { check_command_timeout(); },
      control_callback_group_);

  if (connect_on_startup) {
    const auto ip = parse_ipv4(robot_ip);
    if (!core_.link_controller(ip)) {
      throw std::runtime_error("Gento SDK failed to link controller");
    }
    if (!core_.configure_and_enable(connect_config)) {
      core_.shutdown();
      throw std::runtime_error(
          std::string("Gento SDK failed to enter control mode: ") +
          core_.last_error());
    }
    if (enable_gripper) {
      if (!gripper_.start(gripper_config)) {
        core_.shutdown();
        throw std::runtime_error(
            std::string("failed to start gripper bridge: ") +
            gripper_.start_report());
      }
      gripper_enabled_ = true;
    }
    RCLCPP_INFO(
        get_logger(),
        "Connected to controller %s; mode=%s(%d); teleop_mapping=%s; "
        "vel L/R=%d/%d; acc L/R=%d/%d; cmd_cycle=%dms",
        robot_ip.c_str(), DriverCore::mode_name(connect_config.mode),
        static_cast<int>(connect_config.mode),
        teleop_mapping_mode_ == TeleopMappingMode::kRelative ? "relative"
                                                             : "absolute",
        connect_config.left_vel_ratio,
        connect_config.right_vel_ratio, connect_config.left_acc_ratio,
        connect_config.right_acc_ratio, connect_config.cmd_cycle_time_ms);

    if (enable_gripper) {
      const auto gripper_period =
          std::chrono::duration<double>(1.0 / gripper_rate_hz);
      gripper_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(gripper_period),
          [this]() { tick_gripper(); }, gripper_callback_group_);
      RCLCPP_INFO(
          get_logger(),
          "Executor: control+gripper callback groups (MultiThreaded, 2). "
          "Gripper enabled: L=%s R=%s invert L/R=%s/%s rate=%.1f Hz | %s",
          gripper_.type_name(DriverCore::Arm::kLeft),
          gripper_.type_name(DriverCore::Arm::kRight),
          gripper_invert_left_ ? "true" : "false",
          gripper_invert_right_ ? "true" : "false",
          gripper_rate_hz,
          gripper_.start_report().c_str());
      if (gripper_.start_report().find("fb=NONE") != std::string::npos ||
          gripper_.start_report().find("activated=FAIL") != std::string::npos) {
        RCLCPP_WARN(
            get_logger(),
            "Gripper init incomplete: %s. state may echo target without "
            "hardware feedback.",
            gripper_.start_report().c_str());
      }
    }
  } else {
    RCLCPP_INFO(
        get_logger(),
        "Hardware connection disabled by connect_on_startup=false");
  }
}

DriverNode::~DriverNode() {
  if (gripper_enabled_) {
    gripper_.stop();
    gripper_enabled_ = false;
  }
  core_.shutdown();
}

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

DriverNode::TeleopMappingMode DriverNode::parse_teleop_mapping_mode(
    const std::string &value) {
  if (value == "absolute") {
    return TeleopMappingMode::kAbsolute;
  }
  if (value == "relative") {
    return TeleopMappingMode::kRelative;
  }
  throw std::invalid_argument(
      "teleop_mapping_mode must be relative or absolute");
}

void DriverNode::reset_teleop_session(DriverCore::Arm arm) {
  if (arm == DriverCore::Arm::kLeft) {
    left_gento_ref_.reset();
    left_last_command_.reset();
    left_streaming_ = false;
    return;
  }
  right_gento_ref_.reset();
  right_last_command_.reset();
  right_streaming_ = false;
}

void DriverNode::reset_absolute_session(DriverCore::Arm arm) {
  if (arm == DriverCore::Arm::kLeft) {
    left_abs_last_command_.reset();
    left_abs_streaming_ = false;
    return;
  }
  right_abs_last_command_.reset();
  right_abs_streaming_ = false;
}

bool DriverNode::path_streaming(DriverCore::Arm arm, bool absolute) const {
  const bool left = arm == DriverCore::Arm::kLeft;
  if (absolute) {
    return left ? left_abs_streaming_ : right_abs_streaming_;
  }
  return left ? left_streaming_ : right_streaming_;
}

bool DriverNode::path_active(DriverCore::Arm arm, bool absolute) const {
  if (!path_streaming(arm, absolute)) {
    return false;
  }
  const bool left = arm == DriverCore::Arm::kLeft;
  const auto &stamp =
      absolute ? (left ? left_abs_last_command_time_
                       : right_abs_last_command_time_)
               : (left ? left_last_command_time_ : right_last_command_time_);
  return (now() - stamp).seconds() <= command_timeout_s_;
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
  left_gento_ref_.reset();
  right_gento_ref_.reset();
  left_last_command_.reset();
  right_last_command_.reset();
  reset_absolute_session(DriverCore::Arm::kLeft);
  reset_absolute_session(DriverCore::Arm::kRight);
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
  auto &leader_prev = arm == DriverCore::Arm::kLeft
                          ? left_leader_prev_
                          : right_leader_prev_;
  auto &leader_continuous = arm == DriverCore::Arm::kLeft
                                ? left_leader_continuous_
                                : right_leader_continuous_;
  auto &leader_cont_ref = arm == DriverCore::Arm::kLeft
                              ? left_leader_cont_ref_
                              : right_leader_cont_ref_;
  auto &gento_ref =
      arm == DriverCore::Arm::kLeft ? left_gento_ref_ : right_gento_ref_;
  auto &last_command_time = arm == DriverCore::Arm::kLeft
                                ? left_last_command_time_
                                : right_last_command_time_;
  auto &streaming =
      arm == DriverCore::Arm::kLeft ? left_streaming_ : right_streaming_;

  const bool resuming = !streaming;
  if (resuming) {
    const auto state = core_.read_state();
    if (!state) {
      RCLCPP_ERROR(
          get_logger(),
          "%s command rejected: no feedback to seed teleop session", arm_name);
      return;
    }
    const auto feedback = DriverCore::clamp_to_limits(
        arm == DriverCore::Arm::kLeft ? state->left_position
                                      : state->right_position,
        minimum, maximum);
    leader_prev = leader;
    leader_continuous = leader;
    leader_cont_ref = leader;
    gento_ref = feedback;
    last_command = feedback;
    if (teleop_mapping_mode_ == TeleopMappingMode::kRelative) {
      RCLCPP_INFO(
          get_logger(),
          "%s relative teleop: captured leader/gento refs (big arm holds "
          "current pose on entry)",
          arm_name);
    } else {
      RCLCPP_INFO(
          get_logger(),
          "%s absolute teleop: seeded last_command from feedback", arm_name);
    }
  }

  JointArray mapped{};
  if (teleop_mapping_mode_ == TeleopMappingMode::kRelative) {
    mapped = DriverCore::apply_relative_joint_mapping(
        leader, leader_prev, leader_continuous, leader_cont_ref, *gento_ref,
        order, signs);
  } else {
    mapped = DriverCore::apply_joint_mapping(leader, order, signs, offsets);
  }
  if (const auto nan_j = DriverCore::first_non_finite(mapped)) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s command rejected: j%zu=non-finite (mapped leader teleop frame)",
        arm_name, *nan_j + 1);
    last_command_time = now();
    streaming = false;
    return;
  }
  // This frame was consumed by the continuous unwrap tracking.
  leader_prev = leader;

  const auto desired = mapped;
  mapped = DriverCore::clamp_to_limits(desired, minimum, maximum);
  if (mapped != desired) {
    std::ostringstream oss;
    oss << std::fixed;
    oss.precision(4);
    for (std::size_t j = 0; j < mapped.size(); ++j) {
      if (mapped[j] == desired[j]) {
        continue;
      }
      oss << " j" << (j + 1) << "=" << desired[j] << "->" << mapped[j]
          << " [" << minimum[j] << "," << maximum[j] << "]";
    }
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s command clamped:%s; other joints still tracking",
        arm_name, oss.str().c_str());
    if (teleop_mapping_mode_ == TeleopMappingMode::kRelative) {
      DriverCore::clutch_saturated_joints(
          desired, mapped, leader_continuous, leader_cont_ref, *gento_ref,
          order);
    }
  }

  mapped =
      DriverCore::limit_delta(mapped, *last_command, max_delta_per_cycle_);
  mapped = DriverCore::clamp_to_limits(mapped, minimum, maximum);

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
  if (path_streaming(arm, true)) {
    RCLCPP_INFO(
        get_logger(),
        "%s relative teleop command takes over; invalidating absolute session",
        arm_name);
    reset_absolute_session(arm);
  }
}

void DriverNode::handle_absolute_command(
    DriverCore::Arm arm, const JointState::SharedPtr message) {
  const char *arm_name = arm == DriverCore::Arm::kLeft ? "left" : "right";
  if (message->position.size() != DriverCore::JointArray{}.size()) {
    RCLCPP_ERROR(
        get_logger(),
        "%s absolute command rejected: expected exactly 7 positions, received %zu",
        arm_name, message->position.size());
    return;
  }

  JointArray absolute{};
  std::copy(message->position.begin(), message->position.end(), absolute.begin());

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
  auto &last_command = arm == DriverCore::Arm::kLeft
                           ? left_abs_last_command_
                           : right_abs_last_command_;
  auto &last_command_time = arm == DriverCore::Arm::kLeft
                                ? left_abs_last_command_time_
                                : right_abs_last_command_time_;
  auto &streaming = arm == DriverCore::Arm::kLeft
                        ? left_abs_streaming_
                        : right_abs_streaming_;

  if (!streaming) {
    const auto state = core_.read_state();
    if (!state) {
      RCLCPP_ERROR(
          get_logger(),
          "%s absolute command rejected: no feedback to seed command session",
          arm_name);
      return;
    }
    last_command = DriverCore::clamp_to_limits(
        arm == DriverCore::Arm::kLeft ? state->left_position
                                      : state->right_position,
        minimum, maximum);
  }

  auto mapped = DriverCore::apply_joint_mapping(
      absolute, order, signs, offsets);
  if (const auto nan_j = DriverCore::first_non_finite(mapped)) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s absolute command rejected: j%zu=non-finite", arm_name,
        *nan_j + 1);
    last_command_time = now();
    streaming = false;
    return;
  }

  const auto desired = mapped;
  mapped = DriverCore::clamp_to_limits(desired, minimum, maximum);
  if (mapped != desired) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s absolute command clamped to joint limits", arm_name);
  }
  mapped = DriverCore::limit_delta(mapped, *last_command, max_delta_per_cycle_);
  mapped = DriverCore::clamp_to_limits(mapped, minimum, maximum);

  if (!core_.send_position(arm, mapped)) {
    RCLCPP_ERROR(
        get_logger(),
        "%s absolute command failed: SDK not ready / idle / mode rejected command",
        arm_name);
    return;
  }

  last_command = mapped;
  last_command_time = now();
  streaming = true;
  if (path_streaming(arm, false)) {
    RCLCPP_INFO(
        get_logger(),
        "%s absolute command takes over; invalidating relative teleop session",
        arm_name);
    reset_teleop_session(arm);
  }
}

void DriverNode::check_command_timeout() {
  const auto stamp = now();
  auto maybe_hold = [this, &stamp](
                        DriverCore::Arm arm, bool &streaming,
                        rclcpp::Time &last_command_time, const char *arm_name,
                        bool absolute) {
    if (!streaming) {
      return;
    }
    if ((stamp - last_command_time).seconds() <= command_timeout_s_) {
      return;
    }
    if (path_active(arm, !absolute)) {
      // The other command path owns this arm now; holding would fight it.
      RCLCPP_INFO(
          get_logger(),
          "%s command timeout (%.3f s); other command path is streaming, "
          "skipping hold",
          arm_name, command_timeout_s_);
      if (absolute) {
        reset_absolute_session(arm);
      } else {
        reset_teleop_session(arm);
      }
      return;
    }
    RCLCPP_WARN(
        get_logger(), "%s command timeout (%.3f s); hold %s arm only",
        arm_name, command_timeout_s_, arm_name);
    if (!core_.hold_current(arm)) {
      RCLCPP_ERROR(
          get_logger(), "hold_current(%s) after timeout failed", arm_name);
    }
    if (absolute) {
      reset_absolute_session(arm);
    } else {
      reset_teleop_session(arm);
    }
  };

  maybe_hold(
      DriverCore::Arm::kLeft, left_streaming_, left_last_command_time_, "left",
      false);
  maybe_hold(
      DriverCore::Arm::kRight, right_streaming_, right_last_command_time_,
      "right", false);
  maybe_hold(
      DriverCore::Arm::kLeft, left_abs_streaming_, left_abs_last_command_time_,
      "left absolute", true);
  maybe_hold(
      DriverCore::Arm::kRight, right_abs_streaming_,
      right_abs_last_command_time_, "right absolute", true);
}

void DriverNode::handle_hold_current(
    const std::shared_ptr<Trigger::Request> /*request*/,
    std::shared_ptr<Trigger::Response> response) {
  const bool ok = core_.hold_current();
  response->success = ok;
  response->message =
      ok ? "holding current joint positions" : "hold_current failed";
  if (ok) {
    reset_teleop_session(DriverCore::Arm::kLeft);
    reset_teleop_session(DriverCore::Arm::kRight);
    reset_absolute_session(DriverCore::Arm::kLeft);
    reset_absolute_session(DriverCore::Arm::kRight);
  }
}

void DriverNode::handle_stop_motion(
    const std::shared_ptr<Trigger::Request> /*request*/,
    std::shared_ptr<Trigger::Response> response) {
  const bool ok = core_.stop_motion();
  response->success = ok;
  response->message = ok ? "motion stopped; now IDLE" : "stop_motion failed";
  reset_teleop_session(DriverCore::Arm::kLeft);
  reset_teleop_session(DriverCore::Arm::kRight);
  reset_absolute_session(DriverCore::Arm::kLeft);
  reset_absolute_session(DriverCore::Arm::kRight);
}

void DriverNode::handle_emergency_stop(
    const std::shared_ptr<Trigger::Request> /*request*/,
    std::shared_ptr<Trigger::Response> response) {
  if (gripper_enabled_) {
    gripper_.stop();
    gripper_enabled_ = false;
  }
  const bool ok = core_.emergency_stop();
  response->success = ok;
  response->message = ok ? "emergency stop issued; now IDLE" : "emergency_stop failed";
  reset_teleop_session(DriverCore::Arm::kLeft);
  reset_teleop_session(DriverCore::Arm::kRight);
  reset_absolute_session(DriverCore::Arm::kLeft);
  reset_absolute_session(DriverCore::Arm::kRight);
}

void DriverNode::handle_gripper_command(
    DriverCore::Arm arm, const JointState::SharedPtr message) {
  if (!gripper_enabled_) {
    return;
  }
  if (message->position.empty() || !std::isfinite(message->position[0])) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "gripper command ignored: need finite position[0]");
    return;
  }
  gripper_.set_target(arm, factr_to_motor_norm(arm, message->position[0]));
}

bool DriverNode::gripper_invert_for(DriverCore::Arm arm) const {
  return arm == DriverCore::Arm::kLeft ? gripper_invert_left_
                                       : gripper_invert_right_;
}

double DriverNode::factr_to_motor_norm(
    DriverCore::Arm arm, double factr_norm) const {
  if (gripper_invert_for(arm)) {
    return 1.0 - factr_norm;
  }
  return factr_norm;
}

double DriverNode::motor_to_factr_norm(
    DriverCore::Arm arm, double motor_norm) const {
  if (gripper_invert_for(arm)) {
    return 1.0 - motor_norm;
  }
  return motor_norm;
}

void DriverNode::tick_gripper() {
  if (!gripper_enabled_) {
    return;
  }
  gripper_.tick_control();
  gripper_.tick_feedback();
  publish_gripper_state();
}

void DriverNode::publish_gripper_state() {
  auto publish_one = [this](
                         DriverCore::Arm arm,
                         const rclcpp::Publisher<JointState>::SharedPtr &pub,
                         const char *side) {
    const auto fb = gripper_.feedback(arm);
    JointState msg;
    msg.header.stamp = now();
    msg.name = {"gripper_joint"};
    if (fb.valid) {
      msg.header.frame_id = fb.frame_tag.empty() ? "gripper" : fb.frame_tag;
      msg.position = {motor_to_factr_norm(arm, fb.position)};
      msg.velocity = {fb.velocity};
      msg.effort = {fb.effort};
    } else {
      msg.header.frame_id = "no_feedback";
      msg.position = {motor_to_factr_norm(arm, gripper_.target(arm))};
      msg.velocity = {0.0};
      msg.effort = {0.0};
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "%s gripper (%s): no feedback (dev=%d). state.position is target "
          "echo, not hardware pose.",
          side, gripper_.type_name(arm), gripper_.device_id(arm));
    }
    pub->publish(msg);
  };
  publish_one(
      DriverCore::Arm::kLeft, left_gripper_state_publisher_, "left");
  publish_one(
      DriverCore::Arm::kRight, right_gripper_state_publisher_, "right");
}

void DriverNode::publish_state() {
  const auto state = core_.read_state();
  if (state) {
    const auto stamp = now();
    JointState message;
    message.header.stamp = stamp;
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
    left_state_publisher_->publish(make_arm_joint_state(
        stamp, kLeftJointNames, state->left_position, state->left_velocity));
    right_state_publisher_->publish(make_arm_joint_state(
        stamp, kRightJointNames, state->right_position,
        state->right_velocity));
  }

  std_msgs::msg::Int16MultiArray robot_state;
  robot_state.data = {
      static_cast<int16_t>(core_.current_state(DriverCore::Arm::kLeft)),
      static_cast<int16_t>(core_.current_state(DriverCore::Arm::kRight))};
  robot_state_publisher_->publish(robot_state);
}

}  // namespace skye_robot_driver
