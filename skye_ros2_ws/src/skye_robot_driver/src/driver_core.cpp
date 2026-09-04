#include "skye_robot_driver/driver_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace skye_robot_driver {

FXObjType DriverCore::sdk_object_for_arm(Arm arm) {
  return arm == Arm::kLeft ? FX_OBJ_ARM0 : FX_OBJ_ARM1;
}

const char *DriverCore::mode_name(ControlMode mode) {
  switch (mode) {
    case ControlMode::kIdle:
      return "idle";
    case ControlMode::kPosition:
      return "position";
    case ControlMode::kImpJoint:
      return "imp_joint";
    case ControlMode::kImpCart:
      return "imp_cart";
    case ControlMode::kPd:
      return "pd";
  }
  return "unknown";
}

std::optional<DriverCore::ControlMode> DriverCore::mode_from_int(int value) {
  switch (value) {
    case 0:
      return ControlMode::kIdle;
    case 1:
      return ControlMode::kPosition;
    case 2:
      return ControlMode::kImpJoint;
    case 3:
      return ControlMode::kImpCart;
    case 11:
      return ControlMode::kPd;
    default:
      return std::nullopt;
  }
}

bool DriverCore::validate_target(
    const JointArray &target, const JointArray &minimum,
    const JointArray &maximum) {
  return !first_invalid_joint(target, minimum, maximum).has_value();
}

std::optional<std::size_t> DriverCore::first_invalid_joint(
    const JointArray &target, const JointArray &minimum,
    const JointArray &maximum) {
  for (std::size_t i = 0; i < target.size(); ++i) {
    if (!std::isfinite(target[i]) || target[i] < minimum[i] ||
        target[i] > maximum[i]) {
      return i;
    }
  }
  return std::nullopt;
}

std::optional<std::size_t> DriverCore::first_non_finite(
    const JointArray &target) {
  for (std::size_t i = 0; i < target.size(); ++i) {
    if (!std::isfinite(target[i])) {
      return i;
    }
  }
  return std::nullopt;
}

DriverCore::JointArray DriverCore::clamp_to_limits(
    const JointArray &target, const JointArray &minimum,
    const JointArray &maximum) {
  JointArray clamped{};
  for (std::size_t i = 0; i < target.size(); ++i) {
    if (!std::isfinite(target[i])) {
      clamped[i] = target[i];
      continue;
    }
    clamped[i] = std::clamp(target[i], minimum[i], maximum[i]);
  }
  return clamped;
}

DriverCore::JointArray DriverCore::apply_joint_mapping(
    const JointArray &leader, const std::array<int, 7> &joint_order,
    const JointArray &signs, const JointArray &offsets) {
  JointArray mapped{};
  for (std::size_t out = 0; out < mapped.size(); ++out) {
    const int src = joint_order[out];
    mapped[out] =
        leader[static_cast<std::size_t>(src)] * signs[out] + offsets[out];
  }
  return mapped;
}

DriverCore::JointArray DriverCore::apply_relative_joint_mapping(
    const JointArray &leader_now, const JointArray &leader_prev,
    JointArray &leader_continuous, const JointArray &leader_cont_ref,
    const JointArray &follower_ref, const std::array<int, 7> &joint_order,
    const JointArray &signs) {
  constexpr double kPi = 3.14159265358979323846;
  JointArray mapped{};
  for (std::size_t out = 0; out < mapped.size(); ++out) {
    const int src = joint_order[out];
    const std::size_t s = static_cast<std::size_t>(src);
    // Unwrap the frame-to-frame delta: at the control rate per-frame motion is
    // far below pi, so a jump beyond pi means the reported angle wrapped by
    // 2*pi (e.g. leader J1/J3 crossing +-pi). Correct it to keep tracking the
    // physical angle. Legitimate slow swings accumulate over many small frames.
    double frame_delta = leader_now[s] - leader_prev[s];
    if (frame_delta > kPi) {
      frame_delta -= 2.0 * kPi;
    } else if (frame_delta < -kPi) {
      frame_delta += 2.0 * kPi;
    }
    leader_continuous[s] += frame_delta;
    const double delta = leader_continuous[s] - leader_cont_ref[s];
    mapped[out] = follower_ref[out] + signs[out] * delta;
  }
  return mapped;
}

bool DriverCore::clutch_saturated_joints(
    const JointArray &desired, const JointArray &clamped,
    const JointArray &leader_continuous, JointArray &leader_cont_ref,
    JointArray &gento_ref, const std::array<int, 7> &joint_order) {
  bool clutched = false;
  for (std::size_t out = 0; out < desired.size(); ++out) {
    if (desired[out] == clamped[out]) {
      continue;
    }
    const auto src = static_cast<std::size_t>(joint_order[out]);
    leader_cont_ref[src] = leader_continuous[src];
    gento_ref[out] = clamped[out];
    clutched = true;
  }
  return clutched;
}

bool DriverCore::delta_was_limited(
    const JointArray &desired, const JointArray &limited) {
  for (std::size_t i = 0; i < desired.size(); ++i) {
    if (desired[i] != limited[i]) {
      return true;
    }
  }
  return false;
}

DriverCore::JointArray DriverCore::limit_delta(
    const JointArray &desired, const JointArray &previous,
    double max_delta_per_cycle) {
  JointArray limited{};
  for (std::size_t i = 0; i < desired.size(); ++i) {
    const double delta = desired[i] - previous[i];
    if (delta > max_delta_per_cycle) {
      limited[i] = previous[i] + max_delta_per_cycle;
    } else if (delta < -max_delta_per_cycle) {
      limited[i] = previous[i] - max_delta_per_cycle;
    } else {
      limited[i] = desired[i];
    }
  }
  return limited;
}

DriverCore::JointArray DriverCore::ros_radians_to_sdk_degrees(
    const JointArray &radians) {
  JointArray degrees{};
  constexpr double kDegreesPerRadian = 180.0 / 3.14159265358979323846;
  std::transform(
      radians.begin(), radians.end(), degrees.begin(),
      [kDegreesPerRadian](double value) { return value * kDegreesPerRadian; });
  return degrees;
}

DriverCore::JointArray DriverCore::sdk_degrees_to_ros_radians(
    const JointArray &degrees) {
  JointArray radians{};
  constexpr double kRadiansPerDegree = 3.14159265358979323846 / 180.0;
  std::transform(
      degrees.begin(), degrees.end(), radians.begin(),
      [kRadiansPerDegree](double value) { return value * kRadiansPerDegree; });
  return radians;
}

bool DriverCore::reset_errors_unlocked() {
  unsigned int code = 0;
  FX_L1_State_ResetError(FX_OBJ_ARM0, kModeTimeoutMs, &code);
  FX_L1_State_ResetError(FX_OBJ_ARM1, kModeTimeoutMs, &code);
  return true;
}

bool DriverCore::enter_mode_unlocked(ControlMode mode) {
  double joint_k_l[7];
  double joint_d_l[7];
  double joint_k_r[7];
  double joint_d_r[7];
  double cart_k_l[7];
  double cart_d_l[7];
  double cart_k_r[7];
  double cart_d_r[7];
  for (std::size_t i = 0; i < 7; ++i) {
    joint_k_l[i] = config_.joint_gains.k[i];
    joint_d_l[i] = config_.joint_gains.d[i];
    joint_k_r[i] = config_.joint_gains.k[i];
    joint_d_r[i] = config_.joint_gains.d[i];
    cart_k_l[i] = config_.cart_gains.k[i];
    cart_d_l[i] = config_.cart_gains.d[i];
    cart_k_r[i] = config_.cart_gains.k[i];
    cart_d_r[i] = config_.cart_gains.d[i];
  }

  // Always park in Idle before switching (except when target is Idle).
  FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, kModeTimeoutMs);
  FX_L1_State_SwitchToIdle(FX_OBJ_ARM1, kModeTimeoutMs);

  if (mode == ControlMode::kIdle) {
    mode_ = ControlMode::kIdle;
    control_ready_ = false;
    return true;
  }

  const double left_vel = static_cast<double>(config_.left_vel_ratio);
  const double left_acc = static_cast<double>(config_.left_acc_ratio);
  const double right_vel = static_cast<double>(config_.right_vel_ratio);
  const double right_acc = static_cast<double>(config_.right_acc_ratio);

  bool ok = false;
  switch (mode) {
    case ControlMode::kPosition:
      ok = FX_L1_State_SwitchToPositionMode(
               FX_OBJ_ARM0, kModeTimeoutMs, left_vel, left_acc) == 0 &&
           FX_L1_State_SwitchToPositionMode(
               FX_OBJ_ARM1, kModeTimeoutMs, right_vel, right_acc) == 0;
      break;
    case ControlMode::kImpJoint: {
      const int arm0 = FX_L1_State_SwitchToImpJointMode(
          FX_OBJ_ARM0, kModeTimeoutMs, left_vel, left_acc, joint_k_l, joint_d_l);
      if (arm0 != 0) {
        last_error_ = "Arm0 SwitchToImpJointMode failed, ret=" + std::to_string(arm0);
        ok = false;
        break;
      }
      const int arm1 = FX_L1_State_SwitchToImpJointMode(
          FX_OBJ_ARM1, kModeTimeoutMs, right_vel, right_acc, joint_k_r, joint_d_r);
      if (arm1 != 0) {
        last_error_ = "Arm1 SwitchToImpJointMode failed, ret=" + std::to_string(arm1);
        ok = false;
        break;
      }
      ok = true;
      break;
    }
    case ControlMode::kImpCart:
      ok = FX_L1_State_SwitchToImpCartMode(
               FX_OBJ_ARM0, kModeTimeoutMs, left_vel, left_acc, cart_k_l,
               cart_d_l) == 0 &&
           FX_L1_State_SwitchToImpCartMode(
               FX_OBJ_ARM1, kModeTimeoutMs, right_vel, right_acc, cart_k_r,
               cart_d_r) == 0;
      break;
    case ControlMode::kPd:
      ok = FX_L1_State_SwitchToPDMode(
               FX_OBJ_ARM0, kModeTimeoutMs, left_vel, left_acc, joint_k_l,
               joint_d_l) == 0 &&
           FX_L1_State_SwitchToPDMode(
               FX_OBJ_ARM1, kModeTimeoutMs, right_vel, right_acc, joint_k_r,
               joint_d_r) == 0;
      break;
    case ControlMode::kIdle:
      ok = true;
      break;
  }
  if (!ok) {
    return false;
  }

  if (mode != ControlMode::kIdle) {
    if (FX_L1_Runtime_SetSpeedRatio(
            kThreadId, FX_OBJ_ARM0, left_vel, left_acc) != 0) {
      return false;
    }
    if (FX_L1_Runtime_SetSpeedRatio(
            kThreadId, FX_OBJ_ARM1, right_vel, right_acc) != 0) {
      return false;
    }
  }

  mode_ = mode;
  control_ready_ = (mode != ControlMode::kIdle);
  return true;
}

bool DriverCore::send_position_unlocked(Arm arm, const JointArray &target_rad) {
  auto command = ros_radians_to_sdk_degrees(target_rad);
  switch (mode_) {
    case ControlMode::kIdle:
      return false;
    case ControlMode::kPd:
      return FX_L1_Runtime_SetJointPosPDCmd(
                 kThreadId, sdk_object_for_arm(arm), command.data()) == 0;
    case ControlMode::kPosition:
    case ControlMode::kImpJoint:
    case ControlMode::kImpCart:
      // Joint teleop path. ImpCart typically wants Cartesian targets; joint
      // cmds remain available for nullspace / interim use.
      return FX_L1_Runtime_SetJointPosCmd(
                 kThreadId, sdk_object_for_arm(arm), command.data()) == 0;
  }
  return false;
}

bool DriverCore::link_controller(const std::array<unsigned char, 4> &ip) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (linked_) {
    return true;
  }
  const int link_result =
      FX_L1_System_Link(ip[0], ip[1], ip[2], ip[3], FX_LOG_INFO_FLAG);
  if (link_result < 0) {
    return false;
  }
  linked_ = true;
  return true;
}

bool DriverCore::apply_comm_config(int cmd_cycle_time_ms) {
  if (cmd_cycle_time_ms <= 0) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_) {
    return false;
  }
  if (FX_L1_Config_SetPDCmdCycleTime(cmd_cycle_time_ms) != 0) {
    // Non-fatal on some firmwares.
  }
  return true;
}

bool DriverCore::configure_and_enable(const ConnectConfig &config) {
  auto ratio_ok = [](int value) { return value >= 1 && value <= 100; };
  if (!ratio_ok(config.left_vel_ratio) || !ratio_ok(config.left_acc_ratio) ||
      !ratio_ok(config.right_vel_ratio) || !ratio_ok(config.right_acc_ratio)) {
    last_error_ = "invalid velocity/acceleration ratio";
    return false;
  }
  if (config.cmd_cycle_time_ms <= 0) {
    last_error_ = "invalid cmd_cycle_time_ms";
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_) {
    last_error_ = "SDK not linked";
    return false;
  }
  config_ = config;

  const auto fail_and_disconnect = [this]() {
    control_ready_ = false;
    FX_L1_Runtime_StopTraj(kThreadId, FX_OBJ_ALL_FLAG);
    FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, kModeTimeoutMs);
    FX_L1_State_SwitchToIdle(FX_OBJ_ARM1, kModeTimeoutMs);
    FX_L1_System_Unlink();
    linked_ = false;
    return false;
  };

  if (FX_L1_Config_SetPDCmdCycleTime(config_.cmd_cycle_time_ms) != 0) {
    // Non-fatal on some firmwares.
  }

  reset_errors_unlocked();
  if (!enter_mode_unlocked(config_.mode)) {
    reset_errors_unlocked();
    if (!enter_mode_unlocked(config_.mode)) {
      if (last_error_.empty()) {
        last_error_ = "enter_mode failed after retry";
      }
      return fail_and_disconnect();
    }
  }
  last_error_.clear();
  return true;
}

bool DriverCore::connect_and_enable(
    const std::array<unsigned char, 4> &ip, const ConnectConfig &config) {
  if (!link_controller(ip)) {
    return false;
  }
  return configure_and_enable(config);
}

bool DriverCore::switch_control_mode(ControlMode mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_) {
    return false;
  }
  reset_errors_unlocked();
  if (!enter_mode_unlocked(mode)) {
    control_ready_ = false;
    return false;
  }
  return true;
}

bool DriverCore::set_speed_rates(int left_vel, int left_acc, int right_vel,
                                 int right_acc) {
  auto ok_r = [](int v) { return v >= 1 && v <= 100; };
  if (!ok_r(left_vel) || !ok_r(left_acc) || !ok_r(right_vel) ||
      !ok_r(right_acc)) {
    last_error_ = "motion rates must be in [1,100]";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_ || !control_ready_) {
    last_error_ = "SDK not ready for SetSpeedRatio";
    return false;
  }
  if (FX_L1_Runtime_SetSpeedRatio(
          kThreadId, FX_OBJ_ARM0, left_vel, left_acc) != 0) {
    last_error_ = "ARM0 SetSpeedRatio failed";
    return false;
  }
  if (FX_L1_Runtime_SetSpeedRatio(
          kThreadId, FX_OBJ_ARM1, right_vel, right_acc) != 0) {
    last_error_ = "ARM1 SetSpeedRatio failed";
    return false;
  }
  config_.left_vel_ratio = left_vel;
  config_.left_acc_ratio = left_acc;
  config_.right_vel_ratio = right_vel;
  config_.right_acc_ratio = right_acc;
  last_error_.clear();
  return true;
}

bool DriverCore::command_allowed() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return linked_ && control_ready_ && mode_ != ControlMode::kIdle;
}

DriverCore::ControlMode DriverCore::control_mode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

FXStateType DriverCore::current_state(Arm arm) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_) {
    return FX_STATE_UNKNOWN;
  }
  return FX_L1_Fbk_CurrentState(sdk_object_for_arm(arm));
}

bool DriverCore::hold_current_arm_unlocked(Arm arm) {
  const ROBOT_RT *feedback = FX_L1_Fbk_GetRT();
  if (feedback == nullptr) {
    return false;
  }

  const int arm_idx = arm == Arm::kLeft ? 0 : 1;
  JointArray deg{};
  for (std::size_t i = 0; i < deg.size(); ++i) {
    deg[i] = feedback->m_ARMS[arm_idx].m_ARM_OUT.m_ARM_FBK_Joint_Pos[i];
  }
  return send_position_unlocked(arm, sdk_degrees_to_ros_radians(deg));
}

bool DriverCore::hold_current(Arm arm) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_ || mode_ == ControlMode::kIdle) {
    return false;
  }
  if (!control_ready_ && !enter_mode_unlocked(mode_)) {
    return false;
  }
  const bool ok = hold_current_arm_unlocked(arm);
  if (ok) {
    control_ready_ = true;
  }
  return ok;
}

bool DriverCore::hold_current() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_ || mode_ == ControlMode::kIdle) {
    return false;
  }
  if (!control_ready_ && !enter_mode_unlocked(mode_)) {
    return false;
  }
  const bool left_ok = hold_current_arm_unlocked(Arm::kLeft);
  const bool right_ok = hold_current_arm_unlocked(Arm::kRight);
  if (left_ok && right_ok) {
    control_ready_ = true;
    return true;
  }
  return false;
}

bool DriverCore::stop_motion() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_) {
    return false;
  }
  FX_L1_Runtime_StopTraj(kThreadId, FX_OBJ_ALL_FLAG);
  FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, kModeTimeoutMs);
  FX_L1_State_SwitchToIdle(FX_OBJ_ARM1, kModeTimeoutMs);
  mode_ = ControlMode::kIdle;
  control_ready_ = false;
  return true;
}

bool DriverCore::emergency_stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_) {
    return false;
  }
  const bool ok =
      FX_L1_Runtime_EmergencyStop(kThreadId, FX_OBJ_ALL_FLAG) == 0;
  mode_ = ControlMode::kIdle;
  control_ready_ = false;
  return ok;
}

bool DriverCore::send_position(Arm arm, const JointArray &target_rad) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_ || !control_ready_ || mode_ == ControlMode::kIdle) {
    return false;
  }
  return send_position_unlocked(arm, target_rad);
}

std::optional<DriverCore::DualArmState> DriverCore::read_state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_) {
    return std::nullopt;
  }

  const ROBOT_RT *feedback = FX_L1_Fbk_GetRT();
  if (feedback == nullptr) {
    return std::nullopt;
  }

  JointArray left_position_degrees{};
  JointArray right_position_degrees{};
  JointArray left_velocity_degrees{};
  JointArray right_velocity_degrees{};
  for (std::size_t i = 0; i < left_position_degrees.size(); ++i) {
    left_position_degrees[i] =
        feedback->m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_Pos[i];
    right_position_degrees[i] =
        feedback->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_Pos[i];
    left_velocity_degrees[i] =
        feedback->m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_Vel[i];
    right_velocity_degrees[i] =
        feedback->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_Vel[i];
  }

  DualArmState state;
  state.left_position = sdk_degrees_to_ros_radians(left_position_degrees);
  state.right_position = sdk_degrees_to_ros_radians(right_position_degrees);
  state.left_velocity = sdk_degrees_to_ros_radians(left_velocity_degrees);
  state.right_velocity = sdk_degrees_to_ros_radians(right_velocity_degrees);
  return state;
}

std::optional<int> DriverCore::get_cmd_cycle_time_ms() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_) {
    return std::nullopt;
  }
  int cycle = 0;
  if (FX_L1_Config_GetPDCmdCycleTime(&cycle) != 0) {
    return std::nullopt;
  }
  return cycle;
}

bool DriverCore::linked() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return linked_;
}

const std::string &DriverCore::last_error() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_;
}

bool DriverCore::terminal_clear(FXTerminalType terminal) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_) {
    return false;
  }
  return FX_L1_Terminal_ClearData(terminal) == 0;
}

bool DriverCore::terminal_set(
    FXTerminalType terminal, FXChnType chn, const std::uint8_t *data,
    std::size_t len, unsigned int timeout_ms) {
  if (data == nullptr || len == 0 || len > 64) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_) {
    return false;
  }
  unsigned char buffer[64]{};
  std::copy(data, data + len, buffer);
  unsigned int sending_time = 0;
  return FX_L1_Terminal_SetData(
             terminal, chn, timeout_ms, buffer,
             static_cast<unsigned int>(len), &sending_time) == 0;
}

std::optional<DriverCore::TerminalPacket> DriverCore::terminal_get(
    FXTerminalType terminal, unsigned int timeout_ms) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_) {
    return std::nullopt;
  }
  FXChnType chn = FX_CHN_CANFD;
  unsigned char buffer[64]{};
  unsigned int receiving_time = 0;
  const int n = FX_L1_Terminal_GetData(
      terminal, timeout_ms, &chn, buffer, &receiving_time);
  if (n < 0) {
    return std::nullopt;
  }
  TerminalPacket packet;
  packet.chn = chn;
  packet.receiving_time_ms = receiving_time;
  packet.data.assign(buffer, buffer + n);
  return packet;
}

void DriverCore::shutdown() {
  std::lock_guard<std::mutex> lock(mutex_);
  control_ready_ = false;
  mode_ = ControlMode::kIdle;
  if (!linked_) {
    return;
  }

  FX_L1_Runtime_StopTraj(kThreadId, FX_OBJ_ALL_FLAG);
  FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, kModeTimeoutMs);
  FX_L1_State_SwitchToIdle(FX_OBJ_ARM1, kModeTimeoutMs);
  FX_L1_System_Unlink();
  linked_ = false;
}

}  // namespace skye_robot_driver
