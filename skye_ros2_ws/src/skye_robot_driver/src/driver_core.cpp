#include "skye_robot_driver/driver_core.hpp"

#include <algorithm>
#include <cmath>

namespace skye_robot_driver {

FXObjType DriverCore::sdk_object_for_arm(Arm arm) {
  return arm == Arm::kLeft ? FX_OBJ_ARM0 : FX_OBJ_ARM1;
}

bool DriverCore::validate_target(
    const JointArray &target, const JointArray &minimum,
    const JointArray &maximum) {
  for (std::size_t i = 0; i < target.size(); ++i) {
    if (!std::isfinite(target[i]) || target[i] < minimum[i] ||
        target[i] > maximum[i]) {
      return false;
    }
  }
  return true;
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
  // Best-effort clear for both arms (ResetError takes FXObjType, not mask).
  FX_L1_State_ResetError(FX_OBJ_ARM0, kModeTimeoutMs, &code);
  FX_L1_State_ResetError(FX_OBJ_ARM1, kModeTimeoutMs, &code);
  return true;
}

bool DriverCore::enter_pd_unlocked(
    int left_ratio, int right_ratio, const PdGains &gains) {
  double left_k[7];
  double left_d[7];
  double right_k[7];
  double right_d[7];
  for (std::size_t i = 0; i < 7; ++i) {
    left_k[i] = gains.k[i];
    left_d[i] = gains.d[i];
    right_k[i] = gains.k[i];
    right_d[i] = gains.d[i];
  }

  FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, kModeTimeoutMs);
  FX_L1_State_SwitchToIdle(FX_OBJ_ARM1, kModeTimeoutMs);

  if (FX_L1_State_SwitchToPDMode(
          FX_OBJ_ARM0, kModeTimeoutMs, left_ratio, left_ratio, left_k,
          left_d) != 0) {
    return false;
  }
  if (FX_L1_State_SwitchToPDMode(
          FX_OBJ_ARM1, kModeTimeoutMs, right_ratio, right_ratio, right_k,
          right_d) != 0) {
    return false;
  }
  if (FX_L1_Runtime_SetSpeedRatio(
          kThreadId, FX_OBJ_ARM0, left_ratio, left_ratio) != 0) {
    return false;
  }
  if (FX_L1_Runtime_SetSpeedRatio(
          kThreadId, FX_OBJ_ARM1, right_ratio, right_ratio) != 0) {
    return false;
  }
  return true;
}

bool DriverCore::connect_and_enable(
    const std::array<unsigned char, 4> &ip, int left_ratio, int right_ratio,
    const PdGains &gains) {
  if (left_ratio < 1 || left_ratio > 100 || right_ratio < 1 ||
      right_ratio > 100) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (linked_) {
    return pd_ready_;
  }

  const int link_result =
      FX_L1_System_Link(ip[0], ip[1], ip[2], ip[3], FX_LOG_INFO_FLAG);
  if (link_result < 0) {
    return false;
  }
  linked_ = true;

  const auto fail_and_disconnect = [this]() {
    pd_ready_ = false;
    FX_L1_Runtime_StopTraj(kThreadId, FX_OBJ_ALL_FLAG);
    FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, kModeTimeoutMs);
    FX_L1_State_SwitchToIdle(FX_OBJ_ARM1, kModeTimeoutMs);
    FX_L1_System_Unlink();
    linked_ = false;
    return false;
  };

  // Link → ResetError (best-effort) → Idle → PD
  reset_errors_unlocked();
  if (!enter_pd_unlocked(left_ratio, right_ratio, gains)) {
    reset_errors_unlocked();
    if (!enter_pd_unlocked(left_ratio, right_ratio, gains)) {
      return fail_and_disconnect();
    }
  }

  left_ratio_ = left_ratio;
  right_ratio_ = right_ratio;
  gains_ = gains;
  pd_ready_ = true;
  return true;
}

bool DriverCore::command_allowed() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return linked_ && pd_ready_;
}

bool DriverCore::hold_current() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_) {
    return false;
  }

  const ROBOT_RT *feedback = FX_L1_Fbk_GetRT();
  if (feedback == nullptr) {
    return false;
  }

  JointArray left_deg{};
  JointArray right_deg{};
  for (std::size_t i = 0; i < left_deg.size(); ++i) {
    left_deg[i] = feedback->m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_Pos[i];
    right_deg[i] = feedback->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_Pos[i];
  }

  if (!pd_ready_) {
    if (!enter_pd_unlocked(left_ratio_, right_ratio_, gains_)) {
      return false;
    }
  }

  const bool left_ok =
      FX_L1_Runtime_SetJointPosPDCmd(
          kThreadId, FX_OBJ_ARM0, left_deg.data()) == 0;
  const bool right_ok =
      FX_L1_Runtime_SetJointPosPDCmd(
          kThreadId, FX_OBJ_ARM1, right_deg.data()) == 0;
  if (left_ok && right_ok) {
    pd_ready_ = true;
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
  pd_ready_ = false;
  return true;
}

bool DriverCore::emergency_stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_) {
    return false;
  }
  const bool ok =
      FX_L1_Runtime_EmergencyStop(kThreadId, FX_OBJ_ALL_FLAG) == 0;
  pd_ready_ = false;
  return ok;
}

bool DriverCore::send_pd_position(Arm arm, const JointArray &target_rad) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_ || !pd_ready_) {
    return false;
  }
  auto command = ros_radians_to_sdk_degrees(target_rad);
  return FX_L1_Runtime_SetJointPosPDCmd(
             kThreadId, sdk_object_for_arm(arm), command.data()) == 0;
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

void DriverCore::shutdown() {
  std::lock_guard<std::mutex> lock(mutex_);
  pd_ready_ = false;
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
