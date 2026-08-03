#include "skye_robot_driver/driver_core.hpp"

namespace skye_robot_driver {

FXObjType DriverCore::sdk_object_for_arm(Arm arm) {
  return arm == Arm::kLeft ? FX_OBJ_ARM0 : FX_OBJ_ARM1;
}

bool DriverCore::connect(const std::array<unsigned char, 4> &ip) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (linked_) {
    return true;
  }
  const int ret =
      FX_L1_System_Link(ip[0], ip[1], ip[2], ip[3], FX_LOG_INFO_FLAG);
  linked_ = (ret >= 0);
  return linked_;
}

bool DriverCore::switch_to_pd(Arm arm, int timeout_ms) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_) {
    return false;
  }
  double k[7] = {100, 100, 100, 100, 100, 100, 100};
  double d[7] = {10, 10, 10, 10, 10, 10, 10};
  return FX_L1_State_SwitchToPDMode(
             sdk_object_for_arm(arm), timeout_ms, 10.0, 10.0, k, d) == 0;
}

bool DriverCore::send_pd_position(Arm arm, const JointArray &pos_deg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_) {
    return false;
  }
  double cmd[7];
  for (size_t i = 0; i < 7; ++i) {
    cmd[i] = pos_deg[i];
  }
  return FX_L1_Runtime_SetJointPosPDCmd(
             kThreadId, sdk_object_for_arm(arm), cmd) == 0;
}

std::optional<DriverCore::DualArmState> DriverCore::read_state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_) {
    return std::nullopt;
  }
  const ROBOT_RT *rt = FX_L1_Fbk_GetRT();
  if (rt == nullptr) {
    return std::nullopt;
  }
  DualArmState state;
  for (size_t i = 0; i < 7; ++i) {
    state.left_position[i] = rt->m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_Pos[i];
    state.right_position[i] = rt->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_Pos[i];
  }
  return state;
}

bool DriverCore::emergency_stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_) {
    return false;
  }
  return FX_L1_Runtime_EmergencyStop(kThreadId, FX_OBJ_ALL_FLAG) == 0;
}

void DriverCore::shutdown() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (linked_) {
    FX_L1_System_Unlink();
    linked_ = false;
  }
}

}  // namespace skye_robot_driver
