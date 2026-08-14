#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "L1Robot.h"

namespace skye_robot_driver {

// SDK core only. ROS node lives in driver_node. Degrees stay inside this class.
class DriverCore {
 public:
  enum class Arm { kLeft, kRight };

  struct TerminalPacket {
    FXChnType chn{FX_CHN_CANFD};
    std::vector<std::uint8_t> data;
    unsigned int receiving_time_ms{0};
  };

  // Aligned with FXStateType for 0..3; PD kept as optional escape hatch.
  enum class ControlMode {
    kIdle = 0,
    kPosition = 1,
    kImpJoint = 2,
    kImpCart = 3,
    kPd = 11,
  };

  using JointArray = std::array<double, 7>;

  struct DualArmState {
    JointArray left_position{};
    JointArray right_position{};
    JointArray left_velocity{};
    JointArray right_velocity{};
  };

  struct ImpedanceGains {
    JointArray k{{100, 100, 100, 100, 100, 100, 100}};
    JointArray d{{10, 10, 10, 10, 10, 10, 10}};
  };

  struct ConnectConfig {
    ControlMode mode{ControlMode::kImpJoint};
    int left_vel_ratio{10};
    int left_acc_ratio{10};
    int right_vel_ratio{10};
    int right_acc_ratio{10};
    int cmd_cycle_time_ms{4};
    ImpedanceGains joint_gains{};
    ImpedanceGains cart_gains{};
  };

  DriverCore() = default;
  DriverCore(const DriverCore &) = delete;
  DriverCore &operator=(const DriverCore &) = delete;

  static FXObjType sdk_object_for_arm(Arm arm);
  static const char *mode_name(ControlMode mode);
  static std::optional<ControlMode> mode_from_int(int value);
  static bool validate_target(
      const JointArray &target, const JointArray &minimum,
      const JointArray &maximum);
  // First 1-based joint index that fails validate_target, if any.
  static std::optional<std::size_t> first_invalid_joint(
      const JointArray &target, const JointArray &minimum,
      const JointArray &maximum);
  static JointArray apply_joint_mapping(
      const JointArray &leader, const std::array<int, 7> &joint_order,
      const JointArray &signs, const JointArray &offsets);
  static JointArray limit_delta(
      const JointArray &desired, const JointArray &previous,
      double max_delta_per_cycle);
  static JointArray ros_radians_to_sdk_degrees(const JointArray &radians);
  static JointArray sdk_degrees_to_ros_radians(const JointArray &degrees);

  bool connect_and_enable(
      const std::array<unsigned char, 4> &ip, const ConnectConfig &config);
  bool switch_control_mode(ControlMode mode);
  bool command_allowed() const;
  ControlMode control_mode() const;
  FXStateType current_state(Arm arm) const;
  bool hold_current();
  bool hold_current(Arm arm);
  bool stop_motion();
  bool emergency_stop();
  bool send_position(Arm arm, const JointArray &target_rad);
  std::optional<DualArmState> read_state() const;
  std::optional<int> get_cmd_cycle_time_ms() const;
  bool linked() const;

  // End-effector terminal passthrough (CANFD / 485). Requires linked_.
  bool terminal_clear(FXTerminalType terminal);
  bool terminal_set(
      FXTerminalType terminal, FXChnType chn,
      const std::uint8_t *data, std::size_t len,
      unsigned int timeout_ms = 100);
  std::optional<TerminalPacket> terminal_get(
      FXTerminalType terminal, unsigned int timeout_ms = 100);

  void shutdown();

 private:
  static constexpr unsigned int kThreadId = 1;
  static constexpr unsigned int kModeTimeoutMs = 3000;

  bool reset_errors_unlocked();
  bool enter_mode_unlocked(ControlMode mode);
  bool send_position_unlocked(Arm arm, const JointArray &target_rad);
  bool hold_current_arm_unlocked(Arm arm);

  mutable std::mutex mutex_;
  bool linked_{false};
  bool control_ready_{false};
  ControlMode mode_{ControlMode::kImpJoint};
  ConnectConfig config_{};
};

}  // namespace skye_robot_driver
