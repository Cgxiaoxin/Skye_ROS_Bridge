#pragma once

#include <array>
#include <mutex>
#include <optional>

#include "L1Robot.h"

namespace skye_robot_driver {

// SDK core only. ROS node lives in driver_node. Degrees stay inside this class.
class DriverCore {
 public:
  enum class Arm { kLeft, kRight };
  using JointArray = std::array<double, 7>;

  struct DualArmState {
    JointArray left_position{};
    JointArray right_position{};
    JointArray left_velocity{};
    JointArray right_velocity{};
  };

  struct PdGains {
    JointArray k{{100, 100, 100, 100, 100, 100, 100}};
    JointArray d{{10, 10, 10, 10, 10, 10, 10}};
  };

  DriverCore() = default;
  DriverCore(const DriverCore &) = delete;
  DriverCore &operator=(const DriverCore &) = delete;

  static FXObjType sdk_object_for_arm(Arm arm);
  static bool validate_target(
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
      const std::array<unsigned char, 4> &ip, int left_ratio, int right_ratio,
      const PdGains &gains);
  bool command_allowed() const;
  bool hold_current();
  bool stop_motion();
  bool emergency_stop();
  bool send_pd_position(Arm arm, const JointArray &target_rad);
  std::optional<DualArmState> read_state() const;
  void shutdown();

 private:
  static constexpr unsigned int kThreadId = 1;
  static constexpr unsigned int kModeTimeoutMs = 3000;

  bool reset_errors_unlocked();
  bool enter_pd_unlocked(int left_ratio, int right_ratio, const PdGains &gains);

  mutable std::mutex mutex_;
  bool linked_{false};
  bool pd_ready_{false};
  int left_ratio_{10};
  int right_ratio_{10};
  PdGains gains_{};
};

}  // namespace skye_robot_driver
