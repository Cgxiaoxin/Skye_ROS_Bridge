#pragma once

#include <array>
#include <mutex>
#include <optional>

#include "L1Robot.h"

namespace gento_robot_driver {

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

  DriverCore() = default;
  DriverCore(const DriverCore&) = delete;
  DriverCore& operator=(const DriverCore&) = delete;

  static FXObjType sdk_object_for_arm(Arm arm);
  static bool validate_target(
      const JointArray& target,
      const JointArray& minimum,
      const JointArray& maximum);
  static JointArray apply_joint_mapping(
      const JointArray& leader,
      const std::array<int, 7>& joint_order,
      const JointArray& signs,
      const JointArray& offsets);
  static JointArray limit_delta(
      const JointArray& desired,
      const JointArray& previous,
      double max_delta_per_cycle);
  static JointArray ros_radians_to_sdk_degrees(const JointArray& radians);
  static JointArray sdk_degrees_to_ros_radians(const JointArray& degrees);

  bool connect_and_enable(
      const std::array<unsigned char, 4>& ip,
      int left_ratio,
      int right_ratio);
  bool command_allowed() const;
  bool hold_current();
  bool stop_motion();
  bool send_position(Arm arm, const JointArray& target);
  std::optional<DualArmState> read_state() const;
  void shutdown();

 private:
  static constexpr unsigned int kThreadId = 1;
  static constexpr unsigned int kModeTimeoutMs = 3000;

  mutable std::mutex mutex_;
  bool linked_{false};
  bool position_ready_{false};
  int left_ratio_{10};
  int right_ratio_{10};
};

}  // namespace gento_robot_driver
