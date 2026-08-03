#pragma once

#include <array>
#include <mutex>
#include <optional>

#include "L1Robot.h"

namespace skye_robot_driver {

// SDK core only. ROS node lives in driver_node.
class DriverCore {
 public:
  enum class Arm { kLeft, kRight };
  using JointArray = std::array<double, 7>;

  struct DualArmState {
    JointArray left_position{};
    JointArray right_position{};
  };

  DriverCore() = default;
  DriverCore(const DriverCore &) = delete;
  DriverCore &operator=(const DriverCore &) = delete;

  static FXObjType sdk_object_for_arm(Arm arm);

  bool connect(const std::array<unsigned char, 4> &ip);
  bool switch_to_pd(Arm arm, int timeout_ms = 3000);
  bool send_pd_position(Arm arm, const JointArray &pos_deg);
  std::optional<DualArmState> read_state() const;
  bool emergency_stop();
  void shutdown();

 private:
  static constexpr unsigned int kThreadId = 1;

  mutable std::mutex mutex_;
  bool linked_{false};
};

}  // namespace skye_robot_driver
