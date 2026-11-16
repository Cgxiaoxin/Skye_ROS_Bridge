#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace skye_robot_driver {

enum class GripperDriverType { kDm4310, kRobotiq };

struct GripperFeedback {
  bool valid{false};
  // Normalized [0,1]: 0=open, 1=closed (FACTR / driver convention).
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  int err_code{0};
  std::uint32_t device_id{0};
  std::string frame_tag;
};

std::optional<GripperDriverType> parse_gripper_type(const std::string &value);

// Move current toward target by at most max_delta. max_delta <= 0 snaps to target
// (legacy / disabled rate limit).
double slew_toward(double current, double target, double max_delta);

// Next Hand-E opening command (mm). Large |goal-current| (>= one_shot_delta_mm)
// snaps so Robotiq runs one continuous profile (fast + smooth). Smaller errors
// use slew when max_slew_mm_s > 0. one_shot_delta_mm <= 0 disables the snap path.
double next_robotiq_command_mm(
    double current_mm, double goal_mm, double max_slew_mm_s, double dt_s,
    double one_shot_delta_mm);

}  // namespace skye_robot_driver
