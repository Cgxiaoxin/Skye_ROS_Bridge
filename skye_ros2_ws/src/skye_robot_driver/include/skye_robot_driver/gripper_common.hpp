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

}  // namespace skye_robot_driver
