#include "skye_robot_driver/gripper_common.hpp"

#include <algorithm>
#include <cctype>

namespace skye_robot_driver {

namespace {

std::string lower_copy(std::string value) {
  std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

}  // namespace

std::optional<GripperDriverType> parse_gripper_type(const std::string &value) {
  const auto key = lower_copy(value);
  if (key == "dm4310" || key == "dm" || key == "damiao") {
    return GripperDriverType::kDm4310;
  }
  if (key == "robotiq" || key == "hande" || key == "hand-e") {
    return GripperDriverType::kRobotiq;
  }
  return std::nullopt;
}

}  // namespace skye_robot_driver
