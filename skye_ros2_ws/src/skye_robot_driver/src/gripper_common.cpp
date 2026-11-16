#include "skye_robot_driver/gripper_common.hpp"

#include <algorithm>
#include <cmath>
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

double slew_toward(double current, double target, double max_delta) {
  if (!(max_delta > 0.0) || !std::isfinite(max_delta)) {
    return target;
  }
  if (!std::isfinite(current) || !std::isfinite(target)) {
    return target;
  }
  const double delta = target - current;
  if (std::abs(delta) <= max_delta) {
    return target;
  }
  return current + std::copysign(max_delta, delta);
}

double next_robotiq_command_mm(
    double current_mm, double goal_mm, double max_slew_mm_s, double dt_s,
    double one_shot_delta_mm) {
  if (!std::isfinite(current_mm) || !std::isfinite(goal_mm)) {
    return goal_mm;
  }
  const double err = std::abs(goal_mm - current_mm);
  if (one_shot_delta_mm > 0.0 && std::isfinite(one_shot_delta_mm) &&
      err >= one_shot_delta_mm) {
    return goal_mm;
  }
  if (!(dt_s > 0.0) || !std::isfinite(dt_s)) {
    return goal_mm;
  }
  return slew_toward(current_mm, goal_mm, max_slew_mm_s * dt_s);
}

}  // namespace skye_robot_driver
