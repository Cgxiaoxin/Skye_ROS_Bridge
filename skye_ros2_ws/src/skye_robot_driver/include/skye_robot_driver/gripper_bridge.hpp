#pragma once

#include <memory>
#include <string>

#include "skye_robot_driver/driver_core.hpp"
#include "skye_robot_driver/gripper_arm_backend.hpp"
#include "skye_robot_driver/gripper_common.hpp"
#include "skye_robot_driver/dm4310_gripper_arm.hpp"
#include "skye_robot_driver/robotiq_gripper_arm.hpp"

namespace skye_robot_driver {

// Per-arm gripper facade: DM4310 (CAN) or Robotiq Hand-E (RS485 Modbus).
class GripperBridge {
 public:
  using Arm = DriverCore::Arm;
  using Feedback = GripperFeedback;

  struct ArmConfig {
    GripperDriverType type{GripperDriverType::kDm4310};
    Dm4310GripperArm::Config dm{};
    RobotiqGripperArm::Config robotiq{};
  };

  struct Config {
    ArmConfig left;
    ArmConfig right;
  };

  explicit GripperBridge(DriverCore &core);

  bool start(const Config &config);
  void stop();
  bool started() const;
  const std::string &start_report() const;

  void set_target(Arm arm, double value);
  double target(Arm arm) const;
  void tick_control();
  void tick_feedback();
  Feedback feedback(Arm arm) const;
  const char *type_name(Arm arm) const;
  std::string describe(Arm arm) const;
  // Legacy logging helper (DM4310 motor id or Robotiq slave id).
  int device_id(Arm arm) const;

 private:
  static std::unique_ptr<GripperArmBackend> make_backend(
      DriverCore &core, Arm arm, const ArmConfig &cfg);

  DriverCore &core_;
  Config config_{};
  std::unique_ptr<GripperArmBackend> left_;
  std::unique_ptr<GripperArmBackend> right_;
  bool started_{false};
  std::string start_report_;
};

}  // namespace skye_robot_driver
