#pragma once

#include <mutex>
#include <string>

#include "L1Robot.h"
#include "skye_robot_driver/driver_core.hpp"
#include "skye_robot_driver/gripper_arm_backend.hpp"

namespace skye_robot_driver {

class RobotiqGripperArm : public GripperArmBackend {
 public:
  struct Config {
    int slave_id{9};
    int speed{0x88};
    int force{0x10};
    double pos_min_mm{0.0};
    double pos_max_mm{50.0};
    double close_limit{0.93};
    FXChnType channel{FX_CHN_485A};
    int terminal{1};
    unsigned int modbus_timeout_ms{150};
  };

  RobotiqGripperArm(
      DriverCore &core, DriverCore::Arm arm, Config config);

  bool start(std::string *report) override;
  void stop() override;
  void set_target(double norm) override;
  double target() const override;
  void tick_control() override;
  void tick_feedback() override;
  GripperFeedback feedback() const override;
  const char *type_name() const override;
  std::string describe() const override;

 private:
  static constexpr std::uint16_t kRegAction = 0x03E8;
  static constexpr std::uint16_t kRegPos = 0x03E9;
  static constexpr std::uint16_t kRegSpeed = 0x03EA;
  static constexpr std::uint16_t kRegStatus = 0x07D0;
  static constexpr std::uint16_t kRegPosCur = 0x07D2;
  static constexpr double kFullPosMm = 50.0;
  static constexpr double kPosRatio = 0.1953125;
  static constexpr int kGstaActivated = 0x03;

  FXTerminalType terminal() const;
  bool tx_modbus(const std::uint8_t *data, std::size_t len);
  std::optional<std::uint16_t> modbus_read(std::uint16_t addr);
  bool modbus_write(std::uint16_t addr, std::uint16_t value);
  static std::uint16_t action_reg(
      int r_act, int r_gto = 0, int r_atr = 0, int r_ard = 0);
  bool reset_gripper();
  bool activate_gripper();
  bool write_pending();
  double mm_to_norm(double opening_mm) const;
  double norm_to_mm(double norm) const;
  double read_opening_mm();

  DriverCore &core_;
  DriverCore::Arm arm_;
  Config config_;
  bool started_{false};
  mutable std::mutex target_mutex_;
  double target_{0.0};
  bool dirty_{false};
  mutable std::mutex fb_mutex_;
  GripperFeedback fb_{};
  std::uint32_t feedback_ticks_{0};
};

}  // namespace skye_robot_driver
