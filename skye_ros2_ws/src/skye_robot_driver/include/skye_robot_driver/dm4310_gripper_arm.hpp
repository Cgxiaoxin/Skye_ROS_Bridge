#pragma once

#include <mutex>
#include <string>

#include "skye_robot_driver/dm_mit.hpp"
#include "skye_robot_driver/driver_core.hpp"
#include "skye_robot_driver/gripper_arm_backend.hpp"

namespace skye_robot_driver {

class Dm4310GripperArm : public GripperArmBackend {
 public:
  struct Config {
    int motor_id{1};
    int terminal{0};
    double kp{3.0};
    double kd{0.12};
    double pos_min{0.0};
    double pos_max{1.6};
    double close_limit{0.93};
    unsigned int feedback_timeout_ms{1};
  };

  Dm4310GripperArm(
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
  FXTerminalType terminal() const;
  bool send_raw(const std::uint8_t *data8, unsigned int timeout_ms);
  bool send_mit(double norm);
  bool enable_motor();
  bool disable_motor();
  bool read_one_feedback(unsigned int timeout_ms);
  bool wait_for_feedback(unsigned int timeout_ms, int attempts);
  bool probe_motor_id();
  double norm_to_rad(double norm) const;
  double rad_to_norm(double rad) const;

  DriverCore &core_;
  DriverCore::Arm arm_;
  Config config_;
  bool started_{false};
  mutable std::mutex target_mutex_;
  double target_{0.0};
  mutable std::mutex fb_mutex_;
  GripperFeedback fb_{};
  std::uint32_t control_ticks_{0};
};

}  // namespace skye_robot_driver
