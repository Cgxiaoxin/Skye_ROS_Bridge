#pragma once

#include <cstdint>
#include <mutex>
#include <string>

#include "skye_robot_driver/dm_mit.hpp"
#include "skye_robot_driver/driver_core.hpp"

namespace skye_robot_driver {

// DM4310 gripper via Gento Terminal CANFD (same SDK link as DriverCore).
// SDK has no dedicated gripper API; Hand-24 is a different end-effector.
class GripperBridge {
 public:
  using Arm = DriverCore::Arm;

  struct Config {
    int left_motor_id{1};
    int right_motor_id{1};
    // Right gripper terminal: 0=ARM0 (shared bus with left), 1=ARM1.
    int right_terminal{1};
    double kp{3.0};
    double kd{0.12};
    double pos_min{0.0};
    double pos_max{1.6};
    // Non-blocking CAN read; long timeouts block the single-threaded executor.
    unsigned int feedback_timeout_ms{1};
  };

  // Normalized feedback for FACTR: position in [0,1] (0=open, 1=closed).
  struct Feedback {
    bool valid{false};
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
    int err_code{0};
    std::uint32_t can_id{0};
  };

  explicit GripperBridge(DriverCore &core);

  bool start(const Config &config);
  void stop();
  bool started() const;
  const std::string &start_report() const;

  // Target in [0,1]: 0=fully open, 1=fully closed.
  void set_target(Arm arm, double value);
  double target(Arm arm) const;
  int motor_id(Arm arm) const;

  void tick_control();
  void tick_feedback();

  Feedback feedback(Arm arm) const;

 private:
  FXTerminalType terminal_for_arm(Arm arm) const;
  int motor_id_unlocked(Arm arm) const;
  bool send_raw(Arm arm, const std::uint8_t *data8);
  bool send_mit(Arm arm, double norm);
  bool enable_motors();
  bool disable_motors();
  bool read_one_feedback(Arm arm, unsigned int timeout_ms, Feedback *out);
  bool wait_for_feedback(Arm arm, unsigned int timeout_ms, int attempts);
  bool probe_motor_id(Arm arm);
  void maybe_reenable();
  double norm_to_rad(double norm) const;
  double rad_to_norm(double rad) const;

  DriverCore &core_;
  Config config_{};
  bool started_{false};
  std::string start_report_;

  mutable std::mutex target_mutex_;
  double target_left_{0.0};
  double target_right_{0.0};

  mutable std::mutex fb_mutex_;
  Feedback fb_left_{};
  Feedback fb_right_{};

  std::uint32_t control_ticks_{0};
};
}  // namespace skye_robot_driver
