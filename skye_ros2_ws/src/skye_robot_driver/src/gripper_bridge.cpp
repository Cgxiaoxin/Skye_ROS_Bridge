#include "skye_robot_driver/gripper_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <thread>

namespace skye_robot_driver {
namespace {

constexpr int kProbeIds[] = {1, 2, 0, 3, 4, 5, 6, 7, 8, 16, 17};
constexpr int kEnableRepeats = 3;
constexpr int kProbeAttempts = 4;
constexpr unsigned int kProbeTimeoutMs = 30;
constexpr std::uint32_t kReenablePeriodTicks = 100;  // ~1s at 100 Hz

}  // namespace

GripperBridge::GripperBridge(DriverCore &core) : core_(core) {}

bool GripperBridge::started() const { return started_; }

const std::string &GripperBridge::start_report() const { return start_report_; }

FXTerminalType GripperBridge::terminal_for_arm(Arm arm) const {
  if (arm == Arm::kLeft) {
    return FX_TERMINAL_ARM0;
  }
  return config_.right_terminal == 0 ? FX_TERMINAL_ARM0 : FX_TERMINAL_ARM1;
}

int GripperBridge::motor_id_unlocked(Arm arm) const {
  return arm == Arm::kLeft ? config_.left_motor_id : config_.right_motor_id;
}

int GripperBridge::motor_id(Arm arm) const {
  return motor_id_unlocked(arm);
}

bool GripperBridge::start(const Config &config) {
  if (started_) {
    return true;
  }
  if (!core_.linked()) {
    start_report_ = "SDK not linked";
    return false;
  }
  if (!(std::isfinite(config.kp) && std::isfinite(config.kd) &&
        std::isfinite(config.pos_min) && std::isfinite(config.pos_max)) ||
      config.pos_max <= config.pos_min || config.left_motor_id < 0 ||
      config.right_motor_id < 0 ||
      (config.right_terminal != 0 && config.right_terminal != 1)) {
    start_report_ = "invalid gripper config";
    return false;
  }
  config_ = config;
  core_.terminal_clear(FX_TERMINAL_ARM0);
  core_.terminal_clear(FX_TERMINAL_ARM1);

  if (!enable_motors()) {
    start_report_ = "terminal_set enable failed (SDK rejected TX)";
    return false;
  }

  std::ostringstream report;
  report << "L id=" << config_.left_motor_id << " term=ARM0";
  const bool left_ok = wait_for_feedback(Arm::kLeft, kProbeTimeoutMs, kProbeAttempts);
  report << (left_ok ? " fb=ok" : " fb=NONE");

  report << "; R id=" << config_.right_motor_id
         << " term=ARM" << (config_.right_terminal == 0 ? 0 : 1);
  bool right_ok = wait_for_feedback(Arm::kRight, kProbeTimeoutMs, kProbeAttempts);
  if (!right_ok) {
    report << " fb=NONE, probing";
    if (probe_motor_id(Arm::kRight)) {
      right_ok = true;
      report << " -> id=" << config_.right_motor_id << " fb=ok";
    } else {
      report << " -> still silent (check ARM1 CAN / power / wiring)";
    }
  } else {
    report << " fb=ok";
  }
  start_report_ = report.str();
  started_ = true;
  return true;
}

void GripperBridge::stop() {
  if (!started_) {
    return;
  }
  disable_motors();
  started_ = false;
}

void GripperBridge::set_target(Arm arm, double value) {
  const double clamped = dm_mit::clamp(value, 0.0, 1.0);
  std::lock_guard<std::mutex> lock(target_mutex_);
  if (arm == Arm::kLeft) {
    target_left_ = clamped;
  } else {
    target_right_ = clamped;
  }
}

double GripperBridge::target(Arm arm) const {
  std::lock_guard<std::mutex> lock(target_mutex_);
  return arm == Arm::kLeft ? target_left_ : target_right_;
}

double GripperBridge::norm_to_rad(double norm) const {
  return norm * (config_.pos_max - config_.pos_min) + config_.pos_min;
}

double GripperBridge::rad_to_norm(double rad) const {
  const double span = config_.pos_max - config_.pos_min;
  if (span <= 0.0) {
    return 0.0;
  }
  return dm_mit::clamp((rad - config_.pos_min) / span, 0.0, 1.0);
}

bool GripperBridge::send_raw(Arm arm, const std::uint8_t *data8) {
  const auto packed = dm_mit::pack_terminal(
      static_cast<std::uint32_t>(motor_id_unlocked(arm)), data8,
      dm_mit::kCanMtu);
  return core_.terminal_set(
      terminal_for_arm(arm), FX_CHN_CANFD, packed.data(), packed.size());
}

bool GripperBridge::send_mit(Arm arm, double norm) {
  const auto mit =
      dm_mit::encode_mit(config_.kp, config_.kd, norm_to_rad(norm), 0.0, 0.0);
  return send_raw(arm, mit.data());
}

bool GripperBridge::enable_motors() {
  const auto frame = dm_mit::enable_frame();
  bool ok = true;
  for (int i = 0; i < kEnableRepeats; ++i) {
    ok = send_raw(Arm::kLeft, frame.data()) && ok;
    ok = send_raw(Arm::kRight, frame.data()) && ok;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return ok;
}

bool GripperBridge::disable_motors() {
  const auto frame = dm_mit::disable_frame();
  const bool left_ok = send_raw(Arm::kLeft, frame.data());
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const bool right_ok = send_raw(Arm::kRight, frame.data());
  return left_ok && right_ok;
}

bool GripperBridge::read_one_feedback(
    Arm arm, unsigned int timeout_ms, Feedback *out) {
  const auto packet = core_.terminal_get(terminal_for_arm(arm), timeout_ms);
  if (!packet || packet->data.size() < 5) {
    return false;
  }
  const auto unpacked =
      dm_mit::unpack_terminal(packet->data.data(), packet->data.size());
  if (!unpacked) {
    return false;
  }
  const auto fb = dm_mit::decode_feedback(
      unpacked->second.data(), unpacked->second.size());
  if (!fb) {
    return false;
  }
  Feedback parsed;
  parsed.valid = true;
  parsed.position = rad_to_norm(fb->pos);
  parsed.velocity = fb->vel;
  parsed.effort = fb->torque;
  parsed.err_code = fb->err_code;
  parsed.can_id = unpacked->first;
  if (out != nullptr) {
    *out = parsed;
  }
  std::lock_guard<std::mutex> lock(fb_mutex_);
  if (arm == Arm::kLeft) {
    fb_left_ = parsed;
  } else {
    fb_right_ = parsed;
  }
  return true;
}

bool GripperBridge::wait_for_feedback(
    Arm arm, unsigned int timeout_ms, int attempts) {
  const double q = target(arm);
  for (int i = 0; i < attempts; ++i) {
    send_mit(arm, q);
    if (read_one_feedback(arm, timeout_ms, nullptr)) {
      return true;
    }
  }
  return false;
}

bool GripperBridge::probe_motor_id(Arm arm) {
  const int original = motor_id_unlocked(arm);
  const auto enable = dm_mit::enable_frame();
  for (int id : kProbeIds) {
    if (arm == Arm::kLeft) {
      config_.left_motor_id = id;
    } else {
      config_.right_motor_id = id;
    }
    core_.terminal_clear(terminal_for_arm(arm));
    send_raw(arm, enable.data());
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    if (wait_for_feedback(arm, kProbeTimeoutMs, kProbeAttempts)) {
      return true;
    }
  }
  if (arm == Arm::kLeft) {
    config_.left_motor_id = original;
  } else {
    config_.right_motor_id = original;
  }
  return false;
}

void GripperBridge::maybe_reenable() {
  Feedback left;
  Feedback right;
  {
    std::lock_guard<std::mutex> lock(fb_mutex_);
    left = fb_left_;
    right = fb_right_;
  }
  if (left.valid && right.valid) {
    return;
  }
  const auto frame = dm_mit::enable_frame();
  if (!left.valid) {
    send_raw(Arm::kLeft, frame.data());
  }
  if (!right.valid) {
    send_raw(Arm::kRight, frame.data());
  }
}

void GripperBridge::tick_control() {
  if (!started_) {
    return;
  }
  ++control_ticks_;
  if (control_ticks_ % kReenablePeriodTicks == 0) {
    maybe_reenable();
  }
  double tl = 0.0;
  double tr = 0.0;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    tl = target_left_;
    tr = target_right_;
  }
  send_mit(Arm::kLeft, tl);
  send_mit(Arm::kRight, tr);
}

void GripperBridge::tick_feedback() {
  if (!started_) {
    return;
  }
  read_one_feedback(Arm::kLeft, config_.feedback_timeout_ms, nullptr);
  read_one_feedback(Arm::kRight, config_.feedback_timeout_ms, nullptr);
}

GripperBridge::Feedback GripperBridge::feedback(Arm arm) const {
  std::lock_guard<std::mutex> lock(fb_mutex_);
  return arm == Arm::kLeft ? fb_left_ : fb_right_;
}

}  // namespace skye_robot_driver
