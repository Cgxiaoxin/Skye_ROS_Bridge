#include "skye_robot_driver/gripper_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace skye_robot_driver {
namespace {

FXTerminalType terminal_for_arm(GripperBridge::Arm arm) {
  return arm == GripperBridge::Arm::kLeft ? FX_TERMINAL_ARM0 : FX_TERMINAL_ARM1;
}

}  // namespace

GripperBridge::GripperBridge(DriverCore &core) : core_(core) {}

bool GripperBridge::started() const { return started_; }

bool GripperBridge::start(const Config &config) {
  if (started_) {
    return true;
  }
  if (!core_.linked()) {
    return false;
  }
  if (!(std::isfinite(config.kp) && std::isfinite(config.kd) &&
        std::isfinite(config.pos_min) && std::isfinite(config.pos_max)) ||
      config.pos_max <= config.pos_min || config.left_motor_id < 0 ||
      config.right_motor_id < 0) {
    return false;
  }
  config_ = config;
  if (!enable_motors()) {
    return false;
  }
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
  const int motor_id =
      arm == Arm::kLeft ? config_.left_motor_id : config_.right_motor_id;
  const auto packed = dm_mit::pack_terminal(
      static_cast<std::uint32_t>(motor_id), data8, dm_mit::kCanMtu);
  return core_.terminal_set(
      terminal_for_arm(arm), FX_CHN_CANFD, packed.data(), packed.size());
}

bool GripperBridge::enable_motors() {
  const auto frame = dm_mit::enable_frame();
  const bool left_ok = send_raw(Arm::kLeft, frame.data());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const bool right_ok = send_raw(Arm::kRight, frame.data());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return left_ok && right_ok;
}

bool GripperBridge::disable_motors() {
  const auto frame = dm_mit::disable_frame();
  const bool left_ok = send_raw(Arm::kLeft, frame.data());
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const bool right_ok = send_raw(Arm::kRight, frame.data());
  return left_ok && right_ok;
}

void GripperBridge::tick_control() {
  if (!started_) {
    return;
  }
  double tl = 0.0;
  double tr = 0.0;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    tl = target_left_;
    tr = target_right_;
  }
  const double q_l = norm_to_rad(tl);
  const double q_r = norm_to_rad(tr);
  const auto mit_l =
      dm_mit::encode_mit(config_.kp, config_.kd, q_l, 0.0, 0.0);
  const auto mit_r =
      dm_mit::encode_mit(config_.kp, config_.kd, q_r, 0.0, 0.0);
  send_raw(Arm::kLeft, mit_l.data());
  send_raw(Arm::kRight, mit_r.data());
}

void GripperBridge::tick_feedback() {
  if (!started_) {
    return;
  }
  for (Arm arm : {Arm::kLeft, Arm::kRight}) {
    const auto packet =
        core_.terminal_get(terminal_for_arm(arm), config_.feedback_timeout_ms);
    if (!packet || packet->data.size() < 5) {
      continue;
    }
    const auto unpacked =
        dm_mit::unpack_terminal(packet->data.data(), packet->data.size());
    if (!unpacked) {
      continue;
    }
    const auto fb = dm_mit::decode_feedback(
        unpacked->second.data(), unpacked->second.size());
    if (!fb) {
      continue;
    }
    Feedback out;
    out.valid = true;
    out.position = rad_to_norm(fb->pos);
    out.velocity = fb->vel;
    out.effort = fb->torque;
    out.err_code = fb->err_code;
    out.can_id = unpacked->first;
    std::lock_guard<std::mutex> lock(fb_mutex_);
    if (arm == Arm::kLeft) {
      fb_left_ = out;
    } else {
      fb_right_ = out;
    }
  }
}

GripperBridge::Feedback GripperBridge::feedback(Arm arm) const {
  std::lock_guard<std::mutex> lock(fb_mutex_);
  return arm == Arm::kLeft ? fb_left_ : fb_right_;
}

}  // namespace skye_robot_driver
