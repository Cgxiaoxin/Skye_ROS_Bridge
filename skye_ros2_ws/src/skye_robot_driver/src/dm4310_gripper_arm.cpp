#include "skye_robot_driver/dm4310_gripper_arm.hpp"

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
constexpr unsigned int kSetupTimeoutMs = 100;
constexpr std::uint32_t kReenablePeriodTicks = 100;

}  // namespace

Dm4310GripperArm::Dm4310GripperArm(
    DriverCore &core, DriverCore::Arm arm, Config config)
    : core_(core), arm_(arm), config_(config) {}

FXTerminalType Dm4310GripperArm::terminal() const {
  return config_.terminal == 0 ? FX_TERMINAL_ARM0 : FX_TERMINAL_ARM1;
}

bool Dm4310GripperArm::start(std::string *report) {
  if (started_) {
    return true;
  }
  if (!(std::isfinite(config_.kp) && std::isfinite(config_.kd) &&
        std::isfinite(config_.pos_min) && std::isfinite(config_.pos_max) &&
        std::isfinite(config_.close_limit)) ||
      config_.pos_max <= config_.pos_min || config_.motor_id < 0 ||
      config_.close_limit <= 0.0 || config_.close_limit > 1.0 ||
      (config_.terminal != 0 && config_.terminal != 1)) {
    if (report != nullptr) {
      *report = "invalid dm4310 config";
    }
    return false;
  }
  core_.terminal_clear(terminal());
  if (!enable_motor()) {
    if (report != nullptr) {
      *report = "terminal_set enable failed (SDK rejected TX)";
    }
    return false;
  }
  bool ok = wait_for_feedback(kProbeTimeoutMs, kProbeAttempts);
  if (!ok && probe_motor_id()) {
    ok = true;
  }
  std::ostringstream oss;
  oss << "dm4310 id=" << config_.motor_id
      << " term=ARM" << (config_.terminal == 0 ? 0 : 1)
      << (ok ? " fb=ok" : " fb=NONE");
  if (report != nullptr) {
    *report = oss.str();
  }
  started_ = true;
  return true;
}

void Dm4310GripperArm::stop() {
  if (!started_) {
    return;
  }
  disable_motor();
  started_ = false;
}

void Dm4310GripperArm::set_target(double norm) {
  const double clamped = dm_mit::clamp(norm, 0.0, config_.close_limit);
  std::lock_guard<std::mutex> lock(target_mutex_);
  target_ = clamped;
}

double Dm4310GripperArm::target() const {
  std::lock_guard<std::mutex> lock(target_mutex_);
  return target_;
}

double Dm4310GripperArm::norm_to_rad(double norm) const {
  return norm * (config_.pos_max - config_.pos_min) + config_.pos_min;
}

double Dm4310GripperArm::rad_to_norm(double rad) const {
  const double span = config_.pos_max - config_.pos_min;
  if (span <= 0.0) {
    return 0.0;
  }
  return dm_mit::clamp((rad - config_.pos_min) / span, 0.0, 1.0);
}

bool Dm4310GripperArm::send_raw(
    const std::uint8_t *data8, unsigned int timeout_ms) {
  const auto packed = dm_mit::pack_terminal(
      static_cast<std::uint32_t>(config_.motor_id), data8, dm_mit::kCanMtu);
  return core_.terminal_set(
      terminal(), FX_CHN_CANFD, packed.data(), packed.size(), timeout_ms);
}

bool Dm4310GripperArm::send_mit(double norm) {
  const auto mit =
      dm_mit::encode_mit(config_.kp, config_.kd, norm_to_rad(norm), 0.0, 0.0);
  return send_raw(mit.data(), config_.feedback_timeout_ms);
}

bool Dm4310GripperArm::enable_motor() {
  const auto frame = dm_mit::enable_frame();
  bool ok = true;
  for (int i = 0; i < kEnableRepeats; ++i) {
    ok = send_raw(frame.data(), kSetupTimeoutMs) && ok;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return ok;
}

bool Dm4310GripperArm::disable_motor() {
  const auto frame = dm_mit::disable_frame();
  return send_raw(frame.data(), kSetupTimeoutMs);
}

bool Dm4310GripperArm::read_one_feedback(unsigned int timeout_ms) {
  const auto packet = core_.terminal_get(terminal(), timeout_ms);
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
  GripperFeedback parsed;
  parsed.valid = true;
  parsed.position = rad_to_norm(fb->pos);
  parsed.velocity = fb->vel;
  parsed.effort = fb->torque;
  parsed.err_code = fb->err_code;
  parsed.device_id = unpacked->first;
  parsed.frame_tag = "can_" + std::to_string(unpacked->first);
  std::lock_guard<std::mutex> lock(fb_mutex_);
  fb_ = parsed;
  return true;
}

bool Dm4310GripperArm::wait_for_feedback(
    unsigned int timeout_ms, int attempts) {
  const double q = target();
  for (int i = 0; i < attempts; ++i) {
    send_mit(q);
    if (read_one_feedback(timeout_ms)) {
      return true;
    }
  }
  return false;
}

bool Dm4310GripperArm::probe_motor_id() {
  const int original = config_.motor_id;
  const auto enable = dm_mit::enable_frame();
  for (int id : kProbeIds) {
    config_.motor_id = id;
    core_.terminal_clear(terminal());
    send_raw(enable.data(), kSetupTimeoutMs);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    if (wait_for_feedback(kProbeTimeoutMs, kProbeAttempts)) {
      return true;
    }
  }
  config_.motor_id = original;
  return false;
}

void Dm4310GripperArm::tick_control() {
  if (!started_) {
    return;
  }
  ++control_ticks_;
  if (control_ticks_ % kReenablePeriodTicks == 0) {
    bool valid = false;
    {
      std::lock_guard<std::mutex> lock(fb_mutex_);
      valid = fb_.valid;
    }
    if (!valid) {
      const auto frame = dm_mit::enable_frame();
      send_raw(frame.data(), config_.feedback_timeout_ms);
    }
  }
  send_mit(target());
}

void Dm4310GripperArm::tick_feedback() {
  if (!started_) {
    return;
  }
  read_one_feedback(config_.feedback_timeout_ms);
}

GripperFeedback Dm4310GripperArm::feedback() const {
  std::lock_guard<std::mutex> lock(fb_mutex_);
  return fb_;
}

const char *Dm4310GripperArm::type_name() const { return "dm4310"; }

std::string Dm4310GripperArm::describe() const {
  return "dm4310 id=" + std::to_string(config_.motor_id) +
         " term=ARM" + std::to_string(config_.terminal);
}

}  // namespace skye_robot_driver
