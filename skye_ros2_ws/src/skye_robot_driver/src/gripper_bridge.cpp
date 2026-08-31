#include "skye_robot_driver/gripper_bridge.hpp"

#include <sstream>
#include <utility>

namespace skye_robot_driver {

GripperBridge::GripperBridge(DriverCore &core) : core_(core) {}

std::unique_ptr<GripperArmBackend> GripperBridge::make_backend(
    DriverCore &core, Arm arm, const ArmConfig &cfg) {
  if (cfg.type == GripperDriverType::kRobotiq) {
    return std::make_unique<RobotiqGripperArm>(core, arm, cfg.robotiq);
  }
  return std::make_unique<Dm4310GripperArm>(core, arm, cfg.dm);
}

bool GripperBridge::started() const { return started_; }

const std::string &GripperBridge::start_report() const { return start_report_; }

bool GripperBridge::start(const Config &config) {
  if (started_) {
    return true;
  }
  if (!core_.linked()) {
    start_report_ = "SDK not linked";
    return false;
  }
  config_ = config;
  left_ = make_backend(core_, Arm::kLeft, config_.left);
  right_ = make_backend(core_, Arm::kRight, config_.right);

  std::string left_report;
  std::string right_report;
  const bool left_ok = left_->start(&left_report);
  const bool right_ok = right_->start(&right_report);

  std::ostringstream report;
  report << "L[" << left_->type_name() << "] " << left_report << "; R["
         << right_->type_name() << "] " << right_report;
  start_report_ = report.str();
  started_ = left_ok && right_ok;
  return started_;
}

void GripperBridge::stop() {
  if (!started_) {
    return;
  }
  if (left_) {
    left_->stop();
  }
  if (right_) {
    right_->stop();
  }
  started_ = false;
}

void GripperBridge::set_target(Arm arm, double value) {
  if (arm == Arm::kLeft && left_) {
    left_->set_target(value);
  } else if (arm == Arm::kRight && right_) {
    right_->set_target(value);
  }
}

double GripperBridge::target(Arm arm) const {
  if (arm == Arm::kLeft && left_) {
    return left_->target();
  }
  if (arm == Arm::kRight && right_) {
    return right_->target();
  }
  return 0.0;
}

void GripperBridge::tick_control() {
  if (!started_) {
    return;
  }
  if (left_) {
    left_->tick_control();
  }
  if (right_) {
    right_->tick_control();
  }
}

void GripperBridge::tick_feedback() {
  if (!started_) {
    return;
  }
  if (left_) {
    left_->tick_feedback();
  }
  if (right_) {
    right_->tick_feedback();
  }
}

GripperBridge::Feedback GripperBridge::feedback(Arm arm) const {
  if (arm == Arm::kLeft && left_) {
    return left_->feedback();
  }
  if (arm == Arm::kRight && right_) {
    return right_->feedback();
  }
  return {};
}

const char *GripperBridge::type_name(Arm arm) const {
  if (arm == Arm::kLeft && left_) {
    return left_->type_name();
  }
  if (arm == Arm::kRight && right_) {
    return right_->type_name();
  }
  return "none";
}

std::string GripperBridge::describe(Arm arm) const {
  if (arm == Arm::kLeft && left_) {
    return left_->describe();
  }
  if (arm == Arm::kRight && right_) {
    return right_->describe();
  }
  return "none";
}

int GripperBridge::device_id(Arm arm) const {
  const auto fb = feedback(arm);
  if (fb.device_id != 0) {
    return static_cast<int>(fb.device_id);
  }
  if (arm == Arm::kLeft && config_.left.type == GripperDriverType::kDm4310) {
    return config_.left.dm.motor_id;
  }
  if (arm == Arm::kRight && config_.right.type == GripperDriverType::kDm4310) {
    return config_.right.dm.motor_id;
  }
  if (arm == Arm::kLeft && config_.left.type == GripperDriverType::kRobotiq) {
    return config_.left.robotiq.slave_id;
  }
  if (arm == Arm::kRight && config_.right.type == GripperDriverType::kRobotiq) {
    return config_.right.robotiq.slave_id;
  }
  return -1;
}

}  // namespace skye_robot_driver
