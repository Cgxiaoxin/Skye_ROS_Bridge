#pragma once

#include <string>

#include "skye_robot_driver/gripper_common.hpp"

namespace skye_robot_driver {

class DriverCore;

class GripperArmBackend {
 public:
  virtual ~GripperArmBackend() = default;

  virtual bool start(std::string *report) = 0;
  virtual void stop() = 0;
  virtual void set_target(double norm) = 0;
  virtual double target() const = 0;
  virtual void tick_control() = 0;
  virtual void tick_feedback() = 0;
  virtual GripperFeedback feedback() const = 0;
  virtual const char *type_name() const = 0;
  virtual std::string describe() const = 0;
};

}  // namespace skye_robot_driver
