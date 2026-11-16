#include <gtest/gtest.h>

#include "skye_robot_driver/gripper_common.hpp"

using skye_robot_driver::next_robotiq_command_mm;
using skye_robot_driver::slew_toward;

TEST(GripperSlew, MovesTowardTargetByAtMostMaxDelta) {
  EXPECT_DOUBLE_EQ(slew_toward(0.0, 10.0, 3.0), 3.0);
  EXPECT_DOUBLE_EQ(slew_toward(10.0, 0.0, 3.0), 7.0);
  EXPECT_DOUBLE_EQ(slew_toward(5.0, 5.0, 3.0), 5.0);
}

TEST(GripperSlew, ZeroOrNegativeMaxDeltaSnapsOrHolds) {
  EXPECT_DOUBLE_EQ(slew_toward(1.0, 9.0, 0.0), 9.0);
  EXPECT_DOUBLE_EQ(slew_toward(1.0, 9.0, -1.0), 9.0);
}

TEST(GripperSlew, SmallRemainingGapReachesTarget) {
  EXPECT_DOUBLE_EQ(slew_toward(9.5, 10.0, 3.0), 10.0);
}

TEST(RobotiqCommand, LargeDeltaOneShotsDespiteSlew) {
  // 40 mm close at 250 mm/s would need many ticks; one-shot snaps once.
  EXPECT_DOUBLE_EQ(
      next_robotiq_command_mm(50.0, 2.0, 250.0, 0.01, 3.0), 2.0);
}

TEST(RobotiqCommand, SmallDeltaUsesSlew) {
  // |Δ|=2 < one_shot 3 → slew 100mm/s * 10ms = 1mm/step.
  EXPECT_DOUBLE_EQ(
      next_robotiq_command_mm(10.0, 12.0, 100.0, 0.01, 3.0), 11.0);
}

TEST(RobotiqCommand, OneShotDisabledFallsBackToSlew) {
  EXPECT_DOUBLE_EQ(
      next_robotiq_command_mm(50.0, 2.0, 250.0, 0.01, 0.0), 50.0 - 2.5);
}
