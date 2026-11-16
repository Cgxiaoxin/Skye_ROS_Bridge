#include <gtest/gtest.h>

#include "skye_robot_driver/gripper_common.hpp"

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
