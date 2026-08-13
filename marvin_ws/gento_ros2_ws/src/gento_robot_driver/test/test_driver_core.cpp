#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "gento_robot_driver/driver_core.hpp"

using gento_robot_driver::DriverCore;

TEST(DriverCore, MapsLeftAndRightToDistinctSdkObjects) {
  EXPECT_EQ(DriverCore::sdk_object_for_arm(DriverCore::Arm::kLeft), FX_OBJ_ARM0);
  EXPECT_EQ(DriverCore::sdk_object_for_arm(DriverCore::Arm::kRight), FX_OBJ_ARM1);
}

TEST(DriverCore, RejectsNonFiniteAndOutOfLimitTargets) {
  const std::array<double, 7> valid{0, 0, 0, -0.5, 0, 0, 0};
  const std::array<double, 7> minimum{-3, -2, -3, -2.4, -3, -1, -1};
  const std::array<double, 7> maximum{3, 2, 3, 1, 3, 1, 1};

  EXPECT_TRUE(DriverCore::validate_target(valid, minimum, maximum));

  auto nan_target = valid;
  nan_target[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(DriverCore::validate_target(nan_target, minimum, maximum));

  auto out_of_limit = valid;
  out_of_limit[3] = -2.5;
  EXPECT_FALSE(DriverCore::validate_target(out_of_limit, minimum, maximum));
}

TEST(DriverCore, RejectsCommandsBeforePositionReady) {
  DriverCore core;
  EXPECT_FALSE(core.command_allowed());
}

TEST(DriverCore, ConvertsRosRadiansToSdkDegrees) {
  const DriverCore::JointArray ros_radians{
      0.0, 0.0, 0.0, -0.5235987755982988, 0.0, 0.0, 0.0};

  const auto sdk_degrees = DriverCore::ros_radians_to_sdk_degrees(ros_radians);

  EXPECT_NEAR(sdk_degrees[3], -30.0, 1e-12);
}

TEST(DriverCore, ConvertsSdkDegreesToRosRadians) {
  const DriverCore::JointArray sdk_degrees{0.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0};

  const auto ros_radians = DriverCore::sdk_degrees_to_ros_radians(sdk_degrees);

  EXPECT_NEAR(ros_radians[3], -0.5235987755982988, 1e-12);
}

TEST(DriverCore, ApplyJointMappingUsesOrderSignsOffsets) {
  const DriverCore::JointArray leader{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  const std::array<int, 7> order{0, 1, 2, 3, 4, 5, 6};
  const DriverCore::JointArray signs{1, 1, 1, -1, 1, -1, -1};
  const DriverCore::JointArray offsets{0, 0, 0, 0, 0, 0, 0};

  const auto mapped = DriverCore::apply_joint_mapping(leader, order, signs, offsets);

  EXPECT_NEAR(mapped[0], 0.1, 1e-12);
  EXPECT_NEAR(mapped[3], -0.4, 1e-12);
  EXPECT_NEAR(mapped[5], -0.6, 1e-12);
  EXPECT_NEAR(mapped[6], -0.7, 1e-12);
}

TEST(DriverCore, LimitDeltaCapsPerJointStep) {
  const DriverCore::JointArray previous{0, 0, 0, 0, 0, 0, 0};
  const DriverCore::JointArray desired{0.2, -0.2, 0, 0, 0, 0, 0};

  const auto limited = DriverCore::limit_delta(desired, previous, 0.05);

  EXPECT_NEAR(limited[0], 0.05, 1e-12);
  EXPECT_NEAR(limited[1], -0.05, 1e-12);
}

TEST(DriverCore, StopMotionClearsCommandAllowedWithoutLink) {
  DriverCore core;
  EXPECT_FALSE(core.command_allowed());
  EXPECT_FALSE(core.stop_motion());  // not linked → false, still not allowed
  EXPECT_FALSE(core.command_allowed());
}

TEST(DriverCore, HoldCurrentFailsWithoutLink) {
  DriverCore core;
  EXPECT_FALSE(core.hold_current());
  EXPECT_FALSE(core.command_allowed());
}
