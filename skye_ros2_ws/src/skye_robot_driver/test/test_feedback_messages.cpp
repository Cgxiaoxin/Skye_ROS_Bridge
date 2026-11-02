#include <gtest/gtest.h>

#include <vector>

#include "skye_robot_driver/feedback_messages.hpp"

namespace skye_robot_driver {
namespace {

const std::array<std::string, 7> kNames{
    "r_j1", "r_j2", "r_j3", "r_j4", "r_j5", "r_j6", "r_j7"};
const rclcpp::Time kStamp(123, 456, RCL_ROS_TIME);

TEST(FeedbackMessages, JointFeedbackIncludesSensorTorqueInJointOrder) {
  const DriverCore::JointArray position{1, 2, 3, 4, 5, 6, 7};
  const DriverCore::JointArray velocity{8, 9, 10, 11, 12, 13, 14};
  const DriverCore::JointArray torque{-1, -2, -3, -4, -5, -6, -7};
  const auto message = make_arm_joint_state(
      kStamp, kNames, position, velocity, &torque);
  EXPECT_EQ(message.header.stamp, static_cast<builtin_interfaces::msg::Time>(kStamp));
  EXPECT_EQ(message.name, (std::vector<std::string>(kNames.begin(), kNames.end())));
  EXPECT_EQ(message.position, (std::vector<double>{1, 2, 3, 4, 5, 6, 7}));
  EXPECT_EQ(message.velocity, (std::vector<double>{8, 9, 10, 11, 12, 13, 14}));
  EXPECT_EQ(message.effort, (std::vector<double>{-1, -2, -3, -4, -5, -6, -7}));
}

TEST(FeedbackMessages, AppliedPositionActionDoesNotInventMeasuredEffort) {
  const DriverCore::JointArray position{};
  const DriverCore::JointArray velocity{};
  const auto message = make_arm_joint_state(kStamp, kNames, position, velocity);
  EXPECT_TRUE(message.effort.empty());
}

TEST(FeedbackMessages, ExternalTorqueHasNamesAndEffortWithoutPose) {
  const DriverCore::JointArray torque{11, 12, 13, 14, 15, 16, 17};
  const auto message = make_external_torque_state(kStamp, kNames, torque);
  EXPECT_EQ(message.header.stamp, static_cast<builtin_interfaces::msg::Time>(kStamp));
  EXPECT_EQ(message.name, (std::vector<std::string>(kNames.begin(), kNames.end())));
  EXPECT_EQ(message.effort, (std::vector<double>{11, 12, 13, 14, 15, 16, 17}));
  EXPECT_TRUE(message.position.empty());
  EXPECT_TRUE(message.velocity.empty());
}

TEST(FeedbackMessages, FlangeWrenchPreservesAxesSignsStampAndFrameLabel) {
  const DriverCore::WrenchArray values{1, -2, 3, -4, 5, -6};
  const auto message = make_flange_wrench(kStamp, "right_ft_sensor", values);
  EXPECT_EQ(message.header.stamp, static_cast<builtin_interfaces::msg::Time>(kStamp));
  EXPECT_EQ(message.header.frame_id, "right_ft_sensor");
  EXPECT_DOUBLE_EQ(message.wrench.force.x, 1);
  EXPECT_DOUBLE_EQ(message.wrench.force.y, -2);
  EXPECT_DOUBLE_EQ(message.wrench.force.z, 3);
  EXPECT_DOUBLE_EQ(message.wrench.torque.x, -4);
  EXPECT_DOUBLE_EQ(message.wrench.torque.y, 5);
  EXPECT_DOUBLE_EQ(message.wrench.torque.z, -6);
  EXPECT_TRUE(make_flange_wrench(kStamp, "", values).header.frame_id.empty());
}

}  // namespace
}  // namespace skye_robot_driver
