#include <gtest/gtest.h>

#include <memory>

#include "gento_robot_driver/gento_robot_driver_node.hpp"

class RosContextTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(RosContextTest, ExposesCompatibleJointInterfaces) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("connect_on_startup", false)});
  auto node = std::make_shared<gento_robot_driver::GentoRobotDriverNode>(options);

  EXPECT_EQ(std::string(node->get_name()), "gento_robot_driver");
  EXPECT_EQ(node->get_publishers_info_by_topic("/joint_states").size(), 1U);
  EXPECT_EQ(node->get_subscriptions_info_by_topic("/left_joint_control").size(), 1U);
  EXPECT_EQ(node->get_subscriptions_info_by_topic("/right_joint_control").size(), 1U);

  const auto names = node->get_service_names_and_types();
  bool has_hold = false;
  bool has_stop = false;
  for (const auto& entry : names) {
    const auto& name = entry.first;
    const bool ends_hold =
        name == "/hold_current" ||
        (name.size() >= 13 && name.compare(name.size() - 13, 13, "/hold_current") == 0);
    const bool ends_stop =
        name == "/stop_motion" ||
        (name.size() >= 12 && name.compare(name.size() - 12, 12, "/stop_motion") == 0);
    if (ends_hold) {
      has_hold = true;
    }
    if (ends_stop) {
      has_stop = true;
    }
  }
  EXPECT_TRUE(has_hold);
  EXPECT_TRUE(has_stop);
}
