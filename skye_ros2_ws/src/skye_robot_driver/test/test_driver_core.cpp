#include <gtest/gtest.h>

#include <array>

#include "skye_robot_driver/driver_core.hpp"

using skye_robot_driver::DriverCore;

namespace {

constexpr std::array<int, 7> kOrder{0, 1, 2, 3, 4, 5, 6};
constexpr DriverCore::JointArray kJ4NegSigns{
    1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0};
constexpr DriverCore::JointArray kMin{
    -3.1067, -2.0944, -3.1067, -2.5307, -3.1067, -1.0472, -1.5708};
constexpr DriverCore::JointArray kMax{
    3.1067, 2.0944, 3.1067, 1.0472, 3.1067, 1.0472, 1.5708};

}  // namespace

TEST(DriverCore, ClutchLeavesRefsUnchangedWhenInsideLimits) {
  const DriverCore::JointArray desired{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  const auto clamped = DriverCore::clamp_to_limits(desired, kMin, kMax);
  DriverCore::JointArray leader_now = desired;
  DriverCore::JointArray leader_ref{};
  DriverCore::JointArray gento_ref = desired;
  const auto leader_ref_before = leader_ref;
  const auto gento_ref_before = gento_ref;

  EXPECT_FALSE(DriverCore::clutch_saturated_joints(
      desired, clamped, leader_now, leader_ref, gento_ref, kOrder));
  EXPECT_EQ(leader_ref, leader_ref_before);
  EXPECT_EQ(gento_ref, gento_ref_before);
}

TEST(DriverCore, ClutchAbsorbsOnlySaturatedJoint) {
  DriverCore::JointArray leader_ref{};
  DriverCore::JointArray gento_ref{};
  DriverCore::JointArray leader_now{0.1, 0.2, 0.3, -0.5, 0.5, 0.6, 0.7};
  const auto desired = DriverCore::apply_relative_joint_mapping(
      leader_now, leader_ref, gento_ref, kOrder, kJ4NegSigns);
  // sign[3]=-1, leader J4 -0.5 → desired J4 = +0.5, inside URDF max 1.0472.
  EXPECT_NEAR(desired[3], 0.5, 1e-12);

  leader_now[3] = -2.6;
  const auto desired_out = DriverCore::apply_relative_joint_mapping(
      leader_now, leader_ref, gento_ref, kOrder, kJ4NegSigns);
  EXPECT_GT(desired_out[3], kMax[3]);
  const auto clamped =
      DriverCore::clamp_to_limits(desired_out, kMin, kMax);
  EXPECT_NEAR(clamped[3], kMax[3], 1e-12);
  EXPECT_NEAR(clamped[0], desired_out[0], 1e-12);

  EXPECT_TRUE(DriverCore::clutch_saturated_joints(
      desired_out, clamped, leader_now, leader_ref, gento_ref, kOrder));
  EXPECT_NEAR(leader_ref[3], leader_now[3], 1e-12);
  EXPECT_NEAR(gento_ref[3], kMax[3], 1e-12);
  EXPECT_NEAR(leader_ref[0], 0.0, 1e-12);
  EXPECT_NEAR(gento_ref[0], 0.0, 1e-12);
}

TEST(DriverCore, ClutchPreventsChaseWhenLeaderReversesFromSaturation) {
  DriverCore::JointArray leader_ref{};
  DriverCore::JointArray gento_ref{};
  DriverCore::JointArray leader_now{};

  // Drive J4 into follower +limit via sign=-1 (leader more negative).
  leader_now[3] = -3.0;
  auto desired = DriverCore::apply_relative_joint_mapping(
      leader_now, leader_ref, gento_ref, kOrder, kJ4NegSigns);
  auto clamped = DriverCore::clamp_to_limits(desired, kMin, kMax);
  ASSERT_TRUE(DriverCore::clutch_saturated_joints(
      desired, clamped, leader_now, leader_ref, gento_ref, kOrder));

  // Keep pushing further: still glued to the limit, no extra error.
  leader_now[3] = -3.2;
  desired = DriverCore::apply_relative_joint_mapping(
      leader_now, leader_ref, gento_ref, kOrder, kJ4NegSigns);
  clamped = DriverCore::clamp_to_limits(desired, kMin, kMax);
  EXPECT_TRUE(DriverCore::clutch_saturated_joints(
      desired, clamped, leader_now, leader_ref, gento_ref, kOrder));
  EXPECT_NEAR(clamped[3], kMax[3], 1e-12);

  // Reverse a little: leave the stop from the limit, not from accumulated error.
  const double reverse = 0.1;
  leader_now[3] += reverse;
  const auto after = DriverCore::apply_relative_joint_mapping(
      leader_now, leader_ref, gento_ref, kOrder, kJ4NegSigns);
  EXPECT_NEAR(after[3], kMax[3] + kJ4NegSigns[3] * reverse, 1e-9);
  EXPECT_LT(after[3], kMax[3]);
  EXPECT_GT(after[3], kMax[3] - 0.2);
}

TEST(DriverCore, RelativeJ4NegSignMapsLeaderNegativeToFollowerPositive) {
  const DriverCore::JointArray leader_ref{};
  const DriverCore::JointArray gento_ref{};
  DriverCore::JointArray leader_now{};
  leader_now[3] = -0.2;
  const auto mapped = DriverCore::apply_relative_joint_mapping(
      leader_now, leader_ref, gento_ref, kOrder, kJ4NegSigns);
  EXPECT_NEAR(mapped[3], 0.2, 1e-12);
}
