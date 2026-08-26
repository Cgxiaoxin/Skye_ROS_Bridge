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

// Relative mapping requires per-frame continuous-unwrap state. This helper
// advances one frame and updates prev for the caller, mirroring handle_command.
DriverCore::JointArray map_frame(
    const DriverCore::JointArray &leader_now, DriverCore::JointArray &leader_prev,
    DriverCore::JointArray &leader_continuous,
    const DriverCore::JointArray &leader_cont_ref,
    const DriverCore::JointArray &follower_ref) {
  const auto mapped = DriverCore::apply_relative_joint_mapping(
      leader_now, leader_prev, leader_continuous, leader_cont_ref,
      follower_ref, kOrder, kJ4NegSigns);
  leader_prev = leader_now;
  return mapped;
}

}  // namespace

TEST(DriverCore, ClutchLeavesRefsUnchangedWhenInsideLimits) {
  const DriverCore::JointArray desired{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  const auto clamped = DriverCore::clamp_to_limits(desired, kMin, kMax);
  DriverCore::JointArray leader_continuous = desired;
  DriverCore::JointArray leader_cont_ref{};
  DriverCore::JointArray gento_ref = desired;
  const auto cont_ref_before = leader_cont_ref;
  const auto gento_ref_before = gento_ref;

  EXPECT_FALSE(DriverCore::clutch_saturated_joints(
      desired, clamped, leader_continuous, leader_cont_ref, gento_ref,
      kOrder));
  EXPECT_EQ(leader_cont_ref, cont_ref_before);
  EXPECT_EQ(gento_ref, gento_ref_before);
}

TEST(DriverCore, ClutchAbsorbsOnlySaturatedJoint) {
  DriverCore::JointArray leader_prev{};
  DriverCore::JointArray leader_continuous{};
  DriverCore::JointArray leader_cont_ref{};
  DriverCore::JointArray gento_ref{};
  DriverCore::JointArray leader_now{0.1, 0.2, 0.3, -0.5, 0.5, 0.6, 0.7};
  const auto desired = map_frame(
      leader_now, leader_prev, leader_continuous, leader_cont_ref, gento_ref);
  // sign[3]=-1, leader J4 -0.5 → desired J4 = +0.5, inside URDF max 1.0472.
  EXPECT_NEAR(desired[3], 0.5, 1e-12);

  leader_now[3] = -2.6;
  const auto desired_out = map_frame(
      leader_now, leader_prev, leader_continuous, leader_cont_ref, gento_ref);
  EXPECT_GT(desired_out[3], kMax[3]);
  const auto clamped =
      DriverCore::clamp_to_limits(desired_out, kMin, kMax);
  EXPECT_NEAR(clamped[3], kMax[3], 1e-12);
  EXPECT_NEAR(clamped[0], desired_out[0], 1e-12);

  EXPECT_TRUE(DriverCore::clutch_saturated_joints(
      desired_out, clamped, leader_continuous, leader_cont_ref, gento_ref,
      kOrder));
  EXPECT_NEAR(leader_cont_ref[3], leader_continuous[3], 1e-12);
  EXPECT_NEAR(gento_ref[3], kMax[3], 1e-12);
  EXPECT_NEAR(leader_cont_ref[0], 0.0, 1e-12);
  EXPECT_NEAR(gento_ref[0], 0.0, 1e-12);
}

TEST(DriverCore, ClutchPreventsChaseWhenLeaderReversesFromSaturation) {
  DriverCore::JointArray leader_prev{};
  DriverCore::JointArray leader_continuous{};
  DriverCore::JointArray leader_cont_ref{};
  DriverCore::JointArray gento_ref{};
  DriverCore::JointArray leader_now{};

  // Drive J4 into follower +limit via sign=-1 (leader more negative).
  leader_now[3] = -3.0;
  auto desired = map_frame(
      leader_now, leader_prev, leader_continuous, leader_cont_ref, gento_ref);
  auto clamped = DriverCore::clamp_to_limits(desired, kMin, kMax);
  ASSERT_TRUE(DriverCore::clutch_saturated_joints(
      desired, clamped, leader_continuous, leader_cont_ref, gento_ref,
      kOrder));

  // Keep pushing further: still glued to the limit, no extra error.
  leader_now[3] = -3.2;
  desired = map_frame(
      leader_now, leader_prev, leader_continuous, leader_cont_ref, gento_ref);
  clamped = DriverCore::clamp_to_limits(desired, kMin, kMax);
  EXPECT_TRUE(DriverCore::clutch_saturated_joints(
      desired, clamped, leader_continuous, leader_cont_ref, gento_ref,
      kOrder));
  EXPECT_NEAR(clamped[3], kMax[3], 1e-12);

  // Reverse a little: leave the stop from the limit, not from accumulated error.
  const double reverse = 0.1;
  leader_now[3] += reverse;
  const auto after = map_frame(
      leader_now, leader_prev, leader_continuous, leader_cont_ref, gento_ref);
  EXPECT_NEAR(after[3], kMax[3] + kJ4NegSigns[3] * reverse, 1e-9);
  EXPECT_LT(after[3], kMax[3]);
  EXPECT_GT(after[3], kMax[3] - 0.2);
}

TEST(DriverCore, RelativeJ4NegSignMapsLeaderNegativeToFollowerPositive) {
  DriverCore::JointArray leader_prev{};
  DriverCore::JointArray leader_continuous{};
  DriverCore::JointArray leader_cont_ref{};
  const DriverCore::JointArray gento_ref{};
  DriverCore::JointArray leader_now{};
  leader_now[3] = -0.2;
  const auto mapped = map_frame(
      leader_now, leader_prev, leader_continuous, leader_cont_ref, gento_ref);
  EXPECT_NEAR(mapped[3], 0.2, 1e-12);
}

// J1 crosses +pi: reported angle jumps from +2.6 to -3.1 (a -2pi discontinuity)
// while the physical motion is small. The command must follow the physical
// motion, not spin the follower a full turn.
TEST(DriverCore, RelativeUnwrapHandlesPiCrossing) {
  DriverCore::JointArray leader_prev{};
  leader_prev[0] = 2.6;
  DriverCore::JointArray leader_continuous{};
  leader_continuous[0] = 2.6;
  DriverCore::JointArray leader_cont_ref{};
  leader_cont_ref[0] = 2.6;
  const DriverCore::JointArray gento_ref{};
  DriverCore::JointArray leader_now{};
  leader_now[0] = -3.1;  // physically +2.6+0.583, wrapped by -2pi
  const auto mapped = DriverCore::apply_relative_joint_mapping(
      leader_now, leader_prev, leader_continuous, leader_cont_ref, gento_ref,
      kOrder, kJ4NegSigns);
  // frame delta = -3.1 - 2.6 = -5.7 -> +0.583 after +2pi unwrap.
  EXPECT_NEAR(mapped[0], 0.5831853071795865, 1e-9);
  EXPECT_NEAR(leader_continuous[0], 2.6 + 0.5831853071795865, 1e-9);
}

// The unwrap is generic across all joints. J3 (idx 2) and J5 (idx 4) share the
// same +-3.0 rad range as J1 and can also cross +-pi if the barrier is
// overpowered; both must follow the physical motion, not spin.
TEST(DriverCore, RelativeUnwrapCoversJ3AndJ5) {
  DriverCore::JointArray leader_prev{};
  leader_prev[2] = 2.6;   // J3 near +pi
  leader_prev[4] = -2.6;  // J5 near -pi
  DriverCore::JointArray leader_continuous = leader_prev;
  DriverCore::JointArray leader_cont_ref = leader_prev;
  const DriverCore::JointArray gento_ref{};
  DriverCore::JointArray leader_now{};
  leader_now[2] = -3.1;  // J3 wrapped by -2pi -> physically +0.583
  leader_now[4] = 3.1;   // J5 wrapped by +2pi -> physically -0.583
  const auto mapped = DriverCore::apply_relative_joint_mapping(
      leader_now, leader_prev, leader_continuous, leader_cont_ref, gento_ref,
      kOrder, kJ4NegSigns);
  EXPECT_NEAR(mapped[2], 0.5831853071795865, 1e-9);
  EXPECT_NEAR(mapped[4], -0.5831853071795865, 1e-9);
  EXPECT_NEAR(leader_continuous[2], 2.6 + 0.5831853071795865, 1e-9);
  EXPECT_NEAR(leader_continuous[4], -2.6 - 0.5831853071795865, 1e-9);
}

// A legitimate slow swing across the full J1 range (|delta| > pi) must NOT be
// folded: it accumulates over many small frames, each well below pi.
TEST(DriverCore, RelativeKeepsLegitimateLargeSwing) {
  DriverCore::JointArray leader_prev{};
  leader_prev[0] = 2.0;
  DriverCore::JointArray leader_continuous{};
  leader_continuous[0] = 2.0;
  DriverCore::JointArray leader_cont_ref{};
  leader_cont_ref[0] = 2.0;
  const DriverCore::JointArray gento_ref{};
  auto step = [&](double q) {
    DriverCore::JointArray now{};
    now[0] = q;
    auto m = DriverCore::apply_relative_joint_mapping(
        now, leader_prev, leader_continuous, leader_cont_ref, gento_ref,
        kOrder, kJ4NegSigns);
    leader_prev[0] = q;
    return m[0];
  };
  EXPECT_NEAR(step(1.0), -1.0, 1e-9);
  EXPECT_NEAR(step(0.0), -2.0, 1e-9);
  EXPECT_NEAR(step(-1.0), -3.0, 1e-9);
  EXPECT_NEAR(step(-2.0), -4.0, 1e-9);
}
