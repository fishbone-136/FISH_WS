// Copyright 2026 Go2 Navigation Contributors
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "go2_navigation/rotation_helpers.hpp"

using go2_navigation::latest_path_pose;
using go2_navigation::RearTurnDirection;
using go2_navigation::rotation_velocity;

TEST(GoalCapture, HoldsSmallPositionJitterButReleasesARealDeparture)
{
  go2_navigation::GoalCapture phase;
  EXPECT_FALSE(phase.update(0.4, 0.25, 0.12));
  EXPECT_TRUE(phase.update(0.24, 0.25, 0.12));
  EXPECT_TRUE(phase.update(0.31, 0.25, 0.12));
  EXPECT_FALSE(phase.update(0.38, 0.25, 0.12));
  EXPECT_TRUE(phase.update(0.20, 0.25, 0.12));
  phase.reset();
  EXPECT_FALSE(phase.update(0.31, 0.25, 0.12));
  EXPECT_FALSE(phase.update(std::numeric_limits<double>::quiet_NaN(), 0.25, 0.12));
}

TEST(PathUpdate, SameGoalSurvivesPeriodicReplanButNewGoalResets)
{
  geometry_msgs::msg::PoseStamped a;
  a.header.frame_id = "map";
  a.pose.orientation.w = 1.0;
  auto b = a;
  b.header.stamp.sec = 42;
  EXPECT_TRUE(go2_navigation::same_navigation_goal(a, b));
  b.pose.position.x = 0.4;
  EXPECT_FALSE(go2_navigation::same_navigation_goal(a, b));
  b = a;
  b.pose.orientation.z = std::sin(0.3);
  b.pose.orientation.w = std::cos(0.3);
  EXPECT_FALSE(go2_navigation::same_navigation_goal(a, b));
}

TEST(RearTurn, HoldsDirectionAcrossPiWrap)
{
  RearTurnDirection turn;
  EXPECT_GT(turn.error(3.13), 0.0);
  EXPECT_GT(turn.error(-3.13), 0.0);
  EXPECT_DOUBLE_EQ(turn.error(2.7), 2.7);
  turn.reset();
  EXPECT_LT(turn.error(-3.13), 0.0);
  EXPECT_LT(turn.error(3.13), 0.0);
  // A real overshoot near zero still permits a correction.
  EXPECT_DOUBLE_EQ(turn.error(0.3), 0.3);
}

TEST(RearTurn, RetainsTheCollisionCheckedAlternativeAcrossReplanning)
{
  RearTurnDirection turn;
  EXPECT_GT(turn.error(3.13),0.);
  turn.select(-3.15);
  EXPECT_LT(turn.error(3.13),0.);
  EXPECT_LT(turn.error(-3.10),0.);
  EXPECT_DOUBLE_EQ(turn.error(-2.7),-2.7);
  turn.reset();
  EXPECT_GT(turn.error(3.13),0.);
}

TEST(PathAlignment, TriesRightForALeftBearingOutsideTheOldRearOnlyWindow)
{
  go2_navigation::PathAlignmentTurn turn;
  const double heading = 2.45;
  const double alternate = turn.other_side(heading, heading);
  ASSERT_LT(alternate, -3.0);
  turn.start(heading, 0., 10., alternate, true);
  // Neither a fresh path nor crossing the +/-pi bearing wrap revives the
  // old left turn while the checked right turn makes real progress.
  for (int tick = 1; tick <= 24; ++tick) {
    const double yaw = -.052 * tick;
    const double replanned_heading = heading + (tick % 2 ? .04 : -.04);
    const double error = std::remainder(replanned_heading - yaw, go2_navigation::full_turn);
    turn.update(replanned_heading, yaw, 10. + .10 * tick);
    ASSERT_TRUE(turn.alternative());
    EXPECT_LT(turn.remaining(error, yaw), 0.);
    EXPECT_FALSE(turn.stalled());
  }
  turn.update(heading - .6, -1.25, 12.5);
  EXPECT_FALSE(turn.active());
}

TEST(PathAlignment, StationaryBodyAndOppositeJitterDoNotCountAsProgress)
{
  go2_navigation::PathAlignmentTurn turn;
  turn.start(1.8, 0., 2., 1.8);
  for (int i = 1; i <= 13; ++i) {
    turn.update(1.8, i % 2 ? -.021 : .019, 2. + i * .1);
  }
  EXPECT_TRUE(turn.stalled());
  EXPECT_LT(turn.other_side(1.8, 1.8), 0.);
  turn.reset();
  EXPECT_FALSE(turn.stalled());
}

TEST(PathAlignment, ForwardSideEscapeStopsAfterABoundedRightPivot)
{
  go2_navigation::PathAlignmentTurn turn;
  const double preferred = .9;
  const double alternate = turn.other_side(preferred, preferred);
  EXPECT_DOUBLE_EQ(alternate, -.60);
  turn.start(preferred, 0., 1., alternate, true);
  ASSERT_TRUE(turn.escape());
  turn.update(preferred + .1, -.30, 1.5);
  EXPECT_NEAR(turn.remaining(1.30, -.30), -.30, 1.e-9);
  turn.update(preferred, -.61, 2.);
  EXPECT_LE(turn.direction() * turn.remaining(1.51, -.61), 0.);
  EXPECT_DOUBLE_EQ(turn.other_side(-preferred, -preferred), .60);
}

TEST(RotationVelocity, AcceleratesSymmetricallyAndBrakes)
{
  double left = 0.0;
  double right = 0.0;
  for (int i = 0; i < 8; ++i) {
    const double next = rotation_velocity(3.0, 0.25, left, 0.8, 0.55, 1.8, 1.2, 0.125);
    EXPECT_LE(next - left, 0.150001);
    left = next;
    right = rotation_velocity(-3.0, 0.25, right, 0.8, 0.55, 1.8, 1.2, 0.125);
    EXPECT_NEAR(left, -right, 1.e-9);
  }
  EXPECT_DOUBLE_EQ(left, 0.8);
  const double braking = rotation_velocity(0.45, 0.25, left, 0.8, 0.55, 1.8, 1.2, 0.125);
  EXPECT_LT(braking, left);
  EXPECT_GE(braking, 0.55);
  EXPECT_DOUBLE_EQ(rotation_velocity(0.2, 0.25, braking, 0.8, 0.55, 1.8, 1.2, 0.125), 0.0);
}

TEST(RotationCollision, SweepsBothDirectionsAndStopsAtRemainingAngle)
{
  const auto left = go2_navigation::rotation_sweep(3.0, 0.8, 1.2);
  const auto right = go2_navigation::rotation_sweep(-3.0, -0.8, 1.2);
  ASSERT_EQ(left.size(), right.size());
  EXPECT_DOUBLE_EQ(left.front(), 0.0);
  EXPECT_DOUBLE_EQ(left.back(), 0.96);
  for (std::size_t i = 0; i < left.size(); ++i) {
    EXPECT_DOUBLE_EQ(left[i], -right[i]);
    if (i > 0) {EXPECT_LE(left[i] - left[i - 1], 0.020001);}
  }
  EXPECT_DOUBLE_EQ(go2_navigation::rotation_sweep(0.3, 0.8, 1.2).back(), 0.3);
  EXPECT_EQ(go2_navigation::rotation_sweep(3.0, 0.0, 1.2).size(), 1U);
}

TEST(RotationVelocity, BrakesBeforeANarrowHeadingToleranceInsteadOfHoldingGaitFloor)
{
  const double v = rotation_velocity(0.07, 0.025, 0.4, 0.65, 0.55, 2.2, 2.0, 1.0 / 15.0);
  EXPECT_LT(v, 0.4);
  EXPECT_LT(v, 0.55);
  EXPECT_GE(v, 0.4 - 2.0 / 15.0 - 1.e-9);
  EXPECT_NEAR(v, -rotation_velocity(-0.07, 0.025, -0.4, 0.65, 0.55, 2.2, 2.0, 1.0 / 15.0), 1.e-9);
}

TEST(GoalApproach, SpeedLimitIsContinuousAndNeverAmplifiesCollisionCheckedCommand)
{
  using go2_navigation::approach_speed_limit;
  EXPECT_DOUBLE_EQ(approach_speed_limit(1.0, .25, .9, .24, .45), .45);
  EXPECT_DOUBLE_EQ(approach_speed_limit(.25, .25, .9, .24, .45), .24);
  EXPECT_DOUBLE_EQ(approach_speed_limit(.4, .25, .9, .24, .18), .18);
  EXPECT_NEAR(approach_speed_limit(.9-1.e-6, .25, .9, .24, .45), .45, 1.e-6);
  double previous = 0.0;
  for (double distance = .25; distance < 1.0; distance += .01) {
    const double speed = approach_speed_limit(distance, .25, .9, .24, .45);
    EXPECT_GE(speed, previous);
    EXPECT_LE(speed, .45);
    previous = speed;
  }
}

TEST(GoalApproach, DoesNotChaseTheCentreWhenTheForwardRayAlreadyEntersTheAcceptedDisk)
{
  using go2_navigation::position_approach_error;
  for (const double sign : {-1.0, 1.0}) {
    const double angle = sign * .5;
    EXPECT_LT(.06 * std::abs(std::sin(angle)), .04);
    EXPECT_DOUBLE_EQ(position_approach_error(angle, .06, .05), 0.0);
    EXPECT_DOUBLE_EQ(position_approach_error(angle, .0499, .05), 0.0);
    EXPECT_DOUBLE_EQ(position_approach_error(angle, .0501, .05), 0.0);
    const double correction = position_approach_error(sign * 1.2, .06, .05);
    EXPECT_GT(sign * correction, 0.0);
    EXPECT_LT(std::abs(correction), 1.2);
  }
  EXPECT_DOUBLE_EQ(position_approach_error(3.0, .04, .05), 0.0);
}

TEST(GoalApproach, CurvatureReachesTheAcceptedDiskBeforePassingAnOffAxisTarget)
{
  using go2_navigation::approach_arc_miss;
  using go2_navigation::position_approach_yaw_rate;
  // Captured outbound/return failure: a fixed yaw gain misses this target.
  const double distance = std::hypot(.270, .060), bearing = .615;
  EXPECT_GT(approach_arc_miss(bearing, distance, .18, 1.0), .8*.05);
  for (const double sign : {-1., 1.}) {
    const double rate = position_approach_yaw_rate(sign*bearing, distance, .05, .18, 1.);
    EXPECT_GT(sign*rate, 0.);
    EXPECT_LT(approach_arc_miss(sign*bearing, distance, .18, rate), .05);
    // This much closer side target cannot be reached at the allowed curvature:
    // the controller must use a checked turn rather than silently overshoot.
    const double tight = position_approach_yaw_rate(sign*.9, .085, .05, .18, 1.);
    EXPECT_GT(approach_arc_miss(sign*.9, .085, .18, tight), .9*.05);
    EXPECT_DOUBLE_EQ(position_approach_yaw_rate(sign*.5, .06, .05, .18, 1.), 0.);
  }
}

TEST(GoalApproach, SmallRequiredCorrectionCannotStopOutsideTheArrivalDisk)
{
  // Closed-loop return case: the 0.65 rad/s near-obstacle yaw cap makes
  // this arc miss, but the residual correction is below the normal 0.28 rad
  // path handover threshold. Reusing that threshold returned zero forever.
  const double distance = 0.119, bearing = 0.973, tolerance = 0.10;
  const double error = go2_navigation::position_approach_error(bearing, distance, tolerance);
  const double yaw = go2_navigation::position_approach_yaw_rate(bearing, distance, tolerance, .26, .65);
  ASSERT_GT(go2_navigation::approach_arc_miss(bearing, distance, .26, yaw), .9 * tolerance);
  ASSERT_LT(error, .28);
  const double alignment = go2_navigation::position_alignment_tolerance(error, .28);
  EXPECT_GT(rotation_velocity(error, alignment, 0., .65, .55, 2.2, 2., 1. / 15.), 0.);
  EXPECT_DOUBLE_EQ(go2_navigation::position_alignment_tolerance(-error, .28), alignment);
}

TEST(PathTransform, LatestTfAvoidsFutureQueryButRejectsStaleData)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer buffer(clock);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom_lio";
  transform.child_frame_id = "map";
  transform.transform.rotation.w = 1.0;
  transform.transform.translation.x = 2.0;
  transform.header.stamp = rclcpp::Time(9, 900000000, RCL_ROS_TIME);
  ASSERT_TRUE(buffer.setTransform(transform, "test"));
  geometry_msgs::msg::PoseStamped point;
  point.header.frame_id = "map";
  point.header.stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  point.pose.position.x = 1.0;
  point.pose.orientation.w = 1.0;
  EXPECT_THROW(buffer.lookupTransform("odom_lio", "map", rclcpp::Time(point.header.stamp)),
    tf2::ExtrapolationException);
  const auto result = latest_path_pose(buffer, point, "odom_lio", rclcpp::Time(10, 0), 0.8);
  EXPECT_DOUBLE_EQ(result.pose.position.x, 3.0);
  EXPECT_EQ(result.header.frame_id, "odom_lio");
  EXPECT_THROW(latest_path_pose(buffer, point, "odom_lio", rclcpp::Time(11, 0), 0.8),
    std::runtime_error);
  EXPECT_THROW(latest_path_pose(buffer, point, "missing_frame", rclcpp::Time(10, 0), 0.8),
    tf2::TransformException);
}

TEST(PathTransform, StaticAndSameFramePathsDoNotExpire)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer buffer(clock);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom_lio";
  transform.child_frame_id = "map";
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(buffer.setTransform(transform, "static_test", true));
  geometry_msgs::msg::PoseStamped point;
  point.header.frame_id = "map";
  point.pose.orientation.w = 1.0;
  EXPECT_NO_THROW(latest_path_pose(buffer, point, "odom_lio", rclcpp::Time(100, 0), 0.8));
  EXPECT_NO_THROW(latest_path_pose(buffer, point, "map", rclcpp::Time(100, 0), 0.8));
}
