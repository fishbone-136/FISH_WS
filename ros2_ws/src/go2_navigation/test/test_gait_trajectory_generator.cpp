// Copyright 2026 Go2 Navigation Contributors
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <memory>
#include "gtest/gtest.h"
#include "dwb_core/trajectory_generator.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"

TEST(GaitCandidates, RejectsCreepingArcsBeforeSimulationWithoutRemovingRealTurns)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("FollowPath.min_vel_x", 0.),
    rclcpp::Parameter("FollowPath.max_vel_x", .65),
    rclcpp::Parameter("FollowPath.min_vel_y", 0.),
    rclcpp::Parameter("FollowPath.max_vel_y", 0.),
    rclcpp::Parameter("FollowPath.max_vel_theta", 1.),
    rclcpp::Parameter("FollowPath.min_speed_xy", .22),
    rclcpp::Parameter("FollowPath.max_speed_xy", .65),
    rclcpp::Parameter("FollowPath.min_speed_theta", .55),
    rclcpp::Parameter("FollowPath.acc_lim_x", .7),
    rclcpp::Parameter("FollowPath.decel_lim_x", -.9),
    rclcpp::Parameter("FollowPath.acc_lim_theta", 1.2),
    rclcpp::Parameter("FollowPath.decel_lim_theta", -1.5),
    rclcpp::Parameter("FollowPath.vx_samples", 11),
    rclcpp::Parameter("FollowPath.vy_samples", 1),
    rclcpp::Parameter("FollowPath.vtheta_samples", 25)});
  auto node = std::make_shared<nav2_util::LifecycleNode>("gait_candidates_test", "", options);
  pluginlib::ClassLoader<dwb_core::TrajectoryGenerator> loader(
    "dwb_core", "dwb_core::TrajectoryGenerator");
  auto ordinary = loader.createSharedInstance("dwb_plugins::StandardTrajectoryGenerator");
  auto gait = loader.createSharedInstance("go2_navigation::GaitTrajectoryGenerator");
  ordinary->initialize(node, "FollowPath");
  gait->initialize(node, "FollowPath");
  nav_2d_msgs::msg::Twist2D stopped;
  ordinary->startNewIteration(stopped);
  int baseline = 0;
  bool original_creep = false;
  while (ordinary->hasMoreTwists()) {
    const auto v = ordinary->nextTwist();
    ++baseline;
    original_creep |= v.x > 1.e-5 && v.x < .22-1.e-5;
  }
  EXPECT_TRUE(original_creep);
  gait->startNewIteration(stopped);
  int count = 0;
  bool left = false, right = false, forward = false;
  bool minimum_arc_left = false, minimum_arc_right = false;
  bool boundary_left = false, boundary_right = false;
  bool tight_turn_left = false, tight_turn_right = false;
  while (gait->hasMoreTwists()) {
    const auto v = gait->nextTwist();
    ++count;
    EXPECT_TRUE(v.x < 1.e-5 || v.x >= .22-1.e-5);
    left |= v.x < 1.e-5 && v.theta >= .55;
    right |= v.x < 1.e-5 && v.theta <= -.55;
    forward |= v.x >= .22 && std::abs(v.theta) < 1.e-5;
    boundary_left |= std::abs(v.x-.22)<1.e-5 && std::abs(v.theta-.65)<1.e-5;
    boundary_right |= std::abs(v.x-.22)<1.e-5 && std::abs(v.theta+.65)<1.e-5;
    minimum_arc_left |= std::abs(v.x-.22)<1.e-5 && v.theta>.1 && v.theta<.65;
    minimum_arc_right |= std::abs(v.x-.22)<1.e-5 && v.theta<-.1 && v.theta>-.65;
    tight_turn_left |= std::abs(v.x)<1.e-5 && v.theta>=.55 && v.theta<.56;
    tight_turn_right |= std::abs(v.x)<1.e-5 && v.theta<=-.55 && v.theta>-.56;
  }
  EXPECT_TRUE(left); EXPECT_TRUE(right); EXPECT_TRUE(forward);
  EXPECT_TRUE(minimum_arc_left); EXPECT_TRUE(minimum_arc_right);
  EXPECT_TRUE(boundary_left); EXPECT_TRUE(boundary_right);
  EXPECT_TRUE(tight_turn_left); EXPECT_TRUE(tight_turn_right);
  // Boundary samples add bounded work; the old test assumed pruning only.
  EXPECT_GT(count, 0); EXPECT_LE(count, baseline + 3 * 11 + 6);
  std::cout << "Gait candidates " << count << " versus " << baseline << std::endl;
  gait->setSpeedLimit(.23, false);
  gait->startNewIteration(stopped);
  bool limited_forward = false;
  while (gait->hasMoreTwists()) {
    const auto v = gait->nextTwist();
    EXPECT_LE(std::hypot(v.x,v.y), .23+1.e-5);
    limited_forward |= std::abs(v.x-.22)<1.e-5;
  }
  EXPECT_TRUE(limited_forward);
}
