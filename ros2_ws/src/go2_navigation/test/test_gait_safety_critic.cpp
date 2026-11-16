// Copyright 2026 Go2 Navigation Contributors
// SPDX-License-Identifier: Apache-2.0
#include <memory>
#include "gtest/gtest.h"
#include "dwb_core/exceptions.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"

class CriticCostmap : public nav2_costmap_2d::Costmap2DROS
{
public:
  CriticCostmap() : Costmap2DROS("gait_safety_test_costmap")
  {
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("odom", false, false);
    layered_costmap_->resizeMap(100, 100, 0.04, -2.0, -2.0);
    footprint_padding_ = 0.0;
    std::vector<geometry_msgs::msg::Point> footprint;
    for (const auto & xy : std::vector<std::pair<double, double>>{
        {0.45, 0.27}, {0.45, -0.27}, {-0.45, -0.27}, {-0.45, 0.27}})
    {
      geometry_msgs::msg::Point p; p.x = xy.first; p.y = xy.second;
      footprint.push_back(p);
    }
    setRobotFootprint(footprint);
  }
};

TEST(GaitSafetyCritics, SoftCornerCostDoesNotCloseADoorButHardCollisionStillRejects)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<nav2_util::LifecycleNode>("footprint_critic_test");
  auto map = std::make_shared<CriticCostmap>();
  pluginlib::ClassLoader<dwb_core::TrajectoryCritic> loader("dwb_core", "dwb_core::TrajectoryCritic");
  auto original = loader.createSharedInstance("dwb_critics::ObstacleFootprintCritic");
  auto collision = loader.createSharedInstance("go2_navigation::CollisionFootprintCritic");
  original->initialize(node, "Original", "FollowPath", map);
  collision->initialize(node, "Collision", "FollowPath", map);
  geometry_msgs::msg::Pose2D pose, goal;
  nav_2d_msgs::msg::Twist2D velocity;
  nav_2d_msgs::msg::Path2D path;
  ASSERT_TRUE(original->prepare(pose, velocity, goal, path));
  ASSERT_TRUE(collision->prepare(pose, velocity, goal, path));
  auto * grid = map->getCostmap();
  for (unsigned int y = 0; y < 100; ++y) {
    for (unsigned int x = 0; x < 100; ++x) {grid->setCost(x, y, 252);}
  }
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.poses.push_back(pose);
  EXPECT_GT(original->scoreTrajectory(trajectory), 0.0);
  EXPECT_DOUBLE_EQ(collision->scoreTrajectory(trajectory), original->scoreTrajectory(trajectory));
  for (unsigned int x = 40; x < 90; ++x) {
    grid->setCost(x, 59, nav2_costmap_2d::LETHAL_OBSTACLE);
    grid->setCost(x, 40, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  for (double x = 0.0; x <= 1.4; x += 0.1) {
    geometry_msgs::msg::Pose2D p; p.x = x; path.poses.push_back(p);
  }
  ASSERT_TRUE(collision->prepare(pose, velocity, goal, path));
  EXPECT_DOUBLE_EQ(collision->scoreTrajectory(trajectory), 0.0);
  EXPECT_GT(collision->getScale(), 0.0);  // DWB must execute the hard checker.
  grid->setCost(50, 56, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(collision->scoreTrajectory(trajectory), dwb_core::IllegalTrajectoryException);
  grid->setCost(50, 56, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_THROW(collision->scoreTrajectory(trajectory), dwb_core::IllegalTrajectoryException);
  trajectory.poses.front().x = 10.0;
  EXPECT_THROW(collision->scoreTrajectory(trajectory), dwb_core::IllegalTrajectoryException);
}

TEST(GaitSafetyCritics, EnforcesNearLimitsAndKeepsContinuityOnlyForTheSameGoal)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<nav2_util::LifecycleNode>("gait_velocity_critic_test");
  auto map = std::make_shared<CriticCostmap>();
  for (unsigned int x = 40; x < 90; ++x) {
    map->getCostmap()->setCost(x, 59, nav2_costmap_2d::LETHAL_OBSTACLE);
    map->getCostmap()->setCost(x, 40, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  pluginlib::ClassLoader<dwb_core::TrajectoryCritic> loader("dwb_core", "dwb_core::TrajectoryCritic");
  auto critic = loader.createSharedInstance("go2_navigation::GaitSafetyCritic");
  critic->initialize(node, "GaitSafety", "FollowPath", map);
  geometry_msgs::msg::Pose2D pose, goal; goal.x = 2.0;
  nav_2d_msgs::msg::Twist2D velocity;
  nav_2d_msgs::msg::Path2D path;
  ASSERT_TRUE(critic->prepare(pose, velocity, goal, path));
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.velocity.x = 0.26; trajectory.velocity.theta = 1.0;
  EXPECT_THROW(critic->scoreTrajectory(trajectory), dwb_core::IllegalTrajectoryException);
  trajectory.velocity.x = 0.169; trajectory.velocity.theta = 0.0;
  EXPECT_THROW(critic->scoreTrajectory(trajectory), dwb_core::IllegalTrajectoryException);
  trajectory.velocity.x = 0.26; trajectory.velocity.theta = 0.58;
  EXPECT_DOUBLE_EQ(critic->scoreTrajectory(trajectory), 0.0);
  critic->debrief(trajectory.velocity);
  critic->reset();
  ASSERT_TRUE(critic->prepare(pose, velocity, goal, path));
  EXPECT_DOUBLE_EQ(critic->scoreTrajectory(trajectory), 0.0);
  trajectory.velocity.theta = -0.58;
  EXPECT_GT(critic->scoreTrajectory(trajectory), 1.0);
  goal.x = 3.0;
  ASSERT_TRUE(critic->prepare(pose, velocity, goal, path));
  EXPECT_DOUBLE_EQ(critic->scoreTrajectory(trajectory), 0.0);
}
