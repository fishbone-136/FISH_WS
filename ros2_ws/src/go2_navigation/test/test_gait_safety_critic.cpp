// Copyright 2026 Go2 Navigation Contributors
// SPDX-License-Identifier: Apache-2.0
#include <memory>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <thread>
#include "gtest/gtest.h"
#include "go2_navigation/execution_safety.hpp"
#include "dwb_core/exceptions.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

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

TEST(GaitSafetyCritics, PureTurnCommitmentSurvivesReplansAndYieldsToAnActualBlockedSide)
{
  const char * domain=std::getenv("ROS_DOMAIN_ID");
  const char * local=std::getenv("ROS_LOCALHOST_ONLY");
  if (!domain || std::string(domain)!="86" || !local || std::string(local)!="1") {
    GTEST_SKIP() << "Synthetic safety topics require isolated localhost domain 86";
  }
  if (!rclcpp::ok()) {rclcpp::init(0,nullptr);}
  auto node=std::make_shared<nav2_util::LifecycleNode>("turn_commitment_test");
  node->declare_parameter("FollowPath.GaitSafety.history_timeout",5.0);
  node->declare_parameter("FollowPath.execution_safety.enabled",true);
  node->declare_parameter("FollowPath.execution_safety.body_front",.38);
  node->declare_parameter("FollowPath.execution_safety.body_rear",.38);
  node->declare_parameter("FollowPath.execution_safety.body_half_width",.15);
  auto map=std::make_shared<CriticCostmap>();
  pluginlib::ClassLoader<dwb_core::TrajectoryCritic> loader("dwb_core","dwb_core::TrajectoryCritic");
  auto critic=loader.createSharedInstance("go2_navigation::GaitSafetyCritic");
  critic->initialize(node,"GaitSafety","FollowPath",map);
  const auto qos=rclcpp::SensorDataQoS().keep_last(1);
  auto raw=node->create_publisher<sensor_msgs::msg::LaserScan>("/scan_safety",qos);
  auto memory=node->create_publisher<sensor_msgs::msg::PointCloud2>("/safety/remembered_obstacles",qos);
  raw->on_activate();memory->on_activate();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  auto publish=[&](bool blocked) {
      for (int i=0;i<50;++i) {
        sensor_msgs::msg::LaserScan scan;
        scan.header.frame_id=map->getBaseFrameID();scan.header.stamp=node->now();
        scan.range_min=.05;scan.range_max=5.5;raw->publish(scan);
        sensor_msgs::msg::PointCloud2 points;
        points.header.frame_id=map->getGlobalFrameID();points.header.stamp=node->now();
        points.height=1;points.width=blocked ? 1 : 0;points.point_step=8;
        points.row_step=points.width*8;points.fields.resize(2);
        for (int j=0;j<2;++j) {
          points.fields[j].name=j==0 ? "x" : "y";points.fields[j].offset=j*4;
          points.fields[j].datatype=7;points.fields[j].count=1;
        }
        if (blocked) {
          const float xy[2]={.32F,.22F};points.data.resize(8);
          std::memcpy(points.data.data(),xy,8);
        }
        memory->publish(points);executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    };
  geometry_msgs::msg::Pose2D pose,goal;goal.x=2.;
  nav_2d_msgs::msg::Twist2D velocity;nav_2d_msgs::msg::Path2D path;
  publish(false);
  ASSERT_TRUE(critic->prepare(pose,velocity,goal,path));
  dwb_msgs::msg::Trajectory2D turn;turn.velocity.theta=.56;
  EXPECT_NO_THROW(critic->scoreTrajectory(turn));critic->debrief(turn.velocity);
  critic->reset();ASSERT_TRUE(critic->prepare(pose,velocity,goal,path));
  turn.velocity.theta=-.56;
  EXPECT_THROW(critic->scoreTrajectory(turn),dwb_core::IllegalTrajectoryException);
  publish(true);ASSERT_TRUE(critic->prepare(pose,velocity,goal,path));
  // New evidence blocks the old left turn, so the safe right turn is released
  // immediately, before the two-second commitment has elapsed.
  EXPECT_NO_THROW(critic->scoreTrajectory(turn));
  turn.velocity.theta=.56;
  EXPECT_THROW(critic->scoreTrajectory(turn),dwb_core::IllegalTrajectoryException);
  executor.remove_node(node->get_node_base_interface());
}

TEST(ExecutionAvoidance, SoftApproachKeepsUsableGaitButPreservesHardStops)
{
  go2_navigation::ExecutionContext context;
  context.enabled = context.ready = true;
  context.half_length = .38; context.half_width = .15; context.scan_age = .20;
  auto obstacle = [&](double x) {
      context.raw = {{x, 0.}}; context.memory = context.raw; context.points = context.raw;
    };
  obstacle(.78);  // 40 cm beyond the front: gentle slowdown still exceeds .22 m/s.
  EXPECT_TRUE(context.allows(.26, 0., 0.));
  obstacle(.70);
  EXPECT_FALSE(context.allows(.22, 0., 0.));  // Reduction below the usable gait.
  obstacle(.48);  // Braking distance is occupied, regardless of soft slowdown.
  EXPECT_FALSE(context.allows(.26, 0., 0.));
  context.raw.clear();  // Remembered thin obstacles remain authoritative.
  EXPECT_FALSE(context.allows(.26, 0., 0.));
  context.minimum_translation = .255;
  obstacle(.78);
  EXPECT_FALSE(context.allows(.26, 0., 0.));
  context.minimum_translation = .22;
  context.raw.clear(); context.memory.clear(); context.points.clear();
  for (double x = -.5; x < 1.5; x += .04) {
    context.points.push_back({x, .25}); context.points.push_back({x, -.25});
  }
  context.raw = context.memory = context.points;
  EXPECT_TRUE(context.allows(.26, 0., 0.));  // Straight through a 50 cm corridor.
  EXPECT_FALSE(context.allows(.26, 0., .65));  // Body corner sweeps into a wall.
  context.ready = false;
  EXPECT_FALSE(context.allows(.26, 0., 0.));
}

TEST(ExecutionAvoidance, AvoidsEnteringTheRawInterlockWhileAllowingAnAwayArc)
{
  go2_navigation::ExecutionContext context;
  context.enabled = context.ready = true;
  context.half_length = .38;
  context.half_width = .15;
  context.scan_age = .15;
  context.raw = context.points = {{.53, .17}};
  ASSERT_TRUE(context.allows(.24, 0., 0.));  // Misses the body rectangle.
  context.near_guard_start = .14;
  context.near_guard_end = .43;
  context.near_guard_half_width = .18;
  EXPECT_FALSE(context.allows(.24, 0., 0.));
  EXPECT_TRUE(context.allows(.24, 0., -.60));

  // A projected return already in the region may be outside the raw guard's
  // height band. It must not disqualify a safe turn away from the obstacle.
  context.raw = context.points = {{.42, .17}};
  EXPECT_TRUE(context.allows(0., 0., -.60));
}
