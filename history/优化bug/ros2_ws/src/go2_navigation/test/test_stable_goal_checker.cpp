#include <cmath>
#include <memory>

#include "gtest/gtest.h"
#include "nav2_core/goal_checker.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

TEST(StableGoalChecker, ReplanResetPreservesCaptureOnlyForSameNearbyGoal)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("stable_goal_checker_test");
  pluginlib::ClassLoader<nav2_core::GoalChecker> loader("nav2_core", "nav2_core::GoalChecker");
  auto checker = loader.createSharedInstance("go2_navigation::StableGoalChecker");
  checker->initialize(node, "goal_checker", nullptr);
  geometry_msgs::msg::Pose goal, pose;
  geometry_msgs::msg::Twist velocity;
  goal.orientation.w = 1.0;
  pose.position.x = 0.24;
  pose.orientation.z = std::sin(0.6);
  pose.orientation.w = std::cos(0.6);
  EXPECT_FALSE(checker->isGoalReached(pose, goal, velocity));
  checker->reset();  // ControllerServer does this on a same-goal path update.
  pose.position.x = 0.31;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  EXPECT_TRUE(checker->isGoalReached(pose, goal, velocity));
  pose.position.x = 0.38;
  EXPECT_FALSE(checker->isGoalReached(pose, goal, velocity));
  pose.position.x = 0.24;
  EXPECT_TRUE(checker->isGoalReached(pose, goal, velocity));
  checker->reset();
  goal.position.x = 2.0;
  EXPECT_FALSE(checker->isGoalReached(pose, goal, velocity));
  geometry_msgs::msg::Pose tolerance;
  EXPECT_TRUE(checker->getTolerances(tolerance, velocity));
  EXPECT_DOUBLE_EQ(tolerance.position.x, 0.25);
}

TEST(StableGoalChecker, PositionOnlyUsesFiveCentimetresWithoutHeadingOrCaptureHysteresis)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("goal_checker.xy_goal_tolerance", 0.05),
    rclcpp::Parameter("goal_checker.check_yaw", false)});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("position_goal_test", options);
  pluginlib::ClassLoader<nav2_core::GoalChecker> loader("nav2_core", "nav2_core::GoalChecker");
  auto checker = loader.createSharedInstance("go2_navigation::StableGoalChecker");
  checker->initialize(node, "goal_checker", nullptr);
  geometry_msgs::msg::Pose goal, pose;
  geometry_msgs::msg::Twist velocity;
  goal.orientation.w = 1.0;
  pose.orientation.z = 1.0;  // Opposite heading is acceptable.
  pose.position.x = 0.05;
  EXPECT_TRUE(checker->isGoalReached(pose, goal, velocity));
  checker->reset();
  pose.position.x = 0.051;
  EXPECT_FALSE(checker->isGoalReached(pose, goal, velocity));
  pose.position.x = 0.03;
  pose.position.y = 0.04;
  EXPECT_TRUE(checker->isGoalReached(pose, goal, velocity));
  pose.position.y = 0.041;
  EXPECT_FALSE(checker->isGoalReached(pose, goal, velocity));
}

TEST(StableGoalChecker, TenCentimetreArrivalAcceptsAllHeadingsAndKeepsCircularBoundary)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("goal_checker.xy_goal_tolerance", 0.10),
    rclcpp::Parameter("goal_checker.check_yaw", false)});
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("ten_cm_goal_test",options);
  pluginlib::ClassLoader<nav2_core::GoalChecker> loader("nav2_core","nav2_core::GoalChecker");
  auto checker=loader.createSharedInstance("go2_navigation::StableGoalChecker");
  checker->initialize(node,"goal_checker",nullptr);
  geometry_msgs::msg::Pose goal,pose;geometry_msgs::msg::Twist velocity;
  goal.orientation.w=1.;pose.position.x=.06;pose.position.y=.08;
  for (int degree=-180;degree<=180;degree+=15) {
    double yaw=degree*M_PI/180.;pose.orientation.z=std::sin(yaw/2.);pose.orientation.w=std::cos(yaw/2.);
    EXPECT_TRUE(checker->isGoalReached(pose,goal,velocity));
    checker->reset();pose.position.y=.0801;
    EXPECT_FALSE(checker->isGoalReached(pose,goal,velocity));pose.position.y=.08;
  }
}
