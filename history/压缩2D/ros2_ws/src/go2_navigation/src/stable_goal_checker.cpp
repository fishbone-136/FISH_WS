// Copyright 2026 Go2 Navigation Contributors
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "go2_navigation/rotation_helpers.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace go2_navigation
{
// ControllerServer resets the checker for every replanned path. Preserve XY
// capture only for the same geometric goal and only inside its release radius.
class StableGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
    auto node = parent.lock();
    auto parameter = [&](const std::string & key, double value) {
        nav2_util::declare_parameter_if_not_declared(
          node, name + "." + key, rclcpp::ParameterValue(value));
        return node->get_parameter(name + "." + key).as_double();
      };
    xy_ = parameter("xy_goal_tolerance", 0.25);
    yaw_ = parameter("yaw_goal_tolerance", 0.40);
    hysteresis_ = parameter("position_hysteresis", 0.12);
    identity_ = parameter("goal_identity_distance", 0.15);
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".stateful", rclcpp::ParameterValue(true));
    stateful_ = node->get_parameter(name + ".stateful").as_bool();
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".check_yaw", rclcpp::ParameterValue(true));
    check_yaw_ = node->get_parameter(name + ".check_yaw").as_bool();
    if (!std::isfinite(xy_ + yaw_ + hysteresis_ + identity_) || xy_ <= 0.0 ||
      yaw_ <= 0.0 || (check_yaw_ && yaw_ >= M_PI) || hysteresis_ < 0.0 || identity_ <= 0.0)
    {
      throw std::runtime_error("Invalid stable goal checker limits");
    }
  }

  void reset() override
  {
    // A real new goal is detected below. Repeated reset() calls for an
    // unchanged goal must not turn final-heading alignment into XY hunting.
    if (!stateful_) {capture_.reset(); have_goal_ = false;}
  }

  bool isGoalReached(
    const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Pose & goal,
    const geometry_msgs::msg::Twist &) override
  {
    geometry_msgs::msg::PoseStamped requested;
    requested.pose = goal;
    if (!have_goal_ || !same_navigation_goal(last_goal_, requested, identity_)) {
      capture_.reset();
      last_goal_ = requested;
      have_goal_ = true;
    }
    const double distance = std::hypot(
      pose.position.x - goal.position.x, pose.position.y - goal.position.y);
    // Position-only arrival is strict. Capturing XY on an earlier sample must
    // never turn a later pose outside the requested radius into success.
    if (!check_yaw_) {return distance <= xy_;}
    const bool captured = stateful_ ? capture_.update(distance, xy_, hysteresis_) : distance <= xy_;
    const double error = std::remainder(
      tf2::getYaw(goal.orientation) - tf2::getYaw(pose.orientation), 2.0 * M_PI);
    return captured && std::isfinite(error) && std::abs(error) <= yaw_;
  }

  bool getTolerances(
    geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & velocity) override
  {
    pose = geometry_msgs::msg::Pose();
    pose.position.x = pose.position.y = xy_;
    const double yaw_tolerance = check_yaw_ ? yaw_ : M_PI;
    pose.orientation.z = std::sin(yaw_tolerance / 2.0);
    pose.orientation.w = std::cos(yaw_tolerance / 2.0);
    const double unused = std::numeric_limits<double>::lowest();
    velocity.linear.x = velocity.linear.y = velocity.linear.z = unused;
    velocity.angular.x = velocity.angular.y = velocity.angular.z = unused;
    return true;
  }

private:
  GoalCapture capture_;
  geometry_msgs::msg::PoseStamped last_goal_;
  bool have_goal_{false};
  bool stateful_{true};
  bool check_yaw_{true};
  double xy_{0.25}, yaw_{0.40}, hysteresis_{0.12}, identity_{0.15};
};
}  // namespace go2_navigation

PLUGINLIB_EXPORT_CLASS(go2_navigation::StableGoalChecker, nav2_core::GoalChecker)
