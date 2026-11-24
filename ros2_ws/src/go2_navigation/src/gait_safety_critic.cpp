// Copyright 2026 Go2 Navigation Contributors
// SPDX-License-Identifier: Apache-2.0

#include <memory>
#include <sstream>
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "go2_navigation/costmap_geometry.hpp"
#include "go2_navigation/execution_safety.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/string.hpp"

namespace go2_navigation
{
class GaitSafetyCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override
  {
    auto node = node_.lock();
    auto parameter = [&](const std::string & key, double value) {
        const auto name = dwb_plugin_name_ + "." + name_ + "." + key;
        nav2_util::declare_parameter_if_not_declared(node, name, rclcpp::ParameterValue(value));
        return node->get_parameter(name).as_double();
      };
    minimum_ = parameter("minimum_translation", 0.22);
    history_timeout_ = parameter("history_timeout", 0.5);
    turn_commitment_seconds_ = parameter("turn_commitment_seconds", 2.0);
    if (!std::isfinite(minimum_) || minimum_ <= 0.0 ||
      !std::isfinite(history_timeout_) || history_timeout_ <= 0.0 ||
      !std::isfinite(turn_commitment_seconds_) || turn_commitment_seconds_ < 0.0)
    {throw std::runtime_error("Invalid gait safety critic parameters");}
    clock_ = node->get_clock();
    execution_safety_ = std::make_unique<ExecutionSafety>(
      node, dwb_plugin_name_, costmap_ros_->getBaseFrameID());
    diagnostics_ = rclcpp::create_publisher<std_msgs::msg::String>(
      node, "/navigation/trajectory_diagnostics", rclcpp::QoS(1).best_effort());
  }

  // DWB resets critics on every global replan. Command continuity belongs to
  // a navigation goal, not the latest 2 Hz copy of its path.
  void reset() override {}

  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D &) override
  {
    const double now = clock_->now().seconds();
    if (!have_goal_ || std::hypot(goal.x - goal_.x, goal.y - goal_.y) > 0.15 ||
      now < previous_time_ || now - previous_time_ > history_timeout_)
    {have_previous_ = false; turn_commitment_.reset();}
    evaluation_time_ = now;
    goal_ = goal; have_goal_ = true;
    execution_ = execution_safety_->snapshot(
      costmap_ros_->getGlobalFrameID(), pose.x, pose.y, pose.theta);
    const auto footprint = execution_.enabled ?
      std::make_pair(execution_.half_length, execution_.half_width) :
      footprint_half_extents(costmap_ros_->getRobotFootprint());
    const auto points = execution_.enabled ? execution_.points :
      local_obstacles(*costmap_ros_->getCostmap(), pose.x, pose.y,
      pose.theta, costmap_ros_->getLayeredCostmap()->isTrackingUnknown());
    limits_ = clearance_limits(points, footprint.first, footprint.second);
    turn_limits_ = clearance_limits(points, footprint.first, footprint.second, true);
    // Only commit while the previous same-side gait remains executable NOW.
    // Do not forbid an escape because the old side became occupied/stale.
    committed_turn_clear_ = have_previous_ && execution_.enabled &&
      std::hypot(previous_.x, previous_.y) < 1.e-5 &&
      std::abs(previous_.theta) <= turn_limits_.angular + 1.e-5 &&
      execution_.allows(0., 0., previous_.theta);
    rejected_ = rejected_execution_ = rejected_turn_ = candidates_ = 0;
    return true;
  }

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    ++candidates_;
    const auto & v = trajectory.velocity;
    const double speed = std::hypot(v.x, v.y);
    const auto & limits = speed < 1.e-5 ? turn_limits_ : limits_;
    if (!std::isfinite(speed) || !std::isfinite(v.theta) ||
      (speed > 1.e-5 && speed + 1.e-5 < minimum_) ||
      speed > limits.linear + 1.e-5 || std::abs(v.theta) > limits.angular + 1.e-5)
    {
      ++rejected_;
      throw dwb_core::IllegalTrajectoryException(name_, "Outside gait/clearance velocity envelope");
    }
    if (!execution_.allows(v.x, v.y, v.theta)) {
      ++rejected_execution_;
      throw dwb_core::IllegalTrajectoryException(
              name_, "Physical arc would be stopped or reduced below gait by the final safety gate");
    }
    if (!turn_commitment_.permits(
        v.x, v.y, v.theta, evaluation_time_, committed_turn_clear_))
    {
      ++rejected_turn_;
      throw dwb_core::IllegalTrajectoryException(
              name_, "Opposite pure turn deferred while committed direction remains collision-free");
    }
    return have_previous_ ? continuity_cost(v.x, v.y, v.theta,
      previous_.x, previous_.y, previous_.theta) : 0.0;
  }

  void debrief(const nav_2d_msgs::msg::Twist2D & command) override
  {
    previous_ = command; previous_time_ = clock_->now().seconds(); have_previous_ = true;
    turn_commitment_.record(command.x, command.y, command.theta,
      previous_time_, turn_commitment_seconds_);
    if (previous_time_ - last_diagnostic_ >= 0.5) {
      last_diagnostic_ = previous_time_;
      std::ostringstream out;
      out << "{\"linear_limit\":" << limits_.linear << ",\"angular_limit\":" << limits_.angular
          << ",\"candidates\":" << candidates_ << ",\"rejected_velocity\":" << rejected_
          << ",\"rejected_execution\":" << rejected_execution_
          << ",\"rejected_turn_reversal\":" << rejected_turn_
          << ",\"turn_direction\":" << turn_commitment_.direction()
          << ",\"execution_ready\":" << (execution_.ready ? "true" : "false")
          << ",\"command\":[" << command.x << ',' << command.theta << "]}";
      std_msgs::msg::String message; message.data = out.str(); diagnostics_->publish(message);
    }
  }

private:
  MotionLimits limits_, turn_limits_;
  std::unique_ptr<ExecutionSafety> execution_safety_;
  ExecutionContext execution_;
  geometry_msgs::msg::Pose2D goal_;
  nav_2d_msgs::msg::Twist2D previous_;
  bool have_goal_{false}, have_previous_{false};
  double minimum_{0.22}, history_timeout_{0.5}, previous_time_{0.0}, last_diagnostic_{-1.e30};
  double turn_commitment_seconds_{2.0}, evaluation_time_{0.0};
  bool committed_turn_clear_{false};
  PureTurnCommitment turn_commitment_;
  unsigned int candidates_{0}, rejected_{0}, rejected_execution_{0}, rejected_turn_{0};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_;
};
}  // namespace go2_navigation
PLUGINLIB_EXPORT_CLASS(go2_navigation::GaitSafetyCritic, dwb_core::TrajectoryCritic)
