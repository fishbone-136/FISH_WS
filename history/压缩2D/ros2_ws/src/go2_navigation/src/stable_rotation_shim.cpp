// Copyright 2026 Go2 Navigation Contributors
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "go2_navigation/rotation_helpers.hpp"
#include "nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp"

namespace go2_navigation
{

class RotationBlocked : public nav2_core::PlannerException
{
public:
  RotationBlocked() : nav2_core::PlannerException("Stable rotation footprint is obstructed") {}
};

class StableRotationShim : public nav2_rotation_shim_controller::RotationShimController
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    RotationShimController::configure(parent, name, std::move(tf), std::move(costmap));
    auto node = parent.lock();
    auto parameter = [&](const std::string & key, double value) {
        nav2_util::declare_parameter_if_not_declared(
          node, name + "." + key, rclcpp::ParameterValue(value));
        return node->get_parameter(name + "." + key).as_double();
      };
    min_rotation_speed_ = parameter("min_rotation_angular_vel", 0.55);
    final_rotation_speed_ = parameter("final_rotation_angular_vel", 0.60);
    rotation_gain_ = parameter("rotation_gain", 1.8);
    max_transform_age_ = parameter("max_transform_age", 0.8);
    min_final_rotation_speed_ = parameter("final_min_rotation_angular_vel", 0.50);
    goal_hysteresis_ = parameter("goal_position_hysteresis", 0.12);
    goal_identity_ = parameter("goal_identity_distance", 0.15);
    approach_distance_ = parameter("goal_approach_distance", 0.90);
    approach_speed_ = parameter("goal_approach_speed", 0.24);
    position_approach_distance_ = parameter("position_approach_distance", 0.30);
    if (min_rotation_speed_ < 0.0 || final_rotation_speed_ <= 0.0 ||
      rotation_gain_ <= 0.0 || max_transform_age_ <= 0.0 ||
      rotate_to_heading_angular_vel_ <= 0.0 || max_angular_accel_ <= 0.0 ||
      min_final_rotation_speed_ <= 0.0 || goal_hysteresis_ < 0.0 || goal_identity_ <= 0.0 ||
      approach_distance_ <= 0.0 || approach_speed_ <= 0.0 || position_approach_distance_ <= 0.0)
    {
      throw std::runtime_error("Invalid stable rotation controller limits");
    }
    RCLCPP_INFO(
      logger_, "Stable rotation: fresh latest TF, rear-direction latch, %.2f rad/s path turn",
      rotate_to_heading_angular_vel_);
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    RotationShimController::setPlan(path);
    std::lock_guard<std::mutex> lock(mutex_);
    const bool same_goal = have_goal_ && !path.poses.empty() &&
      same_navigation_goal(last_goal_, path.poses.back(), goal_identity_);
    if (!same_goal) {
      direction_.reset();
      goal_direction_.reset();
      in_rotation_ = false;
      last_angular_vel_ = 0.0;
      goal_capture_.reset();
    }
    have_goal_ = !path.poses.empty();
    if (have_goal_) {last_goal_ = path.poses.back();}
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(
      *costmap_ros_->getCostmap()->getMutex());
    std::unique_lock<std::mutex> state_lock(mutex_);
    if (current_path_.poses.empty() || goal_checker == nullptr) {
      throw nav2_core::PlannerException("Stable rotation needs a path and goal checker");
    }
    const auto current_time = clock_->now();
    const rclcpp::Time pose_stamp(pose.header.stamp, current_time.get_clock_type());
    if (pose_stamp.nanoseconds() == 0 ||
      (current_time - pose_stamp).seconds() > max_transform_age_)
    {
      throw nav2_core::PlannerException("Robot pose is stale during path alignment");
    }
    auto goal = current_path_.poses.back();
    goal.header.frame_id = current_path_.header.frame_id;
    goal = latest_path_pose(*tf_, goal, pose.header.frame_id, current_time, max_transform_age_);
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    if (!goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
      throw nav2_core::PlannerException("Goal checker did not provide tolerances");
    }
    const double goal_distance = std::hypot(
      pose.pose.position.x - goal.pose.position.x, pose.pose.position.y - goal.pose.position.y);
    if (!rotate_to_goal_heading_ && goal_distance < position_approach_distance_) {
      // ControllerServer checks the strict goal radius after this command,
      // using a fresh pose. An independent stop here can make a single noisy
      // boundary sample stop the gait while the action remains active.
      if (auto command = approach_position(
          pose, goal, goal_distance, pose_tolerance.position.x))
      {
        last_angular_vel_ = command->twist.angular.z;
        return *command;
      }
      // If the direct final arc is obstructed, retain full DWB obstacle detours.
    }
    if (rotate_to_goal_heading_ && goal_capture_.update(
        goal_distance, pose_tolerance.position.x, goal_hysteresis_))
    {
      const double yaw_tolerance = std::abs(tf2::getYaw(pose_tolerance.orientation));
      const double error = goal_direction_.error(angles::shortest_angular_distance(
          tf2::getYaw(pose.pose.orientation), tf2::getYaw(goal.pose.orientation)));
      return rotate(error, yaw_tolerance, final_rotation_speed_, pose, min_final_rotation_speed_);
    }

    if (path_updated_) {
      // Even a path shorter than the lookahead still needs heading alignment
      // when its goal is behind the robot.
      auto sample = current_path_.poses.back();
      const auto & start = current_path_.poses.front().pose.position;
      for (const auto & candidate : current_path_.poses) {
        if (std::hypot(candidate.pose.position.x - start.x,
          candidate.pose.position.y - start.y) >= forward_sampling_distance_)
        {
          sample = candidate;
          break;
        }
      }
      sample.header.frame_id = current_path_.header.frame_id;
      sample = latest_path_pose(
        *tf_, sample, pose.header.frame_id, current_time, max_transform_age_);
      const double heading = std::atan2(
        sample.pose.position.y - pose.pose.position.y,
        sample.pose.position.x - pose.pose.position.x);
      const double error = direction_.error(angles::shortest_angular_distance(
          tf2::getYaw(pose.pose.orientation), heading));
      const double threshold = in_rotation_ ?
        angular_disengage_threshold_ : angular_dist_threshold_;
      if (std::abs(error) > threshold) {
        if (!in_rotation_) {
          RCLCPP_INFO(logger_, "Aligning path: error %.2f rad, direction %s", error,
            error > 0.0 ? "left" : "right");
        }
        in_rotation_ = true;
        try {
          return rotate(error, angular_disengage_threshold_, rotate_to_heading_angular_vel_,
                   pose, min_rotation_speed_);
        } catch (const RotationBlocked &) {
          // A rear goal admits two initial turn directions. Check the other
          // sweep before dropping into path tracking with the same blocked
          // initial turn on every replan. Retain the selected direction at pi.
          if (std::abs(error)>2.8) {
            const double alternate=error-std::copysign(2.0*3.14159265358979323846,error);
            if (checked_rotation_limit(alternate,rotate_to_heading_angular_vel_,min_rotation_speed_,pose)>0.) {
              direction_.select(alternate);
              if (last_angular_vel_*alternate<0.) {
                geometry_msgs::msg::TwistStamped stopped;stopped.header=pose.header;
                last_angular_vel_=0.;return stopped;
              }
              RCLCPP_INFO_THROTTLE(logger_,*clock_,2000,
                "Rear alignment: blocked side rejected; taking the other checked turn");
              return rotate(alternate,angular_disengage_threshold_,rotate_to_heading_angular_vel_,
                            pose,min_rotation_speed_);
            }
          }
          // A rectangular quadruped may lack room to rotate in place while a
          // forward arc is clear. Let the primary controller try that arc with
          // its full footprint critic, followed by the independent scan gate.
          // Stale TF and all other errors still propagate as stop/retry.
          RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000,
            "In-place alignment blocked; trying footprint-checked path tracking");
          in_rotation_ = false;
        }
      }
      path_updated_ = false;
      if (in_rotation_) {
        RCLCPP_INFO(logger_, "Path aligned; handing over to path tracking");
      }
    }
    in_rotation_ = false;
    state_lock.unlock();
    costmap_lock.unlock();
    // TF failure and collision exceptions above propagate to ControllerServer
    // for a stop/retry. They must never silently abandon heading alignment.
    auto command = primary_controller_->computeVelocityCommands(pose, velocity, goal_checker);
    const double speed = std::hypot(command.twist.linear.x, command.twist.linear.y);
    const double speed_limit = approach_speed_limit(
      goal_distance, pose_tolerance.position.x, approach_distance_, approach_speed_, speed);
    if (speed > speed_limit) {
      // Uniform scaling preserves the curvature of the collision-checked arc.
      const double ratio = speed_limit / speed;
      command.twist.linear.x *= ratio;
      command.twist.linear.y *= ratio;
      command.twist.angular.z *= ratio;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    last_angular_vel_ = command.twist.angular.z;
    return command;
  }

private:
  bool rotation_clear(double error,double velocity,const geometry_msgs::msg::PoseStamped & pose)
  {
    const double yaw=tf2::getYaw(pose.pose.orientation);
    for (const double offset:rotation_sweep(error,velocity,simulate_ahead_time_)) {
      const double cost=collision_checker_->footprintCostAtPose(
        pose.pose.position.x,pose.pose.position.y,yaw+offset,costmap_ros_->getRobotFootprint());
      if (cost<0. || cost>=nav2_costmap_2d::LETHAL_OBSTACLE ||
          (cost==nav2_costmap_2d::NO_INFORMATION && costmap_ros_->getLayeredCostmap()->isTrackingUnknown()))
        return false;
    }
    return true;
  }

  double checked_rotation_limit(double error,double maximum,double minimum,
    const geometry_msgs::msg::PoseStamped & pose)
  {
    // A blocked far-future sweep at maximum yaw must not immediately abandon
    // stable path alignment for a jittering DWB arc. Try slower complete
    // footprint-checked sweeps first; retain the selected turning direction.
    for(double factor:{1.,.85,.70,.55}) {
      const double limit=std::max(std::min(minimum,maximum),maximum*factor);
      if(rotation_clear(error,std::copysign(limit,error),pose))return limit;
    }
    const double floor=std::min(minimum,maximum);
    return rotation_clear(error,std::copysign(floor,error),pose) ? floor : 0.;
  }

  std::optional<geometry_msgs::msg::TwistStamped> approach_position(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::PoseStamped & goal, double distance, double tolerance)
  {
    const double yaw = tf2::getYaw(pose.pose.orientation);
    const double heading = std::atan2(goal.pose.position.y-pose.pose.position.y,
      goal.pose.position.x-pose.pose.position.x);
    const double bearing = direction_.error(angles::shortest_angular_distance(yaw, heading));
    const double error = position_approach_error(bearing, distance, tolerance);
    const double vx = approach_speed_;
    const double wz = position_approach_yaw_rate(
      bearing, distance, tolerance, vx, rotate_to_heading_angular_vel_);
    if (std::abs(error) > angular_dist_threshold_ ||
      approach_arc_miss(bearing, distance, vx, wz) > .9 * tolerance)
    {
      try {
        return rotate(error, angular_disengage_threshold_, rotate_to_heading_angular_vel_,
          pose, min_rotation_speed_);
      } catch (const RotationBlocked &) {
        return std::nullopt;
      }
    }
    // At the last few centimetres, DWB's full trajectory horizon can favour
    // sub-gait forward samples plus turning forever. Approach position with a
    // usable gait command, ignoring the requested final orientation entirely.
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    const double arc_duration = std::abs(wz) < 1.e-6 ? distance/vx : 2.*std::abs(error/wz);
    const double duration = std::min(1.5, std::max(distance/vx, arc_duration));
    const int steps = std::max(1, static_cast<int>(std::ceil(duration/.025)));
    for (int i = 0; i <= steps; ++i) {
      const double t = duration*i/steps;
      const double dx = std::abs(wz) < 1.e-6 ? vx*t : vx*std::sin(wz*t)/wz;
      const double dy = std::abs(wz) < 1.e-6 ? 0.0 : vx*(1.0-std::cos(wz*t))/wz;
      const double cost = collision_checker_->footprintCostAtPose(
        pose.pose.position.x+std::cos(yaw)*dx-std::sin(yaw)*dy,
        pose.pose.position.y+std::sin(yaw)*dx+std::cos(yaw)*dy,
        yaw+wz*t, costmap_ros_->getRobotFootprint());
      if (cost < 0.0 || cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        return std::nullopt;
      }
    }
    command.twist.linear.x = vx;
    command.twist.angular.z = wz;
    return command;
  }

  geometry_msgs::msg::TwistStamped rotate(
    double error, double tolerance, double maximum,
    const geometry_msgs::msg::PoseStamped & pose, double minimum)
  {
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    const double previous = std::isfinite(last_angular_vel_) &&
      std::abs(last_angular_vel_) <= rotate_to_heading_angular_vel_ ? last_angular_vel_ : 0.0;
    const double checked_maximum=checked_rotation_limit(error,maximum,minimum,pose);
    if(checked_maximum<=0.)throw RotationBlocked();
    command.twist.angular.z = rotation_velocity(
      error, tolerance, previous, checked_maximum, minimum, rotation_gain_,
      max_angular_accel_, control_duration_);
    // Check both turning directions through the same angle. The inherited
    // Humble check uses a signed stopping comparison which is asymmetric.
    // Reserve the turn at its requested maximum speed from the first ramp
    // sample. Checking only that tiny first command misses the future corner
    // sweep until the quadruped is already too close to stop its gait.
    const double sweep_velocity = std::copysign(std::max(checked_maximum,
      std::abs(command.twist.angular.z)),command.twist.angular.z);
    if ((std::abs(command.twist.angular.z)>checked_maximum+1.e-9 ||
      command.twist.angular.z*error<0.) && !rotation_clear(error,sweep_velocity,pose))
      throw RotationBlocked();
    last_angular_vel_ = command.twist.angular.z;
    return command;
  }

  RearTurnDirection direction_;
  GoalCapture goal_capture_;
  geometry_msgs::msg::PoseStamped last_goal_;
  bool have_goal_{false};
  RearTurnDirection goal_direction_;
  double min_rotation_speed_{0.55};
  double final_rotation_speed_{0.60};
  double rotation_gain_{1.8};
  double max_transform_age_{0.8};
  double min_final_rotation_speed_{0.50};
  double goal_hysteresis_{0.12}, goal_identity_{0.15};
  double approach_distance_{0.90}, approach_speed_{0.24};
  double position_approach_distance_{0.30};
};

}  // namespace go2_navigation

PLUGINLIB_EXPORT_CLASS(go2_navigation::StableRotationShim, nav2_core::Controller)
