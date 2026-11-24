// Copyright 2026 Go2 Navigation Contributors
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include "go2_navigation/rotation_helpers.hpp"
#include "go2_navigation/costmap_geometry.hpp"
#include "go2_navigation/execution_safety.hpp"
#include "nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp"
#include "std_msgs/msg/string.hpp"

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
    execution_safety_ = std::make_unique<ExecutionSafety>(
      node, name, costmap_ros_->getBaseFrameID());
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
    corridor_max_width_ = parameter("corridor_max_width", 1.20);
    corridor_margin_ = parameter("corridor_margin", 0.02);
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".corridor_alignment", rclcpp::ParameterValue(true));
    corridor_enabled_ = node->get_parameter(name + ".corridor_alignment").as_bool();
    diagnostics_ = rclcpp::create_publisher<std_msgs::msg::String>(
      node, "/navigation/controller_diagnostics", rclcpp::QoS(1).best_effort());
    if (min_rotation_speed_ < 0.0 || final_rotation_speed_ <= 0.0 ||
      rotation_gain_ <= 0.0 || max_transform_age_ <= 0.0 ||
      rotate_to_heading_angular_vel_ <= 0.0 || max_angular_accel_ <= 0.0 ||
      min_final_rotation_speed_ <= 0.0 || goal_hysteresis_ < 0.0 || goal_identity_ <= 0.0 ||
      approach_distance_ <= 0.0 || approach_speed_ <= 0.0 || position_approach_distance_ <= 0.0 ||
      !std::isfinite(corridor_max_width_) || corridor_max_width_ <= 0.0 ||
      !std::isfinite(corridor_margin_) || corridor_margin_ < 0.0)
    {
      throw std::runtime_error("Invalid stable rotation controller limits");
    }
    RCLCPP_INFO(
      logger_, "Stable rotation: checked turn selection and measured yaw progress, %.2f rad/s path turn",
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
      path_turn_.reset();
      in_rotation_ = false;
      last_angular_vel_ = 0.0;
      goal_capture_.reset();
      handoff_.reset();
      corridor_misses_ = 0;
      corridor_ = {};
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
    const double yaw = tf2::getYaw(pose.pose.orientation);
    execution_ = execution_safety_->snapshot(costmap_ros_->getGlobalFrameID(),
      pose.pose.position.x, pose.pose.position.y, yaw);
    if (execution_.enabled && !execution_.ready) {
      throw nav2_core::PlannerException("Execution safety observations are unavailable or stale");
    }
    const auto footprint = execution_.enabled ?
      std::make_pair(execution_.half_length, execution_.half_width) :
      footprint_half_extents(costmap_ros_->getRobotFootprint());
    obstacles_ = execution_.enabled ? execution_.points :
      local_obstacles(*costmap_ros_->getCostmap(), pose.pose.position.x,
      pose.pose.position.y, yaw, costmap_ros_->getLayeredCostmap()->isTrackingUnknown());
    turn_speed_limit_ = clearance_limits(obstacles_, footprint.first, footprint.second, true).angular;
    auto sampled = sampled_path(pose, current_time);
    const auto & sample = sampled.second;
    const double bearing = std::atan2(sample.pose.position.y - pose.pose.position.y,
      sample.pose.position.x - pose.pose.position.x);
    const double tangent = std::atan2(sample.pose.position.y - sampled.first.pose.position.y,
      sample.pose.position.x - sampled.first.pose.position.x);
    // Use current wall observations for steering. Historical obstacles are
    // still checked by the execution envelope and the local costmap.
    const auto detected = corridor_enabled_ ? detect_path_corridor(
      execution_.enabled ? execution_.raw : obstacles_,
      angles::shortest_angular_distance(yaw, tangent), corridor_max_width_) : Corridor{};
    if (detected.detected) {
      corridor_ = detected; corridor_misses_ = 0;
    } else if (++corridor_misses_ >= 3) {corridor_ = {};}
    // Short hysteresis changes only alignment preference. Every command is
    // still checked against the current costmap and the independent scan gate.
    const bool narrow = corridor_.detected;
    if (detected.detected && detected.width() < 2.0 * std::min(footprint.first, footprint.second)) {
      std::vector<ObstaclePoint> local_path;
      const double c = std::cos(yaw), s = std::sin(yaw);
      for (auto point : current_path_.poses) {
        point.header.frame_id = current_path_.header.frame_id;
        point = latest_path_pose(*tf_, point, pose.header.frame_id, current_time, max_transform_age_);
        const double dx = point.pose.position.x - pose.pose.position.x;
        const double dy = point.pose.position.y - pose.pose.position.y;
        local_path.push_back({c * dx + s * dy, -s * dx + c * dy});
      }
      if (path_enters_corridor(detected, local_path)) {
        geometry_msgs::msg::TwistStamped stopped; stopped.header = pose.header;
        publish_diagnostic("corridor_blocked", stopped, 0.0);
        throw nav2_core::PlannerException("Observed corridor is narrower than the protected body on this path");
      }
    }
    const double heading = detected.detected ? yaw + detected.heading : narrow ? tangent : bearing;
    const double narrow_tolerance = narrow ? corridor_heading_tolerance(
      corridor_, footprint.first, footprint.second, corridor_margin_) : 0.0;
    const double disengage = narrow && narrow_tolerance > 0.0 ?
      std::min(angular_disengage_threshold_, 0.5 * narrow_tolerance) : angular_disengage_threshold_;
    const double engage = narrow && narrow_tolerance > 0.0 ?
      std::min(angular_dist_threshold_, 0.8 * narrow_tolerance) : angular_dist_threshold_;
    const double sample_x = sample.pose.position.x - pose.pose.position.x;
    const double sample_y = sample.pose.position.y - pose.pose.position.y;
    const double c = std::cos(yaw), s = std::sin(yaw);
    if (execution_.enabled && detected.detected && detected.width() <= .70 &&
      detected.width() >= 2.0 * execution_.half_width &&
      std::abs(detected.heading) <= .25 && goal_distance > position_approach_distance_ &&
      path_enters_corridor(detected, {{c * sample_x + s * sample_y, -s * sample_x + c * sample_y}}))
    {
      // Centre between the measured walls with a slow arc. The grid path can
      // be several centimetres off-centre even in a straight passage.
      const double vx = std::min(.24, approach_speed_);
      const double wz = corridor_center_yaw(detected, vx, forward_sampling_distance_);
      if (vx >= execution_.minimum_translation && execution_.allows(vx, 0., wz) &&
        walking_arc_clear(pose, vx, wz, std::min(simulate_ahead_time_, goal_distance / vx)))
      {
        geometry_msgs::msg::TwistStamped command;
        command.header = pose.header;
        command.twist.linear.x = vx;
        command.twist.angular.z = wz;
        in_rotation_ = false;
        path_turn_.reset();
        path_updated_ = false;
        last_angular_vel_ = wz;
        publish_diagnostic("corridor_center", command, narrow_tolerance);
        return command;
      }
    }
    if (!rotate_to_goal_heading_ && goal_distance < position_approach_distance_) {
      // ControllerServer checks the strict goal radius after this command,
      // using a fresh pose. An independent stop here can make a single noisy
      // boundary sample stop the gait while the action remains active.
      if (auto command = approach_position(
          pose, goal, goal_distance, pose_tolerance.position.x))
      {
        last_angular_vel_ = command->twist.angular.z;
        publish_diagnostic("goal_approach", *command, narrow_tolerance);
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

    const double shortest_error = angles::shortest_angular_distance(yaw, heading);
    const double now = current_time.seconds();
    path_turn_.update(heading, yaw, now);
    if (path_updated_ || in_rotation_ || narrow) {
      const double threshold = in_rotation_ ? disengage : engage;
      const bool tracking_arc = handoff_.tracking(heading, shortest_error, disengage);
      if (path_turn_.escape() &&
        path_turn_.remaining(shortest_error, yaw) * path_turn_.direction() <= 0.07)
      {
        path_turn_.reset();
        handoff_.blocked(heading);
        in_rotation_ = false;
        last_angular_vel_ = 0.0;
        geometry_msgs::msg::TwistStamped stopped;
        stopped.header = pose.header;
        publish_diagnostic("escape_complete", stopped, narrow_tolerance);
        return stopped;
      }
      if (std::abs(shortest_error) > threshold && !tracking_arc) {
        in_rotation_ = true;
        if (auto command = path_alignment(
            pose, velocity.angular.z, heading, yaw, now, shortest_error, disengage))
        {
          publish_diagnostic(path_turn_.alternative() ? "alternate_align" :
            narrow ? "corridor_align" : "path_align", *command, narrow_tolerance);
          return *command;
        }
        // A translating arc may fit when the quadruped cannot pivot in place.
        in_rotation_ = false;
        handoff_.blocked(heading);
      } else if (std::abs(shortest_error) <= threshold) {
        path_turn_.reset();
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
    geometry_msgs::msg::TwistStamped command;
    try {
      command = primary_controller_->computeVelocityCommands(pose, velocity, goal_checker);
    } catch (const nav2_core::PlannerException &) {
      // A previously useful arc is no longer available. Reconsider checked
      // alignment on the next cycle instead of keeping a stale handover latch.
      std::lock_guard<std::mutex> lock(mutex_);
      handoff_.reset(); path_updated_ = true;
      throw;
    }
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
    if (!execution_.allows(command.twist.linear.x, command.twist.linear.y,
      command.twist.angular.z))
    {
      handoff_.reset();
      path_updated_ = true;
      throw nav2_core::PlannerException("Selected command is not physically executable");
    }
    if (speed >= 0.10) {
      path_turn_.reset();
    } else if (path_turn_.stalled() && command.twist.angular.z * path_turn_.direction() > 0.) {
      last_angular_vel_ = 0.;
      throw nav2_core::PlannerException("Path tracking selected a turn with no measured progress");
    }
    last_angular_vel_ = command.twist.angular.z;
    publish_diagnostic(narrow ? "corridor_track" : "path_track", command, narrow_tolerance);
    return command;
  }

private:
  std::optional<geometry_msgs::msg::TwistStamped> path_alignment(
    const geometry_msgs::msg::PoseStamped & pose, double measured_angular,
    double heading, double yaw, double now, double shortest_error, double tolerance)
  {
    const double error = path_turn_.remaining(shortest_error, yaw);
    const bool continuing = path_turn_.active() && error * path_turn_.direction() > 0.;
    const bool stalled = continuing && path_turn_.stalled();
    const auto checked_command = [&](double turn, double target_tolerance)
      -> std::optional<geometry_msgs::msg::TwistStamped> {
        const double limit = checked_rotation_limit(
          turn, rotate_to_heading_angular_vel_, min_rotation_speed_, pose);
        if (limit <= 0.) {return std::nullopt;}
        const double direction = std::copysign(1., turn);
        if (last_angular_vel_ * direction < 0. || measured_angular * direction < -0.10) {
          last_angular_vel_ = 0.;
          geometry_msgs::msg::TwistStamped stopped;
          stopped.header = pose.header;
          return stopped;
        }
        try {
          return rotate(turn, target_tolerance, rotate_to_heading_angular_vel_,
            pose, min_rotation_speed_, limit);
        } catch (const RotationBlocked &) {
          return std::nullopt;
        }
      };
    if (!stalled) {
      if (auto command = checked_command(error, path_turn_.escape() ? 0.07 : tolerance)) {
        if (!continuing && command->twist.angular.z != 0.) {
          path_turn_.start(heading, yaw, now, error);
        }
        return command;
      }
      // A direction rejected by geometry did not get a chance to move.
      // Re-evaluate both sides when fresh observations arrive.
      path_turn_.reset();
    }
    if (!path_turn_.alternative()) {
      const double alternate = PathAlignmentTurn::other_side(shortest_error, error);
      if (auto command = checked_command(alternate, std::min(0.07, tolerance))) {
        if (command->twist.angular.z != 0.) {
          path_turn_.start(heading, yaw, now, alternate, true);
        }
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000,
          "Path alignment: %s, switching to the checked %s turn",
          stalled ? "no measured yaw progress" : "preferred turn blocked",
          alternate < 0. ? "right" : "left");
        return command;
      }
    }
    return std::nullopt;
  }

  std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped> sampled_path(
    const geometry_msgs::msg::PoseStamped & pose, const rclcpp::Time & now)
  {
    // A fixed old path start becomes irrelevant while walking between replans.
    // Sample by arc length from the closest path point, in the current frame.
    const auto local_pose = latest_path_pose(*tf_, pose, current_path_.header.frame_id,
      now, max_transform_age_);
    std::size_t closest = 0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < current_path_.poses.size(); ++i) {
      const auto & p = current_path_.poses[i].pose.position;
      const double distance = std::hypot(p.x - local_pose.pose.position.x,
        p.y - local_pose.pose.position.y);
      if (distance < best) {best = distance; closest = i;}
    }
    auto start = current_path_.poses[closest];
    auto sample = start;
    double remaining = forward_sampling_distance_;
    for (std::size_t i = closest + 1; i < current_path_.poses.size(); ++i) {
      const auto & next = current_path_.poses[i];
      const double length = std::hypot(next.pose.position.x - sample.pose.position.x,
        next.pose.position.y - sample.pose.position.y);
      if (length > remaining && length > 1.e-6) {
        sample.pose.position.x += remaining / length * (next.pose.position.x - sample.pose.position.x);
        sample.pose.position.y += remaining / length * (next.pose.position.y - sample.pose.position.y);
        break;
      }
      remaining -= length; sample = next;
    }
    // At the final path point use its preceding tangent rather than atan2(0,0).
    if (closest > 0 && std::hypot(sample.pose.position.x - start.pose.position.x,
      sample.pose.position.y - start.pose.position.y) < 1.e-6)
    {start = current_path_.poses[closest - 1];}
    start.header.frame_id = sample.header.frame_id = current_path_.header.frame_id;
    return {latest_path_pose(*tf_, start, pose.header.frame_id, now, max_transform_age_),
      latest_path_pose(*tf_, sample, pose.header.frame_id, now, max_transform_age_)};
  }

  void publish_diagnostic(const std::string & mode,
    const geometry_msgs::msg::TwistStamped & command, double heading_tolerance)
  {
    const double now = clock_->now().seconds();
    if (mode == last_mode_ && now - last_diagnostic_ < 0.5) {return;}
    last_mode_ = mode; last_diagnostic_ = now;
    std::ostringstream out;
    out << "{\"mode\":\"" << mode << "\",\"corridor\":" << (corridor_.detected ? "true" : "false")
        << ",\"corridor_width\":" << corridor_.width()
        << ",\"heading_tolerance\":" << heading_tolerance
        << ",\"command\":[" << command.twist.linear.x << ',' << command.twist.angular.z << "]}";
    std_msgs::msg::String message; message.data = out.str(); diagnostics_->publish(message);
  }

  bool rotation_clear(double error,double velocity,const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!execution_.allows(0.0, 0.0, velocity)) {return false;}
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
    maximum = std::min(maximum, turn_speed_limit_);
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
    const auto footprint = execution_.enabled ?
      std::make_pair(execution_.half_length, execution_.half_width) :
      footprint_half_extents(costmap_ros_->getRobotFootprint());
    const auto limits = clearance_limits(obstacles_, footprint.first, footprint.second);
    double vx = std::min(approach_speed_, limits.linear);
    double wz = position_approach_yaw_rate(
      bearing, distance, tolerance, vx, std::min(rotate_to_heading_angular_vel_, limits.angular));
    if (std::abs(error) > angular_dist_threshold_ ||
      approach_arc_miss(bearing, distance, vx, wz) > .9 * tolerance)
    {
      try {
        return rotate(error, position_alignment_tolerance(error, angular_disengage_threshold_),
          rotate_to_heading_angular_vel_,
          pose, min_rotation_speed_);
      } catch (const RotationBlocked &) {
        return std::nullopt;
      }
    }
    // At the last few centimetres, DWB's full trajectory horizon can favour
    // sub-gait forward samples plus turning forever. Approach position with a
    // usable gait command, ignoring the requested final orientation entirely.
    const auto approach_clear = [&]() {
        const double arc_duration = std::abs(wz) < 1.e-6 ? distance / vx : 2. * std::abs(error / wz);
        const double duration = std::min(1.5, std::max(distance / vx, arc_duration));
        return execution_.allows(vx, 0., wz) && walking_arc_clear(pose, vx, wz, duration);
      };
    if (!approach_clear()) {
      if (!execution_.enabled || vx <= execution_.minimum_translation) {return std::nullopt;}
      // At a slower gait the command may pass without triggering a safety
      // reduction below the walking floor. Preserve its checked curvature.
      wz *= execution_.minimum_translation / vx;
      vx = execution_.minimum_translation;
      if (!approach_clear()) {return std::nullopt;}
    }
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    command.twist.linear.x = vx;
    command.twist.angular.z = wz;
    return command;
  }

  bool walking_arc_clear(
    const geometry_msgs::msg::PoseStamped & pose, double vx, double wz, double duration)
  {
    const double yaw = tf2::getYaw(pose.pose.orientation);
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
        return false;
      }
    }
    return true;
  }

  geometry_msgs::msg::TwistStamped rotate(
    double error, double tolerance, double maximum,
    const geometry_msgs::msg::PoseStamped & pose, double minimum,
    std::optional<double> checked_limit = std::nullopt)
  {
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    const double previous = std::isfinite(last_angular_vel_) &&
      std::abs(last_angular_vel_) <= rotate_to_heading_angular_vel_ ? last_angular_vel_ : 0.0;
    const double checked_maximum = checked_limit ? *checked_limit :
      checked_rotation_limit(error, maximum, minimum, pose);
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
  PathAlignmentTurn path_turn_;
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
  AlignmentHandoff handoff_;
  std::unique_ptr<ExecutionSafety> execution_safety_;
  ExecutionContext execution_;
  Corridor corridor_;
  std::vector<ObstaclePoint> obstacles_;
  bool corridor_enabled_{true};
  int corridor_misses_{0};
  double corridor_max_width_{1.2}, corridor_margin_{0.02}, turn_speed_limit_{1.4};
  std::string last_mode_;
  double last_diagnostic_{-1.e30};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_;
};

}  // namespace go2_navigation

PLUGINLIB_EXPORT_CLASS(go2_navigation::StableRotationShim, nav2_core::Controller)
