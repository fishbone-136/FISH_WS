// Copyright 2026 Go2 Navigation Contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef GO2_NAVIGATION__ROTATION_HELPERS_HPP_
#define GO2_NAVIGATION__ROTATION_HELPERS_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"

namespace go2_navigation
{

inline bool same_navigation_goal(
  const geometry_msgs::msg::PoseStamped & a, const geometry_msgs::msg::PoseStamped & b,
  double distance_tolerance = 0.05)
{
  return a.header.frame_id == b.header.frame_id &&
         std::hypot(a.pose.position.x - b.pose.position.x,
    a.pose.position.y - b.pose.position.y) < distance_tolerance &&
         std::abs(std::remainder(tf2::getYaw(a.pose.orientation) -
    tf2::getYaw(b.pose.orientation), 2.0 * 3.14159265358979323846)) < 0.15;
}

class GoalCapture
{
public:
  void reset() {captured_ = false;}
  bool update(double distance, double tolerance, double hysteresis)
  {
    if (!std::isfinite(distance)) {captured_ = false;}
    else if (distance <= tolerance) {captured_ = true;}
    else if (distance > tolerance + hysteresis) {captured_ = false;}
    return captured_;
  }

private:
  bool captured_{false};
};

// Global path points are spatial references, not sensor samples. Transform
// them with the latest available frame correction, checking its age instead
// of asking for a future sensor transform at clock->now().
inline geometry_msgs::msg::PoseStamped latest_path_pose(
  tf2_ros::Buffer & buffer, const geometry_msgs::msg::PoseStamped & point,
  const std::string & frame, const rclcpp::Time & now, double max_age)
{
  if (point.header.frame_id == frame) {
    return point;
  }
  const auto transform = buffer.lookupTransform(
    frame, point.header.frame_id, tf2::TimePointZero);
  const rclcpp::Time stamp(transform.header.stamp, now.get_clock_type());
  // Zero-stamped static transforms have no expiry.
  if (stamp.nanoseconds() != 0 &&
    ((now - stamp).seconds() > max_age || (stamp - now).seconds() > max_age))
  {
    throw std::runtime_error("Path frame transform is stale");
  }
  geometry_msgs::msg::PoseStamped result;
  tf2::doTransform(point, result, transform);
  return result;
}

class RearTurnDirection
{
public:
  void reset() {direction_ = 0.0;}
  void select(double error) {direction_ = std::copysign(1.0,error);}

  double error(double shortest_error)
  {
    if (direction_ == 0.0) {
      direction_ = std::copysign(1.0, shortest_error);
    }
    // Near exactly 180 degrees, a tiny pose fluctuation flips atan2 between
    // +pi and -pi. Keep the selected direction across that wrap boundary.
    constexpr double pi = 3.14159265358979323846;
    if (std::abs(shortest_error) > 2.8 && shortest_error * direction_ < 0.0) {
      return direction_ * (2.0 * pi - std::abs(shortest_error));
    }
    return shortest_error;
  }

private:
  double direction_{0.0};
};

inline constexpr double full_turn = 2.0 * 3.14159265358979323846;

inline double rotation_in_direction(double shortest_error, double direction)
{
  return shortest_error * direction < 0.0 ?
         direction * (full_turn - std::abs(shortest_error)) : shortest_error;
}

// Retain a checked direction over small path updates and check progress using
// observed yaw. A direction selected in software alone is not progress.
class PathAlignmentTurn
{
public:
  void reset() {direction_ = 0.0;}
  bool active() const {return direction_ != 0.0;}
  bool alternative() const {return active() && alternative_;}
  bool escape() const {return alternative() && escape_;}
  double direction() const {return direction_;}

  void start(double heading, double yaw, double now, double error, bool alternate = false)
  {
    direction_ = std::copysign(1.0, error);
    heading_ = heading;
    progress_yaw_ = yaw;
    progress_at_ = last_update_ = now;
    alternative_ = alternate;
    escape_ = alternate && std::abs(error) <= escape_angle;
    escape_heading_ = yaw + error;
  }

  void update(double heading, double yaw, double now)
  {
    if (!active()) {return;}
    if (now < last_update_ || now - last_update_ > 0.75 ||
      std::abs(std::remainder(heading - heading_, full_turn)) > 0.50)
    {
      reset();
      return;
    }
    last_update_ = now;
    if (direction_ * std::remainder(yaw - progress_yaw_, full_turn) >= 0.08) {
      progress_yaw_ = yaw;
      progress_at_ = now;
    }
  }

  bool stalled() const {return active() && last_update_ - progress_at_ >= 1.25;}

  double remaining(double shortest_error, double yaw) const
  {
    if (escape()) {return std::remainder(escape_heading_ - yaw, full_turn);}
    if (alternative() || (active() && std::abs(shortest_error) > 2.8)) {
      return rotation_in_direction(shortest_error, direction_);
    }
    return shortest_error;
  }

  static double other_side(double shortest_error, double preferred_error)
  {
    const auto direction = -std::copysign(1., preferred_error);
    const auto alternate = rotation_in_direction(shortest_error, direction);
    // For a forward-side target, try a bounded turn away and then reconsider
    // a translating arc. A nearly full revolution is a poor detour here.
    return std::abs(alternate) > 0.75 * full_turn ? direction * escape_angle : alternate;
  }

private:
  static constexpr double escape_angle = 0.60;
  double direction_{0.0}, heading_{0.0}, progress_yaw_{0.0};
  double progress_at_{0.0}, last_update_{0.0}, escape_heading_{0.0};
  bool alternative_{false}, escape_{false};
};

inline double rotation_velocity(
  double error, double tolerance, double current,
  double maximum, double minimum, double gain, double acceleration, double dt)
{
  if (std::abs(error) <= tolerance) {
    return 0.0;
  }
  const double cruise = std::clamp(
    gain * (std::abs(error) - tolerance), std::min(minimum, maximum), maximum);
  // The minimum is a cruising gait threshold, not a reason to hold full yaw
  // until the handover angle and overshoot through the downstream smoother.
  // Permit a transient braking ramp below it, just as during startup.
  const double remaining = std::max(0.0, std::abs(error) - tolerance -
    std::max(0.0, current * std::copysign(1.0, error)) * dt);
  const double magnitude = std::min(cruise, std::sqrt(2.0 * acceleration * remaining));
  const double desired = std::copysign(magnitude, error);
  const double delta = acceleration * dt;
  return std::clamp(desired, current - delta, current + delta);
}

inline double approach_speed_limit(
  double distance, double position_tolerance, double approach_distance,
  double near_goal_speed, double commanded_speed)
{
  if (commanded_speed <= near_goal_speed || distance >= approach_distance) {
    return commanded_speed;
  }
  const double span = std::max(1.e-6, approach_distance - position_tolerance);
  const double progress = std::clamp((distance-position_tolerance) / span, 0.0, 1.0);
  return near_goal_speed + progress * (commanded_speed-near_goal_speed);
}

// A forward ray that already enters the inner 80% of the accepted position
// disk needs no yaw correction toward its exact centre. Every resulting arc
// still requires the controller's full-footprint collision check.
inline double position_approach_error(double bearing_error, double distance, double tolerance)
{
  if (distance <= tolerance) {
    return 0.0;
  }
  const double angle_margin = std::asin(std::clamp(.8 * tolerance / distance, 0.0, 1.0));
  return std::copysign(std::max(0.0, std::abs(bearing_error) - angle_margin), bearing_error);
}

// If the final arc misses the accepted disk, a requested correction smaller
// than the ordinary path-alignment tolerance must still make progress.
inline double position_alignment_tolerance(double error, double path_tolerance)
{
  return std::min(path_tolerance, 0.5 * std::abs(error));
}

// Match curvature to remaining distance. A fixed yaw gain can drive past a
// nearby off-axis target and leave no room for the subsequent in-place turn.
inline double position_approach_yaw_rate(
  double bearing_error, double distance, double tolerance, double speed, double maximum)
{
  const double error = position_approach_error(bearing_error, distance, tolerance);
  return std::clamp(2.0 * speed * std::sin(error) / std::max(distance, tolerance),
           -maximum, maximum);
}

inline double approach_arc_miss(double bearing, double distance, double speed, double yaw_rate)
{
  const double x = distance * std::cos(bearing), y = distance * std::sin(bearing);
  if (std::abs(yaw_rate) < 1.e-6) {return std::abs(y);}
  const double radius = speed / yaw_rate;
  return std::abs(std::hypot(x, y-radius) - std::abs(radius));
}

inline std::vector<double> rotation_sweep(double error, double velocity, double horizon)
{
  const double travel = std::min(std::abs(error), std::abs(velocity) * horizon);
  if (travel == 0.0) {
    return {0.0};
  }
  const int steps = std::max(1, static_cast<int>(std::ceil(travel / 0.02)));
  std::vector<double> angles;
  angles.reserve(steps + 1);
  for (int i = 0; i <= steps; ++i) {
    angles.push_back(std::copysign(travel * i / steps, velocity));
  }
  return angles;
}

}  // namespace go2_navigation
#endif  // GO2_NAVIGATION__ROTATION_HELPERS_HPP_
