// Copyright 2026 Go2 Navigation Contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef GO2_NAVIGATION__MOTION_GEOMETRY_HPP_
#define GO2_NAVIGATION__MOTION_GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace go2_navigation
{
struct ObstaclePoint
{
  double x, y, radius{0.0};
};

struct MotionLimits
{
  double linear{0.65}, angular{1.4};
};

// Same soft clearance policy as the final scan gate. Applying these bounds
// before DWB scoring keeps an angular cap from turning a valid walking sample
// into a sub-gait command downstream. Independent braking remains mandatory.
inline MotionLimits clearance_limits(
  const std::vector<ObstaclePoint> & points, double half_length, double half_width,
  bool pure_turn = false)
{
  double clearance = std::numeric_limits<double>::infinity();
  for (const auto & p : points) {
    if (pure_turn) {
      clearance = std::min(clearance,
        std::hypot(p.x, p.y) - std::hypot(half_length, half_width) - p.radius);
    } else if (p.x + p.radius >= -half_length && p.x - p.radius <= 1.0) {
      clearance = std::min(clearance, std::hypot(
          std::max(std::abs(p.x) - half_length, 0.0),
          std::max(std::abs(p.y) - half_width, 0.0)) - p.radius);
    }
  }
  const double blend = std::clamp(
    (clearance - (pure_turn ? 0.10 : 0.20)) / (pure_turn ? 0.22 : 0.15), 0.0, 1.0);
  return {0.26 + 0.39 * blend, 0.65 + 0.75 * blend};
}

struct Corridor
{
  bool detected{false};
  double heading{0.0};  // Direction in the robot's current body frame.
  double left{0.0}, right{0.0}, start{0.0}, end{0.0};
  double width() const {return left - right;}
  double center() const {return 0.5 * (left + right);}
};

// Require paired, approximately parallel walls over >= 60 cm. A single thin
// obstacle, one wall, or a divergent corner cannot create a corridor mode.
// This is a motion preference, never evidence that unobserved space is free.
inline Corridor detect_corridor(
  const std::vector<ObstaclePoint> & points, double heading,
  double maximum_width = 1.2, double probe_distance = 1.6)
{
  constexpr double bin_size = 0.2;
  const int count = static_cast<int>(std::floor(probe_distance / bin_size));
  if (count < 3 || !std::isfinite(heading) || maximum_width <= 0.0) {return {};}
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> left(count, inf), right(count, -inf);
  const double c = std::cos(heading), s = std::sin(heading);
  for (const auto & p : points) {
    const double x = c * p.x + s * p.y, y = -s * p.x + c * p.y;
    const int bin = static_cast<int>(std::floor(x / bin_size));
    if (bin < 0 || bin >= count) {continue;}
    if (y > 0.0) {left[bin] = std::min(left[bin], y - p.radius);}
    if (y < 0.0) {right[bin] = std::max(right[bin], y + p.radius);}
  }
  for (int first = 0; first + 2 < count; ++first) {
    double lo_left = inf, hi_left = -inf, lo_right = inf, hi_right = -inf;
    int last = first;
    for (; last < count; ++last) {
      const double width = left[last] - right[last];
      if (!std::isfinite(width) || width <= 0.0 || width > maximum_width) {break;}
      const double next_lo_left = std::min(lo_left, left[last]);
      const double next_hi_left = std::max(hi_left, left[last]);
      const double next_lo_right = std::min(lo_right, right[last]);
      const double next_hi_right = std::max(hi_right, right[last]);
      if (next_hi_left - next_lo_left > 0.10 || next_hi_right - next_lo_right > 0.10) {break;}
      lo_left = next_lo_left; hi_left = next_hi_left;
      lo_right = next_lo_right; hi_right = next_hi_right;
    }
    if (last - first >= 3) {
      return {true, heading, lo_left, hi_right, first * bin_size, last * bin_size};
    }
  }
  return {};
}

// Maximum small heading error that fits a long rectangular body in a straight
// corridor. Returns zero when the current lateral position has no margin.
inline double corridor_heading_tolerance(
  const Corridor & corridor, double half_length, double half_width, double margin = 0.02)
{
  const double available = std::min(corridor.left, -corridor.right) - margin;
  if (!corridor.detected || available <= half_width) {return 0.0;}
  const double peak_angle = std::atan2(half_length, half_width);
  if (available >= std::hypot(half_length, half_width)) {return peak_angle;}
  double low = 0.0, high = peak_angle;
  for (int i = 0; i < 32; ++i) {
    const double angle = 0.5 * (low + high);
    if (half_width * std::cos(angle) + half_length * std::sin(angle) <= available) {
      low = angle;
    } else {high = angle;}
  }
  return low;
}

// The sampled global path is quantized. Use its tangent as the reference and
// look for longer, more parallel wall support within a small heading range.
inline Corridor detect_path_corridor(
  const std::vector<ObstaclePoint> & points, double heading, double maximum_width)
{
  auto best = detect_corridor(points, heading, maximum_width);
  for (const double offset : {-0.10, 0.10, -0.20, 0.20}) {
    const auto candidate = detect_corridor(points, heading + offset, maximum_width);
    if (!candidate.detected) {continue;}
    if (!best.detected || candidate.end - candidate.start > best.end - best.start + 0.05 ||
      (std::abs((candidate.end - candidate.start) - (best.end - best.start)) < 0.05 &&
      candidate.width() > best.width() + 0.01))
    {
      best = candidate;
    }
  }
  return best;
}

inline double corridor_center_yaw(const Corridor & corridor, double speed, double lookahead)
{
  const double center = corridor.center();
  const double y = std::sin(corridor.heading) * lookahead + std::cos(corridor.heading) * center;
  return std::clamp(2.0 * speed * y / (lookahead * lookahead + center * center), -0.30, 0.30);
}

inline bool path_enters_corridor(
  const Corridor & corridor, const std::vector<ObstaclePoint> & path)
{
  if (!corridor.detected) {return false;}
  const double c = std::cos(corridor.heading), s = std::sin(corridor.heading);
  for (const auto & p : path) {
    const double x = c * p.x + s * p.y, y = -s * p.x + c * p.y;
    if (x >= corridor.start + 0.1 && x <= corridor.end - 0.1 &&
      y > corridor.right && y < corridor.left) {return true;}
  }
  return false;
}

// Soft command continuity: direction changes remain available when required
// for collision avoidance. Goal changes and stale history are handled by caller.
inline double continuity_cost(
  double vx, double vy, double wz, double previous_x, double previous_y, double previous_wz)
{
  const double translation = std::hypot(vx - previous_x, vy - previous_y) / 0.65;
  const double yaw = std::abs(wz - previous_wz) / 1.4;
  const double reversal = wz * previous_wz < 0.0 &&
    std::min(std::abs(wz), std::abs(previous_wz)) > 0.15 ? 1.0 : 0.0;
  return translation + yaw + reversal;
}

// A quadruped needs time to establish yaw motion. Do not alternate the only
// two remaining pure-turn candidates every few controller cycles. This is a
// bounded preference: a blocked committed turn releases immediately, and
// translation/new goals are never constrained by it.
class PureTurnCommitment
{
public:
  void reset() {direction_ = 0.; until_ = 0.;}
  double direction() const {return direction_;}
  bool permits(double vx, double vy, double wz, double now, bool can_continue) const
  {
    return std::hypot(vx, vy) > 1.e-5 || std::abs(wz) < 1.e-5 ||
           direction_ == 0. || wz * direction_ >= 0. || !can_continue || now >= until_;
  }
  void record(double vx, double vy, double wz, double now, double seconds)
  {
    if (std::hypot(vx, vy) > 1.e-5 || std::abs(wz) < 1.e-5 || seconds <= 0.) {
      reset(); return;
    }
    if (direction_ == 0. || wz * direction_ < 0.) {
      direction_ = std::copysign(1., wz); until_ = now + seconds;
    }
  }

private:
  double direction_{0.}, until_{0.};
};

class AlignmentHandoff
{
public:
  void reset() {blocked_ = false;}
  void blocked(double path_heading) {blocked_ = true; heading_ = path_heading;}
  bool tracking(double path_heading, double error, double tolerance)
  {
    if (std::abs(error) <= tolerance ||
      std::abs(std::remainder(path_heading - heading_, 2.0 * 3.14159265358979323846)) > 0.5)
    {
      blocked_ = false;
    }
    return blocked_;
  }
private:
  bool blocked_{false};
  double heading_{0.0};
};
}  // namespace go2_navigation
#endif
