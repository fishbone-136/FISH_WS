// Copyright 2026 Go2 Navigation Contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef GO2_NAVIGATION__EXECUTION_SAFETY_HPP_
#define GO2_NAVIGATION__EXECUTION_SAFETY_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "go2_navigation/motion_geometry.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace go2_navigation
{
// Use the same physical rectangle and constant-curvature arc as the final gate.
// Exact measured/remembered endpoints avoid adding a second costmap-cell radius.
inline bool execution_arc_clear(
  const std::vector<ObstaclePoint> & points, double vx, double vy, double wz,
  double half_length, double half_width, double horizon)
{
  const double speed = std::hypot(vx, vy);
  const double radius = std::hypot(half_length, half_width) + speed * horizon;
  std::vector<const ObstaclePoint *> nearby;
  for (const auto & p : points) {
    if (p.x * p.x + p.y * p.y <= radius * radius) {nearby.push_back(&p);}
  }
  if (nearby.empty()) {return true;}
  const int steps = std::max({1, static_cast<int>(std::ceil(horizon / .04)),
      static_cast<int>(std::ceil(std::abs(wz) * horizon / .025)),
      static_cast<int>(std::ceil(speed * horizon / .02))});
  for (int i = 0; i <= steps; ++i) {
    const double t = horizon * i / steps;
    const double c = std::cos(wz * t), s = std::sin(wz * t);
    const double dx = std::abs(wz) < 1.e-6 ? vx * t : (vx * s + vy * (c - 1.)) / wz;
    const double dy = std::abs(wz) < 1.e-6 ? vy * t : (vx * (1. - c) + vy * s) / wz;
    for (const auto * p : nearby) {
      const double x = p->x - dx, y = p->y - dy;
      if (std::abs(c * x + s * y) <= half_length + 1.e-12 &&
        std::abs(-s * x + c * y) <= half_width + 1.e-12) {return false;}
    }
  }
  return true;
}

struct ExecutionContext
{
  bool enabled{false}, ready{false};
  double half_length{0.0}, half_width{0.0}, scan_age{0.0};
  double response_time{.20}, approach_time{1.20};
  std::vector<ObstaclePoint> raw, memory, points;

  bool allows(double vx, double vy, double wz) const
  {
    if (!enabled) {return true;}
    if (!ready || !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
      return false;
    }
    const double speed = std::hypot(vx, vy);
    const double braking = scan_age + .25 + .5 * std::max(speed / .90, std::abs(wz) / 1.50);
    if (!execution_arc_clear(points, vx, vy, wz, half_length, half_width, braking)) {
      return false;
    }
    // The gate applies its 1.2 s contact fraction AFTER choosing a command.
    // Reject those commands before DWB scoring instead of requesting a gait
    // that will be reduced to a sustained, unusable 0.005--0.05 m/s afterwards.
    const double contact_horizon = scan_age + response_time + approach_time;
    if (vx > 1.e-6 && !execution_arc_clear(
        raw, vx, vy, wz, half_length, half_width, contact_horizon)) {return false;}
    if (speed > 1.e-6 && !execution_arc_clear(
        memory, vx, vy, wz, half_length, half_width, contact_horizon)) {return false;}
    return true;
  }
};

class ExecutionSafety
{
public:
  ExecutionSafety(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & controller,
    std::string base_frame)
  : clock_(node->get_clock()), base_frame_(std::move(base_frame))
  {
    const auto prefix = controller + ".execution_safety.";
    nav2_util::declare_parameter_if_not_declared(
      node, prefix + "enabled", rclcpp::ParameterValue(false));
    enabled_ = node->get_parameter(prefix + "enabled").as_bool();
    if (!enabled_) {return;}
    auto parameter = [&](const std::string & key, double value) {
        nav2_util::declare_parameter_if_not_declared(
          node, prefix + key, rclcpp::ParameterValue(value));
        return node->get_parameter(prefix + key).as_double();
      };
    const double front = parameter("body_front", 0.0), rear = parameter("body_rear", 0.0);
    half_length_ = std::max(front, rear);
    half_width_ = parameter("body_half_width", 0.0);
    minimum_ = parameter("minimum_translation", .22);
    nav2_util::declare_parameter_if_not_declared(
      node, prefix + "point_model", rclcpp::ParameterValue(false));
    const bool point_model = node->get_parameter(prefix + "point_model").as_bool();
    if (!std::isfinite(front) || !std::isfinite(rear) || !std::isfinite(half_width_) ||
      !std::isfinite(minimum_) || minimum_ <= 0.0 ||
      (point_model ? (front != 0.0 || rear != 0.0 || half_width_ != 0.0) :
      std::min({front, rear, half_width_}) <= 0.0))
    {throw std::runtime_error("Execution geometry must match the explicit point/rectangle model");}
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_safety", qos, [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        scan_points_.clear();
        scan_supported_ = m->header.frame_id == base_frame_;
        scan_stamp_ = rclcpp::Time(m->header.stamp).seconds();
        scan_received_ = steady();
        if (!scan_supported_) {return;}
        for (std::size_t i = 0; i < m->ranges.size(); ++i) {
          const double r = m->ranges[i], angle = m->angle_min + i * m->angle_increment;
          if (std::isfinite(r) && r >= m->range_min && r <= m->range_max) {
            scan_points_.push_back({r * std::cos(angle), r * std::sin(angle)});
          }
        }
      });
    memory_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/safety/remembered_obstacles", qos, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        memory_points_.clear(); memory_supported_ = false;
        memory_frame_ = m->header.frame_id;
        memory_stamp_ = rclcpp::Time(m->header.stamp).seconds();
        memory_received_ = steady();
        if (memory_frame_ == "unavailable" || m->height != 1 || m->point_step < 8 ||
          m->data.size() < static_cast<std::size_t>(m->width) * m->point_step) {return;}
        int ox = -1, oy = -1;
        for (const auto & f : m->fields) {
          if (f.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
            f.count != 1 || f.offset + 4 > m->point_step) {continue;}
          if (f.name == "x") {ox = f.offset;}
          if (f.name == "y") {oy = f.offset;}
        }
        if (ox < 0 || oy < 0) {return;}
        for (std::size_t i = 0; i < m->width; ++i) {
          const auto * data = m->data.data() + i * m->point_step;
          const double x = read_float(data + ox, m->is_bigendian);
          const double y = read_float(data + oy, m->is_bigendian);
          if (!std::isfinite(x) || !std::isfinite(y)) {return;}
          memory_points_.push_back({x, y});
        }
        memory_supported_ = true;
      });
  }

  bool enabled() const {return enabled_;}
  double minimum() const {return minimum_;}

  ExecutionContext snapshot(const std::string & frame, double x, double y, double yaw)
  {
    ExecutionContext result; result.enabled = enabled_;
    if (!enabled_) {return result;}
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = clock_->now().seconds(), receive_now = steady();
    const double scan_age = now - scan_stamp_, memory_age = now - memory_stamp_;
    result.ready = scan_supported_ && memory_supported_ && memory_frame_ == frame &&
      scan_age >= -.1 && scan_age <= .35 && receive_now - scan_received_ <= .35 &&
      memory_age >= -.05 && memory_age <= .75 && receive_now - memory_received_ <= .75;
    result.half_length = half_length_; result.half_width = half_width_;
    // Reserve one control/transport interval beyond the current scan age.
    result.scan_age = std::max(0.0, scan_age) + .10;
    if (!result.ready) {return result;}
    result.raw = scan_points_;
    const double c = std::cos(yaw), s = std::sin(yaw);
    for (const auto & p : memory_points_) {
      const double dx = p.x - x, dy = p.y - y;
      result.memory.push_back({c * dx + s * dy, -s * dx + c * dy});
    }
    result.points = result.raw;
    result.points.insert(result.points.end(), result.memory.begin(), result.memory.end());
    return result;
  }

private:
  static double steady()
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  static float read_float(const uint8_t * bytes, bool big_endian)
  {
    uint32_t bits = 0;
    for (int i = 0; i < 4; ++i) {bits |= uint32_t(bytes[big_endian ? 3 - i : i]) << (8 * i);}
    float result; std::memcpy(&result, &bits, sizeof(result)); return result;
  }
  rclcpp::Clock::SharedPtr clock_;
  bool enabled_{false}, scan_supported_{false}, memory_supported_{false};
  double half_length_{0.0}, half_width_{0.0}, minimum_{.22};
  double scan_stamp_{-1.e30}, memory_stamp_{-1.e30};
  double scan_received_{-1.e30}, memory_received_{-1.e30};
  std::string base_frame_, memory_frame_;
  std::mutex mutex_;
  std::vector<ObstaclePoint> scan_points_, memory_points_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr memory_sub_;
};
}  // namespace go2_navigation
#endif  // GO2_NAVIGATION__EXECUTION_SAFETY_HPP_
