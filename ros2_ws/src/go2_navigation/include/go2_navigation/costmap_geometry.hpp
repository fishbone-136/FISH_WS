// Copyright 2026 Go2 Navigation Contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef GO2_NAVIGATION__COSTMAP_GEOMETRY_HPP_
#define GO2_NAVIGATION__COSTMAP_GEOMETRY_HPP_

#include <stdexcept>
#include <utility>
#include "go2_navigation/motion_geometry.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"

namespace go2_navigation
{
// Caller holds the costmap mutex. Only occupied cells (and tracked unknowns)
// count as physical obstacles; high soft inflation costs do not close a door.
inline std::vector<ObstaclePoint> local_obstacles(
  nav2_costmap_2d::Costmap2D & map, double x, double y, double yaw,
  bool track_unknown, double radius = 2.25)
{
  std::vector<ObstaclePoint> points;
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double cell_radius = map.getResolution() * std::sqrt(0.5);
  int min_x, min_y, max_x, max_y;
  map.worldToMapEnforceBounds(x - radius, y - radius, min_x, min_y);
  map.worldToMapEnforceBounds(x + radius, y + radius, max_x, max_y);
  for (int iy = min_y; iy <= max_y; ++iy) {
    for (int ix = min_x; ix <= max_x; ++ix) {
      const auto cost = map.getCost(ix, iy);
      if (cost != nav2_costmap_2d::LETHAL_OBSTACLE &&
        !(track_unknown && cost == nav2_costmap_2d::NO_INFORMATION)) {continue;}
      double wx, wy; map.mapToWorld(ix, iy, wx, wy);
      const double dx = wx - x, dy = wy - y;
      if (dx * dx + dy * dy > radius * radius) {continue;}
      points.push_back({c * dx + s * dy, -s * dx + c * dy, cell_radius});
    }
  }
  return points;
}

inline std::pair<double, double> footprint_half_extents(
  const std::vector<geometry_msgs::msg::Point> & footprint)
{
  if (footprint.size() < 3) {
    throw std::runtime_error("Motion constraints require a configured footprint");
  }
  double length = 0.0, width = 0.0;
  for (const auto & p : footprint) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      throw std::runtime_error("Footprint contains non-finite coordinates");
    }
    length = std::max(length, std::abs(p.x)); width = std::max(width, std::abs(p.y));
  }
  // Four coincident origin vertices represent a point in the experimental
  // planner. The final scan gate uses its separate physical body dimensions.
  if (length == 0.0 && width == 0.0) {return {0.0, 0.0};}
  if (!(length > 0.0 && width > 0.0)) {
    throw std::runtime_error("Motion constraints require a configured footprint");
  }
  return {length, width};
}
}  // namespace go2_navigation
#endif
