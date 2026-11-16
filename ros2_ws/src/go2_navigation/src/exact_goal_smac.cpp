// Copyright 2026 Go2 Navigation Contributors
// SPDX-License-Identifier: Apache-2.0

#include <memory>
#include <mutex>
#include <string>

#include "nav2_smac_planner/smac_planner_2d.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace go2_navigation
{

// Humble Smac2D ends at a grid-cell centre, including returning the start pose
// when both poses share a cell. At 8 cm resolution that can exceed a 5 cm goal
// radius. Keep its checked route and finish at the requested point in that
// same checked goal cell; never substitute a neighbouring tolerated cell.
class ExactGoalSmac2D : public nav2_smac_planner::SmacPlanner2D
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    costmap_ros_ = costmap;
    SmacPlanner2D::configure(parent, name, tf, costmap);
  }

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    auto path = SmacPlanner2D::createPlan(start, goal);
    if (path.poses.empty()) {return path;}
    auto * map = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    unsigned int gx, gy, ex, ey;
    const auto & end = path.poses.back().pose.position;
    if (!map->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy) ||
      !map->worldToMap(end.x, end.y, ex, ey) || gx != ex || gy != ey)
    {
      path.poses.clear();
      return path;
    }
    const auto cost = map->getCost(gx, gy);
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
      cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      path.poses.clear();
      return path;
    }
    auto exact = goal;
    exact.header = path.header;
    path.poses.push_back(exact);
    return path;
  }

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};

}  // namespace go2_navigation

PLUGINLIB_EXPORT_CLASS(go2_navigation::ExactGoalSmac2D, nav2_core::GlobalPlanner)
