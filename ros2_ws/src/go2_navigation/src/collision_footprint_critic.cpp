// Copyright 2026 Go2 Navigation Contributors
// SPDX-License-Identifier: Apache-2.0
#include "dwb_critics/obstacle_footprint.hpp"
#include "go2_navigation/costmap_geometry.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace go2_navigation
{
// Preserve the original corner-clearance preference in ordinary obstacle
// detours. Only a supported two-wall corridor can suppress duplicate soft
// inflation scores; hard footprint collision exceptions always propagate.
class CollisionFootprintCritic : public dwb_critics::ObstacleFootprintCritic
{
public:
  void onInit() override
  {
    dwb_critics::ObstacleFootprintCritic::onInit();
    auto node = node_.lock();
    const auto key = dwb_plugin_name_ + ".corridor_max_width";
    nav2_util::declare_parameter_if_not_declared(node, key, rclcpp::ParameterValue(1.2));
    maximum_width_ = node->get_parameter(key).as_double();
  }

  bool prepare(const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & velocity, const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & path) override
  {
    const bool ready = dwb_critics::ObstacleFootprintCritic::prepare(pose, velocity, goal, path);
    narrow_ = false;
    if (!ready || path.poses.size() < 2) {return ready;}
    const auto & start = path.poses.front();
    auto sample = path.poses.back();
    for (const auto & p : path.poses) {
      if (std::hypot(p.x - start.x, p.y - start.y) >= 0.65) {sample = p; break;}
    }
    const double heading = std::atan2(sample.y - start.y, sample.x - start.x);
    const auto footprint = footprint_half_extents(costmap_ros_->getRobotFootprint());
    const auto points = local_obstacles(*costmap_, pose.x, pose.y, pose.theta,
      costmap_ros_->getLayeredCostmap()->isTrackingUnknown());
    const auto corridor = detect_corridor(points, heading - pose.theta, maximum_width_);
    std::vector<ObstaclePoint> local_path;
    const double c = std::cos(pose.theta), s = std::sin(pose.theta);
    for (const auto & p : path.poses) {
      const double dx = p.x - pose.x, dy = p.y - pose.y;
      local_path.push_back({c * dx + s * dy, -s * dx + c * dy});
    }
    narrow_ = corridor.detected && corridor.width() >= 2.0 * std::min(footprint.first, footprint.second) &&
      path_enters_corridor(corridor, local_path);
    return true;
  }

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    const double original = dwb_critics::ObstacleFootprintCritic::scoreTrajectory(trajectory);
    return narrow_ ? 0.0 : original;
  }

private:
  bool narrow_{false};
  double maximum_width_{1.2};
};
}  // namespace go2_navigation
PLUGINLIB_EXPORT_CLASS(go2_navigation::CollisionFootprintCritic, dwb_core::TrajectoryCritic)
