// Copyright 2026 Go2 Navigation Contributors
// SPDX-License-Identifier: Apache-2.0

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace go2_navigation
{

// A navigation goal is a spatial reference in map, not an observation at the
// instant it was clicked. LIO anchors its selected point once in the
// continuous frame; ordinary map navigation uses the latest transform. Both
// avoid expiring a spatial goal with TF history. Keep the original input
// intact so a new user click is detected independently of replanning.
class LatestGoalPose : public BT::SyncActionNode
{
public:
  LatestGoalPose(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config),
    tf_(config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer")) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("input"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output"),
      BT::InputPort<std::string>("anchor_frame", "", "Hold clicked point in this continuous frame")};
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped goal;
    if (!getInput("input", goal) || goal.header.frame_id.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    std::string anchor;
    getInput("anchor_frame", anchor);
    if (!anchor.empty()) {
      if (!have_goal_ || goal != original_) {
        try {
          // On first use, honour the click timestamp if present; later map
          // corrections cannot move the selected physical point mid-goal.
          anchored_ = goal.header.frame_id == anchor ? goal :
            tf_->transform(goal, anchor, tf2::durationFromSec(.2));
          anchored_.header.stamp = builtin_interfaces::msg::Time();
          original_ = goal;
          have_goal_ = true;
        } catch (const tf2::TransformException &) {
          return BT::NodeStatus::FAILURE;
        }
      }
      goal = anchored_;
    } else {
      goal.header.stamp = builtin_interfaces::msg::Time();
    }
    setOutput("output", goal);
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  geometry_msgs::msg::PoseStamped original_, anchored_;
  bool have_goal_{false};
};

}  // namespace go2_navigation

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<go2_navigation::LatestGoalPose>("LatestGoalPose");
}
