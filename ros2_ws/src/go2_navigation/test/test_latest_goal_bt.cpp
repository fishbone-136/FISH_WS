#include "gtest/gtest.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

TEST(LatestGoalPose, ClickTimeAnchorSurvivesMapCorrectionsAndNewGoalReanchors)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<rclcpp::Node>("goal_anchor_test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);  // The real navigator services TF on a separate thread.
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom_lio";
  transform.child_frame_id = "map";
  transform.transform.rotation.w = 1.0;
  transform.header.stamp.sec = 10;
  transform.transform.translation.x = 1.0;
  ASSERT_TRUE(tf->setTransform(transform, "test"));
  auto board = BT::Blackboard::create();
  board->set("tf_buffer", tf);
  geometry_msgs::msg::PoseStamped input;
  input.header.frame_id = "map";
  input.header.stamp.sec = 10;
  input.pose.position.x = 5.0;
  input.pose.orientation.w = 1.0;
  board->set("goal", input);
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(LATEST_GOAL_PLUGIN);
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
    "<LatestGoalPose input='{goal}' output='{plan_goal}' anchor_frame='odom_lio'/>"
    "</BehaviorTree></root>", board);
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(board->get<geometry_msgs::msg::PoseStamped>("plan_goal").pose.position.x, 6.0);
  transform.header.stamp.sec = 11;
  transform.transform.translation.x = 3.0;
  ASSERT_TRUE(tf->setTransform(transform, "test"));
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(board->get<geometry_msgs::msg::PoseStamped>("plan_goal").pose.position.x, 6.0);
  input.header.stamp.sec = 11;
  board->set("goal", input);
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  const auto anchored = board->get<geometry_msgs::msg::PoseStamped>("plan_goal");
  EXPECT_DOUBLE_EQ(anchored.pose.position.x, 8.0);
  EXPECT_EQ(anchored.header.frame_id, "odom_lio");
  EXPECT_EQ(anchored.header.stamp.sec, 0);
}
