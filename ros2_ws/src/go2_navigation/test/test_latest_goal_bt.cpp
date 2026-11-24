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

// Exercise the production XML with Nav2's real retry/pipeline controls. Only
// action servers and time are simulated; no ROS actions or robot commands.
namespace
{
struct ObstacleWaitState
{
  bool occupied{true}, finished{false}, new_goal{false};
  int plans{0}, waits{0}, follows{0}, halted_waits{0};
};
class SimulatedWait : public BT::StatefulActionNode
{
public:
  SimulatedWait(const std::string & name, const BT::NodeConfiguration & config,
    ObstacleWaitState & state) : StatefulActionNode(name, config), state_(state) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<int>("wait_duration")};}
  BT::NodeStatus onStart() override
  {
    EXPECT_EQ(getInput<int>("wait_duration").value(), 1);
    ++state_.waits; return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {return BT::NodeStatus::SUCCESS;}
  void onHalted() override {++state_.halted_waits;}
private:
  ObstacleWaitState & state_;
};
class SimulatedFollow : public BT::StatefulActionNode
{
public:
  SimulatedFollow(const std::string & name, const BT::NodeConfiguration & config,
    ObstacleWaitState & state) : StatefulActionNode(name, config), state_(state) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("server_timeout"), BT::InputPort<std::string>("path"),
      BT::InputPort<std::string>("controller_id"), BT::InputPort<std::string>("goal_checker_id")};
  }
  BT::NodeStatus onStart() override {++state_.follows; return onRunning();}
  BT::NodeStatus onRunning() override
  {return state_.finished ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;}
  void onHalted() override {}
private:
  ObstacleWaitState & state_;
};
class SimulatedRate : public BT::DecoratorNode
{
public:
  using BT::DecoratorNode::DecoratorNode;
  static BT::PortsList providedPorts() {return {BT::InputPort<double>("hz")};}
  BT::NodeStatus tick() override {return child_node_->executeTick();}
};
void registerObstacleWaitNodes(BT::BehaviorTreeFactory & factory, ObstacleWaitState & state)
{
  factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_recovery_node_bt_node.so");
  factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_pipeline_sequence_bt_node.so");
  factory.registerNodeType<SimulatedRate>("RateController");
  factory.registerBuilder<SimulatedWait>("Wait", [&state](
      const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<SimulatedWait>(name, config, state);
    });
  factory.registerBuilder<SimulatedFollow>("FollowPath", [&state](
      const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<SimulatedFollow>(name, config, state);
    });
  factory.registerSimpleAction("LatestGoalPose", [](BT::TreeNode &) {
      return BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<std::string>("input"), BT::OutputPort<std::string>("output")});
  factory.registerSimpleAction("ComputePathToPose", [&state](BT::TreeNode &) {
      ++state.plans;
      return state.occupied ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<std::string>("server_timeout"), BT::InputPort<std::string>("goal"),
      BT::OutputPort<std::string>("path"), BT::InputPort<std::string>("planner_id")});
  factory.registerSimpleCondition("GoalUpdated", [&state](BT::TreeNode &) {
      const bool updated = state.new_goal; state.new_goal = false;
      return updated ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
}
}  // namespace

TEST(ObstacleWaitRecovery, TemporaryObstacleResumesAndPersistentObstacleTerminates)
{
  for (const auto & filename : {"navigate_w_replanning_and_recovery.xml",
      "navigate_w_replanning_and_recovery_mppi.xml"})
  {
    SCOPED_TRACE(filename);
    ObstacleWaitState state;
    BT::BehaviorTreeFactory factory; registerObstacleWaitNodes(factory, state);
    auto tree = factory.createTreeFromFile(std::string(OBSTACLE_WAIT_BT_DIR) + "/" + filename);
    // A transient obstacle should keep the goal active without motion.
    for (int i = 0; i < 10; ++i) {EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);}
    EXPECT_GT(state.plans, 2); EXPECT_EQ(state.follows, 0);
    state.occupied = false;
    for (int i = 0; i < 3; ++i) {EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);}
    EXPECT_GT(state.follows, 0);
    state.finished = true;
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
    tree.haltTree(); state = ObstacleWaitState{};
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    for (int i = 0; i < 100 && status == BT::NodeStatus::RUNNING; ++i) {
      status = tree.tickRoot();
    }
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
    EXPECT_EQ(state.waits, 10); EXPECT_EQ(state.plans, 11); EXPECT_EQ(state.follows, 0);
    tree.haltTree(); state = ObstacleWaitState{};
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
    state.new_goal = true; state.occupied = false;
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
    EXPECT_GT(state.halted_waits, 0); EXPECT_GT(state.follows, 0);
    tree.haltTree();  // Cancel while executing remains immediate.
    EXPECT_EQ(tree.rootNode()->status(), BT::NodeStatus::IDLE);
    state = ObstacleWaitState{};
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
    tree.haltTree();  // Cancel during obstacle waiting halts the wait too.
    EXPECT_EQ(state.halted_waits, 1);
    state = ObstacleWaitState{}; state.occupied = false;
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
    EXPECT_EQ(state.follows, 1);
    state.occupied = true;  // A person blocks the goal after travel has started.
    for (int i = 0; i < 10; ++i) {EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);}
    EXPECT_GT(state.waits, 2);
    state.occupied = false;
    for (int i = 0; i < 3; ++i) {EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);}
    EXPECT_GT(state.follows, 1);
    state.finished = true;
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  }
}
