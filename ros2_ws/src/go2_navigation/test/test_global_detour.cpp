#include <cmath>
#include <memory>
#include "gtest/gtest.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"

class DetourCostmap : public nav2_costmap_2d::Costmap2DROS
{
public:
  DetourCostmap() : Costmap2DROS("detour_test_costmap")
  {
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    layered_costmap_->resizeMap(200, 200, .04, -4., -4.);
    footprint_padding_ = 0.;
    std::vector<geometry_msgs::msg::Point> footprint;
    for (const auto & xy : std::vector<std::pair<double,double>>{
        {.38,.175},{.38,-.175},{-.38,-.175},{-.38,.175}})
    {
      geometry_msgs::msg::Point p; p.x=xy.first; p.y=xy.second; footprint.push_back(p);
    }
    setRobotFootprint(footprint);
  }
};

TEST(GlobalDetour, RoutesAroundPersonAndReturnsToDirectPathAfterRemoval)
{
  if (!rclcpp::ok()) {rclcpp::init(0,nullptr);}
  auto node=std::make_shared<nav2_util::LifecycleNode>("global_detour_test");
  node->declare_parameter("inflation.inflation_radius",.18);
  node->declare_parameter("inflation.cost_scaling_factor",8.);
  auto map=std::make_shared<DetourCostmap>();
  auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto inflation=std::make_shared<nav2_costmap_2d::InflationLayer>();
  map->getLayeredCostmap()->addPlugin(inflation);
  inflation->initialize(map->getLayeredCostmap(),"inflation",tf.get(),node,nullptr);
  map->getLayeredCostmap()->setFootprint(map->getRobotFootprint());
  auto * grid=map->getCostmap();
  auto inflate=[&]() {
      double lx=-4.,ly=-4.,hx=4.,hy=4.;
      inflation->updateBounds(0.,0.,0.,&lx,&ly,&hx,&hy);
      inflation->updateCosts(*grid,0,0,200,200);
    };
  // A standing person blocks a 32 cm wide section of the direct route.
  for (unsigned int y=96;y<104;++y) {
    for (unsigned int x=98;x<102;++x) {grid->setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);}
  }
  inflate();
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core","nav2_core::GlobalPlanner");
  auto planner=loader.createSharedInstance("go2_navigation::ExactGoalSmac2D");
  planner->configure(node,"GridBased",tf,map); planner->activate();
  geometry_msgs::msg::PoseStamped start,goal;
  start.header.frame_id=goal.header.frame_id="map";
  start.pose.orientation.w=goal.pose.orientation.w=1.;
  start.pose.position.x=-1.; goal.pose.position.x=1.;
  auto path=planner->createPlan(start,goal);
  ASSERT_FALSE(path.poses.empty());
  double lateral=0.;
  for (const auto & p:path.poses) {
    lateral=std::max(lateral,std::abs(p.pose.position.y));
    unsigned int x,y; ASSERT_TRUE(grid->worldToMap(p.pose.position.x,p.pose.position.y,x,y));
    EXPECT_LT(grid->getCost(x,y),nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  }
  EXPECT_GT(lateral,.29);  // Person's half width plus physical half width.
  grid->resetMap(0,0,200,200); inflation->reset(); inflate();
  path=planner->createPlan(start,goal); ASSERT_FALSE(path.poses.empty());
  for (const auto & p:path.poses) {EXPECT_LT(std::abs(p.pose.position.y),.08);}
  // At the production 4 cm resolution, the cells leave 48 cm of clearance.
  // The physical width and 18 cm soft inflation must keep that corridor open.
  grid->resetMap(0,0,200,200);
  for (unsigned int x=0;x<200;++x) {
    grid->setCost(x,93,nav2_costmap_2d::LETHAL_OBSTACLE);
    grid->setCost(x,106,nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  inflation->reset(); inflate();
  path=planner->createPlan(start,goal); ASSERT_FALSE(path.poses.empty());
  for (const auto & p:path.poses) {EXPECT_LT(std::abs(p.pose.position.y),.10);}
  planner->deactivate(); planner->cleanup();
}
