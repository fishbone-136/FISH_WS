// Copyright 2026 Go2 Navigation Contributors
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include "gtest/gtest.h"
#include "go2_navigation/costmap_geometry.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

using go2_navigation::ObstaclePoint;

namespace
{
std::vector<ObstaclePoint> walls(double width, double heading = 0.0, double start = 0.0)
{
  std::vector<ObstaclePoint> points;
  for (double x = start; x < 1.6; x += 0.04) {
    for (double y : {-width / 2.0, width / 2.0}) {
      points.push_back({std::cos(heading) * x - std::sin(heading) * y,
          std::sin(heading) * x + std::cos(heading) * y, 0.0});
    }
  }
  return points;
}
}

TEST(MotionGeometry, PointFootprintChecksTheCentreWithoutAnImplicitBodyOrPadding)
{
  std::vector<geometry_msgs::msg::Point> point(4);
  const auto extents = go2_navigation::footprint_half_extents(point);
  EXPECT_DOUBLE_EQ(extents.first, 0.0);
  EXPECT_DOUBLE_EQ(extents.second, 0.0);
  nav2_costmap_2d::Costmap2D map(100, 100, 0.04, -2.0, -2.0, 0);
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(&map);
  // A physical-body corner obstacle must not enlarge the point planning model.
  map.setCost(59, 54, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_DOUBLE_EQ(checker.footprintCostAtPose(0., 0., 1.2, point), 0.);
  map.setCost(50, 50, 100);
  EXPECT_DOUBLE_EQ(checker.footprintCostAtPose(0., 0., 1.2, point), 100.);
  map.setCost(50, 50, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_GE(checker.footprintCostAtPose(0., 0., 1.2, point), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(go2_navigation::footprint_half_extents({}), std::runtime_error);
  point[0].x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(go2_navigation::footprint_half_extents(point), std::runtime_error);
}

TEST(MotionGeometry, CloseWallsPreserveAStableStraightGaitAndBoundYawBeforeScaling)
{
  const auto limits = go2_navigation::clearance_limits(walls(0.70), 0.45, 0.27);
  EXPECT_GE(limits.linear, 0.22);
  EXPECT_DOUBLE_EQ(limits.linear, 0.26);
  EXPECT_DOUBLE_EQ(limits.angular, 0.65);
  // The previously accepted command would fall below gait after yaw scaling.
  EXPECT_LT(0.26 * limits.angular / 1.0, 0.22);
  const auto open = go2_navigation::clearance_limits({}, 0.45, 0.27);
  EXPECT_DOUBLE_EQ(open.linear, 0.65);
  EXPECT_DOUBLE_EQ(open.angular, 1.4);
}

TEST(MotionGeometry, SeventyCentimetreCorridorFitsAlignedButNotAtTenDegrees)
{
  const auto corridor = go2_navigation::detect_corridor(walls(0.70), 0.0);
  ASSERT_TRUE(corridor.detected);
  EXPECT_NEAR(corridor.width(), 0.70, 1.e-9);
  const auto tolerance = go2_navigation::corridor_heading_tolerance(corridor, 0.45, 0.27);
  EXPECT_GT(tolerance, 0.05);
  EXPECT_LT(tolerance, 10.0 * M_PI / 180.0);
  EXPECT_NEAR(0.54 * std::cos(tolerance) + 0.90 * std::sin(tolerance), 0.66, 1.e-8);
}

TEST(MotionGeometry, FindsAnEntranceAheadInRotatedCoordinates)
{
  const auto corridor = go2_navigation::detect_corridor(walls(0.72, 0.2, 0.65), 0.2);
  ASSERT_TRUE(corridor.detected);
  EXPECT_GE(corridor.start, 0.4);
  EXPECT_NEAR(corridor.width(), 0.72, 1.e-8);
  EXPECT_GE(corridor.end - corridor.start, 0.60 - 1.e-9);
}

TEST(MotionGeometry, RequiresTwoSupportedWallsAndRejectsDivergence)
{
  auto points = walls(0.70);
  points.erase(std::remove_if(points.begin(), points.end(),
      [](const auto & p) {return p.y < 0.0;}), points.end());
  EXPECT_FALSE(go2_navigation::detect_corridor(points, 0.0).detected);
  EXPECT_FALSE(go2_navigation::detect_corridor({{0.4, 0.35}, {0.4, -0.35}}, 0.0).detected);
  EXPECT_FALSE(go2_navigation::detect_corridor(walls(2.0), 0.0).detected);
  for (auto & p : points) {p.y += p.x;}
  for (double x = 0.0; x < 1.6; x += 0.04) {points.push_back({x, -0.35 - x});}
  EXPECT_FALSE(go2_navigation::detect_corridor(points, 0.0).detected);
}

TEST(MotionGeometry, LateralOffsetAndInsufficientWidthCannotCreateFalseHeadingRoom)
{
  auto corridor = go2_navigation::detect_corridor(walls(0.70), 0.0);
  const auto centered = go2_navigation::corridor_heading_tolerance(corridor, 0.45, 0.27);
  corridor.left -= 0.04; corridor.right -= 0.04;
  EXPECT_LT(go2_navigation::corridor_heading_tolerance(corridor, 0.45, 0.27), centered);
  corridor = go2_navigation::detect_corridor(walls(0.54), 0.0);
  EXPECT_DOUBLE_EQ(go2_navigation::corridor_heading_tolerance(corridor, 0.45, 0.27), 0.0);
}

TEST(MotionGeometry, SoftInflationDoesNotCountAsASecondPhysicalWall)
{
  nav2_costmap_2d::Costmap2D map(100, 100, 0.04, -2.0, -2.0, 0);
  // A cost of 252 is a strong preference, not a lethal cell.
  for (unsigned int x = 10; x < 90; ++x) {map.setCost(x, 58, 252); map.setCost(x, 41, 252);}
  EXPECT_TRUE(go2_navigation::local_obstacles(map, 0.0, 0.0, 0.0, false).empty());
  for (unsigned int x = 50; x < 90; ++x) {map.setCost(x, 58, 254); map.setCost(x, 41, 254);}
  const auto points = go2_navigation::local_obstacles(map, 0.0, 0.0, 0.0, false);
  ASSERT_FALSE(points.empty());
  const auto corridor = go2_navigation::detect_corridor(points, 0.0);
  ASSERT_TRUE(corridor.detected);
  EXPECT_GT(corridor.width(), 0.54);
  map.setCost(50, 50, 255);
  EXPECT_EQ(go2_navigation::local_obstacles(map, 0.0, 0.0, 0.0, true).size(), points.size() + 1);
}

TEST(MotionGeometry, ArcHandoverSurvivesReplansAndReleasesOnChangedRouteOrAlignment)
{
  go2_navigation::AlignmentHandoff handoff;
  handoff.blocked(1.2);
  for (int i = 0; i < 20; ++i) {EXPECT_TRUE(handoff.tracking(1.21, 1.0, 0.28));}
  EXPECT_FALSE(handoff.tracking(1.21, 0.2, 0.28));
  handoff.blocked(1.2);
  EXPECT_FALSE(handoff.tracking(1.8, 1.0, 0.28));
  handoff.blocked(1.2); handoff.reset();
  EXPECT_FALSE(handoff.tracking(1.2, 1.0, 0.28));
}

TEST(MotionGeometry, ContinuityPrefersAStableSideWithoutForbiddingTheOtherSide)
{
  const double same = go2_navigation::continuity_cost(0.26, 0.0, 0.58, 0.26, 0.0, 0.60);
  const double opposite = go2_navigation::continuity_cost(0.26, 0.0, -0.58, 0.26, 0.0, 0.60);
  EXPECT_LT(same, opposite);
  EXPECT_TRUE(std::isfinite(opposite));
}

TEST(MotionGeometry, AnImpassableDoorOnlyBlocksAPathThatActuallyEntersIt)
{
  const auto corridor = go2_navigation::detect_corridor(walls(0.50, 0.0, 0.6), 0.0);
  ASSERT_TRUE(corridor.detected);
  EXPECT_TRUE(go2_navigation::path_enters_corridor(corridor, {{0.0, 0.0}, {1.0, 0.0}}));
  EXPECT_FALSE(go2_navigation::path_enters_corridor(corridor, {{0.0, 0.0}, {0.3, 0.0}}));
  EXPECT_FALSE(go2_navigation::path_enters_corridor(corridor, {{0.0, 0.0}, {1.0, 0.8}}));
}
