#include "gtest/gtest.h"
#include "pointcloud_to_laserscan/ground_plane.hpp"
#include "pointcloud_to_laserscan/scan_projection.hpp"

using pointcloud_to_laserscan::estimate_ground;
using pointcloud_to_laserscan::GroundPlane;
using pointcloud_to_laserscan::GroundTracker;

TEST(ScanProjection, FullPoseObstacleAndPlanarScanAgreeDuringBodyTilt)
{
  for (double yaw : {-.7,0.,1.2}) {
    const Eigen::Matrix3d rotation=(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(.22,Eigen::Vector3d::UnitY())*
      Eigen::AngleAxisd(-.15,Eigen::Vector3d::UnitX())).toRotationMatrix();
    const Eigen::Vector3d body(1.5,.3,.8);
    const auto scan=pointcloud_to_laserscan::planar_from_body(rotation)*body;
    const Eigen::Vector3d world_scan=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*scan;
    const Eigen::Vector3d world_memory=rotation*body;
    EXPECT_LT((world_scan.head<2>()-world_memory.head<2>()).norm(),1.e-12);
    const Eigen::Vector3d old_scan=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*body;
    EXPECT_GT((old_scan.head<2>()-world_memory.head<2>()).norm(),.10);
  }
}

TEST(GroundPlane, FindsFloorAtDifferentStandingHeightsAndRejectsLowBox)
{
  for (double height : {-0.37, -0.78}) {
    std::vector<Eigen::Vector3d> points;
    for (double x = 0.6; x < 3.0; x += 0.1) {
      for (double y = -1.0; y < 1.0; y += 0.1) {
        points.emplace_back(x, y, height + 0.015 * std::sin(7.0 * x + y));
      }
    }
    for (double y = -0.2; y < 0.2; y += 0.02) {
      for (double z = height + 0.10; z < height + 0.24; z += 0.02) {
        points.emplace_back(1.5, y, z);
      }
    }
    const auto plane = estimate_ground(points);
    ASSERT_TRUE(plane.valid);
    EXPECT_NEAR(plane.height(Eigen::Vector3d(1, 0, height)), 0.0, 0.03);
    EXPECT_GT(plane.height(Eigen::Vector3d(1.5, 0, height + 0.15)), 0.10);
  }
}

TEST(GroundPlane, HandlesTiltWithoutTreatingFloorAsWall)
{
  std::vector<Eigen::Vector3d> points;
  for (double x = 0.6; x < 3.0; x += 0.1) {
    for (double y = -1; y < 1; y += 0.1) {
      points.emplace_back(x, y, -0.70 + 0.1*x + 0.03*y);
    }
  }
  auto plane = estimate_ground(points);
  ASSERT_TRUE(plane.valid);
  EXPECT_NEAR(plane.height(Eigen::Vector3d(2, 0, -0.5)), 0.0, 0.01);
}

TEST(GroundPlane, RejectsVerticalWallSparseAndCollinearEvidence)
{
  std::vector<Eigen::Vector3d> points;
  EXPECT_FALSE(estimate_ground(points).valid);
  for (double y = -1; y < 1; y += 0.05) {
    for (double z = -0.8; z < -0.15; z += 0.03) {points.emplace_back(1, y, z);}
  }
  EXPECT_FALSE(estimate_ground(points).valid);
  points.clear();
  for (int i = 0; i < 100; ++i) {points.emplace_back(0.6 + i*0.01, 0, -0.5);}
  EXPECT_FALSE(estimate_ground(points).valid);
}

TEST(GroundPlane, DenseRaisedReturnsDoNotOutvoteVisibleFloor)
{
  std::vector<Eigen::Vector3d> points;
  for(double x=.6;x<3.;x+=.10) {
    for(double y=-1.;y<1.;y+=.10) {points.emplace_back(x,y,-.37);}
  }
  for(int repeat=0;repeat<8;++repeat) {
    for(double x=1.2;x<2.;x+=.025) {
      for(double y=-.4;y<.4;y+=.025) {points.emplace_back(x,y,-.15);}
    }
  }
  const auto plane=estimate_ground(points);
  ASSERT_TRUE(plane.valid);
  EXPECT_NEAR(plane.height(Eigen::Vector3d(1.,0.,-.37)),0.,.025);
  EXPECT_GT(plane.height(Eigen::Vector3d(1.5,0.,-.15)),.15);
}

TEST(GroundTracker, BootstrapsByConsensusAndRejectsSingleFrameHeightJumps)
{
  GroundTracker tracker;
  GroundPlane floor{{0.,0.,1.},.37,true},wrong{{0.,0.,1.},.13,true};
  EXPECT_FALSE(tracker.observe(wrong,10.).valid);
  EXPECT_FALSE(tracker.observe(floor,10.07).valid);
  EXPECT_FALSE(tracker.observe(floor,10.14).valid);
  EXPECT_TRUE(tracker.observe(floor,10.21).valid);
  EXPECT_TRUE(tracker.observation_supported(10.21));
  EXPECT_DOUBLE_EQ(tracker.observe(wrong,10.28).offset,.37);
  EXPECT_FALSE(tracker.observation_supported(10.28));
  EXPECT_DOUBLE_EQ(tracker.observe(floor,10.35).offset,.37);
}

TEST(GroundTracker, MissingGroundExpiresAndDuplicatesCannotBootstrap)
{
  GroundTracker tracker;
  GroundPlane floor{{0.,0.,1.},.37,true};
  for(int i=0;i<20;++i) {EXPECT_FALSE(tracker.observe(floor,10.).valid);}
  EXPECT_FALSE(tracker.observe(floor,10.07).valid);
  EXPECT_TRUE(tracker.observe(floor,10.14).valid);
  EXPECT_TRUE(tracker.observe(GroundPlane{},10.8).valid);
  EXPECT_FALSE(tracker.observe(GroundPlane{},11.3).valid);
}
TEST(ReferencedGround, BodyPitchAndHeightChangesDoNotInterruptObservedFloor)
{
  pointcloud_to_laserscan::ReferencedGroundTracker tracker;
  GroundPlane world{{0.,0.,1.},0.,true};
  for(int i=0;i<60;++i) {
    const double t=i/15.;
    Eigen::Matrix3d rotation=(Eigen::AngleAxisd(t*1.2,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(.10*std::sin(t*7.),Eigen::Vector3d::UnitY())*
      Eigen::AngleAxisd(.08*std::cos(t*5.),Eigen::Vector3d::UnitX())).toRotationMatrix();
    Eigen::Vector3d translation(.2*t,0.,.35+.07*std::sin(t*8.));
    auto body=pointcloud_to_laserscan::transform_ground(world,rotation.transpose(),
      -rotation.transpose()*translation);
    std::vector<Eigen::Vector3d> points;
    for(double x=.6;x<3.;x+=.12)for(double y=-1.;y<1.;y+=.12) {
      const double z=-(body.normal.x()*x+body.normal.y()*y+body.offset)/body.normal.z();
      points.emplace_back(x,y,z);
    }
    // A wrong raised-plane fit must not displace the repeatedly measured floor.
    auto candidate=body;
    if(i>=12 && i%4!=0)candidate.offset-=.15;
    const auto result=tracker.observe(candidate,10.+t,points,rotation,translation);
    if(i>=2) {
      ASSERT_TRUE(result.valid);EXPECT_TRUE(tracker.supported());
      EXPECT_NEAR(result.height(points.front()),0.,1.e-8);
    }
  }
}

TEST(ReferencedGround, LaggedAttitudeUsesMeasuredFloorWithoutAdmittingRaisedPlane)
{
  pointcloud_to_laserscan::ReferencedGroundTracker tracker;
  GroundPlane flat{{0.,0.,1.},.35,true};
  const Eigen::Vector3d translation(0.,0.,.35);
  std::vector<Eigen::Vector3d> points;
  for(double x=.6;x<3.;x+=.12)for(double y=-1.;y<1.;y+=.12)points.emplace_back(x,y,-.35);
  for(int i=0;i<3;++i)tracker.observe(flat,10.+i/15.,points,Eigen::Matrix3d::Identity(),translation);
  GroundPlane tilted{{-std::sin(.10),0.,std::cos(.10)},.35,true};
  for(auto & p:points)p.z()=-(tilted.normal.x()*p.x()+tilted.offset)/tilted.normal.z();
  auto result=tracker.observe(tilted,10.2,points,Eigen::Matrix3d::Identity(),translation);
  ASSERT_TRUE(tracker.supported());EXPECT_NEAR(result.height(points.front()),0.,1.e-8);
  for(auto & p:points)p.z()+=.15;
  auto raised=tilted;raised.offset-=.15*tilted.normal.z();
  tracker.observe(raised,10.27,points,Eigen::Matrix3d::Identity(),translation);
  EXPECT_FALSE(tracker.supported());
}

TEST(ReferencedGround, RejectsAPlaneAtBodyHeightAndRecoversWhenStanding)
{
  using pointcloud_to_laserscan::ReferencedGroundTracker;
  ReferencedGroundTracker tracker;
  const auto rotation = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d standing(0., 0., .35), crouched(0., 0., .02);
  const GroundPlane floor{{0., 0., 1.}, .35, true};
  std::vector<Eigen::Vector3d> points;
  for (double x = .6; x < 3.; x += .12) {
    for (double y = -1.; y < 1.; y += .12) {
      points.emplace_back(x, y, -.35);
    }
  }
  for (int i = 0; i < 3; ++i) {
    tracker.observe(floor, 10. + .1 * i, points, rotation, standing);
  }
  ASSERT_TRUE(tracker.supported());
  for (auto & point : points) {point.z() = -.02;}
  EXPECT_FALSE(tracker.observe({}, 10.3, points, rotation, crouched).valid);
  EXPECT_FALSE(tracker.supported());
  for (auto & point : points) {point.z() = -.35;}
  EXPECT_TRUE(tracker.observe(floor, 10.4, points, rotation, standing).valid);
  EXPECT_TRUE(tracker.supported());
}
