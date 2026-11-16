#include <cmath>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "go2_lio_navigation/loop_registration.hpp"

namespace loop = go2_lio_navigation::loop;

namespace
{
Eigen::Isometry3d pose(double x, double y, double yaw)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(x, y, 0.0);
  result.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return result;
}

loop::Cloud::Ptr room()
{
  auto cloud = std::make_shared<loop::Cloud>();
  for (int i = 0; i < 32; ++i) {
    for (int j = 0; j < 24; ++j) {
      for (int plane = 0; plane < 3; ++plane) {
        loop::Point p;
        p.x = plane == 1 ? 3.0 : -1.0 + i * 0.125;
        p.y = plane == 2 ? 2.5 : -0.5 + j * 0.125;
        p.z = plane == 0 ? -0.8 : -0.8 + j * 0.1;
        // Different sampling along the two walls avoids exact line copies.
        if (plane == 1) {
          p.y = -0.5 + i * 0.125;
        }
        cloud->push_back(p);
      }
    }
  }
  return cloud;
}

loop::Cloud::Ptr transformed(const loop::Cloud & cloud, const Eigen::Isometry3d & t)
{
  auto output = std::make_shared<loop::Cloud>();
  for (const auto & p : cloud) {
    const Eigen::Vector3d v = t * p.getVector3fMap().cast<double>();
    loop::Point q;
    q.x = v.x();
    q.y = v.y();
    q.z = v.z();
    output->push_back(q);
  }
  return output;
}
}  // namespace

TEST(LoopRegistration, DifferentPhysicalHeadingDoesNotBecomeWorldCorrection)
{
  const auto target = room();
  const auto candidate_world = pose(14.0, -7.0, 0.9);
  const auto current_world = pose(14.2, -7.1, 0.9 + 2.8);
  const Eigen::Isometry3d truth = candidate_world.inverse() * current_world;
  const auto source = transformed(*target, truth.inverse());
  Eigen::Isometry3d predicted = truth;
  predicted.translation() += Eigen::Vector3d(0.30, -0.20, 0.04);
  predicted.linear() = Eigen::AngleAxisd(0.06, Eigen::Vector3d::UnitZ()) * truth.linear();
  const auto result = loop::register_clouds(source, target, predicted, 27, 60, loop::Options{});
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_LT((result.relative.translation() - truth.translation()).norm(), 0.025);
  EXPECT_LT(Eigen::AngleAxisd(result.relative.linear() * truth.linear().transpose()).angle(), 0.01);
  EXPECT_GT(result.overlap, 0.95);
}

TEST(LoopRegistration, GoodOdometrySeedSurvivesAnAmbiguousDescriptorYaw)
{
  const auto target = room();
  const auto truth = pose(0.15, -0.1, 0.25);
  const auto source = transformed(*target, truth.inverse());
  const auto predicted = pose(0.35, -0.2, 0.30);
  const auto result = loop::register_clouds(source, target, predicted, 21, 60, loop::Options{});
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_LT((result.relative.translation() - truth.translation()).norm(), 0.025);
}

TEST(LoopRegistration, RejectsFloorOnlyEvenWithPerfectNearestNeighborFit)
{
  auto floor = std::make_shared<loop::Cloud>();
  for (int i = 0; i < 30; ++i) {
    for (int j = 0; j < 30; ++j) {
      loop::Point p;
      p.x = i * 0.1;
      p.y = j * 0.1;
      p.z = 0.0;
      floor->push_back(p);
    }
  }
  const auto result = loop::assess_alignment(
    *floor, floor, pose(0, 0, 0), pose(0, 0, 0), loop::Options{});
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "degenerate_surfaces");
  EXPECT_NEAR(result.overlap, 1.0, 1.e-6);
}

TEST(LoopRegistration, RejectsTinyCommonCloudPatch)
{
  const auto target = room();
  auto source = transformed(*target, pose(20, 0, 0));
  source->insert(source->end(), target->begin(), target->begin() + 150);
  const auto result = loop::assess_alignment(
    *source, target, pose(0, 0, 0), pose(0, 0, 0), loop::Options{});
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "insufficient_overlap");
  EXPECT_LT(result.overlap, 0.1);
}

TEST(LoopRegistration, EnforcesVerticalAndTiltCorrectionLimits)
{
  const auto target = room();
  auto estimate = pose(0, 0, 0);
  estimate.translation().z() = 0.5;
  auto result = loop::assess_alignment(
    *target, target, estimate, pose(0, 0, 0), loop::Options{});
  EXPECT_EQ(result.reason, "correction_limit");
  estimate = pose(0, 0, 0);
  estimate.linear() = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  result = loop::assess_alignment(*target, target, estimate, pose(0, 0, 0), loop::Options{});
  EXPECT_EQ(result.reason, "correction_limit");
}

TEST(LoopRegistration, FitnessUsesSquaredMetersAndCountsAllSourcePoints)
{
  auto target = std::make_shared<loop::Cloud>();
  for (int i = 0; i < 15; ++i) {
    for (int j = 0; j < 15; ++j) {
      loop::Point p;
      p.x = i;
      p.y = j;
      p.z = 0.0;
      target->push_back(p);
    }
  }
  auto offset = pose(0, 0, 0);
  offset.translation().z() = 0.25;
  const auto source = transformed(*target, offset);
  const auto result = loop::assess_alignment(
    *source, target, pose(0, 0, 0), pose(0, 0, 0), loop::Options{});
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "inlier_rmse");
  EXPECT_NEAR(result.fitness, 0.0625, 1.e-6);
  EXPECT_NEAR(result.overlap, 1.0, 1.e-6);
}

TEST(LoopRegistration, DescriptorRejectsMatchingOneSectorOfAnOtherwiseDifferentView)
{
  Eigen::MatrixXf first = Eigen::MatrixXf::Zero(20, 60);
  Eigen::MatrixXf second = first;
  first.block(0, 0, 20, 20).setOnes();
  second.col(0).setOnes();
  EXPECT_EQ(loop::descriptor_distance(first, second, 0, 0.5, 6), 1.0F);
  second = first;
  EXPECT_NEAR(loop::descriptor_distance(first, second, 0, 0.5, 6), 0.0F, 1.e-6);
  second.setZero();
  second.block(0, 15, 20, 20).setOnes();
  EXPECT_NEAR(loop::descriptor_distance(first, second, 15, 0.5, 6), 0.0F, 1.e-6);
}

TEST(LoopRegistration, VerifiedLoopActuallyCorrectsAccumulatedGraphDrift)
{
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.001;
  parameters.relinearizeSkip = 1;
  gtsam::ISAM2 isam(parameters);
  const auto odom_noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.08);
  for (std::size_t i = 0; i <= 12; ++i) {
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
    const gtsam::Pose3 initial(gtsam::Rot3(), gtsam::Point3(i * 0.05, 0, 0));
    values.insert(i, initial);
    if (i == 0) {
      factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        0, initial, gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));
    } else {
      factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
        i - 1, i, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.05, 0, 0)), odom_noise));
    }
    isam.update(factors, values);
  }
  gtsam::NonlinearFactorGraph factors;
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
    0, 12, gtsam::Pose3(), loop::loop_noise(0.04, 0.08)));
  isam.update(factors, gtsam::Values());
  for (int i = 0; i < 5; ++i) {
    isam.update();
  }
  const double remaining = isam.calculateEstimate().at<gtsam::Pose3>(12).x();
  EXPECT_LT(std::abs(remaining), 0.10) << "Loop accepted but graph barely corrected";
  std::cout << "Graph drift: 0.600 m -> " << remaining << " m" << std::endl;
}

TEST(LoopRegistration, CapturedHapCloudWithInjectedKnownPoseError)
{
  const char * filename = std::getenv("GO2_LOOP_REAL_CLOUD");
  if (filename == nullptr) {
    GTEST_SKIP() << "Set GO2_LOOP_REAL_CLOUD to an ASCII XYZ cloud for offline replay";
  }
  std::ifstream file(filename);
  ASSERT_TRUE(file.good());
  auto target = std::make_shared<loop::Cloud>();
  loop::Point p;
  while (file >> p.x >> p.y >> p.z) {
    target->push_back(p);
  }
  ASSERT_GE(target->size(), 300U);
  const auto truth = pose(0.2, -0.1, 2.6);
  const auto source = transformed(*target, truth.inverse());
  auto predicted = truth;
  predicted.translation() += Eigen::Vector3d(0.45, -0.25, 0.06);
  predicted.linear() = Eigen::AngleAxisd(0.07, Eigen::Vector3d::UnitZ()) * truth.linear();
  const auto start = std::chrono::steady_clock::now();
  const auto result = loop::register_clouds(source, target, predicted, 25, 60, loop::Options{});
  const double elapsed = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  ASSERT_TRUE(result.accepted) << result.reason << " ratio=" << result.normal_eigenvalue_ratio;
  const double error = (result.relative.translation() - truth.translation()).norm();
  EXPECT_LT(error, 0.03);
  std::cout << "Captured HAP offline injection: " << target->size() << " points, error "
            << error << " m, overlap " << result.overlap << ", " << elapsed << " ms" << std::endl;
}
