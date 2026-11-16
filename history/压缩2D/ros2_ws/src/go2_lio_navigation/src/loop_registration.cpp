#include "go2_lio_navigation/loop_registration.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include <Eigen/Eigenvalues>
#include <pcl/common/point_tests.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>

namespace go2_lio_navigation::loop
{
float descriptor_distance(
  const Eigen::MatrixXf & first, const Eigen::MatrixXf & second, int shift,
  double min_overlap, int min_sectors)
{
  if (first.rows() != second.rows() || first.cols() != second.cols() || first.cols() == 0) {
    return 1.0F;
  }
  const int sectors = first.cols();
  shift = (shift % sectors + sectors) % sectors;
  float similarity = 0.0F;
  int compared = 0;
  int first_observed = 0;
  int second_observed = 0;
  for (int sector = 0; sector < sectors; ++sector) {
    const auto a = first.col(sector);
    const auto b = second.col((sector + shift) % sectors);
    const float an = a.norm();
    const float bn = b.norm();
    first_observed += an > 1.0e-6F;
    second_observed += bn > 1.0e-6F;
    if (an > 1.0e-6F && bn > 1.0e-6F) {
      similarity += a.dot(b) / (an * bn);
      ++compared;
    }
  }
  // Use the larger observed footprint: one matching sliver of a large scan
  // must not obtain a perfect score against a mostly empty descriptor.
  const int observed = std::max(first_observed, second_observed);
  if (compared < min_sectors || compared < min_overlap * observed) {
    return 1.0F;
  }
  return std::clamp(1.0F - similarity / compared, 0.0F, 1.0F);
}

Result assess_alignment(
  const Cloud & source, const Cloud::ConstPtr & target,
  const Eigen::Isometry3d & relative, const Eigen::Isometry3d & predicted,
  const Options & options)
{
  Result result;
  result.relative = relative;
  if (source.empty() || target->size() < 20 || !relative.matrix().allFinite() ||
    !relative.linear().transpose().isApprox(relative.linear().inverse(), 1.0e-3) ||
    std::abs(relative.linear().determinant() - 1.0) > 1.0e-3)
  {
    result.reason = "invalid_geometry";
    return result;
  }
  const Eigen::Isometry3d delta = relative * predicted.inverse();
  if ((relative.translation() - predicted.translation()).norm() > options.max_translation ||
    std::abs(Eigen::AngleAxisd(delta.linear()).angle()) > options.max_rotation ||
    std::abs(relative.translation().z() - predicted.translation().z()) > options.max_vertical ||
    std::acos(std::clamp(delta.linear()(2, 2), -1.0, 1.0)) > options.max_tilt)
  {
    result.reason = "correction_limit";
    return result;
  }

  pcl::KdTreeFLANN<Point> tree;
  tree.setInputCloud(target);
  pcl::Indices indices(1);
  std::vector<float> distances(1);
  std::vector<bool> matched(target->size(), false);
  double squared_error = 0.0;
  const double threshold_squared = options.inlier_distance * options.inlier_distance;
  std::size_t valid_source = 0;
  for (const auto & p : source) {
    if (!pcl::isFinite(p)) {
      continue;
    }
    ++valid_source;
    const Eigen::Vector3d transformed = relative * Eigen::Vector3d(p.x, p.y, p.z);
    Point query;
    query.x = transformed.x();
    query.y = transformed.y();
    query.z = transformed.z();
    if (tree.nearestKSearch(query, 1, indices, distances) == 1 &&
      distances[0] <= threshold_squared)
    {
      ++result.inliers;
      squared_error += distances[0];
      matched[indices[0]] = true;
    }
  }
  result.overlap = valid_source == 0 ? 0.0 :
    static_cast<double>(result.inliers) / valid_source;
  if (result.inliers < options.min_inliers || result.overlap < options.min_overlap) {
    result.reason = "insufficient_overlap";
    return result;
  }
  result.fitness = squared_error / result.inliers;
  if (result.fitness > options.fitness_threshold) {
    result.reason = "inlier_rmse";
    return result;
  }

  // A floor (or floor plus one parallel wall) may align with low distance
  // while sliding along its unconstrained direction. Require independent
  // surface normals in the actual matched area, not the entire target map.
  Eigen::Matrix3d normal_information = Eigen::Matrix3d::Zero();
  std::vector<std::size_t> normal_indices;
  for (std::size_t index = 0; index < matched.size(); ++index) {
    if (matched[index]) {
      normal_indices.push_back(index);
    }
  }
  // Fixed-stride sampling aliases ordered clouds (e.g. floor/wall/wall
  // repeated), making a fully constrained room look like a single plane.
  std::mt19937 generator(0);
  std::shuffle(normal_indices.begin(), normal_indices.end(), generator);
  normal_indices.resize(std::min<std::size_t>(600, normal_indices.size()));
  for (std::size_t index : normal_indices) {
    if (tree.nearestKSearch((*target)[index], 12, indices, distances) < 12) {
      continue;
    }
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (int neighbor : indices) {
      mean += (*target)[neighbor].getVector3fMap().cast<double>();
    }
    mean /= indices.size();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int neighbor : indices) {
      const Eigen::Vector3d centered = (*target)[neighbor].getVector3fMap().cast<double>() - mean;
      covariance.noalias() += centered * centered.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success || solver.eigenvalues()[1] < 1.0e-8 ||
      solver.eigenvalues()[0] > 0.15 * solver.eigenvalues()[1])
    {
      continue;
    }
    const Eigen::Vector3d normal = solver.eigenvectors().col(0);
    normal_information.noalias() += normal * normal.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(normal_information);
  if (solver.info() == Eigen::Success && solver.eigenvalues()[2] > 1.0e-8) {
    result.normal_eigenvalue_ratio = solver.eigenvalues()[0] / solver.eigenvalues()[2];
  }
  if (result.normal_eigenvalue_ratio < options.min_normal_eigenvalue_ratio) {
    result.reason = "degenerate_surfaces";
    return result;
  }
  result.accepted = true;
  result.reason = "accepted";
  return result;
}

Result register_clouds(
  const Cloud::ConstPtr & source, const Cloud::ConstPtr & target,
  const Eigen::Isometry3d & predicted, int sector_shift, int sectors,
  const Options & options)
{
  Result result;
  if (source->size() < 100 || target->size() < 300) {
    result.reason = "sparse_submap";
    return result;
  }
  pcl::GeneralizedIterativeClosestPoint<Point, Point> gicp;
  gicp.setMaximumIterations(options.iterations);
  gicp.setMaxCorrespondenceDistance(options.correspondence_distance);
  gicp.setTransformationEpsilon(1.0e-5);
  gicp.setEuclideanFitnessEpsilon(1.0e-5);
  gicp.setInputSource(source);
  gicp.setInputTarget(target);
  const auto attempt = [&](const Eigen::Isometry3d & seed) {
      Cloud aligned;
      gicp.align(aligned, seed.matrix().cast<float>());
      if (!gicp.hasConverged()) {
        return Result{};
      }
      Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
      relative.matrix() = gicp.getFinalTransformation().cast<double>();
      return assess_alignment(*source, target, relative, predicted, options);
    };
  // LIO already predicts the large physical change of robot heading. Never
  // rotate world-frame clouds again by that heading from Scan Context.
  result = attempt(predicted);
  if (result.accepted) {
    return result;
  }
  const double predicted_yaw = std::atan2(predicted.linear()(1, 0), predicted.linear()(0, 0));
  const double descriptor_yaw = 2.0 * M_PI * sector_shift / sectors;
  const double yaw_delta = std::remainder(descriptor_yaw - predicted_yaw, 2.0 * M_PI);
  if (std::abs(yaw_delta) > M_PI / sectors &&
    std::abs(yaw_delta) <= options.max_rotation)
  {
    Eigen::Isometry3d seed = predicted;
    seed.linear() = Eigen::AngleAxisd(yaw_delta, Eigen::Vector3d::UnitZ()) * predicted.linear();
    const Result alternative = attempt(seed);
    if (alternative.accepted) {
      return alternative;
    }
  }
  return result;
}

gtsam::SharedNoiseModel loop_noise(double rotation_sigma, double translation_sigma)
{
  gtsam::Vector6 sigmas;
  sigmas << rotation_sigma, rotation_sigma, rotation_sigma,
    translation_sigma, translation_sigma, translation_sigma;
  // Cauchy(1) almost removes exactly the large residual that a verified loop
  // should correct. Huber retains a bounded, nonzero correcting influence.
  return gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Huber::Create(1.345),
    gtsam::noiseModel::Diagonal::Sigmas(sigmas));
}
}  // namespace go2_lio_navigation::loop
