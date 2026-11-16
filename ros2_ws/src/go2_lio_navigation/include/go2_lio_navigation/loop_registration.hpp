#pragma once

#include <limits>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtsam/linear/NoiseModel.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace go2_lio_navigation::loop
{
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

struct Options
{
  double correspondence_distance{1.5};
  int iterations{40};
  double fitness_threshold{0.04};  // Mean squared inlier distance, m^2.
  double inlier_distance{0.30};
  double min_overlap{0.45};
  int min_inliers{100};
  double min_normal_eigenvalue_ratio{0.01};
  double max_translation{4.0};
  double max_rotation{0.70};
  double max_vertical{0.35};
  double max_tilt{0.0872664626};
};

struct Result
{
  bool accepted{false};
  std::string reason{"not_converged"};
  Eigen::Isometry3d relative{Eigen::Isometry3d::Identity()};
  double fitness{std::numeric_limits<double>::infinity()};
  double overlap{0.0};
  int inliers{0};
  double normal_eigenvalue_ratio{0.0};
};

// Empty sectors cannot count as evidence for a narrow-field-of-view HAP.
float descriptor_distance(
  const Eigen::MatrixXf & first, const Eigen::MatrixXf & second, int shift,
  double min_overlap, int min_sectors);

Result assess_alignment(
  const Cloud & source, const Cloud::ConstPtr & target,
  const Eigen::Isometry3d & relative, const Eigen::Isometry3d & predicted,
  const Options & options);

// All inputs and seeds are expressed in the historical candidate's frame.
Result register_clouds(
  const Cloud::ConstPtr & source, const Cloud::ConstPtr & target,
  const Eigen::Isometry3d & predicted, int sector_shift, int sectors,
  const Options & options);

gtsam::SharedNoiseModel loop_noise(double rotation_sigma, double translation_sigma);
}  // namespace go2_lio_navigation::loop
