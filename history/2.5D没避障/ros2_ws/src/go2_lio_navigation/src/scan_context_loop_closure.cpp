#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "go2_lio_navigation/loop_registration.hpp"

using namespace std::chrono_literals;
using gtsam::symbol_shorthand::X;
namespace loop = go2_lio_navigation::loop;

namespace
{

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

Eigen::Isometry3d pose_to_eigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond q(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (q.norm() < 1.0e-9) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = q.toRotationMatrix();
  result.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  return result;
}

geometry_msgs::msg::Pose eigen_to_pose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Quaterniond q(transform.linear());
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

gtsam::Pose3 eigen_to_gtsam(const Eigen::Isometry3d & transform)
{
  return gtsam::Pose3(
    gtsam::Rot3(transform.linear()),
    gtsam::Point3(
      transform.translation().x(), transform.translation().y(), transform.translation().z()));
}

Eigen::Isometry3d gtsam_to_eigen(const gtsam::Pose3 & pose)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.matrix() = pose.matrix();
  return result;
}

Cloud::Ptr transform_cloud(const Cloud & input, const Eigen::Isometry3d & transform)
{
  auto output = std::make_shared<Cloud>();
  output->reserve(input.size());
  for (const auto & point : input) {
    if (!pcl::isFinite(point)) {
      continue;
    }
    const Eigen::Vector3d p = transform * Eigen::Vector3d(point.x, point.y, point.z);
    Point transformed;
    transformed.x = static_cast<float>(p.x());
    transformed.y = static_cast<float>(p.y());
    transformed.z = static_cast<float>(p.z());
    transformed.intensity = point.intensity;
    output->push_back(transformed);
  }
  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  output->is_dense = false;
  return output;
}

double rotation_angle(const Eigen::Matrix3d & rotation)
{
  const Eigen::AngleAxisd angle_axis(rotation);
  return std::abs(angle_axis.angle());
}

}  // namespace

class ScanContextLoopClosure final : public rclcpp::Node
{
public:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;

  ScanContextLoopClosure()
  : Node("scan_context_loop_closure"),
    isam_(make_isam_parameters())
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    cloud_topic_ = declare_parameter<std::string>(
      "cloud_topic", "/cloud_registered_body");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom_lio");
    keyframe_distance_ = declare_parameter<double>("keyframe_distance", 0.75);
    keyframe_angle_ = declare_parameter<double>("keyframe_angle_deg", 15.0) * M_PI / 180.0;
    keyframe_time_ = declare_parameter<double>("keyframe_time", 2.0);
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.20);
    rings_ = std::max(
      4, static_cast<int>(declare_parameter<int>("descriptor_rings", 20)));
    sectors_ = std::max(
      12, static_cast<int>(declare_parameter<int>("descriptor_sectors", 60)));
    max_radius_ = declare_parameter<double>("descriptor_max_radius", 35.0);
    descriptor_height_offset_ = declare_parameter<double>(
      "descriptor_height_offset", 2.0);
    descriptor_threshold_ = declare_parameter<double>(
      "descriptor_distance_threshold", 0.22);
    descriptor_min_overlap_ = declare_parameter<double>("descriptor_min_overlap", 0.50);
    descriptor_min_sectors_ = declare_parameter<int>("descriptor_min_sectors", 6);
    max_registration_candidates_ = std::max(
      1, static_cast<int>(declare_parameter<int>("max_registration_candidates", 3)));
    candidate_count_ = std::max(
      1, static_cast<int>(declare_parameter<int>("descriptor_candidate_count", 10)));
    recent_exclusion_ = std::max(
      1, static_cast<int>(declare_parameter<int>("recent_keyframe_exclusion", 35)));
    loop_cooldown_ = std::max(
      1, static_cast<int>(declare_parameter<int>("loop_keyframe_cooldown", 8)));
    submap_half_window_ = std::max(
      0, static_cast<int>(declare_parameter<int>("submap_half_window", 6)));
    source_submap_keyframes_ = std::max(
      1, static_cast<int>(declare_parameter<int>("source_submap_keyframes", 3)));
    min_loop_time_ = declare_parameter<double>("min_loop_time", 10.0);
    min_loop_travel_ = declare_parameter<double>("min_loop_travel", 2.0);
    loop_search_radius_ = declare_parameter<double>("loop_search_radius", 5.0);
    gicp_max_correspondence_ = declare_parameter<double>(
      "gicp_max_correspondence_distance", 1.5);
    gicp_max_iterations_ = std::max(
      10, static_cast<int>(declare_parameter<int>("gicp_max_iterations", 64)));
    gicp_fitness_threshold_ = declare_parameter<double>(
      "gicp_fitness_threshold", 0.04);
    registration_options_.inlier_distance = declare_parameter<double>(
      "gicp_inlier_distance", 0.30);
    registration_options_.min_overlap = declare_parameter<double>("gicp_min_overlap", 0.45);
    registration_options_.min_inliers = declare_parameter<int>("gicp_min_inliers", 100);
    registration_options_.min_normal_eigenvalue_ratio = declare_parameter<double>(
      "gicp_min_normal_eigenvalue_ratio", 0.01);
    registration_options_.max_vertical = declare_parameter<double>(
      "max_loop_vertical_correction", 0.35);
    registration_options_.max_tilt = declare_parameter<double>(
      "max_loop_tilt_correction_deg", 5.0) * M_PI / 180.0;
    max_loop_translation_ = declare_parameter<double>(
      "max_loop_translation_correction", 4.0);
    max_loop_rotation_ = declare_parameter<double>(
      "max_loop_rotation_correction_deg", 40.0) * M_PI / 180.0;
    registration_options_.correspondence_distance = gicp_max_correspondence_;
    registration_options_.iterations = gicp_max_iterations_;
    registration_options_.fitness_threshold = gicp_fitness_threshold_;
    registration_options_.max_translation = max_loop_translation_;
    registration_options_.max_rotation = max_loop_rotation_;
    odom_rotation_sigma_ = declare_parameter<double>("odom_rotation_sigma", 0.03);
    odom_translation_sigma_ = declare_parameter<double>("odom_translation_sigma", 0.08);
    loop_rotation_sigma_ = declare_parameter<double>("loop_rotation_sigma", 0.04);
    loop_translation_sigma_ = declare_parameter<double>("loop_translation_sigma", 0.12);
    global_map_leaf_size_ = declare_parameter<double>("global_map_leaf_size", 0.20);
    map_publish_interval_ = std::max(
      1, static_cast<int>(
        declare_parameter<int>("global_map_publish_every_n_keyframes", 5)));
    max_global_map_keyframes_ = std::max(
      1, static_cast<int>(declare_parameter<int>("max_global_map_keyframes", 800)));

    const auto transient_qos = rclcpp::QoS(1).reliable().transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/loop_closure/corrected_path", transient_qos);
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/loop_closure/global_map", transient_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/loop_closure/status", transient_qos);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_sub_.subscribe(this, odom_topic_, rmw_qos_profile_default);
    cloud_sub_.subscribe(this, cloud_topic_, rmw_qos_profile_default);
    synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(30), odom_sub_, cloud_sub_);
    // FAST-LIO publishes both at lidar_end_time. A 50 ms mismatch at an 8 m
    // wall during a turn can permanently smear geometry by tens of cm.
    synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.005));
    synchronizer_->registerCallback(
      std::bind(
        &ScanContextLoopClosure::on_measurement, this,
        std::placeholders::_1, std::placeholders::_2));
    tf_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    tf_timer_ = create_wall_timer(
      50ms, std::bind(&ScanContextLoopClosure::publish_tf, this), tf_callback_group_);

    publish_status("waiting_for_first_keyframe");
    RCLCPP_INFO(
      get_logger(), "Loop closure ready: Scan Context %dx%d, GICP + GTSAM iSAM2",
      rings_, sectors_);
  }

private:
  struct Keyframe
  {
    rclcpp::Time stamp;
    Eigen::Isometry3d raw_pose{Eigen::Isometry3d::Identity()};
    Cloud::Ptr scan;
    Eigen::MatrixXf descriptor;
    Eigen::VectorXf ring_key;
    double travel{0.0};
  };

  struct Candidate
  {
    std::size_t index{0};
    float distance{std::numeric_limits<float>::infinity()};
    int sector_shift{0};
  };

  struct Registration
  {
    Eigen::Isometry3d relative{Eigen::Isometry3d::Identity()};
    double fitness{std::numeric_limits<double>::infinity()};
    double overlap{0.0};
  };

  static gtsam::ISAM2Params make_isam_parameters()
  {
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.05;
    parameters.relinearizeSkip = 1;
    return parameters;
  }

  Cloud::Ptr downsample(const sensor_msgs::msg::PointCloud2 & message) const
  {
    auto input = std::make_shared<Cloud>();
    pcl::fromROSMsg(message, *input);
    auto sane = std::make_shared<Cloud>();
    sane->reserve(input->size());
    constexpr float kMaximumSaneCoordinate = 1.0e6F;
    for (const auto & point : *input) {
      if (pcl::isFinite(point) &&
        std::abs(point.x) < kMaximumSaneCoordinate &&
        std::abs(point.y) < kMaximumSaneCoordinate &&
        std::abs(point.z) < kMaximumSaneCoordinate)
      {
        sane->push_back(point);
      }
    }
    sane->width = static_cast<std::uint32_t>(sane->size());
    sane->height = 1;
    sane->is_dense = false;
    auto output = std::make_shared<Cloud>();
    if (sane->empty()) {
      return output;
    }
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(
      static_cast<float>(voxel_leaf_size_),
      static_cast<float>(voxel_leaf_size_),
      static_cast<float>(voxel_leaf_size_));
    voxel.setInputCloud(sane);
    voxel.filter(*output);
    return output;
  }

  Eigen::MatrixXf make_descriptor(const Cloud & cloud) const
  {
    Eigen::MatrixXf descriptor = Eigen::MatrixXf::Constant(
      rings_, sectors_, -std::numeric_limits<float>::infinity());
    for (const auto & point : cloud) {
      if (!pcl::isFinite(point)) {
        continue;
      }
      const double radius = std::hypot(point.x, point.y);
      if (radius < 0.1 || radius >= max_radius_) {
        continue;
      }
      double angle = std::atan2(point.y, point.x);
      if (angle < 0.0) {
        angle += 2.0 * M_PI;
      }
      const int ring = std::min(
        rings_ - 1, static_cast<int>(radius / max_radius_ * rings_));
      const int sector = std::min(
        sectors_ - 1, static_cast<int>(angle / (2.0 * M_PI) * sectors_));
      descriptor(ring, sector) = std::max(
        descriptor(ring, sector),
        static_cast<float>(point.z + descriptor_height_offset_));
    }
    for (int ring = 0; ring < rings_; ++ring) {
      for (int sector = 0; sector < sectors_; ++sector) {
        if (!std::isfinite(descriptor(ring, sector))) {
          descriptor(ring, sector) = 0.0F;
        }
      }
    }
    return descriptor;
  }

  static Eigen::VectorXf make_ring_key(const Eigen::MatrixXf & descriptor)
  {
    return descriptor.rowwise().mean();
  }

  std::vector<Candidate> find_candidates(const Keyframe & current) const
  {
    if (keyframes_.size() <= static_cast<std::size_t>(recent_exclusion_)) {
      return {};
    }
    const std::size_t current_index = keyframes_.size() - 1;
    const Eigen::Isometry3d current_pose = gtsam_to_eigen(
      estimate_.at<gtsam::Pose3>(X(current_index)));
    const std::size_t candidate_end = keyframes_.size() - recent_exclusion_;
    std::vector<std::pair<float, std::size_t>> ring_distances;
    for (std::size_t index = 0; index < candidate_end; ++index) {
      const auto & previous = keyframes_[index];
      if ((current.stamp - previous.stamp).seconds() < min_loop_time_ ||
        current.travel - previous.travel < min_loop_travel_)
      {
        continue;
      }
      const Eigen::Isometry3d candidate_pose = gtsam_to_eigen(
        estimate_.at<gtsam::Pose3>(X(index)));
      if ((current_pose.translation() - candidate_pose.translation()).norm() > loop_search_radius_) {
        continue;
      }
      ring_distances.emplace_back((current.ring_key - previous.ring_key).norm(), index);
    }
    const std::size_t count = std::min(
      static_cast<std::size_t>(candidate_count_), ring_distances.size());
    std::partial_sort(
      ring_distances.begin(), ring_distances.begin() + count, ring_distances.end());
    std::vector<Candidate> candidates;
    for (std::size_t rank = 0; rank < count; ++rank) {
      const auto index = ring_distances[rank].second;
      Candidate best;
      for (int shift = 0; shift < sectors_; ++shift) {
        const float distance = loop::descriptor_distance(
          current.descriptor, keyframes_[index].descriptor, shift,
          descriptor_min_overlap_, descriptor_min_sectors_);
        if (distance < best.distance) {
          best = Candidate{index, distance, shift};
        }
      }
      if (best.distance <= descriptor_threshold_) {
        candidates.push_back(best);
      }
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
      return a.distance < b.distance;
    });
    if (candidates.size() > static_cast<std::size_t>(max_registration_candidates_)) {
      candidates.resize(max_registration_candidates_);
    }
    return candidates;
  }

  std::optional<Registration> verify_loop(
    std::size_t candidate_index, std::size_t current_index, int sector_shift)
  {
    const Eigen::Isometry3d candidate_pose = gtsam_to_eigen(
      estimate_.at<gtsam::Pose3>(X(candidate_index)));
    const Eigen::Isometry3d current_pose = gtsam_to_eigen(
      estimate_.at<gtsam::Pose3>(X(current_index)));
    const Eigen::Isometry3d candidate_inverse = candidate_pose.inverse();
    auto target = std::make_shared<Cloud>();
    const std::size_t begin = candidate_index > static_cast<std::size_t>(submap_half_window_) ?
      candidate_index - static_cast<std::size_t>(submap_half_window_) : 0;
    // Never allow the current/recent source scans to verify themselves.
    const std::size_t end = std::min(
      current_index - static_cast<std::size_t>(recent_exclusion_),
      candidate_index + static_cast<std::size_t>(submap_half_window_));
    for (std::size_t index = begin; index <= end; ++index) {
      const Eigen::Isometry3d neighbor = gtsam_to_eigen(estimate_.at<gtsam::Pose3>(X(index)));
      *target += *transform_cloud(*keyframes_[index].scan, candidate_inverse * neighbor);
    }
    auto source = std::make_shared<Cloud>();
    const std::size_t source_begin = std::max(
      end + 1, current_index + 1 > static_cast<std::size_t>(source_submap_keyframes_) ?
      current_index + 1 - static_cast<std::size_t>(source_submap_keyframes_) : 0);
    const Eigen::Isometry3d current_raw_inverse = keyframes_[current_index].raw_pose.inverse();
    for (std::size_t index = source_begin; index <= current_index; ++index) {
      *source += *transform_cloud(
        *keyframes_[index].scan, current_raw_inverse * keyframes_[index].raw_pose);
    }
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    auto filtered_target = std::make_shared<Cloud>();
    auto filtered_source = std::make_shared<Cloud>();
    voxel.setInputCloud(target);
    voxel.filter(*filtered_target);
    voxel.setInputCloud(source);
    voxel.filter(*filtered_source);
    const auto result = loop::register_clouds(
      filtered_source, filtered_target, candidate_inverse * current_pose,
      sector_shift, sectors_, registration_options_);
    if (!result.accepted) {
      publish_status(
        "loop_rejected current=" + std::to_string(current_index) +
        " candidate=" + std::to_string(candidate_index) + " reason=" + result.reason +
        " overlap=" + std::to_string(result.overlap) +
        " normal_ratio=" + std::to_string(result.normal_eigenvalue_ratio));
      return std::nullopt;
    }
    return Registration{result.relative, result.fitness, result.overlap};
  }

  gtsam::SharedNoiseModel diagonal_noise(double rotation_sigma, double translation_sigma) const
  {
    gtsam::Vector6 sigmas;
    sigmas << rotation_sigma, rotation_sigma, rotation_sigma,
      translation_sigma, translation_sigma, translation_sigma;
    return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  }

  void add_keyframe_to_graph(std::size_t index)
  {
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values initial;
    if (index == 0) {
      factors.add(
        gtsam::PriorFactor<gtsam::Pose3>(
          X(0), eigen_to_gtsam(keyframes_[0].raw_pose),
          diagonal_noise(0.01, 0.01)));
      initial.insert(X(0), eigen_to_gtsam(keyframes_[0].raw_pose));
    } else {
      const Eigen::Isometry3d relative =
        keyframes_[index - 1].raw_pose.inverse() * keyframes_[index].raw_pose;
      factors.add(
        gtsam::BetweenFactor<gtsam::Pose3>(
          X(index - 1), X(index), eigen_to_gtsam(relative),
          diagonal_noise(odom_rotation_sigma_, odom_translation_sigma_)));
      const gtsam::Pose3 previous = estimate_.at<gtsam::Pose3>(X(index - 1));
      initial.insert(X(index), previous.compose(eigen_to_gtsam(relative)));
    }
    isam_.update(factors, initial);
    isam_.update();
    estimate_ = isam_.calculateEstimate();
  }

  void add_loop_factor(
    std::size_t candidate_index, std::size_t current_index,
    const Registration & registration)
  {
    const auto robust_noise = loop::loop_noise(loop_rotation_sigma_, loop_translation_sigma_);
    gtsam::NonlinearFactorGraph factors;
    factors.add(
      gtsam::BetweenFactor<gtsam::Pose3>(
        X(candidate_index), X(current_index), eigen_to_gtsam(registration.relative), robust_noise));
    gtsam::Values empty;
    isam_.update(factors, empty);
    for (int iteration = 0; iteration < 5; ++iteration) {
      isam_.update();
    }
    estimate_ = isam_.calculateEstimate();
  }

  bool should_make_keyframe(const rclcpp::Time & stamp, const Eigen::Isometry3d & pose) const
  {
    if (keyframes_.empty()) {
      return true;
    }
    const auto & previous = keyframes_.back();
    const Eigen::Isometry3d delta = previous.raw_pose.inverse() * pose;
    const bool time_triggered = keyframe_time_ > 0.0 &&
      (stamp - previous.stamp).seconds() >= keyframe_time_;
    return delta.translation().norm() >= keyframe_distance_ ||
           rotation_angle(delta.linear()) >= keyframe_angle_ ||
           time_triggered;
  }

  void on_measurement(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odometry,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_message)
  {
    const rclcpp::Time stamp(odometry->header.stamp);
    const Eigen::Isometry3d raw_pose = pose_to_eigen(odometry->pose.pose);
    if (!should_make_keyframe(stamp, raw_pose)) {
      return;
    }
    auto scan = downsample(*cloud_message);
    if (scan->size() < 100) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Skipping sparse keyframe (%zu points)", scan->size());
      return;
    }

    Keyframe keyframe;
    keyframe.stamp = stamp;
    keyframe.raw_pose = raw_pose;
    if (!keyframes_.empty()) {
      keyframe.travel = keyframes_.back().travel +
        (raw_pose.translation() - keyframes_.back().raw_pose.translation()).norm();
    }
    keyframe.scan = std::move(scan);
    keyframe.descriptor = make_descriptor(*keyframe.scan);
    keyframe.ring_key = make_ring_key(keyframe.descriptor);
    keyframes_.push_back(std::move(keyframe));
    const std::size_t current_index = keyframes_.size() - 1;
    add_keyframe_to_graph(current_index);

    bool loop_accepted = false;
    const bool cooldown_elapsed = last_loop_keyframe_ < 0 ||
      current_index >= static_cast<std::size_t>(last_loop_keyframe_ + loop_cooldown_);
    if (cooldown_elapsed) {
      const auto candidates = find_candidates(keyframes_.back());
      if (candidates.empty()) {
        publish_status("tracking keyframes=" + std::to_string(current_index + 1) +
          " candidates=0 loops=" + std::to_string(accepted_loops_));
      }
      for (const auto & candidate : candidates) {
        const auto registration = verify_loop(
          candidate.index, current_index, candidate.sector_shift);
        if (registration) {
          const Eigen::Isometry3d predicted = gtsam_to_eigen(
            estimate_.at<gtsam::Pose3>(X(candidate.index))).inverse() *
            gtsam_to_eigen(estimate_.at<gtsam::Pose3>(X(current_index)));
          const double correction =
            (registration->relative.translation() - predicted.translation()).norm();
          add_loop_factor(candidate.index, current_index, *registration);
          last_loop_keyframe_ = static_cast<int>(current_index);
          ++accepted_loops_;
          loop_accepted = true;
          publish_status(
            "loop_accepted current=" + std::to_string(current_index) +
            " candidate=" + std::to_string(candidate.index) +
            " descriptor=" + std::to_string(candidate.distance) +
            " rmse=" + std::to_string(std::sqrt(registration->fitness)) +
            " overlap=" + std::to_string(registration->overlap) +
            " correction_m=" + std::to_string(correction));
          RCLCPP_INFO(
            get_logger(),
            "Loop %zu <-> %zu accepted: descriptor %.3f, RMSE %.3f m, overlap %.0f%%, correction %.3f m",
            current_index, candidate.index, candidate.distance,
            std::sqrt(registration->fitness), registration->overlap * 100.0, correction);
          break;
        }
      }
    }

    update_map_to_odom(current_index);
    publish_path();
    if (loop_accepted || current_index % static_cast<std::size_t>(map_publish_interval_) == 0) {
      publish_global_map();
    }
  }

  void update_map_to_odom(std::size_t latest_index)
  {
    const Eigen::Isometry3d optimized = gtsam_to_eigen(
      estimate_.at<gtsam::Pose3>(X(latest_index)));
    const std::lock_guard<std::mutex> lock(transform_mutex_);
    map_to_odom_ = optimized * keyframes_[latest_index].raw_pose.inverse();
  }

  void publish_tf()
  {
    Eigen::Isometry3d map_to_odom;
    {
      const std::lock_guard<std::mutex> lock(transform_mutex_);
      map_to_odom = map_to_odom_;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = map_to_odom.translation().x();
    transform.transform.translation.y = map_to_odom.translation().y();
    transform.transform.translation.z = map_to_odom.translation().z();
    const Eigen::Quaterniond q(map_to_odom.linear());
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transform);
  }

  void publish_path()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = map_frame_;
    path.poses.reserve(keyframes_.size());
    for (std::size_t index = 0; index < keyframes_.size(); ++index) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = keyframes_[index].stamp;
      pose.header.frame_id = map_frame_;
      pose.pose = eigen_to_pose(gtsam_to_eigen(estimate_.at<gtsam::Pose3>(X(index))));
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }

  void publish_global_map()
  {
    auto accumulated = std::make_shared<Cloud>();
    const std::size_t begin = keyframes_.size() >
      static_cast<std::size_t>(max_global_map_keyframes_) ?
      keyframes_.size() - static_cast<std::size_t>(max_global_map_keyframes_) : 0;
    for (std::size_t index = begin; index < keyframes_.size(); ++index) {
      const Eigen::Isometry3d pose = gtsam_to_eigen(
        estimate_.at<gtsam::Pose3>(X(index)));
      *accumulated += *transform_cloud(*keyframes_[index].scan, pose);
    }
    auto filtered = std::make_shared<Cloud>();
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(
      static_cast<float>(global_map_leaf_size_),
      static_cast<float>(global_map_leaf_size_),
      static_cast<float>(global_map_leaf_size_));
    voxel.setInputCloud(accumulated);
    voxel.filter(*filtered);
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*filtered, message);
    message.header.stamp = now();
    message.header.frame_id = map_frame_;
    map_pub_->publish(message);
  }

  void publish_status(const std::string & text)
  {
    std_msgs::msg::String message;
    message.data = text;
    status_pub_->publish(message);
  }

  std::string odom_topic_;
  std::string cloud_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  double keyframe_distance_{0.75};
  double keyframe_angle_{0.26};
  double keyframe_time_{2.0};
  double voxel_leaf_size_{0.20};
  int rings_{20};
  int sectors_{60};
  double max_radius_{35.0};
  double descriptor_height_offset_{2.0};
  double descriptor_threshold_{0.22};
  double descriptor_min_overlap_{0.50};
  int descriptor_min_sectors_{6};
  int max_registration_candidates_{3};
  int candidate_count_{10};
  int recent_exclusion_{35};
  int loop_cooldown_{8};
  int submap_half_window_{6};
  int source_submap_keyframes_{3};
  double min_loop_time_{10.0};
  double min_loop_travel_{2.0};
  double loop_search_radius_{5.0};
  loop::Options registration_options_;
  double gicp_max_correspondence_{1.5};
  int gicp_max_iterations_{64};
  double gicp_fitness_threshold_{0.04};
  double max_loop_translation_{4.0};
  double max_loop_rotation_{0.70};
  double odom_rotation_sigma_{0.03};
  double odom_translation_sigma_{0.08};
  double loop_rotation_sigma_{0.04};
  double loop_translation_sigma_{0.12};
  double global_map_leaf_size_{0.20};
  int map_publish_interval_{5};
  int max_global_map_keyframes_{800};
  int last_loop_keyframe_{-1};
  int accepted_loops_{0};

  std::vector<Keyframe> keyframes_;
  gtsam::ISAM2 isam_;
  gtsam::Values estimate_;
  Eigen::Isometry3d map_to_odom_{Eigen::Isometry3d::Identity()};
  std::mutex transform_mutex_;
  rclcpp::CallbackGroup::SharedPtr tf_callback_group_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<ScanContextLoopClosure>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
