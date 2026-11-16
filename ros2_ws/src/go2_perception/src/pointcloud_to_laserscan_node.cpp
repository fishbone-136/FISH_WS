/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include "pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp"
#include "pointcloud_to_laserscan/scan_projection.hpp"

#include <chrono>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "tf2_ros/create_timer_ros.h"

namespace pointcloud_to_laserscan
{

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_laserscan", options)
{
  target_frame_ = this->declare_parameter("target_frame", "");
  scan_frame_ = this->declare_parameter("scan_frame", "");
  level_scan_ = this->declare_parameter("level_scan", false);
  if (level_scan_ && (scan_frame_.empty() || scan_frame_ == target_frame_)) {
    throw std::runtime_error("Levelled scan needs a distinct planar frame");
  }
  if (!level_scan_ && !scan_frame_.empty() && scan_frame_ != target_frame_) {
    throw std::invalid_argument("A distinct scan_frame requires level_scan; relabelling is not a transform");
  }
  tolerance_ = this->declare_parameter("transform_tolerance", 0.01);
  // TODO(hidmic): adjust default input queue size based on actual concurrency levels
  // achievable by the associated executor
  input_queue_size_ = this->declare_parameter(
    "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  min_height_ = this->declare_parameter("min_height", 0.08);
  max_height_ = this->declare_parameter("max_height", 1.00);
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter("use_inf", true);
  ground_filter_enabled_ = this->declare_parameter("ground_filter_enabled", false);
  observed_free_space_only_ = this->declare_parameter("observed_free_space_only", false);
  ground_clearance_ = this->declare_parameter("ground_clearance", 0.10);
  obstacle_height_ = this->declare_parameter("obstacle_height_above_ground", 1.3);
  // A 3-D lidar often contributes only one or two returns from a low object to
  // a horizontal angular bin.  Give each real return its physical cross-section
  // and bridge only tiny, same-surface angular holes.  This is single-frame
  // densification, so it cannot create the rotational ghosts caused by temporal
  // point-cloud accumulation.
  point_spread_radius_ = this->declare_parameter("point_spread_radius", 0.04);
  max_spread_bins_ = this->declare_parameter("max_spread_bins", 3);
  max_fill_gap_bins_ = this->declare_parameter("max_fill_gap_bins", 3);
  max_fill_range_difference_ =
    this->declare_parameter("max_fill_range_difference", 0.20);
  tf_buffer_duration_ = this->declare_parameter("tf_buffer_duration", 10.0);
  use_best_effort_ = this->declare_parameter("use_best_effort", true);

  if (input_queue_size_ <= 0 || !std::isfinite(angle_min_) || !std::isfinite(angle_max_) ||
    !std::isfinite(angle_increment_) || angle_increment_ <= 0.0 || angle_max_ <= angle_min_ ||
    (angle_max_ - angle_min_) / angle_increment_ > 1000000.0 ||
    !std::isfinite(range_min_) || !std::isfinite(range_max_) ||
    range_min_ < 0.0 || range_max_ <= range_min_)
  {
    throw std::invalid_argument("Invalid scan angular/range limits or queue size");
  }

  if (point_spread_radius_ < 0.0 || max_spread_bins_ < 0 || max_fill_gap_bins_ < 0 ||
    max_fill_range_difference_ < 0.0)
  {
    throw std::invalid_argument("scan densification parameters must be non-negative");
  }

  rclcpp::SensorDataQoS qos;
  qos.keep_last(input_queue_size_);
  if (use_best_effort_) {
    qos.best_effort();
  } else {
    qos.reliable();
  }

  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);
  obstacle_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "obstacle_points", qos);
  ground_plane_pub_=create_publisher<geometry_msgs::msg::PoseStamped>("ground_plane",qos);
  const auto pose_topic=declare_parameter<std::string>("ground_pose_topic","");
  if (level_scan_ && (target_frame_.empty() || pose_topic.empty())) {
    throw std::invalid_argument("Levelled scan requires target_frame and ground_pose_topic");
  }
  if (!pose_topic.empty() && target_frame_.empty()) {
    throw std::invalid_argument("A pose-referenced scan requires a nonempty target_frame");
  }
  if(!pose_topic.empty()) {
    ground_pose_sub_=create_subscription<nav_msgs::msg::Odometry>(pose_topic,rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr pose) {
        ground_poses_.push_back(pose);
        while(ground_poses_.size()>16)ground_poses_.pop_front();
      });
  }

  using std::placeholders::_1;
  sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());

  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty()) {
    tf2_ = std::make_unique<tf2_ros::Buffer>(
      this->get_clock(), tf2::durationFromSec(tf_buffer_duration_));
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    message_filter_ = std::make_unique<MessageFilter>(
      sub_, *tf2_, target_frame_, input_queue_size_,
      this->get_node_logging_interface(),
      this->get_node_clock_interface());
    message_filter_->registerCallback(
      std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
    message_filter_->setTolerance(rclcpp::Duration::from_seconds(tolerance_));
  } else {  // otherwise setup direct subscription
    sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  }
}

void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  // build laserscan output
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;
  if (!target_frame_.empty()) {
    scan_msg->header.frame_id = target_frame_;
  }

  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (observed_free_space_only_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());
  } else if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  // Transform cloud if necessary
  if (scan_msg->header.frame_id != cloud_msg->header.frame_id) {
    RCLCPP_DEBUG(
      this->get_logger(), "Transforming point cloud from %s to %s",
      cloud_msg->header.frame_id.c_str(), scan_msg->header.frame_id.c_str());
    try {
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_));
      cloud_msg = cloud;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
      return;
    }
  }

  const double cloud_stamp = rclcpp::Time(cloud_msg->header.stamp).seconds();
  Eigen::Matrix3d scan_from_body=Eigen::Matrix3d::Identity();
  bool have_scan_attitude=!level_scan_;
  bool have_ground = false;
  bool ground_observation_supported = !ground_filter_enabled_;
  std::vector<Eigen::Vector3d> points;
  GroundPlane candidate,ground;
  if (ground_filter_enabled_) {
    points.reserve(cloud_msg->width);
    for (sensor_msgs::PointCloud2ConstIterator<float> x(*cloud_msg, "x"),
      y(*cloud_msg, "y"), z(*cloud_msg, "z"); x != x.end(); ++x, ++y, ++z)
    {
      points.emplace_back(*x, *y, *z);
    }
    candidate=estimate_ground(points);
    ground = ground_tracker_.observe(candidate,cloud_stamp);
    ground_observation_supported=ground_tracker_.observation_supported(cloud_stamp);
  }
  // Levelling and floor classification are independent. Disabling the floor
  // filter must not silently disable all scan publication.
  if (ground_filter_enabled_ || level_scan_) {
    nav_msgs::msg::Odometry::ConstSharedPtr nearest;
    double best_age=.18;
    for(const auto & pose:ground_poses_) {
      const double age=cloud_stamp-rclcpp::Time(pose->header.stamp).seconds();
      if(age>=-.05 && std::abs(age)<best_age) {nearest=pose;best_age=std::abs(age);}
    }
    if(nearest) {
      try {
        const auto relative=tf2_->lookupTransform(nearest->child_frame_id,
          cloud_msg->header.frame_id,tf2::TimePointZero);
        const auto & oq=nearest->pose.pose.orientation;const auto & ot=nearest->pose.pose.position;
        const auto & rq=relative.transform.rotation;const auto & rt=relative.transform.translation;
        Eigen::Quaterniond world_q(oq.w,oq.x,oq.y,oq.z),relative_q(rq.w,rq.x,rq.y,rq.z);
        if(!world_q.coeffs().allFinite() || std::abs(world_q.norm()-1.)>.01)
          throw std::runtime_error("Invalid floor reference pose");
        const Eigen::Matrix3d rotation=world_q.normalized().toRotationMatrix()*
          relative_q.normalized().toRotationMatrix();
        const Eigen::Vector3d translation=world_q.normalized()*Eigen::Vector3d(rt.x,rt.y,rt.z)+
          Eigen::Vector3d(ot.x,ot.y,ot.z);
        if (level_scan_) {
          scan_from_body=planar_from_body(rotation);
          have_scan_attitude=true;
        }
        if (ground_filter_enabled_) {
          ground=referenced_ground_tracker_.observe(candidate,cloud_stamp,points,rotation,translation);
          ground_observation_supported=referenced_ground_tracker_.supported();
          if (ground_observation_supported && ground.valid) {
            local_world_ground_=transform_ground(ground,rotation,translation);
            ground_reference_origin_=translation;
          } else if (local_world_ground_.valid &&
            (translation-ground_reference_origin_).head<2>().norm()<=.75 &&
            std::abs(translation.z()-ground_reference_origin_.z())<=.15)
          {
            // Bounded odometry-referenced classification through a wall's
            // ground occlusion. Do not confirm/refresh the measured tracker,
            // publish a ground plane, or turn these returns into free rays.
            const auto reference=transform_ground(
              local_world_ground_,rotation.transpose(),-rotation.transpose()*translation);
            const double floor_z=-reference.offset/reference.normal.z();
            if (reference.normal.allFinite() && reference.normal.z()>=.9063 &&
              std::isfinite(floor_z) && floor_z>=-1.4 && floor_z<=-.12)
            {ground=reference;}
          }
        }
      } catch(const std::exception &) {
        // A levelled scan must withhold output without matching attitude.
      }
    }
  }
  if (ground_filter_enabled_) {
    if (ground.valid) {
      ground_ = ground;
      if (ground_observation_supported) {last_ground_stamp_ = cloud_stamp;}
      if (ground_observation_supported) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Adaptive ground: %.3f m below base plane; obstacles above %.2f m",
          -ground.offset / ground.normal.z(), ground_clearance_);
      }
    }
    have_ground = ground.valid;
    if (ground_observation_supported) {
      geometry_msgs::msg::PoseStamped plane;
      plane.header=cloud_msg->header;
      plane.pose.position.z=-ground_.offset/ground_.normal.z();
      const Eigen::Quaterniond q=Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(),ground_.normal);
      plane.pose.orientation.x=q.x();plane.pose.orientation.y=q.y();
      plane.pose.orientation.z=q.z();plane.pose.orientation.w=q.w();
      ground_plane_pub_->publish(plane);
    }
    if (!ground_observation_supported) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "No measured ground: fresh positive-only obstacle observations; clearing withheld");
    }
  }

  if (!have_scan_attitude) {
    RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,
      "Waiting for a matching validated attitude before publishing a levelled navigation scan");
    return;
  }
  if (!scan_frame_.empty())scan_msg->header.frame_id=scan_frame_;

  // Ground returns provide observed-ray evidence without becoming obstacles.
  // No-return angular bins (including the rear HAP blind sector) stay NaN.
  // Iterate through pointcloud
  const bool collect_obstacles = obstacle_points_pub_->get_subscription_count() > 0;
  std::vector<std::array<float, 3>> observed_obstacles;
  if (collect_obstacles) {observed_obstacles.reserve(cloud_msg->width);}
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
    iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for nan in point(%f, %f, %f)\n",
        *iter_x, *iter_y, *iter_z);
      continue;
    }

    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
      continue;
    }
    if (ground_filter_enabled_ && std::abs(*iter_x) < 0.40 && std::abs(*iter_y) < 0.24) {
      continue;  // self returns inside the padded standing footprint
    }
    if (!ground_filter_enabled_ && (*iter_z > max_height_ || *iter_z < min_height_)) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for height %f not in range (%f, %f)\n",
        *iter_z, min_height_, max_height_);
      continue;
    }

    const Eigen::Vector3d scan_point=scan_from_body*Eigen::Vector3d(*iter_x,*iter_y,*iter_z);
    double range = hypot(scan_point.x(),scan_point.y());
    if (range < range_min_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
        range, range_min_, *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
        range, range_max_, *iter_x, *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(scan_point.y(),scan_point.x());
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for angle %f not in range (%f, %f)\n",
        angle, scan_msg->angle_min, scan_msg->angle_max);
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    const auto index = static_cast<std::size_t>(
      (angle - scan_msg->angle_min) / scan_msg->angle_increment);
    if (index < scan_msg->ranges.size()) {
      if (ground_filter_enabled_ && have_ground) {
        const double height = ground_.height(Eigen::Vector3d(*iter_x, *iter_y, *iter_z));
        if (height < ground_clearance_) {
          if (height>=-.08 && ground_observation_supported && observed_free_space_only_ &&
            std::isnan(scan_msg->ranges[index])) {
            scan_msg->ranges[index] = std::numeric_limits<float>::infinity();
          }
          // Deep returns are not positive obstacles and not free-ray proof.
          // The 2.5D layer retains step/drop geometry; do not flatten a lower
          // floor into a persistent wall when the fitted plane fluctuates.
          continue;
        }
        if (height > obstacle_height_) {continue;}
      }
      if (std::isnan(scan_msg->ranges[index]) || range < scan_msg->ranges[index]) {
        scan_msg->ranges[index] = range;
      }
      if (collect_obstacles) {observed_obstacles.push_back({*iter_x, *iter_y, *iter_z});}
    }
  }

  // Persistent geometry must use genuine filtered endpoints, never the
  // artificial angular fill/spread below. Preserve XYZ and the sensor stamp.
  if (collect_obstacles) {
    sensor_msgs::msg::PointCloud2 observations;
    observations.header = cloud_msg->header;
    observations.height = 1;
    observations.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(observations);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    const bool positive_only=ground_filter_enabled_ && !ground_observation_supported;
    if (positive_only) {
      // xyz uses a 16-byte layout; its padding byte can carry this mode flag
      // without changing genuine XYZ, acquisition stamp, or point stride.
      sensor_msgs::msg::PointField flag;
      flag.name="positive_only";flag.offset=12;
      flag.datatype=sensor_msgs::msg::PointField::UINT8;flag.count=1;
      observations.fields.push_back(flag);
    }
    modifier.resize(observed_obstacles.size());
    if (positive_only) {
      for (std::size_t i=0;i<observed_obstacles.size();++i) {
        observations.data[i*observations.point_step+12]=1;
      }
    }
    sensor_msgs::PointCloud2Iterator<float> x(observations, "x");
    sensor_msgs::PointCloud2Iterator<float> y(observations, "y");
    sensor_msgs::PointCloud2Iterator<float> z(observations, "z");
    for (const auto & point : observed_obstacles) {
      *x = point[0];
      *y = point[1];
      *z = point[2];
      ++x;
      ++y;
      ++z;
    }
    obstacle_points_pub_->publish(observations);
  }

  const auto is_obstacle_range = [this](float range) {
      return std::isfinite(range) && range >= range_min_ && range <= range_max_;
    };

  // Bridge only a few adjacent missing rays when both sides are almost the
  // same range.  Linear interpolation retains slanted wall geometry instead of
  // turning it into a circular arc around the robot.
  std::size_t previous_index = scan_msg->ranges.size();
  for (std::size_t index = 0; index < scan_msg->ranges.size(); ++index) {
    if (!is_obstacle_range(scan_msg->ranges[index])) {
      continue;
    }
    if (previous_index < scan_msg->ranges.size()) {
      const auto missing_bins = index - previous_index - 1U;
      if (missing_bins > 0U &&
        missing_bins <= static_cast<std::size_t>(max_fill_gap_bins_) &&
        std::abs(scan_msg->ranges[index] - scan_msg->ranges[previous_index]) <=
        max_fill_range_difference_)
      {
        const float start = scan_msg->ranges[previous_index];
        const float delta = scan_msg->ranges[index] - start;
        const float denominator = static_cast<float>(missing_bins + 1U);
        for (std::size_t offset = 1; offset <= missing_bins; ++offset) {
          scan_msg->ranges[previous_index + offset] =
            start + delta * static_cast<float>(offset) / denominator;
        }
      }
    }
    previous_index = index;
  }

  // Spread each return by a small metric radius.  The angular width shrinks
  // with distance and is capped, preventing a close speckle from closing a
  // large free-space sector while making real obstacle faces continuous.
  const auto dense_ranges = scan_msg->ranges;
  for (std::size_t index = 0; index < dense_ranges.size(); ++index) {
    const float range = dense_ranges[index];
    if (!is_obstacle_range(range) || point_spread_radius_ == 0.0) {
      continue;
    }
    const double half_angle = std::atan2(point_spread_radius_, range);
    const int spread_bins = std::min(
      max_spread_bins_,
      static_cast<int>(std::lround(half_angle / scan_msg->angle_increment)));
    const auto first = index > static_cast<std::size_t>(spread_bins) ?
      index - static_cast<std::size_t>(spread_bins) : 0U;
    const auto last = std::min(
      dense_ranges.size() - 1U, index + static_cast<std::size_t>(spread_bins));
    for (std::size_t target = first; target <= last; ++target) {
      if (!is_obstacle_range(scan_msg->ranges[target]) ||
        range < scan_msg->ranges[target])
      {
        scan_msg->ranges[target] = range;
      }
    }
  }

  pub_->publish(std::move(scan_msg));
}

}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)
