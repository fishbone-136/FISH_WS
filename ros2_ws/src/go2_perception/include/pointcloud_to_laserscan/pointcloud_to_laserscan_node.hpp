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

#ifndef POINTCLOUD_TO_LASERSCAN__POINTCLOUD_TO_LASERSCAN_NODE_HPP_
#define POINTCLOUD_TO_LASERSCAN__POINTCLOUD_TO_LASERSCAN_NODE_HPP_

#include <memory>
#include <deque>
#include <string>

#include "message_filters/subscriber.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

#include "rclcpp/rclcpp.hpp"
#include "pointcloud_to_laserscan/ground_plane.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "pointcloud_to_laserscan/visibility_control.h"

namespace pointcloud_to_laserscan
{
typedef tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2> MessageFilter;

/**
* Class to process incoming pointclouds into laserscans.
* Some initial code was pulled from the defunct turtlebot pointcloud_to_laserscan implementation.
*/
class PointCloudToLaserScanNode : public rclcpp::Node
{
public:
  POINTCLOUD_TO_LASERSCAN_PUBLIC
  explicit PointCloudToLaserScanNode(const rclcpp::NodeOptions & options);

  ~PointCloudToLaserScanNode() override = default;

private:
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg);
  std::unique_ptr<tf2_ros::Buffer> tf2_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> obstacle_points_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ground_plane_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_pose_sub_;
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> ground_poses_;
  ReferencedGroundTracker referenced_ground_tracker_;
  // An observed local floor may classify positive endpoints while occluded.
  // It never supplies fresh ground/free-space evidence in this mode.
  GroundPlane local_world_ground_;
  Eigen::Vector3d ground_reference_origin_{Eigen::Vector3d::Zero()};
  std::unique_ptr<MessageFilter> message_filter_;

  // ROS Parameters
  int input_queue_size_;
  std::string target_frame_;
  std::string scan_frame_;
  bool level_scan_;
  double tolerance_;
  bool ground_filter_enabled_;
  bool require_ground_for_scan_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_mirror_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_mirror_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ground_mirror_;
  bool observed_free_space_only_;
  double ground_clearance_;
  double obstacle_height_;
  GroundPlane ground_;
  GroundTracker ground_tracker_;
  double last_ground_stamp_{-1.0};
  double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_,
    range_max_;
  double point_spread_radius_, max_fill_range_difference_;
  int max_spread_bins_, max_fill_gap_bins_;
  double tf_buffer_duration_;
  bool use_best_effort_;
  bool use_inf_;
  double inf_epsilon_;
};

}  // namespace pointcloud_to_laserscan

#endif  // POINTCLOUD_TO_LASERSCAN__POINTCLOUD_TO_LASERSCAN_NODE_HPP_
