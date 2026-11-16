#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include "go2_lio_navigation/terrain_geometry.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/common/point_tests.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace
{

using Cloud = pcl::PointCloud<pcl::PointXYZI>;

Eigen::Isometry3d transform_to_eigen(const geometry_msgs::msg::Transform & transform)
{
  Eigen::Quaterniond q(
    transform.rotation.w, transform.rotation.x,
    transform.rotation.y, transform.rotation.z);
  if (q.norm() < 1.0e-9) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = q.toRotationMatrix();
  result.translation() = Eigen::Vector3d(
    transform.translation.x, transform.translation.y, transform.translation.z);
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
    const Eigen::Vector3d transformed = transform * Eigen::Vector3d(point.x, point.y, point.z);
    pcl::PointXYZI result;
    result.x = static_cast<float>(transformed.x());
    result.y = static_cast<float>(transformed.y());
    result.z = static_cast<float>(transformed.z());
    result.intensity = point.intensity;
    output->push_back(result);
  }
  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  output->is_dense = false;
  return output;
}

struct Cell
{
  std::size_t count{0};
  double sum{0.0};
  double sum_sq{0.0};
  double minimum{std::numeric_limits<double>::infinity()};
  double maximum{-std::numeric_limits<double>::infinity()};
};

struct BuiltGrid
{
  nav_msgs::msg::OccupancyGrid traversability;
  nav_msgs::msg::OccupancyGrid elevation;
  Cloud surface;
};

nav_msgs::msg::OccupancyGrid crop_grid(const nav_msgs::msg::OccupancyGrid & source,
  const Eigen::Vector3d & center,double length_x,double length_y,double resolution)
{
  auto result=source;
  const int width=std::max(1,static_cast<int>(std::ceil(length_x/resolution-1.e-5)));
  const int height=std::max(1,static_cast<int>(std::ceil(length_y/resolution-1.e-5)));
  const double x=go2_lio_navigation::aligned_grid_origin(center.x(),width,resolution);
  const double y=go2_lio_navigation::aligned_grid_origin(center.y(),height,resolution);
  const int ox=std::lround((x-source.info.origin.position.x)/resolution);
  const int oy=std::lround((y-source.info.origin.position.y)/resolution);
  result.info.width=width;result.info.height=height;
  result.info.origin.position.x=x;result.info.origin.position.y=y;
  result.data.assign(width*height,-1);
  for(int row=0;row<height;++row) {
    if(oy+row<0 || oy+row>=static_cast<int>(source.info.height))continue;
    const int first=std::max(0,-ox),last=std::min(width,static_cast<int>(source.info.width)-ox);
    if(last>first)std::copy_n(source.data.begin()+(oy+row)*source.info.width+ox+first,
      last-first,result.data.begin()+row*width+first);
  }
  return result;
}

}  // namespace

class TraversabilityMapper final : public rclcpp::Node
{
public:
  TraversabilityMapper()
  : Node("traversability_mapper"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    local_cloud_topic_ = declare_parameter<std::string>(
      "local_cloud_topic", "/cloud_registered");
    global_cloud_topic_ = declare_parameter<std::string>(
      "global_cloud_topic", "/loop_closure/global_map");
    local_map_topic_ = declare_parameter<std::string>(
      "local_map_topic", "/traversability/local");
    global_map_topic_ = declare_parameter<std::string>(
      "global_map_topic", "/traversability/global");
    elevation_topic_ = declare_parameter<std::string>(
      "elevation_topic", "/elevation/local");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom_lio");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    localization_valid_topic_ = declare_parameter<std::string>(
      "localization_valid_topic", "/lio/localization_valid");
    use_pose_graph_cloud_ = declare_parameter<bool>("use_pose_graph_cloud", false);
    update_rate_ = std::max(0.2, declare_parameter<double>("update_rate", 3.0));
    history_seconds_ = std::max(0.1, declare_parameter<double>("history_seconds", 3.0));
    max_history_clouds_ = std::max(
      1, static_cast<int>(declare_parameter<int>("max_history_clouds", 30)));
    resolution_ = std::max(0.03, declare_parameter<double>("resolution", 0.10));
    local_length_x_ = declare_parameter<double>("local_length_x", 10.0);
    local_length_y_ = declare_parameter<double>("local_length_y", 10.0);
    global_length_x_ = declare_parameter<double>("global_length_x", 20.0);
    global_length_y_ = declare_parameter<double>("global_length_y", 20.0);
    min_points_per_cell_ = std::max(
      1, static_cast<int>(declare_parameter<int>("min_points_per_cell", 3)));
    global_min_points_per_cell_ = std::max(
      1, static_cast<int>(declare_parameter<int>("global_min_points_per_cell", 1)));
    max_slope_deg_ = declare_parameter<double>("max_slope_deg", 28.0);
    slope_baseline_ = std::max(resolution_, declare_parameter<double>("slope_baseline", 0.24));
    max_step_height_ = declare_parameter<double>("max_step_height", 0.18);
    max_roughness_ = declare_parameter<double>("max_roughness", 0.07);
    obstacle_height_ = declare_parameter<double>("obstacle_height", 0.22);
    min_relative_height_ = declare_parameter<double>("min_relative_height", -0.65);
    max_relative_height_ = declare_parameter<double>("max_relative_height", 1.20);
    lethal_cost_ = std::clamp(
      static_cast<int>(declare_parameter<int>("lethal_cost", 90)), 1, 100);
    unknown_value_ = std::clamp(
      static_cast<int>(declare_parameter<int>("unknown_value", -1)), -1, 100);
    raytrace_free_space_ = declare_parameter<bool>("raytrace_free_space", true);

    auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    local_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(local_map_topic_, map_qos);
    global_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(global_map_topic_, map_qos);
    elevation_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(elevation_topic_, map_qos);
    surface_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/traversability/terrain_surface",map_qos);

    local_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      local_cloud_topic_, rclcpp::SensorDataQoS().keep_last(2),
      std::bind(&TraversabilityMapper::on_local_cloud, this, std::placeholders::_1));
    if (use_pose_graph_cloud_) {
      global_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        global_cloud_topic_, map_qos,
        std::bind(&TraversabilityMapper::on_global_cloud, this, std::placeholders::_1));
    }
    localization_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      localization_valid_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &TraversabilityMapper::on_localization_valid, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / update_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TraversabilityMapper::publish_maps, this));

    RCLCPP_INFO(
      get_logger(), "2.5D traversability: local %.1fx%.1fm, global %.1fx%.1fm, %.2fm/cell",
      local_length_x_, local_length_y_, global_length_x_, global_length_y_, resolution_);
    RCLCPP_INFO(
      get_logger(), "Pose-graph cloud in navigation grid: %s",
      use_pose_graph_cloud_ ? "enabled" : "disabled (rolling recent observations only)");
  }

private:
  struct TimedCloud
  {
    rclcpp::Time stamp;
    Cloud::Ptr cloud;
  };

  Cloud::Ptr sanitize(const sensor_msgs::msg::PointCloud2 & message) const
  {
    auto raw = std::make_shared<Cloud>();
    pcl::fromROSMsg(message, *raw);
    auto sane = std::make_shared<Cloud>();
    sane->reserve(raw->size());
    constexpr float kMaximumSaneCoordinate = 1.0e6F;
    for (const auto & point : *raw) {
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
    return sane;
  }

  bool lookup(
    const std::string & target, const std::string & source,
    const rclcpp::Time & stamp, Eigen::Isometry3d & output)
  {
    if (target == source) {
      output = Eigen::Isometry3d::Identity();
      return true;
    }
    try {
      output = transform_to_eigen(
        tf_buffer_.lookupTransform(target, source, stamp, 100ms).transform);
      return true;
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "TF %s <- %s unavailable: %s",
        target.c_str(), source.c_str(), error.what());
      return false;
    }
  }

  void on_local_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (!localization_valid_.load()) {
      return;
    }
    Eigen::Isometry3d odom_from_cloud;
    const rclcpp::Time stamp(message->header.stamp);
    if (!lookup(odom_frame_, message->header.frame_id, stamp, odom_from_cloud)) {
      return;
    }
    auto cloud = transform_cloud(*sanitize(*message), odom_from_cloud);
    std::lock_guard<std::mutex> lock(mutex_);
    local_history_.push_back({stamp, cloud});
    while (static_cast<int>(local_history_.size()) > max_history_clouds_) {
      local_history_.pop_front();
    }
    while (!local_history_.empty() &&
      (stamp - local_history_.front().stamp).seconds() > history_seconds_)
    {
      local_history_.pop_front();
    }
  }

  void on_global_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (!localization_valid_.load()) {
      return;
    }
    Eigen::Isometry3d map_from_cloud;
    const rclcpp::Time stamp(message->header.stamp);
    if (!lookup(map_frame_, message->header.frame_id, stamp, map_from_cloud)) {
      return;
    }
    auto cloud = transform_cloud(*sanitize(*message), map_from_cloud);
    std::lock_guard<std::mutex> lock(mutex_);
    global_cloud_ = std::move(cloud);
  }

  void on_localization_valid(const std_msgs::msg::Bool::SharedPtr message)
  {
    localization_valid_.store(message->data);
    if (message->data) {
      return;
    }
    nav_msgs::msg::OccupancyGrid empty_local;
    nav_msgs::msg::OccupancyGrid empty_global;
    nav_msgs::msg::OccupancyGrid empty_elevation;
    bool publish_empty = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      local_history_.clear();
      global_cloud_.reset();
      if (have_published_maps_) {
        empty_local = last_local_map_;
        empty_global = last_global_map_;
        empty_elevation = last_elevation_map_;
        const auto stamp = now();
        for (auto * grid : {&empty_local, &empty_global, &empty_elevation}) {
          grid->header.stamp = stamp;
          grid->info.map_load_time = stamp;
          std::fill(grid->data.begin(), grid->data.end(), unknown_value_);
        }
        publish_empty = true;
      }
    }
    // Transient-local publishers must replace the last good terrain map when
    // localization is lost. Otherwise Nav2 and a newly opened RViz keep the
    // drifted obstacle/inflation footprint indefinitely.
    if (publish_empty) {
      local_map_pub_->publish(empty_local);
      global_map_pub_->publish(empty_global);
      elevation_pub_->publish(empty_elevation);
      sensor_msgs::msg::PointCloud2 surface;
      pcl::toROSMsg(Cloud{},surface);
      surface.header=empty_local.header;surface_pub_->publish(surface);
    }
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Localization invalid or stale; cleared terrain history and published unknown maps");
  }

  BuiltGrid build_grid(
    const Cloud & cloud, const std::string & frame, const Eigen::Vector3d & center,
    double length_x, double length_y, int min_points_per_cell,
    const rclcpp::Time & stamp, const Cloud * raytrace_cloud, bool include_surface=false) const
  {
    const int width = std::max(1, static_cast<int>(std::ceil(length_x / resolution_)));
    const int height = std::max(1, static_cast<int>(std::ceil(length_y / resolution_)));
    const double origin_x = go2_lio_navigation::aligned_grid_origin(center.x(), width, resolution_);
    const double origin_y = go2_lio_navigation::aligned_grid_origin(center.y(), height, resolution_);
    std::vector<Cell> cells(static_cast<std::size_t>(width * height));

    for (const auto & point : cloud) {
      if (!pcl::isFinite(point) || point.z < center.z() + min_relative_height_ ||
        point.z > center.z() + max_relative_height_)
      {
        continue;
      }
      const int x = static_cast<int>(std::floor((point.x - origin_x) / resolution_));
      const int y = static_cast<int>(std::floor((point.y - origin_y) / resolution_));
      if (x < 0 || x >= width || y < 0 || y >= height) {
        continue;
      }
      auto & cell = cells[static_cast<std::size_t>(y * width + x)];
      ++cell.count;
      cell.sum += point.z;
      cell.sum_sq += static_cast<double>(point.z) * point.z;
      cell.minimum = std::min(cell.minimum, static_cast<double>(point.z));
      cell.maximum = std::max(cell.maximum, static_cast<double>(point.z));
    }

    BuiltGrid result;
    for (auto * grid : {&result.traversability, &result.elevation}) {
      grid->header.stamp = stamp;
      grid->header.frame_id = frame;
      grid->info.map_load_time = stamp;
      grid->info.resolution = static_cast<float>(resolution_);
      grid->info.width = static_cast<std::uint32_t>(width);
      grid->info.height = static_cast<std::uint32_t>(height);
      grid->info.origin.position.x = origin_x;
      grid->info.origin.position.y = origin_y;
      grid->info.origin.orientation.w = 1.0;
      grid->data.assign(static_cast<std::size_t>(width * height), unknown_value_);
    }

    // Point endpoints alone describe surfaces, not the observed free space in
    // front of them. Raytrace only the recent local observations: using every
    // historical keyframe from the robot's current position could clear a ray
    // through an obstacle that was not visible from this pose.
    if (raytrace_free_space_ && raytrace_cloud != nullptr) {
      const int origin_cell_x = static_cast<int>(
        std::floor((center.x() - origin_x) / resolution_));
      const int origin_cell_y = static_cast<int>(
        std::floor((center.y() - origin_y) / resolution_));
      // Thousands of returns often end in the same 10 cm cell. The projected
      // ray is identical: trace it once instead of repeating Bresenham per point.
      std::vector<bool> ray_seen(static_cast<std::size_t>(width * height), false);
      for (const auto & point : *raytrace_cloud) {
        if (!pcl::isFinite(point)) {
          continue;
        }
        const int endpoint_x = static_cast<int>(
          std::floor((point.x - origin_x) / resolution_));
        const int endpoint_y = static_cast<int>(
          std::floor((point.y - origin_y) / resolution_));
        if (endpoint_x >= 0 && endpoint_x < width && endpoint_y >= 0 && endpoint_y < height) {
          const auto endpoint_index = static_cast<std::size_t>(endpoint_y * width + endpoint_x);
          if (ray_seen[endpoint_index]) {continue;}
          ray_seen[endpoint_index] = true;
        }
        int x = origin_cell_x;
        int y = origin_cell_y;
        const int dx = std::abs(endpoint_x - origin_cell_x);
        const int dy = std::abs(endpoint_y - origin_cell_y);
        const int step_x = origin_cell_x < endpoint_x ? 1 : -1;
        const int step_y = origin_cell_y < endpoint_y ? 1 : -1;
        int error = dx - dy;
        while (x != endpoint_x || y != endpoint_y) {
          if (x < 0 || x >= width || y < 0 || y >= height) {
            break;
          }
          result.traversability.data[static_cast<std::size_t>(y * width + x)] = 0;
          const int doubled_error = 2 * error;
          if (doubled_error > -dy) {
            error -= dy;
            x += step_x;
          }
          if (doubled_error < dx) {
            error += dx;
            y += step_y;
          }
        }
      }
    }

    const int offsets[8][2] = {
      {-1, -1}, {0, -1}, {1, -1}, {-1, 0},
      {1, 0}, {-1, 1}, {0, 1}, {1, 1}};
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const auto index = static_cast<std::size_t>(y * width + x);
        const auto & cell = cells[index];
        if (cell.count < static_cast<std::size_t>(min_points_per_cell)) {
          continue;
        }
        const double mean = cell.sum / static_cast<double>(cell.count);
        const double variance = std::max(
          0.0, cell.sum_sq / static_cast<double>(cell.count) - mean * mean);
        const double roughness = std::sqrt(variance);
        double max_step = 0.0;
        for (const auto & offset : offsets) {
          const int nx = x + offset[0];
          const int ny = y + offset[1];
          if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
            continue;
          }
          const auto & neighbor = cells[static_cast<std::size_t>(ny * width + nx)];
          if (neighbor.count < static_cast<std::size_t>(min_points_per_cell)) {
            continue;
          }
          max_step = std::max(max_step, std::abs(cell.minimum - neighbor.minimum));
        }
        const double slope = go2_lio_navigation::surface_slope(
          cells,width,height,x,y,resolution_,slope_baseline_,min_points_per_cell);
        const double vertical_extent = cell.maximum - cell.minimum;
        double normalized = std::max(
        {
          slope / std::max(1.0, max_slope_deg_),
          max_step / std::max(0.01, max_step_height_),
          roughness / std::max(0.005, max_roughness_)});
        if (vertical_extent >= obstacle_height_) {
          normalized = 1.0;
        }
        const int cost = normalized >= 1.0 ? 100 :
          std::clamp(static_cast<int>(std::lround(normalized * lethal_cost_)), 0, 100);
        result.traversability.data[index] = static_cast<std::int8_t>(cost);
        if (include_surface) {
          pcl::PointXYZI point;
          point.x=origin_x+(x+.5)*resolution_;point.y=origin_y+(y+.5)*resolution_;
          point.z=cell.minimum;point.intensity=cost;result.surface.push_back(point);
        }
        const double relative_height = cell.minimum - center.z();
        const int encoded_height = std::clamp(
          static_cast<int>(std::lround((relative_height + 0.5) * 100.0)), 0, 100);
        result.elevation.data[index] = static_cast<std::int8_t>(encoded_height);
      }
    }
    return result;
  }

  void publish_maps()
  {
    if (!localization_valid_.load()) {
      return;
    }
    Cloud::Ptr local_cloud = std::make_shared<Cloud>();
    Cloud::Ptr global_cloud;
    rclcpp::Time observation_stamp(0,0,get_clock()->get_clock_type());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!local_history_.empty() && local_history_.back().stamp==last_built_stamp_) {
        return;  // A timer retry is not a new observation.
      }
      for (const auto & entry : local_history_) {
        *local_cloud += *entry.cloud;
      }
      global_cloud = global_cloud_;
      if (!local_history_.empty()) {observation_stamp=local_history_.back().stamp;}
    }
    if (local_cloud->empty()) {
      return;
    }
    const double observation_age=(now()-observation_stamp).seconds();
    if (observation_age < -.05 || observation_age > .35) {
      return;  // A fast timer cannot manufacture fresh terrain from an old scan.
    }

    Eigen::Isometry3d odom_from_base;
    Eigen::Isometry3d map_from_odom;
    if (!lookup(odom_frame_, base_frame_, rclcpp::Time(0), odom_from_base)) {return;}
    const auto stamp = observation_stamp;
    BuiltGrid local,global;
    if (!use_pose_graph_cloud_ && global_min_points_per_cell_==min_points_per_cell_ &&
      global_length_x_>=local_length_x_ && global_length_y_>=local_length_y_)
    {
      // Both live maps use the same measured cells. Classify/raytrace once in
      // the continuous odometry frame, then crop. The persistent environment
      // reference remains in map and still follows loop corrections in RViz.
      global=build_grid(*local_cloud,odom_frame_,odom_from_base.translation(),
        global_length_x_,global_length_y_,min_points_per_cell_,stamp,local_cloud.get(),
        surface_pub_->get_subscription_count()>0);
      local.traversability=crop_grid(global.traversability,odom_from_base.translation(),
        local_length_x_,local_length_y_,resolution_);
      local.elevation=crop_grid(global.elevation,odom_from_base.translation(),
        local_length_x_,local_length_y_,resolution_);
      const auto & info=local.traversability.info;
      for(const auto & point:global.surface) {
        if(point.x>=info.origin.position.x && point.y>=info.origin.position.y &&
          point.x<info.origin.position.x+info.width*info.resolution &&
          point.y<info.origin.position.y+info.height*info.resolution)local.surface.push_back(point);
      }
    } else {
      if(!lookup(map_frame_,odom_frame_,rclcpp::Time(0),map_from_odom))return;
      local=build_grid(*local_cloud,odom_frame_,odom_from_base.translation(),
        local_length_x_,local_length_y_,min_points_per_cell_,stamp,local_cloud.get(),
        surface_pub_->get_subscription_count()>0);
      auto map_cloud=std::make_shared<Cloud>();
      if(global_cloud && !global_cloud->empty())*map_cloud+=*global_cloud;
      auto local_in_map=transform_cloud(*local_cloud,map_from_odom);*map_cloud+=*local_in_map;
      global=build_grid(*map_cloud,map_frame_,(map_from_odom*odom_from_base).translation(),
        global_length_x_,global_length_y_,global_min_points_per_cell_,stamp,local_in_map.get());
    }
    local_map_pub_->publish(local.traversability);
    elevation_pub_->publish(local.elevation);
    if (surface_pub_->get_subscription_count()>0) {
      sensor_msgs::msg::PointCloud2 surface;
      pcl::toROSMsg(local.surface,surface);
      surface.header=local.traversability.header;surface_pub_->publish(surface);
    }

    global_map_pub_->publish(global.traversability);
    last_built_stamp_=observation_stamp;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_local_map_ = local.traversability;
      last_global_map_ = global.traversability;
      last_elevation_map_ = local.elevation;
      have_published_maps_ = true;
    }
  }

  std::string local_cloud_topic_;
  std::string global_cloud_topic_;
  std::string local_map_topic_;
  std::string global_map_topic_;
  std::string elevation_topic_;
  std::string odom_frame_;
  std::string map_frame_;
  std::string base_frame_;
  std::string localization_valid_topic_;
  double update_rate_{3.0};
  double history_seconds_{3.0};
  int max_history_clouds_{30};
  double resolution_{0.10};
  double local_length_x_{10.0};
  double local_length_y_{10.0};
  double global_length_x_{20.0};
  double global_length_y_{20.0};
  int min_points_per_cell_{3};
  int global_min_points_per_cell_{1};
  double max_slope_deg_{28.0};
  double max_step_height_{0.18};
  double slope_baseline_{0.24};
  rclcpp::Time last_built_stamp_{0,0,RCL_ROS_TIME};
  double max_roughness_{0.07};
  double obstacle_height_{0.22};
  double min_relative_height_{-0.65};
  double max_relative_height_{1.20};
  int lethal_cost_{90};
  int unknown_value_{-1};
  bool raytrace_free_space_{true};
  bool use_pose_graph_cloud_{false};
  std::atomic<bool> localization_valid_{false};

  std::mutex mutex_;
  std::deque<TimedCloud> local_history_;
  Cloud::Ptr global_cloud_;
  nav_msgs::msg::OccupancyGrid last_local_map_;
  nav_msgs::msg::OccupancyGrid last_global_map_;
  nav_msgs::msg::OccupancyGrid last_elevation_map_;
  bool have_published_maps_{false};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_valid_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr elevation_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surface_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TraversabilityMapper>());
  rclcpp::shutdown();
  return 0;
}
