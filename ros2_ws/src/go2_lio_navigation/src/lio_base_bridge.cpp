#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

namespace
{

Eigen::Isometry3d pose_to_eigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond q(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (q.norm() < 1.0e-9) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = q.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  return transform;
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

Eigen::Matrix3d rpy_matrix(double roll, double pitch, double yaw)
{
  return (
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

double yaw_of(const Eigen::Matrix3d & rotation)
{
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

class LioBaseBridge final : public rclcpp::Node
{
public:
  LioBaseBridge()
  : Node("lio_base_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/Odometry");
    output_topic_ = declare_parameter<std::string>("output_topic", "/odom");
    validated_raw_topic_ = declare_parameter<std::string>(
      "validated_raw_topic", "/Odometry_validated");
    valid_topic_ = declare_parameter<std::string>(
      "valid_topic", "/lio/localization_valid");
    motion_execution_valid_topic_ = declare_parameter<std::string>(
      "motion_execution_valid_topic", "/go2/motion_execution_valid");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom_lio");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    full_body_frame_ = declare_parameter<std::string>("full_body_frame", "");
    if (!full_body_frame_.empty() && full_body_frame_ == base_frame_) {
      throw std::runtime_error("Full body and planar navigation frames must differ");
    }
    planar_output_ = declare_parameter<bool>("planar_output", false);
    twist_filter_alpha_ = std::clamp(
      declare_parameter<double>("twist_filter_alpha", 0.35), 0.0, 1.0);
    max_linear_speed_ = std::max(
      0.1, declare_parameter<double>("max_linear_speed", 2.0));
    max_vertical_speed_ = std::max(
      0.1, declare_parameter<double>("max_vertical_speed", 1.0));
    max_angular_speed_ = std::max(
      0.1, declare_parameter<double>("max_angular_speed", 3.0));
    max_position_radius_ = std::max(
      1.0, declare_parameter<double>("max_position_radius", 100.0));
    max_absolute_z_ = std::max(
      0.5, declare_parameter<double>("max_absolute_z", 5.0));
    input_timeout_ = std::max(
      0.2, declare_parameter<double>("input_timeout", 0.8));
    max_input_age_ = std::max(
      0.05, declare_parameter<double>("max_input_age", 0.30));
    startup_samples_ = std::max(
      1, static_cast<int>(declare_parameter<int>("startup_samples", 30)));
    recovery_samples_ = std::max(
      1, static_cast<int>(declare_parameter<int>("recovery_samples", 3)));
    jump_confirmation_samples_ = std::max(
      1, static_cast<int>(declare_parameter<int>("jump_confirmation_samples", 2)));
    cancel_navigation_on_invalid_ = declare_parameter<bool>(
      "cancel_navigation_on_invalid", true);
    cancel_navigation_on_motion_failure_ = declare_parameter<bool>(
      "cancel_navigation_on_motion_failure", true);
    clear_costmaps_on_invalid_ = declare_parameter<bool>(
      "clear_costmaps_on_invalid", true);

    const double sensor_x = declare_parameter<double>("base_to_sensor_x", 0.14);
    const double sensor_y = declare_parameter<double>("base_to_sensor_y", 0.0);
    const double sensor_z = declare_parameter<double>("base_to_sensor_z", 0.13);
    const double sensor_roll = declare_parameter<double>("base_to_sensor_roll", 0.0);
    const double sensor_pitch = declare_parameter<double>("base_to_sensor_pitch", 0.0);
    const double sensor_yaw = declare_parameter<double>("base_to_sensor_yaw", 0.0);

    Eigen::Isometry3d base_to_sensor = Eigen::Isometry3d::Identity();
    base_to_sensor.translation() = Eigen::Vector3d(sensor_x, sensor_y, sensor_z);
    base_to_sensor.linear() = rpy_matrix(sensor_roll, sensor_pitch, sensor_yaw);
    sensor_to_base_ = base_to_sensor.inverse();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 20);
    validated_raw_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      validated_raw_topic_, rclcpp::QoS(20).reliable());
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      valid_topic_, rclcpp::QoS(1).reliable().transient_local());
    navigation_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, "navigate_to_pose");
    clear_local_costmap_client_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "/local_costmap/clear_entirely_local_costmap");
    clear_global_costmap_client_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "/global_costmap/clear_entirely_global_costmap");
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      // Old queued poses cannot control the robot. Keep only the newest one
      // so a scheduling pause does not replay a backlog of stale timestamps.
      input_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&LioBaseBridge::on_odometry, this, std::placeholders::_1));
    motion_execution_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      motion_execution_valid_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &LioBaseBridge::on_motion_execution_valid, this, std::placeholders::_1));
    health_timer_ = create_wall_timer(100ms, [this]() {check_input_health();});
    publish_valid(false);

    RCLCPP_INFO(
      get_logger(), "LIO base bridge: %s -> %s, validated=%s, TF %s -> %s",
      input_topic_.c_str(), output_topic_.c_str(), validated_raw_topic_.c_str(),
      odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  void on_motion_execution_valid(const std_msgs::msg::Bool::SharedPtr message)
  {
    motion_execution_valid_ = message->data;
    if (!message->data) {
      RCLCPP_ERROR(
        get_logger(),
        "Go2 did not execute cmd_vel; stopping and cancelling the active NavigateToPose goal");
      cancel_navigation_goals(true, "Go2 motion execution failed");
    }
  }

  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr input)
  {
    if (fault_latched_) {
      return;
    }
    const rclcpp::Time stamp(input->header.stamp);
    if (stamp.nanoseconds() == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Ignoring zero-stamped FAST-LIO odometry");
      return;
    }
    const double input_age = (now() - stamp).seconds();
    if (input_age > max_input_age_ || input_age < -0.1) {
      consecutive_good_samples_ = 0;
      if (input_age > max_input_age_ && has_been_valid_ &&
        (now() - last_input_time_).seconds() <= input_timeout_)
      {
        // Close the physical motion gate immediately, but preserve the goal
        // during a bounded scheduling pause. Never forward/restamp this pose.
        freshness_hold_ = true;
        publish_valid(false, true);
      } else {
        publish_valid(false);
        cancel_navigation_goals(false, "stale or future localization timestamp");
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected FAST-LIO timestamp age %.3f s (limit %.3f s)", input_age, max_input_age_);
      return;
    }
    if (last_stamp_.nanoseconds() != 0 && stamp <= last_stamp_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Ignoring non-monotonic FAST-LIO odometry");
      return;
    }

    const auto & position = input->pose.pose.position;
    const auto & orientation = input->pose.pose.orientation;
    const double quaternion_norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w) || quaternion_norm < 1.0e-6)
    {
      latch_fault("non-finite pose or invalid quaternion");
      return;
    }

    const Eigen::Isometry3d odom_to_sensor = pose_to_eigen(input->pose.pose);
    Eigen::Isometry3d odom_to_base = odom_to_sensor * sensor_to_base_;
    const double xy_radius = std::hypot(
      odom_to_base.translation().x(), odom_to_base.translation().y());
    if (!odom_to_base.matrix().allFinite() || xy_radius > max_position_radius_ ||
      std::abs(odom_to_base.translation().z()) > max_absolute_z_)
    {
      latch_fault("pose exceeded the configured operating envelope");
      return;
    }

    // FAST-LIO can make a short gravity/height correction immediately after
    // building its first ikd-tree. Do not apply derivative limits until the
    // output gate has seen a run of stable samples; the source-side guard still
    // rejects non-finite, high-velocity, and out-of-envelope states here.
    // A freshness failure closes the output gate, but must never erase the
    // continuity guard. Otherwise the first recovered sample could teleport
    // the robot (and all remembered obstacles) before checks resume.
    if (has_been_valid_ && last_stamp_.nanoseconds() != 0) {
      const double dt = (stamp - last_stamp_).seconds();
      if (dt > 1.0e-4) {
        const Eigen::Isometry3d delta = last_odom_to_base_.inverse() * odom_to_base;
        const double planar_speed = std::hypot(
          delta.translation().x(), delta.translation().y()) / dt;
        const double vertical_speed = std::abs(delta.translation().z()) / dt;
        const double angular_speed = std::abs(
          normalize_angle(yaw_of(delta.linear()))) / dt;
        // last_odom_to_base_ is deliberately flattened after acceptance when
        // planar_output is enabled. Comparing the next raw z against that
        // flattened z fabricated a ~3.5 m/s vertical jump on every other HAP
        // frame and halved the odometry rate. The 2-D navigation output does
        // not consume z; retain the absolute-z envelope above, and only apply
        // the derivative-z guard when publishing full 6-DoF odometry.
        if (planar_speed > max_linear_speed_ ||
          (!planar_output_ && vertical_speed > max_vertical_speed_) ||
          angular_speed > max_angular_speed_)
        {
          ++consecutive_jump_samples_;
          RCLCPP_WARN(
            get_logger(),
            "Rejected suspected FAST-LIO jump (%d/%d): planar=%.2f m/s "
            "vertical=%.2f m/s yaw=%.2f rad/s",
            consecutive_jump_samples_, jump_confirmation_samples_,
            planar_speed, vertical_speed, angular_speed);
          // Do not advance last_stamp_/last_odom_to_base_. A single gait or
          // scheduling spike is dropped, and the next sample is compared with
          // the last accepted pose over the longer elapsed interval. A real
          // estimator divergence remains outside the envelope and is latched
          // on consecutive rejected samples without ever reaching Nav2.
          if (consecutive_jump_samples_ >= jump_confirmation_samples_) {
            latch_fault("consecutive implausible FAST-LIO pose jumps");
          }
          return;
        }
      }
    }
    consecutive_jump_samples_ = 0;
    const Eigen::Isometry3d full_odom_to_base = odom_to_base;
    if (planar_output_) {
      const double yaw = yaw_of(odom_to_base.linear());
      odom_to_base.translation().z() = 0.0;
      odom_to_base.linear() = rpy_matrix(0.0, 0.0, yaw);
    }

    geometry_msgs::msg::Twist twist;
    if (last_stamp_.nanoseconds() != 0) {
      const double dt = (stamp - last_stamp_).seconds();
      if (dt > 1.0e-4 && dt < 1.0) {
        const Eigen::Isometry3d delta = last_odom_to_base_.inverse() * odom_to_base;
        geometry_msgs::msg::Twist instantaneous;
        instantaneous.linear.x = delta.translation().x() / dt;
        instantaneous.linear.y = delta.translation().y() / dt;
        instantaneous.linear.z = delta.translation().z() / dt;
        instantaneous.angular.z = normalize_angle(yaw_of(delta.linear())) / dt;
        twist.linear.x = filter(instantaneous.linear.x, last_twist_.linear.x);
        twist.linear.y = filter(instantaneous.linear.y, last_twist_.linear.y);
        twist.linear.z = filter(instantaneous.linear.z, last_twist_.linear.z);
        twist.angular.z = filter(instantaneous.angular.z, last_twist_.angular.z);
      }
    }

    const bool quick_freshness_recovery = freshness_hold_ &&
      last_input_time_.nanoseconds() != 0 &&
      (now() - last_input_time_).seconds() <= input_timeout_;
    last_input_time_ = now();
    last_stamp_ = stamp;
    last_odom_to_base_ = odom_to_base;
    last_twist_ = twist;
    ++consecutive_good_samples_;
    // Every pose/velocity/jump check above still applies. A fresh, continuous
    // pose can release a timestamp-only hold without ten more silent frames.
    const int required_samples = quick_freshness_recovery ? 1 :
      (has_been_valid_ ? recovery_samples_ : startup_samples_);
    if (consecutive_good_samples_ < required_samples) {
      return;
    }
    freshness_hold_ = false;
    publish_valid(true);
    validated_raw_pub_->publish(*input);

    nav_msgs::msg::Odometry output;
    output.header.stamp = input->header.stamp;
    output.header.frame_id = odom_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose = eigen_to_pose(odom_to_base);
    output.pose.covariance = input->pose.covariance;
    output.twist.twist = twist;
    output.twist.covariance = input->twist.covariance;
    odom_pub_->publish(output);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = output.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = odom_to_base.translation().x();
    transform.transform.translation.y = odom_to_base.translation().y();
    transform.transform.translation.z = odom_to_base.translation().z();
    const Eigen::Quaterniond q(odom_to_base.linear());
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transform);
    if (!full_body_frame_.empty()) {
      auto physical = transform;
      physical.child_frame_id = full_body_frame_;
      const auto physical_pose = eigen_to_pose(full_odom_to_base);
      physical.transform.translation.x = physical_pose.position.x;
      physical.transform.translation.y = physical_pose.position.y;
      physical.transform.translation.z = physical_pose.position.z;
      physical.transform.rotation = physical_pose.orientation;
      tf_broadcaster_->sendTransform(physical);
    }

  }

  void check_input_health()
  {
    if (!motion_execution_valid_) {
      // Retry at a bounded rate if the first feedback sample arrived before
      // the NavigateToPose action server was ready.
      cancel_navigation_goals(false, "Go2 motion execution failed");
    }
    if (fault_latched_) {
      publish_valid(false);
      cancel_navigation_goals(false, "localization safety fault");
      return;
    }
    if (last_input_time_.nanoseconds() == 0 ||
      (now() - last_input_time_).seconds() > input_timeout_)
    {
      if (localization_valid_) {
        RCLCPP_ERROR(
          get_logger(), "FAST-LIO odometry timed out for more than %.2f s", input_timeout_);
      }
      consecutive_good_samples_ = 0;
      publish_valid(false);
      cancel_navigation_goals(false, "localization input timed out");
    }
  }

  void latch_fault(const char * reason)
  {
    fault_latched_ = true;
    consecutive_good_samples_ = 0;
    publish_valid(false);
    RCLCPP_FATAL(
      get_logger(), "Localization safety fault latched: %s; restart FAST-LIO after checking the sensor",
      reason);
  }

  void publish_valid(bool valid, bool transient_timestamp_hold = false)
  {
    const bool escalate_hold = !valid && !transient_timestamp_hold && freshness_hold_;
    if (escalate_hold) {freshness_hold_ = false;}
    if (valid_status_published_ && valid == localization_valid_) {
      if (escalate_hold) {stop_navigation_after_localization_loss();}
      return;
    }
    const bool became_invalid = valid_status_published_ && localization_valid_ && !valid;
    localization_valid_ = valid;
    if (valid) {
      has_been_valid_ = true;
    }
    valid_status_published_ = true;
    std_msgs::msg::Bool message;
    message.data = valid;
    valid_pub_->publish(message);
    if ((became_invalid && !transient_timestamp_hold) || escalate_hold) {
      stop_navigation_after_localization_loss();
    }
  }

  void stop_navigation_after_localization_loss()
  {
    // Closing go2_base's velocity gate prevents physical motion. Cancelling
    // the action as well prevents Nav2 from ticking a stale path indefinitely
    // after TF and /scan stop, which previously consumed a full CPU core and
    // left the old obstacle inflation visible in RViz.
    cancel_navigation_goals(true, "localization became invalid");
    if (!clear_costmaps_on_invalid_) {
      return;
    }
    auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    bool requested_clear = false;
    if (clear_local_costmap_client_->service_is_ready()) {
      clear_local_costmap_client_->async_send_request(request);
      requested_clear = true;
    }
    if (clear_global_costmap_client_->service_is_ready()) {
      clear_global_costmap_client_->async_send_request(request);
      requested_clear = true;
    }
    if (requested_clear) {
      RCLCPP_ERROR(get_logger(), "Localization lost; requested local/global costmap clear");
    }
  }

  void cancel_navigation_goals(bool force, const char * reason)
  {
    const bool allowed =
      (motion_execution_valid_ && cancel_navigation_on_invalid_) ||
      (!motion_execution_valid_ && cancel_navigation_on_motion_failure_);
    if (!allowed || !navigation_client_->action_server_is_ready()) {
      return;
    }
    const auto current_time = now();
    if (!force && last_cancel_request_.nanoseconds() != 0 &&
      (current_time - last_cancel_request_).seconds() < 1.0)
    {
      return;
    }
    navigation_client_->async_cancel_all_goals();
    last_cancel_request_ = current_time;
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "%s; cancelling all NavigateToPose goals", reason);
  }

  double filter(double current, double previous) const
  {
    return twist_filter_alpha_ * current + (1.0 - twist_filter_alpha_) * previous;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string validated_raw_topic_;
  std::string valid_topic_;
  std::string motion_execution_valid_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string full_body_frame_;
  bool planar_output_{false};
  double twist_filter_alpha_{0.35};
  double max_linear_speed_{2.0};
  double max_vertical_speed_{1.0};
  double max_angular_speed_{3.0};
  double max_position_radius_{100.0};
  double max_absolute_z_{5.0};
  double input_timeout_{0.8};
  double max_input_age_{0.30};
  int startup_samples_{30};
  int recovery_samples_{3};
  int jump_confirmation_samples_{2};
  bool cancel_navigation_on_invalid_{true};
  bool cancel_navigation_on_motion_failure_{true};
  bool clear_costmaps_on_invalid_{true};
  int consecutive_good_samples_{0};
  int consecutive_jump_samples_{0};
  bool localization_valid_{false};
  bool valid_status_published_{false};
  bool has_been_valid_{false};
  bool freshness_hold_{false};
  bool fault_latched_{false};
  bool motion_execution_valid_{true};
  Eigen::Isometry3d sensor_to_base_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d last_odom_to_base_{Eigen::Isometry3d::Identity()};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_input_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cancel_request_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Twist last_twist_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_execution_valid_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr validated_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_local_costmap_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_costmap_client_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr health_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioBaseBridge>());
  rclcpp::shutdown();
  return 0;
}
