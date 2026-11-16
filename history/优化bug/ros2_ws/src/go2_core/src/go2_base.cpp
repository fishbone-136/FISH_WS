#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "common/ros2_sport_client.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

using namespace std::chrono_literals;

class Go2Base : public rclcpp::Node
{
public:
  Go2Base()
  : Node("go2_base")
  {
    odom_source_ = declare_parameter<std::string>("odom_source", "sport_mode_state");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/utlidar/robot_pose");
    sport_state_topic_ = declare_parameter<std::string>(
      "sport_state_topic", "/lf/sportmodestate");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    interactive_command_topic_ = declare_parameter<std::string>(
      "interactive_command_topic", "/go2/interactive_command");
    interactive_status_topic_ = declare_parameter<std::string>(
      "interactive_status_topic", "/go2/interactive_status");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    cmd_vel_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
    localization_guard_enabled_ = declare_parameter<bool>(
      "localization_guard_enabled", false);
    localization_valid_topic_ = declare_parameter<std::string>(
      "localization_valid_topic", "/lio/localization_valid");
    cmd_vel_deadband_ = std::max(
      0.0, declare_parameter<double>("cmd_vel_deadband", 0.005));
    max_linear_command_ = std::max(
      0.0, declare_parameter<double>("max_linear_command", 0.70));
    max_lateral_command_ = std::max(
      0.0, declare_parameter<double>("max_lateral_command", 0.30));
    max_angular_command_ = std::max(
      0.0, declare_parameter<double>("max_angular_command", 1.00));
    min_pure_rotation_command_ = std::clamp(
      declare_parameter<double>("min_pure_rotation_command", 0.0),
      0.0, max_angular_command_);
    planar_odometry_ = declare_parameter<bool>("planar_odometry", true);
    max_pose_speed_ = declare_parameter<double>("max_pose_speed", 1.8);
    max_pose_angular_speed_ =
      declare_parameter<double>("max_pose_angular_speed", 2.5);
    pose_jump_check_window_ =
      declare_parameter<double>("pose_jump_check_window", 1.0);
    twist_filter_alpha_ = std::clamp(
      declare_parameter<double>("twist_filter_alpha", 0.35), 0.0, 1.0);
    state_stationary_linear_threshold_ = std::max(
      0.0, declare_parameter<double>("state_stationary_linear_threshold", 0.04));
    state_stationary_angular_threshold_ = std::max(
      0.0, declare_parameter<double>("state_stationary_angular_threshold", 0.06));
    state_stationary_position_delta_ = std::max(
      0.0, declare_parameter<double>("state_stationary_position_delta", 0.0035));
    state_stationary_yaw_delta_ = std::max(
      0.0, declare_parameter<double>("state_stationary_yaw_delta", 0.0045));
    motion_feedback_topic_ = declare_parameter<std::string>(
      "motion_feedback_topic", "/go2/motion_execution_valid");
    motion_feedback_timeout_ = std::max(
      1.0, declare_parameter<double>("motion_feedback_timeout", 3.0));
    motion_feedback_stale_timeout_ = std::max(
      0.2, declare_parameter<double>("motion_feedback_stale_timeout", 1.0));
    motion_feedback_command_linear_ = std::max(
      0.0, declare_parameter<double>("motion_feedback_command_linear", 0.15));
    motion_feedback_command_angular_ = std::max(
      0.0, declare_parameter<double>("motion_feedback_command_angular", 0.35));
    motion_feedback_min_linear_ = std::max(
      0.0, declare_parameter<double>("motion_feedback_min_linear", 0.03));
    motion_feedback_min_angular_ = std::max(
      0.0, declare_parameter<double>("motion_feedback_min_angular", 0.05));

    // 初始化运动客户端
    sport_client_ = std::make_shared<SportClient>();

    // 创建TF广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (odom_source_ != "sport_mode_state" && odom_source_ != "pose" &&
      odom_source_ != "none")
    {
      throw std::invalid_argument(
              "odom_source must be 'sport_mode_state', 'pose', or 'none'");
    }

    // Always observe the robot's own state, even when FAST-LIO supplies Nav2
    // odometry. This distinguishes "Nav2 never commanded" from "Move RPCs
    // were sent but the sport controller did not execute them".
    sport_state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
      sport_state_topic_, rclcpp::SensorDataQoS(),
      [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
        observeSportState(msg);
        if (odom_source_ == "sport_mode_state") {
          handleSportState(msg);
        }
      });
    if (odom_source_ == "pose") {
      pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          handlePose(msg);
        });
    }

    // 订阅速度命令
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(1).reliable(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        handleVelocity(msg);
      });

    if (localization_guard_enabled_) {
      localization_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
        localization_valid_topic_, rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          handleLocalizationHealth(msg->data);
        });
    }

    // 语音/手势/视觉使用单独的带租约入口。go2_base 在租约内忽略 Nav2 的
    // cmd_vel（尤其是 velocity_smoother 连续发布的零速度），从而避免双控制源
    // 把交互动作立刻覆盖。
    interactive_status_pub_ = create_publisher<std_msgs::msg::String>(
      interactive_status_topic_, 10);
    interactive_command_sub_ = create_subscription<std_msgs::msg::String>(
      interactive_command_topic_, rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        handleInteractiveCommand(msg);
      });

    // In FAST-LIO command-only mode no odometry is produced here. Avoid
    // advertising a second, silent /odom publisher that makes duplicate-TF
    // diagnosis ambiguous.
    if (odom_source_ != "none") {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    }
    motion_feedback_pub_ = create_publisher<std_msgs::msg::Bool>(
      motion_feedback_topic_, rclcpp::QoS(1).reliable().transient_local());
    publishMotionExecutionValid(true, true);

    // 运动指令发布
    sport_pub_ = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

    // A Unitree Move RPC remains active until another motion RPC replaces it.
    // Clear a command left behind by an abruptly terminated previous process as
    // soon as the robot-side DDS reader is discovered.  Do not publish before
    // discovery: the robot reader is best-effort and would not receive history.
    startup_stop_timer_ = create_wall_timer(100ms, [this]() {
      if (sport_pub_->get_subscription_count() == 0U) {
        return;
      }
      stopRobot();
      startup_stop_timer_->cancel();
      RCLCPP_INFO(get_logger(), "Go2 sport endpoint matched; startup StopMove sent");
    });

    // Handle SIGINT/SIGTERM shutdown as well.  SIGKILL cannot be intercepted,
    // therefore the startup stop above remains necessary on the next launch.
    rclcpp::on_shutdown([this]() {
      if (shutdown_stop_sent_.exchange(true, std::memory_order_relaxed)) {
        return;
      }
      try {
        stopRobot();
        // Give CycloneDDS a short opportunity to hand the sample to UDP before
        // the node and its writer are destroyed.
        std::this_thread::sleep_for(100ms);
      } catch (const std::exception &) {
        // Shutdown must continue even if the DDS writer is already unavailable.
      }
    });

    // Nav2 或上位机异常退出时，确保速度命令不会无限保持。
    watchdog_timer_ = create_wall_timer(100ms, [this]() {checkVelocityTimeout();});

    const std::string active_odom_topic = odom_source_ == "sport_mode_state" ?
      sport_state_topic_ : (odom_source_ == "pose" ? pose_topic_ : "disabled");
    RCLCPP_INFO(
      get_logger(),
      "Go2 base bridge ready: odom_source=%s input=%s cmd_vel=%s "
      "interactive=%s odom=%s planar=%s timeout=%.2fs",
      odom_source_.c_str(), active_odom_topic.c_str(), cmd_vel_topic_.c_str(),
      interactive_command_topic_.c_str(), odom_topic_.c_str(),
      planar_odometry_ ? "true" : "false", cmd_vel_timeout_);
    if (localization_guard_enabled_) {
      RCLCPP_INFO(
        get_logger(), "Navigation motion is gated by %s",
        localization_valid_topic_.c_str());
    }
    RCLCPP_INFO(
      get_logger(), "Base execution feedback: %s, no-motion timeout %.1fs",
      motion_feedback_topic_.c_str(), motion_feedback_timeout_);
  }

private:
  void observeSportState(const unitree_go::msg::SportModeState::SharedPtr msg)
  {
    const double linear_speed = std::hypot(msg->velocity[0], msg->velocity[1]);
    const double angular_speed = std::abs(msg->yaw_speed);
    if (!std::isfinite(linear_speed) || !std::isfinite(angular_speed)) {
      return;
    }
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    last_motion_feedback_time_ = now();
    measured_sport_linear_speed_ = linear_speed;
    measured_sport_angular_speed_ = angular_speed;
    sport_error_code_ = msg->error_code;
    sport_mode_ = msg->mode;
    sport_gait_type_ = msg->gait_type;
  }

  void publishMotionExecutionValid(bool valid, bool force = false)
  {
    const bool previous = motion_execution_valid_.exchange(valid);
    if (!force && previous == valid) {
      return;
    }
    std_msgs::msg::Bool message;
    message.data = valid;
    motion_feedback_pub_->publish(message);
  }

  // A locally matched RPC reader is not evidence of fresh robot feedback.
  // Stop both command sources if state disappears, and require fresh state
  // plus an idle reset before admitting another navigation command.
  bool rejectMotionWithoutFeedback()
  {
    bool send_stop = false;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      if (last_motion_feedback_time_.nanoseconds() != 0 &&
        (now() - last_motion_feedback_time_).seconds() <= motion_feedback_stale_timeout_)
      {
        return false;
      }
      send_stop = !motion_execution_fault_latched_.exchange(true);
      command_active_ = interactive_active_ = false;
      watchdog_stopped_ = stop_sent_for_zero_ = true;
      motion_command_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    publishMotionExecutionValid(false);
    if (send_stop) {
      stopRobot();
    }
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "Rejecting motion: Go2 state feedback is missing or stale");
    return true;
  }

  static double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void handleSportState(const unitree_go::msg::SportModeState::SharedPtr msg)
  {
    const double raw_x = msg->position[0];
    const double raw_y = msg->position[1];
    const double raw_roll = msg->imu_state.rpy[0];
    const double raw_pitch = msg->imu_state.rpy[1];
    const double raw_yaw = msg->imu_state.rpy[2];
    const double linear_speed = std::hypot(msg->velocity[0], msg->velocity[1]);
    const double angular_speed = std::abs(msg->yaw_speed);
    if (!std::isfinite(raw_x) || !std::isfinite(raw_y) ||
      !std::isfinite(raw_roll) || !std::isfinite(raw_pitch) ||
      !std::isfinite(raw_yaw) || !std::isfinite(linear_speed) ||
      !std::isfinite(angular_speed))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring invalid Go2 SportModeState odometry");
      return;
    }

    const auto stamp = now();
    if (!sport_state_initialized_) {
      sport_frame_yaw_ = -raw_yaw;
      previous_sport_raw_x_ = raw_x;
      previous_sport_raw_y_ = raw_y;
      previous_sport_raw_yaw_ = raw_yaw;
      last_sport_state_time_ = stamp;
      sport_state_initialized_ = true;
    } else {
      const double dt = (stamp - last_sport_state_time_).seconds();
      const double raw_dx = raw_x - previous_sport_raw_x_;
      const double raw_dy = raw_y - previous_sport_raw_y_;
      const double raw_dyaw = normalizeAngle(raw_yaw - previous_sport_raw_yaw_);
      const double c = std::cos(sport_frame_yaw_);
      const double s = std::sin(sport_frame_yaw_);
      const double dx = c * raw_dx - s * raw_dy;
      const double dy = s * raw_dx + c * raw_dy;
      const double position_delta = std::hypot(dx, dy);
      const bool discontinuous =
        dt > 1e-3 && dt <= pose_jump_check_window_ &&
        (position_delta / dt > max_pose_speed_ ||
        std::abs(raw_dyaw) / dt > max_pose_angular_speed_);
      const bool stationary =
        linear_speed <= state_stationary_linear_threshold_ &&
        angular_speed <= state_stationary_angular_threshold_ &&
        position_delta <= state_stationary_position_delta_ &&
        std::abs(raw_dyaw) <= state_stationary_yaw_delta_;

      if (discontinuous) {
        // The onboard estimator can occasionally reset its origin.  Consume
        // the new raw reference while preserving continuous navigation odom.
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Rejecting discontinuous SportModeState odometry: linear=%.2fm/s "
          "angular=%.2frad/s",
          position_delta / dt, std::abs(raw_dyaw) / dt);
      } else if (!stationary) {
        sport_odom_x_ += dx;
        sport_odom_y_ += dy;
        sport_odom_yaw_ = normalizeAngle(sport_odom_yaw_ + raw_dyaw);
      }

      previous_sport_raw_x_ = raw_x;
      previous_sport_raw_y_ = raw_y;
      previous_sport_raw_yaw_ = raw_yaw;
      last_sport_state_time_ = stamp;
    }

    auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose->header.stamp = stamp;
    pose->header.frame_id = odom_frame_;
    pose->pose.position.x = sport_odom_x_;
    pose->pose.position.y = sport_odom_y_;
    pose->pose.position.z = 0.0;
    tf2::Quaternion orientation;
    orientation.setRPY(
      planar_odometry_ ? 0.0 : raw_roll,
      planar_odometry_ ? 0.0 : raw_pitch,
      sport_odom_yaw_);
    pose->pose.orientation.x = orientation.x();
    pose->pose.orientation.y = orientation.y();
    pose->pose.orientation.z = orientation.z();
    pose->pose.orientation.w = orientation.w();
    handlePose(pose);
  }

  // 处理位姿消息
  void handlePose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto & p = msg->pose.position;
    const auto & q = msg->pose.orientation;
    const bool finite_pose =
      std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
      std::isfinite(q.x) && std::isfinite(q.y) &&
      std::isfinite(q.z) && std::isfinite(q.w);
    const double quaternion_norm =
      std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!finite_pose || quaternion_norm < 1e-6) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring invalid Unitree pose (non-finite value or zero quaternion)");
      return;
    }

    // Legacy pose fallback preserves the producer timestamp. SportModeState
    // is converted above with the local reception time before entering here.
    const auto reception_stamp = now();
    rclcpp::Time pose_stamp(msg->header.stamp, get_clock()->get_clock_type());
    if (pose_stamp.nanoseconds() == 0) {
      pose_stamp = reception_stamp;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unitree pose has a zero timestamp; falling back to reception time");
    }

    const std::int64_t stamp_ns = pose_stamp.nanoseconds();
    if (has_last_pose_ && stamp_ns <= last_pose_stamp_ns_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring non-monotonic Unitree pose timestamp");
      return;
    }

    tf2::Quaternion source_orientation(q.x, q.y, q.z, q.w);
    source_orientation.normalize();
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(source_orientation).getRPY(roll, pitch, yaw);
    if (!std::isfinite(yaw)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring Unitree pose with invalid yaw");
      return;
    }

    double dt = 0.0;
    if (has_last_pose_) {
      dt = static_cast<double>(stamp_ns - last_pose_stamp_ns_) * 1e-9;
      const double linear_speed =
        std::hypot(p.x - last_pose_x_, p.y - last_pose_y_) / dt;
      const double angular_speed =
        std::abs(normalizeAngle(yaw - last_pose_yaw_)) / dt;
      if (dt <= pose_jump_check_window_ &&
        (linear_speed > max_pose_speed_ ||
        angular_speed > max_pose_angular_speed_))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Rejecting discontinuous pose odometry: linear=%.2fm/s angular=%.2frad/s",
          linear_speed, angular_speed);
        return;
      }
    }

    tf2::Quaternion output_orientation = source_orientation;
    if (planar_odometry_) {
      // Nav2 和 slam_toolbox 都是 2D。Go2 行走时的俯仰、横滚和机身
      // 高度起伏若直接进入 base_link，会把静止墙面投影成周期性移动的扫描。
      output_orientation.setRPY(0.0, 0.0, yaw);
    }

    // 创建TF变换
    geometry_msgs::msg::TransformStamped t;

    // 设置变换的时间戳和坐标系
    t.header.stamp = pose_stamp;
    t.header.frame_id = odom_frame_;
    t.child_frame_id = base_frame_;

    // 设置位置
    t.transform.translation.x = msg->pose.position.x;
    t.transform.translation.y = msg->pose.position.y;
    t.transform.translation.z = planar_odometry_ ? 0.0 : msg->pose.position.z;

    // 设置旋转
    t.transform.rotation.x = output_orientation.x();
    t.transform.rotation.y = output_orientation.y();
    t.transform.rotation.z = output_orientation.z();
    t.transform.rotation.w = output_orientation.w();

    geometry_msgs::msg::Twist measured_twist;
    if (has_last_pose_ && dt > 1e-3 && dt <= pose_jump_check_window_) {
      const double velocity_odom_x = (p.x - last_pose_x_) / dt;
      const double velocity_odom_y = (p.y - last_pose_y_) / dt;
      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      geometry_msgs::msg::Twist instantaneous;
      // nav_msgs/Odometry 的 twist 按 child_frame_id 表达。
      instantaneous.linear.x =
        cos_yaw * velocity_odom_x + sin_yaw * velocity_odom_y;
      instantaneous.linear.y =
        -sin_yaw * velocity_odom_x + cos_yaw * velocity_odom_y;
      instantaneous.angular.z = normalizeAngle(yaw - last_pose_yaw_) / dt;
      measured_twist.linear.x =
        twist_filter_alpha_ * instantaneous.linear.x +
        (1.0 - twist_filter_alpha_) * last_measured_twist_.linear.x;
      measured_twist.linear.y =
        twist_filter_alpha_ * instantaneous.linear.y +
        (1.0 - twist_filter_alpha_) * last_measured_twist_.linear.y;
      measured_twist.angular.z =
        twist_filter_alpha_ * instantaneous.angular.z +
        (1.0 - twist_filter_alpha_) * last_measured_twist_.angular.z;
    }

    // 发布变换
    tf_broadcaster_->sendTransform(t);

    // 构造并发布Odometry消息
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = t.header.stamp;
    odom_msg.header.frame_id = t.header.frame_id;
    odom_msg.child_frame_id = t.child_frame_id;
    odom_msg.pose.pose = msg->pose;
    odom_msg.pose.pose.position.z = t.transform.translation.z;
    odom_msg.pose.pose.orientation = t.transform.rotation;
    // 发布由位姿差分得到的实测速度，而不是控制命令。控制器据此判断真实
    // 停止/转弯状态，可减少过冲和到点误判。
    odom_msg.twist.twist = measured_twist;
    odom_pub_->publish(odom_msg);

    last_pose_stamp_ns_ = stamp_ns;
    last_pose_x_ = p.x;
    last_pose_y_ = p.y;
    last_pose_yaw_ = yaw;
    last_measured_twist_ = measured_twist;
    has_last_pose_ = true;
  }

  // 处理速度命令
  void handleVelocity(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      if (interactive_active_) {
        // 高优先级交互租约由 watchdog 续发和结束；期间忽略 Nav2 的速度，
        // 防止另一个终端里的 velocity_smoother 用零速度覆盖语音指令。
        return;
      }
    }
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
      !std::isfinite(msg->angular.z))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting non-finite cmd_vel and stopping the robot");
      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        received_cmd_ = true;
        command_active_ = false;
        stop_sent_for_zero_ = true;
        watchdog_stopped_ = true;
        last_cmd_time_ = now();
      }
      stopRobot();
      return;
    }

    const double vx = std::clamp(
      msg->linear.x, -max_linear_command_, max_linear_command_);
    const double vy = std::clamp(
      msg->linear.y, -max_lateral_command_, max_lateral_command_);
    double wz = std::clamp(
      msg->angular.z, -max_angular_command_, max_angular_command_);
    // Legacy opt-in compensation. Disabled by default: gait limits belong in
    // the collision-checked controller, before velocity smoothing. Amplifying
    // its output here defeats braking and can cause repeated overshoots.
    const bool pure_rotation =
      std::hypot(vx, vy) <= cmd_vel_deadband_ &&
      std::abs(wz) >= motion_feedback_command_angular_;
    if (pure_rotation && std::abs(wz) < min_pure_rotation_command_) {
      wz = std::copysign(min_pure_rotation_command_, wz);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Raising pure-yaw command to Go2 gait threshold: wz=%.3f rad/s", wz);
    }
    const bool moving =
      std::abs(vx) > cmd_vel_deadband_ ||
      std::abs(vy) > cmd_vel_deadband_ ||
      std::abs(wz) > cmd_vel_deadband_;
    const bool meaningful_motion =
      std::hypot(vx, vy) >= motion_feedback_command_linear_ ||
      std::abs(wz) >= motion_feedback_command_angular_;
    if (moving && rejectMotionWithoutFeedback()) {
      return;
    }
    if (moving && motion_execution_fault_latched_.load()) {
      // A Move RPC was accepted locally but the robot reported no measured
      // motion for the configured feedback timeout.  Do not keep forwarding
      // the old Nav2 command: the sport controller may accept it later and
      // make the robot move toward a stale goal.  The navigation action is
      // cancelled by lio_base_bridge; its following zero command clears this
      // latch and permits a deliberately issued new goal.
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting cmd_vel after Go2 motion execution failure; waiting for navigation cancel");
      return;
    }
    if (moving && localization_guard_enabled_ && !localization_valid_.load()) {
      bool send_stop = false;
      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        received_cmd_ = true;
        command_active_ = false;
        watchdog_stopped_ = true;
        last_cmd_time_ = now();
        send_stop = !localization_stop_sent_;
        localization_stop_sent_ = true;
      }
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting navigation velocity because FAST-LIO localization is invalid or stale");
      if (send_stop) {
        stopRobot();
      }
      return;
    }
    bool send_stop = false;
    bool clear_motion_fault = false;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      last_cmd_time_ = now();
      received_cmd_ = true;
      last_command_velocity_.linear.x = vx;
      last_command_velocity_.linear.y = vy;
      last_command_velocity_.angular.z = wz;
      command_active_ = moving;
      if (moving) {
        // Begin execution monitoring only after smoothing has crossed the
        // physical gait threshold. Otherwise most of the timeout is consumed
        // by commands for which the Go2 is not yet expected to move.
        if (meaningful_motion && motion_command_start_time_.nanoseconds() == 0) {
          motion_command_start_time_ = last_cmd_time_;
        } else if (!meaningful_motion) {
          motion_command_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        }
        stop_sent_for_zero_ = false;
        watchdog_stopped_ = false;
      } else {
        // velocity_smoother publishes zero continuously while idle.  A StopMove
        // edge is sufficient; flooding the reliable Unitree RPC topic can leave
        // stale stop requests queued in front of the first navigation command.
        send_stop = !stop_sent_for_zero_;
        stop_sent_for_zero_ = true;
        watchdog_stopped_ = true;
        motion_command_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        const bool feedback_fresh = last_motion_feedback_time_.nanoseconds() != 0 &&
          (now() - last_motion_feedback_time_).seconds() <= motion_feedback_stale_timeout_;
        clear_motion_fault = feedback_fresh && !motion_execution_valid_.load();
        if (feedback_fresh) {
          motion_execution_fault_latched_.store(false);
        }
      }
    }

    if (clear_motion_fault) {
      publishMotionExecutionValid(true);
    }

    if (!moving) {
      if (send_stop) {
        stopRobot();
      }
      return;
    }

    publishMove(vx, vy, wz, "navigation");
  }

  void handleInteractiveCommand(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string request_id;
    try {
      const auto payload = nlohmann::json::parse(msg->data);
      request_id = payload.value("request_id", "");
      const std::string action = payload.value("action", "");
      if (request_id.empty()) {
        throw std::runtime_error("missing request_id");
      }
      if (sport_pub_->get_subscription_count() == 0U) {
        publishInteractiveStatus(
          request_id, "rejected",
          "Go2 sport service is not matched; check network interface and ROS_DOMAIN_ID");
        return;
      }

      const double requested_speed = std::abs(payload.value("speed", 0.0));
      const double duration = std::clamp(payload.value("duration", 0.0), 0.0, 15.0);
      double vx = 0.0;
      double vy = 0.0;
      double wz = 0.0;
      bool motion = true;
      if (action == "forward") {
        vx = std::min(requested_speed, max_linear_command_);
      } else if (action == "backward") {
        vx = -std::min(requested_speed, max_linear_command_);
      } else if (action == "left") {
        vy = std::min(requested_speed, max_lateral_command_);
      } else if (action == "right") {
        vy = -std::min(requested_speed, max_lateral_command_);
      } else if (action == "turn_left") {
        wz = std::min(requested_speed, max_angular_command_);
      } else if (action == "turn_right") {
        wz = -std::min(requested_speed, max_angular_command_);
      } else {
        motion = false;
      }

      if (std::hypot(vx, vy) <= cmd_vel_deadband_ &&
        std::abs(wz) >= motion_feedback_command_angular_ &&
        std::abs(wz) < min_pure_rotation_command_)
      {
        wz = std::copysign(min_pure_rotation_command_, wz);
      }

      if (motion) {
        if (rejectMotionWithoutFeedback() || motion_execution_fault_latched_.load()) {
          publishInteractiveStatus(request_id, "rejected", "motion feedback is unavailable or faulted");
          return;
        }
        if (localization_guard_enabled_ && !localization_valid_.load()) {
          publishInteractiveStatus(
            request_id, "rejected", "localization is invalid or stale");
          stopRobot();
          return;
        }
        if (duration < 0.1 ||
          (std::abs(vx) <= cmd_vel_deadband_ &&
          std::abs(vy) <= cmd_vel_deadband_ &&
          std::abs(wz) <= cmd_vel_deadband_))
        {
          publishInteractiveStatus(request_id, "rejected", "invalid motion speed or duration");
          return;
        }
        {
          std::lock_guard<std::mutex> lock(cmd_mutex_);
          interactive_active_ = true;
          interactive_velocity_.linear.x = vx;
          interactive_velocity_.linear.y = vy;
          interactive_velocity_.angular.z = wz;
          interactive_deadline_ = now() + rclcpp::Duration::from_seconds(duration);
          received_cmd_ = true;
          command_active_ = true;
          stop_sent_for_zero_ = false;
          watchdog_stopped_ = false;
          last_cmd_time_ = now();
          last_command_velocity_.linear.x = vx;
          last_command_velocity_.linear.y = vy;
          last_command_velocity_.angular.z = wz;
          const bool meaningful_motion =
            std::hypot(vx, vy) >= motion_feedback_command_linear_ ||
            std::abs(wz) >= motion_feedback_command_angular_;
          motion_command_start_time_ = meaningful_motion ?
            last_cmd_time_ : rclcpp::Time(0, 0, RCL_ROS_TIME);
        }
        publishMove(vx, vy, wz, "interactive");
        publishInteractiveStatus(request_id, "accepted", "");
        return;
      }

      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        interactive_active_ = false;
        command_active_ = false;
        watchdog_stopped_ = true;
        stop_sent_for_zero_ = true;
        motion_command_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
      if (action == "stop") {
        stopRobot();
      } else {
        unitree_api::msg::Request req;
        if (action == "sit") {
          sport_client_->StandDown(req);
        } else if (action == "stand") {
          sport_client_->StandUp(req);
        } else if (action == "recovery") {
          sport_client_->RecoveryStand(req);
        } else {
          publishInteractiveStatus(request_id, "rejected", "unsupported action");
          return;
        }
        prepareRequest(req, true);
        sport_pub_->publish(req);
      }
      publishInteractiveStatus(request_id, "accepted", "");
    } catch (const std::exception & error) {
      publishInteractiveStatus(request_id, "rejected", error.what());
    }
  }

  void publishInteractiveStatus(
    const std::string & request_id, const std::string & status, const std::string & error)
  {
    nlohmann::json payload = {
      {"request_id", request_id}, {"status", status}, {"error", error}};
    std_msgs::msg::String message;
    message.data = payload.dump();
    interactive_status_pub_->publish(message);
  }

  void handleLocalizationHealth(bool valid)
  {
    const bool was_valid = localization_valid_.exchange(valid);
    if (valid) {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      localization_stop_sent_ = false;
      if (!was_valid) {
        RCLCPP_INFO(get_logger(), "FAST-LIO localization is healthy; motion gate opened");
      }
      return;
    }

    bool send_stop = false;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      interactive_active_ = false;
      command_active_ = false;
      watchdog_stopped_ = true;
      stop_sent_for_zero_ = true;
      send_stop = !localization_stop_sent_;
      localization_stop_sent_ = true;
    }
    if (send_stop) {
      RCLCPP_ERROR(
        get_logger(), "FAST-LIO localization became invalid or stale; sending StopMove");
      stopRobot();
    }
  }

  void publishMove(double vx, double vy, double wz, const char * source)
  {
    unitree_api::msg::Request req;
    sport_client_->Move(req, vx, vy, wz);
    prepareRequest(req, true);
    sport_pub_->publish(req);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Forwarding %s velocity to Go2: vx=%.3f vy=%.3f wz=%.3f (sport subscribers=%zu)",
      source, vx, vy, wz, sport_pub_->get_subscription_count());
    if (sport_pub_->get_subscription_count() == 0U) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(), 5000,
        "No Go2 sport subscriber matched /api/sport/request; "
        "check GO2_NETWORK_INTERFACE and ROS_DOMAIN_ID");
    }
  }

  void prepareRequest(unitree_api::msg::Request & req, bool no_reply)
  {
    // Current Unitree SDK2 assigns a unique monotonic identity to every RPC.
    // Leaving the generated ROS message at id=0 lets newer firmware treat all
    // later velocity packets as duplicates of the first idle packet.
    req.header.identity.id = next_request_id_.fetch_add(1, std::memory_order_relaxed);
    req.header.lease.id = 0;
    req.header.policy.priority = 0;
    req.header.policy.noreply = no_reply;
  }

  void checkVelocityTimeout()
  {
    bool should_stop = false;
    bool republish_interactive = false;
    bool execution_failed = false;
    bool execution_recovered = false;
    bool feedback_stale = false;
    double requested_linear = 0.0;
    double requested_angular = 0.0;
    double measured_linear = 0.0;
    double measured_angular = 0.0;
    std::uint32_t sport_error_code = 0;
    std::uint8_t sport_mode = 0;
    std::uint8_t sport_gait_type = 0;
    geometry_msgs::msg::Twist interactive_velocity;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      if (interactive_active_) {
        if (now() >= interactive_deadline_) {
          interactive_active_ = false;
          watchdog_stopped_ = true;
          command_active_ = false;
          stop_sent_for_zero_ = true;
          motion_command_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          should_stop = true;
        } else {
          interactive_velocity = interactive_velocity_;
          republish_interactive = true;
        }
      } else {
        if (received_cmd_ && command_active_ && !watchdog_stopped_ &&
          (now() - last_cmd_time_).seconds() > cmd_vel_timeout_)
        {
          watchdog_stopped_ = true;
          command_active_ = false;
          stop_sent_for_zero_ = true;
          motion_command_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          should_stop = true;
        }
      }

      if (command_active_) {
        requested_linear = std::hypot(
          last_command_velocity_.linear.x, last_command_velocity_.linear.y);
        requested_angular = std::abs(last_command_velocity_.angular.z);
        const bool meaningful_command =
          requested_linear >= motion_feedback_command_linear_ ||
          requested_angular >= motion_feedback_command_angular_;
        feedback_stale = last_motion_feedback_time_.nanoseconds() == 0 ||
          (now() - last_motion_feedback_time_).seconds() > motion_feedback_stale_timeout_;
        measured_linear = measured_sport_linear_speed_;
        measured_angular = measured_sport_angular_speed_;
        sport_error_code = sport_error_code_;
        sport_mode = sport_mode_;
        sport_gait_type = sport_gait_type_;
        const bool observed_motion =
          measured_linear >= motion_feedback_min_linear_ ||
          measured_angular >= motion_feedback_min_angular_;
        const bool no_motion_timed_out =
          motion_command_start_time_.nanoseconds() != 0 &&
          (now() - motion_command_start_time_).seconds() >= motion_feedback_timeout_;
        if (feedback_stale) {
          // Previously this reset the timer forever while Move kept flowing.
          execution_failed = motion_execution_valid_.load();
        } else if (!meaningful_command) {
          motion_command_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        } else if (observed_motion) {
          // Check consecutive no-motion time rather than total command time.
          // Brief near-zero samples are normal between quadruped steps.
          motion_command_start_time_ = now();
          if (!motion_execution_valid_.load()) {
            execution_recovered = true;
          }
        } else if (motion_command_start_time_.nanoseconds() == 0) {
          motion_command_start_time_ = now();
        } else if (no_motion_timed_out) {
          execution_failed = motion_execution_valid_.load();
        }
      }
    }
    if (execution_failed) {
      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        command_active_ = false;
        interactive_active_ = false;
        watchdog_stopped_ = true;
        stop_sent_for_zero_ = true;
        motion_execution_fault_latched_.store(true);
      }
      republish_interactive = false;
      publishMotionExecutionValid(false);
      RCLCPP_ERROR(
        get_logger(),
        "Go2 stopped (%s), execution timeout %.2fs: requested linear=%.3f m/s "
        "yaw=%.3f rad/s, measured linear=%.3f m/s yaw=%.3f rad/s; "
        "SportModeState mode=%u gait=%u error_code=%u",
        feedback_stale ? "state feedback stale" : "motion not executed",
        motion_feedback_timeout_, requested_linear, requested_angular,
        measured_linear, measured_angular, static_cast<unsigned int>(sport_mode),
        static_cast<unsigned int>(sport_gait_type), sport_error_code);
      stopRobot();
    } else if (execution_recovered) {
      publishMotionExecutionValid(true);
      RCLCPP_INFO(get_logger(), "Go2 motion execution recovered");
    } else if (feedback_stale) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot verify Go2 motion execution: %s is stale",
        sport_state_topic_.c_str());
    }
    if (should_stop) {
      RCLCPP_WARN(get_logger(), "cmd_vel timed out; sending StopMove");
      stopRobot();
    } else if (republish_interactive) {
      publishMove(
        interactive_velocity.linear.x,
        interactive_velocity.linear.y,
        interactive_velocity.angular.z,
        "interactive");
    }
  }

  // 停止机器人
  void stopRobot()
  {
    unitree_api::msg::Request req;
    sport_client_->StopMove(req);
    prepareRequest(req, true);
    sport_pub_->publish(req);
  }

  // 成员变量
  std::shared_ptr<SportClient> sport_client_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string odom_source_;
  std::string pose_topic_;
  std::string sport_state_topic_;
  std::string cmd_vel_topic_;
  std::string interactive_command_topic_;
  std::string interactive_status_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string localization_valid_topic_;
  std::string motion_feedback_topic_;
  double cmd_vel_timeout_{0.5};
  double cmd_vel_deadband_{0.005};
  double max_linear_command_{0.70};
  double max_lateral_command_{0.30};
  double max_angular_command_{1.00};
  double min_pure_rotation_command_{0.0};
  bool planar_odometry_{true};
  bool localization_guard_enabled_{false};
  double max_pose_speed_{1.8};
  double max_pose_angular_speed_{2.5};
  double pose_jump_check_window_{1.0};
  double twist_filter_alpha_{0.35};
  double state_stationary_linear_threshold_{0.04};
  double state_stationary_angular_threshold_{0.06};
  double state_stationary_position_delta_{0.0035};
  double state_stationary_yaw_delta_{0.0045};
  double motion_feedback_timeout_{3.0};
  double motion_feedback_stale_timeout_{1.0};
  double motion_feedback_command_linear_{0.15};
  double motion_feedback_command_angular_{0.35};
  double motion_feedback_min_linear_{0.03};
  double motion_feedback_min_angular_{0.05};
  std::mutex cmd_mutex_;
  geometry_msgs::msg::Twist last_measured_twist_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time interactive_deadline_{0, 0, RCL_ROS_TIME};
  std::atomic<std::int64_t> next_request_id_{
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count()};
  bool received_cmd_{false};
  bool watchdog_stopped_{false};
  bool command_active_{false};
  bool stop_sent_for_zero_{false};
  bool interactive_active_{false};
  bool localization_stop_sent_{false};
  std::atomic<bool> localization_valid_{false};
  geometry_msgs::msg::Twist interactive_velocity_;
  geometry_msgs::msg::Twist last_command_velocity_;
  rclcpp::Time motion_command_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_motion_feedback_time_{0, 0, RCL_ROS_TIME};
  double measured_sport_linear_speed_{0.0};
  double measured_sport_angular_speed_{0.0};
  std::uint32_t sport_error_code_{0};
  std::uint8_t sport_mode_{0};
  std::uint8_t sport_gait_type_{0};
  std::atomic<bool> motion_execution_valid_{true};
  std::atomic<bool> motion_execution_fault_latched_{false};
  bool has_last_pose_{false};
  std::int64_t last_pose_stamp_ns_{0};
  double last_pose_x_{0.0};
  double last_pose_y_{0.0};
  double last_pose_yaw_{0.0};
  bool sport_state_initialized_{false};
  double sport_frame_yaw_{0.0};
  double sport_odom_x_{0.0};
  double sport_odom_y_{0.0};
  double sport_odom_yaw_{0.0};
  double previous_sport_raw_x_{0.0};
  double previous_sport_raw_y_{0.0};
  double previous_sport_raw_yaw_{0.0};
  rclcpp::Time last_sport_state_time_{0, 0, RCL_ROS_TIME};

  // ROS接口
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_valid_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr interactive_command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr interactive_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_feedback_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_pub_;
  rclcpp::TimerBase::SharedPtr startup_stop_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::atomic<bool> shutdown_stop_sent_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2Base>());
  rclcpp::shutdown();
  return 0;
}
