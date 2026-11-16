"""Publish a SLAM-only scan stream after rejecting unstable turn samples."""

from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from go2_mapping_f.gate_policy import ScanGatePolicy


class ScanStabilityGate(Node):
    """Keep fast-turn and discontinuous scans out of persistent mapping."""

    def __init__(self) -> None:
        super().__init__("scan_stability_gate")

        input_scan = self.declare_parameter("input_scan_topic", "/scan").value
        output_scan = self.declare_parameter(
            "output_scan_topic", "/scan_slam_f"
        ).value
        odom_topic = self.declare_parameter("odom_topic", "/odom").value
        cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.gate_enabled = bool(
            self.declare_parameter("gate_enabled", True).value
        )
        block_velocity = float(
            self.declare_parameter("block_angular_velocity", 0.70).value
        )
        resume_velocity = float(
            self.declare_parameter("resume_angular_velocity", 0.50).value
        )
        settle_time = float(self.declare_parameter("settle_time", 0.20).value)
        max_scan_gap = float(
            self.declare_parameter("max_scan_gap", 0.30).value
        )
        self.motion_state_timeout = float(
            self.declare_parameter("motion_state_timeout", 0.30).value
        )

        self.policy = ScanGatePolicy(
            block_velocity,
            resume_velocity,
            settle_time,
            max_scan_gap,
        )
        self.latest_cmd: Optional[Tuple[float, float]] = None
        self.latest_odom: Optional[Tuple[float, float]] = None
        self.passed_scans = 0
        self.blocked_scans = 0

        self.scan_publisher = self.create_publisher(
            LaserScan, output_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, input_scan, self.scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        self.create_timer(5.0, self.report_statistics)

        self.get_logger().info(
            "F scan gate ready: %s -> %s, block >= %.2f rad/s, "
            "resume <= %.2f rad/s, max gap %.2f s"
            % (
                input_scan,
                output_scan,
                block_velocity,
                resume_velocity,
                max_scan_gap,
            )
        )

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def cmd_vel_callback(self, message: Twist) -> None:
        self.latest_cmd = (self.now_seconds(), abs(float(message.angular.z)))

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = (
            self.now_seconds(),
            abs(float(message.twist.twist.angular.z)),
        )

    def current_angular_velocity(self, now: float) -> float:
        speeds = []
        for sample in (self.latest_cmd, self.latest_odom):
            if sample is not None and now - sample[0] <= self.motion_state_timeout:
                speeds.append(sample[1])
        return max(speeds, default=0.0)

    @staticmethod
    def scan_stamp_seconds(message: LaserScan, fallback: float) -> float:
        stamp = message.header.stamp
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return value if value > 0.0 else fallback

    def scan_callback(self, message: LaserScan) -> None:
        if not self.gate_enabled:
            self.scan_publisher.publish(message)
            self.passed_scans += 1
            return

        now = self.now_seconds()
        was_blocked = self.policy.blocked
        decision = self.policy.evaluate(
            now,
            self.scan_stamp_seconds(message, now),
            self.current_angular_velocity(now),
        )
        if decision.allow:
            self.scan_publisher.publish(message)
            self.passed_scans += 1
            if was_blocked:
                self.get_logger().info("F SLAM scan stream stable; mapping resumed")
            return

        self.blocked_scans += 1
        if not was_blocked:
            detail = ""
            if decision.scan_gap is not None and decision.reason == "scan_gap":
                detail = f" ({decision.scan_gap:.3f} s)"
            self.get_logger().warning(
                f"F SLAM scan stream paused: {decision.reason}{detail}"
            )

    def report_statistics(self) -> None:
        total = self.passed_scans + self.blocked_scans
        if total == 0:
            return
        self.get_logger().info(
            "F scan gate totals: passed=%d blocked=%d state=%s"
            % (
                self.passed_scans,
                self.blocked_scans,
                "paused" if self.policy.blocked else "mapping",
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanStabilityGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # ROS launch may already have shut the shared context down on SIGINT.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
