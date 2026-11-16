#!/usr/bin/env python3
"""Measure Livox CustomMsg timing against its IMU without printing payloads."""

import argparse
import statistics
import time

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class TimingStats(Node):
    def __init__(self, duration: float) -> None:
        super().__init__("livox_timing_stats")
        self.duration = duration
        self.started = time.monotonic()
        self.last_imu_stamp = None
        self.cloud_stamps = []
        self.cloud_delays = []
        self.imu_minus_cloud = []
        self.point_counts = []
        self.scan_durations = []
        qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Imu, "/livox/imu", self.on_imu, qos)
        self.create_subscription(CustomMsg, "/livox/lidar", self.on_cloud, qos)
        self.create_timer(0.1, self.check_done)

    def on_imu(self, message: Imu) -> None:
        self.last_imu_stamp = stamp_seconds(message.header.stamp)

    def on_cloud(self, message: CustomMsg) -> None:
        stamp = stamp_seconds(message.header.stamp)
        self.cloud_stamps.append(stamp)
        self.cloud_delays.append(self.get_clock().now().nanoseconds * 1e-9 - stamp)
        if self.last_imu_stamp is not None:
            self.imu_minus_cloud.append(self.last_imu_stamp - stamp)
        self.point_counts.append(int(message.point_num))
        if message.points:
            self.scan_durations.append(float(message.points[-1].offset_time) * 1e-9)

    def check_done(self) -> None:
        if time.monotonic() - self.started < self.duration:
            return
        intervals = [
            current - previous
            for previous, current in zip(self.cloud_stamps, self.cloud_stamps[1:])
        ]
        negative = sum(value <= 0.0 for value in intervals)

        def summary(values):
            if not values:
                return "n=0"
            return (
                f"n={len(values)} mean={statistics.fmean(values):.6f} "
                f"min={min(values):.6f} max={max(values):.6f}")

        print(f"cloud_intervals: {summary(intervals)} non_monotonic={negative}")
        print(f"receive_delay: {summary(self.cloud_delays)}")
        print(f"latest_imu_minus_cloud: {summary(self.imu_minus_cloud)}")
        print(f"points_per_cloud: {summary(self.point_counts)}")
        print(f"point_offset_duration: {summary(self.scan_durations)}", flush=True)
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=5.0)
    args = parser.parse_args()
    rclpy.init()
    node = TimingStats(args.duration)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
