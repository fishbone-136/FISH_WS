#!/usr/bin/env python3
"""Measure HAP PointCloud2 and IMU header timing with O(1) cloud parsing."""

import os
import statistics
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def summary(values) -> str:
    if not values:
        return "n=0"
    return (
        f"n={len(values)} mean={statistics.fmean(values):.9f} "
        f"min={min(values):.9f} max={max(values):.9f}")


class TimingStats(Node):
    def __init__(self) -> None:
        super().__init__("hap_pc2_timing_stats")
        self.started = time.monotonic()
        self.imu_stamps = []
        self.cloud_stamps = []
        self.cloud_delays = []
        self.imu_minus_cloud = []
        self.scan_durations = []
        imu_qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Imu, "/livox/imu", self.on_imu, imu_qos)
        self.create_subscription(
            PointCloud2, "/livox/lidar", self.on_cloud, qos_profile_sensor_data)
        self.create_timer(0.1, self.check_done)

    def on_imu(self, message: Imu) -> None:
        self.imu_stamps.append(stamp_seconds(message.header.stamp))

    def on_cloud(self, message: PointCloud2) -> None:
        stamp = stamp_seconds(message.header.stamp)
        self.cloud_stamps.append(stamp)
        self.cloud_delays.append(self.get_clock().now().nanoseconds * 1e-9 - stamp)
        if self.imu_stamps:
            self.imu_minus_cloud.append(self.imu_stamps[-1] - stamp)
        point_count = int(message.width) * int(message.height)
        if point_count:
            offset = (point_count - 1) * message.point_step + 18
            endian = ">d" if message.is_bigendian else "<d"
            point_stamp_ns = struct.unpack_from(endian, message.data, offset)[0]
            self.scan_durations.append(point_stamp_ns * 1e-9 - stamp)

    def check_done(self) -> None:
        if time.monotonic() - self.started < 5.0:
            return
        imu_intervals = [b - a for a, b in zip(self.imu_stamps, self.imu_stamps[1:])]
        cloud_intervals = [b - a for a, b in zip(self.cloud_stamps, self.cloud_stamps[1:])]
        print(
            f"imu_intervals: {summary(imu_intervals)} "
            f"non_monotonic={sum(value <= 0 for value in imu_intervals)}")
        print(
            f"cloud_intervals: {summary(cloud_intervals)} "
            f"non_monotonic={sum(value <= 0 for value in cloud_intervals)}")
        print(f"cloud_receive_delay: {summary(self.cloud_delays)}")
        print(f"latest_imu_minus_cloud: {summary(self.imu_minus_cloud)}")
        print(f"point_offset_duration: {summary(self.scan_durations)}", flush=True)
        time.sleep(0.1)
        os._exit(0)


def main() -> None:
    rclpy.init()
    rclpy.spin(TimingStats())


if __name__ == "__main__":
    main()
