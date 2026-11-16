#!/usr/bin/env python3
"""Print compact geometry statistics for one ROS 2 PointCloud2 message."""

import argparse
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class CloudStats(Node):
    def __init__(self, topic: str, transient_local: bool) -> None:
        super().__init__("cloud_stats")
        qos = qos_profile_sensor_data
        if transient_local:
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
        self.subscription = self.create_subscription(
            PointCloud2, topic, self.on_cloud, qos)

    def on_cloud(self, message: PointCloud2) -> None:
        xyz = point_cloud2.read_points_numpy(
            message, field_names=("x", "y", "z"), skip_nans=True)
        xyz = np.asarray(xyz, dtype=np.float64).reshape((-1, 3))
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if xyz.size == 0:
            print("empty cloud", flush=True)
            os._exit(0)

        ranges = np.linalg.norm(xyz, axis=1)
        covariance = np.cov(xyz, rowvar=False)
        eigenvalues = np.linalg.eigvalsh(covariance)
        print(f"frame={message.header.frame_id} points={xyz.shape[0]}")
        print(f"min={xyz.min(axis=0).tolist()}")
        print(f"max={xyz.max(axis=0).tolist()}")
        print(f"mean={xyz.mean(axis=0).tolist()}")
        print(
            "range_quantiles="
            f"{np.quantile(ranges, [0.01, 0.1, 0.5, 0.9, 0.99]).tolist()}")
        print(f"covariance_eigenvalues={eigenvalues.tolist()}", flush=True)
        os._exit(0)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("topic", nargs="?", default="/cloud_registered_body")
    parser.add_argument(
        "--transient-local",
        action="store_true",
        help="request the latched sample from a transient-local publisher",
    )
    arguments = parser.parse_args()
    rclpy.init()
    node = CloudStats(arguments.topic, arguments.transient_local)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
