#!/usr/bin/env python3
"""Estimate how many HAP point-to-plane matches exist while stationary."""

import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2


def voxel(points: np.ndarray, leaf: float) -> np.ndarray:
    keys = np.floor(points / leaf).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


class StaticMatchCheck(Node):
    def __init__(self) -> None:
        super().__init__("hap_static_match_check")
        self.clouds = []
        self.stamps = []
        self.create_subscription(
            PointCloud2,
            sys.argv[1] if len(sys.argv) > 1 else "/livox/lidar",
            self.on_cloud,
            qos_profile_sensor_data,
        )

    def on_cloud(self, message: PointCloud2) -> None:
        endian = ">" if message.is_bigendian else "<"
        xyz_dtype = np.dtype({
            "names": ("x", "y", "z"),
            "formats": (endian + "f4", endian + "f4", endian + "f4"),
            "offsets": (0, 4, 8),
            "itemsize": message.point_step,
        })
        packed = np.frombuffer(message.data, dtype=xyz_dtype)
        points = np.column_stack((packed["x"], packed["y"], packed["z"]))
        points = np.asarray(points, dtype=np.float64).reshape((-1, 3))[::5]
        points = points[np.isfinite(points).all(axis=1)]
        points = points[np.linalg.norm(points, axis=1) > 0.25]
        self.clouds.append(voxel(points, 0.4))
        self.stamps.append(
            float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9)
        if len(self.clouds) < 2:
            return

        map_points = voxel(np.concatenate(self.clouds[:-1], axis=0), 0.4)
        query_points = self.clouds[-1]
        tree = cKDTree(map_points)
        distances, indices = tree.query(query_points, k=5, workers=-1)
        neighbor_ok = distances[:, -1] ** 2 <= 5.0
        plane_ok = np.zeros(query_points.shape[0], dtype=bool)
        residual_ok = np.zeros(query_points.shape[0], dtype=bool)
        for row in np.nonzero(neighbor_ok)[0]:
            neighbors = map_points[indices[row]]
            normal = np.linalg.lstsq(
                neighbors, -np.ones((neighbors.shape[0],)), rcond=None)[0]
            norm = np.linalg.norm(normal)
            if norm < 1e-9:
                continue
            plane = np.r_[normal / norm, 1.0 / norm]
            neighbor_residuals = np.abs(neighbors @ plane[:3] + plane[3])
            if neighbor_residuals.max() > 0.1:
                continue
            plane_ok[row] = True
            residual = abs(query_points[row] @ plane[:3] + plane[3])
            score = 1.0 - 0.9 * residual / np.sqrt(np.linalg.norm(query_points[row]))
            residual_ok[row] = score > 0.9

        intervals = np.diff(np.asarray(self.stamps))
        print(
            f"frames={len(self.clouds)} interval_min={intervals.min():.6f} "
            f"interval_max={intervals.max():.6f} map={len(map_points)} "
            f"query={len(query_points)} neighbor={neighbor_ok.sum()} "
            f"plane={plane_ok.sum()} effective={residual_ok.sum()}",
            flush=True,
        )
        time.sleep(0.1)
        os._exit(0)


def main() -> None:
    rclpy.init()
    rclpy.spin(StaticMatchCheck())


if __name__ == "__main__":
    main()
