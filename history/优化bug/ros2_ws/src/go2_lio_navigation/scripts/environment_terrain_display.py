#!/usr/bin/env python3
"""Coarsen historical terrain display; mapping/navigation inputs stay intact."""
import math
import os
from array import array
from pathlib import Path

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def coarsen_surface(points, costs, resolution):
    """One XY square per bin, retaining the highest-cost endpoint's height."""
    cells = np.floor(points[:, :2] / resolution).astype(np.int64)
    order = np.lexsort((-costs, cells[:, 1], cells[:, 0]))
    grouped = cells[order]
    starts = np.r_[0, np.flatnonzero(np.any(np.diff(grouped, axis=0), axis=1)) + 1]
    chosen = order[starts]
    result = points[chosen].copy()
    result[:, :2] = (cells[chosen] + .5) * resolution
    return result, costs[chosen]


class EnvironmentTerrainDisplay(Node):
    def __init__(self):
        super().__init__('environment_terrain_display')
        self.declare_parameter('display_resolution', .175)
        self.declare_parameter('enabled', True)
        self.declare_parameter('parent_launch_pid', 0)
        self.add_on_set_parameters_callback(self.validate)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output = self.create_publisher(PointCloud2, '/visualization/environment_terrain', qos)
        self.create_subscription(PointCloud2, '/mapping/terrain_surface', self.on_cloud, qos)
        self.last_size = None

    @staticmethod
    def validate(parameters):
        for p in parameters:
            if p.name == 'display_resolution' and (
                    not isinstance(p.value, (float, int)) or isinstance(p.value, bool) or
                    not math.isfinite(p.value) or p.value <= 0.):
                return SetParametersResult(successful=False, reason='Resolution must be positive')
        return SetParametersResult(successful=True)

    def on_cloud(self, message):
        if self.output.get_subscription_count() == 0:
            return
        if not self.get_parameter('enabled').value:
            self.output.publish(message)
            return
        try:
            fields = {f.name: f for f in message.fields}
            names = ('x', 'y', 'z', 'intensity')
            if not all(k in fields and fields[k].datatype == PointField.FLOAT32 and
                       fields[k].count == 1 and fields[k].offset + 4 <= message.point_step
                       for k in names):
                raise ValueError('Expected float32 XYZ and terrain cost')
            if (message.row_step < message.width * message.point_step or
                    len(message.data) < message.height * message.row_step):
                raise ValueError('Truncated terrain display cloud')
            endian = '>' if message.is_bigendian else '<'
            dtype = np.dtype(dict(names=names, formats=[endian+'f4']*4,
                                  offsets=[fields[k].offset for k in names],
                                  itemsize=message.point_step))
            data = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                              strides=(message.row_step, message.point_step))
            values = np.column_stack([data[k].ravel() for k in names])
            values = values[np.isfinite(values).all(axis=1)]
            points, costs = values[:, :3], values[:, 3]
            resolution = float(self.get_parameter('display_resolution').value)
            if not math.isfinite(resolution) or resolution <= 0.:
                raise ValueError('Invalid display resolution')
            if len(points):
                points, costs = coarsen_surface(points, costs, resolution)
            output = PointCloud2()
            output.header = message.header
            output.height, output.width = 1, len(points)
            output.point_step, output.row_step = 16, 16*len(points)
            output.is_dense = True
            output.fields = [PointField(name=k, offset=4*i, datatype=PointField.FLOAT32, count=1)
                             for i, k in enumerate(names)]
            output.data = array('B', np.asarray(np.column_stack([points, costs]), dtype='<f4').tobytes())
            self.output.publish(output)
            size = (message.width * message.height, len(points))
            if size != self.last_size:
                self.get_logger().info('Historical display: %d -> %d squares, %.3f m bins' %
                                       (*size, resolution), throttle_duration_sec=5.)
                self.last_size = size
        except (ValueError, TypeError, OverflowError) as error:
            self.get_logger().warning('Display update skipped: '+str(error), throttle_duration_sec=5.)


def process_identity(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return fields[19] if fields[0] != 'Z' else None
    except (OSError, IndexError):
        return None


def main():
    rclpy.init()
    node = EnvironmentTerrainDisplay()
    owner = int(node.get_parameter('parent_launch_pid').value) or os.getppid()
    identity = process_identity(owner)
    try:
        while rclpy.ok() and identity is not None and process_identity(owner) == identity:
            rclpy.spin_once(node, timeout_sec=.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
