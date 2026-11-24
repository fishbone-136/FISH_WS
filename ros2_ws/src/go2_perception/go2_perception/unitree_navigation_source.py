#!/usr/bin/python3
"""Publish the navigation front-end streams from factory Unitree LiDAR and odometry."""

from collections import OrderedDict, deque
import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Bool, Header, String

from .unitree_cloud import ROTATION, TRANSLATION, decode_cloud, factory_transform_error


NANOSECONDS = 1_000_000_000
SENSOR_FRAME = 'unitree_lidar_aligned'
CLOUD_FIELDS = [
    PointField(name=name, offset=4 * index, datatype=PointField.FLOAT32, count=1)
    for index, name in enumerate(('x', 'y', 'z', 'intensity'))
]


def source_stamp(message):
    stamp = message.header.stamp
    return stamp.sec * NANOSECONDS + stamp.nanosec


def rotate(quaternions, vectors):
    xyz = quaternions[..., :3]
    uv = 2.0 * np.cross(xyz, vectors)
    return vectors + quaternions[..., 3:] * uv + np.cross(xyz, uv)


def interpolate_poses(stamps, positions, orientations, query):
    """Interpolate only with continuous, bracketing native odometry."""
    after = np.clip(np.searchsorted(stamps, query), 1, len(stamps) - 1)
    before = after - 1
    gap = stamps[after] - stamps[before]
    if np.any(gap > 50_000_000):
        raise ValueError('native odometry gap across scan')
    fraction = ((query - stamps[before]) / gap)[:, None]
    nearest = orientations[before]
    following = orientations[after]
    following = np.where(np.sum(nearest * following, axis=1, keepdims=True) < 0,
                         -following, following)
    angle = np.abs(np.sum(nearest * following, axis=1))
    travel = np.linalg.norm(positions[after] - positions[before], axis=1)
    if np.any(travel > .025 + 1.8 * gap / NANOSECONDS) or np.any(angle < .997):
        raise ValueError('native odometry jumped during scan')
    position = positions[before] + fraction * (positions[after] - positions[before])
    rotation = nearest + fraction * (following - nearest)
    rotation /= np.linalg.norm(rotation, axis=1, keepdims=True)
    return position, rotation


def scan_geometry(raw, body):
    """Keep factory-filtered returns and validated raw returns ahead of the body."""
    raw_points, raw_keys = raw
    body_points, body_keys = body
    if factory_transform_error(raw, body) > .005:
        raise ValueError('native lidar-to-body transform failed validation')
    raw_body = raw_points @ ROTATION.T + TRANSLATION
    near = (np.isfinite(raw_body).all(axis=1) &
            (raw_body[:, 0] >= .40) & (raw_body[:, 0] <= .85) &
            (np.abs(raw_body[:, 1]) <= .85) &
            (raw_body[:, 2] >= -.18) & (raw_body[:, 2] <= .80))
    close_points, close_keys = raw_body[near], raw_keys[near]
    extra = ~np.isin(close_keys, body_keys)
    good = np.isfinite(body_points).all(axis=1) & (
        np.linalg.norm(body_points - TRANSLATION, axis=1) > .01)
    points = np.concatenate((body_points[good], close_points[extra]))
    keys = np.concatenate((body_keys[good], close_keys[extra]))
    return points, point_offsets(keys, minimum_points=32)


def point_offsets(keys, minimum_points=0):
    seconds = keys.astype(np.uint32).view(np.float32)
    if len(keys) < minimum_points or not np.all(np.isfinite(seconds) & (seconds >= 0) & (seconds <= .15)):
        raise ValueError('invalid native point times or insufficient lidar returns')
    return np.rint(seconds.astype(float) * NANOSECONDS).astype(np.int64)


def near_clearance_rays(raw, stamp, start, end, odom, sensor_position, end_rotation):
    """Native returns within range of a near-body crossing, at one sensor origin."""
    points, keys = raw
    aligned = points @ ROTATION.T
    horizontal_range = np.linalg.norm(aligned[:, :2], axis=1)
    valid = np.isfinite(aligned).all(axis=1) & (horizontal_range > .01) & (horizontal_range <= 2.)
    query = stamp + point_offsets(keys[valid])
    covered = (query >= start) & (query <= end)
    if not np.any(covered):
        return None
    xyz = aligned[valid][covered]
    positions, rotations = interpolate_poses(*odom, query[covered])
    origins = positions + rotate(rotations, TRANSLATION)
    # The memory ray index assumes one origin. Omit this optional evidence
    # during body motion that would shift the actual rays within a voxel.
    if np.any(np.linalg.norm(origins - sensor_position, axis=1) > .005):
        return None
    return rotate(end_rotation * [-1, -1, -1, 1], rotate(rotations, xyz) + origins - sensor_position)


class UnitreeNavigationSource(Node):
    def __init__(self):
        super().__init__('unitree_navigation_source')
        self.poses = deque(maxlen=200)
        self.scans = OrderedDict()
        self.last_scan = 0
        self.origin = None
        self.clock_offset = None
        self.clock_offsets = deque(maxlen=200)
        self.clock_started = time.monotonic_ns()
        self.last_clock_update = self.clock_started
        self.clock_fault = ''
        self.epoch_errors = 0
        self.last_output = -math.inf
        self.output_stamp = None
        self.good_frames = 0
        self.published_frames = 0
        self.rejected_frames = 0
        self.last_rejection = ''
        self.last_valid = None
        self.world_frame = self.declare_parameter('world_frame', 'odom_lio').value
        self.planar_frame = self.declare_parameter('planar_frame', '').value
        qos = QoSProfile(depth=2)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.valid_pub = self.create_publisher(Bool, '/unitree/navigation_source/valid', latched)
        self.status_pub = self.create_publisher(String, '/unitree/navigation_source/status', latched)
        if self.planar_frame:
            self.planar_pub = self.create_publisher(PointCloud2, '/unitree/navigation_cloud_planar', qos)
        else:
            self.pose_pub = self.create_publisher(Odometry, '/Odometry', qos)
            self.world_pub = self.create_publisher(PointCloud2, '/cloud_registered', qos)
            self.sensor_pub = self.create_publisher(PointCloud2, '/cloud_registered_body', qos)
            self.rays_pub = self.create_publisher(PointCloud2, '/unitree/near_clearance_rays', qos)
        self.create_subscription(Odometry, '/utlidar/robot_odom', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/utlidar/cloud',
                                 lambda msg: self.on_cloud(msg, 'raw'), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/utlidar/cloud_base',
                                 lambda msg: self.on_cloud(msg, 'body'), qos_profile_sensor_data)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05, self.publish_health, clock=self.steady_clock)
        self.create_timer(.5, self.publish_status, clock=self.steady_clock)
        self.publish_health()

    def on_pose(self, message):
        if self.clock_fault:
            return
        stamp = source_stamp(message)
        if (message.header.frame_id != 'odom' or message.child_frame_id != 'base_link' or stamp <= 0):
            self.get_logger().warning('Unexpected native odometry frame or stamp', throttle_duration_sec=2)
            return
        if self.poses and stamp <= self.poses[-1][0]:
            if stamp < self.poses[-1][0] - 100_000_000:
                self.observe_epoch_error('native odometry time moved backwards')
            return
        pose = message.pose.pose
        p, q = pose.position, pose.orientation
        position = np.array([p.x, p.y, p.z])
        orientation = np.array([q.x, q.y, q.z, q.w])
        norm = np.linalg.norm(orientation)
        if not np.isfinite(position).all() or not .98 < norm < 1.02:
            return
        steady_now = time.monotonic_ns()
        arrival_offset = steady_now - stamp
        if self.clock_offset is None:
            self.clock_offsets.append(arrival_offset)
            if len(self.clock_offsets) >= 50 and steady_now - self.clock_started >= 800_000_000:
                earliest = min(self.clock_offsets)
                if max(self.clock_offsets) - earliest <= 80_000_000:
                    # Align device time to a monotonic clock so host wall-clock
                    # steps are distinct from native resets and queue delays.
                    self.clock_offset = earliest - 20_000_000
                    self.last_clock_update = steady_now
                    self.clock_offsets.clear()
                    self.origin = position.copy()
                    self.get_logger().info('Native odometry clock mapping established')
        else:
            estimated_age = arrival_offset - self.clock_offset
            if estimated_age < -250_000_000:
                self.observe_epoch_error('native odometry time advanced discontinuously')
                return
            self.epoch_errors = 0
            if estimated_age > 175_000_000:
                return  # A delayed packet is not a clock reset.
            self.clock_offsets.append(arrival_offset)
            interval = steady_now - self.last_clock_update
            if interval >= NANOSECONDS:
                estimate = min(self.clock_offsets) - 20_000_000
                # Follow oscillator drift at at most 0.5 ms per second.
                # Queued packets cannot instantly move the time mapping.
                limit = min(interval, 2 * NANOSECONDS) // 2000
                correction = max(-limit, min(limit, estimate - self.clock_offset))
                self.clock_offset += correction
                self.last_clock_update = steady_now
                self.clock_offsets.clear()
        self.poses.append((stamp, position, orientation / norm))
        self.publish_scan()

    def observe_epoch_error(self, reason):
        self.epoch_errors += 1
        if self.epoch_errors < 3:
            return
        self.clock_fault = reason
        self.scans.clear()
        self.publish_health()
        self.publish_status()
        self.get_logger().error(f'{reason}. Restart the Unitree navigation session.')

    def scan_age(self):
        if self.output_stamp is None:
            return None
        return (self.get_clock().now().nanoseconds - self.output_stamp) / NANOSECONDS

    def publish_health(self):
        age = self.scan_age()
        fresh = (not self.clock_fault and age is not None and -.05 <= age <= .25 and
                 time.monotonic() - self.last_output <= .25)
        if not fresh:
            self.good_frames = 0
        valid = fresh and self.good_frames >= 3
        if valid != self.last_valid:
            self.valid_pub.publish(Bool(data=bool(valid)))
            self.last_valid = valid

    def publish_status(self):
        age = self.scan_age()
        self.status_pub.publish(String(data=json.dumps({
            'valid': self.last_valid,
            'clock_ready': self.clock_offset is not None,
            'fault': self.clock_fault,
            'estimated_scan_age_ms': round(age * 1000, 1) if age is not None else None,
            'published_frames': self.published_frames,
            'rejected_frames': self.rejected_frames,
            'last_rejection': self.last_rejection,
        })))

    def on_cloud(self, message, kind):
        if self.clock_fault:
            return
        stamp = source_stamp(message)
        expect_frame = 'utlidar_lidar' if kind == 'raw' else 'base_link'
        if stamp <= self.last_scan or message.header.frame_id != expect_frame:
            return
        pair = self.scans.setdefault(stamp, {})
        pair[kind] = message
        while len(self.scans) > 4:
            self.scans.popitem(last=False)
        self.publish_scan()

    def publish_scan(self):
        if self.clock_offset is None or len(self.poses) < 2:
            return
        now = time.monotonic_ns()
        for stamp, pair in list(self.scans.items()):
            if stamp <= self.last_scan or now - stamp - self.clock_offset > 230_000_000:
                del self.scans[stamp]
                continue
            if 'raw' not in pair or 'body' not in pair:
                continue
            try:
                if 'geometry' not in pair:
                    pair['raw_geometry'] = decode_cloud(pair['raw'])
                    pair['geometry'] = scan_geometry(pair['raw_geometry'], decode_cloud(pair['body']))
                points, offsets = pair['geometry']
                query = stamp + offsets
                end = int(query.max())
                if end > self.poses[-1][0]:
                    continue
                if query.min() < self.poses[0][0]:
                    raise ValueError('native odometry does not cover scan start')
                stamps, positions, orientations = map(np.asarray, zip(*self.poses))
                odom = (stamps, positions, orientations)
                positions, rotations = interpolate_poses(*odom, query)
                rays = None
                if not self.planar_frame:
                    end_index = int(query.argmax())
                    end_rotation = rotations[end_index]
                    sensor_position = rotate(end_rotation, TRANSLATION) + positions[end_index]
                    try:
                        rays = near_clearance_rays(pair['raw_geometry'], stamp, query.min(),
                                                   end, odom, sensor_position, end_rotation)
                    except ValueError:
                        # Preserve the validated navigation frame if only the
                        # optional raw clearing returns cannot be registered.
                        pass
                self.publish_geometry(stamp, end, query, points, positions, rotations, rays)
            except (ValueError, TypeError, BufferError) as error:
                self.rejected_frames += 1
                self.last_rejection = str(error)
                self.get_logger().warning(f'Unitree scan dropped: {error}', throttle_duration_sec=2)
            del self.scans[stamp]

    def publish_geometry(self, stamp, end, query, points, positions, rotations, rays=None):
        end_index = int(query.argmax())
        end_position = positions[end_index] - self.origin
        end_rotation = rotations[end_index]
        world = rotate(rotations, points) + positions - self.origin
        age = time.monotonic_ns() - end - self.clock_offset
        if not -50_000_000 <= age <= 200_000_000:
            raise ValueError('native scan is outside the host freshness window')
        mapped = self.get_clock().now().nanoseconds - age
        if self.output_stamp is not None and mapped <= self.output_stamp:
            raise ValueError('ROS clock moved backwards; waiting for monotonic output')
        ros_stamp = Time(nanoseconds=mapped).to_msg()
        if self.planar_frame:
            x, y, z, w = end_rotation
            yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
            inverse_heading = np.array([0., 0., -math.sin(yaw/2), math.cos(yaw/2)])
            planar = rotate(inverse_heading, world - end_position)
            self.planar_pub.publish(self.cloud(ros_stamp, self.planar_frame, planar))
        else:
            if rays is not None:
                self.rays_pub.publish(self.cloud(ros_stamp, SENSOR_FRAME, rays))
            self.publish_registered(ros_stamp, world, end_position, end_rotation)
        if time.monotonic() - self.last_output > .25:
            self.good_frames = 0
        self.last_output = time.monotonic()
        self.output_stamp = mapped
        self.good_frames += 1
        self.published_frames += 1
        self.publish_health()
        self.last_scan = stamp

    def publish_registered(self, ros_stamp, world, end_position, end_rotation):
        sensor_position = rotate(end_rotation, TRANSLATION) + end_position
        inverse_rotation = end_rotation * [-1, -1, -1, 1]
        sensor = rotate(inverse_rotation, world - sensor_position)
        odometry = Odometry(header=Header(stamp=ros_stamp, frame_id=self.world_frame),
                            child_frame_id=SENSOR_FRAME)
        pose = odometry.pose.pose
        pose.position.x, pose.position.y, pose.position.z = map(float, sensor_position)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(
            float, end_rotation)
        self.pose_pub.publish(odometry)
        self.world_pub.publish(self.cloud(ros_stamp, self.world_frame, world))
        self.sensor_pub.publish(self.cloud(ros_stamp, SENSOR_FRAME, sensor))

    @staticmethod
    def cloud(stamp, frame, points):
        values = np.zeros((len(points), 4), dtype=np.float32)
        values[:, :3] = points
        return create_cloud(Header(stamp=stamp, frame_id=frame), CLOUD_FIELDS, values)


def main():
    rclpy.init()
    node = UnitreeNavigationSource()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
