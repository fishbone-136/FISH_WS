#!/usr/bin/python3
"""Test the Unitree front end, dynamic scan projection, and dropout in DDS isolation."""

from array import array
from collections import deque
import json
import math
import os
import signal
import subprocess
import time

import numpy as np
import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, Header, String

from go2_perception.unitree_cloud import ROTATION, TRANSLATION


def native_cloud(stamp, frame, points, offsets):
    data = np.zeros(len(points), dtype={
        'names': ('x', 'y', 'z', 'intensity', 'ring', 'time'),
        'formats': ('<f4', '<f4', '<f4', '<f4', '<u2', '<f4'),
        'offsets': (0, 4, 8, 16, 20, 24), 'itemsize': 32,
    })
    data['x'], data['y'], data['z'] = points.T
    data['time'] = offsets
    fields = [PointField(name=name, offset=data.dtype.fields[name][1], count=1,
                         datatype=PointField.UINT16 if name == 'ring' else PointField.FLOAT32)
              for name in data.dtype.names]
    msg = PointCloud2(header=Header(stamp=stamp, frame_id=frame), fields=fields,
                      height=1, width=len(points), point_step=32, row_step=32*len(points))
    msg.data = array('B', data.tobytes())
    return msg


def main():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '94':
        raise SystemExit('Use isolated ROS_DOMAIN_ID=94 and ROS_LOCALHOST_ONLY=1')
    os.environ['CYCLONEDDS_URI'] = (
        '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
        '<Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>40'
        '</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers>'
        '</Discovery></Domain></CycloneDDS>')
    rclpy.init()
    node = rclpy.create_node('unitree_navigation_fixture')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    raw = node.create_publisher(PointCloud2, '/utlidar/cloud', qos_profile_sensor_data)
    body = node.create_publisher(PointCloud2, '/utlidar/cloud_base', qos_profile_sensor_data)
    odom = node.create_publisher(Odometry, '/utlidar/robot_odom', qos_profile_sensor_data)
    lifecycle = node.create_client(GetState, '/bt_navigator/get_state')
    manager = node.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
    state = {}
    ray_stamps = deque(maxlen=16)
    node.create_subscription(PointCloud2, '/unitree/near_clearance_rays',
        lambda msg: ray_stamps.append(msg.header.stamp)
        if msg.header.frame_id == 'unitree_lidar_aligned' and msg.width else None, 4)
    for name, kind in (
        ('/scan', LaserScan), ('/scan_safety', LaserScan), ('/traversability/local', OccupancyGrid),
        ('/lio/localization_valid', Bool), ('/unitree/navigation_source/valid', Bool),
        ('/go2/navigation_motion_enabled', Bool), ('/unitree/navigation_source/status', String),
    ):
        qos = qos_profile_sensor_data if kind is LaserScan else latched
        node.create_subscription(kind, name, lambda message, key=name: state.update({key: message}), qos)
    floor = np.array([(x, y, -.32) for x in np.arange(.5, 4., .08)
                      for y in np.arange(-1.4, 1.45, .08)])
    obstacle = np.array([(.64, y, z) for y in np.linspace(-.09, .09, 7)
                         for z in np.linspace(-.16, .22, 7)])
    points = np.vstack((floor, obstacle))
    offsets = np.linspace(0., .060, len(points), dtype=np.float32)
    native_points = (points - TRANSLATION) @ ROTATION
    epoch = 1_700_000_000_000_000
    last_scan = 0.

    def tick(with_obstacle=False, send_cloud=True, reset=False):
        nonlocal last_scan
        now = node.get_clock().now().nanoseconds - epoch - (5_000_000_000 if reset else 0)
        pose = Odometry(header=Header(stamp=Time(nanoseconds=now).to_msg(), frame_id='odom'),
                        child_frame_id='base_link')
        pose.pose.pose.orientation.w = 1.
        odom.publish(pose)
        if send_cloud and time.monotonic() - last_scan >= .065:
            stamp = Time(nanoseconds=now-60_000_000).to_msg()
            size = len(points) if with_obstacle else len(floor)
            raw.publish(native_cloud(stamp, 'utlidar_lidar', native_points[:size], offsets[:size]))
            # Keep the nearby obstacle exclusively in the raw source.
            body.publish(native_cloud(stamp, 'base_link', floor, offsets[:len(floor)]))
            last_scan = time.monotonic()
        rclpy.spin_once(node, timeout_sec=.005)
        time.sleep(.003)

    def wait_for(predicate, timeout, **kwargs):
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            tick(**kwargs)
            if predicate():
                return time.monotonic() - start
        raise AssertionError({key: getattr(value, 'data', 'received') if not isinstance(
            value, OccupancyGrid) else 'grid received' for key, value in state.items()})

    def valid(name):
        return bool(name in state and state[name].data)

    def matching_scans():
        main_scan, safety = state.get('/scan'), state.get('/scan_safety')
        return main_scan and safety and main_scan.header.stamp == safety.header.stamp

    lifecycle_request = None

    def navigation_active():
        nonlocal lifecycle_request
        if lifecycle_request and lifecycle_request.done():
            response = lifecycle_request.result()
            lifecycle_request = None
            if response.current_state.id == 3:
                return True
        if lifecycle_request is None and lifecycle.service_is_ready():
            lifecycle_request = lifecycle.call_async(GetState.Request())
        return False

    def nearby_hit():
        scan = state.get('/scan_safety')
        return bool(scan and any(.55 < radius < .8 and
                    abs(scan.angle_min + i*scan.angle_increment) < .24
                    for i, radius in enumerate(scan.ranges) if math.isfinite(radius)))

    args = ['ros2', 'launch', 'go2_lio_navigation', 'go2_lio_navigation.launch.py',
            'enable_go2_base:=false', 'use_rviz:=false', 'enable_nav2:=true',
            'nav2_use_composition:=false']
    process = subprocess.Popen(args, start_new_session=True)
    try:
        wait_for(lambda: matching_scans() and state['/scan'].header.stamp in ray_stamps and
                 valid('/lio/localization_valid') and
                 '/traversability/local' in state and '/go2/navigation_motion_enabled' in state and
                 navigation_active(), 30.)
        assert not valid('/go2/navigation_motion_enabled')
        assert not nearby_hit()
        assert node.count_publishers('/livox/lidar') == 0
        appeared = wait_for(nearby_hit, .7, with_obstacle=True)
        removed = wait_for(lambda: not nearby_hit(), .7)
        assert valid('/unitree/navigation_source/valid')
        dropped = wait_for(lambda: not valid('/unitree/navigation_source/valid') and
                           not valid('/lio/localization_valid'), .8, send_cloud=False)
        wait_for(lambda: valid('/unitree/navigation_source/valid') and
                 valid('/lio/localization_valid'), 2.)
        assert manager.service_is_ready()
        shutdown = manager.call_async(ManageLifecycleNodes.Request(
            command=ManageLifecycleNodes.Request.SHUTDOWN))
        wait_for(shutdown.done, 12.)
        assert shutdown.result().success
        wait_for(lambda: '/unitree/navigation_source/status' in state and bool(json.loads(
            state['/unitree/navigation_source/status'].data)['fault']), 1.5, reset=True)
        assert not valid('/unitree/navigation_source/valid')
        assert process.poll() is None
        print(f'PASS dynamic scan appearance {appeared*1000:.0f} ms, removal {removed*1000:.0f} ms, '
              f'cloud dropout invalidated source and localization in {dropped*1000:.0f} ms')
        print('PASS shared scan stamps, preview inhibit, clock reset, and no HAP publisher')
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
