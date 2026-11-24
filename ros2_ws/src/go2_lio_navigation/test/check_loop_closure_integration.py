#!/usr/bin/env python3
"""Exercise loop correction and TF scheduling in an isolated ROS domain."""

import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--turn', action='store_true', help='Exercise a turn around a fixed body center')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '82' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Requires isolated ROS_DOMAIN_ID=82 and ROS_LOCALHOST_ONLY=1')
    # Jetson's loopback does not advertise multicast. Explicit local peers
    # make discovery work without ever joining the robot's network interface.
    os.environ['CYCLONEDDS_URI'] = (
        '<CycloneDDS><Domain Id="any"><General><AllowMulticast>false</AllowMulticast>'
        '</General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
        '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/go2_loop_isolated_logs')
    rclpy.init()
    workspace = Path(__file__).resolve().parents[3]
    binary = workspace / 'build/go2_lio_navigation/scan_context_loop_closure'
    launch_args = [
        str(binary), '--ros-args', '-p', 'odom_topic:=/loop_test/odom',
        '-p', 'cloud_topic:=/loop_test/cloud', '-p', 'map_frame:=loop_test_map',
        '-p', 'odom_frame:=loop_test_odom', '-p', 'keyframe_distance:=0.15',
        '-p', 'keyframe_time:=-1.0', '-p', 'recent_keyframe_exclusion:=6',
        '-p', 'min_loop_time:=0.0', '-p', 'min_loop_travel:=1.0',
        '-p', 'submap_half_window:=0', '-p', 'source_submap_keyframes:=1',
        '-p', 'loop_keyframe_cooldown:=3', '-p', 'loop_translation_sigma:=0.08',
    ]
    mount_x = .2821600512 if args.turn else 0.
    if args.turn:
        launch_args.extend([
            '-p', 'turn_revisits:=true', '-p', f'sensor_forward_offset:={mount_x}',
            '-p', 'min_loop_travel:=2.0',
            '-p', 'max_loop_translation_correction:=0.25', '-p', 'max_loop_rotation_correction_deg:=12.0',
        ])
    process = subprocess.Popen(launch_args)
    node = Node('loop_closure_isolated_fixture')
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    odom_pub = node.create_publisher(Odometry, '/loop_test/odom', 10)
    cloud_pub = node.create_publisher(PointCloud2, '/loop_test/cloud', 10)
    state = {'path': None, 'accepted': [], 'tf_times': [], 'tf': None}

    def on_status(m):
        if m.data.startswith('loop_accepted'):
            state['accepted'].append(m.data)

    def on_tf(m):
        for t in m.transforms:
            if t.header.frame_id == 'loop_test_map':
                state['tf_times'].append(time.monotonic())
                state['tf'] = t

    node.create_subscription(NavPath, '/loop_closure/corrected_path',
                             lambda m: state.update(path=m), durable)
    node.create_subscription(String, '/loop_closure/status', on_status, durable)
    node.create_subscription(TFMessage, '/tf', on_tf, 100)

    def until(predicate, seconds=8):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate():
                return
        raise AssertionError('Timed out waiting for isolated loop node')

    a, b = np.meshgrid(np.arange(-1, 3, .12), np.arange(-.8, 1.6, .12))
    floor = np.c_[a.ravel(), b.ravel(), np.full(a.size, -.8)]
    wall_x = np.c_[np.full(a.size, 3.0), a.ravel(), b.ravel()]
    wall_y = np.c_[a.ravel(), np.full(a.size, 2.5), b.ravel()]
    room = np.concatenate([floor, wall_x, wall_y]).astype('<f4')
    try:
        until(lambda: odom_pub.get_subscription_count() > 0 and
              cloud_pub.get_subscription_count() > 0)
        route = [0., .4, .8, 1.2, 1.6, 2., 1.6, 1.2, .8, .4, 0.]
        if args.turn:
            route = np.deg2rad(np.arange(0., 361., 20.))
        elapsed_base = node.get_clock().now().nanoseconds
        for index, value in enumerate(route):
            stamp = Time(nanoseconds=elapsed_base + index * 340_000_000).to_msg()
            odometry = Odometry()
            odometry.header.stamp, odometry.header.frame_id = stamp, 'loop_test_odom'
            odometry.child_frame_id = 'loop_test_sensor'
            yaw = value if args.turn else 0.
            heading_drift = math.radians(5.) * index / (len(route)-1) if args.turn else 0.
            heading = yaw + heading_drift
            half_yaw = heading / 2.
            odometry.pose.pose.orientation.z = math.sin(half_yaw)
            odometry.pose.pose.orientation.w = math.cos(half_yaw)
            true_x = 0. if args.turn else value
            drift = .18 * index / (len(route)-1) if args.turn else .06 * index
            odometry.pose.pose.position.x = true_x + drift + mount_x * math.cos(heading)
            odometry.pose.pose.position.y = mount_x * math.sin(heading)
            cloud = PointCloud2()
            cloud.header.stamp, cloud.header.frame_id = stamp, 'loop_test_sensor'
            c, s = math.cos(yaw), math.sin(yaw)
            rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
            points = ((room - [true_x, 0., 0.]) @ rotation - [mount_x, 0., 0.]).astype('<f4')
            cloud.height, cloud.width = 1, len(points)
            cloud.fields = [PointField(name=name, offset=4*i, datatype=7, count=1)
                            for i, name in enumerate(('x', 'y', 'z'))]
            cloud.point_step, cloud.row_step = 12, 12*len(points)
            cloud.is_dense = True
            cloud.data = points.tobytes()
            odom_pub.publish(odometry)
            cloud_pub.publish(cloud)
            until(lambda: state['path'] is not None and len(state['path'].poses) >= index+1)
        until(lambda: len(state['accepted']) > 0 and state['tf'] is not None)
        end = time.monotonic()+.5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        final = state['path'].poses[-1].pose
        correction = state['tf'].transform.translation.x
        if args.turn:
            q = final.orientation
            yaw_error = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            center_error = math.hypot(final.position.x-mount_x*math.cos(yaw_error),
                                      final.position.y-mount_x*math.sin(yaw_error))
            assert any('type=turn' in line for line in state['accepted']), state['accepted']
            assert center_error < .09, (center_error, state['accepted'])
            assert abs(math.degrees(yaw_error)) < 3., (yaw_error, state['accepted'])
            print(f'PASS in-place turn reduced 0.180 m / 5.0° injected drift to '
                  f'{center_error:.3f} m / {abs(math.degrees(yaw_error)):.2f}°', flush=True)
        else:
            assert abs(final.position.x) < .15, (final.position.x, state['accepted'])
            assert correction < -.40, correction
            print(f'PASS graph endpoint {final.position.x:.3f} m after 0.600 m injected drift; '
                  f'map correction {correction:.3f} m', flush=True)
        gaps = np.diff(state['tf_times'])
        assert len(gaps) > 5 and gaps.max() < .30, gaps.tolist()
        print(f'{len(state["accepted"])} accepted loops; max TF gap {gaps.max():.3f} s', flush=True)
        print('\n'.join(state['accepted']), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)


if __name__ == '__main__':
    main()
