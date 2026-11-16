#!/usr/bin/env python3
"""Check persistent mapping, pose correction, health gate, and export via DDS."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from environment_mapper import cloud_message, xyz_from_cloud  # noqa: E402


class Fixture(Node):
    def __init__(self):
        super().__init__('environment_mapping_fixture')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.valid = self.create_publisher(Bool, '/lio/localization_valid', latched)
        self.clear=self.create_publisher(PointCloud2,'/safety/cleared_obstacle_volumes',10)
        self.path = self.create_publisher(NavPath, '/loop_closure/corrected_path', latched)
        self.body = self.create_publisher(
            PointCloud2, '/cloud_registered_body', qos_profile_sensor_data)
        self.grid = self.surface = self.cloud = None
        self.create_subscription(OccupancyGrid, '/mapping/environment_map', self.on_grid, latched)
        self.create_subscription(PointCloud2, '/mapping/terrain_surface', self.on_surface, latched)
        self.create_subscription(PointCloud2, '/mapping/environment_cloud', self.on_cloud, latched)
        self.save = self.create_client(Trigger, '/mapping/save')

    def on_grid(self, message):
        self.grid = message

    def on_surface(self, message):
        self.surface = message

    def on_cloud(self, message):
        self.cloud = message

    def until(self, predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError('Timed out waiting for environment mapper')

    def publish_frame(self, poses, stamp, points):
        path = NavPath()
        path.header.frame_id = 'map'
        for key, x in poses:
            pose = PoseStamped()
            pose.header.frame_id, pose.header.stamp = 'map', key
            pose.pose.position.x, pose.pose.orientation.w = x, 1.0
            path.poses.append(pose)
        # Publish repeatedly until the sensor-data subscription has discovered us.
        for _ in range(10):
            self.body.publish(cloud_message(points, stamp, 'livox_frame'))
            self.path.publish(path)
            rclpy.spin_once(self, timeout_sec=0.04)


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '82' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Requires isolated ROS_DOMAIN_ID=82 and ROS_LOCALHOST_ONLY=1')
    os.environ['CYCLONEDDS_URI']=(
        '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
        '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/>'
        '</Peers></Discovery></Domain></CycloneDDS>')
    root = Path(__file__).resolve().parents[1]
    output = tempfile.mkdtemp(prefix='go2_environment_validation_')
    process = subprocess.Popen([
        '/usr/bin/python3', str(root / 'scripts/environment_mapper.py'), '--ros-args',
        '-p', 'output_directory:=' + output, '-p', 'update_rate:=5.0',
    ])
    rclpy.init()
    fixture = Fixture()
    try:
        fixture.until(fixture.save.service_is_ready)
        fixture.until(lambda: fixture.valid.get_subscription_count() > 0)
        fixture.valid.publish(Bool(data=True))
        fixture.until(lambda: fixture.body.get_subscription_count() > 0)
        points = np.array([[0.82 + i*0.30, 0.02 + j*0.30 + dy, -0.78]
                           for i in range(3) for j in range(3) for dy in (0.0, 0.10)],
                          dtype=np.float32)
        first = fixture.get_clock().now().to_msg()
        fixture.publish_frame([(first, 0.0)], first, points)
        fixture.until(lambda: fixture.surface is not None and fixture.surface.width >= 9)
        assert np.allclose(xyz_from_cloud(fixture.surface)[:, 2], -0.78)
        print('PASS real-height terrain publication', flush=True)
        second = fixture.get_clock().now().to_msg()
        fixture.publish_frame([(first, 0.0), (second, 2.0)], second, points)
        fixture.until(lambda: fixture.cloud is not None and
                      xyz_from_cloud(fixture.cloud)[:, 0].max() > 3.4)
        assert xyz_from_cloud(fixture.cloud)[:, 0].min() < 1.0
        print('PASS old area retained after moving', flush=True)
        fixture.publish_frame([(first, 0.0), (second, 1.0)], second, points)
        fixture.until(lambda: 2.4 < xyz_from_cloud(fixture.cloud)[:, 0].max() < 2.6)
        print('PASS corrected keyframe rebuilt without stale pose ghost', flush=True)
        third=fixture.get_clock().now().to_msg()
        scene=np.concatenate([points,[[.82,.02,-.40],[.82,.02,.10]]])
        fixture.publish_frame([(first,0.),(second,1.),(third,0.)],third,scene)
        fixture.until(lambda:fixture.cloud is not None and
                      np.any(xyz_from_cloud(fixture.cloud)[:,2]>-.7))
        fixture.until(lambda:fixture.clear.get_subscription_count()>0)
        removal=PointCloud2();removal.header.frame_id='map'
        removal.header.stamp=fixture.get_clock().now().to_msg()
        removal.height,removal.width,removal.point_step,removal.row_step=1,1,20,20
        removal.fields=[PointField(name=k,offset=i*4,datatype=7,count=1)
                        for i,k in enumerate(('x','y','z_min','z_max','radius'))]
        removal.data=np.array([[.82,.02,-.45,.15,.04]],dtype='<f4').tobytes()
        fixture.clear.publish(removal)
        fixture.until(lambda:np.allclose(xyz_from_cloud(fixture.cloud)[:,2],-.78))
        print('PASS confirmed moving-object clearance removes reference ghosts while keeping floor',flush=True)
        # A graph correction must not resurrect the pruned local scan points.
        fixture.publish_frame([(first,0.),(second,1.),(third,.1)],third,scene)
        fixture.until(lambda:np.any(np.isclose(xyz_from_cloud(fixture.cloud)[:,0],.9,atol=.07)))
        assert np.allclose(xyz_from_cloud(fixture.cloud)[:,2],-.78)
        print('PASS loop correction preserves previously confirmed dynamic-object removal',flush=True)
        future = fixture.save.call_async(Trigger.Request())
        fixture.until(future.done)
        response = future.result()
        assert response.success, response.message
        saved = Path(response.message)
        assert {'map.pgm', 'map.yaml', 'environment.pcd', 'elevation.npz',
                'metadata.json'} <= {p.name for p in saved.iterdir()}
        with np.load(saved / 'elevation.npz') as layers:
            assert np.allclose(layers['elevation_m'][np.isfinite(layers['elevation_m'])], -0.78)
        assert json.loads((saved / 'metadata.json').read_text())['relocalization_ready'] is False
        assert (saved / 'map.pgm').read_bytes().startswith(b'P5\n')
        fixture.valid.publish(Bool(data=False))
        # Drain prior queued publications, then ensure invalid localization pauses integration.
        until = time.monotonic() + 0.6
        while time.monotonic() < until:
            rclpy.spin_once(fixture, timeout_sec=0.03)
        previous = fixture.cloud.header.stamp
        until = time.monotonic() + 0.6
        while time.monotonic() < until:
            rclpy.spin_once(fixture, timeout_sec=0.03)
        assert fixture.cloud.header.stamp == previous
        print('PASS localization health gate; exported files: ' + str(saved), flush=True)
    finally:
        fixture.destroy_node()
        rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)


if __name__ == '__main__':
    main()
