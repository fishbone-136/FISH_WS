#!/usr/bin/env python3
"""Exercise low-obstacle sensing, planning, replanning and stopping on localhost."""

import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, TransformStamped, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from map_msgs.msg import OccupancyGridUpdate
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
import numpy as np
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
                       'go2_lio_navigation/scripts'))
from environment_mapper import cloud_message  # noqa: E402


class Fixture(Node):
    def __init__(self):
        super().__init__('obstacle_pipeline_fixture')
        self.box_x = None
        self.box_width = 0.44
        self.box_heights = None
        self.extra_points = None
        self.box_y = 0.0
        self.floor_z = -0.78
        self.enabled = True
        self.lio_enabled = True
        self.command_enabled = False
        self.command_linear = 0.55
        self.command_angular = 0.0
        self.synthetic_footprint = True
        self.simulate_motion = False
        self.last_trace_time = 0.
        self.x = self.y = self.yaw = 0.0
        self.map_shift = 0.0
        self.frame_stamp=None
        self.tf_lag = 0.0
        self.crossing_clearances = []
        config = yaml.safe_load((Path(get_package_share_directory('go2_navigation')) /
                                 'config/nav2_params.yaml').read_text())
        self.goal_tolerance=config['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance']
        costmap = config['local_costmap']['local_costmap']['ros__parameters']
        footprint = np.asarray(yaml.safe_load(costmap['footprint']))
        self.half_length = float(np.max(np.abs(footprint[:, 0])))
        self.half_width = float(np.max(np.abs(footprint[:, 1])))
        self.padded_length = self.half_length + costmap['footprint_padding']
        self.padded_width = self.half_width + costmap['footprint_padding']
        self.protection_margin = float(costmap['footprint_padding'])
        self.body_clearances = []
        self.scan_override = None
        self.turn_valid = None
        self.forward_coverage = True
        self.safe_history = []
        self.raw_history = []
        self.raw_command = Twist()
        self.monitor_command = Twist()
        self.scan = self.safe = self.costmap = self.fast_scan = None
        self.plans = []
        self.tf = TransformBroadcaster(self)
        self.cloud = self.create_publisher(PointCloud2, '/test/cloud', QoSProfile(depth=1))
        self.fast_cloud = self.create_publisher(PointCloud2, '/test/cloud_fast', QoSProfile(depth=1))
        self.direct_scan = self.create_publisher(LaserScan, '/scan', 1)
        self.direct_fast_scan = self.create_publisher(LaserScan, '/scan_safety', 1)
        self.command = self.create_publisher(Twist, '/cmd_vel', 1)
        self.odom = self.create_publisher(Odometry, '/odom', 1)
        self.footprint = self.create_publisher(
            PolygonStamped, '/local_costmap/published_footprint', 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.local = self.create_publisher(OccupancyGrid, '/traversability/local', latched)
        self.global_map = self.create_publisher(OccupancyGrid, '/traversability/global', latched)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan_safety',
            lambda m:setattr(self,'fast_scan',m),qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel_guarded', self.on_safe, 10)
        self.create_subscription(Bool, '/safety/turn_clearance_valid',
                                 lambda m: setattr(self, 'turn_valid', m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        def raw_command(m):
            self.raw_command = m
            self.raw_history.append((time.monotonic(), m.linear.x, m.angular.z))
        self.create_subscription(Twist, '/cmd_vel_nav', raw_command, 10)
        self.create_subscription(
            Twist, '/cmd_vel_safe', lambda m: setattr(self, 'monitor_command', m), 10)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.on_costmap, latched)
        self.create_subscription(
            OccupancyGridUpdate, '/global_costmap/costmap_updates', self.on_update, 5)
        self.create_subscription(NavPath, '/plan', lambda m: self.plans.append(m), 5)
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.navigator = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.frame_dt = 1./15.
        self.create_timer(self.frame_dt, self.step)
        self.create_timer(0.05, self.send_command)
        self.floor_points = np.array([[x, y] for x in np.arange(0.5, 4.6, 0.12)
                                      for y in np.arange(-2.5, 2.6, 0.12)])

    def on_scan(self, message):
        self.scan = message

    def on_safe(self, message):
        self.safe = message
        self.safe_history.append((time.monotonic(), message.linear.x, message.angular.z))

    def on_costmap(self, message):
        self.costmap = message

    def on_update(self, message):
        if self.costmap is None:
            return
        grid = np.asarray(self.costmap.data).reshape(
            self.costmap.info.height, self.costmap.info.width).copy()
        grid[message.y:message.y + message.height,
             message.x:message.x + message.width] = np.asarray(message.data).reshape(
                 message.height, message.width)
        self.costmap.data = grid.ravel().tolist()

    def send_command(self):
        if self.command_enabled:
            message = Twist()
            message.linear.x = self.command_linear
            message.angular.z = self.command_angular
            self.command.publish(message)

    def step(self):
        if self.simulate_motion and time.monotonic()-self.last_trace_time > 2.:
            self.last_trace_time = time.monotonic()
            print('SIM pose=(%.3f,%.3f,%.3f) raw=(%.3f,%.3f) safe=(%.3f,%.3f)' % (
                self.x,self.y,self.yaw,self.raw_command.linear.x,self.raw_command.angular.z,
                self.safe.linear.x if self.safe else 0.,
                self.safe.angular.z if self.safe else 0.),flush=True)
        if self.simulate_motion and self.safe is not None:
            self.yaw += self.safe.angular.z * self.frame_dt
            self.x += self.safe.linear.x * math.cos(self.yaw) * self.frame_dt
            self.y += self.safe.linear.x * math.sin(self.yaw) * self.frame_dt
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            corners = np.array([[self.padded_length, self.padded_width],
                                [self.padded_length, -self.padded_width],
                                [-self.padded_length, -self.padded_width],
                                [-self.padded_length, self.padded_width]])
            corners = corners @ np.array([[c, s], [-s, c]]) + (self.x, self.y)
            crossings = []
            for a, b in zip(corners, np.roll(corners, -1, axis=0)):
                if min(a[0], b[0]) <= self.box_x <= max(a[0], b[0]) and abs(b[0]-a[0]) > 1.e-9:
                    fraction = (self.box_x-a[0])/(b[0]-a[0])
                    crossings.append(a[1] + fraction*(b[1]-a[1]))
            if crossings:
                margin = max(min(crossings)-self.box_y, self.box_y-max(crossings)) - self.box_width / 2
                assert margin >= 0.0, 'Swept padded body intersects the person obstacle'
                self.crossing_clearances.append(margin)
            physical = np.array([[self.half_length, self.half_width],
                                 [self.half_length, -self.half_width],
                                 [-self.half_length, -self.half_width],
                                 [-self.half_length, self.half_width]])
            physical = physical @ np.array([[c, s], [-s, c]]) + (self.x, self.y)
            crossings = []
            for a, b in zip(physical, np.roll(physical, -1, axis=0)):
                if min(a[0], b[0]) <= self.box_x <= max(a[0], b[0]) and abs(b[0]-a[0]) > 1.e-9:
                    fraction = (self.box_x-a[0])/(b[0]-a[0])
                    crossings.append(a[1] + fraction*(b[1]-a[1]))
            if crossings:
                self.body_clearances.append(
                    max(min(crossings)-self.box_y, self.box_y-max(crossings)) - self.box_width / 2)
        stamp = (self.get_clock().now() - rclpy.duration.Duration(seconds=0.03)).to_msg()
        self.frame_stamp=stamp
        transforms = []
        for parent, child in [('map', 'odom_lio'), ('odom_lio', 'base_link')]:
            transform = TransformStamped()
            transform.header.stamp = (rclpy.time.Time.from_msg(stamp)-
                                     rclpy.duration.Duration(seconds=self.tf_lag)).to_msg()
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.0
            if parent == 'map':
                transform.transform.translation.x = self.map_shift
            if child == 'base_link':
                transform.transform.translation.x = self.x
                transform.transform.translation.y = self.y
                transform.transform.rotation.z = math.sin(self.yaw / 2)
                transform.transform.rotation.w = math.cos(self.yaw / 2)
            transforms.append(transform)
        self.tf.sendTransform(transforms)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'odom_lio'
        odom.child_frame_id, odom.pose.pose.orientation.w = 'base_link', 1.0
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2)
        if self.simulate_motion and self.safe is not None:
            odom.twist.twist = self.safe
        self.odom.publish(odom)
        if self.synthetic_footprint:
            shape = PolygonStamped()
            shape.header.stamp, shape.header.frame_id = stamp, 'base_link'
            shape.polygon.points = [Point32(x=x, y=y, z=0.0) for x, y in (
                (self.padded_length, self.padded_width),
                (self.padded_length, -self.padded_width),
                (-self.padded_length, -self.padded_width),
                (-self.padded_length, self.padded_width))]
            self.footprint.publish(shape)
        for pub, frame, size in [(self.local, 'odom_lio', 100), (self.global_map, 'map', 200)]:
            grid = OccupancyGrid()
            grid.header.stamp, grid.header.frame_id = stamp, frame
            grid.info.resolution, grid.info.width, grid.info.height = 0.1, size, size
            grid.info.origin.position.x = grid.info.origin.position.y = -size*0.05
            grid.info.origin.orientation.w = 1.0
            grid.data = [0] * (size*size)
            pub.publish(grid)
        if self.scan_override is not None:
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'base_link'
            scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi/900
            scan.range_min, scan.range_max = .1, 5.5
            scan.ranges = [float('nan')]*1801
            if self.forward_coverage:
                scan.ranges[899] = scan.ranges[900] = scan.ranges[901] = float('inf')
            for x, y in self.scan_override:
                index = round((math.atan2(y, x)-scan.angle_min)/scan.angle_increment)
                scan.ranges[index] = math.hypot(x, y)
            if self.lio_enabled: self.direct_scan.publish(scan)
            self.direct_fast_scan.publish(scan)
        elif self.enabled:
            points = np.column_stack([self.floor_points,
                                      np.full(len(self.floor_points), self.floor_z)])
            if self.extra_points is not None:points=np.concatenate([points,self.extra_points])
            if self.box_x is not None:
                box = np.array([[self.box_x, y, self.floor_z + h]
                                for y in np.arange(self.box_y - self.box_width/2,
                                                   self.box_y + self.box_width/2, 0.01)
                                for h in (self.box_heights if self.box_heights is not None
                                          else np.arange(0.11, 0.21, 0.025))])
                points = np.concatenate([points, box])
                if self.simulate_motion:
                    relative = box[:, :2] - (self.x, self.y)
                    c, s = math.cos(self.yaw), math.sin(self.yaw)
                    relative = relative @ np.array([[c, -s], [s, c]])
                    assert not np.any((np.abs(relative[:, 0]) < self.padded_length) &
                                      (np.abs(relative[:, 1]) < self.padded_width)), 'Gait envelope hit obstacle'
            if self.simulate_motion:
                points[:, :2] -= (self.x, self.y)
                c, s = math.cos(self.yaw), math.sin(self.yaw)
                points[:, :2] = points[:, :2] @ np.array([[c, -s], [s, c]])
                points = points[np.abs(np.arctan2(points[:, 1], points[:, 0])) < math.pi / 3]
            message = cloud_message(points, stamp, 'base_link')
            if self.lio_enabled: self.cloud.publish(message)
            self.fast_cloud.publish(message)

    def until(self, predicate, timeout=12):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        details = ''
        if self.scan is not None:
            r = np.asarray(self.scan.ranges)
            details = ' (height=%s box=%s scan finite=%d inf=%d nan=%d)' % (
                self.floor_z, self.box_x, np.isfinite(r).sum(),
                np.isinf(r).sum(), np.isnan(r).sum())
        raise AssertionError('Timed out in isolated obstacle pipeline' + details)

    def lifecycle(self, transition):
        client = self.create_client(ChangeState, '/collision_monitor/change_state')
        self.until(client.service_is_ready)
        request = ChangeState.Request()
        request.transition.id = transition
        future = client.call_async(request)
        self.until(future.done)
        assert future.result().success

    def goal_pose(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = 3.031+self.map_shift, .017
        goal.pose.orientation.z = 1.0  # Requested heading opposite the arrival direction.
        return goal

    def plan(self, target=None):
        self.until(self.planner.server_is_ready)
        client = self.create_client(GetState, '/planner_server/get_state')
        self.until(client.service_is_ready)
        for _ in range(40):
            state = client.call_async(GetState.Request())
            self.until(state.done)
            if state.result().current_state.label == 'active':
                break
            ready = time.monotonic() + 0.1
            self.until(lambda: time.monotonic() > ready)
        assert state.result().current_state.label == 'active'
        request = ComputePathToPose.Goal()
        request.goal, request.planner_id = target or self.goal_pose(), 'GridBased'
        future = self.planner.send_goal_async(request)
        self.until(future.done)
        assert future.result().accepted
        result = future.result().get_result_async()
        self.until(result.done)
        assert result.result().status == 4, result.result()
        return result.result().result.path


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '83' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Requires isolated ROS_DOMAIN_ID=83 and ROS_LOCALHOST_ONLY=1')
    os.environ['CYCLONEDDS_URI'] = (
        '<CycloneDDS><Domain Id="any"><General><AllowMulticast>false</AllowMulticast>'
        '</General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
        '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/go2_obstacle_pipeline_logs')
    processes = []
    fixture = None
    try:
        prefix = get_package_prefix('go2_perception')
        collision_config = str(Path(get_package_share_directory('go2_lio_navigation')) /
                               'config/collision_monitor.yaml')
        processes.append(subprocess.Popen([
            str(Path(prefix) / 'lib/go2_perception/pointcloud_to_laserscan_node'),
            '--ros-args', '-r', 'cloud_in:=/test/cloud', '-p', 'target_frame:=base_link',
            '-r','obstacle_points:=/obstacles/deskewed',
            '-r','ground_plane:=/perception/deskewed_ground_plane',
            '-p','ground_pose_topic:=/odom',
            '-p', 'ground_filter_enabled:=true', '-p', 'observed_free_space_only:=true',
            '-p', 'use_best_effort:=false', '-p', 'queue_size:=1',
            '-p', 'range_max:=5.5', '-p', 'range_min:=0.25',
            '-p', 'angle_increment:=0.00698', '--log-level', 'warn']))
        processes.append(subprocess.Popen([
            str(Path(prefix) / 'lib/go2_perception/pointcloud_to_laserscan_node'),
            '--ros-args', '-r', '__node:=safety_pointcloud_to_laserscan',
            '-r', 'cloud_in:=/test/cloud_fast', '-r', 'scan:=/scan_safety',
            '-p','ground_pose_topic:=/odom',
            '-p', 'target_frame:=base_link', '-p', 'ground_filter_enabled:=true',
            '-p', 'observed_free_space_only:=true', '-p', 'use_best_effort:=false',
            '-p', 'queue_size:=1', '-p', 'range_max:=5.5', '-p', 'range_min:=0.25',
            '-p', 'angle_increment:=0.00698', '--log-level', 'warn']))
        processes.append(subprocess.Popen([
            '/opt/ros/humble/lib/nav2_collision_monitor/collision_monitor',
            '--ros-args', '--params-file', collision_config, '--log-level', 'warn']))
        gate_script = (Path(__file__).resolve().parents[2] /
                       'go2_lio_navigation/scripts/scan_safety_gate.py')
        processes.append(subprocess.Popen([
            '/usr/bin/python3', str(gate_script),
            '--ros-args', '-p', 'memory_required:=false', '--log-level', 'warn']))
        rclpy.init()
        fixture = Fixture()
        fixture.lifecycle(Transition.TRANSITION_CONFIGURE)
        fixture.lifecycle(Transition.TRANSITION_ACTIVATE)
        fixture.until(lambda: fixture.scan is not None)
        for height in [-0.78, -0.37]:
            fixture.floor_z = height
            floor_changed = fixture.get_clock().now().nanoseconds
            fixture.until(lambda: rclpy.time.Time.from_msg(fixture.scan.header.stamp).nanoseconds > floor_changed and
                          np.isfinite(fixture.scan.ranges).sum() == 0 and
                          np.isinf(fixture.scan.ranges).sum() >= 10)
            fixture.box_x = 1.8
            fixture.until(lambda: np.isfinite(fixture.scan.ranges).sum() > 8)
            assert 1.7 < np.nanmin(fixture.scan.ranges) < 1.9
            assert math.isnan(fixture.scan.ranges[0]), 'Rear blind sector falsely cleared'
            fixture.box_x = None
            fixture.until(lambda: np.isfinite(fixture.scan.ranges).sum() == 0 and
                          np.isinf(fixture.scan.ranges).sum() >= 10)
        print('PASS low 10-20 cm obstacle detected at both standing heights; blind rear unknown',
              flush=True)
        fixture.extra_points=[[-1.,.03,fixture.floor_z-.30]]
        changed=fixture.get_clock().now().nanoseconds
        fixture.until(lambda:rclpy.time.Time.from_msg(fixture.scan.header.stamp).nanoseconds>changed)
        index=int((math.atan2(.03,-1.)-fixture.scan.angle_min)/fixture.scan.angle_increment)
        assert math.isnan(fixture.scan.ranges[index]), 'Lower return became an occupied wall or free-space proof'
        fixture.extra_points=None
        print('PASS below-floor return stays unknown in 2D; 2.5D retains negative terrain geometry',flush=True)
        fixture.command_enabled = True
        fixture.until(lambda: fixture.safe is not None and fixture.safe.linear.x > 0.35)
        fixture.box_x = 1.0
        fixture.until(lambda: 0.0 < fixture.safe.linear.x < 0.4)
        print('PASS velocity-dependent approach slows before the emergency zone', flush=True)
        fixture.box_x = None
        fixture.until(lambda: fixture.safe.linear.x > 0.45)
        obstacle_started = time.monotonic()
        fixture.box_x = fixture.padded_length-.03
        fixture.until(lambda: fixture.safe.linear.x == 0.0)
        stop_latency = time.monotonic() - obstacle_started
        assert stop_latency < 0.25, 'Late dynamic-obstacle stop command'
        print('PASS sudden obstacle stop command after %.3f s (simulated sensor)' % stop_latency,
              flush=True)
        fixture.box_x = None
        fixture.until(lambda: fixture.safe.linear.x > .45)
        fixture.lio_enabled = False
        ready = time.monotonic()+.20
        fixture.until(lambda: time.monotonic() >= ready)
        old_stamp = fixture.scan.header.stamp
        obstacle_started = time.monotonic()
        fixture.box_x = fixture.padded_length-.03
        fixture.until(lambda: fixture.safe.linear.x == 0.)
        assert fixture.scan.header.stamp == old_stamp, 'Deskewed lane did not stall'
        assert time.monotonic()-obstacle_started < .25, 'Fast lane waited for FAST-LIO'
        print('PASS raw safety lane stops an intrusion while deskewed scans are stalled', flush=True)
        fixture.lio_enabled = True
        fixture.box_x = None
        fixture.command_linear, fixture.command_angular = 0.0, 1.4
        fixture.until(lambda: abs(fixture.safe.angular.z) > 1.35)
        print('PASS clear scene permits the increased 1.40 rad/s checked turn',flush=True)
        fixture.box_width = 0.80
        fixture.box_x = 0.72
        # The approaching person's nearest spread point enters the caution
        # band. Preserve its slower turn, rather than demanding full yaw speed.
        fixture.until(lambda: 0.8 < abs(fixture.safe.angular.z) < 1.1)
        print('PASS person at 0.72 m permits a cautious turn outside the expanded gait sweep',
              flush=True)
        fixture.box_x = fixture.padded_length+.01
        fixture.until(lambda: fixture.safe.angular.z == 0.0)
        print('PASS safe turn away; close swept-footprint hazard still stops',
              flush=True)
        fixture.scan_override = [(fixture.padded_length-.02, 0.)]
        fixture.until(lambda: fixture.scan is not None and
                      np.isfinite(fixture.scan.ranges).sum() == 1 and
                      fixture.safe.angular.z == 0.)
        print('PASS one single laser return in the stop zone stops a pure turn', flush=True)
        fixture.command_angular = .65
        fixture.scan_override = [(fixture.padded_length-.11, fixture.padded_width+.06)]
        fixture.until(lambda: fixture.turn_valid is False and fixture.safe.angular.z == 0.)
        fixture.command_angular = -.65
        fixture.until(lambda: fixture.safe.angular.z < -.5)
        print('PASS one point in the future corner sweep blocks turning into it but permits away',
              flush=True)
        fixture.scan_override = [(x,side*(fixture.padded_width+.03))
                                 for x in (.60,.80,1.) for side in (-1,1)]
        fixture.command_linear, fixture.command_angular = .3, 0.
        fixture.until(lambda: .22 <= fixture.safe.linear.x <= .265)
        print('PASS %.0f cm corridor permits cautious straight motion with the %.0f cm protected footprint' %
              (200*(fixture.padded_width+.03),200*fixture.padded_width),flush=True)
        fixture.scan_override = [(fixture.padded_length+.005,fixture.padded_width+.005)]
        fixture.command_linear, fixture.command_angular = .25,-.65
        fixture.until(lambda: fixture.safe.linear.x > .10 and fixture.safe.angular.z < -.20)
        fixture.command_angular = .65
        fixture.until(lambda: fixture.safe.linear.x == 0. and fixture.safe.angular.z == 0.)
        print('PASS close side obstacle permits an away arc but blocks turning into it',flush=True)
        fixture.command_linear, fixture.command_angular = .3, 0.
        fixture.scan_override = []
        # Exercise the final gate's sparse-frame contract with a fresh input
        # stream. The upstream collision monitor has its own TF waits and
        # may correctly stop on missing observations; test that separately.
        fixture.lifecycle(Transition.TRANSITION_DEACTIVATE)
        fixture.destroy_publisher(fixture.command)
        fixture.command=fixture.create_publisher(Twist,'/cmd_vel_safe',1)
        fixture.until(lambda: fixture.safe.linear.x > .25)
        first = len(fixture.safe_history)
        fixture.forward_coverage = False
        deadline = time.monotonic()+.12
        fixture.until(lambda: time.monotonic() >= deadline)
        samples = fixture.safe_history[first:]
        assert samples and all(x[1] > .25 for x in samples), samples
        print('PASS short sparse forward frame does not inject a zero-speed pulse', flush=True)
        # The yaw case needs its own fresh coverage budget. A fixed 120 ms
        # pause followed by DDS/TF waits can legitimately exhaust 350 ms.
        refresh=fixture.get_clock().now().nanoseconds
        fixture.forward_coverage=True
        fixture.until(lambda:fixture.fast_scan is not None and
            fixture.fast_scan.header.stamp.sec*1000000000+fixture.fast_scan.header.stamp.nanosec>=refresh and
            np.isinf(fixture.fast_scan.ranges).sum()>=3)
        settled=time.monotonic()+.06
        fixture.until(lambda:time.monotonic()>=settled and fixture.safe.linear.x>.25)
        refresh=fixture.get_clock().now().nanoseconds
        fixture.forward_coverage=False
        fixture.until(lambda:fixture.fast_scan.header.stamp.sec*1000000000+
            fixture.fast_scan.header.stamp.nanosec>=refresh and np.isnan(fixture.fast_scan.ranges).all())
        first = len(fixture.safe_history)
        fixture.command_angular = .35
        fixture.until(lambda: fixture.monitor_command.angular.z > .2,timeout=.15)
        deadline = time.monotonic()+.03
        fixture.until(lambda: time.monotonic() >= deadline)
        samples = fixture.safe_history[first:]
        assert samples and all(x[1]>.25 and abs(x[2])<1.e-6 for x in samples),samples
        fixture.command_angular = 0.
        print('PASS sparse-frame yaw update holds the unseen axis at zero without stopping the forward gait',flush=True)
        missing_started = time.monotonic()
        fixture.until(lambda: fixture.safe.linear.x == 0., timeout=1.)
        assert time.monotonic()-missing_started < .4
        print('PASS continuing sparse frames cannot refresh expired forward coverage', flush=True)
        fixture.forward_coverage = True
        fixture.until(lambda: fixture.safe.linear.x > .25)
        fixture.destroy_publisher(fixture.command)
        fixture.command=fixture.create_publisher(Twist,'/cmd_vel',1)
        fixture.lifecycle(Transition.TRANSITION_ACTIVATE)
        fixture.until(lambda:fixture.safe.linear.x>.25)
        fixture.scan_override = None
        fixture.command_linear, fixture.command_angular = 0.55, 0.0
        fixture.box_width = 0.44
        fixture.box_x = None
        fixture.until(lambda: fixture.safe.linear.x > 0.35)
        fixture.enabled = False
        fixture.until(lambda: fixture.safe.linear.x == 0.0, timeout=3)
        print('PASS independent near-obstacle stop, release, and stale-sensor stop', flush=True)
        fixture.enabled = True
        fixture.until(lambda: fixture.safe.linear.x > 0.35)
        fixture.synthetic_footprint = False
        fixture.until(lambda: fixture.safe.linear.x == 0.0, timeout=3)
        print('PASS missing/stale footprint also blocks unchecked movement', flush=True)
        fixture.command_enabled = False
        fixture.enabled = True
        with_memory='--current-scan-layer' in sys.argv
        if with_memory:
            # CurrentObstacleLayer is deployed with independent persistent
            # memory. Exercise that full topology, including blind geometry
            # during the simulated detour, rather than removing its memory.
            health=fixture.create_publisher(Bool,'/lio/localization_valid',
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            fixture.create_timer(.05,lambda:health.publish(Bool(data=True)))
            memory_script=Path(get_package_prefix('go2_lio_navigation'))/'lib/go2_lio_navigation/obstacle_memory.py'
            processes.append(subprocess.Popen(['/usr/bin/python3',str(memory_script),
                '--ros-args','-p','raw_cloud_topic:=/test/cloud','-p','full_pose_topic:=/odom']))
        nav_command=[
            'ros2', 'launch', 'go2_navigation', 'go2_nav2.launch.py',
            'mapping_mode:=true', 'lio_mode:=true',
            'use_obstacle_memory:='+str(with_memory).lower(), 'log_level:=error']
        processes.append(subprocess.Popen(nav_command,start_new_session=True))
        direct = fixture.plan()
        assert max(abs(p.pose.position.y) for p in direct.poses) < 0.2
        same_cell = fixture.goal_pose()
        same_cell.pose.position.x = same_cell.pose.position.y = .079
        exact = fixture.plan(same_cell)
        assert abs(exact.poses[-1].pose.position.x-.079) < 1.e-6
        assert abs(exact.poses[-1].pose.position.y-.079) < 1.e-6
        print('PASS same-cell goal beyond 10 cm retains the exact clicked endpoint',flush=True)
        follow = ActionClient(fixture, FollowPath, '/follow_path')
        fixture.until(follow.server_is_ready)
        probe = FollowPath.Goal()
        probe.controller_id, probe.goal_checker_id = 'FollowPath', 'goal_checker'
        probe.path.header.frame_id = 'odom_lio'
        probe.path.header.stamp = fixture.get_clock().now().to_msg()
        start_pose, close_goal = PoseStamped(), PoseStamped()
        start_pose.header = close_goal.header = probe.path.header
        start_pose.pose.orientation.w = 1.
        close_goal.pose.position.x = fixture.goal_tolerance-.001
        close_goal.pose.orientation.z = 1.  # Goal yaw remains irrelevant.
        probe.path.poses = [start_pose, close_goal]
        before = len(fixture.raw_history)
        sent = follow.send_goal_async(probe)
        fixture.until(sent.done)
        assert sent.result().accepted
        complete = sent.result().get_result_async()
        fixture.until(complete.done)
        assert complete.result().status == 4
        deadline = time.monotonic()+.2
        fixture.until(lambda: time.monotonic() >= deadline)
        samples = fixture.raw_history[before:]
        assert any(v > .1 and abs(w) < 1.e-6 for _, v, w in samples), samples
        assert samples[-1][1:] == (0., 0.), samples
        print('PASS strict goal checker owns the arrival stop; no independent pre-success zero pulse', flush=True)
        fixture.box_width, fixture.box_y = 0.025, 0.04
        fixture.box_x = 1.8
        fixture.until(lambda: fixture.costmap is not None and
                      any(c >= 99 for c in fixture.costmap.data))
        thin_detour = fixture.plan()
        assert max(abs(p.pose.position.y) for p in thin_detour.poses) > 0.25
        print('PASS thin single-cell chair-leg obstacle retained in global planning', flush=True)
        fixture.box_width, fixture.box_y = 0.44, 0.0
        # Two update periods ensure both scan marking and inflation have run.
        ready = time.monotonic() + 1.0
        fixture.until(lambda: time.monotonic() > ready)
        detour = fixture.plan()
        assert max(abs(p.pose.position.y) for p in detour.poses) > 0.35
        print('PASS actual obstacle layer and Smac planner route around the low obstacle',
              flush=True)
        # Reproduce a loop-closure correction while keeping physical geometry
        # fixed in odom. A map-frame obstacle layer would now retain old cells
        # and mark a second shifted obstacle. The actual planner must keep its
        # continuous frame, including transformation of the map-frame goal.
        assert fixture.costmap.header.frame_id == 'odom_lio'
        fixture.map_shift = 2.0
        ready = time.monotonic()+1.5
        fixture.until(lambda: time.monotonic() > ready)
        shifted = fixture.plan()
        assert shifted.header.frame_id == 'odom_lio'
        assert abs(shifted.poses[-1].pose.position.x-3.031) < 1.e-6
        grid = np.asarray(fixture.costmap.data).reshape(
            fixture.costmap.info.height, fixture.costmap.info.width)
        res = fixture.costmap.info.resolution
        ox = fixture.costmap.info.origin.position.x
        oy = fixture.costmap.info.origin.position.y
        # No copied old-map obstacle at physical x=3.8 m after +2 m map shift.
        mx, my = int((3.8-ox)/res), int((0.-oy)/res)
        assert grid[my,mx] < 99, grid[my,mx]
        print('PASS 2 m loop correction leaves no shifted obstacle ghost; map-frame goal transforms correctly',
              flush=True)
        fixture.map_shift = 0.0
        ready = time.monotonic()+1.0
        fixture.until(lambda: time.monotonic() > ready)
        fixture.until(fixture.navigator.server_is_ready)
        goal = NavigateToPose.Goal()
        goal.pose = fixture.goal_pose()
        sent = fixture.navigator.send_goal_async(goal)
        fixture.until(sent.done)
        assert sent.result().accepted
        start_count = len(fixture.plans)
        fixture.until(lambda: len(fixture.plans) >= start_count + 4, timeout=8)
        shifted_count = len(fixture.plans)
        fixture.map_shift = 2.0
        fixture.until(lambda: len(fixture.plans) >= shifted_count+3, timeout=8)
        assert all(abs(p.poses[-1].pose.position.x-3.031)<1.e-6 for p in fixture.plans[shifted_count:] if p.poses)
        print('PASS active navigation goal stays fixed through a 2 m map correction',flush=True)
        fixture.map_shift = 0.0
        cancelled = sent.result().cancel_goal_async()
        fixture.until(cancelled.done)
        print('PASS NavigateToPose publishes repeated global plans while the goal is unchanged',
              flush=True)
        fixture.floor_points = np.array([[x, y] for x in np.arange(-5, 5.1, 0.12)
                                        for y in np.arange(-5, 5.1, 0.12)])
        fixture.until(lambda: abs(fixture.safe.linear.x) < 1.e-6 and
                      abs(fixture.safe.angular.z) < 1.e-6)
        fixture.box_x, fixture.box_width = 0.85, 0.80
        fixture.simulate_motion = True
        ready = time.monotonic() + 1.0
        fixture.until(lambda: time.monotonic() > ready)
        goal = NavigateToPose.Goal()
        goal.pose = fixture.goal_pose()
        motion_started = len(fixture.safe_history)
        sent = fixture.navigator.send_goal_async(goal)
        fixture.until(sent.done)
        assert sent.result().accepted
        result = sent.result().get_result_async()
        fixture.until(result.done, timeout=65)
        assert result.result().status == 4, (
            result.result(), (fixture.x, fixture.y, fixture.yaw),
            fixture.safe, fixture.raw_command)
        assert fixture.x > 2.5 and fixture.crossing_clearances
        goal_error = math.hypot(fixture.x-3.031, fixture.y-.017)
        assert goal_error <= fixture.goal_tolerance+1.e-6, goal_error
        assert abs(math.remainder(fixture.yaw-math.pi,2*math.pi)) > 1.0
        print('PASS strict original-goal arrival %.4f m with final heading ignored' % goal_error,flush=True)
        assert fixture.body_clearances and min(fixture.body_clearances) >= fixture.protection_margin-.001
        maximum_linear = max(x[1] for x in fixture.safe_history[motion_started:])
        assert maximum_linear >= .50, maximum_linear
        print('PASS closed-loop simulated Nav2 detour around near wide person; '
              'reached x=%.2f m, padded clearance %.2f m, stance clearance %.2f m, peak speed %.2f m/s' %
              (fixture.x, min(fixture.crossing_clearances), min(fixture.body_clearances),
               maximum_linear), flush=True)
    finally:
        if fixture is not None:
            fixture.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for process in reversed(processes):
            if process.poll() is None:
                if process.args[0] == 'ros2':
                    os.killpg(process.pid, signal.SIGINT)
                else:
                    process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                if process.args[0] == 'ros2':
                    os.killpg(process.pid, signal.SIGTERM)
                else:
                    process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
            if process.args[0] == 'ros2':
                # A launch process can exit while a native component remains
                # stuck in teardown. It retains the service name and would
                # intercept the next isolated test's load-node request.
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass


if __name__ == '__main__':
    main()
