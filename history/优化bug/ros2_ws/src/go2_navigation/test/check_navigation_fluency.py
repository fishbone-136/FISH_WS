#!/usr/bin/env python3
"""Real Nav2 controller/smoother/safety chain in a localhost-only synthetic world.

No go2_base, Unitree RPC, real sensor, or robot-domain node is started.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

if os.environ.get('ROS_DOMAIN_ID') != '93' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Requires ROS_DOMAIN_ID=93 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI'] = (
    '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
    '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
    '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PolygonStamped, PoseStamped, TransformStamped, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from go2_navigation.safety_geometry import configured_protection
import yaml


class World(Node):
    def __init__(self):
        super().__init__('navigation_fluency_world')
        self.front, self.rear, self.half_width = configured_protection()
        self.x = self.y = self.yaw = 0.
        self.command = Twist()
        self.scan_enabled = True
        self.scan_extra = []
        self.points = np.empty((0, 2))
        self.history, self.modes, self.gate_diagnostics, self.trajectory_diagnostics = [], [], [], []
        self.body_footprint = None
        self.tf = TransformBroadcaster(self)
        self.odom = self.create_publisher(Odometry, '/odom', 1)
        self.scan = self.create_publisher(LaserScan, '/scan_safety', qos_profile_sensor_data)
        self.map = self.create_publisher(OccupancyGrid, '/test/fluency_map',
                                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Twist, '/cmd_vel_guarded', self.on_command, qos_profile_sensor_data)
        self.create_subscription(String, '/navigation/controller_diagnostics',
                                 lambda m: self.modes.append(json.loads(m.data)), qos_profile_sensor_data)
        self.create_subscription(String, '/navigation/trajectory_diagnostics',
                                 lambda m: self.trajectory_diagnostics.append(json.loads(m.data)),
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/safety/motion_diagnostics',
                                 lambda m: self.gate_diagnostics.append(json.loads(m.data)),
                                 qos_profile_sensor_data)
        self.create_subscription(PolygonStamped, '/safety/body_footprint',
                                 lambda m: setattr(self, 'body_footprint', m), qos_profile_sensor_data)
        self.action = ActionClient(self, FollowPath, '/follow_path')
        self.last = time.monotonic()
        self.next_scan = 0.
        self.create_timer(.02, self.step)

    def on_command(self, message):
        self.command = message
        self.history.append((time.monotonic(), self.x, self.y, self.yaw,
                             message.linear.x, message.angular.z))

    def corridor(self, width, yaw=.18):
        self.x, self.y, self.yaw = 0., 0., yaw
        self.command = Twist()
        self.points = np.array([(x, sign * width / 2.) for x in np.arange(.8, 2.81, .02)
                                for sign in (-1., 1.)])
        self.publish_map()

    def publish_map(self):
        m = OccupancyGrid()
        m.header.frame_id = 'odom_lio'
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution, m.info.width, m.info.height = .04, 200, 150
        m.info.origin.position.x, m.info.origin.position.y = -2., -3.
        m.info.origin.orientation.w = 1.
        grid = np.zeros((150, 200), dtype=np.int8)
        for x, y in self.points:
            ix, iy = int(math.floor((x + 2.) / .04)), int(math.floor((y + 3.) / .04))
            if 0 <= ix < 200 and 0 <= iy < 150:
                grid[iy, ix] = 100
        m.data = grid.ravel().tolist()
        self.map.publish(m)

    def step(self):
        now = time.monotonic()
        dt = min(.05, now - self.last)
        self.last = now
        self.x += self.command.linear.x * math.cos(self.yaw) * dt
        self.y += self.command.linear.x * math.sin(self.yaw) * dt
        self.yaw += self.command.angular.z * dt
        stamp = (self.get_clock().now() - rclpy.duration.Duration(seconds=.04)).to_msg()
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id, tf.header.stamp = 'odom_lio', 'base_link', stamp
        tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.tf.sendTransform(tf)
        odom = Odometry()
        odom.header, odom.child_frame_id = tf.header, 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation = tf.transform.rotation
        odom.twist.twist = self.command
        self.odom.publish(odom)
        if self.scan_enabled and now >= self.next_scan:
            self.next_scan = now + 1. / 15.
            scan = LaserScan()
            scan.header.frame_id, scan.header.stamp = 'base_link', stamp
            scan.angle_min, scan.angle_max = -math.pi, math.pi
            scan.angle_increment = math.pi / 720
            scan.range_min, scan.range_max = .10, 5.50
            ranges = np.full(1441, np.inf)
            if len(self.points):
                dx, dy = self.points[:, 0] - self.x, self.points[:, 1] - self.y
                bx = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy
                by = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy
                # Check actual padded body, independent of the Nav2 map grid.
                assert not np.any((bx < self.front - 1.e-3) & (bx > -self.rear + 1.e-3) &
                                  (np.abs(by) < self.half_width - 1.e-3)), (
                    'Synthetic body intersects a wall', self.x, self.y, self.yaw)
                indices = np.rint((np.arctan2(by, bx) + math.pi) / scan.angle_increment).astype(int)
                np.minimum.at(ranges, np.clip(indices, 0, 1440), np.hypot(bx, by))
            scan.ranges = ranges.tolist()
            for x, y in self.scan_extra:
                i = int(round((math.atan2(y, x) + math.pi) / scan.angle_increment))
                scan.ranges[i] = min(scan.ranges[i], math.hypot(x, y))
            self.scan.publish(scan)

    def until(self, predicate, timeout=10.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.01)
            if predicate():
                return
        raise AssertionError(('Timed out', self.x, self.y, self.yaw,
                              self.gate_diagnostics[-1:] or None, self.modes[-1:] or None))

    def wait(self, seconds):
        end = time.monotonic() + seconds
        self.until(lambda: time.monotonic() >= end, seconds + 2.)

    def lifecycle(self, node, transition):
        client = self.create_client(ChangeState, node + '/change_state')
        self.until(client.service_is_ready)
        req = ChangeState.Request()
        req.transition.id = transition
        future = client.call_async(req)
        self.until(future.done)
        assert future.result().success, (node, transition)

    def follow(self, target=2.20):
        request = FollowPath.Goal()
        request.controller_id, request.goal_checker_id = 'FollowPath', 'goal_checker'
        request.path.header.frame_id = 'odom_lio'
        for x in np.linspace(0., target, 80):
            p = PoseStamped()
            p.header = request.path.header
            p.pose.position.x, p.pose.orientation.w = float(x), 1.
            request.path.poses.append(p)
        future = self.action.send_goal_async(request)
        self.until(future.done)
        handle = future.result()
        assert handle.accepted
        return handle.get_result_async()


def main():
    root = Path(__file__).resolve().parents[4]
    nav_share = Path(get_package_share_directory('go2_navigation'))
    config = yaml.safe_load((nav_share / 'config/nav2_params.yaml').read_text())
    config['controller_server']['ros__parameters']['controller_plugins'] = ['FollowPath']
    local = config['local_costmap']['local_costmap']['ros__parameters']
    point_planning = all(x == 0. and y == 0. for x, y in yaml.safe_load(local['footprint']))
    local.update(global_frame='odom_lio', plugins=['test_layer', 'inflation_layer'])
    local['test_layer'] = {'plugin': 'nav2_costmap_2d::StaticLayer',
                           'map_topic': '/test/fluency_map', 'map_subscribe_transient_local': True,
                           'subscribe_to_updates': False, 'track_unknown_space': False}
    config['controller_server']['ros__parameters']['FollowPath']['debug_trajectory_details'] = True
    log_dir = root / 'reports' / ('navigation_fluency_' + time.strftime('%Y%m%d_%H%M%S'))
    log_dir.mkdir(parents=True, exist_ok=False)
    children, logs = [], []
    rclpy.init()
    world = World()
    world.corridor(.72)
    try:
        with tempfile.TemporaryDirectory(prefix='go2_fluency_') as work:
            params = Path(work) / 'nav2.yaml'
            params.write_text(yaml.safe_dump(config))
            collision = root / 'ros2_ws/src/go2_lio_navigation/config/collision_monitor.yaml'
            commands = [
                ('controller', ['/opt/ros/humble/lib/nav2_controller/controller_server',
                                '--ros-args', '--params-file', str(params), '-r', 'cmd_vel:=/cmd_vel_nav']),
                ('smoother', ['/opt/ros/humble/lib/nav2_velocity_smoother/velocity_smoother',
                              '--ros-args', '--params-file', str(params), '-r', 'cmd_vel:=/cmd_vel_nav',
                              '-r', 'cmd_vel_smoothed:=/cmd_vel']),
                ('collision', ['/opt/ros/humble/lib/nav2_collision_monitor/collision_monitor',
                               '--ros-args', '--params-file', str(collision)]),
                ('gate', ['/usr/bin/python3', str(root / 'ros2_ws/src/go2_lio_navigation/scripts/scan_safety_gate.py'),
                          '--ros-args', '-p', 'memory_required:=false']),
            ]
            for label, command in commands:
                log = (log_dir / (label + '.log')).open('w')
                logs.append(log)
                children.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                                 start_new_session=True))
            for name in ('/controller_server', '/velocity_smoother', '/collision_monitor'):
                world.lifecycle(name, Transition.TRANSITION_CONFIGURE)
                world.lifecycle(name, Transition.TRANSITION_ACTIVATE)
            world.until(world.action.server_is_ready)
            world.wait(.5)
            start = time.monotonic()
            result = world.follow()
            next_replan = time.monotonic() + .5
            replans = 0
            while not result.done() and time.monotonic() - start < 25.:
                rclpy.spin_once(world, timeout_sec=.01)
                if time.monotonic() >= next_replan:
                    result = world.follow()
                    replans += 1
                    next_replan = time.monotonic() + .5
            assert result.done() and result.result().status == 4, ('corridor goal', world.x, world.y, world.yaw)
            rows = [r for r in world.history if r[0] >= start and 1.0 <= r[1] <= 1.8]
            assert len(rows) >= 40, ('insufficient corridor samples', len(rows))
            assert all(r[4] >= .20 for r in rows), ('creep/stop in static corridor', min(r[4] for r in rows))
            if not point_planning:
                assert any(m['mode'] == 'corridor_align' for m in world.modes), world.modes
                assert any(m['mode'] == 'corridor_track' for m in world.modes), world.modes
            body = world.body_footprint
            assert body and body.header.frame_id == 'base_link'
            assert abs(max(p.x for p in body.polygon.points) - world.front) < 1.e-6
            assert abs(min(p.x for p in body.polygon.points) + world.rear) < 1.e-6
            assert abs(max(p.y for p in body.polygon.points) - world.half_width) < 1.e-6
            assert world.trajectory_diagnostics, 'Critic diagnostics not published'
            print('PASS 72 cm corridor: %d replans, no cruise stop/creep, %.2f s' %
                  (replans, time.monotonic() - start), flush=True)

            if point_planning:
                for width in (1.0, .60, .50):
                    world.corridor(width, yaw=0.)
                    world.wait(.5)
                    result = world.follow()
                    world.until(result.done, 20.)
                    assert result.result().status == 4, ('point corridor', width, world.x)
                    print('PASS point planning through %.0f cm corridor%s' %
                          (100. * width, ' inside the 30 cm soft band' if width < .60 else ''), flush=True)

            # A new goal resets state. Scan-loss protection must still close
            # even when the cached obstacle geometry is unchanged.
            world.corridor(.72, yaw=0.)
            world.wait(.4)
            result = world.follow(target=2.4)
            world.until(lambda: world.command.linear.x > .22)
            world.scan_enabled = False
            lost = time.monotonic()
            world.until(lambda: world.command.linear.x == 0., .45)
            world.until(lambda: world.gate_diagnostics[-1]['stop_reason'] == 'scan_unavailable', .2)
            assert time.monotonic() - lost < .5
            world.scan_enabled = True
            world.until(lambda: world.command.linear.x > .22)
            world.scan_extra = [(world.front - .01, 0.)]
            danger = time.monotonic()
            world.until(lambda: world.command.linear.x == 0., .20)
            reaction = time.monotonic() - danger
            world.scan_extra = []
            world.until(lambda: world.command.linear.x > .22)
            world.until(result.done, 20.)
            assert result.result().status == 4
            print('PASS cached geometry: scan loss stops, fresh scan recovers and reaches goal', flush=True)
            print('PASS one new return inside protected footprint stops in %.3f s' % reaction, flush=True)

            blocked_width = 2. * world.half_width - .04
            world.corridor(blocked_width, yaw=0.)
            world.wait(.5)
            result = world.follow(target=.20)
            world.until(result.done, 5.)
            assert result.result().status == 4, 'A blocked door beyond a nearby goal must not reject arrival'
            print('PASS short goal before an impassable door still succeeds', flush=True)
            result = world.follow()
            world.until(result.done, 12.)
            assert result.result().status == 6, result.result()
            assert world.x < .8 - world.front + .04, ('entered physically impassable corridor', world.x)
            print('PASS %.0f cm corridor: cannot enter with %.0f cm protected width' %
                  (100. * blocked_width, 200. * world.half_width), flush=True)
            report = {'point_planning': point_planning,
                      'physical_rectangle': [world.front, world.rear, world.half_width],
                      'replans': replans, 'corridor_min_cruise_speed': min(r[4] for r in rows),
                      'controller_modes': sorted({m['mode'] for m in world.modes}),
                      'gate': world.gate_diagnostics[-1], 'logs': str(log_dir)}
            (log_dir / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    finally:
        (log_dir / 'trace.json').write_text(json.dumps({
            'commands': world.history, 'modes': world.modes,
            'gate': world.gate_diagnostics, 'trajectories': world.trajectory_diagnostics}) + '\n')
        for p in reversed(children):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        bad_exits = []
        for p in children:
            try:
                p.wait(timeout=7.)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait(timeout=3.)
            if p.returncode not in (0, -signal.SIGINT):
                bad_exits.append(p.returncode)
        for log in logs:
            log.close()
        world.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('Logs: ' + str(log_dir), flush=True)
        if bad_exits:
            raise AssertionError(('Native child shutdown failed', bad_exits))


if __name__ == '__main__':
    main()
