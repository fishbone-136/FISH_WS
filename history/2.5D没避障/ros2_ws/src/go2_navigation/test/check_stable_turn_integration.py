#!/usr/bin/env python3
"""Exercise Nav2 turns and cancellation in an isolated localhost DDS domain."""

import math
import os
import signal
import subprocess
import time
import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class Fixture(Node):
    def __init__(self):
        super().__init__('stable_turn_fixture')
        self.x = self.y = self.yaw = 0.0
        self.command = Twist()
        self.received = []
        self.raw = []
        self.position_noise = 0.0
        self.tf_pub = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 1)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 1)
        self.create_subscription(Twist, '/test/cmd_vel', self.on_command, 1)
        self.create_subscription(Twist, '/test/cmd_vel_nav', self.on_raw, 1)
        self.client = ActionClient(self, FollowPath, '/follow_path')
        self.previous = time.monotonic()
        self.timer = self.create_timer(0.025, self.step)

    def on_command(self, command):
        self.command = command
        self.received.append((time.monotonic(), command.linear.x, command.angular.z))

    def on_raw(self, command):
        self.raw.append((time.monotonic(), command.linear.x, command.angular.z, self.yaw))

    def step(self):
        current = time.monotonic()
        dt = min(0.1, current - self.previous)
        self.previous = current
        self.x += self.command.linear.x * math.cos(self.yaw) * dt
        self.y += self.command.linear.x * math.sin(self.yaw) * dt
        self.yaw += self.command.angular.z * dt
        noise = self.position_noise * (1 if int(current * 5) % 2 else -1)
        # Deliberately lag sensor TF by 120 ms, reproducing the real HAP timing.
        stamp = (self.get_clock().now() - rclpy.duration.Duration(seconds=0.12)).to_msg()
        frame = TransformStamped()
        frame.header.frame_id = 'map'
        frame.child_frame_id = 'odom'
        frame.header.stamp = stamp
        frame.transform.rotation.w = 1.0
        base = TransformStamped()
        base.header.frame_id = 'odom'
        base.child_frame_id = 'base_link'
        base.header.stamp = stamp
        base.transform.translation.x = self.x + noise
        base.transform.translation.y = self.y
        base.transform.rotation.z = math.sin(self.yaw / 2)
        base.transform.rotation.w = math.cos(self.yaw / 2)
        self.tf_pub.sendTransform([frame, base])
        odom = Odometry()
        odom.header = base.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x + noise
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = base.transform.rotation
        odom.twist.twist = self.command
        self.odom_pub.publish(odom)
        scan = LaserScan()
        scan.header.frame_id = 'base_link'
        scan.header.stamp = stamp
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2 * math.pi / 360
        scan.range_min = 0.1
        scan.range_max = 5.5
        scan.ranges = [float('inf')] * 361
        self.scan_pub.publish(scan)

    def until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if predicate():
                return
        raise AssertionError('Timed out waiting for isolated controller')

    def lifecycle(self, name, transition):
        client = self.create_client(ChangeState, name + '/change_state')
        self.until(client.service_is_ready, 8)
        request = ChangeState.Request()
        request.transition.id = transition
        future = client.call_async(request)
        self.until(future.done, 12)
        assert future.result().success, (name, transition)

    def goal(self, dx, final_yaw=None, target_x=None, target_y=None):
        request = FollowPath.Goal()
        request.controller_id = 'FollowPath'
        request.goal_checker_id = 'goal_checker'
        request.path.header.frame_id = 'map'
        request.path.header.stamp = self.get_clock().now().to_msg()
        yaw = final_yaw if final_yaw is not None else math.pi if dx < 0 else 0.0
        end_x = self.x + dx if target_x is None else target_x
        end_y = self.y if target_y is None else target_y
        for i in range(11):
            pose = PoseStamped()
            pose.header = request.path.header
            pose.pose.position.x = self.x + (end_x - self.x) * i / 10
            pose.pose.position.y = self.y + (end_y - self.y) * i / 10
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            request.path.poses.append(pose)
        future = self.client.send_goal_async(request)
        self.until(future.done, 5)
        handle = future.result()
        assert handle.accepted
        return handle


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '81' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Requires ROS_DOMAIN_ID=81 and ROS_LOCALHOST_ONLY=1; '
                         'never use robot domain')
    os.environ['CYCLONEDDS_URI']=(
        '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
        '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/>'
        '</Peers></Discovery></Domain></CycloneDDS>')
    config = os.path.join(get_package_share_directory('go2_navigation'), 'config/nav2_params.yaml')
    with open(config, encoding='utf-8') as stream:
        limits = yaml.safe_load(stream)['controller_server']['ros__parameters']['FollowPath']
    processes = []
    try:
        for package, executable, remaps in [
            ('nav2_controller', 'controller_server', ['cmd_vel:=/test/cmd_vel_nav']),
            ('nav2_velocity_smoother', 'velocity_smoother',
             ['cmd_vel:=/test/cmd_vel_nav', 'cmd_vel_smoothed:=/test/cmd_vel']),
        ]:
            command = ['/opt/ros/humble/lib/' + package + '/' + executable,
                       '--ros-args', '--params-file', config, '--log-level', 'warn']
            for remap in remaps:
                command += ['-r', remap]
            processes.append(subprocess.Popen(command))
        rclpy.init()
        fixture = Fixture()
        fixture.lifecycle('/controller_server', Transition.TRANSITION_CONFIGURE)
        fixture.lifecycle('/velocity_smoother', Transition.TRANSITION_CONFIGURE)
        fixture.lifecycle('/controller_server', Transition.TRANSITION_ACTIVATE)
        fixture.lifecycle('/velocity_smoother', Transition.TRANSITION_ACTIVATE)
        fixture.until(lambda: fixture.client.server_is_ready(), 8)
        started = time.monotonic()
        goal = fixture.goal(-1.0)
        result = goal.get_result_async()
        fixture.until(result.done, 35)
        assert result.result().status == 4, result.result()
        forward = [row for row in fixture.raw if row[0] >= started and row[1] > 0.12]
        assert forward, 'Never handed over to path tracking'
        turning = [row[2] for row in fixture.raw if started <= row[0] < forward[0][0]
                   and abs(row[2]) > 0.2]
        assert turning and all(v * turning[0] > 0 for v in turning), 'Rear turn reversed direction'
        peak=max(abs(v) for v in turning)
        assert peak >= .95*limits['rotate_to_heading_angular_vel'], 'Yaw limit never reached on a clear rear turn'
        smoothed=[abs(row[2]) for row in fixture.received if started<=row[0]<forward[0][0]]
        assert smoothed and max(smoothed)>=.90*limits['rotate_to_heading_angular_vel'], 'Smoother clipped the checked turn'
        assert forward[0][0] - started < 10.0, 'Excessive path-alignment delay'
        print('PASS rear goal with delayed TF: aligned after %.2fs, reached after %.2fs' %
              (forward[0][0] - started, time.monotonic() - started), flush=True)
        print('PASS checked turn reaches %.2f rad/s, smoothed peak %.2f rad/s' %
              (peak,max(smoothed)),flush=True)
        started = time.monotonic()
        goal = fixture.goal(1.0)
        fixture.until(lambda: any(t > started and abs(w) > 0.5
                                  for t, v, w in fixture.received), 10)
        cancel = goal.cancel_goal_async()
        fixture.until(cancel.done, 5)
        result = goal.get_result_async()
        fixture.until(result.done, 5)
        assert result.result().status == 5, result.result()
        fixture.until(lambda: abs(fixture.command.linear.x) < 1.e-6 and
                      abs(fixture.command.angular.z) < 1.e-6, 1.5)
        print('PASS cancellation: smoothed command returned to zero', flush=True)
        fixture.position_noise = 0.04
        target_x, target_y = fixture.x + 0.75, fixture.y
        started = time.monotonic()
        latest = fixture.goal(0.0, final_yaw=math.pi, target_x=target_x, target_y=target_y)
        result = latest.get_result_async()
        next_replan = time.monotonic() + 0.5
        replans = 0
        while time.monotonic() - started < 18:
            rclpy.spin_once(fixture, timeout_sec=0.01)
            if result.done() and result.result().status == 4:
                break
            if time.monotonic() >= next_replan:
                latest = fixture.goal(
                    0.0, final_yaw=math.pi, target_x=target_x, target_y=target_y)
                result = latest.get_result_async()
                next_replan = time.monotonic() + 0.5
                replans += 1
        assert result.done() and result.result().status == 4, 'Goal hunting under replanning/noise'
        assert replans >= 2
        print('PASS reached final heading with 4 cm pose jitter and %d path replans in %.2fs' %
              (replans, time.monotonic() - started), flush=True)
        fixture.destroy_node()
        rclpy.shutdown()
    finally:
        for process in processes:
            process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)


if __name__ == '__main__':
    main()
