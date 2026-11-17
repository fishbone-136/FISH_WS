#!/usr/bin/env python3
"""Test localization timestamp rejection in an isolated domain; no motion commands."""

import os
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ClearEntireCostmap
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '85' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Requires isolated domain 85 and ROS_LOCALHOST_ONLY=1')
    executable = os.path.join(get_package_prefix('go2_lio_navigation'),
                              'lib/go2_lio_navigation/lio_base_bridge')
    process = subprocess.Popen([
        executable, '--ros-args', '-p', 'startup_samples:=1', '-p', 'recovery_samples:=5',
        '-p', 'input_timeout:=1.2',
        '-p', 'cancel_navigation_on_invalid:=false', '-p', 'clear_costmaps_on_invalid:=true'])
    try:
        rclpy.init()
        node = rclpy.create_node('lio_bridge_freshness_test')
        state, output = [], []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions = [
            node.create_subscription(Bool, '/lio/localization_valid',
                                     lambda m: state.append(m.data), qos),
            node.create_subscription(Odometry, '/Odometry_validated', output.append, 10)]
        publisher = node.create_publisher(Odometry, '/Odometry', 10)
        clears=[]
        def cleared(request,response):
            clears.append(time.monotonic())
            return response
        node.create_service(ClearEntireCostmap,'/local_costmap/clear_entirely_local_costmap',cleared)
        node.create_service(ClearEntireCostmap,'/global_costmap/clear_entirely_global_costmap',cleared)

        def send_for(seconds, age, x=0.):
            started = time.monotonic()
            while time.monotonic() - started < seconds:
                message = Odometry()
                stamp = node.get_clock().now().nanoseconds - int(age * 1e9)
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
                message.header.frame_id = 'odom_lio'
                message.pose.pose.orientation.w = 1.0
                message.pose.pose.position.x = x
                publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.02)
                time.sleep(0.02)

        send_for(1.5, 0.01)
        assert state and state[-1] and output, 'Fresh localization did not open the gate'
        send_for(0.3, 0.8)
        assert not state[-1], 'Stale localization kept the motion gate open'
        count = len(output)
        send_for(0.3, 0.8)
        assert len(output) == count, 'Stale odometry was forwarded'
        assert not clears, 'A brief timestamp hold erased costmaps/cancelled the navigation goal'
        send_for(0.08, 0.01)
        assert state[-1], 'Timestamp-only hold waited for the full recovery sequence'
        send_for(0.4, 0.01)
        assert state[-1], 'Fresh localization did not recover'
        print('PASS transient age failure stops immediately, retains goal/maps, and recovers on fresh evidence')
        end=time.monotonic()+1.5
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        assert not state[-1] and clears,'Persistent input loss did not escalate to navigation stop'
        send_for(0.4,0.01)
        assert state[-1],'Fresh input did not recover after the full timeout sequence'
        print('PASS sustained input loss escalates and clears stale navigation maps')
        send_for(0.3, -1.0)
        assert not state[-1], 'Future localization kept the motion gate open'
        print('PASS fresh data, stale data stop/no forwarding, recovery and future timestamp stop')
        send_for(0.3, 0.01, x=10.)
        assert not any(m.pose.pose.position.x > 5. for m in output), (
            'Recovery bypassed the pose-jump guard and forwarded a 10 m discontinuity')
        assert not state[-1], 'Implausible pose jump reopened the localization gate'
        send_for(0.3, 0.01)
        assert not state[-1], 'A latched localization fault recovered without a restart'
        print('PASS recovery rejects discontinuous poses and keeps a real divergence latched')
        node.destroy_node()
        rclpy.shutdown()
        del subscriptions
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=5)


if __name__ == '__main__':
    main()
