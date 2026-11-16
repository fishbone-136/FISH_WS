#!/usr/bin/python3
"""Exercise the real base RPC boundary using fake state, on localhost only."""
import json
import os
import signal
import subprocess
import time

if os.environ.get('ROS_DOMAIN_ID') != '86' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Requires isolated ROS_DOMAIN_ID=86 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI'] = ('<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast>'
    '</General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
    '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from unitree_api.msg import Request
from unitree_go.msg import SportModeState


def main():
    rclpy.init()
    node = rclpy.create_node('base_feedback_regression')
    process = subprocess.Popen([get_package_prefix('go2_core')+'/lib/go2_core/go2_base',
        '--ros-args', '-p', 'odom_source:=none', '-p', 'motion_feedback_stale_timeout:=0.3',
        '-p', 'motion_feedback_timeout:=1.0',
        '--log-level', 'warn'])
    requests, health = [], []
    node.create_subscription(Request, '/api/sport/request',
        lambda m: requests.append((time.monotonic(), m.header.identity.api_id)), 100)
    node.create_subscription(Bool, '/go2/motion_execution_valid', lambda m: health.append(m.data),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    commands = node.create_publisher(Twist, '/cmd_vel', 1)
    states = node.create_publisher(SportModeState, '/lf/sportmodestate', qos_profile_sensor_data)
    interactive = node.create_publisher(String, '/go2/interactive_command', 10)

    def pump(seconds, velocity=0., state=True, measured=.1):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            if state:
                m = SportModeState(); m.velocity[0] = measured; m.body_height = .30
                states.publish(m)
            cmd = Twist(); cmd.linear.x = velocity; commands.publish(cmd)
            rclpy.spin_once(node, timeout_sec=.01)
            time.sleep(.02)

    try:
        pump(1.5)
        begin = len(requests); pump(.5, .3)
        assert any(api == 1008 for _, api in requests[begin:]), 'Fresh feedback did not allow Move'
        pump(.8, .3, state=False)
        cutoff = time.monotonic()-.2
        assert not any(t >= cutoff and api == 1008 for t, api in requests), (
            'Base continues forwarding Move indefinitely after feedback loss')
        assert health and health[-1] is False, 'Feedback loss is not visible to the action cancel gate'
        assert any(api == 1003 for _, api in requests[begin:]), 'No StopMove after feedback loss'
        pump(.35, 0., state=False)
        assert health[-1] is False, 'Idle zeros falsely declared missing feedback healthy'
        pump(.5)
        assert health[-1] is True, 'Fresh state plus idle reset did not recover'
        begin = len(requests); pump(.4, .3)
        assert any(api == 1008 for _, api in requests[begin:]), 'New command could not recover'
        print('PASS feedback loss stops RPCs, closes execution health, and fresh idle reset recovers', flush=True)

        pump(.4)
        begin = len(requests)
        interactive.publish(String(data=json.dumps({'request_id':'feedback-test', 'action':'forward',
                                                     'speed':.3, 'duration':3.})))
        pump(.25)
        assert any(api == 1008 for _, api in requests[begin:]), 'Interactive lease never started'
        pump(.8, state=False)
        cutoff = time.monotonic()-.2
        assert not any(t >= cutoff and api == 1008 for t, api in requests), (
            'Interactive lease keeps republishing Move after feedback loss')
        print('PASS interactive motion also stops after feedback loss', flush=True)
        pump(.5)
        pump(1.5,.3,measured=0.)
        cutoff=time.monotonic()-.2
        assert health[-1] is False,'Fresh but stationary robot feedback bypassed execution watchdog'
        assert not any(t>=cutoff and api==1008 for t,api in requests), 'Failed execution kept forwarding RPCs'
        print('PASS continuously fresh stationary feedback still triggers the no-motion watchdog',flush=True)
    finally:
        process.send_signal(signal.SIGINT)
        try: process.wait(timeout=5.)
        except subprocess.TimeoutExpired: process.kill(); process.wait()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
