#!/usr/bin/env python3
"""Exercise the installed guard against a mock sport service on localhost only."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

if os.environ.get('ROS_DOMAIN_ID') != '84' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Requires ROS_DOMAIN_ID=84 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI'] = (
    '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
    '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
    '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')

from action_msgs.msg import GoalStatus, GoalStatusArray
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState


def main():
    rclpy.init()
    node = rclpy.create_node('idle_stand_mock_sport')
    velocity = node.create_publisher(Twist, '/cmd_vel_guarded', 1)
    sport = node.create_publisher(SportModeState, '/lf/sportmodestate', 1)
    api = node.create_publisher(Request, '/api/sport/request', 1)
    replies = node.create_publisher(Response, '/api/sport/response', 1)
    goals = node.create_publisher(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    interactive = node.create_publisher(String, '/go2/interactive_command', 1)
    state = {'vx':0., 'feed':True, 'ack_stop':True, 'requests':[], 'diagnostic':{}}

    def request(m):
        if m.header.identity.api_id not in (1002, 1003):
            return
        state['requests'].append(m.header.identity.api_id)
        if m.header.identity.api_id == 1003 and not state['ack_stop']:
            return
        reply = Response()
        reply.header.identity = m.header.identity
        reply.header.status.code = 0
        replies.publish(reply)

    node.create_subscription(Request, '/api/sport/request', request, 10)
    node.create_subscription(String, '/go2/idle_stand_status',
        lambda m: state.update(diagnostic=json.loads(m.data)),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def feed():
        if not state['feed']:
            return
        cmd = Twist()
        cmd.linear.x = state['vx']
        velocity.publish(cmd)
        msg = SportModeState()
        msg.velocity[0] = state['vx']
        sport.publish(msg)
        if state['vx']:
            move = Request()
            move.header.identity.api_id = 1008
            api.publish(move)

    node.create_timer(.02, feed)

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            assert proc.poll() is None, 'Installed idle guard exited'
            rclpy.spin_once(node, timeout_sec=.01)

    def until(predicate, seconds=5.):
        end = time.monotonic()+seconds
        while not predicate() and time.monotonic() < end:
            spin(.01)
        assert predicate(), state

    def goal(code):
        msg = GoalStatusArray()
        entry = GoalStatus()
        entry.goal_info.goal_id.uuid[0] = 42
        entry.status = code
        msg.status_list = [entry]
        goals.publish(msg)
        spin(.15)

    executable = Path(get_package_prefix('go2_lio_navigation'))/'lib/go2_lio_navigation/idle_stand_guard.py'
    # Use the installed executable exactly as launch_ros does; invoking Python
    # explicitly would hide missing executable permission in symlink installs.
    proc = subprocess.Popen([str(executable)])
    try:
        until(lambda: state['diagnostic'] != {})
        spin(1.)
        assert state['requests'] == []
        print('PASS startup without an observed active goal sends no pose RPC', flush=True)
        goal(2)
        spin(1.)
        assert state['requests'] == []
        print('PASS zero velocity during active navigation does not change standing mode', flush=True)
        state['vx'] = .08
        spin(.2)
        goal(4)
        spin(.25)
        state['vx'] = 0.
        until(lambda: state['diagnostic'].get('phase') == 'done')
        assert state['requests'] == [1003, 1002], state
        spin(1.)
        assert state['requests'] == [1003, 1002]
        print('PASS terminal deceleration followed by exactly one acknowledged stop/stand sequence', flush=True)
        state['ack_stop'] = False
        goal(2)
        goal(4)
        until(lambda: state['diagnostic'].get('error') is not None)
        assert state['requests'] == [1003, 1002, 1003], state
        spin(.5)
        assert state['requests'] == [1003, 1002, 1003]
        print('PASS missing stop acknowledgement prevents standing and is not repeatedly retried', flush=True)
        state['ack_stop'] = True
        goal(2)
        goal(4)
        until(lambda: len(state['requests']) == 4)
        goal(2)
        spin(1.)
        assert state['requests'] == [1003, 1002, 1003, 1003], state
        print('PASS new active goal interrupts pending standing transition', flush=True)
        goal(4)
        interactive.publish(String(data='{"action":"forward"}'))
        spin(1.2)
        assert len(state['requests']) == 4
        print('PASS interactive control disarms the previous navigation settle', flush=True)
        goal(2)
        goal(4)
        state['feed'] = False
        spin(1.2)
        assert len(state['requests']) == 4
        state['feed'] = True
        until(lambda: state['diagnostic'].get('phase') == 'done')
        assert state['requests'][-2:] == [1003, 1002]
        print('PASS missing velocity/state feedback blocks settling until fresh stationary data returns', flush=True)
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
