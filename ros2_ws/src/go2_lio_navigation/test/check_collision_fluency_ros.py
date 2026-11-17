#!/usr/bin/python3
"""Verify command continuity with delayed TF on an isolated localhost graph."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

if os.environ.get('ROS_DOMAIN_ID') != '84' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Requires isolated ROS_DOMAIN_ID=84 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI'] = (
    '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
    '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
    '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

root = Path(os.environ['GO2_HUMBLE_PROJECT_ROOT'])
sys.path.insert(0, str(root / 'ros2_ws/src/go2_navigation/test'))
from check_obstacle_pipeline import Fixture


def main():
    rclpy.init()
    f = Fixture(publish_maps='--continuity-only' not in sys.argv)
    children = []
    samples = {'input': [], 'monitor': [], 'gate': [], 'diagnostics': []}
    f.scan_override = []
    f.tf_lag = .2
    # Replace only the synthetic command timer: 40 Hz matches the real smoother.
    list(f.timers)[1].cancel()
    f.create_timer(.025, f.send_command)
    f.command_enabled = True
    for topic, key in [('/cmd_vel', 'input'), ('/cmd_vel_safe', 'monitor'),
                       ('/cmd_vel_guarded', 'gate')]:
        f.create_subscription(Twist, topic, lambda m, k=key: samples[k].append(
            (time.monotonic(), m.linear.x, m.angular.z)), qos_profile_sensor_data)
    f.create_subscription(String, '/safety/motion_diagnostics',
        lambda m: samples['diagnostics'].append((time.monotonic(), json.loads(m.data))),
        qos_profile_sensor_data)

    def wait(seconds):
        deadline = time.monotonic() + seconds
        f.until(lambda: time.monotonic() >= deadline, timeout=seconds + 2.)

    def continuity(lag):
        f.tf_lag = lag
        f.until(lambda: f.safe is not None and f.safe.linear.x > .54)
        wait(.3)
        begin = time.monotonic()
        wait(3.)
        result = {}
        for key, minimum in [('input', 35.), ('monitor', 35.), ('gate', 43.)]:
            rows = [r for r in samples[key] if begin <= r[0]]
            gaps = [b[0]-a[0] for a, b in zip(rows, rows[1:])]
            hz = (len(rows)-1)/(rows[-1][0]-rows[0][0]) if len(rows) > 1 else 0.
            assert hz >= minimum, (key, lag, hz)
            assert max(gaps) < .10, (key, lag, max(gaps))
            assert all(r[1] > .54 for r in rows), (key, lag, rows)
            result[key] = {'hz': round(hz, 2), 'max_gap_ms': round(max(gaps)*1000, 2)}
        diagnostics = [d for t, d in samples['diagnostics'] if t >= begin]
        assert diagnostics and all(d['command_fresh'] for d in diagnostics), diagnostics
        print('PASS delayed TF %.2f s, no zero pulses: %s' % (lag, json.dumps(result)), flush=True)
        return result

    try:
        config = Path(get_package_share_directory('go2_lio_navigation')) / 'config/collision_monitor.yaml'
        gate = root / 'ros2_ws/src/go2_lio_navigation/scripts/scan_safety_gate.py'
        for args in [
            ['/opt/ros/humble/lib/nav2_collision_monitor/collision_monitor', '--ros-args',
             '--params-file', str(config), '--log-level', 'warn'],
            ['/usr/bin/python3', str(gate), '--ros-args', '-p', 'memory_required:=false',
             '--log-level', 'warn'],
        ]:
            children.append(subprocess.Popen(args, start_new_session=True))
        f.lifecycle(Transition.TRANSITION_CONFIGURE)
        f.lifecycle(Transition.TRANSITION_ACTIVATE)
        results = {str(lag): continuity(lag) for lag in (.2, 1.)}
        if '--continuity-only' in sys.argv:
            print(json.dumps({'results': results}), flush=True)
            return
        f.scan_override = [(f.padded_length-.02, 0.)]
        began = time.monotonic()
        f.until(lambda: f.safe.linear.x == 0., timeout=.25)
        results['single_return_stop_seconds'] = time.monotonic()-began
        print('PASS one return stops under delayed TF in %.3f s' %
              results['single_return_stop_seconds'], flush=True)
        f.scan_override = []
        f.until(lambda: f.safe.linear.x > .54)
        f.command_enabled = False
        f.until(lambda: f.safe.linear.x == 0., timeout=.3)
        print('PASS command loss still stops within original 0.2 s gate timeout', flush=True)
        f.command_enabled = True
        f.until(lambda: f.safe.linear.x > .54)
        f.scan_override, f.enabled = None, False
        f.until(lambda: f.safe.linear.x == 0., timeout=.4)
        print('PASS scan loss still stops within original 0.35 s scan timeout', flush=True)
        f.enabled, f.scan_override = True, []
        f.until(lambda: f.safe.linear.x > .54)
        f.synthetic_footprint = False
        f.until(lambda: f.safe.linear.x == 0., timeout=.55)
        print('PASS footprint loss still stops within original 0.5 s footprint timeout', flush=True)
        print(json.dumps({'results': results}), flush=True)
    finally:
        f.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for p in reversed(children):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in children:
            try:
                p.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait(timeout=3.)


if __name__ == '__main__':
    main()
