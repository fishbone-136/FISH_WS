#!/usr/bin/python3
"""Accepted poses must keep memory fresh despite a delayed navigation TF."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

if os.environ.get('ROS_DOMAIN_ID') != '89' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Requires isolated ROS_DOMAIN_ID=89 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI'] = (
    '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
    '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
    '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'go2_navigation/test'))
from check_obstacle_pipeline import Fixture
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from ament_index_python.packages import get_package_prefix


def main():
    rclpy.init()
    f = Fixture()
    children = []
    state = {'relay': True, 'memory': {}, 'diagnostic': {}, 'samples': []}
    accepted = f.create_publisher(Odometry, '/Odometry_validated', 16)
    f.create_subscription(Odometry, '/odom',
                          lambda m: accepted.publish(m) if state['relay'] else None, 16)
    health = f.create_publisher(Bool, '/lio/localization_valid',
                               QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    f.create_timer(.05, lambda: health.publish(Bool(data=True)))

    def status(m):
        state['memory'] = json.loads(m.data)
        state['samples'].append(state['memory']['scan_age'])

    f.create_subscription(String, '/safety/obstacle_memory_status', status,
                          QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    f.create_subscription(String, '/safety/motion_diagnostics',
                          lambda m: state.update(diagnostic=json.loads(m.data)), qos_profile_sensor_data)

    def start(args):
        p = subprocess.Popen(args, start_new_session=True)
        children.append(p)

    def wait(seconds):
        deadline = time.monotonic() + seconds
        f.until(lambda: time.monotonic() >= deadline, timeout=seconds + 2.)

    try:
        # Old code cannot process any scan with a one-second TF delay, even
        # though that scan's accepted physical pose has already arrived.
        f.tf_lag = 1.
        f.floor_z = -.37
        f.box_x = 1.
        f.box_heights = [.11, .2, .3, .45, .6, .7]
        f.floor_points = np.array([[x, y] for x in np.arange(.3, 5., .12)
                                   for y in np.arange(-2., 2., .12)])
        projector = Path(get_package_prefix('go2_perception')) / 'lib/go2_perception/pointcloud_to_laserscan_node'
        start([str(projector), '--ros-args', '-r', '__node:=freshness_projector',
               '-r', 'cloud_in:=/test/cloud_fast', '-r', 'scan:=/scan_safety',
               '-r', 'obstacle_points:=/obstacles/deskewed', '-p', 'target_frame:=base_link',
               '-p', 'ground_filter_enabled:=true', '-p', 'observed_free_space_only:=true',
               '-p', 'range_min:=0.25', '-p', 'range_max:=5.5', '--log-level', 'warn'])
        executable = Path(get_package_prefix('go2_lio_navigation')) / 'lib/go2_lio_navigation'
        start(['/usr/bin/python3', str(executable / 'obstacle_memory.py'), '--ros-args',
               '-p', 'raw_cloud_topic:=/test/cloud_fast'])
        start(['/usr/bin/python3', str(executable / 'scan_safety_gate.py')])
        f.until(lambda: state['diagnostic'].get('memory_valid') is True, timeout=15.)
        state['samples'].clear()
        wait(3.)
        assert state['memory']['validated_pose_source'] is True, state
        assert state['memory']['cells'] > 0, state
        assert len(state['samples']) >= 15, state
        assert max(state['samples']) < .5, state
        print('PASS accepted poses keep occupied memory fresh with 1 s delayed TF; max age %.3f s' %
              max(state['samples']), flush=True)
        # Fresh /odom and scans alone cannot refresh memory without accepted
        # full-pose evidence. The existing 0.75 s failure threshold stays live.
        state['relay'] = False
        f.until(lambda: state['diagnostic'].get('memory_valid') is False, timeout=3.)
        assert state['memory']['cells'] > 0
        print('PASS missing validated poses closes the gate while retaining obstacles', flush=True)
        state['relay'] = True
        f.until(lambda: state['diagnostic'].get('memory_valid') is True, timeout=4.)
        print('PASS fresh validated poses recover memory health', flush=True)
    finally:
        f.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for p in reversed(children):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in children:
            try:
                p.wait(timeout=6.)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait(timeout=3.)


if __name__ == '__main__':
    main()
