#!/usr/bin/python3
"""Exercise dynamic/occluded input and StopMove feedback on localhost only."""
import os
if os.environ.get('ROS_DOMAIN_ID') != '86' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Requires isolated ROS_DOMAIN_ID=86 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI'] = ('<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast>'
    '</General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
    '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
import json
import math
from pathlib import Path
import signal
import subprocess
import time
from array import array
import numpy as np
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from unitree_near_guard import ROTATION, TRANSLATION
import rclpy
from geometry_msgs.msg import Twist, PolygonStamped, Point32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String
from unitree_api.msg import Request
from unitree_go.msg import SportModeState
from rclpy.qos import qos_profile_sensor_data


def main():
    root = Path(__file__).resolve().parents[4]
    rclpy.init()
    n = rclpy.create_node('dynamic_safety_isolated_fixture')
    children = []
    samples, turns, diagnostics, requests = [], [], [], []
    raw_pub = n.create_publisher(PointCloud2, '/livox/lidar', qos_profile_sensor_data)
    unitree_raw_pub = n.create_publisher(PointCloud2, '/utlidar/cloud', qos_profile_sensor_data)
    unitree_body_pub = n.create_publisher(PointCloud2, '/utlidar/cloud_base', qos_profile_sensor_data)
    scan_pub = n.create_publisher(LaserScan, '/scan_safety', qos_profile_sensor_data)
    footprint_pub = n.create_publisher(PolygonStamped, '/local_costmap/published_footprint', 1)
    command_pub = n.create_publisher(Twist, '/cmd_vel_safe', 1)
    sport_pub = n.create_publisher(SportModeState, '/lf/sportmodestate', 1)
    odom_pub = n.create_publisher(Odometry, '/odom', 1)
    def on_output(message):
        now = time.monotonic()
        samples.append((now, message.linear.x))
        turns.append((now, message.angular.z))

    n.create_subscription(Twist, '/cmd_vel_guarded', on_output, 1)
    n.create_subscription(String, '/safety/motion_diagnostics', lambda m: diagnostics.append(json.loads(m.data)), 1)
    n.create_subscription(Request, '/api/sport/request', lambda m: requests.append(
        (time.monotonic(), m.header.identity.api_id)), 10)
    state = dict(raw='clear', obstacle=None, scan_enabled=True, moving_feedback=True,
                 vx=.6, vy=0., wz=0., corridor=False)
    state['unitree'] = 'clear'

    def unitree_clouds(stamp):
        if state['unitree'] == 'missing': return
        stamp.sec -= 1309820  # The field device has a progressing offset clock.
        rng = np.random.default_rng(10)
        body = rng.uniform([1., -.5, -.3], [3., .5, .8], (100,3))
        if state['unitree'] == 'near': body[0] = [.43, .02, -.1]
        if state['unitree'] == 'outside_turn': body[0] = [.55, .02, -.1]
        raw = (body-TRANSLATION) @ np.linalg.inv(ROTATION.T)
        for publisher, points, frame in ((unitree_raw_pub, raw, 'utlidar_lidar'),
                                        (unitree_body_pub, body, 'base_link')):
            a = np.zeros(len(points), dtype=np.dtype({'names':['x','y','z','ring','time'],
                'formats':['<f4']*3+['<u2','<f4'], 'offsets':[0,4,8,20,24], 'itemsize':32}))
            for i,k in enumerate(('x','y','z')): a[k] = points[:,i]
            a['time'] = np.arange(len(points), dtype=np.float32)*.0001
            # The factory body output omits the close return; raw still stops.
            if frame == 'base_link' and state['unitree'] in ('near','outside_turn'): a = a[1:]
            if state['unitree'] == 'bad_transform' and frame == 'base_link': a['x'] += .02
            message = PointCloud2(); message.header.frame_id = frame; message.header.stamp = stamp
            message.height, message.width, message.point_step = 1, len(a), 32
            message.row_step = len(a)*32
            message.fields = [PointField(name=k,offset=o,datatype=4 if k=='ring' else 7,count=1)
                              for k,o in [('x',0),('y',4),('z',8),('ring',20),('time',24)]]
            message.data = array('B',a.tobytes()); publisher.publish(message)

    def send():
        odom = Odometry(); odom.header.frame_id = 'odom'; odom.child_frame_id = 'base_link'
        odom.header.stamp = n.get_clock().now().to_msg(); odom.pose.pose.orientation.w = 1.
        odom_pub.publish(odom)
        unitree_clouds(n.get_clock().now().to_msg())
        stamp = n.get_clock().now().to_msg()
        if state['raw'] != 'missing':
            points = [[2., 0., 0.], [2., .05, .1], [2., -.05, -.1]]
            if state['raw'] == 'zero': points = [[0., 0., 0.]]*20
            if state['raw'] == 'near': points.append([.03, 0., 0.])
            raw = PointCloud2();raw.header.frame_id = 'livox_frame';raw.header.stamp = stamp
            raw.height, raw.width, raw.point_step = 1, len(points), 12
            raw.row_step = raw.width*12
            raw.fields = [PointField(name=k, offset=i*4, datatype=7, count=1) for i,k in enumerate(('x','y','z'))]
            raw.data = array('B', np.asarray(points, dtype='<f4').tobytes());raw_pub.publish(raw)
        if state['scan_enabled']:
            scan = LaserScan();scan.header.frame_id = 'base_link';scan.header.stamp = stamp
            scan.angle_min, scan.angle_max, scan.angle_increment = -1., 1., .01
            scan.range_min, scan.range_max = .05, 5.5
            scan.ranges = [math.inf]*201
            if state['corridor']:
                for i in (30,170):scan.ranges[i] = math.hypot(.3,.25)
            if state['obstacle'] is not None:scan.ranges[100] = state['obstacle']
            scan_pub.publish(scan)
        footprint = PolygonStamped();footprint.header.frame_id = 'base_link';footprint.header.stamp = stamp
        footprint.polygon.points = [Point32(x=0., y=0.) for _ in range(4)]
        footprint_pub.publish(footprint)
        command = Twist();command.linear.x = state['vx'];command.linear.y = state['vy']
        command.angular.z = state['wz'];command_pub.publish(command)
        sport = SportModeState();sport.velocity[0] = .3 if state['moving_feedback'] else 0.
        sport_pub.publish(sport)

    n.create_timer(.05, send)

    def until(predicate, timeout=3.):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(n, timeout_sec=.005)
            if predicate():return
        raise AssertionError({'last_output': samples[-5:], 'diagnostic': diagnostics[-1:]})

    def spin(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline:rclpy.spin_once(n, timeout_sec=.005)

    def ready():
        until(lambda: samples and samples[-1][1] > .4, 4.)

    results = {}
    def stopped(name, deadline=.15):
        began = time.monotonic()
        until(lambda: samples and samples[-1][0] >= began and samples[-1][1] == 0., deadline)
        stopped_at = time.monotonic()
        results[name] = round(stopped_at-began, 4)
        spin(.2)
        assert all(v == 0. for t,v in samples if t > stopped_at), (name, samples[-10:])

    try:
        gate = root/'ros2_ws/src/go2_lio_navigation/scripts/scan_safety_gate.py'
        base = root/'ros2_ws/install/go2_core/lib/go2_core/go2_base'
        for args in [
            ['/usr/bin/python3', str(gate), '--ros-args', '-p', 'memory_required:=false',
             '-p', 'unitree_near_required:=true', '--log-level', 'warn'],
            [str(base), '--ros-args', '-p', 'odom_source:=none', '-p', 'cmd_vel_topic:=/cmd_vel_guarded', '--log-level', 'error'],
        ]:
            children.append(subprocess.Popen(args, start_new_session=True))
        ready()
        # A live front return outside the complete turning sweep must not
        # force repeated obstacle/health recovery while the controller pivots.
        state.update(unitree='outside_turn', vx=0., wz=.70)
        until(lambda: diagnostics and abs(diagnostics[-1]['output'][1]) > .60, 3.)
        spin(.35)
        assert diagnostics[-1]['unitree_braking_scale'] == 1., diagnostics[-1]
        results['unitree_front_return_safe_turn'] = True
        state['obstacle'] = .30
        until(lambda: diagnostics and diagnostics[-1]['stop_reason'] == 'occupied_footprint')
        until(lambda: turns and turns[-1][1] == 0.)
        hold_started = time.monotonic()
        state['obstacle'] = None
        until(lambda: diagnostics[-1]['stop_reason'] == 'turn_recovery_wait')
        state['obstacle'] = .30
        until(lambda: diagnostics[-1]['stop_reason'] == 'occupied_footprint')
        assert all(yaw == 0. for t, yaw in turns if t >= hold_started), turns[-10:]
        state['obstacle'] = None
        until(lambda: turns and turns[-1][1] > .60)
        results['turn_resumes_after_continuous_clearance'] = True
        state.update(unitree='clear',vx=.6,wz=0.)
        ready()
        # Straight travel in a 50 cm corridor is feasible, while turning the
        # long rectangular body into its walls remains forbidden.
        state.update(corridor=True,vx=.3,wz=0.)
        until(lambda: samples and samples[-1][1] > .22, 3.)
        results['narrow_corridor_translation'] = True
        state.update(vx=0.,wz=.70)
        spin(.45)
        # A small pivot can be safe; the long body cannot execute an entire
        # quarter turn here. Check the permitted braking arc, not a blanket ban.
        angular = diagnostics[-1]['output'][1]
        assert abs(angular) < .60, diagnostics[-1]
        from turn_clearance import turn_is_clear, braking_horizon
        walls = [(math.hypot(.3,.25)*math.cos(a), math.hypot(.3,.25)*math.sin(a))
                 for a in (-.70,.70)]
        assert turn_is_clear(walls,0.,0.,angular,.38,.15,
                             braking_horizon(0.,0.,angular,diagnostics[-1]['scan_age']))
        results['narrow_corridor_turn_braking_arc_safe'] = True
        state.update(corridor=False,vx=.6,wz=0.)
        ready()
        state['unitree'] = 'near'
        stopped('unitree_raw_near_return_filtered_by_factory')
        assert diagnostics[-1]['stop_reason'] == 'unitree_braking_arc_blocked', diagnostics[-1]
        state['unitree'] = 'clear';spin(.09)
        assert samples[-1][1] == 0., samples[-5:]
        ready()
        state['unitree'] = 'missing'
        stopped('unitree_transport_loss', .3)
        state['unitree'] = 'clear';ready()
        state['unitree'] = 'bad_transform'
        stopped('unitree_transform_mismatch', .7)
        state['unitree'] = 'clear';ready()
        state['obstacle'] = .65
        stopped('new_pedestrian_65cm')
        state['obstacle'] = None;ready()
        state['raw'] = 'near'
        stopped('near_occluder_before_projection')
        # A single recovered raw frame cannot release the stop.
        state['raw'] = 'clear';spin(.07)
        assert samples[-1][1] == 0., samples[-5:]
        ready()
        state['raw'] = 'zero'
        stopped('full_cover_zero_returns')
        state['raw'] = 'clear';ready()
        state['raw'] = 'missing'
        stopped('raw_transport_loss', .25)
        state['raw'] = 'clear';ready()
        state['scan_enabled'] = False
        stopped('projection_or_attitude_loss', .32)
        # While the fixture reports residual walking, stop RPCs are retried.
        began = time.monotonic();spin(.85)
        stops = [t for t,api in requests if t > began and api == 1003]
        assert len(stops) >= 2, requests[-10:]
        gaps = [b-a for a,b in zip(stops,stops[1:])]
        assert all(g >= .18 for g in gaps), gaps
        results['stop_retries_while_moving'] = len(stops)
        state['moving_feedback'] = False;spin(.25)
        began = time.monotonic();spin(.55)
        assert not [t for t,api in requests if t > began and api == 1003], requests[-10:]
        results['stationary_rpc_flood'] = False
        print(json.dumps({'passed': True, 'results': results}, indent=2))
    finally:
        for p in children:
            if p.poll() is None:os.killpg(p.pid, signal.SIGINT)
        for p in children:
            try:p.wait(timeout=5.)
            except subprocess.TimeoutExpired:os.killpg(p.pid, signal.SIGKILL);p.wait(timeout=3.)
        n.destroy_node();rclpy.shutdown()


if __name__ == '__main__':main()
