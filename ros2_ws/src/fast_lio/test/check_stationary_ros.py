#!/usr/bin/python3
"""Exercise the installed estimator with mock sensors on isolated localhost."""
import os
if os.environ.get('ROS_DOMAIN_ID') != '92' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Requires ROS_DOMAIN_ID=92 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI'] = (
    '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
    '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
    '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
import json
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField
from unitree_go.msg import SportModeState


def main():
    root = Path(os.environ['GO2_HUMBLE_PROJECT_ROOT'])
    rclpy.init()
    node = rclpy.create_node('stationary_localization_fixture')
    imu_pub = node.create_publisher(Imu, '/livox/imu', 2000)
    cloud_pub = node.create_publisher(PointCloud2, '/livox/lidar', qos_profile_sensor_data)
    cmd_pub = node.create_publisher(Twist, '/cmd_vel_guarded', 1)
    sport_pub = node.create_publisher(SportModeState, '/lf/sportmodestate', 1)
    odom = []
    node.create_subscription(Odometry, '/Odometry', odom.append, qos_profile_sensor_data)
    a, b, z = np.arange(.3, 4.01, .12), np.arange(-2., 2.01, .12), np.arange(-.37, 1.51, .12)
    xyz = np.array([(x, y, -.37) for x in a for y in b] +
                   [(4., y, h) for y in b for h in z] +
                   [(x, 2., h) for x in a for h in z], dtype='<f4')
    dtype = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp'],
                     'formats': ['<f4', '<f4', '<f4', '<f4', 'u1', 'u1', '<f8'],
                     'offsets': [0, 4, 8, 12, 16, 17, 18], 'itemsize': 26})
    points = np.zeros(len(xyz), dtype=dtype)
    for i, key in enumerate(('x', 'y', 'z')):
        points[key] = xyz[:, i]
    points['intensity'] = 100.
    points['line'] = np.arange(len(points)) % 6
    state = {'bias': 0., 'moving': False, 'feedback': True}

    def imu():
        msg = Imu()
        msg.header.frame_id = 'livox_frame'
        msg.header.stamp = (node.get_clock().now() - rclpy.duration.Duration(seconds=.005)).to_msg()
        msg.linear_acceleration.z = 1.
        msg.linear_acceleration.x = state['bias']
        imu_pub.publish(msg)

    def cloud():
        stamp = node.get_clock().now() - rclpy.duration.Duration(seconds=.08)
        points['timestamp'] = stamp.nanoseconds + np.linspace(0, 45e6, len(points))
        msg = PointCloud2()
        msg.header.frame_id, msg.header.stamp = 'livox_frame', stamp.to_msg()
        msg.height, msg.width, msg.point_step = 1, len(points), 26
        msg.row_step, msg.is_dense = len(points) * 26, True
        msg.fields = [PointField(name=key, offset=dtype.fields[key][1], count=1,
                     datatype=PointField.UINT8 if key in ('tag', 'line') else
                     PointField.FLOAT64 if key == 'timestamp' else PointField.FLOAT32)
                     for key in dtype.names]
        msg.data = points.tobytes()
        cloud_pub.publish(msg)

    def feedback():
        cmd = Twist()
        cmd.linear.x = .1 if state['moving'] else 0.
        cmd_pub.publish(cmd)
        if state['feedback']:
            msg = SportModeState()
            msg.velocity[0] = cmd.linear.x
            sport_pub.publish(msg)

    node.create_timer(.005, imu)
    node.create_timer(1. / 15., cloud)
    node.create_timer(.02, feedback)
    log_path = root / 'reports/stationary_localization_20261110.log'
    with log_path.open('w') as log:
        proc = subprocess.Popen([str(root / 'ros2_ws/install/fast_lio/lib/fast_lio/fastlio_mapping'),
            '--ros-args', '--params-file', str(root / 'ros2_ws/src/go2_lio_navigation/config/fast_lio_hap.yaml'),
            '-p', 'mapping.imu_init_duration:=0.6', '-p', 'mapping.initial_map_duration:=0.5',
            '-p', 'mapping.input_warmup_duration:=0.1', '-p', 'point_filter_num:=1'],
            stdout=log, stderr=subprocess.STDOUT)

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                assert proc.poll() is None, log_path.read_text()[-4000:]
                rclpy.spin_once(node, timeout_sec=.005)

        try:
            end = time.monotonic() + 14.
            while time.monotonic() < end and 'Stationary constraint engaged' not in log_path.read_text():
                spin(.1)
            assert len(odom) >= 8, 'No healthy odometry'
            assert 'Stationary constraint engaged' in log_path.read_text(), 'No stationary fusion'
            anchor = odom[-1].pose.pose.position
            anchor = np.array([anchor.x, anchor.y, anchor.z])
            start = len(odom)
            state['bias'] = .01
            spin(10.)
            positions = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
                                  for m in odom[start:]])
            drift = float(np.linalg.norm(positions - anchor, axis=1).max())
            assert drift < .05, ('Static drift with IMU bias', drift)
            state['moving'] = True
            spin(.5)
            assert 'Stationary constraint released' in log_path.read_text(), 'Movement did not release'
            state['moving'] = False
            spin(2.)
            releases = log_path.read_text().count('Stationary constraint released')
            assert log_path.read_text().count('Stationary constraint engaged') >= 2
            state['feedback'] = False
            spin(.5)
            assert log_path.read_text().count('Stationary constraint released') > releases
            report = {'max_static_drift_m': drift, 'biased_static_samples': len(positions),
                      'motion_releases_constraint': True, 'stale_feedback_releases_constraint': True,
                      'isolated_ros_domain': 92}
            print(json.dumps(report), flush=True)
            (root / 'reports/stationary_localization_20261110.json').write_text(json.dumps(report, indent=2))
        finally:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                proc.wait(timeout=8.)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        assert proc.returncode == 0, ('Unclean shutdown', proc.returncode)


if __name__ == '__main__':
    main()
