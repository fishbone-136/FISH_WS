#!/usr/bin/env python3
"""Record a user-issued navigation trial; publishes no goals or motion commands."""
import argparse
import collections
import json
import math
from pathlib import Path
import signal
import time
import numpy as np

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu,LaserScan,PointCloud2
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--wait', type=float, default=300.)
    parser.add_argument('--duration', type=float, default=90.)
    parser.add_argument('--post-terminal',type=float,default=5.,
                        help='Keep recording this long after a goal ends, allowing return-goal trials')
    parser.add_argument('--raw-clouds',action='store_true',
                        help='After goal acceptance, save 5 Hz sampled raw XYZ within 3 m to a sidecar NPZ')
    parser.add_argument('--capture-without-goal',action='store_true',
                        help='Start raw capture immediately and stop at duration, for manual stationary/turn trials')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Refusing to overwrite an existing trial')
    if args.raw_clouds and args.output.with_suffix('.raw.npz').exists():
        parser.error('Refusing to overwrite existing raw-cloud evidence')
    rclpy.init()
    node = rclpy.create_node('navigation_readonly_trial_recorder')
    began = time.monotonic()
    history = collections.defaultdict(list)
    state = {'first':None, 'finished':None, 'goals':{}, 'stop':False}
    raw_arrays={}
    raw_subscription=[None]
    last_raw=[-math.inf]

    def elapsed():
        return time.monotonic()-began

    def status(msg):
        for entry in msg.status_list:
            key = bytes(entry.goal_info.goal_id.uuid).hex()
            if key not in state['goals'] and entry.status not in (1, 2, 3):
                continue
            if entry.status in (1, 2, 3) and state['first'] is None:
                state['first'] = elapsed()
                if args.raw_clouds and raw_subscription[0] is None:
                    raw_subscription[0]=node.create_subscription(PointCloud2,'/livox/lidar',
                        raw_cloud,qos_profile_sensor_data)
                print('User-issued goal observed: '+key, flush=True)
            if state['goals'].get(key) != entry.status:
                history['statuses'].append([elapsed(), key, entry.status])
            state['goals'][key] = entry.status
        if state['goals'] and all(v in (4, 5, 6) for v in state['goals'].values()):
            if state['finished'] is None:
                state['finished'] = elapsed()
        else:
            state['finished'] = None

    node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', status,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    for topic in ('/cmd_vel_nav', '/cmd_vel', '/cmd_vel_safe', '/cmd_vel_guarded'):
        node.create_subscription(Twist, topic, lambda m, t=topic: history[t].append(
            [elapsed(), m.linear.x, m.linear.y, m.angular.z]), qos_profile_sensor_data)

    def odom(m):
        history['odom'].append([elapsed(), m.pose.pose.position.x, m.pose.pose.position.y,
                               m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z,
                               m.pose.pose.position.z,m.pose.pose.orientation.x,m.pose.pose.orientation.y,
                               m.pose.pose.orientation.z,m.pose.pose.orientation.w,
                               m.header.stamp.sec+m.header.stamp.nanosec/1.e9])
        state['odom_frames'] = [m.header.frame_id, m.child_frame_id]

    node.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data)
    node.create_subscription(Imu,'/livox/imu',lambda m:
        history['imu_timestamps'].append([elapsed(),
            m.header.stamp.sec+m.header.stamp.nanosec/1.e9,
            node.get_clock().now().nanoseconds/1.e9-m.header.stamp.sec-m.header.stamp.nanosec/1.e9]),
        QoSProfile(depth=2000,reliability=ReliabilityPolicy.RELIABLE))
    node.create_subscription(SportModeState, '/lf/sportmodestate', lambda m:
        history['sport'].append([elapsed(), *map(float,m.velocity), float(m.yaw_speed),
                                int(m.mode), int(m.gait_type), int(m.error_code)]), qos_profile_sensor_data)
    node.create_subscription(Request, '/api/sport/request', lambda m:
        history['requests'].append([elapsed(), int(m.header.identity.id),
                                    int(m.header.identity.api_id), m.parameter]), qos_profile_sensor_data)
    node.create_subscription(Response, '/api/sport/response', lambda m:
        history['responses'].append([elapsed(), int(m.header.identity.id),
                                     int(m.header.identity.api_id), int(m.header.status.code)]), qos_profile_sensor_data)
    for topic in ('/go2/idle_stand_status', '/safety/motion_diagnostics','/safety/obstacle_memory_status'):
        node.create_subscription(String, topic, lambda m, t=topic:
            history[t].append([elapsed(), json.loads(m.data)]),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            if topic != '/safety/motion_diagnostics' else qos_profile_sensor_data)
    node.create_subscription(NavPath, '/plan', lambda m:
        history['plan_endpoints'].append([elapsed(), m.header.frame_id,
            m.poses[-1].pose.position.x, m.poses[-1].pose.position.y]) if m.poses else None,
        qos_profile_sensor_data)

    def scan(m):
        history['scan_replay'].append([elapsed(), m.header.stamp.sec+m.header.stamp.nanosec/1.e9,
            m.header.frame_id, m.angle_min, m.angle_increment,
            [[i, float(r)] for i, r in enumerate(m.ranges)
             if math.isfinite(r) and m.range_min <= r <= m.range_max]])

    node.create_subscription(LaserScan, '/scan_safety', scan, qos_profile_sensor_data)

    def remembered(m):
        fields={f.name:f for f in m.fields}
        if m.height!=1 or not all(k in fields for k in ('x','y')):return
        dtype=np.dtype({'names':['x','y'],'formats':['<f4']*2,
            'offsets':[fields[k].offset for k in ('x','y')],'itemsize':m.point_step})
        points=np.frombuffer(m.data,dtype=dtype,count=m.width)
        history['remembered_obstacles'].append([elapsed(),
            m.header.stamp.sec+m.header.stamp.nanosec/1.e9,m.header.frame_id,
            [[float(p['x']),float(p['y'])] for p in points]])
    node.create_subscription(PointCloud2,'/safety/remembered_obstacles',remembered,qos_profile_sensor_data)

    def raw_cloud(m):
        now=elapsed()
        if now-last_raw[0]<.20 or m.height!=1:return
        fields={f.name:f for f in m.fields}
        if not all(k in fields and fields[k].datatype==7 for k in ('x','y','z')):return
        dtype=np.dtype({'names':['x','y','z'],'formats':['>f4' if m.is_bigendian else '<f4']*3,
                        'offsets':[fields[k].offset for k in ('x','y','z')],'itemsize':m.point_step})
        data=np.frombuffer(m.data,dtype=dtype,count=m.width)
        p=np.column_stack([data[k] for k in ('x','y','z')])
        p=p[np.isfinite(p).all(axis=1)&(np.sum(p[:,:2]*p[:,:2],axis=1)<=9.)][::4]
        key='frame_%06d'%len(raw_arrays)
        raw_arrays[key]=p.astype('<f4',copy=True)
        history['raw_cloud_frames'].append([now,m.header.stamp.sec+m.header.stamp.nanosec/1e9,
                                           m.header.frame_id,int(m.width),key])
        last_raw[0]=now

    def static_tf(m):
        cache=state.setdefault('static_transforms',{})
        for t in m.transforms:
            p,q=t.transform.translation,t.transform.rotation
            cache[t.child_frame_id]=[t.header.frame_id,p.x,p.y,p.z,q.x,q.y,q.z,q.w]
    node.create_subscription(TFMessage,'/tf_static',static_tf,
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    if args.raw_clouds and args.capture_without_goal:
        raw_subscription[0]=node.create_subscription(PointCloud2,'/livox/lidar',raw_cloud,qos_profile_sensor_data)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: state.update(stop=True))
    print('Read-only recorder ready; no goals or motion commands published', flush=True)
    try:
        while not state['stop']:
            rclpy.spin_once(node, timeout_sec=.02)
            if args.capture_without_goal and elapsed()>args.duration:
                break
            if state['first'] is None and elapsed() > args.wait:
                break
            if state['first'] is not None and elapsed()-state['first'] > args.duration:
                break
            if state['finished'] is not None and elapsed()-state['finished'] > args.post_terminal:
                break
    finally:
        result = {'goal_seen':state['first'] is not None, 'state':state,
                  'duration':elapsed(), 'history':dict(history)}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if raw_arrays:
            sidecar=args.output.with_suffix('.raw.npz')
            np.savez_compressed(sidecar,**raw_arrays)
            result['raw_cloud_sidecar']=str(sidecar)
        args.output.write_text(json.dumps(result))
        print(json.dumps({k:v for k,v in result.items() if k != 'history'}), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
