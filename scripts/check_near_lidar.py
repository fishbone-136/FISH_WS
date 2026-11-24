#!/usr/bin/env python3
"""Read-only near-field cloud audit using transforms at each message stamp.

Only subscriptions and get_parameters are used. No goals, commands, clearing,
or device configuration are sent. Counts from different topics are independent
observations, not an exact point-for-point filter comparison.
"""
import argparse
from collections import Counter, defaultdict, deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import numpy as np


def cloud_array(message):
    """Respect PointCloud2 field offsets, byte order and padded rows."""
    types = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
    fields = [f for f in message.fields if f.name in ('x', 'y', 'z', 'tag', 'line', 'timestamp')]
    if not all(any(f.name == name and f.count == 1 for f in fields) for name in ('x', 'y', 'z')):
        raise ValueError('Missing scalar XYZ fields')
    dtype = np.dtype({'names': [f.name for f in fields],
        'formats': [('>' if message.is_bigendian else '<') + types[f.datatype] for f in fields],
        'offsets': [f.offset for f in fields], 'itemsize': message.point_step})
    return np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
        strides=(message.row_step, message.point_step)).reshape(-1)


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def stamp_ns(message):
    return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec


def clock_diagnostics(samples):
    """Distinguish progressing device data from ROS-clock-compatible data.

    A constant clock offset is reported, never silently fixed: receive time
    alone cannot certify measurement time or calibration for motion control.
    """
    a=np.asarray(samples,dtype=float).reshape(-1,2)
    if not len(a):return {'messages':0,'host_clock_compatible':False}
    if not np.isfinite(a).all():raise ValueError('Non-finite clock sample')
    offsets=a[:,1]-a[:,0]
    delta=np.diff(a[:,0])
    device_span=float(a[-1,0]-a[0,0]);host_span=float(a[-1,1]-a[0,1])
    return {'messages':len(a),'device_span_seconds':device_span,
            'host_span_seconds':host_span,'regressions':int((delta<0.).sum()),
            'duplicate_stamps':int((delta==0.).sum()),
            'offset_seconds_min_median_max':[float(np.min(offsets)),float(np.median(offsets)),float(np.max(offsets))],
            'host_clock_compatible':bool(np.all((offsets>=-.05)&(offsets<=.5))),
            'device_time_progressing':bool(len(a)>=3 and np.all(delta>0.) and host_span>.1
                and abs(device_span-host_span)<=max(.1,.1*host_span))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=10.)
    parser.add_argument('--label', default='uncontrolled scene')
    parser.add_argument('--unitree-only', action='store_true',
        help='Audit built-in lidar in its native frames; do not assume HAP TF or synchronized clocks')
    parser.add_argument('--half-width', type=float, default=.6)
    args = parser.parse_args()
    if args.output.exists() or args.output.with_suffix('.npz').exists() or not 3 <= args.seconds <= 60 or args.half_width <= 0:
        parser.error('Output must be new; seconds 3..60; half-width positive')

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rcl_interfaces.srv import GetParameters
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = rclpy.create_node('near_lidar_readonly_audit')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    topics = (('/utlidar/cloud', '/utlidar/cloud_base', '/utlidar/cloud_deskewed')
              if args.unitree_only else ('/livox/lidar', '/cloud_registered_body', '/obstacles/raw',
              '/obstacles/deskewed', '/safety/remembered_obstacles'))
    latest, arrivals, planes = {}, defaultdict(list), defaultdict(lambda: deque(maxlen=100))
    motion, errors, rows, processed = [], Counter(), defaultdict(list), {}
    snapshots = {}
    clocks = defaultdict(list)
    status_history = deque(maxlen=200)
    target = 'base_link_3d'
    bins = ((0., .5), (.5, 1.), (1., 1.5), (1.5, 2.), (2., 3.))

    def cloud(message, topic):
        arrivals[topic].append(time.monotonic())
        latest[topic] = message
        clocks[topic].append((stamp_ns(message)/1.e9, node.get_clock().now().nanoseconds/1.e9))

    def odom(message):
        p, v, w = message.pose.pose.position, message.twist.twist.linear, message.twist.twist.angular
        motion.append({'stamp_ns': stamp_ns(message), 'xyz': [p.x, p.y, p.z],
            'linear_speed': math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z),
            'angular_speed': math.sqrt(w.x*w.x+w.y*w.y+w.z*w.z)})

    for topic in topics:
        node.create_subscription(PointCloud2, topic, lambda m, t=topic: cloud(m, t), qos_profile_sensor_data)
    for lane in ('raw', 'deskewed'):
        node.create_subscription(PoseStamped, '/perception/'+lane+'_ground_plane',
            lambda m, t=lane: planes[t].append(m), qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data)
    def status(message):
        try:
            status_history.append({'receive_time_utc': datetime.now(timezone.utc).isoformat(),
                                   'data': json.loads(message.data)})
        except ValueError:
            errors['invalid memory status JSON'] += 1
    node.create_subscription(String, '/safety/obstacle_memory_status', status, qos_profile_sensor_data)
    clients, queries = [], []
    parameter_queries = (
        ('laser_mapping', ['preprocess.blind', 'point_filter_num', 'preprocess.scan_line', 'preprocess.scan_rate']),
        ('livox_lidar_publisher', ['user_config_path', 'frame_id', 'xfer_format']),
        ('safety_pointcloud_to_laserscan', ['range_min', 'range_max', 'ground_clearance', 'obstacle_height_above_ground']),
        ('pointcloud_to_laserscan_node', ['range_min', 'ground_clearance']))
    for name, parameters in (() if args.unitree_only else parameter_queries):
        client = node.create_client(GetParameters, '/'+name+'/get_parameters')
        clients.append(client)
        queries.append((name, parameters, client.call_async(GetParameters.Request(names=parameters))))

    started = time.monotonic()
    warmup_end = started + 2.
    next_sample = warmup_end
    try:
        while time.monotonic() < warmup_end + args.seconds:
            rclpy.spin_once(node, timeout_sec=.01)
            if time.monotonic() < next_sample:
                continue
            next_sample = time.monotonic() + .2
            for topic, message in list(latest.items()):
                stamp = stamp_ns(message)
                if processed.get(topic) == stamp:
                    continue
                try:
                    data = cloud_array(message)
                    p = np.column_stack([data[k] for k in ('x', 'y', 'z')]).astype(float)
                    valid = np.isfinite(p).all(axis=1) & (np.linalg.norm(p, axis=1) > 1.e-4)
                    source = p[valid]
                    entry = {'stamp_ns': stamp, 'frame': message.header.frame_id,
                        'total_points': len(p), 'finite_nonzero_points': int(valid.sum())}
                    if topic == '/livox/lidar':
                        entry['invalid_or_zero_points'] = int((~valid).sum())
                        entry['sensor_range_percentiles'] = np.percentile(np.linalg.norm(source, axis=1), [0, 1, 50, 99]).tolist() if len(source) else None
                        if len(source):
                            front = source[(source[:, 0] > 0) & (np.abs(np.arctan2(source[:, 1], source[:, 0])) < math.radians(10))]
                            entry['center_elevation_deg_percentiles'] = np.percentile(np.degrees(np.arctan2(front[:, 2], np.hypot(front[:, 0], front[:, 1]))), [0, 1, 50, 99, 100]).tolist() if len(front) else None
                        if 'tag' in data.dtype.names:
                            entry['tag_histogram'] = {str(int(k)): int(v) for k, v in zip(*np.unique(data['tag'][valid], return_counts=True))}
                    if args.unitree_only:
                        # Native geometry is diagnostic only: a matching frame
                        # NAME is not proof of calibration to the LIO body.
                        p = source
                        entry['coordinate_frame'] = message.header.frame_id
                        entry['hap_alignment_validated'] = False
                        radii = np.linalg.norm(source, axis=1)
                        entry['native_radius_bins'] = {label: int(((radii>=low)&(radii<high)).sum())
                            for label,low,high in [('5_20cm',.05,.20),('20_50cm',.20,.50),('50_100cm',.50,1.)]}
                    elif message.header.frame_id != target:
                        tf = buffer.lookup_transform(target, message.header.frame_id, Time.from_msg(message.header.stamp))
                        t = tf.transform.translation
                        p = source @ rotation(tf.transform.rotation).T + [t.x, t.y, t.z]
                    else:
                        p = source
                    lane = 'deskewed' if topic in ('/cloud_registered_body', '/obstacles/deskewed') else 'raw'
                    plane = min(planes[lane], key=lambda m: abs(stamp_ns(m)-stamp), default=None)
                    heights = None
                    # Memory clouds are synthetic, flattened visualization points.
                    if plane is not None and abs(stamp_ns(plane)-stamp) <= 150000000 and plane.header.frame_id == target and topic != '/safety/remembered_obstacles':
                        normal = rotation(plane.pose.orientation)[:, 2]
                        pos = plane.pose.position
                        heights = (p - [pos.x, pos.y, pos.z]) @ normal
                        entry['ground_stamp_delta_ms'] = (stamp_ns(plane)-stamp) / 1.e6
                        entry['ground_normal'] = normal.tolist()
                        entry['ground_z_at_base'] = pos.z
                    entry['bins'] = []
                    for low, high in bins:
                        mask = (p[:, 0] >= low) & (p[:, 0] < high) & (np.abs(p[:, 1]) <= args.half_width)
                        item = {'x': [low, high], 'points': int(mask.sum())}
                        if heights is not None:
                            h = heights[mask]
                            item['height_above_plane_percentiles'] = np.percentile(h, [0, 5, 50, 95, 100]).tolist() if len(h) else None
                            item['above_10cm'] = int((h >= .10).sum())
                            item['above_30cm'] = int((h >= .30).sum())
                        entry['bins'].append(item)
                    rows[topic].append(entry)
                    roi = (p[:, 0] >= 0.) & (p[:, 0] < 3.) & (np.abs(p[:, 1]) <= args.half_width)
                    key = topic.strip('/').replace('/', '_')
                    snapshots[key] = p[roi]
                    snapshots[key+'_stamp_ns'] = np.array(stamp, dtype=np.int64)
                    if heights is not None:
                        snapshots[key+'_height'] = heights[roi]
                    else:
                        snapshots.pop(key+'_height', None)
                    processed[topic] = stamp
                except Exception as exc:
                    errors[topic+': '+str(exc)] += 1

        report = {'time_utc': datetime.now(timezone.utc).isoformat(), 'label': args.label,
            'duration_seconds': args.seconds, 'target_frame': 'native per topic' if args.unitree_only else target, 'half_width': args.half_width,
            'method': '5 Hz samples; message-stamped TF; independent topics; ground match <=150 ms; no target identity inference',
            'topics': {}, 'parameters': {}, 'samples': dict(rows), 'errors': dict(errors)}
        report['memory_status_history'] = list(status_history)
        report['unitree_native_mode'] = args.unitree_only
        report['clock_diagnostics'] = {topic: clock_diagnostics(samples) for topic,samples in clocks.items()}
        if args.unitree_only:
            report['method'] = '5 Hz independent native-frame samples; clock offset reported, not corrected; no calibration or target identity inference'
        if not args.unitree_only:
            try:
                mount = buffer.lookup_transform(target, 'livox_frame', Time()).transform
                report['sensor_mount'] = {'xyz': [mount.translation.x, mount.translation.y, mount.translation.z],
                    'quaternion_xyzw': [mount.rotation.x, mount.rotation.y, mount.rotation.z, mount.rotation.w]}
            except Exception as exc:
                report['errors']['sensor mount unavailable: '+str(exc)] = 1
        for topic in topics:
            times = [t for t in arrivals[topic] if t >= warmup_end]
            topic_rows = rows[topic]
            report['topics'][topic] = {'publishers': node.count_publishers(topic),
                'messages': len(times), 'hz': (len(times)-1)/(times[-1]-times[0]) if len(times)>1 else None,
                'sampled_frames': len(topic_rows), 'bins': [
                    {'x': list(bounds), 'points_min_median_max': [int(np.min(v)), float(np.median(v)), int(np.max(v))]}
                    for index, bounds in enumerate(bins) if (v := [r['bins'][index]['points'] for r in topic_rows])],
                'last': topic_rows[-1] if topic_rows else None}
        for name, parameters, future in queries:
            if future.done() and future.exception() is None:
                report['parameters'][name] = {key: rclpy.parameter.parameter_value_to_python(value) for key, value in zip(parameters, future.result().values)}
                if len(future.result().values) != len(parameters):
                    report['errors']['parameter response incomplete: '+name] = 1
            else:
                report['errors']['parameter query unavailable: '+name] = 1
        if motion:
            points = np.array([m['xyz'] for m in motion])
            report['motion'] = {'samples': len(motion),
                'max_linear_speed': max(m['linear_speed'] for m in motion),
                'max_angular_speed': max(m['angular_speed'] for m in motion),
                'max_position_change_from_first': float(np.max(np.linalg.norm(points-points[0], axis=1)))}
        else:
            report['motion'] = {'samples': 0, 'stationary_verified': False}
        report['snapshot_file'] = str(args.output.with_suffix('.npz'))
        np.savez_compressed(args.output.with_suffix('.npz'), **snapshots)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        summary = {'output': str(args.output), 'motion': report['motion'], 'errors': report['errors'],
                   'clock_diagnostics': report['clock_diagnostics']}
        summary['topics'] = {t: {'hz': s['hz'], 'sampled_frames': s['sampled_frames'],
            'bin_medians': [b['points_min_median_max'][1] for b in s['bins']]}
            for t, s in report['topics'].items()}
        print(json.dumps(summary, indent=2, allow_nan=False))
        required = topics if args.unitree_only else topics[:4]
        return 0 if all(rows[t] for t in required) else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
