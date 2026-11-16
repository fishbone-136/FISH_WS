#!/usr/bin/env python3
"""Read-only frequency, latency, lifecycle, CPU and memory audit (no motion goals)."""

import argparse
import collections
import json
import os
from pathlib import Path
import time

from geometry_msgs.msg import PolygonStamped, Twist
from lifecycle_msgs.srv import GetState
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from std_msgs.msg import Bool, String


def cpu_sample():
    rows = Path('/proc/stat').read_text().splitlines()
    values = list(map(int, rows[0].split()[1:9]))
    cores = sum(r.startswith('cpu') and r[3:4].isdigit() for r in rows)
    return sum(values), values[3] + values[4], cores


def process_sample():
    result = {}
    for directory in Path('/proc').glob('[0-9]*'):
        try:
            fields = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
            name = (directory / 'comm').read_text().strip()
            result[directory.name] = (name, int(fields[11]) + int(fields[12]))
        except (OSError, ValueError):
            continue
    return result


def udp_sample():
    rows = Path('/proc/net/snmp').read_text().splitlines()
    for header, values in zip(rows, rows[1:]):
        if header.startswith('Udp:') and values.startswith('Udp:'):
            return dict(zip(header.split()[1:], map(int, values.split()[1:])))
    return {}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=12)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--include-3d', action='store_true',
                        help='Subscribe to optional 3D visualization outputs (adds serialization load)')
    args = parser.parse_args()
    if not 3 <= args.seconds <= 120:
        parser.error('--seconds must be between 3 and 120')
    if args.output and args.output.exists():
        parser.error('Refusing to overwrite an existing report')
    rclpy.init()
    node = rclpy.create_node('navigation_readonly_runtime_audit')
    times = collections.defaultdict(list)
    ages = collections.defaultdict(list)
    stamps = collections.defaultdict(list)
    false_events = collections.Counter()
    last = {}
    first_odom = {}
    first_diagnostics = {}
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def receive(topic, message):
        times[topic].append(time.monotonic())
        last[topic] = message
        if isinstance(message, Bool) and not message.data:
            false_events[topic] += 1
        if isinstance(message,String) and topic not in first_diagnostics:
            try:first_diagnostics[topic]=json.loads(message.data)
            except ValueError:pass
        if isinstance(message, Odometry) and topic not in first_odom:
            p = message.pose.pose.position
            first_odom[topic] = np.array([p.x, p.y, p.z])
        if isinstance(message, OccupancyGridUpdate):
            full = last.get(topic.removesuffix('_updates'))
            if full is not None:
                grid = np.asarray(full.data).reshape(full.info.height, full.info.width).copy()
                grid[message.y:message.y + message.height,
                     message.x:message.x + message.width] = np.asarray(message.data).reshape(
                         message.height, message.width)
                full.data = grid.ravel().tolist()
        if hasattr(message, 'header') and (
                message.header.stamp.sec != 0 or message.header.stamp.nanosec != 0):
            stamp = message.header.stamp
            stamps[topic].append(stamp.sec+stamp.nanosec/1.e9)
            age = (node.get_clock().now().nanoseconds / 1.e9 -
                   stamp.sec - stamp.nanosec / 1.e9)
            ages[topic].append(age * 1000)

    topics = [
        ('/livox/lidar', PointCloud2, False),
        ('/livox/imu', Imu, False), ('/cloud_registered_body', PointCloud2, False),
        ('/obstacles/deskewed', PointCloud2, False),
        ('/Odometry', Odometry, False), ('/odom', Odometry, False),
        ('/scan', LaserScan, False), ('/scan_safety', LaserScan, False),
        ('/mapping/environment_map', OccupancyGrid, True),
        ('/traversability/local', OccupancyGrid, True),
        ('/traversability/global', OccupancyGrid, True),
        ('/safety/navigation_local', OccupancyGrid, True),
        ('/safety/navigation_global', OccupancyGrid, True),
        ('/safety/remembered_obstacles', PointCloud2, False),
        ('/local_costmap/costmap', OccupancyGrid, True),
        ('/global_costmap/costmap', OccupancyGrid, True),
        ('/local_costmap/costmap_updates', OccupancyGridUpdate, False),
        ('/global_costmap/costmap_updates', OccupancyGridUpdate, False),
        ('/local_costmap/published_footprint', PolygonStamped, False),
        ('/global_costmap/published_footprint', PolygonStamped, False),
        ('/plan', NavPath, False), ('/cmd_vel_nav', Twist, False),
        ('/cmd_vel', Twist, False),
        ('/cmd_vel_safe', Twist, False), ('/cmd_vel_guarded', Twist, False),
        ('/safety/scan_valid', Bool, True), ('/lio/localization_valid', Bool, True),
        ('/safety/footprint_valid', Bool, True),
        ('/safety/turn_clearance_valid', Bool, True),
        ('/go2/motion_execution_valid', Bool, True),
        ('/safety/motion_diagnostics', String, False),
        ('/safety/obstacle_memory_status', String, True),
    ]
    if args.include_3d:
        topics += [('/mapping/terrain_surface', PointCloud2, True),
                   ('/mapping/environment_cloud', PointCloud2, True)]
    for topic, kind, durable in topics:
        qos = latched if durable else qos_profile_sensor_data
        if topic == '/cloud_registered_body':
            qos = QoSProfile(depth=1)
        if topic == '/livox/lidar':
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(kind, topic, lambda m, t=topic: receive(t, m),
                                 qos)
    clients, queries, states = [], [], []
    for name, parameters in [
        ('environment_mapper', ['update_rate', 'cloud_publish_rate']),
        ('traversability_mapper', ['update_rate']),
        ('livox_lidar_publisher', ['publish_freq']),
        ('laser_mapping', ['preprocess.scan_rate']),
        ('velocity_smoother', ['smoothing_frequency', 'max_velocity']),
        ('global_costmap/global_costmap', ['update_frequency', 'publish_frequency',
             'static_layer.map_topic','inflation_layer.inflation_radius','footprint_padding']),
        ('local_costmap/local_costmap', ['update_frequency', 'publish_frequency',
             'obstacle_layer.scan.topic','traversability_layer.map_topic',
             'inflation_layer.inflation_radius','footprint_padding','width','height']),
        ('controller_server', ['controller_frequency', 'FollowPath.critics',
                               'FollowPath.max_vel_x', 'FollowPath.max_vel_theta',
                               'FollowPath.rotate_to_heading_angular_vel',
                               'FollowPath.max_angular_accel','FollowPath.acc_lim_theta',
                               'FollowPath.decel_lim_theta',
                               'FollowPath.vx_samples','FollowPath.vy_samples','FollowPath.vtheta_samples',
                               'FollowPath.trajectory_generator_name','FollowPath.gait_min_translation',
                               'FollowPath.sim_time','progress_checker.movement_time_allowance',
                               'goal_checker.plugin', 'goal_checker.xy_goal_tolerance',
                               'goal_checker.yaw_goal_tolerance']),
        ('scan_safety_gate', ['max_scan_age', 'max_footprint_age',
                              'command_timeout', 'publish_rate', 'scan_topic','memory_required']),
        ('pointcloud_to_laserscan_node',
         ['ground_filter_enabled', 'observed_free_space_only', 'use_best_effort']),
        ('safety_pointcloud_to_laserscan',
         ['ground_filter_enabled', 'observed_free_space_only', 'scan_time']),
        ('go2_base', ['cmd_vel_topic','max_angular_command']),
        ('obstacle_memory', ['memory_publish_rate','navigation_publish_rate','clearing_rate']),
    ]:
        client = node.create_client(GetParameters, '/' + name + '/get_parameters')
        clients.append(client)
        queries.append((name, parameters, client.call_async(
            GetParameters.Request(names=parameters))))
    for name in ['bt_navigator', 'collision_monitor']:
        client = node.create_client(GetState, '/' + name + '/get_state')
        clients.append(client)
        states.append((name, client.call_async(GetState.Request())))
    ready = time.monotonic() + 2
    while time.monotonic() < ready:
        rclpy.spin_once(node, timeout_sec=0.02)
    times.clear()
    ages.clear()
    stamps.clear()
    false_events.clear()
    first_odom.clear()
    first_diagnostics.clear()
    c0, p0, began = cpu_sample(), process_sample(), time.monotonic()
    udp0 = udp_sample()
    while time.monotonic() - began < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.02)
    duration = time.monotonic() - began
    c1, p1 = cpu_sample(), process_sample()
    udp1 = udp_sample()
    processes = []
    for pid, (name, ticks) in p1.items():
        if pid in p0:
            value = (ticks - p0[pid][1]) * 100 / os.sysconf('SC_CLK_TCK') / duration
            if value > 0.5:
                processes.append(dict(
                    pid=int(pid), name=name, single_core_cpu_pct=round(value, 1)))
    memory = {k: int(v.split()[0]) for k, v in (
        row.split(':', 1) for row in Path('/proc/meminfo').read_text().splitlines())}
    report = dict(seconds=round(duration, 2), online_cores=c1[2],
                  cpu_busy_pct=round(100 * (1 - (c1[1]-c0[1]) / (c1[0]-c0[0])), 1),
                  used_memory_gib=round((memory['MemTotal']-memory['MemAvailable']) / 1048576, 2),
                  total_memory_gib=round(memory['MemTotal'] / 1048576, 2), topics={},
                  parameters={}, lifecycle={},
                  processes=sorted(processes, key=lambda p: -p['single_core_cpu_pct'])[:20])
    report['system_udp_counter_delta'] = {key: value-udp0.get(key, value)
                                          for key, value in udp1.items()}
    for topic, kind, durable in topics:
        samples = times[topic]
        rate = (len(samples)-1) / (samples[-1]-samples[0]) if len(samples) > 1 else None
        item = dict(messages=len(samples), hz=round(rate, 2) if rate is not None else None,
                    publishers=node.count_publishers(topic))
        if ages[topic]:
            item.update(age_ms_median=round(float(np.median(ages[topic])), 1),
                        age_ms_p95=round(float(np.percentile(ages[topic], 95)), 1))
        if len(stamps[topic]) > 1:
            differences=np.diff(stamps[topic])
            item['maximum_stamp_gap_ms'] = round(float(np.max(differences))*1000, 1)
            item['stamp_regressions']=int(np.count_nonzero(differences<0.))
            item['duplicate_stamp_count']=int(np.count_nonzero(differences==0.))
        message = last.get(topic)
        if isinstance(message, Bool):
            item['valid'] = message.data
            item['false_events_during_sample'] = false_events[topic]
        if isinstance(message, Twist):
            item['last_velocity'] = [message.linear.x, message.linear.y, message.angular.z]
        if isinstance(message, String):
            try:
                item['diagnostic'] = json.loads(message.data)
                first=first_diagnostics.get(topic,{})
                keys=('accepted_scans','clearing_jobs','cleared_cells','stale_clearing_results')
                item['counter_deltas']={k:item['diagnostic'][k]-first[k] for k in keys
                    if k in first and k in item['diagnostic']}
            except ValueError:
                item['diagnostic'] = message.data
        if isinstance(message, OccupancyGrid):
            costs = np.asarray(message.data)
            item.update(resolution=message.info.resolution, width=message.info.width,
                        height=message.info.height, free=int((costs == 0).sum()),
                        occupied=int((costs >= 90).sum()), unknown=int((costs < 0).sum()))
        if isinstance(message, LaserScan):
            values = np.asarray(message.ranges)
            item.update(obstacle_bins=int(np.isfinite(values).sum()),
                        observed_clear_bins=int(np.isinf(values).sum()),
                        unknown_bins=int(np.isnan(values).sum()))
        report['topics'][topic] = item
    for name, names, future in queries:
        if future.done() and future.result():
            values = []
            for p in future.result().values:
                values.append({1: p.bool_value, 2: p.integer_value, 3: p.double_value,
                               4: p.string_value, 8: list(p.double_array_value),
                               9: list(p.string_array_value)}.get(p.type))
            report['parameters'][name] = dict(zip(names, values))
    for name, future in states:
        report['lifecycle'][name] = (
            future.result().current_state.label
            if future.done() and future.result() else 'unavailable')
    report['odometry_translation_during_sample_m'] = {}
    for topic, start in first_odom.items():
        p = last[topic].pose.pose.position
        report['odometry_translation_during_sample_m'][topic] = round(
            float(np.linalg.norm(np.array([p.x, p.y, p.z]) - start)), 4)
    node.destroy_node()
    rclpy.shutdown()
    # The subscriptions/large map conversions above add measurable CPU load.
    # Sample again after shutting DDS down, rather than presenting probe load
    # as navigation load. Per-process percentages still refer to one CPU core.
    report['cpu_with_audit_pct'] = report['cpu_busy_pct']
    c0, p0, began = cpu_sample(), process_sample(), time.monotonic()
    time.sleep(5)
    duration = time.monotonic() - began
    c1, p1 = cpu_sample(), process_sample()
    report['cpu_busy_pct'] = round(100 * (1-(c1[1]-c0[1]) / (c1[0]-c0[0])), 1)
    report['cpu_measurement_seconds'] = round(duration, 2)
    processes = []
    for pid, (name, ticks) in p1.items():
        if pid in p0:
            value = (ticks-p0[pid][1]) * 100 / os.sysconf('SC_CLK_TCK') / duration
            if value > 0.5:
                processes.append(dict(
                    pid=int(pid), name=name, single_core_cpu_pct=round(value, 1)))
    report['processes'] = sorted(
        processes, key=lambda p: -p['single_core_cpu_pct'])[:20]
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
