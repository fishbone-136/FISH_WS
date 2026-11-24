#!/usr/bin/env python3
"""Read-only live audit of the Unitree stop supplement; sends no commands."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/go2_lio_navigation/scripts'))
from unitree_near_guard import UnitreeNearGuard, decode_cloud


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=8.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or not 3. <= args.seconds <= 30.:
        parser.error('Use a new output and duration 3..30 seconds')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    rclpy.init()
    node = rclpy.create_node('unitree_near_guard_readonly_audit')
    guard = UnitreeNearGuard()
    counts, reasons, errors = Counter(), Counter(), []
    timings, offsets, snapshots = [], [], []

    def observe(message, kind):
        began = time.perf_counter()
        received = time.monotonic()
        try:
            expected = 'utlidar_lidar' if kind == 'raw' else 'base_link'
            if message.header.frame_id != expected:
                raise ValueError('Unexpected frame '+message.header.frame_id)
            points, keys = decode_cloud(message)
            stamp = message.header.stamp.sec+message.header.stamp.nanosec/1.e9
            if kind == 'raw':
                guard.observe_raw(points, keys, stamp, received)
                offsets.append(time.time()-stamp)
            else:
                guard.observe_body(points, keys, stamp, received)
            counts[kind] += 1
        except (ValueError, TypeError, BufferError) as exc:
            errors.append(str(exc)); guard.invalidate()
        timings.append(1000.*(time.perf_counter()-began))

    node.create_subscription(PointCloud2, '/utlidar/cloud', lambda m: observe(m, 'raw'), qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/utlidar/cloud_base', lambda m: observe(m, 'body'), qos_profile_sensor_data)
    began = time.monotonic()
    cpu_began = time.process_time()
    last_sample = -1.
    try:
        while time.monotonic()-began < args.seconds:
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            if now-last_sample >= .1:
                last_sample = now
                reason = guard.reason(now)
                reasons[reason] += 1
                snapshots.append({'elapsed': round(now-began, 3), 'reason': reason,
                                  'near_returns': len(guard.near_points)})
        result = {'read_only': True, 'policy': 'directional_braking_v2',
            'assessment': 'sensor health only; no motion trajectory evaluated',
            'messages': dict(counts), 'reasons': dict(reasons),
            'errors': errors, 'snapshots': snapshots,
            'factory_transform_max_error_m': guard.calibration_error if guard.checked > 0. else None,
            'callback_max_ms': max(timings, default=0.),
            'cpu_one_core_percent': 100.*(time.process_time()-cpu_began)/(time.monotonic()-began),
            'source_clock_offset_seconds': offsets[-1] if offsets else None,
            'initial_absolute_transport_delay_verified': False}
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='snapshots'}, ensure_ascii=False))
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
