"""Pre-projection HAP visibility and near-contact interlock, independent of LIO.

No-return/zero-coordinate packets are unknown, never evidence of free space.
The guard examines every real return; it does not downsample safety geometry.
"""
import math
import numpy as np


class RawLidarGuard:
    def __init__(self, sensor_x=.14, front=.38, half_width=.15,
                 max_age=.25, receive_timeout=.15, recovery_seconds=.10):
        values = (sensor_x, front, half_width, max_age, receive_timeout, recovery_seconds)
        if (not all(math.isfinite(v) for v in values) or front <= sensor_x or
                min(half_width, max_age, receive_timeout, recovery_seconds) <= 0.):
            raise ValueError('Invalid raw LiDAR safety geometry or timing')
        self.sensor_x, self.front, self.half_width = sensor_x, front, half_width
        self.max_age, self.receive_timeout = max_age, receive_timeout
        self.recovery_seconds = recovery_seconds
        self.stamp = self.received = -math.inf
        self.clear_since = self.clear_received_since = None
        self.good_frames = self.forward_returns = self.near_returns = 0
        self.condition = 'raw_lidar_unavailable'

    def observe(self, points, stamp, received, supported_frame=True):
        previous_stamp, previous_received = self.stamp, self.received
        self.stamp, self.received = stamp, received
        self.forward_returns = self.near_returns = 0
        if (not supported_frame or not math.isfinite(stamp) or
                not math.isfinite(received) or stamp <= previous_stamp):
            self.condition = 'raw_lidar_invalid'
        else:
            cloud = np.asarray(points, dtype=float).reshape(-1, 3)
            # Zero placeholders and out-of-range/behind-sensor returns cannot
            # certify forward visibility. Any genuine close return still stops.
            finite = np.isfinite(cloud).all(axis=1)
            x, y, z = cloud.T
            radius = np.linalg.norm(cloud, axis=1)
            real = finite & (radius > .005) & (radius <= 8.) & (x > .005)
            forward = real & (np.abs(y) <= x * math.tan(.35))
            self.forward_returns = int(np.count_nonzero(forward))
            # Deliberately precedes own-body and ground projection filters.
            # A low floor (-~0.5 m in sensor coordinates) is outside this band.
            near = (real & (x <= self.front - self.sensor_x + .05) &
                    (np.abs(y) <= self.half_width + .03) & (z >= -.18) & (z <= .9))
            self.near_returns = int(np.count_nonzero(near))
            self.condition = ('raw_lidar_near_contact' if self.near_returns else
                              'raw_lidar_blind' if self.forward_returns < 3 else 'none')
        if self.condition != 'none':
            self.clear_since = self.clear_received_since = None
            self.good_frames = 0
        else:
            if (stamp-previous_stamp > self.max_age or
                    received-previous_received > self.receive_timeout):
                self.clear_since = self.clear_received_since = None
                self.good_frames = 0
            if self.clear_since is None:
                self.clear_since, self.clear_received_since = stamp, received
            self.good_frames += 1

    def reason(self, now, steady):
        if not (-.05 <= now-self.stamp <= self.max_age and
                0. <= steady-self.received <= self.receive_timeout):
            # A timeout must re-arm recovery even when the sensor clock jumps
            # backwards or callbacks resume in a burst after being blocked.
            self.clear_since = self.clear_received_since = None
            self.good_frames = 0
            return 'raw_lidar_unavailable'
        if self.condition != 'none':
            return self.condition
        if (self.good_frames < 3 or self.clear_since is None or
                self.stamp-self.clear_since < self.recovery_seconds-1.e-9 or
                self.received-self.clear_received_since < self.recovery_seconds-1.e-9):
            return 'raw_lidar_recovery'
        return 'none'
