"""Stop-only supplement using unfiltered Unitree returns, without clock rewriting.

The measured factory raw->body transform is checked against same-stamp factory
body points at runtime. It is NOT an HAP extrinsic calibration. Relative source
time detects freezing, queue growth and clock jumps; an initial constant transport
delay cannot be measured without a synchronized device clock.
"""
import math
from collections import OrderedDict, deque
import numpy as np
from turn_clearance import braking_speed_scale


from go2_perception.unitree_cloud import (
    ROTATION, TRANSLATION, decode_cloud, factory_transform_error,
)


class UnitreeNearGuard:
    def __init__(self, front=.38, half_width=.15):
        if not all(math.isfinite(v) and v > 0. for v in (front, half_width)):
            raise ValueError('Invalid body geometry')
        self.front, self.half_width = front, half_width
        self.received = self.stamp = self.checked = -math.inf
        self.anchor = self.clear_since = None
        self.good_frames = 0
        self.condition = 'unitree_unavailable'
        self.calibration_error = math.inf
        self.near_points = np.empty((0, 3))
        self.pending = OrderedDict()
        # A sparse frame cannot instantly erase a return. Retain short-lived
        # geometry in accepted odometry coordinates, not a global stop latch.
        self.history = deque(maxlen=8)
        self.hold_seconds = .20
        self.geometry_revision = 0
        self.geometry_key = None
        self.motion_points = np.empty((0, 2))

    def invalidate(self):
        self.condition = 'unitree_invalid'
        self.clear_since = None
        self.good_frames = 0
        self.checked = -math.inf
        self.history.clear()
        self.geometry_revision += 1

    def _pair(self, kind, stamp, points, keys, received):
        pair = self.pending.setdefault(stamp, {})
        pair[kind] = (points, keys)
        while len(self.pending) > 4:
            self.pending.popitem(last=False)
        if len(pair) != 2:
            return
        self.pending.pop(stamp)
        # Check at 2 Hz, using at most 128 exact metadata correspondences.
        if received-self.checked < .5:
            return
        self.calibration_error = factory_transform_error(pair['raw'], pair['body'])
        if self.calibration_error > .005:
            self.invalidate()
            return
        self.checked = received

    def observe_body(self, points, keys, stamp, received):
        self._pair('body', stamp, points, keys, received)

    def observe_raw(self, points, keys, stamp, received, pose=None):
        points = np.asarray(points).reshape(-1, 3)
        previous_stamp, previous_received = self.stamp, self.received
        self.stamp, self.received = stamp, received
        valid = (math.isfinite(stamp) and math.isfinite(received) and
                 stamp > previous_stamp and received > previous_received)
        gap = received-previous_received > .2 or stamp-previous_stamp > .2
        if self.anchor is None or gap:
            self.anchor = received-stamp
            self.clear_since = None
            self.good_frames = 0
        # Keep the initial relative clock anchor. Do not chase growing delay.
        lag = received-stamp-self.anchor
        valid = valid and -.08 <= lag <= .15
        real = np.isfinite(points).all(1) & (np.linalg.norm(points, axis=1) > .01)
        body = points[real] @ ROTATION.T + TRANSLATION
        # Observe the front and adjacent corridor walls. This is a candidate
        # region, NOT an unconditional stop box. Own-body exclusion stays put.
        near = ((body[:, 0] >= self.front+.02) & (body[:, 0] <= self.front+.47) &
                (np.abs(body[:, 1]) <= .85) &
                (body[:, 2] >= -.18) & (body[:, 2] <= .8))
        self.near_points = body[near]
        self.condition = ('unitree_clock_invalid' if not valid else
                          'unitree_blind' if np.count_nonzero(real) < 32 else 'none')
        if self.condition != 'none':
            self.clear_since = None
            self.good_frames = 0
            self.history.clear()
            self.geometry_revision += 1
            if not valid:
                self.anchor = None
        else:
            if self.clear_since is None:
                self.clear_since = received
            self.good_frames += 1
            self._prune_geometry(received)
            if len(self.near_points):
                self.history.append((received, self.near_points[:, :2].copy(), pose))
                self.geometry_revision += 1
        self._pair('raw', stamp, points, keys, received)

    def _prune_geometry(self, steady):
        while self.history and steady-self.history[0][0] > self.hold_seconds+1.e-9:
            self.history.popleft()
            self.geometry_revision += 1

    def geometry(self, steady, pose):
        """Express retained observations in the current accepted body pose."""
        self._prune_geometry(steady)
        key = (self.geometry_revision, pose)
        if key == self.geometry_key:
            return self.motion_points
        transformed = []
        for _, points, original in self.history:
            if pose is None or original is None:
                return None
            if not all(math.isfinite(v) for v in (*pose, *original)):
                return None
            c, s = math.cos(original[2]), math.sin(original[2])
            world = points @ np.array([[c, s], [-s, c]]) + original[:2]
            c, s = math.cos(pose[2]), math.sin(pose[2])
            transformed.append((world-pose[:2]) @ np.array([[c, -s], [s, c]]))
        self.motion_points = np.concatenate(transformed) if transformed else np.empty((0, 2))
        self.geometry_key = key
        return self.motion_points

    def motion_scale(self, vx, vy, wz, steady, pose, half_length=None, half_width=None):
        """Check the requested stopping arc; safe turns do not need empty space.

        Every return is checked, including a single thin pole. Source health
        recovery is independent of obstacle presence. Missing ego poses cannot
        silently turn retained body-relative observations into free space.
        """
        reason = self.reason(steady)
        if reason != 'none':
            return 0., reason
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            return 0., 'unitree_invalid_command'
        if max(abs(vx), abs(vy), abs(wz)) <= 1.e-6:
            return 1., 'none'
        points = self.geometry(steady, pose)
        if points is None:
            return 0., 'unitree_pose_unavailable'
        length = self.front if half_length is None else half_length
        width = self.half_width if half_width is None else half_width
        if not all(math.isfinite(v) and v > 0. for v in (length, width)):
            return 0., 'unitree_invalid_geometry'
        # One centimetre numerical/measurement margin, never costmap inflation.
        # Include the oldest retained acquisition interval in the stopping arc.
        age = max((steady-t for t, _, _ in self.history), default=steady-self.received)
        scale = braking_speed_scale(points, vx, vy, wz, length+.01, width+.01, age)
        return scale, 'unitree_braking_arc_blocked' if scale == 0. else 'none'

    def reason(self, steady):
        if not 0. <= steady-self.received <= .2:
            self.clear_since = None
            self.good_frames = 0
            return 'unitree_unavailable'
        if self.condition != 'none':
            return self.condition
        if not 0. <= steady-self.checked <= 1.:
            self.clear_since = None
            self.good_frames = 0
            return 'unitree_transform_unverified'
        if self.good_frames < 4 or self.clear_since is None or steady-self.clear_since < .2:
            return 'unitree_recovery'
        return 'none'
