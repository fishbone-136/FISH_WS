"""Immediate scan braking and bounded recovery after a safety intervention."""

import math

import numpy as np

from turn_clearance import arc_collision_time, braking_horizon, turn_is_clear


class ObstacleGeometryCache:
    """Reuse geometry only while its exact scan, memory and accepted pose agree.

    Freshness is deliberately not cached: the caller must check message ages
    on every control tick. Sources are retained to prevent Python id reuse.
    """

    def __init__(self):
        self.scan = self.memory = self.pose = None
        self.supported = None
        self.remembered, self.combined = [], []
        self.remembered_array = np.empty((0, 2))
        self.combined_array = np.empty((0, 2))

    def update(self, scan, memory, pose, supported):
        if (scan is self.scan and memory is self.memory and pose == self.pose and
                supported == self.supported):
            return self.remembered, self.combined
        remembered = []
        if supported and pose is not None:
            px, py, yaw = pose
            c, s = math.cos(yaw), math.sin(yaw)
            delta = memory - (px, py)
            delta = delta[np.sum(delta * delta, axis=1) <= 2.25**2]
            dx, dy = delta[:, 0], delta[:, 1]
            remembered = np.column_stack((c * dx + s * dy, -s * dx + c * dy)).tolist()
        self.scan, self.memory, self.pose, self.supported = scan, memory, pose, supported
        self.remembered, self.combined = remembered, scan + remembered
        self.remembered_array = np.asarray(remembered, dtype=float).reshape(-1, 2)
        self.combined_array = np.asarray(self.combined, dtype=float).reshape(-1, 2)
        return self.remembered, self.combined


def motion_stop_reason(scan_valid, footprint_valid, memory_valid, command_fresh,
                       finite_command, hard_clear, turn_clear):
    """Describe the first blocking condition; idle input is reported separately."""
    for valid, reason in ((scan_valid, 'scan_unavailable'),
                          (footprint_valid, 'footprint_unavailable'),
                          (memory_valid, 'memory_unavailable'),
                          (command_fresh, 'command_timeout'),
                          (finite_command, 'invalid_command'),
                          (hard_clear, 'occupied_footprint'),
                          (turn_clear, 'braking_arc_blocked')):
        if not valid:
            return reason
    return 'none'


def stop_zone_clear(points, front=.51, rear=.51, half_width=.33):
    cloud = np.asarray(points, dtype=float).reshape(-1, 2)
    return not bool(np.any((cloud[:, 0] >= -rear) & (cloud[:, 0] <= front) &
                          (np.abs(cloud[:, 1]) <= half_width)))


class ClosingApproach:
    """Estimate surface-normal approach after measured ego-motion compensation.

    This is a conservative range-rate guard, not pedestrian identification or
    object tracking. A new nearby return is handled on its first frame too.
    """

    def __init__(self, stop_front=.51, half_width=.50, horizon=1.2):
        self.stop_front, self.half_width, self.horizon = stop_front, half_width, horizon
        self.distance = math.inf
        self.stamp = -math.inf
        self.closing = 0.
        self.points = []
        self.rates = []
        self.tracks = []

    def observe(self, points, stamp, ego_velocity=(0., 0., 0.)):
        self.points = list(points)
        distance = min((x for x, y in points if x > 0. and abs(y) <= self.half_width),
                       default=math.inf)
        dt = stamp-self.stamp
        rates, tracks = [], []
        previous = []
        if .02 <= dt <= .25 and all(math.isfinite(v) for v in ego_velocity):
            vx, vy, wz = ego_velocity
            c, s = math.cos(wz*dt), math.sin(wz*dt)
            for px, py, old_rate in self.tracks:
                px, py = px-vx*dt, py-vy*dt
                previous.append((c*px+s*py, -s*px+c*py, old_rate))
        # Batch correspondence in NumPy and fit each 5 cm surface neighborhood
        # once per frame. Re-fitting a dense patch for every individual return
        # in Python can block the scan callback far beyond its 15 Hz budget.
        previous = np.asarray(previous, dtype=float).reshape(-1, 3)
        nearest = {}
        models = {}
        eligible = [(i, x, y) for i, (x, y) in enumerate(self.points)
                    if x > 0. and abs(y) <= self.half_width]
        if eligible and len(previous):
            current = np.asarray([(x, y) for _, x, y in eligible])
            dx = previous[None, :, 0]-current[:, None, 0]
            dy = previous[None, :, 1]-current[:, None, 1]
            distances = dx*dx+dy*dy
            distances[(np.abs(dy) > .06) | (np.abs(dx) > 4.*dt+.03)] = np.inf
            indices = np.argmin(distances, axis=1)
            nearest = {index:int(j) for row, ((index, _, _), j) in
                       enumerate(zip(eligible, indices)) if math.isfinite(distances[row, j])}
        for index, (x, y) in enumerate(self.points):
            candidate = confirmed = 0.
            if index in nearest:
                px, py, old_rate = previous[nearest[index]]
                # Nonrepetitive scanning can slide along a static slanted
                # surface. Nearest-point longitudinal differences then
                # invent incoming motion. Measure normal displacement of
                # a locally supported line instead; reject noisy corners
                # and extrapolation beyond the observed surface.
                key = (math.floor(px/.05), math.floor(py/.05))
                if key not in models:
                    cx, cy = (key[0]+.5)*.05, (key[1]+.5)*.05
                    nearby = previous[(previous[:, 0]-cx)**2+
                                      (previous[:, 1]-cy)**2 <= .25**2]
                    models[key] = (len(nearby), self.surface_model(nearby))
                count, model = models[key]
                candidate = self.model_rate(model, x, y, dt)
                # A single newly close return must still trigger imminent
                # braking without waiting for a supported surface track.
                raw_rate = max(0., (px-x)/dt) if px-x > .03 else 0.
                # Preserve tracking of an isolated thin return if its
                # lateral correspondence is stable. Ambiguous pairs and
                # corners cannot fall back to longitudinal nearest-point
                # motion. As below, a non-imminent rate needs confirmation.
                if candidate is None and count == 1 and abs(py-y) <= .015:
                    candidate = raw_rate
                if (candidate is None and abs(y) <= .25 and
                        x <= self.stop_front+.35 and raw_rate > 1. and
                        (x-self.stop_front)/raw_rate < .25):
                    candidate = raw_rate
                candidate = candidate or 0.
                # Two consistent intervals reject a newly sampled static
                # edge without delaying first-frame geometric braking.
                imminent = candidate > 1. and (x-self.stop_front)/candidate < .25
                if candidate > .2 and (imminent or (
                        old_rate > .2 and abs(candidate-old_rate) < 1.0)):
                    confirmed = max(candidate, old_rate)
            rates.append(float(confirmed))
            if x > 0. and abs(y) <= self.half_width:
                tracks.append((x, y, candidate))
        self.rates, self.tracks = rates, tracks
        self.closing = max(rates, default=0.)
        self.distance, self.stamp = distance, stamp

    @staticmethod
    def surface_model(points):
        if len(points) < 3:
            return None
        points = np.asarray(points)[:, :2]
        mx, my = points.mean(axis=0)
        centered = points-(mx, my)
        xx, yy = np.mean(centered*centered, axis=0)
        xy = np.mean(centered[:, 0]*centered[:, 1])
        delta = math.hypot(xx-yy, 2*xy)
        major, minor = .5*(xx+yy+delta), max(0., .5*(xx+yy-delta))
        if major < .0003 or minor > .10*major:
            return None
        tangent = .5*math.atan2(2*xy, xx-yy)
        tx, ty = math.cos(tangent), math.sin(tangent)
        nx, ny = -ty, tx
        if nx < 0.:
            nx, ny = -nx, -ny
        if nx < .5:
            return None
        support = centered[:, 0]*tx+centered[:, 1]*ty
        return (mx, my, tx, ty, nx, ny, max(.025, 3*math.sqrt(minor)),
                float(support.min())-.03, float(support.max())+.03)

    @staticmethod
    def model_rate(model, x, y, dt):
        if model is None:
            return None
        mx, my, tx, ty, nx, ny, noise, low, high = model
        along = (x-mx)*tx+(y-my)*ty
        if not low <= along <= high:
            return None
        displacement = nx*(mx-x)+ny*(my-y)
        if displacement <= noise:
            return 0.
        return max(0., nx*displacement/dt)

    @classmethod
    def surface_rate(cls, points, x, y, dt):
        return cls.model_rate(cls.surface_model(points), x, y, dt)

    def scale(self, vx, now, vy=0., wz=0., path_half_width=.33):
        if vx <= 1.e-6 or not math.isfinite(self.distance):
            return 1.
        age = max(0., now-self.stamp)
        contact = arc_collision_time(
            self.points, vx, vy, wz, self.stop_front, path_half_width,
            age+.20+self.horizon, point_closing=self.rates)
        if not math.isfinite(contact):
            return 1.
        # Reserve 0.20 s before predicted contact for command/physical response.
        available_time = contact-age-.20
        return max(0., min(1., available_time/self.horizon))

    def braking_scale(self, vx, vy, wz, now, front, half_width):
        """Hard recheck after recovery; do not compound the soft TTC fraction.

        Static returns use the existing decelerating swept arc. Moving returns
        additionally cover the full physical response/stopping interval, with
        conservative constant ego velocity over that interval. Every emitted
        ratio is checked itself, rather than multiplying another soft limit
        into the already limited acceleration ramp on every 20 ms tick.
        """
        age = max(0., now-self.stamp)
        moving = [(p, rate) for p, rate in zip(self.points,self.rates) if rate > 0.]
        for ratio in (1., .85, .70, .55, .40, .25):
            x,y,w = ratio*vx,ratio*vy,ratio*wz
            if ratio < 1. and math.hypot(vx,vy)<.01 and abs(w)<.50:
                break
            if not turn_is_clear(self.points,x,y,w,front,half_width,
                                 braking_horizon(x,y,w,age)):
                continue
            horizon = age+.25+max(math.hypot(x,y)/.90,abs(w)/1.50)
            if moving and math.isfinite(arc_collision_time(
                    [p for p,rate in moving],x,y,w,front,half_width,horizon,
                    point_closing=[rate for p,rate in moving])):
                continue
            return ratio
        return 0.


def close_clearance_scale(points, vx, vy, wz, half_length=.51, half_width=.33):
    """Keep passing a nearby obstacle slow, without declaring its side unsafe."""
    cloud = np.asarray(points, dtype=float).reshape(-1, 2)
    if math.hypot(vx,vy)<.01 and abs(wz)>1.e-6:
        # A pure turn stays inside the padded body's circumscribed circle.
        # Use its actual clearance, rather than repeatedly throttling because
        # an object is near the front rectangle but outside the whole sweep.
        radius=math.hypot(half_length,half_width)
        clearance=float(np.min(np.hypot(cloud[:, 0],cloud[:, 1])-radius,initial=np.inf))
        blend=max(0.,min(1.,(clearance-.10)/.22))
        limit=.65+.75*blend
        return min(1.,limit/abs(wz))
    cloud = cloud[(cloud[:, 0] >= -half_length) & (cloud[:, 0] <= 1.0)]
    clearance = float(np.min(np.hypot(np.maximum(np.abs(cloud[:, 0])-half_length, 0.),
                                    np.maximum(np.abs(cloud[:, 1])-half_width, 0.)),
                             initial=np.inf))
    if clearance >= .35:
        return 1.
    # Retain full caution within 20 cm of the protected footprint; continuously
    # join normal speed across the next 15 cm instead of a binary limit jump.
    blend = max(0., min(1., (clearance-.20)/.15))
    # Soft passing caution must stay above the Go2's stable 0.22 m/s gait
    # threshold. The subsequent full-footprint braking check can still stop.
    linear_limit, angular_limit = .26+.39*blend, .65+.75*blend
    speed = math.hypot(vx, vy)
    return min(1., linear_limit/speed if speed > linear_limit else 1.,
               angular_limit/abs(wz) if abs(wz) > angular_limit else 1.)


class TurnRecoveryHold:
    """Stop immediately, then wait briefly for a stable clear turn."""

    def __init__(self, clear_seconds=.10):
        self.clear_seconds = clear_seconds
        self.blocked = False
        self.clear_since = None

    def allows(self, turning, permitted, now):
        if not turning:
            self.clear_since = None
            return True
        if not permitted:
            self.blocked = True
            self.clear_since = None
            return False
        if not self.blocked:
            return True
        if self.clear_since is None:
            self.clear_since = now
        if now - self.clear_since < self.clear_seconds:
            return False
        self.blocked = False
        self.clear_since = None
        return True


class SafetyRecoveryRamp:
    """Apply safety reductions immediately; ramp only recovery accelerations.

    Limit recovery per axis so a small yaw reversal cannot collapse a steady
    forward command. The caller MUST collision-check the resulting arc again:
    independently bounded components can change its curvature. Safety
    reductions and zero commands are never delayed.
    """

    def __init__(self, linear_accel=.70, angular_accel=1.20):
        self.accels = (linear_accel, linear_accel, angular_accel)
        self.previous = (0., 0., 0.)
        self.last_time = None
        self.armed = False

    def apply(self, requested, now, limited):
        dt = 0. if self.last_time is None else max(0., min(.05, now-self.last_time))
        self.last_time = now
        self.armed = self.armed or limited
        if not all(math.isfinite(x) for x in requested):
            requested, self.armed = (0., 0., 0.), True
        bounded = list(requested)
        if self.armed:
            for i, (value, old, acceleration) in enumerate(zip(
                    requested, self.previous, self.accels)):
                if abs(value) > 1.e-6:
                    initial = abs(old) if value*old > 0. else 0.
                    bounded[i] = math.copysign(min(abs(value), initial+acceleration*dt), value)
            if not limited and tuple(bounded) == requested and any(
                    abs(x) > 1.e-6 for x in requested):
                self.armed = False
        self.previous = tuple(bounded)
        return self.previous

    def commit_checked(self, command):
        """Record the command actually emitted after the final arc check."""
        self.previous = tuple(command)
