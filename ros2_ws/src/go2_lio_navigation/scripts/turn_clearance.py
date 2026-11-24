"""Check a padded rectangular Go2 envelope along a commanded turning arc."""

import math
import numpy as np


MINIMUM_TURN_SPEED = .50


def braking_horizon(vx, vy, wz, scan_age=0., reaction=.25,
                    linear_decel=.90, angular_decel=1.50):
    """Equivalent arc time for reaction followed by uniform braking to rest.

    Scaling both axes together retains curvature. The travelled arc during
    linear deceleration is half the stopping time, not an arbitrary 1.5 s
    constant-speed turn beyond the controller's intended manoeuvre.
    """
    stop_time = max(math.hypot(vx, vy)/linear_decel, abs(wz)/angular_decel)
    return max(0., scan_age)+reaction+.5*stop_time


def braking_speed_scale(points, vx, vy, wz, half_length, half_width, scan_age=0.):
    # Check the exact gait floor when the next discrete scale would skip it.
    # A command already below the floor may be an ordinary acceleration ramp.
    floor = 0.
    if math.hypot(vx, vy) < .01:
        floor = MINIMUM_TURN_SPEED / max(MINIMUM_TURN_SPEED, abs(wz))
    for ratio in (1., .85, .70, .55, .40, .25):
        ratio = max(ratio, floor)
        x, y, angular = ratio*vx, ratio*vy, ratio*wz
        horizon = braking_horizon(x, y, angular, scan_age)
        if turn_is_clear(points, x, y, angular, half_length, half_width, horizon):
            return ratio
        if ratio == floor:
            break
    return 0.


def arc_collision_time(points, vx, vy, wz, front, half_width, horizon, closing_other=0.,
                       point_closing=None):
    """Predict contact on this commanded arc, with conservative forward closing.

    Only forward-corridor returns are extrapolated towards the robot; other
    returns remain static. This does not identify or track individual people.
    """
    rates = [closing_other]*len(points) if point_closing is None else point_closing
    radius = math.hypot(front, half_width)+(math.hypot(vx, vy)+max(rates,default=0.))*horizon
    cloud = np.asarray(points, dtype=float).reshape(-1, 2)
    rates = np.asarray(rates, dtype=float)
    count = min(len(cloud), len(rates))
    cloud, rates = cloud[:count], rates[:count]
    nearby = np.hypot(cloud[:, 0], cloud[:, 1]) <= radius
    cloud, rates = cloud[nearby], rates[nearby]
    if not len(cloud):return math.inf
    steps = max(1, math.ceil(horizon/.04), math.ceil(abs(wz)*horizon/.025))
    t = horizon*np.arange(steps+1)/steps
    c,s = np.cos(wz*t)[:,None],np.sin(wz*t)[:,None]
    if abs(wz)<1.e-6:
        dx,dy=vx*t,vy*t
    else:
        dx=(vx*s[:,0]+vy*(c[:,0]-1.))/wz
        dy=(vx*(1.-c[:,0])+vy*s[:,0])/wz
    closing=np.where((cloud[:,0]>0.)&(np.abs(cloud[:,1])<=.50),rates,0.)
    px=cloud[None,:,0]-t[:,None]*closing-dx[:,None]
    py=cloud[None,:,1]-dy[:,None]
    contact=((np.abs(c*px+s*py)<=front+1.e-12)&
             (np.abs(-s*px+c*py)<=half_width+1.e-12)).any(axis=1)
    found=np.flatnonzero(contact)
    return float(t[found[0]]) if len(found) else math.inf


def turn_is_clear(points, vx, vy, wz, half_length, half_width, horizon):
    """Return false for even one observed obstacle in the swept rectangle.

    Points and velocities are in base_link at the scan time. This checks known
    returns only; a blind sector is not a source of free-space evidence.
    """
    values = (vx, vy, wz, half_length, half_width, horizon)
    point_model = half_length == 0. and half_width == 0.
    if (not all(math.isfinite(v) for v in values) or horizon <= 0. or
            (not point_model and min(half_length, half_width) <= 0.)):
        return False
    speed = math.hypot(vx, vy)
    radius = math.hypot(half_length, half_width) + speed * horizon
    cloud=np.asarray(points,dtype=float).reshape(-1,2)
    nearby=np.isfinite(cloud).all(axis=1)&(np.hypot(cloud[:,0],cloud[:,1])<=radius)
    cloud=cloud[nearby]
    if not len(cloud):
        return True
    steps = max(1, math.ceil(horizon / .05),
                math.ceil(abs(wz) * horizon / .025), math.ceil(speed * horizon / .02))
    t=horizon*np.arange(steps+1)/steps
    c,s=np.cos(wz*t)[:,None],np.sin(wz*t)[:,None]
    if abs(wz)<1.e-6:
        dx,dy=vx*t,vy*t
    else:
        dx=(vx*s[:,0]+vy*(c[:,0]-1.))/wz
        dy=(vx*(1.-c[:,0])+vy*s[:,0])/wz
    x,y=cloud[None,:,0]-dx[:,None],cloud[None,:,1]-dy[:,None]
    return not bool(np.any((np.abs(c*x+s*y)<=half_length+1.e-12)&
                          (np.abs(-s*x+c*y)<=half_width+1.e-12)))
