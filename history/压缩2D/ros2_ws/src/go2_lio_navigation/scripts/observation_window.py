"""Track actual forward-observation age across sparse HAP scan frames."""

import math


class ForwardObservationWindow:
    def __init__(self):
        self.stamp = self.received = -math.inf

    def observe(self, count, stamp, received, supported_frame=True):
        if not supported_frame:
            self.stamp = self.received = -math.inf
        elif count >= 2 and math.isfinite(stamp) and math.isfinite(received):
            self.stamp, self.received = stamp, received

    def fresh(self, now, steady, max_age):
        return (-0.1 <= now-self.stamp <= max_age and
                0.0 <= steady-self.received <= max_age)


def bound_sparse_command(requested, previous):
    """A sparse frame may continue a safe command, never release a stop.

    A new axis/direction stays zero until observed again. Other existing axes
    can continue without acceleration; the caller MUST check the resulting
    arc again. A tiny yaw reversal must not inject a whole-body StopMove.
    """
    ratio = 1.0
    bounded = []
    for current, old in zip(requested, previous):
        if not math.isfinite(current) or not math.isfinite(old):
            return (0., 0., 0.)
        if abs(current) <= 1.e-6:
            bounded.append(0.)
            continue
        if current*old <= 0.:
            bounded.append(0.)
        else:
            bounded.append(current)
            ratio = min(ratio, abs(old/current))
    return tuple(ratio*x for x in bounded)
