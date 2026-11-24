"""Back off optional map/tracking work under host load; never gate safety I/O."""
import time


class BackgroundBudget:
    def __init__(self, high=.85, low=.75):
        if not 0. < low < high < 1.:
            raise ValueError('Invalid CPU budget hysteresis')
        self.high, self.low = high, low
        self.overloaded = False
        self.busy = None
        self.previous = None
        self.last_sample = -float('inf')
        self.last_jobs = {}

    def update(self, total, idle):
        if self.previous is not None:
            dt, di = total-self.previous[0], idle-self.previous[1]
            if dt > 0 and 0 <= di <= dt:
                self.busy = 1.-di/dt
                if self.busy >= self.high:
                    self.overloaded = True
                elif self.busy <= self.low:
                    self.overloaded = False
        self.previous = total, idle

    def sample(self, now=None):
        now = time.monotonic() if now is None else now
        if now-self.last_sample < 1.:
            return
        self.last_sample = now
        try:
            with open('/proc/stat') as stream:
                values = list(map(int, stream.readline().split()[1:9]))
            self.update(sum(values), values[3]+values[4])
        except (OSError, ValueError, IndexError):
            # If host accounting is unavailable, use the lower background rate.
            self.overloaded = True

    def due(self, name, normal_hz, loaded_hz, now=None, poll_interval=0.):
        now = time.monotonic() if now is None else now
        self.sample(now)
        rate = loaded_hz if self.overloaded else normal_hz
        period = 1. / rate
        previous = self.last_jobs.get(name, -float('inf'))
        if now - previous < period:
            return False
        # Retain at most one poll's phase error. A slow timer never accumulates
        # missed work to run in a burst when the executor catches up.
        self.last_jobs[name] = max(previous + period, now - poll_interval)
        return True
