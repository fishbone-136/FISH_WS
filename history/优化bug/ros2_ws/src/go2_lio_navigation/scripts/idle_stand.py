"""One acknowledged standing transition after a navigation action ends."""


class IdleStand:
    def __init__(self, idle_delay=.8, timeout=2.):
        self.idle_delay, self.timeout = idle_delay, timeout
        self.active = False
        self.armed = False
        self.terminal_time = None
        self.zero_since = None
        self.phase = 'idle'
        self.pending = None
        self.deadline = 0.
        self.error = None

    def navigation(self, active, now):
        if active and not self.active:
            self.armed = True
            self.phase, self.pending, self.error = 'idle', None, None
            self.terminal_time = None
        elif not active and self.active:
            self.terminal_time = now
        self.active = active

    def velocity(self, zero, now):
        if zero:
            if self.zero_since is None:
                self.zero_since = now
        else:
            self.zero_since = None
            # Smoothing can finish decelerating after the action status turns
            # terminal. Keep that action armed, but cancel any in-flight settle
            # sequence and require a new continuous idle interval.
            if self.pending is not None or self.phase == 'stop_ack':
                self.pending, self.phase = None, 'idle'

    def suspend(self):
        self.armed, self.pending, self.phase = False, None, 'idle'

    def eligible(self, now, fresh_and_stationary):
        return (self.armed and not self.active and fresh_and_stationary and
                self.terminal_time is not None and self.zero_since is not None and
                now-max(self.terminal_time, self.zero_since) >= self.idle_delay)

    def tick(self, now, fresh_and_stationary, request_id):
        if self.pending is not None and now >= self.deadline:
            self.error = 'RPC acknowledgement timed out'
            self.suspend()
        if not self.eligible(now, fresh_and_stationary):
            return None
        if self.phase == 'idle':
            api = 1003
        elif self.phase == 'stop_ack' and now >= self.deadline:
            api = 1002
        else:
            return None
        self.pending, self.deadline = request_id, now+self.timeout
        self.phase = 'waiting_stop' if api == 1003 else 'waiting_stand'
        return api

    def response(self, request_id, api, code, now):
        expected = 1003 if self.phase == 'waiting_stop' else 1002
        if self.pending != request_id or api != expected:
            return False
        if now >= self.deadline:
            self.error = 'RPC acknowledgement arrived after deadline'
            self.suspend()
            return False
        self.pending = None
        if code != 0:
            self.error = f'API {api} returned {code}'
            self.suspend()
        elif api == 1003:
            self.phase, self.deadline = 'stop_ack', now+.3
        else:
            self.phase, self.armed = 'done', False
        return True
