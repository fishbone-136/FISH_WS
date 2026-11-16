"""Serialize gesture RPCs off the GUI thread; retain only fresh intent."""
import logging
import threading
import time

from voice import config


class GestureActionSender:
    def __init__(self, robot):
        self.robot = robot
        self._condition = threading.Condition()
        self._pending = None
        self._stop_pending = False
        self._closed = False
        self._thread = threading.Thread(target=self._run, name='go2-gesture-actions', daemon=True)
        self._thread.start()

    def submit(self, action, speed, duration):
        with self._condition:
            if self._closed:
                return
            if action == 'stop':
                self._pending = None
                self._stop_pending = True
            else:
                # An old movement must not be replayed after a slow RPC.
                self._pending = (action, speed, duration, time.monotonic() + .30)
            self._condition.notify_all()

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._closed or self._stop_pending or self._pending)
                if self._stop_pending:
                    request = ('stop', 0., 0., float('inf'))
                    self._stop_pending = False
                elif self._closed:
                    return
                else:
                    request, self._pending = self._pending, None
            action, speed, duration, expires = request
            if time.monotonic() > expires:
                continue
            try:
                if not self.robot.execute_action(action, speed=speed, duration=duration):
                    logging.error('手势动作未接受: %s (%s)', action, self.robot.last_error)
            except Exception:
                logging.exception('手势动作发送失败: %s', action)

    def close(self):
        with self._condition:
            if not self._closed:
                self._closed = True
                self._pending = None
                self._stop_pending = True
                self._condition.notify_all()
        # Mode handoff waits for the final stop before releasing the bridge.
        timeout = 2 * max(config.ROBOT_COMMAND_ACK_TIMEOUT, config.ROBOT_SDK_RPC_TIMEOUT + .5) + 2
        self._thread.join(timeout=timeout)
        if self._thread.is_alive():
            raise RuntimeError('手势运动桥仍在停止，暂不能切换控制源')
