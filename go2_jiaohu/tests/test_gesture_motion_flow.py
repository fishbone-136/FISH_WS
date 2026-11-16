"""Motion renewal and asynchronous command tests using fake clients only."""
import json
import os
import queue
import threading
import time

import pytest
from voice import sdk_worker, config
from deploy.gesture_sender import GestureActionSender


class Client:
    def __init__(self):
        self.calls = []
        self.fail_next_move = False

    def Move(self, x, y, z):
        self.calls.append((x, y, z))
        if self.fail_next_move:
            self.fail_next_move = False
            return 3104
        return 0

    def StopMove(self):
        self.calls.append('stop')
        return 0


@pytest.fixture
def motion_loop(monkeypatch):
    monkeypatch.setattr(config, 'ROBOT_SDK_MOVE_INTERVAL', .005)
    replies = queue.Queue()
    monkeypatch.setattr(sdk_worker, '_send_status', lambda status, error='', **kw: replies.put((status, error, kw)))
    client = Client()
    read_fd, write_fd = os.pipe()
    def run():
        with os.fdopen(read_fd) as stream:
            sdk_worker._run_command_loop(client, stream)
    thread = threading.Thread(target=run)
    thread.start()
    def send(action='forward', speed=.3, duration=.2):
        payload = dict(action=action, speed=speed, duration=duration, request_id=str(time.monotonic_ns()))
        os.write(write_fd, (json.dumps(payload)+'\n').encode())
        return replies.get(timeout=1.)[0]
    yield client, send
    os.close(write_fd)
    thread.join(timeout=2)
    assert not thread.is_alive()


def test_identical_motion_renews_without_zero_velocity_or_stop(motion_loop):
    client, send = motion_loop
    assert send(duration=.1) == 'accepted'
    time.sleep(.06)
    assert send(duration=.2) == 'accepted'
    time.sleep(.06)  # Past the first lease; the renewal remains active.
    assert len(client.calls) > 4
    assert set(client.calls) == {(.3, 0., 0.)}


@pytest.mark.parametrize('action,speed', [('backward',.3), ('forward',.2), ('turn_left',.5), ('stop',0.)])
def test_changed_velocity_or_explicit_stop_still_stops_first(motion_loop, action, speed):
    client, send = motion_loop
    send()
    send(action, speed)
    assert 'stop' in client.calls
    index = client.calls.index('stop')
    assert client.calls[index-3:index] == [(0.,0.,0.)]*3


def test_missing_renewal_expires_and_stops(motion_loop):
    client, send = motion_loop
    send(duration=.1)
    deadline = time.monotonic()+.5
    while 'stop' not in client.calls and time.monotonic()<deadline:
        time.sleep(.005)
    assert client.calls[-1] == 'stop'


def test_failed_renewal_stops_and_does_not_accept_new_lease(motion_loop):
    client, send = motion_loop
    send()
    client.fail_next_move = True
    assert send() == 'rejected'
    assert client.calls[-1] == 'stop'


class BlockingRobot:
    last_error = ''

    def __init__(self):
        self.calls = []
        self.entered = threading.Event()
        self.release = threading.Event()

    def execute_action(self, action, **kwargs):
        self.calls.append(action)
        if len(self.calls) == 1:
            self.entered.set()
            assert self.release.wait(2.)
        return True


def test_sender_does_not_block_ui_and_stop_discards_pending_motion():
    robot = BlockingRobot()
    sender = GestureActionSender(robot)
    try:
        sender.submit('forward', .3, .75)
        assert robot.entered.wait(1.)
        started = time.monotonic()
        sender.submit('turn_left', .5, .75)
        sender.submit('backward', .3, .75)
        sender.submit('stop', 0., 0.)
        assert time.monotonic()-started < .05
        robot.release.set()
    finally:
        robot.release.set()
        sender.close()
    assert robot.calls[0] == 'forward'
    assert all(action == 'stop' for action in robot.calls[1:])


def test_sender_drops_motion_that_expired_during_rpc():
    robot = BlockingRobot()
    sender = GestureActionSender(robot)
    try:
        sender.submit('forward', .3, .75)
        assert robot.entered.wait(1.)
        sender.submit('turn_left', .5, .75)
        time.sleep(.32)
        robot.release.set()
        time.sleep(.03)
    finally:
        robot.release.set()
        sender.close()
    assert 'turn_left' not in robot.calls
    assert robot.calls[-1] == 'stop'


def test_deployed_controller_queues_refresh_and_rearms_after_mode_release(monkeypatch):
    from deploy import gesture_controller as module
    events = []
    now = [10.]

    class Robot:
        def __init__(self, **kwargs):
            self.client = self

        def reconnect(self):
            events.append('reconnect')

    class Sender:
        def __init__(self, robot):
            self.requests = []

        def submit(self, *args):
            self.requests.append(args)

        def close(self):
            events.append('close sender')

    monkeypatch.setattr(module, 'Go2RobotController', Robot)
    monkeypatch.setattr(module, 'GestureActionSender', Sender)
    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
    controller = module.GestureController()
    controller.is_system_active = True
    for _ in range(3):
        controller.update({'Left':'open_palm'})
    sender = controller._sender
    assert sender.requests == [('forward', .3, .75)]
    now[0] += .1
    controller.update({'Left':'open_palm'})
    assert len(sender.requests) == 1
    now[0] += .3
    controller.update({'Left':'open_palm'})
    assert len(sender.requests) == 2
    controller.stop_motion()
    controller.stop_motion()
    assert sender.requests[-1] == ('stop', 0., 0.)
    assert len(sender.requests) == 3
    controller.close_motion()
    controller.resume_motion()
    assert events == ['close sender','reconnect']
    assert controller._sender is not sender
