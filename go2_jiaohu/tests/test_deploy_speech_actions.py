"""Exercise the real voice scheduler with a fake SDK; never connect a robot."""
import threading
from types import SimpleNamespace

from deploy import speech_controller as speech
from offline_test.speech_controller import SpeechController as LocalController
from voice.command_safety import SafeAction


class Robot:
    def __init__(self):
        self.client = object()
        self.backend = 'fake'
        self.last_error = ''
        self.calls = []

    def execute_action(self, name, **kwargs):
        self.calls.append(name)
        return True


def controller(monkeypatch, robot=None):
    ctrl = speech.SpeechController.__new__(speech.SpeechController)
    LocalController.__init__(ctrl)
    ctrl._robot = robot or Robot()
    ctrl._action_stop = threading.Event()
    ctrl._action_thread = None
    ctrl._guidance = SimpleNamespace(is_running=False)
    ctrl._running = True
    ctrl.statuses, ctrl.errors = [], []
    ctrl.on_status = ctrl.statuses.append
    ctrl.on_error = ctrl.errors.append
    monkeypatch.setattr(speech, 'navigation_is_idle', lambda: True)
    return ctrl


def test_sequence_has_per_step_ack_and_final_stop(monkeypatch):
    ctrl = controller(monkeypatch)
    assert ctrl._execute_actions([SafeAction('forward', .3, .01),
                                  SafeAction('turn_left', .5, .01, 1)])
    ctrl._action_thread.join(2)
    assert ctrl._robot.calls[-3:] == ['forward', 'turn_left', 'stop']
    assert any('第 1/2' in s for s in ctrl.statuses)
    assert any('第 2/2' in s for s in ctrl.statuses)
    assert any('停止回执' in s for s in ctrl.statuses)


def test_navigation_start_aborts_remaining_actions(monkeypatch):
    ctrl = controller(monkeypatch)
    active = threading.Event()
    monkeypatch.setattr(speech, 'navigation_is_idle', lambda: not active.is_set())
    assert ctrl._execute_actions([SafeAction('forward', .3, 1), SafeAction('backward', .3, 1)])
    active.set()
    ctrl._action_thread.join(2)
    assert 'backward' not in ctrl._robot.calls
    assert ctrl._robot.calls[-1] == 'stop'
    assert any('导航' in s for s in ctrl.errors)


def test_slow_cancelled_worker_cannot_be_revived_by_new_sequence(monkeypatch):
    ctrl = controller(monkeypatch)
    entered, release = threading.Event(), threading.Event()
    send = ctrl._robot.execute_action

    def execute(name, **kwargs):
        result = send(name, **kwargs)
        if name == 'turn_left':
            entered.set()
            assert release.wait(5)
        return result

    ctrl._robot.execute_action = execute
    assert ctrl._execute_actions([SafeAction('forward', .3, .01),
                                  SafeAction('turn_left', .5, .01, 1), SafeAction('stand')])
    worker = ctrl._action_thread
    assert entered.wait(2)
    old_cancel = ctrl._action_stop
    try:
        assert not ctrl._execute_actions([SafeAction('backward', .3, .01)])
        assert old_cancel.is_set()
        assert ctrl._action_thread is worker
        assert 'backward' not in ctrl._robot.calls
    finally:
        release.set()
        worker.join(2)
    assert 'stand' not in ctrl._robot.calls
    assert ctrl._robot.calls[-1] == 'stop'


def test_disabled_during_connect_cannot_dispatch_late_action(monkeypatch):
    ctrl = controller(monkeypatch)
    ctrl._robot.client = None

    def connect():
        ctrl._running = False
        ctrl._stop.set()
        ctrl._action_stop.set()
        ctrl._robot.client = object()
        return True

    ctrl._robot.reconnect = connect
    assert not ctrl._execute_actions([SafeAction('forward', .3, .01)])
    assert 'forward' not in ctrl._robot.calls


def test_navigation_busy_never_connects_or_moves(monkeypatch):
    ctrl = controller(monkeypatch)
    ctrl._robot.client = None
    monkeypatch.setattr(speech, 'navigation_is_idle', lambda: False)
    assert not ctrl._execute_actions([SafeAction('forward', .3, .01)])
    assert ctrl._robot.calls == []
    assert any('导航' in s for s in ctrl.errors)
