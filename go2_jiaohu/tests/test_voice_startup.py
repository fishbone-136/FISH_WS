import threading
from types import SimpleNamespace

from offline_test import speech_controller as speech
from voice import robot_interface


def test_listening_does_not_wait_for_qwen_warmup(monkeypatch):
    controller = speech.SpeechController()
    warming = threading.Event()
    release = threading.Event()
    listening = threading.Event()

    def warm_up():
        warming.set()
        release.wait(2)
        return {"wall_seconds": 0.0}

    class Microphone:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

    def listen(*args, **kwargs):
        assert warming.wait(1)
        listening.set()
        controller._running = False
        raise speech.sr.WaitTimeoutError()

    monkeypatch.setattr(speech.config, "OLLAMA_WARMUP", True)
    monkeypatch.setattr(speech, "OllamaClient", lambda: SimpleNamespace(warm_up=warm_up))
    controller._mic = Microphone()
    controller._recog = SimpleNamespace(adjust_for_ambient_noise=lambda *a, **kw: None,
                                       energy_threshold=300, listen=listen)
    controller._asr = SimpleNamespace(start=lambda: None, runtime="fake")
    monkeypatch.setattr(controller, "_open_microphone", lambda: True)
    controller._running = True
    try:
        controller._loop()
        assert listening.is_set()
        assert not release.is_set()
    finally:
        release.set()
        controller._warmup_thread.join(1)


def test_start_returns_while_microphone_discovery_is_pending(monkeypatch):
    controller = speech.SpeechController()
    entered = threading.Event()
    release = threading.Event()

    def open_microphone():
        entered.set()
        release.wait(2)
        return False

    monkeypatch.setattr(controller, "_open_microphone", open_microphone)
    try:
        assert controller.start() is True
        assert entered.wait(1)
        assert controller.is_running
        assert not release.is_set()
    finally:
        release.set()
        controller._thread.join(1)
        controller.stop()


def test_robot_connection_is_deferred_until_action(monkeypatch):
    events = []
    bridge = SimpleNamespace(ready=True, error="", send_action=lambda *a: True)
    monkeypatch.setattr(robot_interface, "_get_bridge", lambda interface: events.append(interface) or bridge)
    robot = robot_interface.Go2RobotController(auto_connect=False)
    assert events == [] and robot.client is None
    assert robot.execute_action("stop") is True
    assert events == [robot.interface]
