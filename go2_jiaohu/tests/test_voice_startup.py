import threading
import logging
from types import SimpleNamespace

from deploy.speech_controller import SpeechController as DeployController
from offline_test import speech_controller as speech
from voice import robot_interface


def test_listening_does_not_wait_for_qwen_warmup(monkeypatch):
    controller = speech.SpeechController()
    warming = threading.Event()
    release = threading.Event()
    listening = threading.Event()
    asr_ready = threading.Event()

    def warm_up():
        assert asr_ready.is_set()
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
    controller._asr = SimpleNamespace(start=asr_ready.set, runtime="fake")
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


def _deploy_warmup_controller():
    # Do not initialize TTS, cameras or a real robot in these scheduler tests.
    controller = DeployController.__new__(DeployController)
    speech.SpeechController.__init__(controller)
    return controller


def test_qwen_waits_for_tts_allocation_before_warming(monkeypatch):
    controller = _deploy_warmup_controller()
    waiting, tts_ready, warmed = (threading.Event() for _ in range(3))

    def wait_for_tts(timeout):
        waiting.set()
        return tts_ready.wait(timeout)

    controller._tts = SimpleNamespace(wait_until_neural_ready=wait_for_tts)
    monkeypatch.setattr(speech, "OllamaClient", lambda: SimpleNamespace(
        warm_up=lambda: warmed.set() or {}))
    worker = threading.Thread(target=controller._warm_up_llm)
    worker.start()
    try:
        assert waiting.wait(1)
        assert not warmed.is_set()
        tts_ready.set()
        assert warmed.wait(1)
    finally:
        tts_ready.set()
        controller._stop.set()
        worker.join(1)
    assert not worker.is_alive()


def test_stopping_cancels_dependency_wait_without_loading_qwen(monkeypatch):
    controller = _deploy_warmup_controller()
    waiting, warmed = threading.Event(), threading.Event()

    def wait_for_tts(timeout):
        waiting.set()
        controller._stop.wait(timeout)
        return False

    controller._tts = SimpleNamespace(wait_until_neural_ready=wait_for_tts)
    monkeypatch.setattr(speech, "OllamaClient", lambda: SimpleNamespace(
        warm_up=lambda: warmed.set() or {}))
    worker = threading.Thread(target=controller._warm_up_llm)
    worker.start()
    try:
        assert waiting.wait(1)
    finally:
        controller._stop.set()
        worker.join(1)
    assert not worker.is_alive()
    assert not warmed.is_set()


def test_tts_dependency_timeout_still_allows_warmup(monkeypatch, caplog):
    controller = _deploy_warmup_controller()
    warmed = []
    controller._tts = SimpleNamespace(wait_until_neural_ready=lambda timeout: False)
    monkeypatch.setattr(speech.config, "OLLAMA_WARMUP_DEPENDENCY_TIMEOUT", 0)
    monkeypatch.setattr(speech, "OllamaClient", lambda: SimpleNamespace(
        warm_up=lambda: warmed.append(True) or {}))
    with caplog.at_level(logging.WARNING):
        controller._warm_up_llm()
    assert warmed == [True]
    assert "等待 TTS 初始化超时" in caplog.text
