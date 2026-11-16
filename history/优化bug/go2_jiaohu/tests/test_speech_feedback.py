from types import SimpleNamespace
import struct
import logging

from offline_test import speech_controller as speech


def test_calibration_never_places_gate_below_measured_noise(monkeypatch):
    monkeypatch.setattr(speech.config, "MICROPHONE_MIN_ENERGY", 200)
    monkeypatch.setattr(speech.config, "MICROPHONE_MAX_ENERGY", 450)
    assert speech._microphone_energy_threshold(3000) == 3000
    assert speech._microphone_energy_threshold(80) == 200


def test_noise_estimator_requires_long_stationary_audio():
    audio = speech.sr.AudioData(struct.pack('<h', 2000) * 48000, 16000, 2)
    assert speech._steady_noise_rms(audio) == 2000
    short = speech.sr.AudioData(struct.pack('<h', 2000) * 8000, 16000, 2)
    assert speech._steady_noise_rms(short) is None
    varying = speech.sr.AudioData((b'\0\0' * 800 + struct.pack('<h', 2000) * 800) * 40, 16000, 2)
    assert speech._steady_noise_rms(varying) is None


def test_filtered_audio_restores_listening_without_fake_robot_reply(monkeypatch):
    controller = speech.SpeechController()
    audio = speech.sr.AudioData(struct.pack('<h', 2000) * 48000, 16000, 2)
    states, statuses, replies = [], [], []
    controller.on_state = states.append
    controller.on_status = statuses.append
    controller.on_response = replies.append

    class Microphone:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

    calls = []
    def listen(*args, **kwargs):
        if not calls:
            calls.append(True)
            return audio
        controller._running = False
        raise speech.sr.WaitTimeoutError()

    controller._mic = Microphone()
    controller._recog = SimpleNamespace(adjust_for_ambient_noise=lambda *a, **kw: None,
                                       energy_threshold=450, listen=listen)
    controller._asr = SimpleNamespace(start=lambda: None, runtime="fake",
                                    transcribe_audio=lambda audio: ("", "zh"),
                                    last_result={"total_seconds": 2.1, "no_speech_prob": 1.0})
    monkeypatch.setattr(controller, "_open_microphone", lambda: True)
    monkeypatch.setattr(controller, "_start_background_warmup", lambda: None)
    controller._running = True
    controller._loop()
    assert any("未识别到清晰语音（2.1秒），继续监听" == value for value in statuses)
    assert replies == ["语音识别已就绪，请开始说话。"]
    assert states.index("listening", states.index("thinking") + 1) >= 0
    assert controller._recog.energy_threshold == 3000


def test_rejected_speech_does_not_raise_microphone_gate():
    controller = speech.SpeechController()
    controller._asr.last_result = {"no_speech_prob": 0.2}
    controller._recog.energy_threshold = 450
    audio = speech.sr.AudioData(struct.pack('<h', 2000) * 48000, 16000, 2)
    controller._adjust_rejected_noise(audio)
    assert controller._recog.energy_threshold == 450


def test_runtime_log_persists_terminal_and_ui_messages(tmp_path, monkeypatch):
    from go2_ui.runtime_logging import configure_logging

    root = logging.getLogger()
    monkeypatch.setattr(root, "handlers", [])
    monkeypatch.setattr(root, "level", root.level)
    path = tmp_path / "ui_runtime.log"
    try:
        configure_logging(path)
        logging.info("ASR 完成：主模型=2.6s")
        logging.getLogger("go2.ui").info("语音状态：继续监听")
        for handler in root.handlers:
            handler.flush()
        text = path.read_text()
        assert "ASR 完成：主模型=2.6s" in text
        assert "语音状态：继续监听" in text
        assert "[INFO]" in text
    finally:
        for handler in root.handlers:
            handler.close()
