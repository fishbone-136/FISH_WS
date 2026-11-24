import json
import subprocess
import wave
from contextlib import nullcontext
from types import SimpleNamespace

import numpy as np
import pytest

from voice import config, qwen_asr_worker as worker
from voice.local_asr import LocalAsrError, LocalQwenASR, LocalWhisperASR, create_local_asr


def _wav(path, samples, rate=16000):
    with wave.open(str(path), "wb") as stream:
        stream.setnchannels(1)
        stream.setsampwidth(2)
        stream.setframerate(rate)
        stream.writeframes(np.array(samples, dtype="<i2").tobytes())


def test_factory_selects_isolated_qwen_and_explicit_whisper(monkeypatch):
    monkeypatch.setattr(config, "ASR_BACKEND", "qwen")
    assert type(create_local_asr()) is LocalQwenASR
    monkeypatch.setattr(config, "ASR_BACKEND", "whisper")
    assert type(create_local_asr()) is LocalWhisperASR
    monkeypatch.setattr(config, "ASR_BACKEND", "typo")
    with pytest.raises(LocalAsrError, match="GO2_ASR_BACKEND"):
        create_local_asr()


def test_qwen_launch_uses_own_python_and_child_only_library_paths(monkeypatch, tmp_path):
    import sys

    private = tmp_path / ".cuda-libs"
    private.mkdir()
    monkeypatch.setattr(config, "QWEN_ASR_ROOT", str(tmp_path))
    monkeypatch.setattr(config, "QWEN_ASR_PYTHON", sys.executable)
    monkeypatch.setenv("PYTHONPATH", "/unwanted/ros/packages")
    monkeypatch.setenv("PYTHONHOME", "/unwanted/python")
    monkeypatch.setenv("LD_LIBRARY_PATH", "/existing/libraries")
    bridge = LocalQwenASR()
    command = bridge._worker_command()
    env = bridge._worker_environment()
    assert command[:4] == [sys.executable, "-I", "-B", "-u"]
    assert command[4].endswith("/voice/qwen_asr_worker.py")
    assert env["LD_LIBRARY_PATH"] == f"{private}:/existing/libraries"
    assert "PYTHONPATH" not in env and "PYTHONHOME" not in env
    assert env["HF_HUB_OFFLINE"] == env["TRANSFORMERS_OFFLINE"] == "1"
    import os
    assert os.environ["PYTHONPATH"] == "/unwanted/ros/packages"
    assert os.environ["LD_LIBRARY_PATH"] == "/existing/libraries"


def test_missing_qwen_environment_reports_error_without_whisper_fallback(monkeypatch, tmp_path):
    monkeypatch.setattr(config, "QWEN_ASR_PYTHON", str(tmp_path / "missing-python"))
    bridge = LocalQwenASR()
    with pytest.raises(LocalAsrError, match="Qwen 独立解释器"):
        bridge.start()
    assert not bridge.ready


def test_worker_serves_multiple_requests_and_keeps_diagnostics_out_of_protocol(tmp_path, capsys):
    wav = tmp_path / "speech.wav"
    _wav(wav, [100, -100] * 800)
    calls = []

    def transcribe(**kwargs):
        calls.append(kwargs)
        print("model diagnostic")
        return [SimpleNamespace(text="  向前走一米。  ", language="Chinese")]

    requests = [json.dumps({"wav_path": str(wav)}) + "\n"] * 2
    requests += ['{"command":"shutdown"}\n', '{"wav_path":"must-not-be-read"}\n']
    worker.serve(SimpleNamespace(transcribe=transcribe),
                 SimpleNamespace(inference_mode=nullcontext), "Chinese", requests)
    captured = capsys.readouterr()
    messages = [json.loads(line) for line in captured.out.splitlines()]
    assert [msg["status"] for msg in messages] == ["progress", "ok", "progress", "ok"]
    assert messages[1]["text"] == "向前走一米。"
    assert messages[1]["language"] == "zh"
    assert "no_speech_prob" not in messages[1]  # Qwen does not supply Whisper confidence.
    assert captured.err.count("model diagnostic") == 2
    assert len(calls) == 2
    assert calls[0]["audio"][1] == 16000
    assert calls[0]["audio"][0].dtype == np.float32


@pytest.mark.parametrize("samples", [[], [0] * 1600])
def test_empty_or_silent_audio_does_not_run_decoder(tmp_path, capsys, samples):
    wav = tmp_path / "silence.wav"
    _wav(wav, samples)
    worker.serve(object(), object(), None, [json.dumps({"wav_path": str(wav)})])
    message = json.loads(capsys.readouterr().out)
    assert message["status"] == "ok" and message["text"] == ""
    assert message["rejection_reason"] == "silent_audio"


def test_bad_audio_returns_protocol_error_and_next_request_can_shutdown(tmp_path, capsys):
    wav = tmp_path / "wrong-rate.wav"
    _wav(wav, [100] * 800, rate=8000)
    worker.serve(object(), object(), None,
                 [json.dumps({"wav_path": str(wav)}), '{"command":"shutdown"}'])
    message = json.loads(capsys.readouterr().out)
    assert message["status"] == "error"
    assert "16kHz" in message["error"]


@pytest.mark.parametrize("device,should_raise", [("auto", False), ("cuda:0", True)])
def test_gpu_probe_failure_only_falls_back_in_automatic_mode(monkeypatch, tmp_path, device, should_raise):
    torch = SimpleNamespace(cuda=SimpleNamespace(is_available=lambda: True),
                            device=lambda name: SimpleNamespace(type=name.split(":")[0], name=name))
    monkeypatch.setattr(subprocess, "run", lambda *a, **kw: SimpleNamespace(
        returncode=1, stdout='{"ok":false,"error":"kernel unavailable"}'))
    if should_raise:
        with pytest.raises(RuntimeError, match="kernel unavailable"):
            worker._select_device(torch, device, tmp_path)
    else:
        assert worker._select_device(torch, device, tmp_path).type == "cpu"
