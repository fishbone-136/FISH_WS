"""面向中英文连续语音的本地 faster-whisper 进程桥。"""

from __future__ import annotations

import json
import os
import select
import subprocess
import sys
import tempfile
import threading
import time


class LocalAsrError(RuntimeError):
    pass


class LocalWhisperASR:
    """将 ASR 隔离到子进程，避免 CTranslate2 与 ROS/DDS native 库冲突。"""

    def __init__(self, startup_timeout: float = 600.0, request_timeout: float = 90.0):
        self.startup_timeout = startup_timeout
        self.request_timeout = request_timeout
        self._process: subprocess.Popen | None = None
        self._lock = threading.Lock()
        self.runtime = ""
        self.last_result: dict = {}
        self.on_progress = None
        self._read_buffer = b""

    @property
    def ready(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def start(self) -> None:
        if self.ready:
            return
        project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self._process = subprocess.Popen(
            [sys.executable, "-u", "-m", "voice.asr_worker"],
            cwd=project_dir,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=None,
            bufsize=0,
        )
        self._read_buffer = b""
        try:
            response = self._read_response(self.startup_timeout)
        except Exception:
            self.close(force=True)
            raise
        if response.get("status") != "ready":
            error = response.get("error", "ASR 子进程启动失败")
            self.close(force=True)
            raise LocalAsrError(str(error))
        self.runtime = (
            f"{response.get('model')} / {response.get('device')} / "
            f"{response.get('compute_type')}"
        )
        if response.get("rescue_model"):
            self.runtime += (
                f" / 复核模型已预载 {response.get('rescue_preload_seconds', 0)}s"
            )

    def transcribe_audio(self, audio_data) -> tuple[str, str]:
        """转写 SpeechRecognition AudioData，返回 (文本, 检测语言)。"""
        if not self.ready:
            self.start()
        wav_path = ""
        try:
            with tempfile.NamedTemporaryFile(prefix="bangbang-asr-", suffix=".wav", delete=False) as wav:
                wav.write(audio_data.get_wav_data(convert_rate=16000, convert_width=2))
                wav_path = wav.name
            with self._lock:
                self._write({"wav_path": wav_path})
                response = self._read_response(self.request_timeout)
            if response.get("status") != "ok":
                raise LocalAsrError(str(response.get("error", "本地语音识别失败")))
            self.last_result = response
            return str(response.get("text", "")).strip(), str(response.get("language", ""))
        except LocalAsrError:
            # A timed-out worker may still emit its old result. Restart it
            # before the next request instead of attributing that result to it.
            self.close(force=True)
            raise
        finally:
            if wav_path:
                try:
                    os.unlink(wav_path)
                except OSError:
                    pass

    def _write(self, payload: dict) -> None:
        if not self.ready or self._process is None or self._process.stdin is None:
            raise LocalAsrError("ASR 子进程不可用")
        try:
            self._process.stdin.write((json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8"))
            self._process.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            raise LocalAsrError(f"ASR 子进程连接中断: {exc}") from exc

    def _read_response(self, timeout: float) -> dict:
        process = self._process
        if process is None or process.stdout is None:
            raise LocalAsrError("ASR 子进程未启动")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._process is not process:
                raise LocalAsrError("ASR 已停止")
            # Drain buffered lines before select: progress and result can arrive
            # in the same read. TextIOWrapper.readline can otherwise hide the
            # result in its buffer while select waits on an empty OS pipe.
            while b"\n" in self._read_buffer:
                line, self._read_buffer = self._read_buffer.split(b"\n", 1)
                try:
                    response = json.loads(line)
                except (json.JSONDecodeError, UnicodeDecodeError):
                    continue
                if response.get("status") == "progress":
                    if self.on_progress:
                        self.on_progress(response)
                    continue
                return response
            try:
                readable, _, _ = select.select([process.stdout], [], [], min(0.25, max(0, deadline - time.monotonic())))
                if not readable:
                    if process.poll() is not None:
                        raise LocalAsrError(f"ASR 子进程异常退出: {process.returncode}")
                    continue
                chunk = os.read(process.stdout.fileno(), 65536)
            except (OSError, ValueError) as exc:
                raise LocalAsrError(f"ASR 连接已关闭：{exc}") from exc
            if not chunk:
                raise LocalAsrError("ASR 子进程输出已关闭")
            self._read_buffer += chunk
        raise LocalAsrError("本地语音识别超时")

    def close(self, force: bool = False) -> None:
        process = self._process
        if process is None:
            return
        if process.poll() is None and not force:
            try:
                self._write({"command": "shutdown"})
                process.wait(timeout=3.0)
            except Exception:
                force = True
        if process.poll() is None and force:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
        self._process = None

    def __del__(self):
        self.close(force=True)
