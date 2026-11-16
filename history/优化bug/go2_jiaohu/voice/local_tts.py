"""Persistent bridge to the isolated Kokoro CUDA TTS worker."""

from __future__ import annotations

import json
import logging
import os
import select
import subprocess
import tempfile
import threading
import time

from . import config


class LocalTtsError(RuntimeError):
    pass


class LocalNeuralTTS:
    def __init__(self):
        self._process: subprocess.Popen | None = None
        self._lock = threading.RLock()
        self._startup_thread: threading.Thread | None = None
        self._startup_error = ""
        self.runtime = ""

    @property
    def configured(self) -> bool:
        required = (
            os.path.join(config.TTS_MODEL_DIR, "config.json"),
            os.path.join(config.TTS_MODEL_DIR, "kokoro-v1_0.pth"),
            os.path.join(config.TTS_MODEL_DIR, "voices", f"{config.TTS_VOICE}.pt"),
        )
        return bool(config.TTS_PYTHON and all(os.path.isfile(path) for path in required))

    @property
    def ready(self) -> bool:
        return self._process is not None and self._process.poll() is None and bool(self.runtime)

    def start_async(self) -> None:
        if not self.configured or self.ready:
            return
        if self._startup_thread and self._startup_thread.is_alive():
            return
        self._startup_thread = threading.Thread(
            target=self._start_in_background,
            daemon=True,
            name="bangbang-tts-loader",
        )
        self._startup_thread.start()

    def wait_until_ready(self, timeout: float) -> bool:
        """等待后台权重加载结束，供 Qwen 在最后一次 GPU 分配之后再预热。"""
        thread = self._startup_thread
        if thread and thread.is_alive():
            thread.join(timeout=max(0.0, timeout))
        return self.ready

    def _start_in_background(self) -> None:
        try:
            self.start()
            logging.info("本地 Kokoro CUDA 温柔男声已就绪: %s", self.runtime)
        except LocalTtsError as exc:
            self._startup_error = str(exc)
            logging.warning("Kokoro CUDA TTS 初始化失败，将使用系统语音: %s", exc)

    def start(self) -> None:
        with self._lock:
            if self.ready:
                return
            self._startup_error = ""
            project_dir = config.PROJECT_DIR
            third_party = os.path.join(project_dir, "third_party", "kokoro-python")
            cuda_sysroot = os.path.join(project_dir, "third_party", "cuda-sysroot")
            cuda_root = os.path.join(cuda_sysroot, "usr", "local", "cuda-12.6")
            python_version = "python3.10"
            user_torch_lib = os.path.expanduser(
                f"~/.local/lib/{python_version}/site-packages/torch/lib"
            )
            library_paths = [
                os.path.join(third_party, "torch", "lib"),
                os.path.join(cuda_sysroot, "usr", "lib", "aarch64-linux-gnu"),
                os.path.join(cuda_root, "lib64"),
                user_torch_lib,
            ]
            env = os.environ.copy()
            # run_ui intentionally hides user packages from ROS. This isolated child needs the
            # local Kokoro/Misaki installation, while PYTHONPATH selects Jetson torch first.
            env.pop("PYTHONNOUSERSITE", None)
            env["PYTHONPATH"] = os.pathsep.join(
                [third_party, project_dir] + ([env["PYTHONPATH"]] if env.get("PYTHONPATH") else [])
            )
            env["LD_LIBRARY_PATH"] = os.pathsep.join(
                [path for path in library_paths if os.path.isdir(path)]
                + ([env["LD_LIBRARY_PATH"]] if env.get("LD_LIBRARY_PATH") else [])
            )
            env["BANGBANG_TTS_MODEL_DIR"] = config.TTS_MODEL_DIR
            env["BANGBANG_TTS_VOICE"] = config.TTS_VOICE
            env["HF_HUB_OFFLINE"] = "1"
            env["TRANSFORMERS_OFFLINE"] = "1"
            env["TOKENIZERS_PARALLELISM"] = "false"
            try:
                self._process = subprocess.Popen(
                    [config.TTS_PYTHON, "-u", "-m", "voice.tts_worker"],
                    cwd=project_dir,
                    env=env,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    text=True,
                    bufsize=1,
                )
                response = self._read_response(config.TTS_STARTUP_TIMEOUT)
            except Exception as exc:
                self.close(force=True)
                raise LocalTtsError(str(exc)) from exc
            if response.get("status") != "ready":
                error = response.get("error", "Kokoro TTS worker failed to start")
                self.close(force=True)
                raise LocalTtsError(str(error))
            self.runtime = (
                f"{response.get('device')} / {response.get('voice')} / "
                f"{response.get('load_seconds')}s"
            )

    def synthesize(self, text: str) -> tuple[str, dict]:
        if not self.ready:
            thread = self._startup_thread
            if thread and thread.is_alive():
                thread.join(timeout=config.TTS_STARTUP_TIMEOUT)
            if not self.ready:
                self.start()
        wav_path = ""
        try:
            with tempfile.NamedTemporaryFile(
                prefix="bangbang-tts-", suffix=".wav", delete=False
            ) as stream:
                wav_path = stream.name
            with self._lock:
                self._write({
                    "text": text,
                    "output_path": wav_path,
                    "speed": config.TTS_SPEED,
                    "volume": config.TTS_VOLUME,
                })
                response = self._read_response(config.TTS_REQUEST_TIMEOUT)
            if response.get("status") != "ok":
                raise LocalTtsError(str(response.get("error", "Kokoro synthesis failed")))
            return wav_path, response
        except Exception:
            if wav_path:
                try:
                    os.unlink(wav_path)
                except OSError:
                    pass
            raise

    def _write(self, payload: dict) -> None:
        if not self.ready or self._process is None or self._process.stdin is None:
            raise LocalTtsError("Kokoro TTS worker is unavailable")
        try:
            self._process.stdin.write(json.dumps(payload, ensure_ascii=False) + "\n")
            self._process.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            raise LocalTtsError(f"Kokoro TTS connection interrupted: {exc}") from exc

    def _read_response(self, timeout: float) -> dict:
        if self._process is None or self._process.stdout is None:
            raise LocalTtsError("Kokoro TTS worker is not running")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._process.poll() is not None:
                raise LocalTtsError(
                    f"Kokoro TTS worker exited: {self._process.returncode}"
                )
            readable, _, _ = select.select([self._process.stdout], [], [], 0.25)
            if not readable:
                continue
            line = self._process.stdout.readline().strip()
            if not line:
                continue
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                continue
        raise LocalTtsError("Kokoro TTS timed out")

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
        self.runtime = ""

    def __del__(self):
        self.close(force=True)
