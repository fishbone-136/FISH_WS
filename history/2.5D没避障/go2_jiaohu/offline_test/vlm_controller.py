"""D435i snapshot description and conversational questions via local Ollama."""

from __future__ import annotations

import base64
import logging
import re
import threading

import cv2

from voice import config
from voice.ollama_client import OllamaClient, OllamaError


logging.basicConfig(level=logging.INFO, format="%(asctime)s [Local VLM] %(message)s")

PERSONA = """你是机器狗邦邦的视觉助手。每轮图片都是当前相机画面。
用中文简洁回答用户问题，主要描述物体、人物数量、颜色和位置。
只有用户明确要求读字、识别文字或询问标牌内容时才读出清晰可辨的文字。
普通场景描述和其他追问不要附加文字识别段落，不要输出“画面中可以看到的文字是”等套话。
看不清、遮挡或无法确认时明确说明，不编造图片中没有的内容。
历史只用于理解追问；当前图片优先，不能把上一张图片的内容当作当前事实。
你只描述和讨论画面，不控制机器狗，不声称已经移动或执行任何动作。"""


class VLMController:
    """One image request at a time, with bounded text history and cancellation."""

    def __init__(self):
        self._ollama = OllamaClient()
        self.on_error = None
        self._task_lock = threading.Lock()
        self._task_thread: threading.Thread | None = None
        self._task_generation = 0
        self._history = []

    @property
    def is_ready(self) -> bool:
        # Availability is checked in the worker so device/model checks never
        # block the Qt thread. Missing models produce a visible error.
        return True

    @property
    def is_busy(self) -> bool:
        with self._task_lock:
            return self._task_thread is not None and self._task_thread.is_alive()

    def analyze_scene_async(self, frame_bgr, on_result):
        return self._start_task(
            frame_bgr,
            "请用两三句话简洁描述当前画面的主要物体、人物和位置。只描述场景，不识别或列举文字。",
            on_result,
            reset_history=True,
        )

    def query_scene_async(self, frame_bgr, user_text: str, on_result):
        return self._start_task(frame_bgr, user_text, on_result)

    def query_navigation_async(self, frame_bgr, user_text: str, on_result):
        """Compatibility alias for older callers; answers never execute motion."""
        return self.query_scene_async(frame_bgr, user_text, on_result)

    def _start_task(self, frame_bgr, prompt: str, on_result, *, reset_history=False) -> bool:
        """Start one visual task, rejecting duplicate concurrent requests."""
        with self._task_lock:
            if self._task_thread is not None and self._task_thread.is_alive():
                return False
            if reset_history:
                self._history.clear()
            self._task_generation += 1
            generation = self._task_generation
            thread = threading.Thread(
                target=self._run_task,
                args=(frame_bgr, prompt, on_result, generation, reset_history),
                daemon=True,
                name="bangbang-vlm",
            )
            self._task_thread = thread
            thread.start()
        return True

    def _run_task(self, frame_bgr, prompt: str, on_result, generation: int,
                  describe_only: bool = False) -> None:
        try:
            self._run(frame_bgr, prompt, on_result, generation, describe_only)
        finally:
            current = threading.current_thread()
            with self._task_lock:
                if self._task_thread is current:
                    self._task_thread = None

    def cancel_pending(self) -> None:
        """Invalidate callbacks from the current task when its mode is closed.

        OpenCV work is short.  A request already inside Ollama cannot be killed
        safely, so its eventual result is discarded and no second request is
        allowed until that worker exits.
        """
        with self._task_lock:
            self._task_generation += 1
            self._history.clear()

    def _is_current(self, generation: int) -> bool:
        with self._task_lock:
            return generation == self._task_generation

    def _deliver(self, generation: int, callback, text: str) -> None:
        if self._is_current(generation):
            callback(text)

    def _run(self, frame_bgr, prompt: str, on_result, generation: int,
             describe_only: bool = False) -> None:
        if frame_bgr is None:
            self._error("没有可用的当前彩色画面", generation)
            return
        if not self._is_current(generation):
            return
        try:
            # Bound image cost; 640x360 preserves the D435i preview's detail.
            height, width = frame_bgr.shape[:2]
            if max(height, width) > 960:
                scale = 960.0 / max(height, width)
                frame_bgr = cv2.resize(frame_bgr, (round(width * scale), round(height * scale)))
            ok, encoded = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ok:
                raise ValueError("画面编码失败")
            with self._task_lock:
                history = list(self._history)
            messages = [{"role": "system", "content": PERSONA}, *history,
                        {"role": "user", "content": prompt,
                         "images": [base64.b64encode(encoded).decode("ascii")]}]
            description = self._ollama.chat(
                messages, temperature=0.2, vision=True, max_tokens=256,
                model=config.OLLAMA_VISION_MODEL,
            )
            if describe_only:
                # Some small models still append an OCR template despite the
                # instruction. Keep it out of both the UI and follow-up history.
                description = re.split(
                    r"(?:画面|图像|图片)(?:中|里|上)?[^。！？\n]{0,16}?文字(?:内容)?(?:是|为|[：:])",
                    description, maxsplit=1,
                )[0].rstrip(" \n，,；;：:")
                if not description:
                    raise ValueError("未获得有效的场景描述，请重新识别")
            with self._task_lock:
                if generation != self._task_generation:
                    return
                self._history.extend([{"role": "user", "content": prompt},
                                      {"role": "assistant", "content": description}])
                self._history = self._history[-8:]
            self._deliver(generation, on_result, description)
        except (OllamaError, ValueError, cv2.error) as exc:
            self._error(f"当前画面识别失败：{exc}", generation)

    def _error(self, message: str, generation: int | None = None) -> None:
        logging.warning(message)
        if self.on_error and (generation is None or self._is_current(generation)):
            self.on_error(message)
