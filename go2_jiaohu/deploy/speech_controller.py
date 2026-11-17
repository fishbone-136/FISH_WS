"""邦邦实机语音控制器：本地 TTS、Go2 动作与 D435i 视觉引导。"""

from __future__ import annotations

import logging
import os
import subprocess
import threading
import time

from offline_test.speech_controller import SpeechController as _LocalSpeechController
from voice import config
from voice.audio_processor import AudioSystem
from voice.command_safety import MOTION_ACTIONS, SafeAction
from voice.robot_interface import Go2RobotController
from voice.visual_guidance import VisualGuidanceController
from deploy.text_motion_controller import navigation_is_idle
from voice.text_motion import describe_action


logging.basicConfig(level=logging.INFO, format="%(asctime)s [Deploy Speech] %(message)s")
logging.info("语音动作优化已加载：严格连续动作直解析、执行期间保持急停监听（20261111）")
logging.info("自然语音动作已加载：站立同义理解，无需固定执行口令（20261111）")


class SpeechController(_LocalSpeechController):
    """经安全门控后异步执行动作，保持麦克风可随时接收急停或报警。"""

    def __init__(self, frame_provider=None, depth_provider=None):
        super().__init__(frame_provider=frame_provider, depth_provider=depth_provider)
        self._tts = AudioSystem()
        # Ordinary conversation should not wait for SDK discovery or robot ACKs.
        self._robot = Go2RobotController(auto_connect=False)
        self._action_stop = threading.Event()
        self._action_thread: threading.Thread | None = None
        self._guidance = VisualGuidanceController(
            self._robot,
            frame_provider=frame_provider,
            depth_provider=depth_provider,
            on_status=self._guidance_status,
        )
        logging.info("邦邦语音控制器（实机版）初始化完成：严格连续动作直解析、执行期间保持急停监听（20261111）")

    @property
    def is_action_busy(self) -> bool:
        return bool((self._action_thread and self._action_thread.is_alive())
                    or self._guidance.is_running)

    def stop(self) -> None:
        self._running = False
        self._stop.set()
        self._cancel_actions()
        super().stop()

    def shutdown(self) -> None:
        self.stop()
        self._tts.close()
        self._robot.shutdown()

    def _wait_for_warmup_dependencies(self) -> bool:
        # Kokoro 的 CUDA 初始化也会占用内存。等它加载完成后再预热 Qwen，
        # 避免刚预热的 runner 因后续 GPU 分配被卸载。仅阻塞后台预热线程。
        started = time.monotonic()
        deadline = started + config.OLLAMA_WARMUP_DEPENDENCY_TIMEOUT
        while not self._stop.is_set():
            remaining = max(0.0, deadline - time.monotonic())
            if self._tts.wait_until_neural_ready(min(0.1, remaining)):
                logging.info("Qwen 预热依赖已就绪：等待 TTS %.3fs", time.monotonic() - started)
                return not self._stop.is_set()
            if remaining <= 0:
                logging.warning("等待 TTS 初始化超时，继续后台预热 Qwen，首轮仍可能发生模型加载")
                return not self._stop.is_set()
            # 初始化失败时 wait_until_neural_ready 会立即返回，避免忙循环。
            if self._stop.wait(min(0.1, max(0.0, deadline - time.monotonic()))):
                return False
        return False

    def _speak(self, text: str) -> None:
        self._emit_state("speaking")
        # 半双工防回声：播报期间将精确配置的 PulseAudio 输入源静音。否则 USB
        # 扬声器声音会被同一块 UAC 麦克风收回，形成“邦邦和自己对话”的循环。
        source_muted = self._set_microphone_muted(True)
        try:
            self._tts.play_text_to_speech(text)
        finally:
            time.sleep(max(0.0, config.TTS_ECHO_COOLDOWN_SECONDS))
            # 当前通常就是持续监听线程，此时可以安全丢弃播报期间积压的零数据。
            if threading.current_thread() is self._thread:
                self._reset_microphone_stream()
            if source_muted:
                self._set_microphone_muted(False)
            if self._running:
                self._emit_state("listening")

    def _set_microphone_muted(self, muted: bool) -> bool:
        source_name = os.environ.get("PULSE_SOURCE", "").strip()
        if not source_name:
            return False
        try:
            result = subprocess.run(
                ["pactl", "set-source-mute", source_name, "1" if muted else "0"],
                check=False,
                timeout=2.0,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            return result.returncode == 0
        except (OSError, subprocess.SubprocessError):
            return False

    def _reset_microphone_stream(self) -> None:
        stream_wrapper = getattr(self._mic, "stream", None)
        stream = getattr(stream_wrapper, "pyaudio_stream", None)
        if stream is None:
            return
        try:
            if not stream.is_stopped():
                stream.stop_stream()
            stream.start_stream()
        except Exception as exc:
            logging.warning("播报后清理麦克风缓冲失败: %s", exc)

    def _execute_actions(self, actions: list[SafeAction]) -> bool:
        if not actions:
            return False
        if self._stop.is_set() or not self._running:
            logging.info("语音模式已关闭，忽略延迟到达的动作")
            return False
        ready = self._cancel_actions()
        if actions[0].action == "stop":
            return self._robot.execute_action("stop")
        if not ready:
            self._emit_error("上一条动作仍在停止，未下发新动作，请稍后重试。")
            return False
        # Each sequence owns its cancellation event. Never clear an event that
        # an older worker may still be waiting on inside a slow SDK call.
        self._action_stop = threading.Event()
        cancel = self._action_stop
        if not navigation_is_idle():
            self._emit_error("导航仍在运行，请关闭导航后使用语音动作。")
            return False

        first = actions[0]
        if first.action == "visual_guide":
            return self._guidance.start(duration=first.duration, speed=first.speed)
        if self._robot.client is None and not self._robot.reconnect():
            self._emit_error(
                f"{self._robot.backend} 运动桥未连接，已拒绝执行实机动作："
                f"{self._robot.last_error or '无连接'}"
            )
            return False
        if cancel.is_set() or self._stop.is_set() or not self._running:
            return False
        if not navigation_is_idle():
            self._emit_error("检测到导航启动，已拒绝语音动作。")
            return False
        # 第一条动作在当前调用线程同步等待 SDK/ROS 回执。这样 UI 不会再把
        # “写入子进程 stdin”误报为已执行，动作也不会被 TTS 阻塞。
        if not self._robot.execute_action(
            first.action,
            speed=first.speed,
            duration=first.duration,
            angle=first.angle,
        ):
            self._emit_error(
                f"动作 {first.action} 未被 {self._robot.backend} 接受："
                f"{self._robot.last_error or '无回执'}"
            )
            return False
        if cancel.is_set() or self._stop.is_set() or not self._running:
            self._robot.execute_action("stop")
            return False
        self._emit_status(f"已下发第 1/{len(actions)} 个动作：{describe_action(first)}。")
        self._action_thread = threading.Thread(
            target=self._run_actions,
            args=(list(actions), True, cancel),
            daemon=True,
            name="bangbang-action-sequence",
        )
        self._action_thread.start()
        return True

    def _run_actions(self, actions: list[SafeAction], first_sent: bool = False,
                     cancel: threading.Event | None = None) -> None:
        cancel = cancel if cancel is not None else self._action_stop
        moved = False
        failed = False
        try:
            for index, action in enumerate(actions):
                if cancel.is_set():
                    break
                if not navigation_is_idle():
                    raise RuntimeError("检测到导航控制源启动，已中止语音动作。")
                if action.action == "visual_guide":
                    self._guidance.start(duration=action.duration, speed=action.speed)
                    return
                if self._robot.client is None:
                    raise RuntimeError(f"{self._robot.backend} 运动桥未连接，已拒绝执行实机动作")
                if not (first_sent and index == 0):
                    sent = self._robot.execute_action(
                        action.action,
                        speed=action.speed,
                        duration=action.duration,
                        angle=action.angle,
                    )
                    if not sent:
                        self._emit_error(
                            f"动作 {action.action} 下发失败：{self._robot.last_error or '无回执'}"
                        )
                        failed = True
                        return
                    self._emit_status(f"已下发第 {index+1}/{len(actions)} 个动作：{describe_action(action)}。")
                if action.action in MOTION_ACTIONS:
                    moved = True
                    deadline = time.monotonic() + action.duration + 0.1
                    while time.monotonic() < deadline:
                        if cancel.wait(min(.1, max(0., deadline-time.monotonic()))):
                            break
                        if not navigation_is_idle():
                            raise RuntimeError("检测到导航控制源启动，已中止语音动作。")
                elif action.duration > 0:
                    cancel.wait(action.duration)
        except Exception as exc:
            failed = True
            self._emit_error(f"语音动作中止：{exc}")
        finally:
            # first_sent may have moved even when cancellation arrived before
            # the worker entered its first iteration.
            if (moved or first_sent) and not self._guidance.is_running:
                if not self._robot.execute_action("stop"):
                    failed = True
                    self._emit_error("停止未收到运动接口确认，请使用遥控器急停。")
            if not failed:
                self._emit_status("语音动作已取消，已请求停止。" if cancel.is_set()
                                  else "语音动作定时控制已结束，已收到停止回执。")

    def _cancel_actions(self) -> bool:
        self._action_stop.set()
        if self._guidance.is_running:
            self._guidance.stop()
        # Request zero velocity before joining an SDK worker: stopping must
        # not wait for the previous action's acknowledgement timeout first.
        if self._robot.client is not None:
            self._robot.execute_action("stop")
        thread = self._action_thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.5)
        ready = thread is None or not thread.is_alive()
        if ready:
            self._action_thread = None
        return ready

    def _guidance_status(self, message: str) -> None:
        if self.on_response:
            self.on_response(f"视觉引导：{message}")
