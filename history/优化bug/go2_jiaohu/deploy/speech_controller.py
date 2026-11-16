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


logging.basicConfig(level=logging.INFO, format="%(asctime)s [Deploy Speech] %(message)s")


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
        logging.info("邦邦语音控制器（实机版）初始化完成")

    def stop(self) -> None:
        super().stop()
        self._cancel_actions()

    def shutdown(self) -> None:
        self.stop()
        self._tts.close()
        self._robot.shutdown()

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
        self._cancel_actions()
        if actions[0].action == "stop":
            return self._robot.execute_action("stop")
        self._action_stop.clear()

        first = actions[0]
        if first.action == "visual_guide":
            return self._guidance.start(duration=first.duration, speed=first.speed)
        if self._robot.client is None and not self._robot.reconnect():
            self._emit_error(
                f"{self._robot.backend} 运动桥未连接，已拒绝执行实机动作："
                f"{self._robot.last_error or '无连接'}"
            )
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
        self._action_thread = threading.Thread(
            target=self._run_actions,
            args=(list(actions), True),
            daemon=True,
            name="bangbang-action-sequence",
        )
        self._action_thread.start()
        return True

    def _run_actions(self, actions: list[SafeAction], first_sent: bool = False) -> None:
        moved = False
        try:
            for index, action in enumerate(actions):
                if self._action_stop.is_set():
                    break
                if action.action == "visual_guide":
                    self._guidance.start(duration=action.duration, speed=action.speed)
                    return
                if self._robot.client is None:
                    self._emit_error(
                        f"{self._robot.backend} 运动桥未连接，已拒绝执行实机动作"
                    )
                    return
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
                        return
                if action.action in MOTION_ACTIONS:
                    moved = True
                    self._action_stop.wait(action.duration + 0.1)
                elif action.duration > 0:
                    self._action_stop.wait(action.duration)
        finally:
            if moved and not self._guidance.is_running:
                self._robot.execute_action("stop")

    def _cancel_actions(self) -> None:
        self._action_stop.set()
        if self._guidance.is_running:
            self._guidance.stop()
        thread = self._action_thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.5)
        self._action_thread = None
        if self._robot.client is not None:
            self._robot.execute_action("stop")

    def _guidance_status(self, message: str) -> None:
        if self.on_response:
            self.on_response(f"视觉引导：{message}")
