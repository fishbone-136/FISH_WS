"""邦邦语音控制器：本地 Whisper ASR + 本地 Ollama/Qwen。"""

from __future__ import annotations

import logging
import audioop
import re
import threading
import time
from typing import Callable

try:
    import speech_recognition as sr
    HAS_SR = True
except ImportError:
    sr = None
    HAS_SR = False

from voice import config
from voice.command_safety import (
    SafeAction,
    gate_command,
    is_fixed_action_command,
    is_alarm_request,
    is_emergency_stop,
    parse_deterministic_command,
    validate_actions,
)
from voice.emergency_alarm import EmergencyAlarm
from voice.llm_processor import OllamaInstructionParser
from voice.local_asr import LocalAsrError, LocalWhisperASR
from voice.ollama_client import OllamaClient, OllamaError


logging.basicConfig(level=logging.INFO, format="%(asctime)s [BangBang Speech] %(message)s")


class SpeechController:
    """麦克风 → 本地中英文 ASR → 安全门控 → Qwen 对话/动作语义。"""

    def __init__(self, frame_provider: Callable | None = None,
                 depth_provider: Callable | None = None):
        self._running = False
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._warmup_thread: threading.Thread | None = None
        self._lifecycle_lock = threading.Lock()
        self._mic = None
        self._mic_device_name = ""
        self._recog = sr.Recognizer() if HAS_SR else None
        if self._recog is not None:
            self._recog.dynamic_energy_threshold = True
            self._recog.pause_threshold = max(0.2, config.MICROPHONE_PAUSE_THRESHOLD)
            self._recog.non_speaking_duration = min(
                max(0.1, config.MICROPHONE_NON_SPEAKING_DURATION),
                self._recog.pause_threshold,
            )
            self._recog.phrase_threshold = max(
                0.2, config.MICROPHONE_PHRASE_THRESHOLD
            )
        self._asr = LocalWhisperASR()
        self._asr.on_progress = self._on_asr_progress
        self._llm = OllamaInstructionParser()
        self._alarm = EmergencyAlarm()
        self.frame_provider = frame_provider
        self.depth_provider = depth_provider

        # 回调从后台线程发出，UI 层负责封送到 Qt 主线程。
        self.on_state: Callable[[str], None] | None = None
        self.on_asr: Callable[[str], None] | None = None
        self.on_response: Callable[[str], None] | None = None
        self.on_error: Callable[[str], None] | None = None
        self.on_status: Callable[[str], None] | None = None

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self) -> bool:
        with self._lifecycle_lock:
            if self._thread is not None and self._thread.is_alive():
                # The same worker may still be unwinding a blocking microphone
                # read.  Never clear its stop event and create a second reader.
                return self._running
            self._thread = None
        if self._running:
            return True
        if not self._recog:
            self._emit_error("缺少 SpeechRecognition/PyAudio，无法采集麦克风")
            return False
        with self._lifecycle_lock:
            self._running = True
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._loop,
                daemon=True,
                name="bangbang-speech",
            )
            self._thread.start()
        self._emit_state("thinking")
        return True

    def _open_microphone(self) -> bool:
        """Device discovery runs on the speech worker, never the Qt event loop."""
        try:
            device_index, device_name = _resolve_input_device(config.VOICE_INPUT_DEVICE)
            self._mic_device_name = device_name
            self._mic = sr.Microphone(
                device_index=device_index,
                sample_rate=config.SAMPLE_RATE,
                chunk_size=config.CHUNK_SIZE,
            )
            logging.info("已选择麦克风: %s (index=%s)", device_name, device_index)
            if self.on_response:
                self.on_response(f"已选择麦克风：{device_name}，正在加载语音模型…")
        except Exception as exc:
            self._emit_error(f"麦克风初始化失败: {exc}")
            return False

        return True

    def stop(self) -> bool:
        self._running = False
        self._stop.set()
        self._asr.close(force=True)
        thread = self._thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        stopped = thread is None or not thread.is_alive()
        if stopped:
            with self._lifecycle_lock:
                if self._thread is thread:
                    self._thread = None
        self._emit_state("idle")
        return stopped

    def shutdown(self) -> None:
        self.stop()

    def simulate_text(self, text: str) -> str:
        """绕过麦克风测试完整安全门控和本地 Qwen。"""
        normalized = (text or "").strip()
        if not normalized:
            return ""
        if self.on_asr:
            self.on_asr(normalized)
        self._emit_state("thinking")
        reply = self._call_llm(normalized)
        if self.on_response:
            self.on_response(reply)
        self._emit_state("idle")
        return reply

    def _loop(self) -> None:
        started = time.monotonic()
        if not self._open_microphone():
            self._running = False
            self._emit_state("idle")
            return
        if self._stop.is_set():
            self._emit_state("idle")
            return
        # Warm Qwen independently while ASR loads. Listening does not wait for
        # Qwen, Kokoro, or discovery of the robot's motion service.
        self._start_background_warmup()
        try:
            self._emit_state("thinking")
            self._asr.start()
            logging.info("本地中英文 ASR 已就绪: %s", self._asr.runtime)
        except LocalAsrError as exc:
            self._emit_error(str(exc))
            self._running = False
            self._emit_state("idle")
            return

        if self._stop.is_set() or not self._running:
            self._emit_state("idle")
            return

        # UACDemoV1.0 在 PulseAudio 下反复打开/关闭输入流后可能继续显示 RUNNING，
        # 但只返回全零 PCM。整个语音会话只打开一次麦克风并持续复用同一条流。
        try:
            with self._mic as source:
                self._recog.adjust_for_ambient_noise(
                    source,
                    duration=config.MICROPHONE_CALIBRATION_SECONDS,
                )
                calibrated_energy = self._recog.energy_threshold
                self._recog.energy_threshold = _microphone_energy_threshold(calibrated_energy)
                # 校准后锁定门限。持续动态下调会在安静环境中逐渐把 USB 声卡底噪
                # 当成一句话的起点，引发 Whisper 静音幻觉。
                self._recog.dynamic_energy_threshold = False
                logging.info(
                    "麦克风已连接: %s (%s Hz, energy_threshold=%.1f, calibrated=%.1f)",
                    self._mic_device_name,
                    config.SAMPLE_RATE,
                    self._recog.energy_threshold,
                    calibrated_energy,
                )
                if self.on_response:
                    self.on_response("语音识别已就绪，请开始说话。")
                logging.info("语音开始监听，启动耗时 %.3fs", time.monotonic() - started)

                while self._running and not self._stop.is_set():
                    self._emit_state("listening")
                    try:
                        audio = self._recog.listen(source, timeout=4, phrase_time_limit=10)
                    except sr.WaitTimeoutError:
                        continue
                    except Exception as exc:
                        if self._running:
                            self._emit_error(f"录音异常: {exc}")
                        continue

                    self._emit_state("thinking")
                    audio_seconds = len(audio.frame_data) / (audio.sample_rate * audio.sample_width)
                    logging.info("录音结束，音频 %.3fs，开始本地识别", audio_seconds)
                    self._emit_status("正在识别语音…")
                    try:
                        text, language = self._asr.transcribe_audio(audio)
                    except LocalAsrError as exc:
                        if self._running:
                            self._emit_error(f"本地语音识别失败: {exc}")
                        continue
                    if self._stop.is_set() or not self._running:
                        break
                    asr_seconds = float(self._asr.last_result.get("total_seconds", 0.0))
                    logging.info(
                        "ASR 完成：录音=%.3fs，识别=%.3fs，主模型=%.3fs，复核加载=%.3fs，"
                        "复核解码=%.3fs，有效文本=%s，静音概率=%.3f，拒绝原因=%s",
                        audio_seconds, asr_seconds,
                        float(self._asr.last_result.get("primary_seconds", 0.0)),
                        float(self._asr.last_result.get("rescue_load_seconds", 0.0)),
                        float(self._asr.last_result.get("rescue_decode_seconds", 0.0)),
                        bool(text), float(self._asr.last_result.get("no_speech_prob", 0.0)),
                        self._asr.last_result.get("rejection_reason", ""),
                    )
                    if not text:
                        self._adjust_rejected_noise(audio)
                        self._emit_state("listening")
                        self._emit_status(f"未识别到清晰语音（{asr_seconds:.1f}秒），继续监听")
                        rejection_reason = self._asr.last_result.get("rejection_reason")
                        if rejection_reason:
                            logging.warning(
                                "已拒绝无效 ASR 结果: %s", rejection_reason
                            )
                        else:
                            logging.info("已忽略静音或低置信度音频")
                        continue
                    self._emit_status(f"语音识别完成（{asr_seconds:.1f}秒）")
                    if self._asr.last_result.get("rescue_attempted"):
                        logging.info(
                            "ASR 高精度复核%s（%s）：主模型=%s，复核模型=%s，决策=%s",
                            "已采用" if self._asr.last_result.get("rescue_applied") else "未替换",
                            self._asr.last_result.get("rescue_reason", "低置信度"),
                            self._asr.last_result.get("primary_text", ""),
                            self._asr.last_result.get("rescue_text", ""),
                            self._asr.last_result.get("rescue_decision", ""),
                        )
                    if self._asr.last_result.get("hallucination_stripped"):
                        logging.info(
                            "已移除 Whisper 字幕幻觉尾巴: %s",
                            self._asr.last_result["hallucination_stripped"],
                        )
                    logging.info(
                        "识别语言=%s，耗时=%.3fs（主模型=%.3fs，复核加载=%.3fs，复核解码=%.3fs），文本=%s",
                        language or "unknown",
                        float(self._asr.last_result.get("total_seconds", 0.0)),
                        float(self._asr.last_result.get("primary_seconds", 0.0)),
                        float(self._asr.last_result.get("rescue_load_seconds", 0.0)),
                        float(self._asr.last_result.get("rescue_decode_seconds", 0.0)),
                        text,
                    )
                    if self.on_asr:
                        self.on_asr(text)

                    self._emit_status("正在处理语音内容…")
                    try:
                        reply = self._call_llm(text)
                    except Exception as exc:
                        # 单条模型响应或动作解析异常不能终止持续麦克风流。
                        logging.exception("本轮语音处理失败")
                        self._emit_error(f"本轮语音处理失败: {exc}")
                        reply = (
                            "这句话没有处理成功，请再说一次。"
                            if _contains_chinese(text)
                            else "That request could not be processed. Please try again."
                        )
                    if self.on_response:
                        self.on_response(reply)
        except Exception as exc:
            if self._running:
                self._emit_error(f"麦克风采集失败: {exc}")

        self._running = False
        self._emit_state("idle")

    def _start_background_warmup(self) -> None:
        if not config.OLLAMA_WARMUP:
            return
        if self._warmup_thread is not None and self._warmup_thread.is_alive():
            return
        self._warmup_thread = threading.Thread(
            target=self._warm_up_llm, daemon=True, name="bangbang-qwen-warmup"
        )
        self._warmup_thread.start()

    def _warm_up_llm(self) -> None:
        # A separate client keeps warmup and foreground conversation state apart.
        try:
            if self._stop.is_set():
                return
            timing = OllamaClient().warm_up()
            logging.info("Qwen 后台预热完成: %.3fs", timing.get("wall_seconds", 0.0))
        except (OllamaError, ValueError) as exc:
            if self._running:
                self._emit_error(f"Qwen 后台预热未完成，首轮对话可能较慢：{exc}")

    def _adjust_rejected_noise(self, audio) -> None:
        # Only learn from a long, steady recording ASR classified as silence.
        # Rejected speech with varying loudness must not raise the threshold.
        if float(self._asr.last_result.get("no_speech_prob", 0.0)) < 0.8:
            return
        level = _steady_noise_rms(audio)
        if level is None:
            return
        previous = self._recog.energy_threshold
        target = level * getattr(self._recog, "dynamic_energy_adjustment_ratio", 1.5)
        if target > previous:
            self._recog.energy_threshold = target
            logging.warning("检测到持续底噪，录音门限 %.1f → %.1f", previous, target)

    def _emit_status(self, message: str) -> None:
        logging.info("语音状态：%s", message)
        if self.on_status:
            self.on_status(message)

    def _on_asr_progress(self, progress: dict) -> None:
        if self._stop.is_set() or not self._running:
            return
        stage = progress.get("stage")
        logging.info("ASR 阶段=%s，主模型已耗时=%.3fs", stage,
                     float(progress.get("primary_seconds", 0.0)))
        if stage == "rescue_load":
            self._emit_status("正在加载高精度复核模型…")
        elif stage == "rescue_decode":
            self._emit_status("正在进行高精度复核…")

    def _call_llm(self, text: str) -> str:
        # 报警和急停是安全动作，不要求移动口令，也不依赖模型在线。
        if is_alarm_request(text):
            return self._handle_alarm(text)
        if is_emergency_stop(text):
            stopped = self._execute_actions([SafeAction("stop")])
            if stopped:
                reply = (
                    "邦邦已紧急停止。"
                    if _contains_chinese(text)
                    else "Bangbang has stopped immediately."
                )
            else:
                reply = (
                    "紧急停止未收到控制回执，请立即使用遥控器急停。"
                    if _contains_chinese(text)
                    else (
                        "No control acknowledgement for emergency stop. "
                        "Use the physical remote stop now."
                    )
                )
            self._speak(reply)
            return reply

        gate = gate_command(text)
        if gate.authorized:
            if not gate.command_text:
                reply = ("请在执行口令后说明具体动作。" if _contains_chinese(text)
                         else "Please say the specific action after the execute phrase.")
                self._speak(reply)
                return reply
            keyword_actions = parse_deterministic_command(gate.command_text)
            if keyword_actions is None and is_fixed_action_command(text):
                reply = (
                    "我没听清具体动作。请说执行前进、后退、左转、右转、停止、趴下或站起动作。"
                    if _contains_chinese(text)
                    else (
                        "I did not catch the action. Say execute forward, backward, left turn, "
                        "right turn, stop, sit, or stand action."
                    )
                )
                self._speak(reply)
                return f"{reply}\n→ 未执行任何动作"
            # 用户要求动作先经过 Qwen 做语义分析；随后对“前进/后退/转向”等
            # 常用词做确定性参数提取。这样既能理解自然表达，又不会把模型自由文本
            # 直接传入 SDK。复杂动作在关键词解析不适用时才采用 Qwen 的 JSON 动作。
            logging.info("正在使用本地 Qwen 分析动作: %s", gate.command_text)
            parsed = self._llm.parse_command(gate.command_text)
            if keyword_actions is not None:
                parsed["actions"] = keyword_actions
                # 关键词覆盖后使用固定短确认，避免模型一边要求重述、一边实际
                # 移动，也缩短动作期间因半双工 TTS 导致的麦克风静音窗口。
                parsed["reply"] = (
                    "收到。" if _contains_chinese(text) else "Command received."
                )
                parsed["parser"] = "qwen_then_keyword"
                logging.info("已将动作关键词转换为 SDK 参数: %s", gate.command_text)
            else:
                parsed["parser"] = "qwen_json"
            actions, warnings = validate_actions(parsed.get("actions", []))
            reply = str(parsed.get("reply", "")).strip() or (
                "已收到动作指令。" if _contains_chinese(text) else "Command received."
            )
            if parsed.get("error"):
                self._emit_error(f"本地 Qwen 指令解析失败：{parsed['error']}")
            if warnings:
                logging.warning("动作安全校验: %s", "; ".join(warnings))
            if actions:
                # 先同步完成第一条动作的下发/ACK，再播报。旧顺序会等待 Qwen、
                # Kokoro 合成和整段播放之后才动，首次使用时常被误判为完全失效。
                accepted = self._execute_actions(actions)
                summary = ", ".join(_describe_action(action) for action in actions)
                if accepted:
                    self._speak(reply)
                    return f"{reply}\n→ 已下发动作: {summary}"
                failure = (
                    "动作未能下发，请检查 Unitree SDK2、控制网卡和机器狗连接。"
                    if _contains_chinese(text)
                    else (
                        "The action was not sent. Check Unitree SDK2, the control "
                        "interface, and the robot connection."
                    )
                )
                self._speak(failure)
                return f"{failure}\n→ 未执行任何动作"
            self._speak(reply)
            return f"{reply}\n→ 未执行任何动作"

        if _looks_like_motion_request(text):
            if _contains_chinese(text):
                reply = "为了安全，我不会直接移动。请说“执行指令”加动作，或说“执行前进动作”。"
            else:
                reply = (
                    "For safety I will not move. Say “execute command” followed "
                    "by the command, or “execute forward action”."
                )
            self._speak(reply)
            return reply

        try:
            reply = self._llm.chat(text)
        except OllamaError as exc:
            self._emit_error(f"本地 Qwen 对话失败：{exc}")
            reply = (
                "本地 Qwen 暂时没有响应，请检查 Ollama 服务。"
                if _contains_chinese(text)
                else "Local Qwen is not responding. Please check the Ollama service."
            )
        self._speak(reply)
        return reply

    def _handle_alarm(self, text: str) -> str:
        stopped = self._execute_actions([SafeAction("stop")])
        result = self._alarm.trigger(text)
        for error in result.errors:
            self._emit_error(error)
        external = any(channel in result.channels for channel in ("ros2", "site_command"))
        chinese = _contains_chinese(text)
        if not stopped:
            reply = (
                "报警已触发，但停车没有收到控制回执，请立即使用遥控器急停。"
                if chinese
                else (
                    "The alert was triggered, but stop was not acknowledged. "
                    "Use the physical remote stop now."
                )
            )
        elif external:
            reply = (
                "紧急报警已触发，邦邦已经停车，并已发送现场报警事件。"
                if chinese
                else (
                    "Emergency alert triggered. Bangbang has stopped and sent "
                    "the on-site alert event."
                )
            )
        elif result.success:
            reply = (
                "邦邦已经停车并记录本地报警；尚未连接现场报警设备。"
                if chinese
                else (
                    "Bangbang has stopped and logged the alert locally; no "
                    "on-site alarm device is connected."
                )
            )
        else:
            reply = (
                "邦邦已经停车，但报警链路触发失败，请立即人工求助。"
                if chinese
                else (
                    "Bangbang has stopped, but the alert chain failed. "
                    "Please seek human help now."
                )
            )
        self._speak(reply)
        return reply

    def _execute_actions(self, actions: list[SafeAction]) -> bool:
        """离线版只验证动作；部署版重写并下发到 Go2。"""
        logging.info("[模拟] 安全动作: %s", actions)
        return True

    def _speak(self, text: str) -> None:
        """部署版重写为本地 TTS。"""

    def _emit_state(self, state: str) -> None:
        if self.on_state:
            self.on_state(state)

    def _emit_error(self, message: str) -> None:
        logging.warning(message)
        if self.on_error:
            self.on_error(message)


def _contains_chinese(text: str) -> bool:
    return bool(re.search(r"[\u4e00-\u9fff]", text))


def _microphone_energy_threshold(calibrated: float) -> float:
    threshold = max(calibrated, config.MICROPHONE_MIN_ENERGY)
    if calibrated > config.MICROPHONE_MAX_ENERGY:
        logging.warning(
            "现场底噪校准门限 %.1f 超过配置参考上限 %.1f，保留实测门限以避免持续误录音",
            calibrated, config.MICROPHONE_MAX_ENERGY,
        )
    return threshold


def _steady_noise_rms(audio) -> float | None:
    samples_per_chunk = max(1, round(audio.sample_rate * 0.05))
    raw = audio.frame_data
    chunk_bytes = samples_per_chunk * audio.sample_width
    levels = sorted(audioop.rms(raw[start:start + chunk_bytes], audio.sample_width)
                    for start in range(0, len(raw) - chunk_bytes + 1, chunk_bytes))
    if len(levels) < 40:
        return None
    low, high = levels[len(levels) // 10], levels[len(levels) * 9 // 10]
    if low <= 0 or high > low * 2.0:
        return None
    return float(levels[len(levels) // 2])


def _choose_input_device(device_infos: list[dict], preferred: str) -> tuple[int | None, str]:
    """只从具备输入通道的 PyAudio 设备中按索引或名称选择。"""
    target = (preferred or "").strip()
    if not target or target.lower() in {"auto", "default"}:
        return None, "system default"

    inputs = [info for info in device_infos if int(info.get("maxInputChannels", 0)) > 0]
    if target.isdigit():
        index = int(target)
        for info in inputs:
            if int(info.get("index", -1)) == index:
                return index, str(info.get("name", index))
    else:
        folded = target.casefold()
        exact = [info for info in inputs if str(info.get("name", "")).casefold() == folded]
        partial = [info for info in inputs if folded in str(info.get("name", "")).casefold()]
        matches = exact or partial
        if matches:
            info = matches[0]
            return int(info["index"]), str(info.get("name", info["index"]))

    available = ", ".join(
        f"{info.get('index')}:{info.get('name')}" for info in inputs
    ) or "无"
    raise RuntimeError(f"找不到输入设备 {target!r}；可用输入设备：{available}")


def _resolve_input_device(preferred: str) -> tuple[int | None, str]:
    pyaudio = sr.Microphone.get_pyaudio()
    manager = pyaudio.PyAudio()
    try:
        infos = [manager.get_device_info_by_index(i) for i in range(manager.get_device_count())]
        return _choose_input_device(infos, preferred)
    finally:
        manager.terminate()


def _looks_like_motion_request(text: str) -> bool:
    lowered = text.lower()
    zh = ("前进", "后退", "左转", "右转", "移动", "走过去", "趴下", "站起来", "沿着", "引导线")
    en = re.search(
        r"^(?:bangbang[, ]*)?(?:(?:please|can you|could you|would you)\s+)?"
        r"(?:move|go|walk|forward|backward|turn|sit|stand|follow)\b",
        lowered.strip(),
    )
    return any(word in text for word in zh) or bool(en)


def _describe_action(action: SafeAction) -> str:
    if action.action == "visual_guide":
        return f"visual_guide({action.duration:.0f}s)"
    if action.action in {"stop", "sit", "stand", "recovery"}:
        return action.action
    if action.action.startswith("turn_") and action.angle:
        return f"{action.action}({action.angle:.0f}°)"
    return f"{action.action}({action.duration:.1f}s)"
