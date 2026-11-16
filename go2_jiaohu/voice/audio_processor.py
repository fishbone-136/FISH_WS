"""邦邦本地中英文 TTS。"""

from __future__ import annotations

import logging
import os
import re
import shutil
import subprocess
import threading

from .local_tts import LocalNeuralTTS, LocalTtsError


logging.basicConfig(level=logging.INFO, format="%(asctime)s [BangBang TTS] %(message)s")


class AudioSystem:
    def __init__(self):
        self.tts_engine = None
        self.tts_available = False
        self._lock = threading.Lock()
        self._neural_tts = LocalNeuralTTS()
        if self._neural_tts.configured:
            logging.info("正在后台加载本地 Kokoro CUDA 温柔男声")
            self._neural_tts.start_async()
        else:
            logging.warning("未找到完整 Kokoro 模型，将使用系统语音")
            self._initialize_system_tts()

    def wait_until_neural_ready(self, timeout: float) -> bool:
        if not self._neural_tts.configured:
            return True
        return self._neural_tts.wait_until_ready(timeout)

    def _initialize_system_tts(self) -> None:
        if self.tts_available:
            return
        try:
            import pyttsx3

            self.tts_engine = pyttsx3.init()
            self.tts_engine.setProperty("rate", 155)
            self.tts_engine.setProperty("volume", 1.0)
            self.tts_available = True
            logging.info("系统 TTS 回退已初始化")
        except Exception as exc:
            logging.warning("pyttsx3 初始化失败，将回退到 espeak: %s", exc)

    def play_text_to_speech(self, text: str) -> None:
        if not text:
            return
        # UI 附加的动作摘要不需要朗读。
        spoken = text.split("\n→", 1)[0].strip()
        if not spoken:
            return
        logging.info("邦邦准备播报: %s", spoken)
        with self._lock:
            if self._neural_tts.configured:
                wav_path = ""
                try:
                    wav_path, timing = self._neural_tts.synthesize(spoken)
                    self._play_wave(wav_path)
                    logging.info(
                        "Kokoro 播报完成: language=%s generate=%ss audio=%ss",
                        timing.get("language", ""),
                        timing.get("generate_seconds", ""),
                        timing.get("audio_seconds", ""),
                    )
                    return
                except (LocalTtsError, OSError, subprocess.SubprocessError) as exc:
                    logging.error("Kokoro 播报失败，回退到系统语音: %s", exc)
                    self._initialize_system_tts()
                finally:
                    if wav_path:
                        try:
                            os.unlink(wav_path)
                        except OSError:
                            pass

            if self.tts_available and self.tts_engine is not None:
                try:
                    self._select_voice(spoken)
                    self.tts_engine.say(spoken)
                    self.tts_engine.runAndWait()
                    return
                except Exception as exc:
                    logging.error("pyttsx3 播报失败，回退到 espeak: %s", exc)

            executable = shutil.which("espeak-ng") or shutil.which("espeak")
            if not executable:
                logging.error("未找到可用的本地 TTS 程序")
                return
            language = "zh" if re.search(r"[\u4e00-\u9fff]", spoken) else "en"
            try:
                # 参数数组直接执行，不经过 shell，模型文本不能注入系统命令。
                subprocess.run(
                    [executable, "-a", "200", "-v", language, spoken],
                    check=False,
                    timeout=30.0,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except (OSError, subprocess.SubprocessError) as exc:
                logging.error("本地 TTS 播报失败: %s", exc)

    @staticmethod
    def _play_wave(wav_path: str) -> None:
        pulse_player = shutil.which("paplay")
        if pulse_player:
            command = [pulse_player]
            pulse_sink = os.environ.get("PULSE_SINK", "").strip()
            if pulse_sink:
                command.append(f"--device={pulse_sink}")
            command.append(wav_path)
        else:
            alsa_player = shutil.which("aplay")
            if not alsa_player:
                raise OSError("未找到 paplay 或 aplay")
            command = [alsa_player, "-q", wav_path]
        result = subprocess.run(
            command,
            check=False,
            timeout=90.0,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if result.returncode != 0:
            raise OSError(f"音频播放器退出码 {result.returncode}")

    def close(self) -> None:
        self._neural_tts.close(force=True)

    def _select_voice(self, text: str) -> None:
        if self.tts_engine is None:
            return
        target = "zh" if re.search(r"[\u4e00-\u9fff]", text) else "en"
        try:
            voices = self.tts_engine.getProperty("voices") or []
            ranked = []
            for voice in voices:
                blob = " ".join([
                    str(getattr(voice, "id", "")),
                    str(getattr(voice, "name", "")),
                    str(getattr(voice, "languages", "")),
                ]).lower()
                language_match = (
                    target == "zh" and any(mark in blob for mark in ("zh", "chinese", "mandarin"))
                    or target == "en" and any(mark in blob for mark in ("en", "english"))
                )
                if language_match:
                    male_score = 1 if any(mark in blob for mark in ("male", "man", "yunxi")) else 0
                    ranked.append((male_score, voice))
            if ranked:
                ranked.sort(key=lambda item: item[0], reverse=True)
                self.tts_engine.setProperty("voice", ranked[0][1].id)
        except Exception:
            pass

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
