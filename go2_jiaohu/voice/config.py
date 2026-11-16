"""邦邦本地交互系统配置。

所有推理都指向本机 Ollama，不需要任何云端 API Key。环境变量仅用于部署时
覆盖模型名称、设备和安全阈值。
"""

from __future__ import annotations

import os


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORKSPACE_DIR = os.path.dirname(PROJECT_DIR)


def _load_local_env() -> None:
    """让直接运行 Python 入口时也能读取项目 .env，且不覆盖已导出的变量。"""
    path = os.path.join(PROJECT_DIR, ".env")
    try:
        with open(path, "r", encoding="utf-8") as stream:
            for raw_line in stream:
                line = raw_line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, value = line.split("=", 1)
                key = key.strip()
                value = value.strip().strip("'\"")
                if key.replace("_", "").isalnum() and not key[0].isdigit():
                    os.environ.setdefault(key, value)
    except OSError:
        pass


_load_local_env()

ROBOT_NAME = os.environ.get("GO2_ROBOT_NAME", "邦邦").strip() or "邦邦"

# 只允许本机 Ollama。OllamaClient 还会校验 host，防止误连云端地址。
OLLAMA_HOST = os.environ.get("OLLAMA_HOST", "http://127.0.0.1:11434").strip()
OLLAMA_MODEL = os.environ.get("OLLAMA_MODEL", "qwen2.5:3b").strip() or "qwen2.5:3b"
OLLAMA_VISION_MODEL = os.environ.get("OLLAMA_VISION_MODEL", "").strip() or "qwen3-vl:2b-instruct"
OLLAMA_TIMEOUT = float(os.environ.get("OLLAMA_TIMEOUT", "30"))
# 语音问答只保留很短的历史，4096 上下文会无谓增加首 token 时间和显存占用。
OLLAMA_CONTEXT_LENGTH = int(os.environ.get("OLLAMA_CONTEXT_LENGTH", "2048"))
OLLAMA_MAX_PREDICT = int(os.environ.get("OLLAMA_MAX_PREDICT", "192"))
OLLAMA_CHAT_MAX_PREDICT = int(os.environ.get("OLLAMA_CHAT_MAX_PREDICT", "64"))
OLLAMA_HISTORY_MESSAGES = int(os.environ.get("OLLAMA_HISTORY_MESSAGES", "4"))
OLLAMA_KEEP_ALIVE = os.environ.get("OLLAMA_KEEP_ALIVE", "30m").strip() or "30m"
OLLAMA_WARMUP = os.environ.get("OLLAMA_WARMUP", "1") == "1"
OLLAMA_WARMUP_DEPENDENCY_TIMEOUT = max(
    0.0, float(os.environ.get("OLLAMA_WARMUP_DEPENDENCY_TIMEOUT", "30"))
)

# faster-whisper 在独立子进程运行，避免与 ROS 2 / CycloneDDS 的 native 库冲突。
# 模型路径允许相对项目目录配置，部署后不依赖启动命令的当前工作目录。
_bundled_whisper_model = os.path.join(
    PROJECT_DIR, "voice", "models", "faster-whisper-small"
)
_bundled_whisper_cpu_model = os.path.join(
    PROJECT_DIR, "voice", "models", "faster-whisper-small"
)
_default_whisper_model = (
    _bundled_whisper_model if os.path.isdir(_bundled_whisper_model) else "small"
)
WHISPER_MODEL = (
    os.environ.get("WHISPER_MODEL", _default_whisper_model).strip()
    or _default_whisper_model
)
if not os.path.isabs(WHISPER_MODEL) and ("/" in WHISPER_MODEL or "\\" in WHISPER_MODEL):
    WHISPER_MODEL = os.path.abspath(os.path.join(PROJECT_DIR, WHISPER_MODEL))
WHISPER_CPU_FALLBACK_MODEL = os.environ.get(
    "WHISPER_CPU_FALLBACK_MODEL",
    _bundled_whisper_cpu_model if os.path.isdir(_bundled_whisper_cpu_model) else "",
).strip()
if (WHISPER_CPU_FALLBACK_MODEL
        and not os.path.isabs(WHISPER_CPU_FALLBACK_MODEL)
        and ("/" in WHISPER_CPU_FALLBACK_MODEL or "\\" in WHISPER_CPU_FALLBACK_MODEL)):
    WHISPER_CPU_FALLBACK_MODEL = os.path.abspath(
        os.path.join(PROJECT_DIR, WHISPER_CPU_FALLBACK_MODEL)
    )
WHISPER_DEVICE = os.environ.get("WHISPER_DEVICE", "auto").strip() or "auto"
WHISPER_COMPUTE_TYPE = os.environ.get("WHISPER_COMPUTE_TYPE", "auto").strip() or "auto"
WHISPER_LOCAL_FILES_ONLY = os.environ.get(
    "WHISPER_LOCAL_FILES_ONLY", "1" if os.path.isdir(WHISPER_MODEL) else "0"
) == "1"
WHISPER_LANGUAGE = os.environ.get("WHISPER_LANGUAGE", "auto").strip().lower() or "auto"
WHISPER_MIN_AVG_LOGPROB = float(
    os.environ.get("WHISPER_MIN_AVG_LOGPROB", "-0.65")
)
WHISPER_MAX_NO_SPEECH_PROB = float(
    os.environ.get("WHISPER_MAX_NO_SPEECH_PROB", "0.70")
)
WHISPER_SUSPICIOUS_MIN_AVG_LOGPROB = float(
    os.environ.get("WHISPER_SUSPICIOUS_MIN_AVG_LOGPROB", "-0.35")
)
WHISPER_BEAM_SIZE = max(1, int(os.environ.get("WHISPER_BEAM_SIZE", "5")))
WHISPER_BEST_OF = max(1, int(os.environ.get("WHISPER_BEST_OF", "5")))
WHISPER_CHUNK_LENGTH = min(
    max(5, int(os.environ.get("WHISPER_CHUNK_LENGTH", "10"))), 30
)
WHISPER_MAX_NEW_TOKENS = max(
    16, int(os.environ.get("WHISPER_MAX_NEW_TOKENS", "96"))
)
# 抑制 Whisper 在底噪上循环生成同一短语。后处理还会再次拒绝已经形成的
# 循环，二者分别保护解码阶段和运动安全门。
WHISPER_REPETITION_PENALTY = max(
    1.0, float(os.environ.get("WHISPER_REPETITION_PENALTY", "1.15"))
)
WHISPER_NO_REPEAT_NGRAM_SIZE = max(
    0, int(os.environ.get("WHISPER_NO_REPEAT_NGRAM_SIZE", "3"))
)
# 现场日志确认：将整套运动词作为 hotwords 会在底噪上原样吐出，并循环
# “执行指令/左转”。默认关闭提示注入；如部署方确需实验，可显式配置少量词。
WHISPER_HOTWORDS = os.environ.get(
    "WHISPER_HOTWORDS",
    "",
).strip()

# small 负责低延迟实时识别；只有结果可信但置信度不足时，才懒加载 turbo
# 复核。清晰语音保持单次解码，避免运动口令固定做两遍 ASR。
_bundled_whisper_rescue_model = os.path.join(
    PROJECT_DIR, "voice", "models", "faster-whisper-large-v3-turbo"
)
_default_whisper_rescue_model = (
    _bundled_whisper_rescue_model
    if os.path.isdir(_bundled_whisper_rescue_model)
    else ""
)
WHISPER_RESCUE_MODEL = os.environ.get(
    "WHISPER_RESCUE_MODEL", _default_whisper_rescue_model
).strip()
if (
    WHISPER_RESCUE_MODEL
    and not os.path.isabs(WHISPER_RESCUE_MODEL)
    and ("/" in WHISPER_RESCUE_MODEL or "\\" in WHISPER_RESCUE_MODEL)
):
    WHISPER_RESCUE_MODEL = os.path.abspath(
        os.path.join(PROJECT_DIR, WHISPER_RESCUE_MODEL)
    )
WHISPER_PRELOAD_RESCUE = os.environ.get("WHISPER_PRELOAD_RESCUE", "0") == "1"
WHISPER_RESCUE_BEAM_SIZE = max(
    1, int(os.environ.get("WHISPER_RESCUE_BEAM_SIZE", "5"))
)
WHISPER_RESCUE_BEST_OF = max(
    1, int(os.environ.get("WHISPER_RESCUE_BEST_OF", "5"))
)
WHISPER_RESCUE_MIN_AVG_LOGPROB = float(
    os.environ.get("WHISPER_RESCUE_MIN_AVG_LOGPROB", "-0.45")
)
WHISPER_RESCUE_MIN_LANGUAGE_PROB = float(
    os.environ.get("WHISPER_RESCUE_MIN_LANGUAGE_PROB", "0.45")
)
WHISPER_RESCUE_MAX_NO_SPEECH_PROB = float(
    os.environ.get("WHISPER_RESCUE_MAX_NO_SPEECH_PROB", "0.45")
)
# 动作口令采用比闲聊更严格的声学门限；清晰口令仍只跑 small，只有边界结果
# 才交给 turbo 复核。
WHISPER_COMMAND_RESCUE_MIN_AVG_LOGPROB = float(
    os.environ.get("WHISPER_COMMAND_RESCUE_MIN_AVG_LOGPROB", "-0.35")
)
WHISPER_COMMAND_RESCUE_MAX_NO_SPEECH_PROB = float(
    os.environ.get("WHISPER_COMMAND_RESCUE_MAX_NO_SPEECH_PROB", "0.25")
)
WHISPER_COMMAND_RESCUE_MIN_LANGUAGE_PROB = float(
    os.environ.get("WHISPER_COMMAND_RESCUE_MIN_LANGUAGE_PROB", "0.60")
)

# Jetson 上 USB 声卡由 PulseAudio 管理。PyAudio 看到的 UAC 硬件节点可能只有
# 输出通道，因此默认通过名为 pulse 的虚拟设备采集，再由 PULSE_SOURCE 精确路由。
VOICE_INPUT_DEVICE = os.environ.get("GO2_VOICE_INPUT_DEVICE", "pulse").strip() or "pulse"
MICROPHONE_CALIBRATION_SECONDS = float(
    os.environ.get("GO2_MIC_CALIBRATION_SECONDS", "0.6")
)
MICROPHONE_MIN_ENERGY = float(os.environ.get("GO2_MIC_MIN_ENERGY", "200"))
# 旧上限 450 会把现场校准出的更高底噪门限强行压回 450；日志中每隔数秒把
# 底噪当语音正是该上限持续命中的表现。保留上限防止校准时有人说话，但放宽到
# USB UAC 麦克风仍能可靠触发正常说话的范围。
# Reference ceiling for diagnostics; never force the calibrated threshold below
# measured room noise, which would continually open ten-second recordings.
MICROPHONE_MAX_ENERGY = float(os.environ.get("GO2_MIC_MAX_ENERGY", "1200"))
MICROPHONE_PAUSE_THRESHOLD = float(
    os.environ.get("GO2_MIC_PAUSE_THRESHOLD", "0.45")
)
MICROPHONE_NON_SPEAKING_DURATION = float(
    os.environ.get("GO2_MIC_NON_SPEAKING_DURATION", "0.25")
)
MICROPHONE_PHRASE_THRESHOLD = float(
    os.environ.get("GO2_MIC_PHRASE_THRESHOLD", "0.45")
)
TTS_ECHO_COOLDOWN_SECONDS = float(
    os.environ.get("GO2_TTS_ECHO_COOLDOWN_SECONDS", "0.25")
)

# Kokoro 在独立的系统 Python 子进程中运行，避免 PyTorch/NumPy 与 ROS 2 的
# Python 环境互相污染。模型与 Jetson CUDA 运行库均位于工作空间，不访问云端。
TTS_PYTHON = os.environ.get("GO2_TTS_PYTHON", "/usr/bin/python3").strip()
TTS_MODEL_DIR = os.environ.get(
    "GO2_TTS_MODEL_DIR",
    os.path.join(PROJECT_DIR, "voice", "models", "kokoro-pytorch"),
).strip()
if TTS_MODEL_DIR and not os.path.isabs(TTS_MODEL_DIR):
    TTS_MODEL_DIR = os.path.abspath(os.path.join(PROJECT_DIR, TTS_MODEL_DIR))
TTS_VOICE = os.environ.get("GO2_TTS_VOICE", "zm_yunxi").strip() or "zm_yunxi"
TTS_SPEED = float(os.environ.get("GO2_TTS_SPEED", "1.04"))
TTS_VOLUME = float(os.environ.get("GO2_TTS_VOLUME", "3.0"))
TTS_STARTUP_TIMEOUT = float(os.environ.get("GO2_TTS_STARTUP_TIMEOUT", "180"))
TTS_REQUEST_TIMEOUT = float(os.environ.get("GO2_TTS_REQUEST_TIMEOUT", "60"))

# 机器狗网络配置。
ROBOT_INTERFACE = os.environ.get("GO2_NETWORK_INTERFACE", "eth0").strip() or "eth0"
ROBOT_MOTION_BACKEND = os.environ.get("GO2_MOTION_BACKEND", "sdk").strip().lower()
if ROBOT_MOTION_BACKEND not in {"sdk", "ros"}:
    ROBOT_MOTION_BACKEND = "sdk"
ROBOT_SDK_RPC_TIMEOUT = max(
    0.2, float(os.environ.get("GO2_SDK_RPC_TIMEOUT", "2.0"))
)
ROBOT_SDK_VERIFY_SERVER = os.environ.get("GO2_SDK_VERIFY_SERVER", "1") == "1"
ROBOT_SDK_MOVE_INTERVAL = min(
    max(float(os.environ.get("GO2_SDK_MOVE_INTERVAL", "0.05")), 0.02), 0.20
)
ROBOT_COMMAND_TOPIC = os.environ.get(
    "GO2_INTERACTIVE_COMMAND_TOPIC", "/go2/interactive_command"
).strip() or "/go2/interactive_command"
ROBOT_COMMAND_STATUS_TOPIC = os.environ.get(
    "GO2_INTERACTIVE_STATUS_TOPIC", "/go2/interactive_status"
).strip() or "/go2/interactive_status"
ROBOT_COMMAND_ACK_TIMEOUT = float(
    os.environ.get("GO2_COMMAND_ACK_TIMEOUT", "2.0")
)

SAMPLE_RATE = 16000
CHUNK_SIZE = 512

# 离线唤醒模型存放路径（保留兼容现有部署）。
KWS_MODEL_DIR = os.path.join(os.path.dirname(__file__), "kws-model")

# 运动安全边界。模型输出会再次经过这些硬限制。
MAX_LINEAR_SPEED = float(os.environ.get("GO2_MAX_LINEAR_SPEED", "0.45"))
MAX_TURN_SPEED = float(os.environ.get("GO2_MAX_TURN_SPEED", "0.65"))
MAX_ACTION_DURATION = float(os.environ.get("GO2_MAX_ACTION_DURATION", "12"))
MAX_TOTAL_DURATION = float(os.environ.get("GO2_MAX_TOTAL_DURATION", "30"))
DEFAULT_LINEAR_SPEED = float(os.environ.get("GO2_DEFAULT_LINEAR_SPEED", "0.30"))
DEFAULT_TURN_SPEED = float(os.environ.get("GO2_DEFAULT_TURN_SPEED", "0.50"))
DEFAULT_STEP_LENGTH_M = min(
    max(float(os.environ.get("GO2_STEP_LENGTH_M", "0.40")), 0.10), 1.0
)
# 现场已有的开环标定系数只在安全层应用一次；执行端不再重复放大时间。
LINEAR_TIME_MULTIPLIER = float(
    os.environ.get("GO2_LINEAR_TIME_MULTIPLIER", "1.30")
)
TURN_TIME_MULTIPLIER = float(
    os.environ.get("GO2_TURN_TIME_MULTIPLIER", "2.00")
)
VISUAL_GUIDE_TIMEOUT = float(os.environ.get("GO2_VISUAL_GUIDE_TIMEOUT", "120"))
OBSTACLE_STOP_DISTANCE_M = float(os.environ.get("GO2_OBSTACLE_STOP_DISTANCE_M", "0.60"))

# A real, calibrated optical frame is mandatory for semantic navigation.
# Optional mount: x,y,z,roll,pitch,yaw (metres and degrees), camera body axes
# x forward/y left/z up, relative to base_link. No guessed mounting offsets.
VISUAL_CAMERA_FRAME = os.environ.get('GO2_VISUAL_CAMERA_FRAME', 'camera_color_optical_frame').strip()
VISUAL_CAMERA_MOUNT = os.environ.get('GO2_VISUAL_CAMERA_MOUNT', '').strip()
PERSON_DETECTOR_MODEL = os.environ.get('GO2_PERSON_DETECTOR_MODEL', os.path.join(
    WORKSPACE_DIR, 'ros2_ws', 'src', 'go2_navigation', 'go2_navigation', 'yolov8n.pt'))
# Measured optical-to-HAP calibration from the existing rigid sensor assembly.
# The HAP-to-base leg must come from runtime TF, including body roll/pitch.
VISUAL_CAMERA_CALIBRATION = os.environ.get(
    'GO2_VISUAL_CAMERA_CALIBRATION',
    os.path.join(PROJECT_DIR, 'config', 'visual_camera_calibration.json'),
).strip()
if VISUAL_CAMERA_CALIBRATION and not os.path.isabs(VISUAL_CAMERA_CALIBRATION):
    VISUAL_CAMERA_CALIBRATION = os.path.join(PROJECT_DIR, VISUAL_CAMERA_CALIBRATION)

# 报警始终先停止运动并写本地审计记录，再发布 ROS 2 事件。若现场已有短信、
# 声光或平台报警程序，可用 BANGBANG_ALARM_COMMAND 接入；执行时不会经过 shell。
ALARM_LOG_PATH = os.environ.get(
    "BANGBANG_ALARM_LOG",
    os.path.join(WORKSPACE_DIR, "log", "bangbang_emergency_alerts.jsonl"),
).strip()
ALARM_COMMAND = os.environ.get("BANGBANG_ALARM_COMMAND", "").strip()
ALARM_ROS_TOPIC = os.environ.get(
    "BANGBANG_ALARM_ROS_TOPIC", "/bangbang/emergency_alert"
).strip()
