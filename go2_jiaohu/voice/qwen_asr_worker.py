"""在 yuyin 的隔离环境中常驻运行 Qwen ASR，stdin/stdout 使用 JSON Lines。

直接作为脚本执行，不导入 ROS/UI 或依赖它们的 Python 包路径。
"""

from __future__ import annotations

import argparse
from contextlib import redirect_stdout
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import wave


def _send(payload: dict) -> None:
    try:
        print(json.dumps(payload, ensure_ascii=False), flush=True)
    except BrokenPipeError:
        raise SystemExit(0)


def _language(value: str) -> str | None:
    return {"auto": None, "zh": "Chinese", "中文": "Chinese",
            "en": "English", "英文": "English", "yue": "Cantonese"}.get(value.lower(), value)


def _language_code(value: str) -> str:
    return {"chinese": "zh", "english": "en", "cantonese": "yue"}.get(value.lower(), value)


def _select_device(torch, requested: str, root: Path):
    automatic = requested == "auto"
    device = torch.device(("cuda:0" if torch.cuda.is_available() else "cpu") if automatic else requested)
    if device.type not in {"cpu", "cuda"}:
        raise ValueError("QWEN_ASR_DEVICE 支持 auto、cpu 或 cuda:0")
    if device.type == "cpu":
        return device
    # 沿用 yuyin 的真实 GPU 运算检查；失败的 CUDA 上下文留在探测子进程。
    try:
        probe = subprocess.run(
            [sys.executable, "-I", "-B", str(root / "cuda_probe.py"), "--device", str(device)],
            capture_output=True, text=True, timeout=45, check=False,
        )
        report = json.loads(probe.stdout)
        if probe.returncode == 0 and report.get("ok"):
            return device
        error = report.get("error", "CUDA 运算检查失败")
    except (OSError, ValueError, subprocess.TimeoutExpired) as exc:
        error = str(exc)
    if not automatic:
        raise RuntimeError(f"Qwen CUDA 自检失败: {error}")
    print(f"Qwen CUDA 自检失败，使用 CPU: {error}", file=sys.stderr, flush=True)
    return torch.device("cpu")


def _read_audio(path: str):
    import numpy as np

    # 与 LocalWhisperASR.transcribe_audio 写出的 16kHz / PCM16 / mono 对应。
    with wave.open(path, "rb") as audio:
        if (audio.getnchannels(), audio.getsampwidth(), audio.getframerate()) != (1, 2, 16000):
            raise ValueError("Qwen ASR 需要 16kHz 单声道 PCM16 WAV")
        samples = np.frombuffer(audio.readframes(audio.getnframes()), dtype="<i2").astype(np.float32) / 32768.0
    return samples


def serve(model, torch, language: str | None, requests) -> None:
    """模型只加载一次；空音频不进入解码器，不虚构 Whisper 置信度。"""
    import numpy as np

    for line in requests:
        try:
            request = json.loads(line)
            if request.get("command") == "shutdown":
                return
            started = time.monotonic()
            audio = _read_audio(str(request["wav_path"]))
            if not audio.size or not np.any(audio):
                _send({"status": "ok", "backend": "qwen", "text": "", "language": "",
                       "total_seconds": round(time.monotonic() - started, 3),
                       "rejection_reason": "silent_audio"})
                continue
            _send({"status": "progress", "stage": "primary"})
            decode_started = time.monotonic()
            with redirect_stdout(sys.stderr), torch.inference_mode():
                result = model.transcribe(audio=(audio, 16000), language=language)[0]
            _send({"status": "ok", "backend": "qwen", "text": result.text.strip(),
                   "language": _language_code(result.language or ""),
                   "primary_seconds": round(time.monotonic() - decode_started, 3),
                   "total_seconds": round(time.monotonic() - started, 3),
                   "rejection_reason": "" if result.text.strip() else "empty_transcript"})
        except Exception as exc:
            _send({"status": "error", "error": f"Qwen 语音识别失败: {exc}"})


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--qwen-root", type=Path, required=True)
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--language", default="Chinese")
    args = parser.parse_args()
    os.environ["HF_HUB_OFFLINE"] = "1"
    os.environ["TRANSFORMERS_OFFLINE"] = "1"
    try:
        if not (args.model_dir / "config.json").is_file() or not any(args.model_dir.glob("*.safetensors")):
            raise ValueError(f"本地 Qwen 模型文件不完整: {args.model_dir}")
        with redirect_stdout(sys.stderr):
            import torch
            from qwen_asr import Qwen3ASRModel

            device = _select_device(torch, args.device, args.qwen_root)
            dtype = torch.float32 if device.type == "cpu" else torch.float16
            # 与 yuyin/asr_realtime.py 的 Transformers 模式保持一致。
            model = Qwen3ASRModel.from_pretrained(
                str(args.model_dir), device_map=str(device), dtype=dtype,
                attn_implementation="sdpa", local_files_only=True,
                max_inference_batch_size=1, max_new_tokens=256,
            )
        _send({"status": "ready", "backend": "qwen", "model": str(args.model_dir),
               "device": str(device), "compute_type": str(dtype)})
    except Exception as exc:
        _send({"status": "error", "error": f"Qwen ASR 初始化失败: {exc}"})
        return 1
    serve(model, torch, _language(args.language), sys.stdin)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
