"""Kokoro CUDA TTS worker using a JSON Lines stdin/stdout protocol."""

from __future__ import annotations

import json
import logging
import os
import re
import sys
import time
import warnings
import wave


_PROTOCOL_OUTPUT = sys.stdout


def _send(payload: dict) -> None:
    print(json.dumps(payload, ensure_ascii=False), file=_PROTOCOL_OUTPUT, flush=True)


def _language_for(text: str) -> str:
    chinese_count = len(re.findall(r"[\u3400-\u9fff]", text or ""))
    latin_count = len(re.findall(r"[A-Za-z]", text or ""))
    return "zh" if chinese_count > latin_count else "en"


def _generate(pipeline, text: str, voice: str, speed: float):
    chunks = []
    for result in pipeline(text, voice=voice, speed=speed):
        if result.audio is not None:
            chunks.append(result.audio.detach().float().cpu().numpy())
    return chunks


def main() -> int:
    # Keep the protocol clean and the UI terminal readable. Fatal errors are returned as JSON.
    warnings.filterwarnings("ignore")
    logging.getLogger().setLevel(logging.ERROR)
    logging.getLogger("jieba").setLevel(logging.ERROR)
    started = time.monotonic()
    try:
        import numpy as np
        import torch
        from kokoro import KModel, KPipeline

        if not torch.cuda.is_available():
            raise RuntimeError("Jetson CUDA is not available")

        model_dir = os.environ["BANGBANG_TTS_MODEL_DIR"]
        voice_name = os.environ.get("BANGBANG_TTS_VOICE", "zm_yunxi")
        voice_path = (
            voice_name
            if voice_name.endswith(".pt")
            else os.path.join(model_dir, "voices", f"{voice_name}.pt")
        )
        config_path = os.path.join(model_dir, "config.json")
        weights_path = os.path.join(model_dir, "kokoro-v1_0.pth")
        for required in (config_path, weights_path, voice_path):
            if not os.path.isfile(required):
                raise FileNotFoundError(required)

        # Replies have a different sequence length each turn. cuDNN benchmarking every new
        # shape costs more than it saves, so keep deterministic heuristic selection enabled.
        torch.backends.cudnn.benchmark = False
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
        model = KModel(config=config_path, model=weights_path).to("cuda").eval()
        zh_pipeline = KPipeline(
            lang_code="z",
            repo_id="hexgrad/Kokoro-82M",
            model=model,
            device="cuda",
        )
        en_pipeline = KPipeline(
            lang_code="a",
            repo_id="hexgrad/Kokoro-82M",
            model=model,
            device="cuda",
        )

        # Load G2P dictionaries, voice tensors, and CUDA kernels before the first real reply.
        _generate(zh_pipeline, "你好", voice_path, 1.04)
        _generate(en_pipeline, "Hello", voice_path, 1.04)
        _send({
            "status": "ready",
            "device": torch.cuda.get_device_name(0),
            "voice": voice_name,
            "load_seconds": round(time.monotonic() - started, 3),
        })
    except Exception as exc:
        _send({"status": "error", "error": f"Kokoro CUDA 初始化失败: {exc}"})
        return 1

    pipelines = {"zh": zh_pipeline, "en": en_pipeline}
    for line in sys.stdin:
        try:
            request = json.loads(line)
            if request.get("command") == "shutdown":
                break
            text = str(request.get("text", "")).strip()
            output_path = os.path.abspath(str(request["output_path"]))
            if not text:
                raise ValueError("TTS text is empty")
            speed = min(max(float(request.get("speed", 1.04)), 0.75), 1.35)
            volume = min(max(float(request.get("volume", 3.0)), 0.1), 4.0)
            language = _language_for(text)
            generated_at = time.monotonic()
            chunks = _generate(pipelines[language], text, voice_path, speed)
            if not chunks:
                raise RuntimeError("Kokoro returned no audio")
            audio = np.concatenate(chunks)
            # Kokoro 的瞬时峰值较高但平均响度偏低。线性增益会直接削平峰值，
            # 因此用 tanh 软限幅提升人声主体，同时平滑约束到合法 PCM 范围。
            amplified = np.tanh(audio * volume)
            pcm = (amplified * 32767).astype("<i2")
            with wave.open(output_path, "wb") as stream:
                stream.setnchannels(1)
                stream.setsampwidth(2)
                stream.setframerate(24000)
                stream.writeframes(pcm.tobytes())
            _send({
                "status": "ok",
                "language": language,
                "generate_seconds": round(time.monotonic() - generated_at, 3),
                "audio_seconds": round(len(audio) / 24000.0, 3),
            })
        except Exception as exc:
            _send({"status": "error", "error": str(exc)})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
