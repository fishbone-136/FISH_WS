"""不依赖第三方 SDK 的本机 Ollama 客户端。"""

from __future__ import annotations

import json
import logging
import urllib.error
import urllib.parse
import urllib.request
import time
from typing import Any

from . import config


class OllamaError(RuntimeError):
    """本地 Ollama 不可用或返回无效内容。"""


class OllamaClient:
    """通过 Ollama 的本机回环端口运行 Qwen，不访问外部服务。"""

    _LOCAL_HOSTS = {"127.0.0.1", "localhost", "::1"}

    def __init__(self, host: str | None = None, model: str | None = None,
                 timeout: float | None = None):
        self.host = (host or config.OLLAMA_HOST).rstrip("/")
        parsed = urllib.parse.urlparse(self.host)
        if parsed.scheme not in {"http", "https"} or parsed.hostname not in self._LOCAL_HOSTS:
            raise ValueError("OLLAMA_HOST 必须指向本机 127.0.0.1、localhost 或 ::1")
        self.model = (model or config.OLLAMA_MODEL).strip()
        self.timeout = float(timeout or config.OLLAMA_TIMEOUT)
        self._resolved_models: dict[tuple[str, bool], str] = {}
        self.last_timing: dict[str, float | int] = {}
        # 明确禁用代理；此客户端只访问已经校验过的本机回环地址。
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def list_models(self) -> list[str]:
        data = self._request("GET", "/api/tags")
        return [str(item.get("name", "")) for item in data.get("models", []) if item.get("name")]

    def is_ready(self, model: str | None = None, vision: bool = False) -> tuple[bool, str]:
        try:
            selected = self.resolve_model(model, vision=vision)
            return True, selected
        except Exception as exc:
            return False, str(exc)

    def resolve_model(self, requested: str | None = None, vision: bool = False) -> str:
        requested = (requested or self.model).strip()
        key = (requested, vision)
        if key in self._resolved_models:
            return self._resolved_models[key]

        names = self.list_models()
        if requested and requested in names:
            selected = requested
        elif vision:
            selected = next((name for name in names if "qwen" in name.lower()
                             and any(tag in name.lower() for tag in ("vl", "vision"))), "")
        else:
            selected = next((name for name in names if "qwen" in name.lower()), "")

        if not selected:
            purpose = "Qwen 视觉模型" if vision else "Qwen 模型"
            installed = ", ".join(names) if names else "无"
            raise OllamaError(f"本机未找到{purpose}；当前模型：{installed}")
        if selected != requested:
            logging.warning("Ollama 模型 %s 未找到，自动使用 %s", requested, selected)
        self._resolved_models[key] = selected
        return selected

    def chat(self, messages: list[dict[str, Any]], *, temperature: float = 0.4,
             json_output: bool = False, model: str | None = None,
             vision: bool = False, max_tokens: int | None = None) -> str:
        selected = self.resolve_model(model, vision=vision)
        payload: dict[str, Any] = {
            "model": selected,
            "messages": messages,
            "stream": False,
            "keep_alive": config.OLLAMA_KEEP_ALIVE,
            "options": {
                "temperature": float(temperature),
                "num_ctx": config.OLLAMA_CONTEXT_LENGTH,
                "num_predict": int(max_tokens or config.OLLAMA_MAX_PREDICT),
            },
        }
        if json_output:
            payload["format"] = "json"
        started = time.monotonic()
        data = self._request("POST", "/api/chat", payload)
        self.last_timing = self._timing_from_response(data, time.monotonic() - started)
        logging.info(
            "Ollama 推理耗时 %.3fs（模型加载 %.3fs，prompt %.3fs，生成 %.3fs，%s tokens）",
            self.last_timing["wall_seconds"],
            self.last_timing["load_seconds"],
            self.last_timing["prompt_seconds"],
            self.last_timing["eval_seconds"],
            self.last_timing["eval_count"],
        )
        content = data.get("message", {}).get("content", "")
        if not isinstance(content, str) or not content.strip():
            raise OllamaError("Ollama 返回了空回复")
        return content.strip()

    def warm_up(self, model: str | None = None) -> dict[str, float | int]:
        """在开始监听前装载 Qwen，避免首句话承担几十秒冷启动。"""
        selected = self.resolve_model(model)
        payload = {
            "model": selected,
            "messages": [{"role": "user", "content": "好"}],
            "stream": False,
            "keep_alive": config.OLLAMA_KEEP_ALIVE,
            "options": {
                "temperature": 0.0,
                # 与正式请求使用相同缓存大小，避免首轮重新加载 runner。
                "num_ctx": config.OLLAMA_CONTEXT_LENGTH,
                "num_predict": 1,
            },
        }
        started = time.monotonic()
        data = self._request("POST", "/api/chat", payload)
        self.last_timing = self._timing_from_response(data, time.monotonic() - started)
        return dict(self.last_timing)

    def chat_with_image(self, prompt: str, image_base64: str, *,
                        system: str = "", model: str | None = None) -> str:
        messages: list[dict[str, Any]] = []
        if system:
            messages.append({"role": "system", "content": system})
        messages.append({"role": "user", "content": prompt, "images": [image_base64]})
        return self.chat(messages, temperature=0.2, model=model, vision=True)

    def _request(self, method: str, path: str,
                 payload: dict[str, Any] | None = None) -> dict[str, Any]:
        body = None
        headers = {"Accept": "application/json"}
        if payload is not None:
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            headers["Content-Type"] = "application/json"
        request = urllib.request.Request(
            f"{self.host}{path}", data=body, headers=headers, method=method
        )
        try:
            with self._opener.open(request, timeout=self.timeout) as response:
                raw = response.read().decode("utf-8")
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")[:300]
            raise OllamaError(f"Ollama HTTP {exc.code}: {detail}") from exc
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise OllamaError(
                f"无法连接本机 Ollama（{self.host}），请先执行 ollama serve：{exc}"
            ) from exc
        try:
            data = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise OllamaError("Ollama 返回了非 JSON 内容") from exc
        if data.get("error"):
            raise OllamaError(str(data["error"]))
        return data

    @staticmethod
    def _timing_from_response(data: dict[str, Any], wall_seconds: float) -> dict[str, float | int]:
        def seconds(key: str) -> float:
            try:
                return round(float(data.get(key, 0)) / 1_000_000_000.0, 3)
            except (TypeError, ValueError):
                return 0.0

        return {
            "wall_seconds": round(float(wall_seconds), 3),
            "total_seconds": seconds("total_duration"),
            "load_seconds": seconds("load_duration"),
            "prompt_seconds": seconds("prompt_eval_duration"),
            "eval_seconds": seconds("eval_duration"),
            "prompt_count": int(data.get("prompt_eval_count", 0) or 0),
            "eval_count": int(data.get("eval_count", 0) or 0),
        }
