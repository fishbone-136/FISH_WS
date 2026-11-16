"""使用本机 Ollama/Qwen 的对话与受限动作解析器。"""

from __future__ import annotations

import json
import logging
import re
from typing import Any

from . import config
from .ollama_client import OllamaClient, OllamaError


CHAT_SYSTEM = f"""You are Bangbang（{config.ROBOT_NAME}）, a Unitree Go2 quadruped robot.
你活泼、可靠、友好。Match the language of the user's latest message exactly.
普通聊天绝不能声称已经移动、报警或操作硬件，也不要输出 JSON 或动作指令。
Even if ordinary chat mentions motion, only answer in words;
physical actions use a separate safety gate.
Keep every answer natural and concise, normally one or two short sentences."""

COMMAND_SYSTEM = f"""你是机器狗“{config.ROBOT_NAME}”的受限动作语义解析器。
输入已经通过“执行指令 / 执行命令 / 执行…动作 / execute command / execute ... action”安全口令校验。
只输出一个 JSON 对象，禁止 Markdown，格式为：
{{"reply":"简短确认语，同输入语言", "actions":[{{"action":"forward","speed":0.3,"duration":3.33,"angle":0}}]}}

只允许以下 action：
- forward / backward / left / right：前后或横向移动。默认距离 1 米，duration=距离/speed。
  “一步”按 {config.DEFAULT_STEP_LENGTH_M:.2f} 米换算，例如“两步”的距离是
  {config.DEFAULT_STEP_LENGTH_M * 2:.2f} 米，再用 duration=距离/speed。
- turn_left / turn_right：转向。默认 90 度，angle 填角度，speed 默认 0.5。
- stop / sit / stand / recovery：停止、趴下、站起、恢复站立。
- visual_guide：当用户要求沿地面引导线、盲道/导引线前进，或要求结合红绿灯通行时使用。
  该动作会调用 D435i 的本地视觉闭环，同时检测红灯停车、绿灯通行、近障停车；
  speed 默认 0.18，duration 默认 60 秒。不要在它前后添加普通移动动作。

最多 4 个动作。不要生成代码、工具名、路径、网络请求或白名单之外的动作。
reply 只能表示“收到/准备执行”，动作尚未收到硬件回执时禁止使用“已完成/完成了”。
无法确定动作时返回 actions 为空，并在 reply 中请用户重新说明。"""


_ACTION_ALIASES = {
    "forward": "forward",
    "moveforward": "forward",
    "goforward": "forward",
    "前进": "forward",
    "向前": "forward",
    "向前走": "forward",
    "backward": "backward",
    "movebackward": "backward",
    "gobackward": "backward",
    "后退": "backward",
    "向后": "backward",
    "向后走": "backward",
    "left": "left",
    "moveleft": "left",
    "左移": "left",
    "向左移动": "left",
    "right": "right",
    "moveright": "right",
    "右移": "right",
    "向右移动": "right",
    "turnleft": "turn_left",
    "左转": "turn_left",
    "向左转": "turn_left",
    "turnright": "turn_right",
    "右转": "turn_right",
    "向右转": "turn_right",
    "stop": "stop",
    "停止": "stop",
    "停下": "stop",
    "sit": "sit",
    "趴下": "sit",
    "卧倒": "sit",
    "stand": "stand",
    "standup": "stand",
    "站起": "stand",
    "站起来": "stand",
    "recovery": "recovery",
    "recoverystand": "recovery",
    "恢复站立": "recovery",
    "visualguide": "visual_guide",
    "视觉引导": "visual_guide",
}


class OllamaInstructionParser:
    def __init__(self, client: OllamaClient | None = None):
        self.client = client or OllamaClient()
        self._history: list[dict[str, str]] = []

    @property
    def is_ready(self) -> bool:
        return self.client.is_ready()[0]

    def chat(self, user_text: str) -> str:
        messages: list[dict[str, str]] = [{"role": "system", "content": CHAT_SYSTEM}]
        history_count = max(0, config.OLLAMA_HISTORY_MESSAGES)
        if history_count:
            messages.extend(self._history[-history_count:])
        language = _response_language(user_text)
        messages.append({"role": "system", "content": _language_instruction(language)})
        messages.append({"role": "user", "content": user_text})
        reply = self.client.chat(
            messages,
            temperature=0.35,
            max_tokens=config.OLLAMA_CHAT_MAX_PREDICT,
        )
        # Qwen 在长对话历史切换语言时偶尔会沿用上一轮语言。仅在确实不匹配时
        # 进行一次无历史重试，正常路径不会增加延迟。
        if not _reply_matches_language(reply, language):
            logging.warning("Qwen 回复语言与当前问题不一致，正在按当前语言重试")
            retry_messages = [
                {"role": "system", "content": CHAT_SYSTEM},
                {"role": "system", "content": _language_instruction(language)},
                {"role": "user", "content": user_text},
            ]
            try:
                retried_reply = self.client.chat(
                    retry_messages,
                    temperature=0.15,
                    max_tokens=config.OLLAMA_CHAT_MAX_PREDICT,
                )
                reply = (
                    retried_reply
                    if _reply_matches_language(retried_reply, language)
                    else _language_fallback(language)
                )
            except OllamaError:
                logging.exception("Qwen 同语言重试失败")
                reply = _language_fallback(language)
        self._history.extend([
            {"role": "user", "content": user_text},
            {"role": "assistant", "content": reply},
        ])
        history_limit = max(2, config.OLLAMA_HISTORY_MESSAGES * 2)
        self._history = self._history[-history_limit:]
        return reply

    def parse_command(self, user_text: str) -> dict[str, Any]:
        language = _response_language(user_text)
        try:
            raw = self.client.chat(
                [
                    {"role": "system", "content": COMMAND_SYSTEM},
                    {"role": "system", "content": _language_instruction(language)},
                    {"role": "user", "content": user_text},
                ],
                temperature=0.05,
                json_output=True,
                max_tokens=160,
            )
            data = self._decode_json_object(raw)
            if not isinstance(data.get("reply", ""), str):
                data["reply"] = ""
            elif data["reply"] and not _reply_matches_language(data["reply"], language):
                data["reply"] = (
                    "已收到动作指令。" if language == "zh" else "Command received."
                )
            elif re.search(r"已?完成|执行完|completed|finished", data["reply"], re.IGNORECASE):
                data["reply"] = (
                    "已收到动作指令。" if language == "zh" else "Command received."
                )
            raw_actions = data.get("actions", [])
            if not isinstance(raw_actions, list):
                data["actions"] = []
            else:
                # 模型偶尔只返回 reply 而省略 actions。之前这里先用 get 判断、
                # 随后又直接索引，导致 KeyError 冒泡并重启整个麦克风循环。
                data["actions"] = _normalize_model_actions(raw_actions)
            return data
        except (OllamaError, ValueError, json.JSONDecodeError) as exc:
            logging.error("本地 Qwen 指令解析失败: %s", exc)
            return {
                "reply": (
                    f"{config.ROBOT_NAME}没有听懂这条动作，请换一种说法。"
                    if language == "zh"
                    else "Bangbang did not understand that action. Please rephrase it."
                ),
                "actions": [],
                "error": str(exc),
            }

    @staticmethod
    def _decode_json_object(raw: str) -> dict[str, Any]:
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            match = re.search(r"\{.*\}", raw, re.DOTALL)
            if not match:
                raise
            data = json.loads(match.group(0))
        if not isinstance(data, dict):
            raise ValueError("模型输出不是 JSON 对象")
        return data


def _normalize_action_name(value: Any) -> str:
    """把 Qwen 可能输出的中英文动作关键词收敛到 SDK 白名单名称。"""
    raw = str(value or "").strip().casefold()
    compact = re.sub(r"[\s_-]+", "", raw)
    return _ACTION_ALIASES.get(compact, raw)


def _normalize_model_actions(raw_actions: list[Any]) -> list[dict[str, Any]]:
    normalized: list[dict[str, Any]] = []
    for item in raw_actions:
        if isinstance(item, str):
            normalized.append({"action": _normalize_action_name(item)})
        elif isinstance(item, dict):
            action = dict(item)
            action["action"] = _normalize_action_name(action.get("action", ""))
            normalized.append(action)
        else:
            # 保留非法项给统一安全校验器生成明确警告。
            normalized.append({"action": ""})
    return normalized


def _response_language(text: str) -> str:
    """按有效字符数量判断当前话语的主语言，支持中英夹杂和设备型号。"""
    chinese_count = len(re.findall(r"[\u3400-\u9fff]", text or ""))
    latin_count = len(re.findall(r"[A-Za-z]", text or ""))
    return "zh" if chinese_count > latin_count else "en"


def _language_instruction(language: str) -> str:
    if language == "zh":
        return (
            "语言锁定：用户当前使用中文。reply 必须只使用自然、简洁的简体中文；"
            "不要用英文作答，产品名和设备型号除外。"
        )
    return (
        "LANGUAGE LOCK: The user's latest message is English. Reply only in natural, "
        "concise English. Do not answer in Chinese. Write the robot's name as Bangbang."
    )


def _reply_matches_language(reply: str, language: str) -> bool:
    has_chinese = bool(re.search(r"[\u3400-\u9fff]", reply or ""))
    return has_chinese if language == "zh" else not has_chinese


def _language_fallback(language: str) -> str:
    if language == "zh":
        return "抱歉，我暂时没能组织好回答，请再问我一次。"
    return "Sorry, I could not formulate that answer. Please ask me once more."
