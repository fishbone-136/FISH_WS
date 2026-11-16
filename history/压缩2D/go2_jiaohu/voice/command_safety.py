"""语音命令门控、报警识别和模型动作输出校验。"""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Any

from . import config


@dataclass(frozen=True)
class GateResult:
    authorized: bool
    command_text: str = ""
    reason: str = "chat"


@dataclass(frozen=True)
class SafeAction:
    action: str
    speed: float = 0.0
    duration: float = 0.0
    angle: float = 0.0


_ZH_COMMAND_PATTERNS = (
    re.compile(r"执行\s*(?:指令|命令)\s*[:：,，]?\s*(?P<command>.*)", re.IGNORECASE),
    re.compile(r"执行\s*动作\s*[:：,，]?\s*(?P<command>.*)", re.IGNORECASE),
    re.compile(
        r"执行\s*(?P<command>[^.。！？!?;；]{1,60}?)\s*动作"
        r"(?:\s*(?:好[啊呀吧]?|啊|吧|呀|哦|啦))?(?:\s*[.。！？!?;；]|$)",
        re.IGNORECASE,
    ),
)

# 这些是现场同一组短口令反复出现的稳定音近误写。恢复必须同时满足：句首是
# “执行”的已知误写、动作来自有限白名单、句尾是“动作”的已知误写。这样不会
# 在普通聊天里把“哈萨”“顿座”等词单独改成机器人动作。
_ASR_FIXED_ACTION_FRAME = re.compile(
    r"^(?P<leading>\s*(?:(?:邦邦|bangbang)\s*[,，:：]?\s*)?)"
    r"(?:执行\s*(?:(?:指令|命令)\s*[:：,，]?\s*)?|"
    r"实行|直銜|直衔|直旋|直行|只想|提醒|尺寸使用|行)\s*"
    r"(?P<action>"
    r"前进|全劲|全进|前劲|前近|前镜|钱进|雄徑|雄径|闪电|"
    r"后退|左转|左传|左砖|折断|截断|右转|左移|右移|"
    r"停止|停下|急停|趴下|爬下|趴夏|帕下|帕萨|哈萨|"
    r"站起|站起来|起立|恢复站立|恢复站起|摔倒恢复"
    r")\s*(?:的\s*)?(?:动作|顿座|顿作|正左)"
    r"(?P<ending>\s*(?:好[啊呀吧]?|啊|吧|呀|哦|啦)?"
    r"\s*[.。！？!?;；]*\s*)$",
    re.IGNORECASE,
)

_ASR_FIXED_ACTION_ALIASES = {
    "全劲": "前进", "全进": "前进", "前劲": "前进", "前近": "前进",
    "前镜": "前进", "钱进": "前进", "雄徑": "前进", "雄径": "前进",
    "闪电": "前进",
    "左传": "左转", "左砖": "左转", "折断": "左转", "截断": "左转",
    "停下": "停止", "急停": "停止",
    "爬下": "趴下", "趴夏": "趴下", "帕下": "趴下", "帕萨": "趴下",
    "哈萨": "趴下",
    "站起来": "站起", "起立": "站起",
    "恢复站起": "恢复站立", "摔倒恢复": "恢复站立",
}

# 现场录音中 small 和 large-v3-turbo 都曾把句首的“执行”稳定地
# 转写成“直銜/直衔/直旋”。只有当整句仍是完整的“<支持动作>+动作”
# 句式时才恢复这个前缀；不对普通对话做模糊匹配，以免误打开运动安全门。
_ASR_MISHEARD_EXECUTE_ACTION = re.compile(
    r"^(?P<leading>\s*(?:(?:邦邦|bangbang)\s*[,，:：]?\s*)?)"
    r"(?:直銜|直衔|直旋|直行|只想|提醒|行)"
    r"(?=\s*(?:前进|全劲|全进|前劲|前近|前镜|钱进|"
    r"雄徑|雄径|闪电|"
    r"向前|往前|朝前|后退|向后|往后|倒退|"
    r"左转|左傳|左传|左砖|折断|截断|向左转|往左转|"
    r"右转|向右转|往右转|"
    r"左移|右移|停止|停下|急停|趴下|坐下|卧倒|"
    r"站起|站起来|起立|恢复站立|恢复站起|摔倒恢复)"
    r"[^.。！？!?;；]{0,24}?\s*动作"
    r"(?:\s*(?:好[啊呀吧]?|啊|吧|呀|哦|啦))?\s*[.。！？!?;；]*\s*$)",
    re.IGNORECASE,
)

# 动作词的近音恢复只在句首已经有完整“执行”授权词时生效。
# 不会把普通对话中的“全劲/前镜”解释为机器狗动作。
_ASR_MISHEARD_ACTION_AFTER_AUTHORIZATION = re.compile(
    r"^(?P<authorization>\s*(?:(?:邦邦|bangbang)\s*[,，:：]?\s*)?"
    r"执行\s*(?:(?:指令|命令)\s*[:：,，]?\s*)?)"
    r"(?:全劲|全进|前劲|前近|前镜|钱进|雄徑|雄径|闪电)"
    r"(?P<remainder>[^.。！？!?;；]{0,24}(?:\s*动作)?"
    r"(?:\s*(?:好|啊|吧|呀|哦|啦))?\s*[.。！？!?;；]*\s*)$",
    re.IGNORECASE,
)

_ASR_MISHEARD_LEFT_TURN_AFTER_AUTHORIZATION = re.compile(
    r"^(?P<authorization>\s*(?:(?:邦邦|bangbang)\s*[,，:：]?\s*)?"
    r"执行\s*(?:(?:指令|命令)\s*[:：,，]?\s*)?)"
    r"(?:左傳|左传|左砖|折断|截断)"
    r"(?P<remainder>\s*(?:(?:\d+(?:\.\d+)?|"
    r"[零〇一二两三四五六七八九十百点半]+)\s*(?:度|°))?"
    r"\s*动作(?:\s*(?:好[啊呀吧]?|啊|吧|呀|哦|啦))?"
    r"\s*[.。！？!?;；]*\s*)$",
    re.IGNORECASE,
)

_ASR_REPEATED_EXECUTE_ACTION = re.compile(
    r"^(?P<authorization>\s*(?:(?:邦邦|bangbang)\s*[,，:：]?\s*)?"
    r"执行\s*)(?P<action>前进|后退|左转|右转|左移|右移)"
    r"\s*动作(?:\s*(?P=action)\s*动作)+\s*[.。！？!?;；]*\s*$",
    re.IGNORECASE,
)

_CONTROL_TEXT_TRANSLATION = str.maketrans({
    "執": "执",
    "動": "动",
    "兩": "两",
    "後": "后",
    "進": "进",
    "轉": "转",
    "幫": "帮",
    "請": "请",
    "緊": "紧",
    "復": "复",
    "來": "来",
    "點": "点",
    "頓": "顿",
})


def normalize_control_text(text: str) -> str:
    """统一 Whisper 常见字形，并修复已验证的执行口令误识别。"""
    normalized = (text or "").translate(_CONTROL_TEXT_TRANSLATION)
    normalized = _ASR_FIXED_ACTION_FRAME.sub(
        lambda match: (
            f"{match.group('leading')}执行"
            f"{_ASR_FIXED_ACTION_ALIASES.get(match.group('action'), match.group('action'))}"
            f"动作{match.group('ending')}"
        ),
        normalized,
        count=1,
    )
    normalized = _ASR_MISHEARD_EXECUTE_ACTION.sub(
        lambda match: f"{match.group('leading')}执行",
        normalized,
        count=1,
    )
    normalized = _ASR_MISHEARD_ACTION_AFTER_AUTHORIZATION.sub(
        lambda match: (
            f"{match.group('authorization')}前进{match.group('remainder')}"
        ),
        normalized,
        count=1,
    )
    normalized = _ASR_MISHEARD_LEFT_TURN_AFTER_AUTHORIZATION.sub(
        lambda match: (
            f"{match.group('authorization')}左转{match.group('remainder')}"
        ),
        normalized,
        count=1,
    )
    return _ASR_REPEATED_EXECUTE_ACTION.sub(
        lambda match: (
            f"{match.group('authorization')}{match.group('action')}动作"
        ),
        normalized,
        count=1,
    )


def is_fixed_action_command(text: str) -> bool:
    """判断文本是否采用“执行某动作”的封闭式短口令句型。"""
    normalized = normalize_control_text(text).strip()
    normalized = re.sub(
        r"^(?:邦邦|bangbang)\s*[,，:：]?\s*", "", normalized,
        flags=re.IGNORECASE,
    )
    return bool(
        re.fullmatch(
            r"执行\s*(?!指令|命令|动作)"
            r"[^.。！？!?;；]{1,60}?\s*动作"
            r"(?:\s*(?:好[啊呀吧]?|啊|吧|呀|哦|啦))?\s*[.。！？!?;；]*",
            normalized,
            flags=re.IGNORECASE,
        )
        or re.fullmatch(
            r"execute\s+[^.!?;]{1,80}?\s+action\s*[.!?;]*",
            normalized,
            flags=re.IGNORECASE,
        )
    )


_EN_COMMAND_PATTERNS = (
    re.compile(r"\bexecute\s+(?:the\s+)?command\s*[:;,]?\s*(?P<command>.*)", re.IGNORECASE),
    re.compile(r"\bexecute\s+(?:the\s+)?action\s*[:;,]?\s*(?P<command>.*)", re.IGNORECASE),
    re.compile(r"\bexecute\s+(?P<command>[^.!?;]{1,80}?)\s+action(?:\s*[.!?;]|$)", re.IGNORECASE),
)

_ALARM_PATTERNS = (
    re.compile(r"一键报警|立即报警|紧急报警|报警求助|呼叫救援|紧急求助", re.IGNORECASE),
    re.compile(r"\b(?:raise|trigger|sound)\s+(?:the\s+)?alarm\b", re.IGNORECASE),
    re.compile(r"\bemergency\s+alert\b|\bcall\s+for\s+help\b|\bs\.?o\.?s\.?\b", re.IGNORECASE),
)
_STOP_PATTERNS = (
    re.compile(r"^(?:邦邦[,， ]*)?(?:立即|马上)?(?:停止|停下|急停|别动)[。！？!?;； ]*$", re.IGNORECASE),
    re.compile(
        r"^(?:bangbang[, ]*)?(?:stop|emergency\s+stop|halt)(?:\s+now)?[.!?; ]*$",
        re.IGNORECASE,
    ),
)
_META_OR_QUESTION = re.compile(
    r"如果|假如|比如|示例|举例|如何|怎么|怎样|为什么|什么是|是什么|是否|能不能|会不会|"
    r"不要执行|别执行|没有说|不要报警|别报警|\bwhat\s+(?:if|is|does)\b|"
    r"\bhow\s+(?:do|to|can)\b|\bexample\b|\bdo\s+not\s+(?:execute|alarm)\b|"
    r"\bdon't\s+(?:execute|alarm)\b",
    re.IGNORECASE,
)
_NEGATED_MOTION = re.compile(
    r"(?:不要|别|不准|不许)\s*(?:让\s*)?(?:邦邦\s*)?"
    r"(?:前进|后退|左转|右转|移动|走|趴下|站起)|"
    r"\b(?:do\s+not|don't|never)\s+(?:move|go|walk|turn|sit|stand)\b",
    re.IGNORECASE,
)
_REPORTED_OR_PLAYED_SPEECH = re.compile(
    r"(?:听到|听见|听说|电视|视频|影片|录音|广播|手机|别人|有人)"
    r".{0,24}(?:执行|报警|execute|alarm)",
    re.IGNORECASE,
)

ALLOWED_ACTIONS = {
    "forward", "backward", "left", "right", "turn_left", "turn_right",
    "stop", "sit", "stand", "recovery", "visual_guide",
}
MOTION_ACTIONS = {"forward", "backward", "left", "right", "turn_left", "turn_right"}

# 旧版 ASR initial_prompt 中的关键词。Whisper 在静音或噪声下可能把整段提示词
# 当成识别结果返回；其中同时含有运动口令和报警短语，必须在安全门控前识别。
_ASR_PROMPT_ECHO_MARKERS = (
    "机器狗名",
    "执行指令",
    "执行前进动作",
    "一键报警",
    "紧急停止",
    "地面引导线",
    "红绿灯",
    "executecommand",
    "executeforwardaction",
    "emergencyalert",
    "groundguideline",
    "trafficlight",
)

# 当前 faster-whisper 热词配置曾在底噪上被逐项吐出，并随后循环“左转”或
# “执行指令”。这类结果即便声学分数看似可信，也不能打开运动安全门。
_ASR_CONTROL_PROMPT_MARKERS = (
    "执行指令",
    "执行命令",
    "前进",
    "后退",
    "左转",
    "右转",
    "停止",
    "趴下",
    "站起",
)


def is_asr_prompt_echo(text: str) -> bool:
    """Return true when a transcript is an ASR keyword-prompt regurgitation."""
    compact = re.sub(r"[^a-z0-9\u4e00-\u9fff]+", "", (text or "").casefold())
    marker_count = sum(marker in compact for marker in _ASR_PROMPT_ECHO_MARKERS)
    if marker_count >= 3:
        return True
    control_counts = [compact.count(marker) for marker in _ASR_CONTROL_PROMPT_MARKERS]
    distinct_controls = sum(count > 0 for count in control_counts)
    repeated_authorizer = control_counts[0] >= 4 or control_counts[1] >= 4
    return distinct_controls >= 6 or repeated_authorizer


def gate_command(text: str) -> GateResult:
    """只有固定中英文句式能够打开运动命令门。"""
    normalized = normalize_control_text(text).strip()
    if is_asr_prompt_echo(normalized):
        return GateResult(False, reason="asr_prompt_echo")
    if (
        _META_OR_QUESTION.search(normalized)
        or _NEGATED_MOTION.search(normalized)
        or _REPORTED_OR_PLAYED_SPEECH.search(normalized)
        or re.search(r"(?:吗|么|呢)[？?。.!;； ]*$", normalized)
    ):
        return GateResult(False, reason="quoted_or_question")
    authorization_text = re.sub(
        r"^(?:邦邦|bangbang)\s*[,，:：]?\s*", "", normalized, flags=re.IGNORECASE
    )
    for pattern in (*_ZH_COMMAND_PATTERNS, *_EN_COMMAND_PATTERNS):
        match = pattern.fullmatch(authorization_text)
        if match:
            command = match.group("command").strip(" ：:，,。.!！?？;；")
            command = re.sub(r"^(?:邦邦|bangbang)[,， ]*", "", command, flags=re.IGNORECASE)
            return GateResult(True, command, "explicit_execute_phrase")
    return GateResult(False)


def is_alarm_request(text: str) -> bool:
    normalized = normalize_control_text(text).strip()
    if (
        is_asr_prompt_echo(normalized)
        or _META_OR_QUESTION.search(normalized)
        or _REPORTED_OR_PLAYED_SPEECH.search(normalized)
        or re.search(r"(?:吗|么|呢)[？?。.!;； ]*$", normalized)
    ):
        return False
    return any(pattern.search(normalized) for pattern in _ALARM_PATTERNS)


def is_emergency_stop(text: str) -> bool:
    normalized = normalize_control_text(text).strip()
    return any(pattern.fullmatch(normalized) for pattern in _STOP_PATTERNS)


def parse_deterministic_command(command_text: str) -> list[dict[str, float | str]] | None:
    """解析常用中英文动作，不让底盘控制依赖通用大模型。

    这里只接受很窄的动作语法；任何不能完整理解的片段都返回 ``None``，由上层
    决定是否交给 Qwen。调用方必须先通过 :func:`gate_command`，本函数本身不是
    授权门。
    """
    normalized = normalize_control_text(command_text).strip(" ：:，,。.!！?？;；")
    if not normalized:
        return None

    # 视觉引导通常包含“沿线前进并识别红绿灯”，不能按“并”拆成两个动作。
    lowered = normalized.casefold()
    if (
        any(marker in normalized for marker in ("引导线", "导引线", "盲道", "红绿灯"))
        or re.search(r"\b(?:guide\s*line|guidance\s*line|traffic\s*light)\b", lowered)
    ):
        return [{
            "action": "visual_guide",
            "speed": 0.18,
            "duration": min(60.0, config.VISUAL_GUIDE_TIMEOUT),
            "angle": 0.0,
        }]

    chunks = [
        chunk.strip(" ：:，,。.!！?？;；")
        for chunk in re.split(
            r"(?:\s*(?:然后|接着|随后|之后|并且|再)\s*|\s*(?:,|，|;|；)\s*|"
            r"\s+and\s+then\s+|\s+then\s+)",
            normalized,
            flags=re.IGNORECASE,
        )
        if chunk.strip(" ：:，,。.!！?？;；")
    ]
    if not chunks or len(chunks) > 4:
        return None

    actions: list[dict[str, float | str]] = []
    for chunk in chunks:
        parsed = _parse_one_deterministic_action(chunk)
        if parsed is None:
            return None
        actions.append(parsed)
    return actions or None


def _parse_one_deterministic_action(text: str) -> dict[str, float | str] | None:
    lowered = text.casefold().strip()
    if _NEGATED_MOTION.search(lowered):
        return None
    # 礼貌词和机器人称呼不携带运动语义，去掉后要求剩余文本命中明确动作。
    simplified = re.sub(
        r"^(?:请|请你|麻烦|麻烦你|邦邦|bangbang|please|could\s+you|can\s+you)\s*[,，]?\s*",
        "",
        lowered,
        flags=re.IGNORECASE,
    ).strip()

    if re.fullmatch(r"(?:停止|停下|急停|别动|stop|halt|emergency\s+stop)", simplified):
        return _raw_action("stop")
    if re.fullmatch(r"(?:趴下|坐下|卧倒|sit|lie\s+down)", simplified):
        return _raw_action("sit")
    if re.fullmatch(r"(?:站起|站起来|起立|stand|stand\s+up)", simplified):
        return _raw_action("stand")
    if re.fullmatch(r"(?:恢复站立|恢复站起|摔倒恢复|recovery|recover|recovery\s+stand)", simplified):
        return _raw_action("recovery")

    turn_left = bool(re.search(r"左转|向左转|往左转|turn\s+left|left\s+turn", simplified))
    turn_right = bool(re.search(r"右转|向右转|往右转|turn\s+right|right\s+turn", simplified))
    if turn_left or turn_right:
        if turn_left and turn_right:
            return None
        angle = _extract_quantity(simplified, ("度", "°", "degree", "degrees"))
        angle = 90.0 if angle is None else angle
        return _raw_action(
            "turn_left" if turn_left else "turn_right",
            speed=config.DEFAULT_TURN_SPEED,
            duration=0.0,
            angle=angle,
        )

    directions = []
    direction_patterns = {
        "forward": r"前进|向前|往前|朝前|move\s+forward|go\s+forward|walk\s+forward|forward",
        "backward": r"后退|向后|往后|倒退|move\s+backward|go\s+backward|backward|back\s+up",
        "left": r"左移|向左移动|往左移动|向左平移|move\s+left|go\s+left|step\s+left",
        "right": r"右移|向右移动|往右移动|向右平移|move\s+right|go\s+right|step\s+right",
    }
    for action, pattern in direction_patterns.items():
        if re.search(pattern, simplified):
            directions.append(action)
    if len(directions) != 1:
        return None

    speed = min(max(config.DEFAULT_LINEAR_SPEED, 0.10), config.MAX_LINEAR_SPEED)
    distance = _extract_quantity(
        simplified,
        ("米", "公尺", "m", "meter", "meters", "metre", "metres"),
    )
    if distance is None:
        steps = _extract_quantity(simplified, ("步", "step", "steps"))
        distance = (
            1.0 if steps is None else steps * config.DEFAULT_STEP_LENGTH_M
        )
    if distance <= 0.0:
        return None
    return _raw_action(
        directions[0],
        speed=speed,
        duration=distance / speed,
        angle=0.0,
    )


def _raw_action(name: str, speed: float = 0.0, duration: float = 0.0,
                angle: float = 0.0) -> dict[str, float | str]:
    return {
        "action": name,
        "speed": float(speed),
        "duration": float(duration),
        "angle": float(angle),
    }


_EN_SMALL_NUMBER = (
    r"zero|one|two|three|four|five|six|seven|eight|nine|ten|eleven|twelve|"
    r"thirteen|fourteen|fifteen|sixteen|seventeen|eighteen|nineteen"
)
_EN_TENS_NUMBER = r"twenty|thirty|forty|fifty|sixty|seventy|eighty|ninety"
_EN_NUMBER = (
    rf"(?:(?:one|two)\s+hundred(?:\s+and)?(?:\s+(?:{_EN_TENS_NUMBER})"
    rf"(?:[-\s](?:one|two|three|four|five|six|seven|eight|nine))?|"
    rf"\s+(?:{_EN_SMALL_NUMBER}))?|(?:{_EN_TENS_NUMBER})"
    rf"(?:[-\s](?:one|two|three|four|five|six|seven|eight|nine))?|"
    rf"(?:{_EN_SMALL_NUMBER})|half)"
)
_NUMBER_TOKEN = (
    rf"(?:\d+(?:\.\d+)?|[零〇一二两三四五六七八九十百点半]+|{_EN_NUMBER})"
)


def _extract_quantity(text: str, units: tuple[str, ...]) -> float | None:
    unit_pattern = "|".join(re.escape(unit) for unit in sorted(units, key=len, reverse=True))
    match = re.search(
        rf"(?P<number>{_NUMBER_TOKEN})\s*(?:{unit_pattern})",
        text,
        flags=re.IGNORECASE,
    )
    if not match:
        return None
    try:
        return _parse_spoken_number(match.group("number"))
    except ValueError:
        return None


def _parse_spoken_number(token: str) -> float:
    token = token.strip().casefold()
    if re.fullmatch(r"\d+(?:\.\d+)?", token):
        return float(token)
    if token == "半":
        return 0.5
    if re.fullmatch(r"[a-z -]+", token):
        english_values = {
            "zero": 0, "one": 1, "two": 2, "three": 3, "four": 4,
            "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9,
            "ten": 10, "eleven": 11, "twelve": 12, "thirteen": 13,
            "fourteen": 14, "fifteen": 15, "sixteen": 16,
            "seventeen": 17, "eighteen": 18, "nineteen": 19,
            "twenty": 20, "thirty": 30, "forty": 40, "fifty": 50,
            "sixty": 60, "seventy": 70, "eighty": 80, "ninety": 90,
        }
        if token == "half":
            return 0.5
        words = [word for word in re.split(r"[-\s]+", token) if word != "and"]
        if not words or any(word not in english_values and word != "hundred" for word in words):
            raise ValueError(token)
        total = 0
        current = 0
        for word in words:
            if word == "hundred":
                current = (current or 1) * 100
            else:
                current += english_values[word]
        total += current
        return float(total)
    digits = {"零": 0, "〇": 0, "一": 1, "二": 2, "两": 2, "三": 3,
              "四": 4, "五": 5, "六": 6, "七": 7, "八": 8, "九": 9}
    if "点" in token:
        integer, fractional = token.split("点", 1)
        if not fractional or any(char not in digits for char in fractional):
            raise ValueError(token)
        integer_value = _parse_chinese_integer(integer) if integer else 0
        fraction_value = int("".join(str(digits[char]) for char in fractional))
        return integer_value + fraction_value / (10 ** len(fractional))
    return float(_parse_chinese_integer(token))


def _parse_chinese_integer(token: str) -> int:
    digits = {"零": 0, "〇": 0, "一": 1, "二": 2, "两": 2, "三": 3,
              "四": 4, "五": 5, "六": 6, "七": 7, "八": 8, "九": 9}
    units = {"十": 10, "百": 100}
    if not token:
        return 0
    total = 0
    current = 0
    for char in token:
        if char in digits:
            current = digits[char]
        elif char in units:
            total += (current or 1) * units[char]
            current = 0
        else:
            raise ValueError(token)
    return total + current


def validate_actions(raw_actions: Any) -> tuple[list[SafeAction], list[str]]:
    """将解析结果收敛为唯一的最终执行参数。

    ``SafeAction.duration`` 是执行端直接采用的租约时间。距离/角度补偿、限速和
    总时长都在这里完成，ROS/SDK 执行端不得再次乘系数或重算 duration。
    """
    if not isinstance(raw_actions, list):
        return [], ["actions 不是数组"]

    safe: list[SafeAction] = []
    warnings: list[str] = []
    total_duration = 0.0
    for index, item in enumerate(raw_actions[:4]):
        if not isinstance(item, dict):
            warnings.append(f"第 {index + 1} 个动作格式无效")
            continue
        name = str(item.get("action", "")).strip().lower()
        if name not in ALLOWED_ACTIONS:
            warnings.append(f"已拒绝未知动作 {name or '?'}")
            continue
        try:
            speed = _finite_float(item.get("speed", 0.3 if name in MOTION_ACTIONS else 0.0))
            duration = _finite_float(item.get("duration", 2.0 if name in MOTION_ACTIONS else 0.0))
            angle = _finite_float(item.get("angle", 0.0))
        except (TypeError, ValueError):
            warnings.append(f"动作 {name} 的参数不是有效数字")
            continue

        if name == "visual_guide":
            speed = min(max(abs(speed or 0.18), 0.10), min(config.MAX_LINEAR_SPEED, 0.25))
            duration = min(max(duration or 60.0, 5.0), config.VISUAL_GUIDE_TIMEOUT)
            angle = 0.0
        elif name in MOTION_ACTIONS:
            max_speed = (
                config.MAX_TURN_SPEED
                if name.startswith("turn_")
                else config.MAX_LINEAR_SPEED
            )
            original_speed = max(abs(speed), 0.10)
            speed = min(original_speed, max_speed)
            angle = min(max(abs(angle), 0.0), 180.0)
            if name.startswith("turn_") and angle > 0.0:
                duration = (
                    math.radians(angle) / speed * config.TURN_TIME_MULTIPLIER
                )
            elif name.startswith("turn_"):
                duration = max(duration, 0.10)
            else:
                # 模型的 speed 可能被限幅。先还原目标距离再用最终速度计算，
                # 防止限速后又乘补偿导致实际距离悄悄改变。
                target_distance = original_speed * max(duration, 0.10)
                duration = (
                    target_distance / speed * config.LINEAR_TIME_MULTIPLIER
                )
                angle = 0.0
            if duration > config.MAX_ACTION_DURATION:
                warnings.append(f"动作 {name} 时长已限制为 {config.MAX_ACTION_DURATION:.1f}s")
            duration = min(max(duration, 0.10), config.MAX_ACTION_DURATION)
        else:
            speed = 0.0
            duration = 0.0
            angle = 0.0

        if name != "visual_guide":
            remaining = config.MAX_TOTAL_DURATION - total_duration
            if remaining < 0.10:
                warnings.append("动作总时长超过安全上限，后续动作已丢弃")
                break
            if duration > remaining:
                warnings.append("动作总时长超过安全上限，当前动作已截短")
            duration = min(duration, remaining)
            total_duration += duration
        safe.append(SafeAction(name, speed, duration, angle))

    if len(raw_actions) > 4:
        warnings.append("动作数量超过 4，已截断")
    return safe, warnings


def _finite_float(value: Any) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("non-finite")
    return number
