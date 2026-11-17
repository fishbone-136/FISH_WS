"""faster-whisper 独立进程：stdin/stdout 使用 JSON Lines 协议。"""

from __future__ import annotations

import json
import math
import re
import sys
import time
from functools import lru_cache

from . import config
from .command_safety import (
    gate_command,
    is_asr_prompt_echo,
    normalize_control_text,
    parse_deterministic_command,
)
from .text_motion import parse_relative_command_body


def _send(payload: dict) -> None:
    try:
        print(json.dumps(payload, ensure_ascii=False), flush=True)
    except BrokenPipeError:
        # The UI can close the pipe while a long rescue decode is still running.
        # In that case the worker has no consumer and should exit quietly instead
        # of trying to report the same BrokenPipeError through the broken pipe.
        raise SystemExit(0)


def _best_cpu_compute_type() -> str:
    """Choose a CPU type supported by the installed CTranslate2 build."""
    try:
        import ctranslate2

        supported = ctranslate2.get_supported_compute_types("cpu")
        return "int8" if "int8" in supported else "float32"
    except Exception:
        # Official CPU wheels normally support int8. Model initialization will
        # still produce a useful error if this assumption does not hold.
        return "int8"


def _has_credible_speech(segments) -> bool:
    return any(
        segment.avg_logprob >= config.WHISPER_MIN_AVG_LOGPROB
        and segment.no_speech_prob <= config.WHISPER_MAX_NO_SPEECH_PROB
        for segment in segments
    )


def _quality_stats(segments) -> tuple[float, float]:
    """整句按 token 数加权；不能用一个好片段掩盖其余片段的低置信度。"""
    weighted = [(segment, max(1, len(getattr(segment, "tokens", ()))))
                for segment in segments]
    if not weighted:
        return -99.0, 1.0
    return (sum(segment.avg_logprob * weight for segment, weight in weighted)
            / sum(weight for _, weight in weighted),
            max(segment.no_speech_prob for segment, _ in weighted))


@lru_cache(maxsize=4)
def _load_rescue_vocabulary(path: str) -> tuple:
    if not path:
        return ()
    try:
        with open(path, encoding="utf-8") as stream:
            entries = json.load(stream)
        if not isinstance(entries, list):
            raise ValueError("词表必须是数组")
        result = []
        for entry in entries:
            term, language, aliases = entry["term"], entry["language"], entry.get("aliases", [])
            if (not isinstance(term, str) or not 1 <= len(term) <= 40
                    or language not in {"zh", "en"} or not isinstance(aliases, list)
                    or any(not isinstance(alias, str) or not 1 <= len(alias) <= 40
                           for alias in aliases)):
                raise ValueError("词表条目格式无效")
            result.append((term, language, tuple(aliases)))
        return tuple(result)
    except (OSError, ValueError, TypeError, KeyError) as exc:
        print(f"ASR 复核词表不可用，继续声学复核: {exc}", file=sys.stderr, flush=True)
        return ()


def _contains_term(text: str, term: str, language: str) -> bool:
    if language == "en":
        return bool(re.search(r"(?<!\w)" + re.escape(term) + r"(?!\w)", text, re.I))
    return term in text


def _rescue_vocabulary_matches(text: str, language: str) -> tuple:
    # 词表只决定本轮解码提示；绝不在最终文本里直接替换同音词。
    matches = []
    for term, lang, aliases in _load_rescue_vocabulary(config.WHISPER_RESCUE_VOCABULARY):
        if lang == language and any(_contains_term(text, value, lang)
                                   for value in (term, *aliases)):
            matches.append((term, lang, not _contains_term(text, term, lang)))
        if len(matches) >= 4:
            break
    return tuple(matches)


def _rescue_language(info, command_rescue: bool = False) -> str | None:
    if config.WHISPER_LANGUAGE != "auto":
        return config.WHISPER_LANGUAGE
    language = getattr(info, "language", "")
    if language in {"zh", "en"} and (
        command_rescue or getattr(info, "language_probability", 0.0)
        >= config.WHISPER_RESCUE_LANGUAGE_LOCK_PROB
    ):
        return language
    return None


def _rescue_quality_rejection(segments, info, expected_language: str | None) -> str:
    avg_logprob, no_speech_prob = _quality_stats(segments)
    if (not segments or not math.isfinite(avg_logprob) or not math.isfinite(no_speech_prob)
            or any(not math.isfinite(s.avg_logprob) or not math.isfinite(s.no_speech_prob)
                   or s.avg_logprob < config.WHISPER_MIN_AVG_LOGPROB
                   or s.no_speech_prob > config.WHISPER_RESCUE_ACCEPT_MAX_NO_SPEECH_PROB
                   for s in segments)
            or avg_logprob < config.WHISPER_RESCUE_ACCEPT_MIN_AVG_LOGPROB
            or no_speech_prob > config.WHISPER_RESCUE_ACCEPT_MAX_NO_SPEECH_PROB):
        return "rescue_low_confidence"
    if expected_language and getattr(info, "language", "") != expected_language:
        return "rescue_language_mismatch"
    return ""


def _review_rescue(primary_text, rescue_text, segments, info,
                   expected_language=None, vocabulary_matches=()) -> tuple[str, str, str]:
    """保留动作一致性检查，再检查声学质量与本轮专名是否得到确认。"""
    selected, decision = _choose_rescue_text(primary_text, rescue_text)
    if decision == "command_disagreement":
        return "", decision, decision
    rejection = _rescue_quality_rejection(segments, info, expected_language)
    if rejection:
        return "", "rejected_rescue", rejection
    if decision.startswith("kept_primary_"):
        return selected, decision, ""
    # 热词只是解码提示。若整句退化成只输出提示词，不能把提示回声当成纠正。
    hint_text = "".join(term for term, _, _ in vocabulary_matches)
    primary_compact = re.sub(r"[^\w\u4e00-\u9fff]+", "", primary_text.casefold())
    rescue_compact = re.sub(r"[^\w\u4e00-\u9fff]+", "", rescue_text.casefold())
    hint_compact = re.sub(r"[^\w\u4e00-\u9fff]+", "", hint_text.casefold())
    if (hint_text and rescue_compact == hint_compact
            and len(primary_compact) > len(hint_compact) + 2):
        return "", "rejected_rescue", "rescue_prompt_echo"
    for term, language, was_alias in vocabulary_matches:
        if not _contains_term(rescue_text, term, language):
            if not was_alias:
                return primary_text, "kept_primary_vocabulary", ""
            return "", "rejected_rescue", "rescue_unresolved_vocabulary"
    return selected, decision, ""


_COMMON_SUBTITLE_HALLUCINATIONS = (
    "thankyouforwatching",
    "thanksforwatching",
    "thankyouforyourattention",
    "pleasesubscribe",
    "subtitlesby",
    "translatedby",
    "谢谢观看",
    "感谢观看",
    "感谢您的观看",
    "请订阅",
    "请不吝点赞订阅转发打赏支持明镜与点点栏目",
    "請不吝點贊訂閱轉發打賞支持明鏡與點點欄目",
    "字幕制作",
    "字幕by索兰娅",
    "字幕by索兰亚",
    "字幕by索蘭婭",
)
_COMMON_SHORT_HALLUCINATIONS = {
    "thankyou",
    "thanks",
    "thankyouverymuch",
    "谢谢",
    "多谢",
}

# 这些长句来自 Whisper 训练数据中的视频片尾字幕，可能被接在真实转写之后。
# 与上面的可疑文本检测分开处理，是为了只裁掉已知尾巴、保留用户前面说的话。
_TRAILING_SUBTITLE_HALLUCINATIONS = tuple(sorted({
    # small 模型在本次现场录音中稳定附加的 B 站字幕署名幻觉。
    "字幕by索兰娅",
    "字幕by索兰亚",
    "字幕by索蘭婭",
    "字幕bysolanya",
    "请不吝点赞订阅转发打赏支持明镜与点点",
    "请不吝点赞订阅转发打赏支持明镜与点点栏目",
    "请不吝点赞订阅转发打赏支持明镜与点点节目",
    "请不吝点赞订阅转发打赏支持明镜与点点频道",
    "請不吝點贊訂閱轉發打賞支持明鏡與點點",
    "請不吝點贊訂閱轉發打賞支持明鏡與點點欄目",
    "請不吝點贊訂閱轉發打賞支持明鏡與點點節目",
    "請不吝點贊訂閱轉發打賞支持明鏡與點點頻道",
}, key=len, reverse=True))

# “谢谢大家”本身是正常中文，不能像上面的字幕署名那样
# 全局删除。它只在紧跟完整的“执行…动作”转写时视为 Whisper 尾巴。
_COMMAND_TRAILING_HALLUCINATIONS = tuple(sorted({
    "谢谢大家",
    "感谢大家",
    "感谢观看",
}, key=len, reverse=True))


def _looks_like_execute_action_frame(text: str) -> bool:
    compact = re.sub(r"[^a-z0-9\u4e00-\u9fff]+", "", (text or "").casefold())
    return bool(re.fullmatch(
        r"(?:邦邦|bangbang)?(?:执行|直銜|直衔|直旋|直行|只想|提醒|行)"
        r".{1,48}动作",
        compact,
        flags=re.IGNORECASE,
    ))


def _strip_trailing_hallucination(text: str) -> tuple[str, str]:
    """Remove a known subtitle hallucination suffix without losing real speech."""
    original = (text or "").strip()
    compact_chars: list[str] = []
    source_offsets: list[int] = []
    for match in re.finditer(r"[\w\u4e00-\u9fff]", original.casefold()):
        compact_chars.append(match.group(0))
        source_offsets.append(match.start())
    compact = "".join(compact_chars)

    for phrase in _TRAILING_SUBTITLE_HALLUCINATIONS:
        if not compact.endswith(phrase):
            continue
        compact_start = len(compact) - len(phrase)
        source_start = source_offsets[compact_start]
        prefix = original[:source_start].rstrip(" \t\r\n，,。.!！?？；;：:")
        return prefix, original[source_start:].strip()

    for phrase in _COMMAND_TRAILING_HALLUCINATIONS:
        if not compact.endswith(phrase):
            continue
        compact_start = len(compact) - len(phrase)
        source_start = source_offsets[compact_start]
        prefix = original[:source_start].rstrip(" \t\r\n，,。.!！?？；;：:")
        if _looks_like_execute_action_frame(prefix):
            return prefix, original[source_start:].strip()
    return original, ""


def _sanitize_transcript(text: str, language: str) -> tuple[str, str, str]:
    """Normalize ASR text and reject prompt echoes or corrupt decoder output."""
    cleaned = normalize_control_text(text)
    cleaned = _normalize_robot_name(cleaned, language)
    cleaned, stripped_hallucination = _strip_trailing_hallucination(cleaned)
    # 去掉幻觉尾巴后，句子才重新成为完整的“执行…动作”结构，
    # 因此在这里再做一次受限动作词恢复。
    cleaned = normalize_control_text(cleaned)
    if is_asr_prompt_echo(cleaned):
        return "", stripped_hallucination, "prompt_echo"
    if _has_decoder_repetition(cleaned):
        return "", stripped_hallucination, "decoder_repetition"
    if "\ufffd" in cleaned:
        return "", stripped_hallucination, "invalid_unicode"
    return cleaned, stripped_hallucination, ""


_CONTROL_PHRASES = (
    "执行指令", "执行命令", "前进", "后退", "左转", "右转", "停止", "趴下", "站起",
    "executecommand", "forward", "backward", "turnleft", "turnright", "stop", "sit", "stand",
)


def _has_decoder_repetition(text: str) -> bool:
    """Reject decoding loops while retaining ordinary repeated natural speech.

    Whisper may assign acceptable acoustic scores to a token loop, especially
    when a hotword prompt is echoed over noise. Movement transcripts are treated
    conservatively: three adjacent repetitions or one control phrase repeated
    four times cannot describe one supported command.
    """
    normalized = normalize_control_text(text).casefold()
    tokens = re.findall(
        r"[a-z]+(?:'[a-z]+)?|\d+(?:\.\d+)?|[\u4e00-\u9fff]{1,12}",
        normalized,
    )
    if len(tokens) >= 3:
        run_length = 1
        for previous, current in zip(tokens, tokens[1:]):
            if current == previous:
                run_length += 1
                if run_length >= 3:
                    return True
            else:
                run_length = 1

    if len(tokens) >= 6:
        largest_count = max((tokens.count(token) for token in set(tokens)), default=0)
        if largest_count >= 4 and largest_count / len(tokens) >= 0.35:
            return True

    compact = re.sub(r"[^a-z0-9\u4e00-\u9fff]+", "", normalized)
    for phrase in _CONTROL_PHRASES:
        if compact.count(phrase) >= 4:
            return True
    return False


def _is_suspicious_transcript(text: str) -> bool:
    """Detect frequent Whisper subtitle-hallucination candidates."""
    compact = re.sub(r"[^\w\u4e00-\u9fff]+", "", text.casefold())
    return (
        compact in _COMMON_SHORT_HALLUCINATIONS
        or any(phrase in compact for phrase in _COMMON_SUBTITLE_HALLUCINATIONS)
    )


def _should_filter_suspicious(segments, text: str) -> bool:
    """Reject subtitle-like text only when its acoustic confidence is also weak."""
    avg_logprob, _ = _quality_stats(segments)
    return (
        _is_suspicious_transcript(text)
        and avg_logprob < config.WHISPER_SUSPICIOUS_MIN_AVG_LOGPROB
    )


def _normalize_robot_name(text: str, language: str) -> str:
    """Keep the product name readable in the detected conversation language."""
    if language == "zh":
        text = re.sub(r"bang\s*bang", config.ROBOT_NAME, text, flags=re.IGNORECASE)
        # 只修正句首作为称呼使用的常见同音误写；“帮帮我”等正常表达不改动。
        return re.sub(
            r"^(?:帮帮|包包|棒棒|磅磅|邦帮|斑斑|东东)"
            r"(?=[，,。.!！?\s]|请|你|介绍|执行|命令|停止|向|前进|后退|左转|右转)",
            config.ROBOT_NAME,
            text,
        )
    if language == "en":
        return text.replace(config.ROBOT_NAME, "Bangbang")
    return text


def _is_fast_safety_stop(text: str) -> bool:
    compact = re.sub(r"[^a-z\u4e00-\u9fff]+", "", text.casefold())
    for name in (config.ROBOT_NAME, "bangbang"):
        if compact.startswith(name.casefold()):
            compact = compact[len(name):]
            break
    return compact in {
        "停",
        "停止",
        "立即停止",
        "立刻停止",
        "马上停止",
        "紧急停止",
        "stop",
        "stopnow",
        "emergencystop",
    }


def _looks_like_command_candidate(text: str) -> bool:
    """识别疑似被 small 听错的执行口令，只用于触发 turbo 复核。"""
    compact = re.sub(
        r"[^a-z0-9\u4e00-\u9fff]+", "", normalize_control_text(text).casefold()
    )
    for name in (config.ROBOT_NAME.casefold(), "bangbang"):
        if compact.startswith(name):
            compact = compact[len(name):]
            break
    command_prefixes = (
        "执行", "实行", "直銜", "直衔", "直旋", "直行", "只想", "提醒",
        "尺寸使用", "execute",
    )
    if not compact.startswith(command_prefixes):
        return False
    has_action_word = any(marker in compact for marker in (
        "前", "后", "左", "右", "走", "移动", "转", "停止", "停下", "趴下",
        "站起", "forward", "backward", "move", "turn", "stop", "sit", "stand",
    ))
    # “帕萨顿作”“右转正左”这类结果连动作词或句尾都可能错，但整体仍保留了
    # 很短的执行口令外形。这里只触发二次识别，不直接授予动作权限。
    has_action_ending = compact.endswith((
        "动作", "顿座", "顿作", "正左", "action",
    ))
    return has_action_word or (has_action_ending and len(compact) <= 24)


def _command_rescue_prompt(language: str) -> str:
    """Return a short vocabulary prompt only for a credible command candidate."""
    if language == "en":
        return (
            "Robot voice commands: execute forward action; execute backward action; "
            "execute left turn action; execute right turn action; execute stop action; "
            "execute sit action; execute stand action."
        )
    return (
        "机器狗语音口令：执行前进动作；执行后退动作；执行左转动作；"
        "执行右转动作；执行停止动作；执行趴下动作；执行站起动作。"
    )


def _rescue_reason(segments, info, text: str) -> str:
    """Explain why a credible primary-model result needs second-model verification."""
    if not text or not _has_credible_speech(segments):
        # Do not spend several seconds running a second model on silence/noise.
        return ""
    avg_logprob, no_speech_prob = _quality_stats(segments)
    weak_segment = any(s.avg_logprob < config.WHISPER_MIN_AVG_LOGPROB for s in segments)
    if _is_suspicious_transcript(text):
        return "疑似字幕幻觉"
    if _is_fast_safety_stop(text):
        # 急停不能为了二级识别增加数秒延迟；运动层仍会执行独立安全校验。
        return ""
    if gate_command(text).authorized:
        # 清晰的固定口令只跑一次 small；边界声学结果才交给 turbo。之前所有
        # 运动口令固定再跑一次 large-v3，是用户感知到约 10 秒 ASR 延迟的主因。
        language_probability = getattr(info, "language_probability", 0.0)
        if (
            avg_logprob < config.WHISPER_COMMAND_RESCUE_MIN_AVG_LOGPROB
            or no_speech_prob > config.WHISPER_COMMAND_RESCUE_MAX_NO_SPEECH_PROB
            or language_probability < config.WHISPER_COMMAND_RESCUE_MIN_LANGUAGE_PROB
            or weak_segment
        ):
            return "运动指令声学置信度复核"
        return ""
    if _looks_like_command_candidate(text):
        # 例如现场出现的“执行秘密，校前走两步”。它不能直接打开安全门，但需要
        # 让 turbo 尝试恢复成“执行命令，向前走两步”。复核后仍须重新通过门控。
        return "疑似运动口令复核"
    if any(was_alias for _, _, was_alias in _rescue_vocabulary_matches(
        text, getattr(info, "language", "")
    )):
        return "专名候选复核"
    if weak_segment:
        return "片段置信度偏低"
    if avg_logprob < config.WHISPER_RESCUE_MIN_AVG_LOGPROB:
        return "语音置信度偏低"
    if no_speech_prob > config.WHISPER_RESCUE_MAX_NO_SPEECH_PROB:
        return "静音概率偏高"
    # Very short commands such as “停” or “stop” naturally have unstable language
    # probabilities and must remain on the fast path.
    compact_length = len(re.sub(r"\s+", "", text))
    if (
        compact_length >= 8
        and getattr(info, "language_probability", 0.0)
        < config.WHISPER_RESCUE_MIN_LANGUAGE_PROB
    ):
        return "语言判断置信度偏低"
    return ""


def _deterministic_command_signature(text: str) -> tuple | None:
    """返回可确定执行口令的安全签名，用于比较两个 ASR 模型的结果。"""
    gate = gate_command(text)
    normalized = normalize_control_text(text).strip()
    # Comparison alone may inspect an explicit candidate whose second clause
    # is damaged (“执前进三米”). This never authorizes execution. Otherwise two
    # models can disagree on 8m/3m simply because neither passes the old gate.
    candidate = re.sub(r"^(?:邦邦|bangbang)\s*[,，:：]?\s*", "", normalized,
                       flags=re.IGNORECASE)
    if not gate.authorized and not candidate.startswith("执行"):
        return None
    body = gate.command_text if gate.authorized else candidate[2:]
    try:
        strict = parse_relative_command_body(body)
    except ValueError:
        strict = None
    if strict is not None:
        actions = [{"action": a.action, "speed": a.speed,
                    "duration": (a.duration/config.LINEAR_TIME_MULTIPLIER
                                 if a.action in ("forward", "backward", "left", "right") else 0.),
                    "angle": a.angle} for a in strict[0]]
        merged = []
        for action in actions:
            if (merged and action["action"].startswith("turn_")
                    and merged[-1]["action"] == action["action"]):
                merged[-1]["angle"] += action["angle"]
            else:
                merged.append(action)
        actions = merged
    else:
        actions = parse_deterministic_command(body)
    if not actions:
        return None
    return tuple(
        (
            str(action.get("action", "")),
            round(float(action.get("speed", 0.0)), 4),
            round(float(action.get("duration", 0.0)), 4),
            round(float(action.get("angle", 0.0)), 4),
        )
        for action in actions
    )


def _choose_rescue_text(primary_text: str, rescue_text: str) -> tuple[str, str]:
    """选择复核文本，防止高精度模型把已正确的口令覆盖成幻觉。

    返回 ``(文本, 决策)``。两个模型都解析为物理动作但方向或
    参数不一致时拒绝执行，不在安全关键信息上猜测。
    """
    primary_gate = gate_command(primary_text)
    rescue_gate = gate_command(rescue_text)
    if primary_gate.reason == "quoted_or_question" and rescue_gate.authorized:
        return primary_text, "kept_primary_nonimperative"
    primary_signature = _deterministic_command_signature(primary_text)
    rescue_signature = _deterministic_command_signature(rescue_text)
    primary_authorized = primary_gate.authorized
    rescue_authorized = rescue_gate.authorized
    if (primary_signature is not None and rescue_signature is not None
            and rescue_signature != primary_signature):
        return "", "command_disagreement"
    if primary_authorized:
        # 主模型已经听到完整授权句式时，复核的普通对话或乱码
        # 没有资格覆盖它。这正是现场“执行前进”变成“直行雄徑”的原因。
        if not rescue_authorized or (
            primary_signature is not None and rescue_signature is None
        ):
            return primary_text, "kept_primary_command"
        if (
            primary_signature is not None
            and rescue_signature is not None
            and rescue_signature != primary_signature
        ):
            return "", "command_disagreement"
    return rescue_text, "applied_rescue"


def _select_runtime() -> tuple[str, str]:
    device = config.WHISPER_DEVICE
    compute = config.WHISPER_COMPUTE_TYPE
    if device == "auto":
        try:
            import ctranslate2
            device = "cuda" if ctranslate2.get_cuda_device_count() > 0 else "cpu"
        except Exception:
            device = "cpu"
    if compute == "auto":
        compute = "float16" if device == "cuda" else _best_cpu_compute_type()
    return device, compute


def main() -> int:
    try:
        from faster_whisper import WhisperModel

        device, compute = _select_runtime()
        active_model = config.WHISPER_MODEL
        try:
            model = WhisperModel(
                active_model,
                device=device,
                compute_type=compute,
                local_files_only=config.WHISPER_LOCAL_FILES_ONLY,
            )
        except Exception as cuda_error:
            if device != "cuda":
                raise
            # GPU 临时不可用时用较小的本地模型回退，避免 large-v3 在 CPU 上长时间阻塞。
            device, compute = "cpu", _best_cpu_compute_type()
            active_model = config.WHISPER_CPU_FALLBACK_MODEL or config.WHISPER_MODEL
            print(
                json.dumps({
                    "status": "warning",
                    "warning": f"CUDA 初始化失败，回退 CPU: {cuda_error}",
                }, ensure_ascii=False),
                file=sys.stderr,
                flush=True,
            )
            model = WhisperModel(
                active_model,
                device=device,
                compute_type=compute,
                local_files_only=config.WHISPER_LOCAL_FILES_ONLY,
            )
        rescue_model = None
        rescue_disabled = False
        rescue_preload_seconds = 0.0
        rescue_allowed = bool(
            device == "cuda"
            and config.WHISPER_RESCUE_MODEL
            and config.WHISPER_RESCUE_MODEL != active_model
        )
        if rescue_allowed and config.WHISPER_PRELOAD_RESCUE:
            try:
                preload_started = time.monotonic()
                print(
                    f"ASR 正在预载复核模型: {config.WHISPER_RESCUE_MODEL}",
                    file=sys.stderr,
                    flush=True,
                )
                rescue_model = WhisperModel(
                    config.WHISPER_RESCUE_MODEL,
                    device=device,
                    compute_type=compute,
                    local_files_only=config.WHISPER_LOCAL_FILES_ONLY,
                )
                rescue_preload_seconds = time.monotonic() - preload_started
            except Exception as rescue_error:
                rescue_disabled = True
                print(
                    f"ASR 复核模型预载失败，继续使用主模型: {rescue_error}",
                    file=sys.stderr,
                    flush=True,
                )
        _send({
            "status": "ready",
            "model": active_model,
            "device": device,
            "compute_type": compute,
            "rescue_model": config.WHISPER_RESCUE_MODEL if rescue_model else "",
            "rescue_preload_seconds": round(rescue_preload_seconds, 3),
        })
    except Exception as exc:
        _send({"status": "error", "error": f"faster-whisper 初始化失败: {exc}"})
        return 1

    def transcribe_once(
        active_whisper_model,
        wav_path: str,
        beam_size: int,
        best_of: int,
        *,
        language_override: str | None = None,
        initial_prompt: str | None = None,
        hotwords_override: str | None = None,
    ):
        language = language_override
        if language is None:
            language = None if config.WHISPER_LANGUAGE == "auto" else config.WHISPER_LANGUAGE
        segment_iter, info = active_whisper_model.transcribe(
            wav_path,
            language=language,
            # 录音端已经按停顿切成单句；本机实测双束几乎不增加延迟，但专名更准。
            beam_size=beam_size,
            best_of=best_of,
            temperature=0.0,
            repetition_penalty=config.WHISPER_REPETITION_PENALTY,
            no_repeat_ngram_size=config.WHISPER_NO_REPEAT_NGRAM_SIZE,
            without_timestamps=True,
            chunk_length=config.WHISPER_CHUNK_LENGTH,
            max_new_tokens=config.WHISPER_MAX_NEW_TOKENS,
            # SpeechRecognition 已完成端点检测；禁用 Silero ONNX VAD 可避开部分
            # Jetson ARM64 环境的 native assertion。
            vad_filter=False,
            condition_on_previous_text=False,
            hotwords=(config.WHISPER_HOTWORDS if hotwords_override is None
                      else hotwords_override) or None,
            initial_prompt=initial_prompt,
        )
        return list(segment_iter), info

    for line in sys.stdin:
        try:
            request = json.loads(line)
            if request.get("command") == "shutdown":
                break
            wav_path = str(request["wav_path"])
            request_started = time.monotonic()
            _send({"status": "progress", "stage": "primary"})
            primary_started = time.monotonic()
            segments, info = transcribe_once(
                model,
                wav_path,
                config.WHISPER_BEAM_SIZE,
                config.WHISPER_BEST_OF,
            )
            primary_seconds = time.monotonic() - primary_started
            # Whisper 在纯底噪上偶尔会生成“Thank you for watching”等常见字幕。
            # 只有整句至少存在一个可信语音段时才返回文本；不能仅依赖解码器默认的
            # no_speech 联合条件，否则低置信度噪声仍可能进入对话和 TTS 回路。
            credible = _has_credible_speech(segments)
            text = "".join(segment.text for segment in segments).strip() if credible else ""
            text, stripped_hallucination, rejection_reason = _sanitize_transcript(
                text, getattr(info, "language", "")
            )
            if rejection_reason or (stripped_hallucination and not text):
                credible = False
            if _should_filter_suspicious(segments, text):
                credible = False
                text = ""
            primary_text = text
            primary_avg_logprob, primary_no_speech_prob = _quality_stats(segments)
            rescue_reason = _rescue_reason(segments, info, text)
            rescue_attempted = False
            rescue_applied = False
            rescue_decision = ""
            rescue_candidate_text = ""
            rescue_rejection_reason = ""
            rescue_load_seconds = 0.0
            rescue_decode_seconds = 0.0
            rescue_language = None
            rescue_hint = ""
            rescue_avg_logprob = None
            rescue_no_speech_prob = None

            if rescue_allowed and not rescue_disabled and rescue_reason:
                rescue_attempted = True
                try:
                    if rescue_model is None:
                        _send({"status": "progress", "stage": "rescue_load", "primary_seconds": round(primary_seconds, 3)})
                        print(
                            f"ASR 置信度复核：正在加载 {config.WHISPER_RESCUE_MODEL}",
                            file=sys.stderr,
                            flush=True,
                        )
                        rescue_load_started = time.monotonic()
                        rescue_model = WhisperModel(
                            config.WHISPER_RESCUE_MODEL,
                            device=device,
                            compute_type=compute,
                            local_files_only=config.WHISPER_LOCAL_FILES_ONLY,
                        )
                        rescue_load_seconds = time.monotonic() - rescue_load_started
                    rescue_started = time.monotonic()
                    _send({"status": "progress", "stage": "rescue_decode", "primary_seconds": round(primary_seconds, 3), "rescue_reason": rescue_reason})
                    command_rescue = rescue_reason in {
                        "运动指令声学置信度复核", "疑似运动口令复核",
                    }
                    rescue_language = _rescue_language(info, command_rescue)
                    vocabulary_matches = (() if command_rescue else
                        _rescue_vocabulary_matches(text, getattr(info, "language", "")))
                    rescue_hint = "、".join(term for term, _, _ in vocabulary_matches)
                    rescue_segments, rescue_info = transcribe_once(
                        rescue_model,
                        wav_path,
                        config.WHISPER_RESCUE_BEAM_SIZE,
                        config.WHISPER_RESCUE_BEST_OF,
                        language_override=rescue_language,
                        initial_prompt=(
                            _command_rescue_prompt(rescue_language or "zh")
                            if command_rescue else None
                        ),
                        hotwords_override=rescue_hint,
                    )
                    rescue_decode_seconds = time.monotonic() - rescue_started
                    rescue_avg_logprob, rescue_no_speech_prob = _quality_stats(rescue_segments)
                    rescue_credible = _has_credible_speech(rescue_segments)
                    rescue_text = "".join(segment.text for segment in rescue_segments).strip()
                    rescue_text, rescue_stripped_hallucination, rescue_rejection = (
                        _sanitize_transcript(
                            rescue_text, getattr(rescue_info, "language", "")
                        )
                    )
                    rescue_candidate_text = rescue_text
                    if not rescue_rejection and _should_filter_suspicious(rescue_segments, rescue_text):
                        rescue_rejection = "rescue_suspicious_transcript"
                    if (not rescue_credible or not rescue_text) and not rescue_rejection:
                        rescue_rejection = "rescue_low_confidence"
                    if rescue_rejection:
                        text, credible = "", False
                        rescue_decision = "rejected_rescue"
                        rejection_reason = rescue_rejection_reason = rescue_rejection
                    else:
                        selected_text, rescue_decision, rescue_rejection_reason = _review_rescue(
                            text, rescue_text, rescue_segments, rescue_info,
                            rescue_language, vocabulary_matches,
                        )
                        if selected_text and selected_text == rescue_text:
                            segments = rescue_segments
                            info = rescue_info
                            rescue_applied = True
                            if rescue_stripped_hallucination:
                                stripped_hallucination = rescue_stripped_hallucination
                        text = selected_text
                        credible = bool(text)
                        if rescue_rejection_reason:
                            rejection_reason = rescue_rejection_reason
                except Exception as rescue_error:
                    # Primary-model text remains usable; disable repeated load
                    # attempts for this worker so one deployment issue cannot add delay.
                    rescue_disabled = True
                    print(
                        f"ASR 二级复核不可用，继续使用主模型: {rescue_error}",
                        file=sys.stderr,
                        flush=True,
                    )

            avg_logprob, no_speech_prob = _quality_stats(segments)
            _send({
                "status": "ok",
                "text": text,
                "language": getattr(info, "language", ""),
                "language_probability": getattr(info, "language_probability", 0.0),
                "filtered": bool(segments and not credible),
                "avg_logprob": avg_logprob,
                "no_speech_prob": no_speech_prob,
                "rescue_attempted": rescue_attempted,
                "rescue_applied": rescue_applied,
                "rescue_decision": rescue_decision,
                "primary_text": primary_text,
                "rescue_text": rescue_candidate_text,
                "rescue_reason": rescue_reason,
                "hallucination_stripped": stripped_hallucination,
                "rejection_reason": rejection_reason,
                "rescue_rejection_reason": rescue_rejection_reason,
                "primary_avg_logprob": primary_avg_logprob,
                "primary_no_speech_prob": primary_no_speech_prob,
                "rescue_avg_logprob": rescue_avg_logprob,
                "rescue_no_speech_prob": rescue_no_speech_prob,
                "rescue_language": rescue_language,
                "rescue_hint": rescue_hint,
                "primary_seconds": round(primary_seconds, 3),
                "rescue_load_seconds": round(rescue_load_seconds, 3),
                "rescue_decode_seconds": round(rescue_decode_seconds, 3),
                "total_seconds": round(time.monotonic() - request_started, 3),
            })
        except Exception as exc:
            _send({"status": "error", "error": str(exc)})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
