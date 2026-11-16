from types import SimpleNamespace

from voice.command_safety import gate_command
from voice.asr_worker import (
    _choose_rescue_text,
    _has_decoder_repetition,
    _has_credible_speech,
    _is_suspicious_transcript,
    _looks_like_command_candidate,
    _normalize_robot_name,
    _rescue_reason,
    _should_filter_suspicious,
    _sanitize_transcript,
    _strip_trailing_hallucination,
)


def _segment(avg_logprob: float, no_speech_prob: float):
    return SimpleNamespace(
        avg_logprob=avg_logprob,
        no_speech_prob=no_speech_prob,
    )


def test_rejects_low_confidence_noise_hallucination():
    assert not _has_credible_speech([_segment(-1.062, 0.322)])


def test_rejects_probable_silence():
    assert not _has_credible_speech([_segment(-0.2, 0.95)])


def test_accepts_confident_speech():
    assert _has_credible_speech([_segment(-0.3, 0.1)])


def _info(language_probability: float = 0.9):
    return SimpleNamespace(language="zh", language_probability=language_probability)


def test_requests_turbo_rescue_for_credible_but_uncertain_speech():
    assert _rescue_reason([_segment(-0.6, 0.1)], _info(), "邦邦介绍一下你自己")


def test_does_not_waste_turbo_on_rejected_noise():
    assert not _rescue_reason(
        [_segment(-1.062, 0.322)],
        _info(0.2),
        "Thank you for watching.",
    )


def test_detects_common_subtitle_hallucination_for_verification():
    assert _is_suspicious_transcript("Thank you for watching!")
    assert _is_suspicious_transcript("感谢您的观看。")
    assert _is_suspicious_transcript("Thank you")
    assert _is_suspicious_transcript(
        "请不吝点赞、订阅、转发、打赏支持明镜与点点栏目"
    )


def test_strips_known_subtitle_hallucination_from_real_transcript_tail():
    text, stripped = _strip_trailing_hallucination(
        "邦邦，介绍一下你自己。请不吝点赞 订阅 转发 打赏支持明镜与点点栏目。"
    )
    assert text == "邦邦，介绍一下你自己"
    assert stripped == "请不吝点赞 订阅 转发 打赏支持明镜与点点栏目。"


def test_strips_logged_solanya_subtitle_tail_and_normalizes_command():
    text, stripped, reason = _sanitize_transcript(
        "執行命令 向前走兩步字幕by索兰娅", "zh"
    )
    assert text == "执行命令 向前走两步"
    assert stripped == "字幕by索兰娅"
    assert reason == ""


def test_repairs_logged_forward_command_and_strips_short_command_tail():
    text, stripped, reason = _sanitize_transcript(
        "执行全劲动作谢谢大家", "zh"
    )
    assert text == "执行前进动作"
    assert stripped == "谢谢大家"
    assert reason == ""

    text, stripped, reason = _sanitize_transcript(
        "行前进动作感谢观看", "zh"
    )
    assert text == "执行前进动作"
    assert stripped == "感谢观看"
    assert reason == ""


def test_repairs_logged_left_turn_command_variants():
    assert _sanitize_transcript("执行折断动作", "zh") == (
        "执行左转动作", "", ""
    )
    assert _sanitize_transcript("直行截断动作好啊", "zh") == (
        "执行左转动作好啊", "", ""
    )


def test_repairs_latest_logged_fixed_command_variants():
    assert _sanitize_transcript("实行右转正左", "zh") == (
        "执行右转动作", "", ""
    )
    assert _sanitize_transcript("直行右转頓座", "zh") == (
        "执行右转动作", "", ""
    )
    assert _sanitize_transcript("直行后退頓座", "zh") == (
        "执行后退动作", "", ""
    )
    assert _sanitize_transcript("执行帕萨顿作", "zh") == (
        "执行趴下动作", "", ""
    )
    assert _sanitize_transcript("执行哈萨动作", "zh") == (
        "执行趴下动作", "", ""
    )


def test_preserves_and_authorizes_asr_terminal_semicolon():
    text, stripped, reason = _sanitize_transcript("邦邦执行后退动作；", "zh")

    assert text == "邦邦执行后退动作；"
    assert stripped == ""
    assert reason == ""
    assert gate_command(text).command_text == "后退"


def test_corrupt_short_action_frame_requests_command_rescue():
    assert _looks_like_command_candidate("执行陌生顿作")
    assert _looks_like_command_candidate("尺寸使用右转的动作")


def test_collapses_repeated_fixed_action_phrase():
    assert _sanitize_transcript("执行前进动作 前进动作", "zh") == (
        "执行前进动作", "", ""
    )

    text, stripped, reason = _sanitize_transcript(
        "直行雄徑动作谢谢大家", "zh"
    )
    assert text == "执行前进动作"
    assert stripped == "谢谢大家"
    assert reason == ""

    text, stripped, reason = _sanitize_transcript(
        "只想闪电动作谢谢大家", "zh"
    )
    assert text == "执行前进动作"
    assert stripped == "谢谢大家"
    assert reason == ""


def test_does_not_strip_short_thanks_from_normal_conversation():
    text = "我想对帮助我的人说谢谢大家"
    assert _strip_trailing_hallucination(text) == (text, "")


def test_rescue_cannot_downgrade_or_change_a_deterministic_command():
    assert _choose_rescue_text(
        "执行前进动作", "执行奇怪动作"
    ) == ("执行前进动作", "kept_primary_command")
    assert _choose_rescue_text(
        "执行前进动作", "执行后退动作"
    ) == ("", "command_disagreement")
    assert _choose_rescue_text(
        "执行前进动作", "执行全劲动作"
    ) == ("执行全劲动作", "applied_rescue")


def test_rejects_whole_known_subtitle_hallucination():
    text, stripped = _strip_trailing_hallucination(
        "请不吝点赞、订阅、转发、打赏支持明镜与点点栏目"
    )
    assert text == ""
    assert stripped


def test_does_not_strip_normal_subscription_discussion():
    text = "请帮我解释点赞和订阅有什么区别"
    assert _strip_trailing_hallucination(text) == (text, "")


def test_rejects_logged_initial_prompt_echo_even_when_confident():
    text, _, reason = _sanitize_transcript(
        "Bangbang, execute command, execute forward action, emergency alert, "
        "stop, ground guide line, 机器狗名，ვვვვვვვვ",
        "nn",
    )
    assert text == ""
    assert reason == "prompt_echo"


def test_rejects_decoder_replacement_character():
    assert _sanitize_transcript("邦邦向前走�", "zh")[2] == "invalid_unicode"


def test_rejects_logged_hotword_prompt_echo():
    text, _, reason = _sanitize_transcript(
        "邦邦 执行指令 前进 后退 左转 右转 停止 趴下 站起执行命令 "
        "前进 后退 左转 右转 左转 左转 左转 左转 左转 右转 左转",
        "zh",
    )
    assert text == ""
    assert reason == "prompt_echo"


def test_rejects_logged_repeated_authorizer():
    text, _, reason = _sanitize_transcript("执行指令 " * 11, "zh")
    assert text == ""
    assert reason == "prompt_echo"


def test_rejects_generic_decoder_token_loop():
    assert _has_decoder_repetition("左转 左转 左转 左转 左转")
    assert _sanitize_transcript("hello hello hello hello", "en")[2] == "decoder_repetition"


def test_keeps_normal_command_with_non_looping_repetition():
    text = "邦邦，执行指令，前进两米，然后左转"
    assert not _has_decoder_repetition(text)
    assert _sanitize_transcript(text, "zh") == (text, "", "")


def test_only_filters_short_thanks_when_acoustic_confidence_is_weak():
    assert _should_filter_suspicious([_segment(-0.82, 0.38)], "Thank you.")
    assert not _should_filter_suspicious([_segment(-0.08, 0.01)], "Thank you.")


def test_short_safety_command_stays_on_fast_path():
    assert not _rescue_reason([_segment(-0.2, 0.1)], _info(0.2), "stop")


def test_clear_authorized_movement_command_stays_on_fast_path():
    assert not _rescue_reason(
        [_segment(-0.30, 0.01)],
        _info(0.99),
        "执行指令，前进一米",
    )


def test_uncertain_authorized_movement_command_gets_turbo_review():
    assert _rescue_reason(
        [_segment(-0.50, 0.10)],
        _info(0.99),
        "执行指令，前进一米",
    ) == "运动指令声学置信度复核"


def test_misheard_execute_phrase_only_requests_review_not_authorization():
    text = "執行秘密,校前走兩步"
    assert _looks_like_command_candidate(text)
    assert _rescue_reason(
        [_segment(-0.2, 0.01)], _info(0.99), text
    ) == "疑似运动口令复核"


def test_long_low_language_confidence_text_gets_verified():
    assert _rescue_reason(
        [_segment(-0.2, 0.1)],
        _info(0.2),
        "execute forward action",
    )


def test_normalizes_robot_name_for_detected_language():
    assert _normalize_robot_name("Bangbang请前进", "zh") == "邦邦请前进"
    assert _normalize_robot_name("bang bang，介绍一下你自己", "zh") == "邦邦，介绍一下你自己"
    assert _normalize_robot_name("帮帮，请介绍一下你自己", "zh") == "邦邦，请介绍一下你自己"
    assert _normalize_robot_name("棒棒执行停止动作", "zh") == "邦邦执行停止动作"
    assert _normalize_robot_name("包包向前走两步", "zh") == "邦邦向前走两步"
    assert _normalize_robot_name("帮帮我拿一下东西", "zh") == "帮帮我拿一下东西"
    assert _normalize_robot_name("邦邦, please stop", "en") == "Bangbang, please stop"
