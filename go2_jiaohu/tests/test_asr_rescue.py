import io
import json
import sys
from types import SimpleNamespace

import pytest

from voice import asr_worker as asr


def segment(text="你好", score=-0.2, silence=0.01, tokens=3):
    return SimpleNamespace(text=text, avg_logprob=score, no_speech_prob=silence,
                           tokens=list(range(tokens)))


def info(language="zh", probability=0.99):
    return SimpleNamespace(language=language, language_probability=probability)


def test_whole_sentence_quality_cannot_mix_the_best_of_different_segments():
    segments = [segment(score=-0.1, silence=0.8, tokens=1),
                segment(score=-0.9, silence=0.01, tokens=9)]
    score, silence = asr._quality_stats(segments)
    assert score == pytest.approx(-0.82)
    assert silence == 0.8
    assert asr._rescue_quality_rejection(segments, info(), "zh") == "rescue_low_confidence"


def test_one_bad_command_fragment_still_triggers_review_when_average_is_good():
    segments = [segment("执行指令，前进", tokens=10), segment("三米", -0.9, tokens=1)]
    assert asr._quality_stats(segments)[0] > -0.35
    assert asr._rescue_reason(segments, info(), "执行指令，前进三米") == "运动指令声学置信度复核"
    assert not asr._rescue_reason(segments, info(), "停止")


@pytest.mark.parametrize("score,silence", [(-0.6, 0.01), (-0.2, 0.6),
                                         (float("nan"), 0.01), (-0.2, float("nan"))])
def test_review_rejects_unreliable_rescue_instead_of_forwarding_a_guess(score, silence):
    selected, decision, reason = asr._review_rescue(
        "你能帮我吗", "你好", [segment(score=score, silence=silence)], info(), "zh")
    assert selected == "" and decision == "rejected_rescue"
    assert reason == "rescue_low_confidence"


def test_confident_language_is_reused_but_uncertain_detection_can_be_corrected(monkeypatch):
    monkeypatch.setattr(asr.config, "WHISPER_LANGUAGE", "auto")
    assert asr._rescue_language(info()) == "zh"
    assert asr._rescue_language(info("en")) == "en"
    assert asr._rescue_language(info("zh", 0.3)) is None
    monkeypatch.setattr(asr.config, "WHISPER_LANGUAGE", "en")
    assert asr._rescue_language(info()) == "en"


def test_wrong_language_cannot_replace_primary_transcript():
    assert asr._review_rescue("你好", "Hello", [segment()], info("en"), "zh")[2] == (
        "rescue_language_mismatch")


def test_alias_only_requests_redecode_and_never_rewrites_transcript():
    text = "你能带我去大煙塔吗"
    assert asr._rescue_reason([segment(text)], info(), text) == "专名候选复核"
    assert asr._sanitize_transcript(text, "zh")[0] == text
    assert asr._rescue_vocabulary_matches(text, "zh") == (("大雁塔", "zh", True),)
    # 普通餐厅名不做无依据的纠正，也不向无关对话或英语注入中文地名。
    assert asr._rescue_vocabulary_matches("大眼堂餐厅", "zh") == ()
    assert asr._rescue_vocabulary_matches("你能做什么工作", "zh") == ()
    assert asr._rescue_vocabulary_matches(text, "en") == ()
    assert not asr._rescue_reason([segment(text, -1.0, 0.9)], info(), text)
    assert asr._rescue_reason([segment("你能帶我去大燕塔嗎?", -0.1)], info(),
                              "你能帶我去大燕塔嗎?") == "专名候选复核"


def test_unresolved_name_asks_for_repeat_instead_of_hallucinated_answer():
    assert asr._review_rescue(
        "你能带我去大煙塔吗", "你能带我去大眼堂吗", [segment()], info(), "zh",
        (("大雁塔", "zh", True),),
    ) == ("", "rejected_rescue", "rescue_unresolved_vocabulary")


def test_correct_name_is_not_replaced_with_a_new_misspelling():
    assert asr._review_rescue(
        "你能带我去大雁塔吗", "你能带我去大眼堂吗", [segment()], info(), "zh",
        (("大雁塔", "zh", False),),
    ) == ("你能带我去大雁塔吗", "kept_primary_vocabulary", "")


def test_hint_echo_cannot_replace_a_complete_utterance():
    assert asr._review_rescue(
        "你能带我去大煙塔吗", "大雁塔", [segment()], info(), "zh",
        (("大雁塔", "zh", True),),
    ) == ("", "rejected_rescue", "rescue_prompt_echo")
    assert asr._review_rescue(
        "大煙塔", "大雁塔", [segment()], info(), "zh",
        (("大雁塔", "zh", True),),
    ) == ("大雁塔", "applied_rescue", "")


def test_bad_vocabulary_file_does_not_disable_acoustic_review(tmp_path, capsys):
    path = tmp_path / "vocabulary.json"
    path.write_text('{"unexpected":"object"}')
    assert asr._load_rescue_vocabulary(str(path)) == ()
    assert "词表不可用" in capsys.readouterr().err


def run_worker(monkeypatch, primary, rescue, requests=1):
    models, calls, output = [], [], []

    class Model:
        def __init__(self, name, **kwargs):
            self.name = name
            models.append(name)

        def transcribe(self, path, **kwargs):
            calls.append((self.name, kwargs))
            transcript = primary if self.name == "primary" else rescue
            return iter([transcript]), info()

    monkeypatch.setitem(sys.modules, "faster_whisper", SimpleNamespace(WhisperModel=Model))
    monkeypatch.setattr(sys, "stdin", io.StringIO('{"wav_path":"fake.wav"}\n' * requests))
    monkeypatch.setattr(asr, "_select_runtime", lambda: ("cuda", "float16"))
    monkeypatch.setattr(asr, "_send", output.append)
    monkeypatch.setattr(asr.config, "WHISPER_MODEL", "primary")
    monkeypatch.setattr(asr.config, "WHISPER_RESCUE_MODEL", "rescue")
    monkeypatch.setattr(asr.config, "WHISPER_PRELOAD_RESCUE", False)
    monkeypatch.setattr(asr.config, "WHISPER_LANGUAGE", "auto")
    assert asr.main() == 0
    # Also verify newly added diagnostics remain valid JSON, including absent scores.
    json.dumps(output, allow_nan=False)
    return output[-1], models, calls


def test_worker_applies_language_and_only_the_relevant_name_hint(monkeypatch):
    result, models, calls = run_worker(monkeypatch,
        segment("你能带我去大煙塔吗", -0.5), segment("你能带我去大雁塔吗"))
    assert models == ["primary", "rescue"]
    assert calls[0][1]["hotwords"] is None
    assert calls[1][1]["hotwords"] == "大雁塔"
    assert calls[1][1]["language"] == "zh"
    assert calls[1][1]["initial_prompt"] is None
    assert result["text"] == "你能带我去大雁塔吗"
    assert result["primary_text"] == "你能带我去大煙塔吗"
    assert result["rescue_applied"] is True


@pytest.mark.parametrize("rescue,reason", [
    (segment("你能带我去大眼堂吗"), "rescue_unresolved_vocabulary"),
    (segment("你能带我去大雁塔吗", -0.6), "rescue_low_confidence"),
])
def test_worker_does_not_send_unconfirmed_text_to_llm(monkeypatch, rescue, reason):
    result, _, _ = run_worker(monkeypatch, segment("你能带我去大煙塔吗", -0.5), rescue)
    assert result["text"] == "" and result["filtered"]
    assert result["rejection_reason"] == reason
    assert not result["rescue_applied"]


def test_worker_noise_never_loads_or_prompts_rescue(monkeypatch):
    result, models, calls = run_worker(monkeypatch,
        segment("大烟塔", -1.0, 0.95), segment("大雁塔"))
    assert models == ["primary"] and len(calls) == 1
    assert result["text"] == "" and not result["rescue_attempted"]


def test_worker_still_rejects_disagreement_on_motion_distance(monkeypatch):
    result, _, calls = run_worker(monkeypatch,
        segment("执行指令，前进一米", -0.5), segment("执行指令，前进三米"))
    assert result["text"] == "" and result["rejection_reason"] == "command_disagreement"
    assert calls[1][1]["initial_prompt"]
    assert calls[1][1]["hotwords"] is None


def test_worker_reuses_rescue_model_for_later_utterances(monkeypatch):
    result, models, calls = run_worker(monkeypatch,
        segment("你能带我去大煙塔吗", -0.5), segment("你能带我去大雁塔吗"), requests=2)
    assert models == ["primary", "rescue"]
    assert [name for name, _ in calls] == ["primary", "rescue", "primary", "rescue"]
    assert result["rescue_load_seconds"] == 0
