from voice.llm_processor import (
    OllamaInstructionParser,
    _reply_matches_language,
    _response_language,
)
from voice.tts_worker import _language_for as _tts_language_for


class _FakeClient:
    def __init__(self, replies):
        self.replies = list(replies)
        self.calls = []

    def chat(self, messages, **kwargs):
        self.calls.append((messages, kwargs))
        return self.replies.pop(0)


def test_detects_dominant_language_in_mixed_device_text():
    assert _response_language("D435i相机怎么使用") == "zh"
    assert _response_language("Bangbang, what is D435i?") == "en"
    assert _tts_language_for("D435i相机怎么使用") == "zh"
    assert _tts_language_for("Bangbang, what is D435i?") == "en"


def test_language_match_rejects_chinese_in_english_reply():
    assert _reply_matches_language("当然可以。", "zh")
    assert _reply_matches_language("Of course.", "en")
    assert not _reply_matches_language("当然可以。", "en")


def test_english_chat_has_last_turn_language_lock():
    client = _FakeClient(["I am Bangbang, your local robot companion."])
    parser = OllamaInstructionParser(client=client)

    reply = parser.chat("Please introduce yourself.")

    assert reply.startswith("I am Bangbang")
    messages, options = client.calls[0]
    assert messages[-2]["role"] == "system"
    assert "Reply only" in messages[-2]["content"]
    assert options["max_tokens"] == 64


def test_wrong_language_chat_is_retried_without_history():
    client = _FakeClient(["我是邦邦。", "I am Bangbang."])
    parser = OllamaInstructionParser(client=client)

    assert parser.chat("Who are you?") == "I am Bangbang."
    assert len(client.calls) == 2
    assert len(client.calls[1][0]) == 3


def test_second_wrong_language_uses_safe_english_fallback():
    client = _FakeClient(["我是邦邦。", "我还是使用中文。"])
    parser = OllamaInstructionParser(client=client)

    reply = parser.chat("Who are you?")

    assert reply == "Sorry, I could not formulate that answer. Please ask me once more."
    assert not any("\u3400" <= character <= "\u9fff" for character in reply)


def test_english_command_never_returns_chinese_confirmation():
    client = _FakeClient([
        '{"reply":"收到，正在前进。","actions":[{"action":"forward","speed":0.2,"duration":2}]}'
    ])
    parser = OllamaInstructionParser(client=client)

    parsed = parser.parse_command("move forward")

    assert parsed["reply"] == "Command received."
    assert parsed["actions"][0]["action"] == "forward"


def test_command_action_keywords_are_normalized_to_sdk_whitelist():
    client = _FakeClient([
        '{"reply":"收到。","actions":[{"action":"前进"},'
        '{"action":"turn_right"},"停止"]}'
    ])
    parser = OllamaInstructionParser(client=client)

    parsed = parser.parse_command("向前走，然后右转并停止")

    assert [action["action"] for action in parsed["actions"]] == [
        "forward", "turn_right", "stop"
    ]


def test_command_reply_without_actions_is_safe_empty_result():
    client = _FakeClient(['{"reply":"请重新说明。"}'])
    parser = OllamaInstructionParser(client=client)

    parsed = parser.parse_command("不清楚的动作")

    assert parsed["reply"] == "请重新说明。"
    assert parsed["actions"] == []
