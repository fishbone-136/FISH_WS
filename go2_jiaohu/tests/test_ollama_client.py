import json
import unittest
from unittest.mock import Mock

from voice import config
from voice.ollama_client import OllamaClient


class _Response:
    def __init__(self, payload):
        self.payload = json.dumps(payload).encode()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        return False

    def read(self):
        return self.payload


class OllamaClientTests(unittest.TestCase):
    def test_rejects_non_local_host(self):
        with self.assertRaises(ValueError):
            OllamaClient(host="https://example.com")

    def test_chat_uses_installed_qwen_and_bounded_generation(self):
        responses = [
            _Response({"models": [{"name": "qwen2.5:3b"}]}),
            _Response({"message": {"content": "你好"}}),
        ]
        requests = []

        def fake_urlopen(request, timeout):
            requests.append(request)
            return responses.pop(0)

        client = OllamaClient(host="http://127.0.0.1:11434", model="qwen2.5:3b")
        client._opener.open = Mock(side_effect=fake_urlopen)
        reply = client.chat([{"role": "user", "content": "你好"}], max_tokens=42)
        self.assertEqual(reply, "你好")
        payload = json.loads(requests[1].data.decode())
        self.assertEqual(payload["model"], "qwen2.5:3b")
        self.assertEqual(payload["options"]["num_predict"], 42)
        self.assertFalse(payload["stream"])

    def test_warmup_and_chat_reuse_same_model_context_and_keep_alive(self):
        payloads = []
        client = OllamaClient(model="qwen2.5:3b")
        client.resolve_model = Mock(return_value="qwen2.5:3b")

        def request(method, path, payload):
            payloads.append(payload)
            return {"message": {"content": "你好"}}

        client._request = request
        client.warm_up()
        client.chat([{"role": "user", "content": "你好"}],
                    max_tokens=config.OLLAMA_CHAT_MAX_PREDICT)
        warmup, chat = payloads
        for key in ("model", "keep_alive"):
            self.assertEqual(warmup[key], chat[key])
        self.assertEqual(warmup["options"]["num_ctx"], chat["options"]["num_ctx"])
        self.assertEqual(chat["options"]["num_ctx"], config.OLLAMA_CONTEXT_LENGTH)
        self.assertEqual(warmup["options"]["num_predict"], 1)
        self.assertEqual(chat["options"]["num_predict"], config.OLLAMA_CHAT_MAX_PREDICT)


if __name__ == "__main__":
    unittest.main()
