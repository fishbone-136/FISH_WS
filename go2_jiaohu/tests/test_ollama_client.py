import json
import unittest
from unittest.mock import Mock

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


if __name__ == "__main__":
    unittest.main()
