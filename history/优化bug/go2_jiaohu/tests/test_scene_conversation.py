from unittest.mock import Mock

import numpy as np

from offline_test.vlm_controller import VLMController
from voice.ollama_client import OllamaError


def _finish(controller):
    thread = controller._task_thread
    if thread is not None:
        thread.join(2)
        assert not thread.is_alive()


def test_scene_questions_include_current_image_and_bounded_history():
    controller = VLMController()
    controller._ollama.chat = Mock(return_value="有一个红色杯子。")
    frame = np.zeros((360, 640, 3), dtype=np.uint8)
    results = []
    assert controller.analyze_scene_async(frame, results.append)
    _finish(controller)
    assert controller.query_scene_async(frame, "它是什么颜色？", results.append)
    _finish(controller)
    messages = controller._ollama.chat.call_args.args[0]
    assert messages[-1]["content"] == "它是什么颜色？"
    assert len(messages[-1]["images"]) == 1
    assert messages[-2] == {"role": "assistant", "content": "有一个红色杯子。"}
    assert all("images" not in message for message in messages[:-1])
    for _ in range(5):
        assert controller.query_scene_async(frame, "再描述一下", results.append)
        _finish(controller)
    assert len(controller._history) == 8
    controller.cancel_pending()
    assert controller._history == []


def test_missing_model_is_reported_without_fabricated_scene():
    controller = VLMController()
    controller._ollama.chat = Mock(side_effect=OllamaError("未安装视觉模型"))
    errors, results = [], []
    controller.on_error = errors.append
    assert controller.analyze_scene_async(np.zeros((16, 16, 3), dtype=np.uint8), results.append)
    _finish(controller)
    assert results == []
    assert "未安装视觉模型" in errors[0]


def test_scene_discards_unsolicited_ocr_but_explicit_question_keeps_text():
    controller = VLMController()
    controller._ollama.chat = Mock(return_value="桌边有两个人。画面中可以看到的文字是：11 11 11")
    frame = np.zeros((16, 16, 3), dtype=np.uint8)
    results = []
    assert controller.analyze_scene_async(frame, results.append)
    _finish(controller)
    assert results == ["桌边有两个人。"]
    assert controller._history[-1]["content"] == results[0]
    controller._ollama.chat.return_value = "标牌上的文字是：出口 EXIT。"
    assert controller.query_scene_async(frame, "标牌上写了什么？", results.append)
    _finish(controller)
    assert results[-1] == "标牌上的文字是：出口 EXIT。"
