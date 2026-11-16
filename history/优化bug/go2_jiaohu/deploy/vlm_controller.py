"""
VLM 视觉感知控制器 — 部署版
UI 独占 D435i 并传入当前帧；此控制器只进行画面识别和文字对话。
"""
import logging

from offline_test.vlm_controller import VLMController as _MockVLMController

logging.basicConfig(level=logging.INFO, format="%(asctime)s [Deploy VLM] %(message)s")


class VLMController(_MockVLMController):
    """Read-only visual conversation; opens no camera or robot motion bridge."""
