"""邦邦本地 D435i + Ollama 交互终端。

`/cam` 只分析画面；任何物理移动仍必须使用“执行指令/执行…动作”或对应英文
口令，和主 UI 共用同一个安全入口。
"""

from __future__ import annotations

import os
import sys
import threading

import numpy as np


if __package__ in {None, ""}:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from deploy.speech_controller import SpeechController
from offline_test.vlm_controller import VLMController


class RealSenseCamera:
    def __init__(self, width: int = 640, height: int = 360, fps: int = 15):
        import pyrealsense2 as rs

        self.rs = rs
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.profile = self.pipeline.start(cfg)
        self.align = rs.align(rs.stream.color)
        self._lock = threading.Lock()
        self._color = None
        self._depth = None
        self._running = True
        self._thread = threading.Thread(target=self._capture, daemon=True)
        self._thread.start()

    def _capture(self) -> None:
        while self._running:
            try:
                frames = self.align.process(self.pipeline.wait_for_frames(timeout_ms=1000))
                color = frames.get_color_frame()
                depth = frames.get_depth_frame()
                if not color:
                    continue
                color_array = np.asanyarray(color.get_data()).copy()
                depth_array = np.asanyarray(depth.get_data()).copy() if depth else None
                with self._lock:
                    self._color = color_array
                    self._depth = depth_array
            except Exception:
                continue

    def color(self):
        with self._lock:
            return None if self._color is None else self._color.copy()

    def depth(self):
        with self._lock:
            return None if self._depth is None else self._depth.copy()

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=2.0)
        try:
            self.pipeline.stop()
        except Exception:
            pass


def main() -> int:
    try:
        camera = RealSenseCamera()
    except Exception as exc:
        print(f"D435i 初始化失败: {exc}")
        return 1

    speech = SpeechController(frame_provider=camera.color, depth_provider=camera.depth)
    vision = VLMController()
    speech.on_error = lambda message: print(f"[错误] {message}")
    print("邦邦本地视觉终端已启动。输入 /cam 分析，/quit 退出。")
    try:
        while True:
            text = input("主人: ").strip()
            if text in {"/quit", "/exit", "/q"}:
                break
            if text == "/cam":
                frame = camera.color()
                if frame is None:
                    print("尚未取得 D435i 画面")
                    continue
                finished = threading.Event()

                def show(result: str) -> None:
                    print(f"邦邦: {result}")
                    finished.set()

                vision.analyze_scene_async(frame, show)
                finished.wait(120.0)
                continue
            if text:
                print(f"邦邦: {speech.simulate_text(text)}")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        speech.stop()
        camera.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
