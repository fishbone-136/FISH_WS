"""
VLM 视觉感知控制器 — 部署版
UI 独占 D435i；文字问答与显式目标定位使用同一视觉模型。
"""
import logging
import threading

from voice.visual_navigation import locate_target

from offline_test.vlm_controller import VLMController as _MockVLMController

logging.basicConfig(level=logging.INFO, format="%(asctime)s [Deploy VLM] %(message)s")


class VLMController(_MockVLMController):
    """Ground targets in RGB-D; navigation is dispatched by the shared UI bridge."""

    def __init__(self):
        super().__init__()
        from voice.person_detector import detector
        detector.warmup()

    def locate_target_async(self, snapshot, intent, request_id, on_result, on_error=None):
        with self._task_lock:
            if self._task_thread is not None and self._task_thread.is_alive():
                return False
            self._task_generation += 1
            generation = self._task_generation
            def run():
                try:
                    from voice.person_detector import person_side, locate_person, PersonNotFound
                    if person_side(intent.target) is not None:
                        try:
                            target = locate_person(snapshot, intent, request_id)
                        except (RuntimeError, PersonNotFound) as error:
                            logging.warning('快速检测不可用，使用视觉模型定位：%s', error)
                            target = locate_target(self._ollama, snapshot, intent, request_id)
                    else:
                        target = locate_target(self._ollama, snapshot, intent, request_id)
                    self._deliver(generation, on_result, target)
                except Exception as error:
                    message = f'目标定位失败：{error}'
                    if on_error is not None:
                        if self._is_current(generation):
                            on_error(request_id, message)
                    else:
                        self._error(message, generation)
                finally:
                    with self._task_lock:
                        if self._task_thread is threading.current_thread():
                            self._task_thread = None
            self._task_thread = threading.Thread(target=run, daemon=True, name='bangbang-visual-target')
            self._task_thread.start()
        return True
