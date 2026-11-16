import threading

from offline_test.speech_controller import SpeechController
from offline_test.vlm_controller import VLMController


def test_speech_does_not_start_while_previous_worker_is_stopping():
    controller = SpeechController()
    release = threading.Event()
    worker = threading.Thread(target=release.wait, daemon=True)
    worker.start()
    controller._thread = worker
    controller._running = False

    try:
        assert controller.start() is False
        assert controller._thread is worker
    finally:
        release.set()
        worker.join(timeout=1.0)


def test_vlm_rejects_duplicate_task_and_discards_cancelled_result():
    controller = VLMController()
    entered = threading.Event()
    release = threading.Event()
    results = []

    def slow_run(_frame, _prompt, callback, generation, _describe_only=False):
        entered.set()
        release.wait(timeout=2.0)
        controller._deliver(generation, callback, "stale result")

    controller._run = slow_run

    assert controller.analyze_scene_async(object(), results.append) is True
    assert entered.wait(timeout=1.0)
    assert controller.query_navigation_async(object(), "go", results.append) is False

    controller.cancel_pending()
    release.set()
    thread = controller._task_thread
    if thread is not None:
        thread.join(timeout=1.0)

    assert results == []
    assert controller.is_busy is False
