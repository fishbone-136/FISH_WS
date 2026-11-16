"""Slow detector simulation: no camera, MediaPipe model or robot required."""
import threading
import time

import numpy as np
import pytest
from PyQt5.QtWidgets import QApplication
from go2_ui.hand_worker import LatestHandWorker
from go2_ui.ui.main_control_panel import VideoWidget


@pytest.fixture(scope='session')
def qapp():
    return QApplication.instance() or QApplication([])


def until(app, condition, timeout=1.):
    end = time.monotonic()+timeout
    while not condition() and time.monotonic()<end:
        app.processEvents()
        time.sleep(.002)
    assert condition()


class Detector:
    def __init__(self):
        self.started = threading.Event()
        self.release = threading.Event()
        self.frames = []
        self.threads = []
        self.closed = False

    def detect(self, frame, annotate=True):
        self.frames.append(int(frame[0,0,0]))
        self.threads.append(threading.get_ident())
        if len(self.frames) == 1:
            self.started.set()
            assert self.release.wait(2.)
        return frame, [self.frames[-1]], ['Left']

    def close(self):
        self.closed = True


def test_inference_runs_off_ui_and_retains_only_latest_frame(qapp):
    detector = Detector()
    worker = LatestHandWorker(detector)
    results = []
    worker.result_ready.connect(results.append)
    worker.set_enabled(True)
    try:
        worker.submit(np.zeros((8,8,3), dtype=np.uint8), time.monotonic())
        assert detector.started.wait(1.)
        worker.submit(np.ones((8,8,3), dtype=np.uint8), time.monotonic())
        worker.submit(np.full((8,8,3), 2, dtype=np.uint8), time.monotonic())
        detector.release.set()
        until(qapp, lambda: len(results)==2)
        assert detector.frames == [0,2]
        assert all(t != threading.get_ident() for t in detector.threads)
    finally:
        detector.release.set()
        worker.shutdown()
    assert detector.closed


def test_old_mode_inference_cannot_emit_after_disable_and_restart(qapp):
    detector = Detector()
    worker = LatestHandWorker(detector)
    results = []
    worker.result_ready.connect(results.append)
    worker.set_enabled(True)
    try:
        worker.submit(np.zeros((8,8,3), dtype=np.uint8), time.monotonic())
        assert detector.started.wait(1.)
        worker.set_enabled(False)
        worker.set_enabled(True)
        worker.submit(np.ones((8,8,3), dtype=np.uint8), time.monotonic())
        detector.release.set()
        until(qapp, lambda: len(results)==1)
        assert results[0][2] == [1]
    finally:
        detector.release.set()
        worker.shutdown()


def test_video_rejects_stale_results_and_watchdog_reports_loss(qapp, monkeypatch):
    monkeypatch.setattr(VideoWidget, '_start_camera', lambda self: None)
    detector = Detector()
    video = VideoWidget(hand_detector=detector)
    results = []
    video.landmarks_updated.connect(results.append)
    try:
        video.set_hand_detection(True)
        generation = video._hand_worker.generation
        video._on_hand_result((generation, time.monotonic()-1., [1], ['Left']))
        assert results == []
        video._hand_last_result_at = time.monotonic()-1.
        video._check_hand_freshness()
        video._check_hand_freshness()
        assert results == [([],[],None)]
        video.set_hand_detection(False)
        video._on_hand_result((generation, time.monotonic(), [1], ['Left']))
        assert len(results) == 1
    finally:
        video.shutdown()
        video.deleteLater()
