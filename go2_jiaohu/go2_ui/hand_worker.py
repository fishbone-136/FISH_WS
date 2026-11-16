"""Run hand inference on the newest frame, without a growing frame queue."""
import logging
import threading
import time

import cv2
from PyQt5.QtCore import QObject, pyqtSignal


class LatestHandWorker(QObject):
    result_ready = pyqtSignal(object)
    MAX_AGE = .30

    def __init__(self, detector, parent=None):
        super().__init__(parent)
        self.detector = detector
        self._condition = threading.Condition()
        self._pending = None
        self._enabled = False
        self._closed = False
        self.generation = 0
        self._thread = None

    def set_enabled(self, enabled):
        with self._condition:
            self.generation += 1
            self._enabled = enabled
            self._pending = None
            self._condition.notify_all()

    def submit(self, frame, observed_at):
        with self._condition:
            if self._closed or not self._enabled:
                return
            self._pending = (self.generation, observed_at, frame.copy())
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name='go2-hand-inference', daemon=True)
                self._thread.start()
            self._condition.notify_all()

    def _run(self):
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._closed or self._pending is not None)
                    if self._closed:
                        return
                    generation, observed_at, frame = self._pending
                    self._pending = None
                if time.monotonic() - observed_at > self.MAX_AGE:
                    continue
                try:
                    if frame.shape[1] > 640:
                        height = round(frame.shape[0] * 640 / frame.shape[1])
                        frame = cv2.resize(frame, (640, height), interpolation=cv2.INTER_AREA)
                    _image, landmarks, handedness = self.detector.detect(frame, annotate=False)
                except Exception:
                    logging.exception('手势识别异常')
                    landmarks, handedness = [], []
                with self._condition:
                    valid = (not self._closed and self._enabled and generation == self.generation)
                if valid and time.monotonic() - observed_at <= self.MAX_AGE:
                    self.result_ready.emit((generation, observed_at, landmarks, handedness))
        finally:
            close = getattr(self.detector, 'close', None)
            if callable(close):
                close()

    def shutdown(self):
        with self._condition:
            self._closed = True
            self._enabled = False
            self._pending = None
            self.generation += 1
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=2.)
        # A model still initializing can finish in its daemon thread. It owns
        # and closes the detector; no GUI object or robot command is used there.
