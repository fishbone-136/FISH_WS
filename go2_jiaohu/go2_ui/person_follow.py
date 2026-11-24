"""Qt adapter for the tested SDK follower, with one camera owner and async stop."""
from concurrent.futures import ThreadPoolExecutor
from collections import deque
import logging
import threading
import time

from PyQt5.QtCore import QObject, QTimer, pyqtSignal
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QDoubleSpinBox, QTextEdit, QSizePolicy


class FollowControls(QWidget):
    start_requested = pyqtSignal(float,float)
    stop_requested = pyqtSignal()

    def __init__(self,parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0,0,0,0)
        row = QHBoxLayout()
        self.speed = QDoubleSpinBox()
        self.speed.setRange(.05,.40)
        self.speed.setSingleStep(.05)
        self.speed.setValue(.35)
        self.speed.setSuffix(' m/s')
        self.distance = QDoubleSpinBox()
        self.distance.setRange(1.,2.)
        self.distance.setSingleStep(.1)
        self.distance.setValue(1.5)
        self.distance.setSuffix(' m')
        row.addWidget(QLabel('速度上限'))
        row.addWidget(self.speed)
        row.addWidget(QLabel('跟随距离'))
        row.addWidget(self.distance)
        layout.addLayout(row)
        self.status = QTextEdit()
        self.status.setReadOnly(True)
        self.status.setFixedHeight(44)
        self.status.setPlainText('正在准备广角画面…')
        self.status.setStyleSheet('color:#00E5FF; font-size:14px; background:transparent; border:none; padding:0;')
        layout.addWidget(self.status,1)
        self.metrics = QLabel('预览中 · 尚未启动跟随')
        layout.addWidget(self.metrics)
        buttons = QHBoxLayout()
        self.start_btn = QPushButton('开始跟随')
        self.stop_btn = QPushButton('停止跟随')
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(False)
        buttons.addWidget(self.start_btn)
        buttons.addWidget(self.stop_btn)
        layout.addLayout(buttons)
        hint = QLabel('先预览再开始，每次最多 2 分钟。\n“停止当前功能”退出并恢复彩色画面。')
        hint.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Fixed)
        hint.setStyleSheet('color:#8FAEC6; font-size:12px;')
        layout.addWidget(hint)
        self.start_btn.clicked.connect(lambda:self.start_requested.emit(self.speed.value(),self.distance.value()))
        self.stop_btn.clicked.connect(self.stop_requested.emit)

    def set_state(self,ready,moving,closing):
        self.start_btn.setEnabled(ready and not moving and not closing)
        self.stop_btn.setEnabled(moving and not closing)
        self.speed.setEnabled(not moving and not closing)
        self.distance.setEnabled(not moving and not closing)


class PersonFollowSession(QObject):
    status_changed = pyqtSignal(str)
    state_changed = pyqtSignal(bool,bool,bool)
    metrics_changed = pyqtSignal(str)
    closed = pyqtSignal()

    def __init__(self,depth_widget,parent=None,*,camera_factory=None,people=None,controller_factory=None,project=None):
        super().__init__(parent)
        self.depth_widget = depth_widget
        self._camera_factory,self._people,self._controller_factory = camera_factory,people,controller_factory
        self._project = project
        self._camera = self._controller = self._pool = self._pending = None
        self._opened = self._closing = self._restoring = False
        self._restore_camera = True
        self._restore_deadline = 0.
        self._ready = self._moving = self._motion_ended = False
        self._frame = None
        self._sequence = 0
        self._last_sample = -1.
        self._last_result = self._feedback_frame = None
        self._status = ''
        self._preview_status = ''
        self._preview_status_since = 0.
        self._opencv_threads = None
        self._events = deque(maxlen=32)
        self._lock = threading.Lock()
        self._timer = QTimer(self)
        self._timer.setInterval(33)
        self._timer.timeout.connect(self._poll)

    @property
    def active(self):
        return self._opened or self._closing

    @property
    def closing(self):
        return self._closing

    @property
    def motion_active(self):
        return self._moving or bool(self._controller is not None and self._controller.active)

    def _set_status(self,message):
        if message != self._status:
            self._status = message
            logging.info('UI 人体跟随：%s',message)
            self.status_changed.emit(message)

    def _state(self):
        self.state_changed.emit(self._ready,self.motion_active,self._closing)

    def open(self):
        if self.active:
            return False
        self._opened = True
        self._restore_camera = True
        try:
            import cv2
            from deploy.person_follow import FollowCamera,detector
            from deploy.person_follow_controller import PersonFollowController
            from voice.near_approach import body_projection
            self._people = self._people if self._people is not None else detector
            self._project = self._project if self._project is not None else body_projection()
            self._opencv_threads = cv2.getNumThreads()
            cv2.setNumThreads(1)
            self.depth_widget.pause_capture()
            self._camera = (self._camera_factory or FollowCamera)()
            self._camera.start()
            self._camera.snapshot()  # Surface a start error before enabling anything.
            self._pool = ThreadPoolExecutor(max_workers=1,thread_name_prefix='ui-follow-preview')
            self._controller = (self._controller_factory or PersonFollowController)(
                self._camera.snapshot,self._receive,people=self._people,project=self._project)
            self._set_status('广角预览已打开，正在后台加载人物检测…')
            self._timer.start()
            self._state()
            return True
        except Exception:
            self.close()
            raise

    def _receive(self,message,**details):
        with self._lock:
            if self._closing:
                return
            if 'sample' in details:
                self._feedback_frame = details
            # Do not enqueue image arrays; keep only one latest feedback image.
            self._events.append((message,bool(details.get('stopped'))))

    def start_motion(self,speed=.35,distance=1.5):
        if not self._opened or self._closing or not self._ready or self.motion_active:
            return False
        if not self._controller.start(stand_off=distance,max_speed=speed,duration=120.):
            return False
        self._moving = True
        self._motion_ended = False
        self._set_status('正在锁定人物并启动跟随…')
        self._state()
        return True

    def stop_motion(self):
        if self._controller is not None:
            self._controller.cancel()
        if self.motion_active:
            self._set_status('正在停止跟随，保留预览…')

    def frame(self):
        return self._sequence,self._frame if self._opened and not self._closing else None

    def close(self,restore_camera=True):
        if not self.active:
            return True
        self._restore_camera = self._restore_camera and restore_camera
        self._closing = True
        self._ready = False
        self._frame = None
        if self._controller is not None:
            self._controller.cancel()
        if self._pending is not None:
            self._pending.cancel()
        self._set_status('正在停止人体跟随并释放设备…')
        self._state()
        self._timer.start()
        return False

    def _drain_close(self):
        if self._restoring:
            _,image = self.depth_widget.get_latest_color_sample()
            if image is not None or time.monotonic() >= self._restore_deadline or not self._restore_camera:
                self._finish_close()
            return
        if self._controller is not None and self._controller.active:
            return
        if self._pending is not None and not self._pending.done():
            return
        if self._camera is not None:
            self._camera.close()
            self._camera = None
        if self._pool is not None:
            self._pool.shutdown(wait=False,cancel_futures=True)
            self._pool = None
        people,self._people = self._people,None
        if people is not None:
            try:
                people.close()
            except Exception:
                logging.exception('释放 UI 人物检测进程失败')
        if self._opencv_threads is not None:
            import cv2
            cv2.setNumThreads(self._opencv_threads)
            self._opencv_threads = None
        self._controller = self._pending = None
        self._opened = False
        if self._restore_camera:
            self._restoring = True
            self._restore_deadline = time.monotonic()+3.
            self.depth_widget.resume_capture()
        else:
            self._finish_close()

    def _finish_close(self):
        self._closing = self._restoring = self._moving = False
        self._timer.stop()
        self._state()
        self.closed.emit()

    def _poll(self):
        try:
            if self._closing:
                self._drain_close()
                return
            self._poll_preview()
        except Exception as error:
            self._set_status('人体跟随出错：'+str(error))
            self.close()

    def _poll_preview(self):
        from deploy.person_follow import analyze_preview,draw_result
        with self._lock:
            events = list(self._events)
            self._events.clear()
            feedback = self._feedback_frame
        for message,stopped in events:
            self._set_status(message)
            if stopped:
                self._motion_ended = True
        if self._moving and not self._controller.active:
            self._moving = False
            self._state()
        if not self.motion_active and self._pending is not None and self._pending.done():
            self._last_result = self._pending.result()
            self._pending = None
            if not self._ready:
                self._ready = True
                self._state()
                self._set_status('人物检测已就绪，确认目标后点击“开始跟随”。')
            if not self._motion_ended:
                boxes,measurement = self._last_result[1:3]
                message = '预览就绪，点击“开始跟随”。' if measurement is not None else self._last_result[3]
                if message != self._preview_status:
                    self._preview_status,self._preview_status_since = message,time.monotonic()
                if time.monotonic()-self._preview_status_since >= .5:
                    self._set_status(message)
        sample = self._camera.snapshot()
        if sample is None or sample.captured_monotonic <= self._last_sample:
            return
        self._last_sample = sample.captured_monotonic
        self.depth_widget.show_external_snapshot(sample)
        frame_sample,boxes,measurement = sample,[],None
        caption = 'FOLLOW' if self.motion_active else ('PREVIEW' if self._ready else 'LOADING DETECTOR')
        if self.motion_active and feedback is not None and time.monotonic()-feedback['sample'].captured_monotonic <= .20:
            frame_sample,boxes,measurement = feedback['sample'],[feedback['box']],feedback['measurement']
            self.metrics_changed.emit(f'目标距离 {measurement.distance:.2f} m · 前进速度 {feedback["vx"]:.2f} m/s')
        elif not self.motion_active and self._last_result is not None and time.monotonic()-self._last_result[0].captured_monotonic <= .20:
            frame_sample,boxes,measurement = self._last_result[:3]
            if measurement is not None:
                self.metrics_changed.emit(f'预览测距 {measurement.distance:.2f} m · 尚未启动跟随')
        self._frame = draw_result(frame_sample,boxes,measurement,caption,show_controls=False)
        self._sequence += 1
        if not self.motion_active and self._pending is None:
            self._pending = self._pool.submit(analyze_preview,sample,dict(height=.25,rpy=(0.,0.,0.)),
                                               self._project,self._people)
