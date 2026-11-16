"""Qt handoff between exclusive near SDK control and shared Nav2 control."""
import uuid

from PyQt5.QtCore import QObject, pyqtSignal


class NearApproachSession(QObject):
    updated = pyqtSignal(str, str, str)

    def __init__(self, snapshot, update, fallback, mode_allowed, parent=None, *, controller_factory=None):
        super().__init__(parent)
        self._snapshot, self._update, self._fallback = snapshot, update, fallback
        self._mode_allowed = mode_allowed
        self._factory = controller_factory
        self._controller = None
        self._ticket = ''
        self._intent = None
        self._cancelled = False
        self.updated.connect(self._on_update)

    @property
    def active(self):
        return bool(self._ticket) or (self._controller is not None and self._controller.active)

    def request(self, intent, allow_fallback=True):
        if self.active:
            self._update('notice', '已有近距视觉任务，请先停止或等待完成。')
            return False
        if not self._mode_allowed():
            return False
        if self._factory is None:
            from deploy.near_approach_controller import NearApproachController
            factory = NearApproachController
        else:
            factory = self._factory
        self._ticket, self._intent = uuid.uuid4().hex, intent
        self._cancelled = False
        self._controller = factory(self._snapshot, self.updated.emit)
        self._update('starting', '正在检查是否可以使用近距 SDK 快速靠近…')
        self._controller.start(intent, self._ticket, allow_fallback)
        return True

    def _on_update(self, ticket, state, message):
        if ticket != self._ticket:
            return
        if self._cancelled:
            if state not in ('finished', 'rejected', 'succeeded', 'fallback'):
                return
            state, message = 'finished', 'SDK 视觉靠近已取消。'
        if state in ('finished', 'rejected', 'succeeded', 'fallback'):
            self._ticket = ''
            if state == 'fallback':
                if self._mode_allowed():
                    self._update('starting', message+'；正在切换到导航规划…')
                    self._fallback(self._intent)
                return
        self._update(state, message)

    def cancel(self):
        self._cancelled = True
        if self._controller is not None:
            return self._controller.close()
        return True
