"""Qt ownership for relative behaviors while keeping the current Nav2 stack warm."""
import time
import uuid

from PyQt5.QtCore import QObject, QTimer


class RelativeNavigationSession(QObject):
    def __init__(self, page_factory, update, mode_allowed, parent=None):
        super().__init__(parent)
        self._page_factory, self._update, self._mode_allowed = page_factory, update, mode_allowed
        self._ticket = ''
        self._worker = self._page = None
        self._sent = False
        self._canceling = False
        self._actions = ()
        self._deadline = 0.
        self._timer = QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._poll)

    @property
    def active(self):
        return bool(self._ticket)

    def request(self, actions, warnings):
        if self.active:
            self._update('notice', '已有导航相对动作，请先停止或等待完成。')
            return False
        try:
            page = self._page_factory()
            if page._navigation_closed or page._control_pending or page._close_failed:
                raise ValueError('导航正在切换或已关闭，未发送相对动作。')
            if page._capabilities.get('goal_active') or page._goal_active or page._goal_submitting:
                raise ValueError('导航任务仍在执行，请先停止或等待完成。')
            worker = page.worker
            if worker is not self._worker:
                if self._worker is not None:
                    self._worker.relative_action_updated.disconnect(self._on_update)
                worker.relative_action_updated.connect(self._on_update)
                self._worker = worker
            self._page, self._actions = page, tuple(actions)
            self._ticket = uuid.uuid4().hex
            self._sent = self._canceling = False
            self._deadline = time.monotonic()+95.
            self._update('starting', '复用现有导航执行转向/直行，保持雷达定位和地图，无需重新初始化。')
            for warning in warnings:
                self._update('notice', '动作限制：'+warning)
            if page._navigation_state == 'paused':
                page._start_stack(background=True)
            self._timer.start()
            self._poll()
        except Exception as error:
            self._finish('rejected', str(error))
            return False
        return self.active

    def _poll(self):
        if not self.active:
            return
        if not self._mode_allowed() and not self._canceling:
            self.cancel()
            return
        if self._page._navigation_closed:
            self._finish('finished', '导航已关闭，相对动作已停止。')
            return
        if self._page._navigation_state in ('error','fault','offline'):
            if self._sent:
                self._worker.enqueue('cancel_relative', self._ticket)
            self._finish('rejected', '导航连接或定位异常，已请求停止：'+self._page._last_navigation_message)
            return
        if time.monotonic() > self._deadline and not self._canceling:
            self.cancel()
            return
        if not self._sent and self._page._navigation_state == 'ready':
            self._sent = True
            self._deadline = time.monotonic()+90.
            self._worker.enqueue('relative_sequence', self._ticket, self._actions)

    def _on_update(self, ticket, state, message):
        if ticket != self._ticket:
            return
        if state in ('succeeded','finished','rejected'):
            self._finish(state, message)
        elif not self._canceling:
            self._update(state, message)

    def cancel(self):
        if not self.active:
            return
        if self._sent:
            self._canceling = True
            self._worker.enqueue('cancel_relative', self._ticket)
            self._update('feedback', '已请求停止导航相对动作，等待取消回执；不会切换 SDK。')
        else:
            self._finish('finished', '已取消待执行的导航相对动作。')

    def _finish(self, state, message):
        self._timer.stop()
        self._ticket = ''
        self._update(state, message)
