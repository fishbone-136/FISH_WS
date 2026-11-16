"""Explicit relative command: finish closing Nav2 before allowing SDK control."""
import time

from PyQt5.QtCore import QObject, QTimer


class RelativeMotionHandoff(QObject):
    def __init__(self, page_factory, execute, update, mode_allowed, parent=None, *, idle=None):
        super().__init__(parent)
        self._page_factory, self._execute, self._update = page_factory, execute, update
        self._mode_allowed = mode_allowed
        if idle is None:
            from deploy.text_motion_controller import navigation_is_idle
            idle = navigation_is_idle
        self._idle = idle
        self._pending = None
        self._page = None
        self._deadline = 0.
        self._timer = QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._poll)

    @property
    def active(self):
        return self._pending is not None

    def request(self, actions, warnings):
        if self.active or not self._mode_allowed():
            return False
        try:
            page = self._page_factory()
            if page._control_pending:
                raise ValueError('导航正在切换，请等待完成后重新发送动作。')
            if not page._capabilities.get('online'):
                raise ValueError('导航状态尚未确认，请等待连接恢复，或在导航页关闭导航后重新发送。')
            if (page._capabilities.get('goal_active') or page._goal_active or page._goal_submitting):
                raise ValueError('导航任务仍在执行，请先停止或等待到达后再发送转身、直行动作。')
            self._page, self._pending = page, (tuple(actions), tuple(warnings))
            self._deadline = time.monotonic()+25.
            self._update('waiting', '已识别动作序列，正在完整关闭导航；确认退出后执行，可发送“停止”取消。')
            page._close_navigation()
            self._timer.start()
        except Exception as error:
            self._finish('error', f'无法切换到文字动作：{error}')
            return False
        return True

    def _poll(self):
        if not self.active:
            return
        if not self._mode_allowed():
            self.cancel()
            return
        page = self._page
        if page._close_failed:
            self._finish('error', '导航未完全关闭，动作已取消；请检查导航关闭状态后重新发送。')
            return
        if time.monotonic() > self._deadline:
            self._finish('error', '等待导航关闭超时，动作已取消，不会在稍后自动补发。')
            return
        if not page._navigation_closed or page._control_pending:
            return
        if page._launch_process.state() or page.worker.isRunning() or not self._idle():
            return
        actions, warnings = self._pending
        self._pending = None
        self._timer.stop()
        self._update('ready', '导航已完全关闭，正在下发已确认的动作序列。')
        # Execute on this Qt turn. No delayed SDK replay can outlive cancellation.
        try:
            self._execute(actions, warnings)
        except Exception as error:
            self._finish('error', f'文字动作启动失败，未保留待执行任务：{error}')

    def _finish(self, state, message):
        self._pending = None
        self._timer.stop()
        self._update(state, message)

    def cancel(self):
        if self.active:
            self._finish('canceled', '已取消等待中的文字动作，导航关闭后不会自动补发。')
