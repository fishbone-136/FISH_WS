"""Qt-side visual navigation lifecycle; all robot motion stays in Nav2."""
import time
import uuid

from PyQt5.QtCore import QObject, QTimer

from voice.visual_navigation import revalidate_target, refresh_grounded_target


class VisualNavigationSession(QObject):
    def __init__(self, page_factory, snapshot_provider, locate, update, mode_allowed, parent=None):
        super().__init__(parent)
        self._page_factory = page_factory
        self._snapshot_provider = snapshot_provider
        self._locate = locate
        self._update = update
        self._mode_allowed = mode_allowed
        self._page = self._worker = None
        self._request_id = ''
        self._stage = ''
        self._intent = None
        self._deadline = 0.
        self._grounded = None
        self._grounding_inflight = self._nav_prepared = False
        self._grounding_round = 0
        self._locate_ticket = ''
        self._retry_at = 0.
        self._parallel_grounding = False
        self._timer = QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._poll)

    @property
    def active(self):
        return bool(self._request_id)

    def request(self, intent):
        if self.active:
            self._update('notice', '已有视觉移动任务，请先停止或等待完成。')
            return False
        self._request_id = uuid.uuid4().hex
        self._intent = intent
        self._grounded = None
        self._grounding_inflight = self._nav_prepared = False
        self._grounding_round = 0
        self._locate_ticket = ''
        self._retry_at = 0.
        self._parallel_grounding = False
        self._stage = 'camera'
        self._deadline = time.monotonic()+5.
        self._update('starting', f'正在检查相机标定，准备定位“{intent.target}”，保留 {intent.stand_off_m:g} 米距离。')
        try:
            if self._snapshot_provider() is None:
                raise ValueError('没有同步对齐的 D435i 实时彩色和深度画面，未执行移动。')
            self._page = self._page_factory()
            if self._page._control_pending or self._page._close_failed:
                raise ValueError('导航正在切换或上次关闭失败，请先处理导航状态。')
            prepare=getattr(self._page,'prepare_visual_bridge',None)
            if prepare is not None and not prepare():
                raise ValueError('显示连接仍在退出，请稍后重试。')
            self._worker = self._page.worker
            # A restarted navigation bridge is a new QObject. Disconnect the old one.
            if getattr(self, '_connected_worker', None) is not self._worker:
                old = getattr(self, '_connected_worker', None)
                if old is not None:
                    try:
                        old.visual_goal_updated.disconnect(self._on_navigation)
                    except (TypeError, RuntimeError):
                        pass
                self._worker.visual_goal_updated.connect(self._on_navigation)
                self._connected_worker = self._worker
            self._timer.start()
            self._poll()
        except Exception as error:
            self._finish('rejected', f'无法准备视觉导航：{error}')
        return self.active

    def _poll(self):
        if not self.active:
            return
        if not self._mode_allowed():
            self.cancel()
            return
        if time.monotonic() > self._deadline:
            if self._stage=='camera':
                capabilities=getattr(self._page,'_capabilities',{})
                self._finish('rejected',capabilities.get('camera_error') or '相机标定未就绪，请配置 D435i 到机身的实测变换后重试。')
                return
            self.cancel('视觉移动任务超时，已请求取消，请检查导航状态。')
            return
        page = self._page
        if page._control_pending == 'close' or page._navigation_closed:
            self.cancel('导航已关闭，视觉移动任务已取消。')
            return
        if page._navigation_state in ('error', 'fault', 'offline'):
            self.cancel('导航不可用：'+page._last_navigation_message)
            return
        if self._stage=='camera':
            if not getattr(page,'_capabilities',{}).get('camera_calibrated'):
                return
            self._stage='starting'
            self._deadline=time.monotonic()+95.
            self._update('starting','相机安装标定已确认，正在准备导航…')
            if page._navigation_state == 'ready':
                self._update('starting', '复用已就绪的导航，正在快速定位目标…')
            else:
                page._start_stack(background=True)
                self._begin_grounding()
        if not self.active:
            return
        if self._retry_at and time.monotonic() >= self._retry_at:
            self._retry_at = 0.
            self._begin_grounding()
        if self._stage == 'starting' and page._navigation_state == 'ready':
            self._stage = 'preflight'
            self._worker.enqueue('visual_preflight', self._request_id)

    def _on_navigation(self, request_id, state, message):
        if request_id != self._request_id or not self.active:
            return
        if state == 'prepared':
            if self._stage != 'preflight':
                return
            self._stage = 'locating'
            self._deadline = time.monotonic()+35.
            self._nav_prepared = True
            self._update('locating', '导航已就绪，正在复核并行识别结果及实时深度…' if self._grounding_round else message)
            if self._grounded is not None:
                self._dispatch_target()
            elif not self._grounding_inflight:
                self._begin_grounding()
        elif state in ('rejected', 'succeeded', 'finished'):
            if state == 'succeeded':
                message = f'已到达“{self._intent.target}”前方的停靠点，设定距离 {self._intent.stand_off_m:g} 米。'
            self._finish(state, message)
        elif state == 'accepted':
            self._stage = 'running'
            self._deadline = time.monotonic()+180.
            self._update(state, message)
        elif state == 'feedback':
            self._update(state, message)

    def target_ready(self, target):
        if (not self.active or target.request_id != self._locate_ticket
                or self._stage not in ('starting', 'preflight', 'locating')):
            return
        if not self._mode_allowed():
            self.cancel()
            return
        self._grounding_inflight = False
        self._grounded = target
        if self._nav_prepared:
            self._dispatch_target()
        else:
            self._update('starting', '目标已识别，正在等待导航定位就绪；就绪后复核实时深度。')

    def _begin_grounding(self):
        if not self.active or self._grounding_inflight:
            return
        try:
            if self._grounding_round >= 2:
                raise ValueError('初始化期间目标或画面变化较大，请保持静止后重新发送。')
            snapshot = self._snapshot_provider()
            if snapshot is None:
                raise ValueError('没有同步对齐的 D435i 实时画面，未执行移动。')
            self._grounding_round += 1
            self._parallel_grounding = not self._nav_prepared
            self._locate_ticket = self._request_id if self._grounding_round==1 else self._request_id+':refresh'
            self._grounding_inflight = True
            if not self._locate(snapshot, self._intent, self._locate_ticket):
                self._grounding_inflight = False
                self._grounding_round -= 1
                self._retry_at = time.monotonic()+.2
            elif self.active and not self._nav_prepared:
                self._update('starting', '正在并行准备导航与识别目标，暂不发送移动。')
        except Exception as error:
            self._finish('rejected', str(error))

    def _dispatch_target(self):
        try:
            target = self._grounded
            current = self._snapshot_provider()
            if current is None:
                raise ValueError('D435i 实时画面已中断，未执行移动。')
            if self._parallel_grounding:
                try:
                    target = refresh_grounded_target(target, current, self._request_id)
                except ValueError:
                    self._grounded = None
                    self._update('locating', '画面已有变化，正在使用导航就绪后的新画面重新定位…')
                    self._begin_grounding()
                    return
            else:
                revalidate_target(target, current)
                # Fresh grounding still carries the refresh correlation ticket.
                from dataclasses import replace
                target = replace(target, request_id=self._request_id)
            self._stage = 'goal'
            self._deadline = time.monotonic()+12.
            self._update('sending', f'已定位“{target.intent.target}”，正在检查可达停靠点…')
            self._worker.enqueue('visual_goal', target)
        except Exception as error:
            self._finish('rejected', str(error))

    def grounding_failed(self, request_id, message):
        if self.active and self._locate_ticket == request_id and self._grounding_inflight:
            self._finish('rejected', message)

    def cancel(self, message='已请求取消视觉移动任务。'):
        if not self.active:
            return
        if self._stage in ('goal', 'running') and self._worker is not None:
            self._worker.enqueue('cancel_visual', self._request_id)
        self._finish('finished', message)

    def _finish(self, state, message):
        self._timer.stop()
        self._request_id = ''
        self._stage = ''
        self._intent = None
        self._grounded = None
        self._grounding_inflight = self._nav_prepared = False
        self._retry_at = 0.
        self._update(state, message)
