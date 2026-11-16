"""ROS-thread-only behavior sequencing, using the existing Nav2 velocity lane."""
import math
import time
import uuid

from action_msgs.msg import GoalStatus
from nav2_msgs.action import Spin, DriveOnHeading, BackUp
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from unique_identifier_msgs.msg import UUID

from voice.navigation_actions import navigation_steps


class RelativeNavigationExecutor:
    def __init__(self, worker, *, clients=None, activity=None):
        self.worker = worker
        node = worker._node
        self.clients = clients if clients is not None else {
            'spin': ActionClient(node, Spin, 'spin'),
            'forward': ActionClient(node, DriveOnHeading, 'drive_on_heading'),
            'backup': ActionClient(node, BackUp, 'backup')}
        self.activity = activity if activity is not None else node.create_publisher(Bool, '/go2/relative_navigation_active', 1)
        self.owner = ''
        self.steps = ()
        self.index = 0
        self.handle = None
        self.pending = False
        self.cancel_requested = False
        self.deadline = 0.
        self.next_step_at = 0.
        self.completed_at = 0.
        self.own_ids = set()
        self._heartbeat_at = 0.

    @property
    def active(self):
        return bool(self.owner)

    def emit(self, state, message):
        self.worker.relative_action_updated.emit(self.owner, state, message)

    def start(self, owner, actions):
        try:
            if self.active:
                raise ValueError('已有导航相对动作正在执行')
            self.worker._relative_preflight()
            steps = navigation_steps(actions)
            if any(not self.clients[step.kind].server_is_ready() for step in steps):
                raise ValueError('导航动作服务尚未就绪，未切换 SDK；请稍后重试。')
        except Exception as error:
            self.worker.relative_action_updated.emit(owner, 'rejected', str(error))
            return
        self.owner, self.steps, self.index = owner, steps, 0
        self.handle, self.pending, self.cancel_requested = None, False, False
        self.completed_at = 0.
        self._activity(True)
        self.emit('starting', '复用现有导航执行相对动作，保留定位和地图，不切换 SDK。')
        self._send_next()

    def _activity(self, active):
        self.activity.publish(Bool(data=active))
        self._heartbeat_at = time.monotonic()

    def _send_next(self):
        if not self.active or self.cancel_requested:
            return
        if self.index >= len(self.steps):
            self._finish('succeeded', '导航相对动作已完成，定位和地图保持就绪。')
            return
        try:
            self.worker._relative_preflight(owned=True)
            step = self.steps[self.index]
            self.worker._validate_relative_path(step)
            if step.kind == 'spin':
                goal = Spin.Goal()
                goal.target_yaw = step.quantity
                seconds = abs(step.quantity)/.4+8.
                description = f'原地转向 {math.degrees(step.quantity):g}°'
            else:
                goal = DriveOnHeading.Goal() if step.kind=='forward' else BackUp.Goal()
                goal.target.x, goal.speed = step.quantity, step.speed
                seconds = abs(step.quantity)/step.speed+8.
                description = f'{"前进" if step.kind=="forward" else "后退"} {abs(step.quantity):g} 米'
            goal.time_allowance.sec = math.ceil(seconds)
            self.pending = True
            self.deadline = time.monotonic()+3.
            owner, index = self.owner, self.index
            goal_id = UUID(uuid=list(uuid.uuid4().bytes))
            self.own_ids.add(bytes(goal_id.uuid))
            future = self.clients[step.kind].send_goal_async(goal,
                goal_uuid=goal_id,
                feedback_callback=lambda msg: self._feedback(owner, index, msg))
            future.add_done_callback(lambda result: self._accepted(owner, index, result, seconds+3.))
            self.emit('sending', f'正在执行第 {self.index+1}/{len(self.steps)} 步：{description}。')
        except Exception as error:
            self.pending = False
            self._finish('rejected', f'相对动作未继续：{error}')

    def _accepted(self, owner, index, future, allowance):
        handle = None
        try:
            handle = future.result()
            if owner != self.owner or index != self.index:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            self.pending = False
            if not handle.accepted:
                self._finish('rejected', 'Nav2 拒绝了相对动作，未继续后续步骤。')
                return
            self.handle = handle
            self.own_ids.add(bytes(handle.goal_id.uuid))
            self.deadline = time.monotonic()+allowance
            result = handle.get_result_async()
            result.add_done_callback(lambda response: self._result(owner, index, handle, response))
            if self.cancel_requested:
                handle.cancel_goal_async()
            else:
                self.emit('accepted', f'Nav2 已接受第 {index+1}/{len(self.steps)} 步，按实际转角/位移完成。')
        except Exception as error:
            if owner != self.owner or index != self.index:
                return
            self.cancel_requested = True
            if handle is not None and handle.accepted:
                self.handle = handle
                handle.cancel_goal_async()
            # Retain ownership if the result subscription failed after acceptance.
            if self.handle is None:
                self._finish('rejected', f'相对动作连接失败：{error}')
            else:
                self.emit('rejected', f'动作回执异常，已请求取消：{error}')

    def _result(self, owner, index, handle, future):
        if owner != self.owner or index != self.index or self.handle is not handle:
            return
        self.handle = None
        try:
            status = future.result().status
            if self.cancel_requested or status != GoalStatus.STATUS_SUCCEEDED:
                self._finish('finished' if self.cancel_requested else 'rejected', '导航相对动作已停止，未执行剩余步骤。')
                return
            self.index += 1
            self.completed_at = time.monotonic()
            # Allow one executor turn for terminal action status / final zero velocity.
            self.next_step_at = time.monotonic()+.10
        except Exception as error:
            self._finish('rejected', f'相对动作结果异常，未继续：{error}')

    def _feedback(self, owner, index, msg):
        if owner != self.owner or index != self.index or self.cancel_requested:
            return
        feedback = msg.feedback
        text = (f'已旋转 {math.degrees(feedback.angular_distance_traveled):.0f}°'
                if self.steps[index].kind=='spin' else f'已移动 {feedback.distance_traveled:.2f} 米')
        self.emit('feedback', f'第 {index+1}/{len(self.steps)} 步：{text}。')

    def cancel(self, owner):
        if not self.active or owner != self.owner:
            return
        self.cancel_requested = True
        if self.handle is not None:
            self.handle.cancel_goal_async()
        elif not self.pending:
            self._finish('finished', '已取消待执行的导航相对动作。')

    def tick(self):
        if not self.active:
            return
        now = time.monotonic()
        if now-self._heartbeat_at >= .2:
            self._activity(True)
        try:
            self.worker._relative_preflight(owned=True)
        except Exception as error:
            if not self.cancel_requested:
                self.emit('rejected', f'定位或控制状态发生变化，已请求停止：{error}')
                self.cancel(self.owner)
        if self.pending or self.handle is not None:
            if now > self.deadline and not self.cancel_requested:
                self.emit('rejected', '导航相对动作超时，已请求取消，不会切换 SDK 补发。')
                self.cancel(self.owner)
            return
        if not self.cancel_requested and now >= self.next_step_at:
            if self.completed_at and not self.worker._relative_stationary(self.completed_at):
                if now-self.completed_at > 3.:
                    self._finish('rejected', '前一步结束后未取得停稳反馈，未继续后续动作。')
                return
            self._send_next()

    def _finish(self, state, message):
        owner = self.owner
        self._activity(False)
        self.owner, self.steps, self.handle, self.pending = '', (), None, False
        self.next_step_at = 0.
        self.worker._last_status_at = 0.
        self.worker.relative_action_updated.emit(owner, state, message)

    def close(self):
        if self.active:
            self.cancel(self.owner)
        for client in self.clients.values():
            client.destroy()
