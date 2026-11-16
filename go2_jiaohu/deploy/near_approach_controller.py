"""Exclusive SDK visual approach with fresh perception and short motor leases."""
import logging
import math
import threading
import time

from deploy.text_motion_controller import navigation_is_idle
from voice import config
from voice.near_approach import (NearUnavailable, body_projection, valid_state, appearance,
    track_person, target_in_body, near_limits, corridor_clear, approach_velocity)
from voice.person_detector import detector, person_side, select_person, PersonNotFound
from voice.robot_interface import Go2RobotController


class NearApproachController:
    def __init__(self, snapshot, update, *, robot=None, people=None, state_factory=None, guard=None):
        self._snapshot, self._update = snapshot, update
        self._robot = robot or Go2RobotController(auto_connect=False)
        self._detector = people or detector
        self._state_factory = state_factory
        self._guard = guard or navigation_is_idle
        self._stop = threading.Event()
        self._thread = None

    @property
    def active(self):
        return self._thread is not None and self._thread.is_alive()

    def start(self, intent, ticket, allow_fallback):
        if self.active:
            return False
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, args=(intent, ticket, allow_fallback),
            daemon=True, name='visual-sdk-approach')
        self._thread.start()
        return True

    def cancel(self):
        self._stop.set()

    def close(self):
        self.cancel()
        if self._thread is not None:
            self._thread.join(timeout=8.)
        return not self.active

    def _run(self, intent, ticket, allow_fallback):
        state_monitor = None
        moved = connected = False
        state, message = 'rejected', '视觉靠近未完成'
        started = time.monotonic()
        def notify(status, text):
            logging.info('SDK 视觉靠近 [%s] %s: %s', ticket, status, text)
            self._update(ticket, status, text)
        try:
            if not self._guard() or config.ROBOT_MOTION_BACKEND != 'sdk':
                raise NearUnavailable('导航正在运行或 SDK 后端不可用；近距 SDK 模式需要先完整关闭导航')
            if person_side(intent.target) is None:
                raise NearUnavailable('此目标需要视觉模型和导航规划')
            project = body_projection()
            notify('locating', '正在快速定位人物，检查近距直线路径…')
            # Warm detector before capturing the first motion-authorizing image.
            try:
                self._detector.detect(self._require_snapshot().color)
            except (RuntimeError, OSError, TimeoutError) as error:
                raise NearUnavailable(f'人物快速检测不可用：{error}') from error
            if self._state_factory is None:
                from deploy.robot_state_monitor import RobotStateMonitor
                state_monitor = RobotStateMonitor()
            else:
                state_monitor = self._state_factory()
            deadline = time.monotonic()+3.
            while True:
                if self._stop.is_set():
                    return
                try:
                    robot_state = state_monitor.get()
                    valid_state(robot_state, stationary=True)
                    break
                except ValueError as error:
                    if time.monotonic() >= deadline:
                        raise ValueError(f'近距控制未就绪：{error}') from error
                    self._stop.wait(.05)
            sample = self._require_snapshot()
            boxes = self._detector.detect(sample.color)
            box = select_person(boxes, person_side(intent.target))
            point = target_in_body(sample, box, project)
            near_limits(point, intent.stand_off_m)
            corridor_clear(sample, robot_state, project, point, intent.stand_off_m)
            original = appearance(sample.color, box)
            previous_distance = math.hypot(*point[:2])
            if self._stop.is_set() or not self._guard():
                return
            if not self._robot.reconnect():
                raise RuntimeError('SDK 连接失败：'+self._robot.last_error)
            connected = True
            notify('sending', f'近距路径检查通过，使用 SDK 深度闭环靠近“{intent.target}”。')
            origin = robot_state['position']
            deadline = time.monotonic()+30.
            settled = 0
            last_frame = last_progress = 0.
            last_cycle, last_vx, travelled = time.monotonic(), 0., 0.
            while not self._stop.is_set():
                cycle = time.monotonic()
                travelled += last_vx*(cycle-last_cycle)
                last_cycle = cycle
                if travelled > 3.2:
                    raise ValueError('已达到近距速度积分上限，已停止')
                if cycle >= deadline:
                    raise ValueError('近距靠近超时，已停止')
                if not self._guard():
                    raise ValueError('检测到导航控制源，已停止 SDK 靠近')
                robot_state = state_monitor.get()
                valid_state(robot_state)
                if math.hypot(robot_state['position'][0]-origin[0], robot_state['position'][1]-origin[1]) > 3.2:
                    raise ValueError('已达到近距移动上限，已停止')
                sample = self._require_snapshot()
                if sample.captured_monotonic <= last_frame:
                    raise ValueError('RGB-D 帧未更新，已停止')
                last_frame = sample.captured_monotonic
                boxes = self._detector.detect(sample.color)
                box = track_person(boxes, box, sample.color, original)
                point = target_in_body(sample, box, project)
                distance = near_limits(point, intent.stand_off_m)
                if distance < intent.stand_off_m-.15:
                    raise ValueError('人物已进入设定距离以内，已停止靠近')
                if abs(distance-previous_distance) > .25:
                    raise ValueError('目标距离发生跳变，已停止')
                previous_distance = distance
                corridor_clear(sample, robot_state, project, point, intent.stand_off_m)
                vx, wz = approach_velocity(point, intent.stand_off_m)
                # Recheck after inference and geometry, immediately before issuing a lease.
                if self._stop.is_set():
                    break
                valid_state(state_monitor.get())
                if time.monotonic()-sample.captured_monotonic > .20 or not self._guard():
                    raise ValueError('感知反馈过期或控制源变化，已停止')
                if not self._robot.send_visual_velocity(vx, wz):
                    raise RuntimeError('SDK 速度下发失败：'+self._robot.last_error)
                moved = True
                last_vx = vx
                settled = settled+1 if vx == 0. and abs(distance-intent.stand_off_m) <= .12 else 0
                if settled >= 3:
                    state, message = 'succeeded', f'深度反馈确认已停在“{intent.target}”前约 {distance:.2f} 米。'
                    break
                if cycle-last_progress > 1.:
                    notify('feedback', f'SDK 闭环靠近中，当前距离约 {distance:.2f} 米，速度 {vx:.2f} 米/秒。')
                    last_progress = cycle
                self._stop.wait(max(0., .10-(time.monotonic()-cycle)))
            else:
                state, message = 'finished', 'SDK 视觉靠近已停止。'
        except (NearUnavailable, PersonNotFound) as error:
            state = 'fallback' if allow_fallback and not moved and not self._stop.is_set() else 'rejected'
            message = str(error)
        except Exception as error:
            state, message = 'rejected', str(error)
        finally:
            stop_ok = True
            if connected:
                try:
                    stop_ok = self._robot.execute_action('stop', speed=0., duration=0.)
                except Exception:
                    stop_ok = False
                try:
                    self._robot.shutdown()
                except Exception:
                    stop_ok = False
            if state_monitor is not None:
                state_monitor.close()
            if self._stop.is_set():
                state, message = 'finished', 'SDK 视觉靠近已停止。'
            if not stop_ok:
                state, message = 'rejected', 'SDK 停止回执异常，请检查机器狗状态。'
            logging.info('SDK 视觉靠近耗时 %.3fs，已发送速度=%s', time.monotonic()-started, moved)
            notify(state, message)

    def _require_snapshot(self):
        sample = self._snapshot()
        if sample is None or time.monotonic()-sample.captured_monotonic > .2:
            raise ValueError('没有新鲜的同步 RGB-D 画面')
        return sample
