"""Standalone SDK following lifecycle. No ROS, LIO, Nav2, or fallback."""
import logging
import math
import os
from pathlib import Path
import threading
import time

from voice import config
from voice.near_approach import body_projection, valid_state
from voice.person_detector import detector
from voice.person_follow import FollowFilter, depth_cloud, check_path, PersonTrack, TargetUncertain
from voice.robot_interface import Go2RobotController


def sdk_control_available():
    """Read process identities only; refuse local competing motion owners."""
    names_to_block = {'run_lio_navigation.sh', 'go2_lio_navigation.launch.py',
        'go2_base', 'scan_safety_gate.py', 'idle_stand_guard.py', 'vel_ctrl_repub',
        'controller_server', 'nav2_container'}
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            if int(proc.name) == os.getpid() or proc.stat().st_uid != os.getuid():
                continue
            fields = (proc/'stat').read_text().rsplit(')', 1)[1].split()
            if fields[0] == 'Z':
                continue
            args = [x.decode(errors='replace') for x in (proc/'cmdline').read_bytes().split(b'\0') if x]
            if any(os.path.basename(x) in names_to_block for x in args) or '__node:=nav2_container' in args:
                return False
            if 'voice.sdk_worker' in args and int(fields[1]) != os.getpid():
                return False
        except (OSError, ValueError, IndexError):
            continue
    return True


class ControlGuard:
    """Scan at most 4 Hz; repeated calls must not eat the camera frame budget."""
    def __init__(self, scan=sdk_control_available, clock=time.monotonic):
        self._scan, self._clock = scan, clock
        self._checked = -math.inf
        self._allowed = False

    def __call__(self):
        if self._clock()-self._checked >= .25:
            self._allowed = bool(self._scan())
            self._checked = self._clock()
        return self._allowed


class PersonFollowController:
    def __init__(self, snapshot, update, *, robot=None, people=None, guard=None, project=None):
        self._snapshot, self._update = snapshot, update
        self._robot = robot if robot is not None else Go2RobotController(auto_connect=False)
        self._people = people if people is not None else detector
        self._guard = guard if guard is not None else ControlGuard()
        self._project = project
        self._stop = threading.Event()
        self._thread = None

    @property
    def active(self):
        return self._thread is not None and self._thread.is_alive()

    def start(self, stand_off=1.5, max_speed=.35, duration=120.):
        if self.active:
            return False
        feedback = FollowFilter(stand_off, max_speed)
        if not math.isfinite(duration) or not 1. <= duration <= 600.:
            raise ValueError('单次跟随时长应在 1–600 秒之间')
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, args=(feedback, duration),
            name='sdk-person-follow', daemon=True)
        self._thread.start()
        return True

    def cancel(self):
        self._stop.set()

    def close(self):
        self.cancel()
        if self._thread is not None:
            self._thread.join(timeout=10.)
        return not self.active

    def _notify(self, message, **details):
        self._update(message, **details)

    def _sample(self, previous=-1.):
        sample = self._snapshot()
        if (sample is None or not 0. <= time.monotonic()-sample.captured_monotonic <= .20
                or sample.captured_monotonic <= previous):
            raise ValueError('同步 RGB-D 画面已过期或未更新')
        return sample

    def _run(self, feedback, duration):
        connected = False
        message = '跟随已停止，按空格重新锁定。'
        try:
            if config.ROBOT_MOTION_BACKEND != 'sdk':
                raise ValueError('跟随仅支持 SDK，请设置 GO2_MOTION_BACKEND=sdk')
            if not self._guard():
                raise ValueError('其他运动控制程序仍在运行，请先关闭')
            project = self._project if self._project is not None else body_projection()
            self._notify('正在预热检测并连接 SDK…')
            self._people.detect(self._sample().color)
            if self._stop.is_set() or not self._guard():
                return
            if not self._robot.reconnect():
                raise RuntimeError('SDK 连接失败：'+self._robot.last_error)
            connected = True
            deadline = time.monotonic()+3.
            while not self._stop.is_set():
                try:
                    valid_state(self._robot.read_sport_state(), stationary=True)
                    break
                except ValueError:
                    if time.monotonic() >= deadline:
                        raise
                    self._stop.wait(.05)
            tracker = PersonTrack()
            last_frame = -1.
            missing_since = None
            last_log = -math.inf
            last_yaw = None
            deadline = time.monotonic()+duration
            while not self._stop.is_set() and time.monotonic() < deadline:
                cycle = time.monotonic()
                guard_started = cycle
                if not self._guard():
                    raise ValueError('检测到其他运动控制源，已停止')
                guard_ms = (time.monotonic()-guard_started)*1000
                state = self._robot.read_sport_state()
                try:
                    valid_state(state)
                except ValueError as error:
                    raise ValueError(f'{error}（roll={math.degrees(state["rpy"][0]):.1f}°，'
                                     f'pitch={math.degrees(state["rpy"][1]):.1f}°）') from error
                imu_yaw_rate = None
                if last_yaw is not None:
                    interval = state['observed_at']-last_yaw[0]
                    if .02 <= interval <= .5:
                        change = state['rpy'][2]-last_yaw[1]
                        imu_yaw_rate = math.atan2(math.sin(change),math.cos(change))/interval
                        if abs(imu_yaw_rate) > 1.5:
                            imu_yaw_rate = None
                last_yaw = (state['observed_at'],state['rpy'][2])
                sample = self._sample(last_frame)
                last_frame = sample.captured_monotonic
                detect_started = time.monotonic()
                boxes = self._people.detect(sample.color)
                detect_ms = (time.monotonic()-detect_started)*1000
                geometry_started = time.monotonic()
                cloud = depth_cloud(sample, state, project)
                try:
                    box,measurement = tracker.update(boxes,sample,cloud,project)
                except TargetUncertain as error:
                    if missing_since is None:
                        missing_since = time.monotonic()
                        self._notify(f'人物短暂漏检或失配：{error}；已停车等待（最多 1.0 秒）')
                    if not self._robot.send_follow_velocity(0.,0.):
                        raise RuntimeError('漏检停车速度下发失败：'+self._robot.last_error)
                    feedback.reset()
                    if time.monotonic()-missing_since >= 1.:
                        raise ValueError(f'人物持续丢失或无法确认：{error}；已停止')
                    self._stop.wait(max(0.,.10-(time.monotonic()-cycle)))
                    continue
                vx, wz = feedback.velocity(measurement, sample.captured_monotonic,
                                            yaw_speed=imu_yaw_rate)
                check_path(cloud, vx, wz)
                geometry_ms = (time.monotonic()-geometry_started)*1000
                # Recheck after detection and projection, just before a motor lease.
                valid_state(self._robot.read_sport_state())
                if self._stop.is_set():
                    break
                guard_started = time.monotonic()
                if not self._guard():
                    raise ValueError('检测到其他运动控制源，已停止')
                guard_ms += (time.monotonic()-guard_started)*1000
                age_ms = (time.monotonic()-sample.captured_monotonic)*1000
                if age_ms > 200.:
                    raise ValueError(f'感知反馈超过 200ms：帧龄 {age_ms:.0f}ms，检测 {detect_ms:.0f}ms，'
                                     f'几何 {geometry_ms:.0f}ms，控制检查 {guard_ms:.0f}ms；已停止')
                if not self._robot.send_follow_velocity(vx, wz):
                    raise RuntimeError('SDK 速度下发失败：'+self._robot.last_error)
                if missing_since is not None:
                    self._notify('原目标已重新确认，恢复跟随')
                    missing_since = None
                if time.monotonic()-last_log >= 1.:
                    centre_bearing = measurement.image_bearing
                    if centre_bearing is None:
                        centre_bearing = math.atan2(measurement.point[1],measurement.point[0])
                    logging.info('跟随反馈：距离 %.2fm，画面偏角 %.1f°，vx=%.2f，wz=%.2f，'
                                 'SDK实测转速=%.2f，IMU转速=%s，'
                                 '帧龄 %.0fms（检测 %.0f / 几何 %.0f / 控制检查 %.0fms）',
                                 measurement.distance,math.degrees(centre_bearing),
                                 vx,wz,state['yaw_speed'],
                                 '未就绪' if imu_yaw_rate is None else f'{imu_yaw_rate:.2f}',
                                 age_ms,detect_ms,geometry_ms,guard_ms)
                    last_log = time.monotonic()
                self._notify('转向对准人物' if wz and not vx else ('跟随中' if vx else '已到跟随距离，等待目标继续走'),
                    sample=sample, box=box, measurement=measurement, vx=vx, wz=wz)
                self._stop.wait(max(0., .10-(time.monotonic()-cycle)))
            if not self._stop.is_set():
                message = '已达到单次跟随时长，按空格重新启动。'
        except Exception as error:
            message = str(error)+'；按空格重新锁定。'
            logging.warning('SDK 人体跟随停止：%s', error)
        finally:
            if connected:
                try:
                    if not self._robot.execute_action('stop', speed=0., duration=0.):
                        message = 'SDK 停止回执异常，请检查机器狗状态。'
                except Exception as error:
                    message = f'SDK 停止失败：{error}'
                finally:
                    try:
                        self._robot.shutdown()
                    except Exception as error:
                        message = f'SDK 释放失败：{error}'
            self._notify(message, stopped=True)
