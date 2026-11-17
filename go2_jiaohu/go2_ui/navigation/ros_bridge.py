"""Threaded ROS 2 bridge used by the Qt navigation page.

All rclpy entities live in this QThread.  The GUI only exchanges immutable
payloads through Qt signals and a command queue, so ROS callbacks never touch
widgets directly.
"""

from __future__ import annotations

import math
import json
import os
import queue
import time
from array import array
from typing import Any

from PyQt5.QtCore import QThread, pyqtSignal

ROS_IMPORT_ERROR = ""
try:
    import rclpy
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.srv import ManageLifecycleNodes
    from lifecycle_msgs.srv import GetState
    from std_msgs.msg import Bool, String
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.duration import Duration
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    from std_srvs.srv import Trigger

    ROS_AVAILABLE = True
except Exception as exc:  # UI must still open when ROS is not sourced.
    ROS_AVAILABLE = False
    ROS_IMPORT_ERROR = f"{type(exc).__name__}: {exc}"


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _set_yaw(orientation, yaw: float) -> None:
    orientation.x = 0.0
    orientation.y = 0.0
    orientation.z = math.sin(yaw * 0.5)
    orientation.w = math.cos(yaw * 0.5)


class RosNavigationWorker(QThread):
    map_updated = pyqtSignal(object)
    pose_updated = pyqtSignal(float, float, float)
    path_updated = pyqtSignal(object)
    status_updated = pyqtSignal(str, str)
    feedback_updated = pyqtSignal(float, str)
    goal_finished = pyqtSignal(bool, str)
    map_saved = pyqtSignal(bool, str)
    log_message = pyqtSignal(str)
    terrain_publishers_updated = pyqtSignal(int)
    capabilities_updated = pyqtSignal(object)
    control_finished = pyqtSignal(str, bool, str)
    visual_goal_updated = pyqtSignal(str, str, str)
    relative_action_updated = pyqtSignal(str, str, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._commands: queue.Queue[tuple[str, tuple[Any, ...]]] = queue.Queue()
        self._stopping = False
        self._context = None
        self._executor = None
        self._node = None
        self._navigator = None
        self._tf_buffer = None
        self._tf_listener = None
        self._goal_handle = None
        self._goal_request_pending = False
        self._cancel_pending = False
        self._display_frame = 'odom_lio'
        self._map_seen_at = 0.0
        self._pose_seen_at = 0.0
        self._map_pose_seen_at = 0.0
        self._last_status_at = 0.0
        self._last_pose_emit_at = 0.0
        self._last_tf_lookup_at = 0.0
        self._map_payload = None
        self._last_odom_pose = None
        self._localization_valid = None
        self._fault_message = ''
        self._fault_seen_at = 0.
        self._lifecycle_state = 0
        self._lifecycle_seen_at = 0.
        self._state_future = None
        self._state_deadline = 0.
        self._control_future = None
        self._control_operation = ''
        self._control_deadline = 0.
        self._control_error = ''
        self._operator_stopped = False
        self._external_goal_active = False
        self._save_future = None
        self._save_deadline = 0.
        self._last_capabilities = None
        self._goal_owner = ''
        self._relative_executor = None
        self._behavior_status = {}
        self._capture_pending = None
        self._odom_motion = None
        self._motion_still_since = 0.

    def enqueue(self, command: str, *args: Any) -> None:
        self._commands.put((command, args))

    def stop(self) -> None:
        # Closing a display is not an operator request to stop the robot.
        self.enqueue("shutdown")

    def run(self) -> None:
        if not ROS_AVAILABLE:
            self.status_updated.emit("offline", "ROS 2 Python 环境不可用")
            self.log_message.emit(
                "未加载 rclpy。请用 scripts/run_ui.sh 启动。" +
                (f" ({ROS_IMPORT_ERROR})" if ROS_IMPORT_ERROR else "")
            )
            return

        try:
            from go2_navigation.runtime_guard import navigation_network_error
            network_error = navigation_network_error()
            if network_error:
                self.status_updated.emit('error', network_error)
                self.log_message.emit(network_error)
                return
            self._context = Context()
            rclpy.init(context=self._context)
            self._node = Node("fishbone_navigation_ui", context=self._context)
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
            self._create_entities()
            self.status_updated.emit("waiting", "ROS 2 已连接，等待地图与 Nav2")
            self.log_message.emit("ROS 2 Humble 导航桥接器已启动")

            while not self._stopping and self._context.ok():
                self._drain_commands()
                self._executor.spin_once(timeout_sec=0.05)
                self._check_request_timeouts()
                self._poll_map_pose()
                self._flush_visual_capture()
                if self._relative_executor is not None:
                    self._relative_executor.tick()
                self._emit_periodic_status()
        except Exception as exc:
            self.status_updated.emit("error", str(exc))
            self.log_message.emit(f"ROS 2 导航桥接器异常: {type(exc).__name__}: {exc}")
        finally:
            self._cancel_owned_visual_on_shutdown()
            if self._control_future is not None:
                self._control_future = None
                self.control_finished.emit(self._control_operation, False, 'ROS 连接已退出，启停操作未完成')
            if self._save_future is not None:
                self._save_future = None
                self.map_saved.emit(False, 'ROS 连接已退出，地图保存未完成')
            self.capabilities_updated.emit({'online': False})
            self._cleanup()

    def _create_entities(self) -> None:
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._node.create_subscription(OccupancyGrid, "/safety/navigation_global", self._on_map, map_qos)
        self._node.create_subscription(Odometry, "/odom", self._on_odom, pose_qos)
        self._node.create_subscription(Path, "/plan", self._on_path, pose_qos)

        # Display poses and paths in the received LIO map frame. RViz handles
        # its own fixed-frame transforms independently with the existing config.
        self._tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
        self._tf_listener = TransformListener(
            self._tf_buffer, self._node, spin_thread=False
        )

        self._cmd_vel_pub = self._node.create_publisher(Twist, "/cmd_vel", 10)
        self._navigator = ActionClient(self._node, NavigateToPose, "navigate_to_pose")
        self._cancel_client = self._node.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self._environment_save_client = self._node.create_client(Trigger, '/mapping/save')
        self._lifecycle_client = self._node.create_client(
            ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        self._state_client = self._node.create_client(GetState, '/bt_navigator/get_state')
        self._node.create_subscription(Bool, '/lio/localization_valid',
            lambda m: setattr(self, '_localization_valid', m.data), map_qos)
        self._node.create_subscription(String, '/safety/navigation_fault', self._on_fault, map_qos)
        self._node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
            lambda m: setattr(self, '_external_goal_active',
                any(s.status in (1, 2, 3) for s in m.status_list)), map_qos)
        from go2_ui.navigation.relative_executor import RelativeNavigationExecutor
        self._relative_executor = RelativeNavigationExecutor(self)
        for action in ('spin', 'drive_on_heading', 'backup'):
            self._node.create_subscription(GoalStatusArray, f'/{action}/_action/status',
                lambda msg, name=action: self._on_behavior_status(name, msg), map_qos)

    def _drain_commands(self) -> None:
        for _ in range(30):
            try:
                command, args = self._commands.get_nowait()
            except queue.Empty:
                return
            if command == "shutdown":
                self._stopping = True
                return
            try:
                {
                    "goal": self._send_goal,
                    "cancel": self._cancel_goal,
                    "save_environment": self._save_environment,
                    "emergency_stop": self._emergency_stop,
                    "pause": lambda: self._control_navigation('pause'),
                    "resume": lambda: self._control_navigation('resume'),
                    "visual_preflight": self._visual_preflight,
                    "visual_goal": self._queue_visual_capture,
                    "cancel_visual": self._cancel_visual_goal,
                    "relative_sequence": lambda owner, actions: self._relative_executor.start(owner, actions),
                    "cancel_relative": lambda owner: self._relative_executor.cancel(owner),
                }[command](*args)
            except KeyError:
                self.log_message.emit(f"忽略未知 ROS 命令: {command}")
            except Exception as exc:
                self.log_message.emit(f"执行 {command} 失败: {type(exc).__name__}: {exc}")
                if command in ('pause', 'resume'):
                    self.control_finished.emit(command, False, f'启停请求失败：{exc}')
                elif command == 'save_environment':
                    self._save_future = None
                    self.map_saved.emit(False, f'保存请求失败：{exc}')
                elif command == 'goal':
                    self._goal_request_pending = False
                    self.goal_finished.emit(False, f'目标发送失败：{exc}')

    def _camera_calibration_ready(self):
        """Validate extrinsics before starting Nav2, which supplies the body TF."""
        from voice import config
        from voice.camera_calibration import load_camera_calibration
        from voice.visual_navigation import calibrated_mount_point, transform_point
        try:
            transform = self._tf_buffer.lookup_transform('base_link', config.VISUAL_CAMERA_FRAME, Time())
        except Exception:
            if config.VISUAL_CAMERA_MOUNT:
                calibrated_mount_point((0., 0., 1.), config.VISUAL_CAMERA_MOUNT)
            else:
                load_camera_calibration(config.VISUAL_CAMERA_CALIBRATION, config.VISUAL_CAMERA_FRAME)
        else:
            transform_point((0., 0., 1.), transform.transform)

    def _camera_point_in_base(self, point, when):
        from voice import config
        from voice.camera_calibration import load_camera_calibration
        from voice.visual_navigation import calibrated_mount_point, transform_point
        try:
            transform = self._tf_buffer.lookup_transform('base_link', config.VISUAL_CAMERA_FRAME, when)
        except Exception:
            if config.VISUAL_CAMERA_MOUNT:
                return calibrated_mount_point(point, config.VISUAL_CAMERA_MOUNT)
            calibration = load_camera_calibration(config.VISUAL_CAMERA_CALIBRATION, config.VISUAL_CAMERA_FRAME)
            try:
                parent_in_base = self._tf_buffer.lookup_transform('base_link', calibration.parent_frame, when)
            except Exception as error:
                raise ValueError(f'已加载 D435i 联合标定，但采图时刻的 {calibration.parent_frame} '
                                 '到 base_link 变换未就绪，请等待导航定位就绪后重试。未执行移动。') from error
            return transform_point(calibration.point_in_parent(point), parent_in_base.transform)
        return transform_point(point, transform.transform)

    def _visual_preflight(self, request_id):
        try:
            if (self._operator_stopped or self._control_future is not None or self._lifecycle_state != 3
                    or self._localization_valid is not True or self._fault_active()
                    or time.monotonic()-self._map_pose_seen_at > 2.):
                raise ValueError('导航或有效定位未就绪，未执行移动。')
            if (self._goal_request_pending or self._goal_handle is not None or self._external_goal_active
                    or self._relative_busy() or self._capture_pending is not None or self._behavior_busy()):
                raise ValueError('已有导航任务，请先停止或等待完成后再发送视觉移动请求。')
            self._camera_point_in_base((0., 0., 1.), Time())
            self.visual_goal_updated.emit(request_id, 'prepared', '导航已就绪，正在定位目标并测距…')
        except Exception as error:
            self.visual_goal_updated.emit(request_id, 'rejected', str(error))

    def _relative_busy(self):
        return bool(self._relative_executor is not None and self._relative_executor.active)

    def _on_behavior_status(self, name, msg):
        self._behavior_status[name] = [(bytes(item.goal_info.goal_id.uuid), item.status) for item in msg.status_list]

    def _behavior_busy(self):
        owned = self._relative_executor.own_ids if self._relative_executor is not None else set()
        return any(status in (1, 2, 3) and identity not in owned
                   for items in self._behavior_status.values() for identity, status in items)

    def _relative_preflight(self, owned=False):
        if (self._operator_stopped or self._control_future is not None or self._lifecycle_state != 3
                or time.monotonic()-self._lifecycle_seen_at > 4. or self._localization_valid is not True
                or self._fault_active() or time.monotonic()-self._map_pose_seen_at > 2.):
            raise ValueError('导航、定位或位姿未就绪，未继续相对动作。')
        if (self._goal_request_pending or self._goal_handle is not None or self._external_goal_active
                or self._capture_pending is not None or self._behavior_busy() or (not owned and self._relative_busy())):
            raise ValueError('已有导航任务或动作，请先停止或等待完成。')

    def _relative_stationary(self, since):
        now = time.monotonic()
        return bool(self._odom_motion is not None and self._odom_motion[0] >= since
                    and now-self._odom_motion[0] < .3 and self._motion_still_since
                    and now-self._motion_still_since >= .15)

    def _validate_relative_path(self, step):
        pose = self._tf_buffer.lookup_transform(self._display_frame, 'base_link', Time()).transform
        values = (pose.translation.x,pose.translation.y,pose.translation.z,
                  pose.rotation.x,pose.rotation.y,pose.rotation.z,pose.rotation.w)
        if not all(math.isfinite(v) for v in values) or not .99 <= math.sqrt(sum(v*v for v in values[3:])) <= 1.01:
            raise ValueError('机身坐标变换无效')
        if step.kind == 'spin':
            return  # The behavior checks its rotating footprint in the live local costmap.
        payload = self._map_payload
        if payload is None:
            raise ValueError('地图尚未接收')
        yaw, resolution = _yaw_from_quaternion(pose.rotation), float(payload['resolution'])
        if not math.isfinite(resolution) or resolution <= 0.:
            raise ValueError('地图分辨率无效')
        count = max(1, math.ceil(abs(step.quantity)/min(.10, resolution)))
        for index in range(1, count+1):
            distance = step.quantity*index/count
            x, y = pose.translation.x+math.cos(yaw)*distance, pose.translation.y+math.sin(yaw)*distance
            error = self._validate_goal(x, y, True)
            if error:
                raise ValueError(error)
            dx, dy = x-payload['origin_x'], y-payload['origin_y']
            c, s = math.cos(payload['origin_yaw']), math.sin(payload['origin_yaw'])
            col, row = math.floor((c*dx+s*dy)/resolution), math.floor((-s*dx+c*dy)/resolution)
            cell = payload['data'][row*payload['width']+col]
            if cell == 255 or cell >= 65:
                raise ValueError('相对直行路径包含未知或障碍区域，未执行该步。')

    def _queue_visual_capture(self, target):
        if (self._capture_pending is not None or self._relative_busy() or self._behavior_busy()
                or self._goal_request_pending or self._goal_handle is not None or self._external_goal_active):
            self.visual_goal_updated.emit(target.request_id, 'rejected', '已有导航任务，未发送视觉目标。')
            return
        self._capture_pending = (target, time.monotonic()+1.)

    def _flush_visual_capture(self):
        if self._capture_pending is None:
            return
        target, deadline = self._capture_pending
        if time.monotonic() > deadline:
            self._capture_pending = None
            self.visual_goal_updated.emit(target.request_id, 'rejected', '实时画面对应的位姿尚未到达，未发送移动目标，请重试。')
            return
        when = Time(seconds=target.snapshot.captured_at)
        try:
            available = self._tf_buffer.can_transform(self._display_frame, 'base_link', when)
        except Exception:
            available = False
        if available:
            self._capture_pending = None
            self._send_visual_goal(target)

    def _send_visual_goal(self, target):
        from voice.visual_navigation import approach_goal, transform_point
        request_id = target.request_id
        try:
            now = self._node.get_clock().now().nanoseconds / 1.e9
            if not 0. <= now-target.snapshot.captured_at <= 30.:
                raise ValueError('视觉画面时间戳已过期或与导航时钟不一致，请重新发送。')
            when = Time(seconds=target.snapshot.captured_at)
            base_at_capture = self._tf_buffer.lookup_transform(self._display_frame, 'base_link', when)
            base_now = self._tf_buffer.lookup_transform(self._display_frame, 'base_link', Time())
            a, b = base_at_capture.transform, base_now.transform
            moved = math.hypot(a.translation.x-b.translation.x, a.translation.y-b.translation.y)
            turned = math.atan2(math.sin(_yaw_from_quaternion(a.rotation)-_yaw_from_quaternion(b.rotation)),
                               math.cos(_yaw_from_quaternion(a.rotation)-_yaw_from_quaternion(b.rotation)))
            if moved > .10 or abs(turned) > .10:
                raise ValueError('识别期间机器狗已移动，请停止后重新识别目标。')
            body_point = self._camera_point_in_base(target.optical_point, when)
            if body_point[0] < .15:
                raise ValueError('目标不在机身前方，未执行移动。')
            mapped_point = transform_point(body_point, a)
            x, y, yaw = approach_goal((a.translation.x, a.translation.y), mapped_point, target.intent.stand_off_m)
            error = self._validate_goal(x, y, True)
            if error:
                raise ValueError(error)
            payload = self._map_payload
            dx, dy = x-payload['origin_x'], y-payload['origin_y']
            c, s = math.cos(payload['origin_yaw']), math.sin(payload['origin_yaw'])
            col, row = math.floor((c*dx+s*dy)/payload['resolution']), math.floor((-s*dx+c*dy)/payload['resolution'])
            occupancy = payload['data'][row*payload['width']+col]
            if occupancy == 255 or occupancy >= 65:
                raise ValueError('停靠点位于未知或不安全地图区域，请换一个可达的目标。')
            self._send_goal(x, y, yaw, True, owner=request_id)
        except Exception as error:
            self.visual_goal_updated.emit(request_id, 'rejected', f'未执行移动：{error}')

    def _cancel_visual_goal(self, request_id):
        if self._capture_pending is not None and self._capture_pending[0].request_id == request_id:
            self._capture_pending = None
        # Mode changes cancel only the goal owned by this visual request.
        if self._goal_owner != request_id:
            return
        self._cancel_pending = self._goal_request_pending
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def _cancel_owned_visual_on_shutdown(self):
        self._capture_pending = None
        if self._relative_busy() and self._executor is not None:
            self._relative_executor.cancel(self._relative_executor.owner)
            deadline = time.monotonic()+1.2
            while self._relative_busy() and time.monotonic() < deadline:
                self._executor.spin_once(timeout_sec=.05)
        if not self._goal_owner or self._executor is None:
            return
        owner = self._goal_owner
        try:
            self._cancel_visual_goal(owner)
            # Deliver a late goal acceptance and its cancellation before destroying
            # the action client. External/RViz goals have no visual owner.
            deadline = time.monotonic()+1.2
            while self._goal_owner == owner and time.monotonic() < deadline:
                self._executor.spin_once(timeout_sec=.05)
        except Exception as error:
            self.log_message.emit(f'取消视觉导航请求失败：{error}')

    def _on_map(self, msg) -> None:
        info = msg.info
        payload = {
            'frame': msg.header.frame_id,
            "width": int(info.width),
            "height": int(info.height),
            "resolution": float(info.resolution),
            "origin_x": float(info.origin.position.x),
            "origin_y": float(info.origin.position.y),
            "origin_yaw": _yaw_from_quaternion(info.origin.orientation),
            # rclpy usually exposes int8[] as array('b').  Converting its raw
            # buffer avoids a slow Python loop over every map cell; -1 remains
            # 0xFF for the NumPy renderer.
            "data": self._occupancy_bytes(msg.data),
        }
        self._map_seen_at = time.monotonic()
        self._map_payload = payload
        self._display_frame = msg.header.frame_id
        self.map_updated.emit(payload)

    def _on_odom(self, msg) -> None:
        motion = msg.twist.twist
        values = (motion.linear.x, motion.linear.y, motion.angular.z)
        now = time.monotonic()
        still = all(math.isfinite(v) for v in values) and math.hypot(*values[:2]) < .06 and abs(values[2]) < .10
        if still:
            if not self._motion_still_since or (self._odom_motion is not None and now-self._odom_motion[0] > .3):
                self._motion_still_since = now
        else:
            self._motion_still_since = 0.
        self._odom_motion = (now, *values)
        pose = msg.pose.pose
        self._last_odom_pose = (
            float(pose.position.x),
            float(pose.position.y),
            _yaw_from_quaternion(pose.orientation),
        )
        # Before /map is available the raw odometry pose is still useful for a
        # connectivity indication. Never mix it into an existing map display.
        if self._map_seen_at <= 0.0 and msg.header.frame_id == self._display_frame:
            self._emit_pose(pose, map_frame=False)

    def _emit_pose(self, pose, map_frame: bool) -> None:
        self._emit_pose_values(
            float(pose.position.x),
            float(pose.position.y),
            _yaw_from_quaternion(pose.orientation),
            map_frame,
        )

    def _emit_pose_values(self, x: float, y: float, yaw: float, map_frame: bool) -> None:
        now = time.monotonic()
        self._pose_seen_at = now
        if map_frame:
            self._map_pose_seen_at = now
        # Unitree odometry can exceed 100 Hz.  The map only needs a smooth
        # 20 Hz visual pose; throttling prevents the Qt event queue growing.
        if now - self._last_pose_emit_at < 0.05:
            return
        self._last_pose_emit_at = now
        self.pose_updated.emit(x, y, yaw)

    def _poll_map_pose(self) -> None:
        if self._map_seen_at <= 0.0 or self._tf_buffer is None:
            return
        now = time.monotonic()
        if now - self._last_tf_lookup_at < 0.04:
            return
        self._last_tf_lookup_at = now
        try:
            transform = self._tf_buffer.lookup_transform(self._display_frame, "base_link", Time())
        except Exception:
            return
        # A successful latest-TF lookup can still return the last pose of a
        # stopped estimator. Do not refresh UI health using that cached pose.
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1.e9
        age = self._node.get_clock().now().nanoseconds / 1.e9 - stamp
        if stamp <= 0. or age > 1.8 or age < -.1:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self._emit_pose_values(
            float(translation.x),
            float(translation.y),
            _yaw_from_quaternion(rotation),
            map_frame=True,
        )

    def _on_path(self, msg) -> None:
        # Rendering thousands of nearly coincident poses adds no visible
        # precision. Keep at most about 600 evenly spaced display points.
        stride = max(1, len(msg.poses) // 600)
        points = [
            (float(p.pose.position.x), float(p.pose.position.y))
            for p in msg.poses[::stride]
        ]
        if msg.header.frame_id != self._display_frame:
            try:
                tf = self._tf_buffer.lookup_transform(self._display_frame, msg.header.frame_id, Time())
                p, q = tf.transform.translation, tf.transform.rotation
                yaw = _yaw_from_quaternion(q);c, s = math.cos(yaw), math.sin(yaw)
                points = [(p.x+c*x-s*y, p.y+s*x+c*y) for x, y in points]
            except Exception:
                self.path_updated.emit([])
                return
        self.path_updated.emit(points)

    @staticmethod
    def _occupancy_bytes(values) -> bytes:
        if hasattr(values, "tobytes"):
            return values.tobytes()
        return array("b", values).tobytes()

    def _send_goal(
        self, x: float, y: float, yaw: float, require_map_bounds: bool = False, owner: str = ''
    ) -> None:
        def reject(message):
            self.goal_finished.emit(False, message)
            if owner:
                self.visual_goal_updated.emit(owner, 'rejected', message)
        if self._operator_stopped or self._control_future is not None or self._lifecycle_state != 3:
            reject('导航尚未启用，请先启动 / 恢复导航')
            return
        if self._localization_valid is not True or self._fault_active():
            reject('定位尚未有效或已故障，未发送导航目标')
            return
        if not self._navigator.server_is_ready():
            reject("Nav2 navigate_to_pose 尚未就绪")
            return
        if (self._goal_request_pending or self._goal_handle is not None or self._external_goal_active
                or self._relative_busy() or self._capture_pending is not None or self._behavior_busy()):
            reject("已有导航目标正在执行，请先取消或等待完成")
            return
        if time.monotonic() - self._map_pose_seen_at > 2.0:
            reject("机器人位姿已超时，未发送导航目标")
            return
        goal_error = self._validate_goal(x, y, require_map_bounds)
        if goal_error:
            reject(goal_error)
            self.log_message.emit(goal_error)
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._display_frame
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        _set_yaw(goal.pose.pose.orientation, float(yaw))
        self._goal_request_pending = True
        self._cancel_pending = False
        self._goal_owner = owner
        try:
            future = self._navigator.send_goal_async(goal, feedback_callback=lambda msg: self._on_feedback(msg, owner))
            future.add_done_callback(lambda result: self._on_goal_response(result, owner))
        except Exception as error:
            self._goal_request_pending = False
            self._goal_owner = ''
            reject(f'发送目标失败：{error}')
            return
        self.log_message.emit(
            f"发送导航目标: x={x:.2f}, y={y:.2f}（终点朝向不限制）"
        )

    def _validate_goal(self, x: float, y: float, require_map_bounds: bool) -> str:
        payload = self._map_payload
        if payload is None:
            return "地图尚未接收，未发送导航目标"
        resolution = float(payload["resolution"])
        if resolution <= 0.0:
            return "地图分辨率无效，未发送导航目标"
        dx = float(x) - float(payload["origin_x"])
        dy = float(y) - float(payload["origin_y"])
        origin_yaw = float(payload["origin_yaw"])
        cell_x = math.floor(
            (math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy) / resolution
        )
        cell_y = math.floor(
            (-math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy) / resolution
        )
        width = int(payload["width"])
        height = int(payload["height"])
        if cell_x < 0 or cell_y < 0 or cell_x >= width or cell_y >= height:
            if require_map_bounds:
                return "目标位于已载入地图边界之外，未发送（请在地图内部选点）"
            # Online mapping uses a rolling global costmap and may deliberately
            # navigate just beyond the currently published SLAM rectangle.
            return ""
        occupancy = payload["data"][cell_y * width + cell_x]
        if occupancy != 255 and occupancy >= 100:
            return "目标落在占用栅格/墙体上，未发送（请重新选点）"
        return ""

    def _on_goal_response(self, future, owner='') -> None:
        self._goal_request_pending = False
        try:
            handle = future.result()
            if not handle.accepted:
                self.goal_finished.emit(False, "目标被 Nav2 拒绝")
                if owner:
                    self.visual_goal_updated.emit(owner, 'rejected', '目标被 Nav2 拒绝')
                self._goal_owner = ''
                return
            self._goal_handle = handle
            if self._cancel_pending:
                handle.cancel_goal_async()
            elif owner:
                self.visual_goal_updated.emit(owner, 'accepted', 'Nav2 已接受目标，正在避障前往停靠点。')
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda result, goal_handle=handle: self._on_goal_result(result, goal_handle, owner)
            )
            self.log_message.emit("Nav2 已接受目标")
        except Exception as exc:
            self.goal_finished.emit(False, f"发送目标失败: {exc}")
            if owner:
                if self._goal_handle is not None:
                    try:
                        self._goal_handle.cancel_goal_async()
                    except Exception as cancel_error:
                        self.log_message.emit(f'取消视觉目标请求失败：{cancel_error}')
                self.visual_goal_updated.emit(owner, 'rejected', f'发送目标失败：{exc}')
            if self._goal_handle is None:
                self._goal_owner = ''

    def _on_feedback(self, feedback_msg, owner='') -> None:
        feedback = feedback_msg.feedback
        distance = float(feedback.distance_remaining)
        eta = feedback.estimated_time_remaining
        eta_seconds = float(eta.sec) + float(eta.nanosec) / 1e9
        self.feedback_updated.emit(distance, f"预计 {eta_seconds:.0f} 秒")
        if owner:
            self.visual_goal_updated.emit(owner, 'feedback', f'正在前往停靠点，剩余路径约 {distance:.2f} 米。')

    def _on_goal_result(self, future, goal_handle, owner='') -> None:
        try:
            status = future.result().status
            labels = {
                GoalStatus.STATUS_SUCCEEDED: (True, "已到达目标"),
                GoalStatus.STATUS_CANCELED: (False, "导航已取消"),
                GoalStatus.STATUS_ABORTED: (False, "导航失败/中止"),
            }
            success, text = labels.get(status, (False, f"导航结束，状态码 {status}"))
            self.goal_finished.emit(success, text)
            if owner:
                self.visual_goal_updated.emit(owner, 'succeeded' if success else 'finished', text)
        except Exception as exc:
            self.goal_finished.emit(False, f"读取导航结果失败: {exc}")
            if owner:
                self.visual_goal_updated.emit(owner, 'finished', f'读取导航结果失败：{exc}')
        finally:
            if self._goal_handle is goal_handle:
                self._goal_handle = None
                self._goal_owner = ''

    def _cancel_goal(self) -> None:
        self._capture_pending = None
        if self._relative_busy():
            self._relative_executor.cancel(self._relative_executor.owner)
        self._cancel_pending = self._goal_request_pending
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        # An RViz GoalTool goal is owned by RViz, not this action client.
        # The explicit cancel button must also cancel that active goal.
        if self._cancel_client.service_is_ready():
            future = self._cancel_client.call_async(CancelGoal.Request())
            future.add_done_callback(self._on_cancel_response)
        self._publish_zero_velocity(5)
        self.log_message.emit("已请求取消当前导航")

    def _emergency_stop(self) -> None:
        self._cancel_goal()
        self._publish_zero_velocity(12)
        self.log_message.emit("已发送急停：取消目标并连续发布零速度")

    def _on_cancel_response(self, future):
        try:
            response = future.result()
            self.log_message.emit(f'已请求取消 {len(response.goals_canceling)} 个导航目标（含 RViz 目标）')
        except Exception as error:
            self.log_message.emit(f'取消导航请求失败：{error}')

    def _save_environment(self):
        if self._save_future is not None:
            return
        if not self._environment_save_client.service_is_ready():
            self.map_saved.emit(False, '当前环境地图保存服务尚未就绪')
            return
        future = self._environment_save_client.call_async(Trigger.Request())
        self._save_future = future
        self._save_deadline = time.monotonic() + 10.
        def finished(result):
            if self._save_future is not result:
                return
            self._save_future = None
            try:
                response = result.result()
                self.map_saved.emit(response.success, response.message)
            except Exception as error:
                self.map_saved.emit(False, str(error))
        future.add_done_callback(finished)

    def _on_fault(self, message):
        try:
            payload = json.loads(message.data)
            self._fault_message = payload.get('message', 'FAST-LIO 定位已停止，需重启导航进程')
        except (ValueError, TypeError):
            self._fault_message = message.data
        self._fault_seen_at = time.monotonic()

    def _fault_active(self):
        return bool(self._fault_message and
            (self._localization_valid is not True or time.monotonic()-self._fault_seen_at < 3.))

    def _control_navigation(self, operation):
        if self._control_future is not None:
            self.log_message.emit('导航状态切换仍在进行，请等待结果')
            return
        if operation == 'pause':
            self._operator_stopped = True
            self._cancel_goal()
        elif (self._localization_valid is not True or self._fault_active() or
              time.monotonic()-self._map_pose_seen_at > 3.):
            self._control_error = '定位无效或已超时，不能恢复；请检查定位日志'
            self.control_finished.emit(operation, False, self._control_error)
            return
        if operation == 'resume' and self._lifecycle_state == 3 and time.monotonic()-self._lifecycle_seen_at < 2.:
            self._operator_stopped = False
            self._control_error = ''
            self.control_finished.emit(operation, True, '导航已启用，可以重新选择目标')
            return
        if not self._lifecycle_client.service_is_ready():
            self._control_error = '导航启停服务尚未就绪，请稍后重试'
            self.control_finished.emit(operation, False, self._control_error)
            return
        request = ManageLifecycleNodes.Request()
        request.command = (request.PAUSE if operation == 'pause' else request.RESUME)
        self._control_operation = operation
        self._control_error = ''
        self._control_deadline = time.monotonic() + 15.
        self._control_future = self._lifecycle_client.call_async(request)
        self._control_future.add_done_callback(self._on_control_response)

    def _on_control_response(self, future):
        if self._control_future is not future:
            return
        operation = self._control_operation
        self._control_future = None
        if self._state_future is not None:
            stale, self._state_future = self._state_future, None
            stale.cancel()
        try:
            success = bool(future.result().success)
            message = ('导航已停止，定位和地图继续运行' if operation == 'pause' else
                       '导航已恢复，可以重新选择目标') if success else '导航状态切换被拒绝，请检查运行日志后重试'
        except Exception as error:
            success, message = False, f'导航状态切换失败：{error}'
        if success:
            self._operator_stopped = operation == 'pause'
            self._lifecycle_state = 2 if operation == 'pause' else 3
            self._lifecycle_seen_at = time.monotonic()
        self._control_error = '' if success else message
        self.control_finished.emit(operation, success, message)
        self._last_status_at = 0.

    def _check_request_timeouts(self):
        now = time.monotonic()
        if self._control_future is not None and now > self._control_deadline:
            future, self._control_future = self._control_future, None
            future.cancel()
            self._control_error = '导航状态切换超时，请检查状态后重试'
            self.control_finished.emit(self._control_operation, False, self._control_error)
        if self._save_future is not None and now > self._save_deadline:
            future, self._save_future = self._save_future, None
            future.cancel()
            self.map_saved.emit(False, '地图保存超时，请检查保存服务后重试')

    def _poll_lifecycle(self, now):
        if self._state_future is not None:
            if now < self._state_deadline:
                return
            future, self._state_future = self._state_future, None
            future.cancel()
        if not self._state_client.service_is_ready():
            return
        self._state_deadline = now + 2.
        self._state_future = self._state_client.call_async(GetState.Request())
        def finished(future):
            if self._state_future is not future:
                return
            self._state_future = None
            try:
                self._lifecycle_state = future.result().current_state.id
                self._lifecycle_seen_at = time.monotonic()
            except Exception as error:
                self.log_message.emit(f'读取导航启停状态失败：{error}')
        self._state_future.add_done_callback(finished)

    def _publish_zero_velocity(self, count: int) -> None:
        zero = Twist()
        for _ in range(count):
            self._cmd_vel_pub.publish(zero)

    def _emit_periodic_status(self) -> None:
        now = time.monotonic()
        if now - self._last_status_at < 1.0:
            return
        self._last_status_at = now
        self._poll_lifecycle(now)
        nav_ready = bool(self._navigator and self._navigator.server_is_ready())
        if nav_ready:
            self.terrain_publishers_updated.emit(
                self._node.count_publishers('/visualization/environment_terrain'))
        # Static-map publishers normally publish only once with transient-local
        # durability; once received, the map remains valid until replaced.
        map_ok = self._map_seen_at > 0.0
        pose_ok = (
            now - self._map_pose_seen_at < 3.0
            if map_ok else now - self._pose_seen_at < 3.0
        )
        active = self._lifecycle_state == 3 and now-self._lifecycle_seen_at < 4.
        paused = self._lifecycle_state == 2 and now-self._lifecycle_seen_at < 4.
        if self._fault_active():
            state, text = 'fault', self._fault_message
        elif self._control_future is not None:
            state, text = 'waiting', '正在停止导航…' if self._control_operation == 'pause' else '正在恢复导航…'
        elif self._control_error:
            state, text = 'error', self._control_error
        elif paused or self._operator_stopped:
            state, text = 'paused', '导航已停止；定位和地图仍可查看'
        elif nav_ready and active and map_ok and pose_ok and self._localization_valid is True:
            state, text = 'ready', 'Nav2、地图、位姿均正常'
        else:
            missing = []
            if not nav_ready:
                missing.append("Nav2")
            if not map_ok:
                missing.append("地图")
            if not pose_ok:
                missing.append("位姿")
            if not active:
                missing.append('导航启用状态')
            if self._localization_valid is not True:
                missing.append('有效定位')
            state, text = 'waiting', '等待: ' + ' / '.join(missing)
        self.status_updated.emit(state, text)
        camera_error=''
        try:
            self._camera_calibration_ready()
        except Exception as error:
            camera_error=str(error)
        capabilities = {'online': True, 'can_control': self._lifecycle_client.service_is_ready(),
            'camera_calibrated': not camera_error, 'camera_error': camera_error,
            'paused': paused or self._operator_stopped,
            'can_save': map_ok and self._environment_save_client.service_is_ready(),
            'goal_active': (self._external_goal_active or self._goal_request_pending or self._goal_handle is not None
                            or self._relative_busy() or self._capture_pending is not None or self._behavior_busy()),
            'relative_ready': bool(self._relative_executor is not None and
                all(client.server_is_ready() for client in self._relative_executor.clients.values())),
            'save_busy': self._save_future is not None}
        if capabilities != self._last_capabilities:
            self._last_capabilities = capabilities
            self.capabilities_updated.emit(capabilities)

    def _cleanup(self) -> None:
        try:
            if self._relative_executor is not None:
                self._relative_executor.close()
            if self._executor is not None and self._node is not None:
                self._executor.remove_node(self._node)
            if self._navigator is not None:
                self._navigator.destroy()
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass
        try:
            if self._context is not None and self._context.ok():
                rclpy.shutdown(context=self._context)
        except Exception:
            pass
