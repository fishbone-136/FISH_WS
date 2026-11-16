"""Threaded ROS 2 bridge used by the Qt navigation page.

All rclpy entities live in this QThread.  The GUI only exchanges immutable
payloads through Qt signals and a command queue, so ROS callbacks never touch
widgets directly.
"""

from __future__ import annotations

import math
import os
import queue
import time
from array import array
from typing import Any

from PyQt5.QtCore import QThread, pyqtSignal

ROS_IMPORT_ERROR = ""
try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.srv import ClearEntireCostmap, SaveMap
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.duration import Duration
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

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
        self._map_seen_at = 0.0
        self._pose_seen_at = 0.0
        self._map_pose_seen_at = 0.0
        self._amcl_seen_at = 0.0
        self._last_status_at = 0.0
        self._last_pose_emit_at = 0.0
        self._last_tf_lookup_at = 0.0
        self._map_payload = None
        self._last_odom_pose = None

    def enqueue(self, command: str, *args: Any) -> None:
        self._commands.put((command, args))

    def stop(self) -> None:
        self.enqueue("emergency_stop")
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
                self._poll_map_pose()
                self._emit_periodic_status()
        except Exception as exc:
            self.status_updated.emit("error", str(exc))
            self.log_message.emit(f"ROS 2 导航桥接器异常: {type(exc).__name__}: {exc}")
        finally:
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
        initial_pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._node.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self._node.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, pose_qos
        )
        self._node.create_subscription(Odometry, "/odom", self._on_odom, pose_qos)
        self._node.create_subscription(Path, "/plan", self._on_path, pose_qos)

        # The odometry pose is expressed in odom, while goals and the rendered
        # map are expressed in map.  Always render map->base_link from TF when a
        # map exists; plotting raw odometry on the map makes normal SLAM
        # map->odom corrections look like robot/map drift.
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(
            self._tf_buffer, self._node, spin_thread=False
        )

        self._initial_pose_pub = self._node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", initial_pose_qos
        )
        self._cmd_vel_pub = self._node.create_publisher(Twist, "/cmd_vel", 10)
        self._navigator = ActionClient(self._node, NavigateToPose, "navigate_to_pose")
        self._save_map_client = self._node.create_client(SaveMap, "/map_saver/save_map")
        self._clear_global_client = self._node.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap"
        )
        self._clear_local_client = self._node.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap"
        )

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
                    "initial_pose": self._publish_initial_pose,
                    "save_map": self._save_map,
                    "clear_costmaps": self._clear_costmaps,
                    "emergency_stop": self._emergency_stop,
                }[command](*args)
            except KeyError:
                self.log_message.emit(f"忽略未知 ROS 命令: {command}")
            except Exception as exc:
                self.log_message.emit(f"执行 {command} 失败: {type(exc).__name__}: {exc}")

    def _on_map(self, msg) -> None:
        info = msg.info
        payload = {
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
        self.map_updated.emit(payload)

    def _on_amcl_pose(self, msg) -> None:
        pose = msg.pose.pose
        self._amcl_seen_at = time.monotonic()
        self._emit_pose(pose, map_frame=True)

    def _on_odom(self, msg) -> None:
        pose = msg.pose.pose
        self._last_odom_pose = (
            float(pose.position.x),
            float(pose.position.y),
            _yaw_from_quaternion(pose.orientation),
        )
        # Before /map is available the raw odometry pose is still useful for a
        # connectivity indication. Never mix it into an existing map display.
        if self._map_seen_at <= 0.0:
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
            transform = self._tf_buffer.lookup_transform("map", "base_link", Time())
        except Exception:
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
        self.path_updated.emit(points)

    @staticmethod
    def _occupancy_bytes(values) -> bytes:
        if hasattr(values, "tobytes"):
            return values.tobytes()
        return array("b", values).tobytes()

    def _send_goal(
        self, x: float, y: float, yaw: float, require_map_bounds: bool = False
    ) -> None:
        if not self._navigator.server_is_ready():
            self.goal_finished.emit(False, "Nav2 navigate_to_pose 尚未就绪")
            return
        if self._goal_request_pending or self._goal_handle is not None:
            self.goal_finished.emit(False, "已有导航目标正在执行，请先取消或等待完成")
            return
        if time.monotonic() - self._pose_seen_at > 2.0:
            self.goal_finished.emit(False, "机器人位姿已超时，未发送导航目标")
            return
        goal_error = self._validate_goal(x, y, require_map_bounds)
        if goal_error:
            self.goal_finished.emit(False, goal_error)
            self.log_message.emit(goal_error)
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        _set_yaw(goal.pose.pose.orientation, float(yaw))
        self._goal_request_pending = True
        future = self._navigator.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)
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
        if occupancy != 255 and occupancy >= 65:
            return "目标落在占用栅格/墙体上，未发送（请重新选点）"
        return ""

    def _on_goal_response(self, future) -> None:
        self._goal_request_pending = False
        try:
            handle = future.result()
            if not handle.accepted:
                self.goal_finished.emit(False, "目标被 Nav2 拒绝")
                return
            self._goal_handle = handle
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda result, goal_handle=handle: self._on_goal_result(result, goal_handle)
            )
            self.log_message.emit("Nav2 已接受目标")
        except Exception as exc:
            self.goal_finished.emit(False, f"发送目标失败: {exc}")

    def _on_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        distance = float(feedback.distance_remaining)
        eta = feedback.estimated_time_remaining
        eta_seconds = float(eta.sec) + float(eta.nanosec) / 1e9
        self.feedback_updated.emit(distance, f"预计 {eta_seconds:.0f} 秒")

    def _on_goal_result(self, future, goal_handle) -> None:
        try:
            status = future.result().status
            labels = {
                GoalStatus.STATUS_SUCCEEDED: (True, "已到达目标"),
                GoalStatus.STATUS_CANCELED: (False, "导航已取消"),
                GoalStatus.STATUS_ABORTED: (False, "导航失败/中止"),
            }
            success, text = labels.get(status, (False, f"导航结束，状态码 {status}"))
            self.goal_finished.emit(success, text)
        except Exception as exc:
            self.goal_finished.emit(False, f"读取导航结果失败: {exc}")
        finally:
            if self._goal_handle is goal_handle:
                self._goal_handle = None

    def _cancel_goal(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._publish_zero_velocity(5)
        self.log_message.emit("已请求取消当前导航")

    def _emergency_stop(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._publish_zero_velocity(12)
        self.log_message.emit("已发送急停：取消目标并连续发布零速度")

    def _publish_zero_velocity(self, count: int) -> None:
        zero = Twist()
        for _ in range(count):
            self._cmd_vel_pub.publish(zero)

    def _publish_initial_pose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        _set_yaw(msg.pose.pose.orientation, float(yaw))
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self._initial_pose_pub.publish(msg)
        self.log_message.emit(f"已设置初始位姿: x={x:.2f}, y={y:.2f}")

    def _save_map(self, map_url: str) -> None:
        target = os.path.abspath(os.path.expanduser(map_url))
        os.makedirs(os.path.dirname(target), exist_ok=True)
        if not self._save_map_client.wait_for_service(timeout_sec=0.2):
            self.map_saved.emit(False, "map_saver 服务不可用，请先启动建图")
            return
        request = SaveMap.Request()
        request.map_topic = "/map"
        request.map_url = target
        request.image_format = "pgm"
        request.map_mode = "trinary"
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65
        future = self._save_map_client.call_async(request)
        future.add_done_callback(lambda f: self._on_map_saved(f, target))

    def _on_map_saved(self, future, target: str) -> None:
        try:
            result = bool(future.result().result)
            self.map_saved.emit(result, f"地图已保存: {target}" if result else "地图保存服务返回失败")
        except Exception as exc:
            self.map_saved.emit(False, f"地图保存失败: {exc}")

    def _clear_costmaps(self) -> None:
        requested = 0
        for client in (self._clear_global_client, self._clear_local_client):
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())
                requested += 1
        self.log_message.emit(f"已请求清理 {requested}/2 个代价地图")

    def _emit_periodic_status(self) -> None:
        now = time.monotonic()
        if now - self._last_status_at < 1.0:
            return
        self._last_status_at = now
        nav_ready = bool(self._navigator and self._navigator.server_is_ready())
        # Static-map publishers normally publish only once with transient-local
        # durability; once received, the map remains valid until replaced.
        map_ok = self._map_seen_at > 0.0
        pose_ok = (
            now - self._map_pose_seen_at < 3.0
            if map_ok else now - self._pose_seen_at < 3.0
        )
        if nav_ready and map_ok and pose_ok:
            self.status_updated.emit("ready", "Nav2、地图、位姿均正常")
        else:
            missing = []
            if not nav_ready:
                missing.append("Nav2")
            if not map_ok:
                missing.append("地图")
            if not pose_ok:
                missing.append("位姿")
            self.status_updated.emit("waiting", "等待: " + " / ".join(missing))

    def _cleanup(self) -> None:
        try:
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
