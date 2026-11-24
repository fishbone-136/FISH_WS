"""Go2 运动接口：默认通过隔离进程调用 ``unitree_sdk2_python``。

SDK2 使用的 CycloneDDS 与 ROS 2 运行库版本可能冲突，因此动作客户端始终放在
独立进程中。父进程只有在 SDK 初始化、服务探测和首个 API 调用都成功后才返回
动作已接受。保留 ROS 桥作为显式配置的兼容后端。
"""

from __future__ import annotations

import atexit
import json
import logging
import os
import select
import subprocess
import sys
import threading
import time

from . import config


logging.basicConfig(level=logging.INFO)


class _SdkProcessBridge:
    def __init__(self, interface: str):
        self.interface = interface
        self._lock = threading.Lock()
        self._process: subprocess.Popen | None = None
        self.error = ""
        self.api_version = ""
        self._request_counter = 0
        self._start()

    @property
    def ready(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def _start(self) -> None:
        project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        env = os.environ.copy()
        # unitree_sdk2py 固定搭配 CycloneDDS 0.10.x；ROS Humble 的库若排在
        # 前面会在 import 或 ChannelFactoryInitialize 时发生 ABI 冲突。
        library_paths = ["/usr/local/lib"]
        if env.get("LD_LIBRARY_PATH"):
            library_paths.append(env["LD_LIBRARY_PATH"])
        env["LD_LIBRARY_PATH"] = os.pathsep.join(library_paths)
        try:
            self._process = subprocess.Popen(
                [sys.executable, "-u", "-m", "voice.sdk_worker", self.interface],
                cwd=project_dir,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=None,
                env=env,
                text=True,
                bufsize=1,
            )
            message = self._read_response(max(4.0, config.ROBOT_SDK_RPC_TIMEOUT + 2.0))
            if message.get("status") == "ready":
                self.api_version = str(message.get("api_version", "unknown"))
                logging.info(
                    "Unitree SDK2 运动桥已连接: interface=%s api=%s",
                    self.interface,
                    self.api_version,
                )
                return
            self.error = str(message.get("error", "Unitree SDK2 运动桥初始化失败"))
        except Exception as exc:
            self.error = str(exc)

        logging.error("无法连接 Unitree SDK2 运动服务（实机动作已禁用）: %s", self.error)
        self.shutdown(send_stop=False)

    def send_action(
        self, action_name: str, speed: float, duration: float, angle: float
    ) -> bool:
        return self._send_payload({
            'action': action_name, 'speed': float(speed),
            'duration': float(duration), 'angle': float(angle),
        })

    def send_visual_velocity(self, vx: float, wz: float) -> bool:
        return self._send_payload({'command': 'visual_velocity', 'vx': float(vx),
            'wz': float(wz), 'issued_at': time.monotonic(), 'lease': .25})

    def send_follow_velocity(self, vx: float, wz: float) -> bool:
        return self._send_payload({'command': 'follow_velocity', 'vx': float(vx),
            'wz': float(wz), 'issued_at': time.monotonic(), 'lease': .25})

    def _send_payload(self, payload) -> bool:
        return self._request_payload(payload) is not None

    def read_sport_state(self) -> dict:
        response = self._request_payload({'command': 'read_sport_state'})
        if response is None:
            raise ValueError(self.error)
        return response['state']

    def _request_payload(self, payload):
        if not self.ready or self._process is None or self._process.stdin is None:
            self.error = self.error or "Unitree SDK2 运动桥不可用"
            return None
        with self._lock:
            self._request_counter += 1
            request_id = f"{os.getpid()}-{time.monotonic_ns()}-{self._request_counter}"
            payload = {**payload, 'request_id': request_id}
            try:
                self._process.stdin.write(json.dumps(payload, ensure_ascii=False) + "\n")
                self._process.stdin.flush()
                wait_seconds = max(
                    0.2,
                    config.ROBOT_COMMAND_ACK_TIMEOUT,
                    config.ROBOT_SDK_RPC_TIMEOUT + 0.5,
                )
                deadline = time.monotonic() + wait_seconds
                while time.monotonic() < deadline:
                    response = self._read_response(deadline - time.monotonic())
                    if str(response.get("request_id", "")) != request_id:
                        continue
                    if response.get("status") == "accepted":
                        self.error = ""
                        return response
                    self.error = str(response.get("error", "Unitree SDK2 拒绝动作"))
                    return None
                self.error = "等待 Unitree SDK2 动作回执超时"
            except (BrokenPipeError, OSError, RuntimeError, TimeoutError) as exc:
                self.error = str(exc)
            return None

    def _read_response(self, timeout: float) -> dict:
        if self._process is None or self._process.stdout is None:
            raise RuntimeError("Unitree SDK2 运动桥未启动")
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            if self._process.poll() is not None:
                raise RuntimeError(
                    f"Unitree SDK2 运动桥异常退出: {self._process.returncode}"
                )
            readable, _, _ = select.select(
                [self._process.stdout], [], [], min(0.1, deadline - time.monotonic())
            )
            if not readable:
                continue
            line = self._process.stdout.readline().strip()
            if not line:
                continue
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                logging.info("Unitree SDK2 运动桥: %s", line)
        raise TimeoutError("等待 Unitree SDK2 运动桥响应超时")

    def shutdown(self, send_stop: bool = True) -> None:
        process = self._process
        if process is None:
            return
        if process.poll() is None and process.stdin is not None:
            try:
                process.stdin.write(json.dumps({
                    "command": "shutdown" if send_stop else "quit"
                }) + "\n")
                process.stdin.flush()
            except (BrokenPipeError, OSError):
                pass
        try:
            process.wait(timeout=max(3.0, config.ROBOT_SDK_RPC_TIMEOUT + 1.0))
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
        self._process = None


class _RosProcessBridge:
    def __init__(self, interface: str):
        self.interface = interface
        self._lock = threading.Lock()
        self._process: subprocess.Popen | None = None
        self.error = ""
        self._request_counter = 0
        self._start()

    @property
    def ready(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def _start(self) -> None:
        project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        try:
            self._process = subprocess.Popen(
                [sys.executable, "-u", "-m", "voice.ros_command_worker"],
                cwd=project_dir,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=None,
                env=os.environ.copy(),
                text=True,
                bufsize=1,
            )
            message = self._read_response(8.0)
            if message.get("status") == "ready":
                logging.info(
                    "ROS 2 交互运动桥已就绪: command=%s status=%s",
                    config.ROBOT_COMMAND_TOPIC,
                    config.ROBOT_COMMAND_STATUS_TOPIC,
                )
                return
            self.error = str(message.get("error", "ROS 2 运动桥初始化失败"))
        except Exception as exc:
            self.error = str(exc)

        logging.error("无法初始化 ROS 2 交互运动桥（实机动作已禁用）: %s", self.error)
        self.shutdown(send_stop=False)

    def send_action(self, action_name: str, speed: float, duration: float, angle: float) -> bool:
        if not self.ready or self._process is None or self._process.stdin is None:
            self.error = self.error or "ROS 2 交互运动桥不可用"
            return False
        with self._lock:
            self._request_counter += 1
            request_id = f"{os.getpid()}-{time.monotonic_ns()}-{self._request_counter}"
            payload = {
                "request_id": request_id,
                "action": action_name,
                "speed": float(speed),
                "duration": float(duration),
                "angle": float(angle),
            }
            try:
                self._process.stdin.write(json.dumps(payload, ensure_ascii=False) + "\n")
                self._process.stdin.flush()
                deadline = time.monotonic() + max(0.2, config.ROBOT_COMMAND_ACK_TIMEOUT)
                while time.monotonic() < deadline:
                    response = self._read_response(deadline - time.monotonic())
                    if str(response.get("request_id", "")) != request_id:
                        continue
                    if response.get("status") == "accepted":
                        self.error = ""
                        return True
                    self.error = str(response.get("error", "go2_base 拒绝动作"))
                    return False
                self.error = "等待 go2_base 动作回执超时"
            except (BrokenPipeError, OSError, RuntimeError, TimeoutError) as exc:
                self.error = str(exc)
            return False

    def _read_response(self, timeout: float) -> dict:
        if self._process is None or self._process.stdout is None:
            raise RuntimeError("ROS 2 交互运动桥未启动")
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            if self._process.poll() is not None:
                raise RuntimeError(f"ROS 2 交互运动桥异常退出: {self._process.returncode}")
            readable, _, _ = select.select(
                [self._process.stdout], [], [], min(0.1, deadline - time.monotonic())
            )
            if not readable:
                continue
            line = self._process.stdout.readline().strip()
            if not line:
                continue
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                logging.info("ROS 2 交互运动桥: %s", line)
        raise TimeoutError("等待 ROS 2 交互运动桥响应超时")

    def shutdown(self, send_stop: bool = True) -> None:
        process = self._process
        if process is None:
            return
        if process.poll() is None and process.stdin is not None:
            try:
                process.stdin.write(json.dumps({
                    "command": "shutdown" if send_stop else "quit"
                }) + "\n")
                process.stdin.flush()
            except (BrokenPipeError, OSError):
                pass
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
        self._process = None


_BRIDGE_LOCK = threading.Lock()
_SHARED_BRIDGE: _SdkProcessBridge | _RosProcessBridge | None = None
_SHARED_BRIDGE_KEY: tuple[str, str] | None = None


def _get_bridge(interface: str) -> _SdkProcessBridge | _RosProcessBridge:
    global _SHARED_BRIDGE, _SHARED_BRIDGE_KEY
    with _BRIDGE_LOCK:
        key = (config.ROBOT_MOTION_BACKEND, interface)
        if (
            _SHARED_BRIDGE is None
            or not _SHARED_BRIDGE.ready
            or _SHARED_BRIDGE_KEY != key
        ):
            if _SHARED_BRIDGE is not None:
                _SHARED_BRIDGE.shutdown()
            bridge_class = (
                _SdkProcessBridge
                if config.ROBOT_MOTION_BACKEND == "sdk"
                else _RosProcessBridge
            )
            _SHARED_BRIDGE = bridge_class(interface)
            _SHARED_BRIDGE_KEY = key
        return _SHARED_BRIDGE


def _shutdown_shared_bridge() -> None:
    global _SHARED_BRIDGE, _SHARED_BRIDGE_KEY
    with _BRIDGE_LOCK:
        if _SHARED_BRIDGE is not None:
            _SHARED_BRIDGE.shutdown()
            _SHARED_BRIDGE = None
            _SHARED_BRIDGE_KEY = None


atexit.register(_shutdown_shared_bridge)


class Go2RobotController:
    """向所选后端发送动作，并同步等待其首个 API 调用回执。"""

    def __init__(self, interface=None, *, auto_connect=True):
        self.interface = (interface or config.ROBOT_INTERFACE).strip() or "eth0"
        self.backend = config.ROBOT_MOTION_BACKEND
        self._bridge = _get_bridge(self.interface) if auto_connect else None
        self.client = self if self._bridge is not None and self._bridge.ready else None

    @property
    def last_error(self) -> str:
        return self._bridge.error if self._bridge is not None else "运动接口尚未连接"

    def execute_action(self, action_name, speed=0.3, duration=2.0, angle=0.0) -> bool:
        if self._bridge is None or not self._bridge.ready:
            self.client = None
            if not self.reconnect():
                logging.warning(
                    "[%s 离线] 机器狗动作未发送: %s (%s)",
                    self.backend,
                    action_name,
                    self.last_error or "运动桥不可用",
                )
                return False
        if self.client is None:
            logging.warning(
                "[离线] 机器狗动作未发送: %s, speed=%s, duration=%s, angle=%s",
                action_name, speed, duration, angle,
            )
            return False
        accepted = self._bridge.send_action(action_name, speed, duration, angle)
        if not accepted:
            logging.error(
                "%s 后端未接受动作 %s: %s",
                self.backend,
                action_name,
                self._bridge.error,
            )
        return accepted

    def reconnect(self) -> bool:
        if self._bridge is None or not self._bridge.ready:
            self._bridge = _get_bridge(self.interface)
        self.client = self if self._bridge.ready else None
        return self.client is not None

    def send_visual_velocity(self, vx: float, wz: float) -> bool:
        """Only the SDK backend supports bounded visual feedback leases."""
        if self.backend != 'sdk' or self._bridge is None or not self._bridge.ready:
            return False
        return self._bridge.send_visual_velocity(vx, wz)

    def read_sport_state(self) -> dict:
        """Read stance through the isolated SDK process, without ROS nodes."""
        if self.backend != 'sdk' or self._bridge is None or not self._bridge.ready:
            raise ValueError('SDK 运动桥未连接')
        return self._bridge.read_sport_state()

    def send_follow_velocity(self, vx: float, wz: float) -> bool:
        if self.backend != 'sdk' or self._bridge is None or not self._bridge.ready:
            return False
        return self._bridge.send_follow_velocity(vx, wz)

    def shutdown(self) -> None:
        if self._bridge is not None:
            _shutdown_shared_bridge()
        self.client = None
