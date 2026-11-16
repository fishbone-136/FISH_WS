"""
手势指令识别器 — 部署版
继承离线版 GestureController，追加经统一运动接口的机器人控制。
"""
import logging
import time

from offline_test.gesture_controller import GestureController as _MockGestureController
from voice.robot_interface import Go2RobotController

logging.basicConfig(level=logging.INFO, format="%(asctime)s [Deploy Gesture] %(message)s")


# 指令 → (action_name, speed) 映射
_CMD_MAP = {
    "前进": ("forward", 0.3),
    "后退": ("backward", 0.3),
    "停止": ("stop", 0.0),
    "向左转": ("turn_left", 0.5),
    "向右转": ("turn_right", 0.5),
    "趴下": ("sit", 0.0),
    "起立": ("stand", 0.0),
}


class GestureController(_MockGestureController):
    """手势控制器 — 部署版：识别手势后直接驱动机器狗运动"""

    def __init__(self, confirm_frames: int = 3, network_interface: str | None = None):
        super().__init__(confirm_frames)
        self._robot = Go2RobotController(interface=network_interface)
        self._last_sent_cmd: str | None = None
        self._last_sent_at = 0.0
        # 运动命令采用短租约并定期续期。手势消失、模式切换或进程异常时，
        # SDK/ROS worker 会在租约到期后自动 StopMove，避免运动任务持续执行。
        self._motion_refresh_interval = 0.35
        self._motion_lease = 0.75
        logging.info("手势控制器 (部署版) 初始化完成")

    def update(self, gestures_dict: dict, landmark_counts: dict | None = None):
        cmd, toggle = super().update(gestures_dict, landmark_counts)

        if cmd and self._robot.client is not None:
            action = _CMD_MAP.get(cmd)
            if action:
                action_name, speed = action
                now = time.monotonic()
                is_motion = action_name in {
                    "forward", "backward", "left", "right", "turn_left", "turn_right"
                }
                should_send = cmd != self._last_sent_cmd
                if is_motion and now - self._last_sent_at >= self._motion_refresh_interval:
                    should_send = True
                if should_send:
                    duration = self._motion_lease if is_motion else 0.0
                    self._robot.execute_action(action_name, speed=speed, duration=duration)
                    self._last_sent_cmd = cmd
                    self._last_sent_at = now

        return cmd, toggle

    def stop_motion(self):
        """立即停止运动并清空发送状态；模式切换和丢失手部时调用。"""
        if self._robot.client is not None and self._last_sent_cmd != self.CMD_STOP:
            self._robot.execute_action("stop", speed=0.0, duration=0.0)
        self._last_sent_cmd = self.CMD_STOP
        self._last_sent_at = time.monotonic()
