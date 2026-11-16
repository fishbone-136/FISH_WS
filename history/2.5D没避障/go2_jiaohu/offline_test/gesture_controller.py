"""
手势指令识别器 — UI 测试版
纯手势分类 + 防抖 + 状态机，不依赖 unitree_sdk2py 等机器人硬件库。
指令映射完全对齐 hand/hand.py 的 _determine_raw_command 逻辑。
可直接在开发机上运行，用于 UI 手势交互功能测试。
"""

import math
import time


class GestureController:
    """基于 MediaPipe 21 点手部关键点的手势 → 运动指令映射

    完全对齐 hand.py 的指令体系：
        前进 / 后退 / 停止 / 向左转 / 向右转 / 趴下 / 起立 / 切换系统
    """

    # ---- 指令常量 ----
    CMD_FORWARD    = "前进"
    CMD_BACKWARD   = "后退"
    CMD_STOP       = "停止"
    CMD_TURN_LEFT  = "向左转"
    CMD_TURN_RIGHT = "向右转"
    CMD_LIE_DOWN   = "趴下"
    CMD_STAND_UP   = "起立"
    CMD_TOGGLE     = "切换系统"

    def __init__(self, confirm_frames: int = 3):
        self._confirm_frames = confirm_frames
        self._cmd_candidate: str | None = None
        self._cmd_stable_frames: int = 0
        self._last_toggle_time: float = 0.0

        # ---- 状态机 (对齐 hand.py RobotDogApp) ----
        self.is_system_active: bool = False
        self.is_lying_down: bool = False
        self.locked_hands: list | None = None
        self.MIN_LANDMARKS: int = 18

    # ==================================================================
    # 几何工具
    # ==================================================================

    @staticmethod
    def _joint_angle(ax, ay, az, bx, by, bz, cx, cy, cz):
        """以 b 为顶点，计算 ∠abc（度数）"""
        ba = (ax - bx, ay - by, az - bz)
        bc = (cx - bx, cy - by, cz - bz)
        dot = ba[0] * bc[0] + ba[1] * bc[1] + ba[2] * bc[2]
        mag_ba = math.hypot(*ba)
        mag_bc = math.hypot(*bc)
        if mag_ba < 1e-6 or mag_bc < 1e-6:
            return 180.0
        cos_val = max(-1.0, min(1.0, dot / (mag_ba * mag_bc)))
        return math.degrees(math.acos(cos_val))

    # ==================================================================
    # 单只手 → 手势分类 (对齐 HandGestureRecognizer._get_finger_states + _classify_gesture)
    # ==================================================================

    @classmethod
    def _classify_one_hand(cls, lm: list):
        """输入 21 个 (x,y,z) 归一化坐标，返回手势名称。"""
        thumb  = cls._joint_angle(*lm[2], *lm[3], *lm[4]) > 150
        index  = cls._joint_angle(*lm[5], *lm[6], *lm[8]) > 150
        middle = cls._joint_angle(*lm[9], *lm[10], *lm[12]) > 150
        ring   = cls._joint_angle(*lm[13], *lm[14], *lm[16]) > 150
        pinky  = cls._joint_angle(*lm[17], *lm[18], *lm[20]) > 150

        fingers = [thumb, index, middle, ring, pinky]

        hand_scale = max(0.01, math.hypot(lm[9][0] - lm[0][0], lm[9][1] - lm[0][1]))
        pinch_dist = math.hypot(lm[4][0] - lm[8][0], lm[4][1] - lm[8][1])

        if pinch_dist < hand_scale * 0.3 and middle and ring and pinky:
            return "ok"

        n_fingers = sum(fingers)
        if n_fingers == 0:
            return "fist"
        if n_fingers == 5:
            return "open_palm"
        if index and not middle and not ring and not pinky:
            return "index_only"

        return "unknown"

    # ==================================================================
    # 多只手 → gestures dict (对齐 HandGestureRecognizer.process_frame)
    # ==================================================================

    @classmethod
    def classify_all(cls, landmarks_list: list, handedness_list: list):
        """
        输入关键点列表和左右手标签，返回 {handedness: gesture_name}。
        landmarks_list: [[(x,y,z)*21], ...]
        handedness_list: ["Left", "Right", ...]
        """
        gestures = {}
        for lm, hand_label in zip(landmarks_list, handedness_list):
            gestures[hand_label] = cls._classify_one_hand(lm)
        return gestures

    # ==================================================================
    # 状态机 — 指令决策 (完整对齐 hand.py _determine_raw_command)
    # ==================================================================

    def _determine_command(self, current_gestures: dict, landmark_counts: dict | None = None):
        """
        根据当前手势 + 系统状态，返回原始指令字符串。
        完全对齐 hand.py RobotDogApp._determine_raw_command()。
        """
        if landmark_counts is None:
            landmark_counts = {}

        # 安全检查：单手且节点不足时强制停止
        if len(current_gestures) == 1:
            hand_label = list(current_gestures.keys())[0]
            count = landmark_counts.get(hand_label, 21)
            if count < self.MIN_LANDMARKS:
                return self.CMD_STOP

        # 应用锁定逻辑
        gestures = current_gestures
        if self.locked_hands is not None:
            gestures = {k: v for k, v in current_gestures.items() if k in self.locked_hands}
            if not gestures:
                return self.CMD_STOP if self.is_system_active else None

        if not gestures:
            return self.CMD_STOP if self.is_system_active else None

        gesture_vals = list(gestures.values())

        # OK 手势 → 切换系统
        if "ok" in gesture_vals:
            return self.CMD_TOGGLE

        # 系统未激活时不响应运动指令
        if not self.is_system_active:
            return None

        # 全是 unknown → 停止
        if all(g == "unknown" for g in gesture_vals):
            return self.CMD_STOP

        hand_count = len(gestures)

        # ---- 趴下状态 ----
        if self.is_lying_down:
            if hand_count == 2 and all(g == "index_only" for g in gesture_vals):
                return self.CMD_STAND_UP
            return self.CMD_STOP

        # ---- 双手 ----
        if hand_count == 2:
            if all(g == "fist" for g in gesture_vals):
                return self.CMD_LIE_DOWN
            if all(g == "open_palm" for g in gesture_vals):
                return self.CMD_BACKWARD
            if all(g == "index_only" for g in gesture_vals):
                return self.CMD_STAND_UP
            return self.CMD_STOP

        # ---- 单手 ----
        if hand_count == 1:
            hand_label = list(gestures.keys())[0]
            gesture = gesture_vals[0]
            if gesture == "open_palm":
                return self.CMD_FORWARD
            if gesture == "fist":
                return self.CMD_STOP
            if hand_label == "Left" and gesture == "index_only":
                return self.CMD_TURN_LEFT
            if hand_label == "Right" and gesture == "index_only":
                return self.CMD_TURN_RIGHT

        return self.CMD_STOP

    # ==================================================================
    # 防抖 + 状态转移 (对齐 hand.py run loop 中的防抖 + 状态更新)
    # ==================================================================

    def update(self, gestures_dict: dict, landmark_counts: dict | None = None):
        """
        每帧调用。传入 {handedness: gesture_name}，返回 (confirmed_command, toggle_event)。

        返回值:
            (command_str, toggle_str)
            - command_str: 确认后的运动指令，如 "前进"/"停止"/None(无变化)
            - toggle_str: 系统切换事件，如 "系统已激活"/"系统已待机"/None

        防抖逻辑 + 状态机完全对齐 hand.py。
        """
        if landmark_counts is None:
            landmark_counts = {}

        raw_command = self._determine_command(gestures_dict, landmark_counts)
        current_time = time.time()

        # ---- 防抖计数 ----
        if raw_command == self._cmd_candidate:
            if raw_command is not None:
                self._cmd_stable_frames += 1
        else:
            self._cmd_candidate = raw_command
            self._cmd_stable_frames = 1

        result_cmd = None
        result_toggle = None

        if self._cmd_candidate is not None:
            # ---- 系统开关 ----
            if self._cmd_stable_frames == self._confirm_frames and self._cmd_candidate == self.CMD_TOGGLE:
                if current_time - self._last_toggle_time > 1.0:
                    self.is_system_active = not self.is_system_active
                    self._last_toggle_time = current_time
                    result_toggle = "系统已激活" if self.is_system_active else "系统已待机"
                    if not self.is_system_active:
                        result_cmd = self.CMD_STOP

            # ---- 运动指令 ----
            elif self._cmd_stable_frames >= self._confirm_frames and self.is_system_active and self._cmd_candidate != self.CMD_TOGGLE:
                result_cmd = self._cmd_candidate

                # 状态转移
                if self._cmd_candidate == self.CMD_LIE_DOWN:
                    self.is_lying_down = True
                elif self._cmd_candidate == self.CMD_STAND_UP:
                    self.is_lying_down = False

        return result_cmd, result_toggle

    # ==================================================================
    # 状态查询
    # ==================================================================

    @property
    def status_text(self) -> str:
        """当前系统状态文本。"""
        if not self.is_system_active:
            return "STANDBY"
        return "LYING DOWN" if self.is_lying_down else "ACTIVE"

    def reset(self):
        """重置防抖状态（模式切换时调用）。"""
        self._cmd_candidate = None
        self._cmd_stable_frames = 0
