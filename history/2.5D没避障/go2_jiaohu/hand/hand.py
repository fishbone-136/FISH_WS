import cv2
import os
import time
import math
import threading
import mediapipe as mp

# 导入宇树 SDK
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

# ==========================================
# 1. 视觉与手势识别层 (极速版)
# ==========================================
class HandGestureRecognizer:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=2,
            model_complexity=0,  # 强制使用最快模型
            min_detection_confidence=0.6,
            min_tracking_confidence=0.5,
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.draw_spec = self.mp_draw.DrawingSpec(thickness=2, circle_radius=2)

    @staticmethod
    def _joint_angle(a, b, c):
        ba = (a.x - b.x, a.y - b.y, a.z - b.z)
        bc = (c.x - b.x, c.y - b.y, c.z - b.z)
        dot = ba[0] * bc[0] + ba[1] * bc[1] + ba[2] * bc[2]
        mag_ba = math.hypot(ba[0], ba[1], ba[2])
        mag_bc = math.hypot(bc[0], bc[1], bc[2])
        if mag_ba < 1e-6 or mag_bc < 1e-6:
            return 180.0
        cos_val = max(-1.0, min(1.0, dot / (mag_ba * mag_bc)))
        return math.degrees(math.acos(cos_val))

    def _get_finger_states(self, hand_landmarks):
        lm = hand_landmarks.landmark
        states = []
        states.append(self._joint_angle(lm[2], lm[3], lm[4]) > 150)
        fingers = [(5, 6, 8), (9, 10, 12), (13, 14, 16), (17, 18, 20)]
        for mcp, pip, tip in fingers:
            states.append(self._joint_angle(lm[mcp], lm[pip], lm[tip]) > 150)
        return states

    def _classify_gesture(self, fingers, hand_landmarks):
        thumb, index, middle, ring, pinky = fingers
        lm = hand_landmarks.landmark
        hand_size = max(0.01, math.hypot(lm[9].x - lm[0].x, lm[9].y - lm[0].y))
        pinch_dist = math.hypot(lm[4].x - lm[8].x, lm[4].y - lm[8].y)
        
        if pinch_dist < hand_size * 0.3 and middle and ring and pinky: return "ok"
        
        count = sum(fingers)
        if count == 0: return "fist"
        if count == 5: return "open_palm"
        if index and not middle and not ring and not pinky: return "index_only"
        return "unknown"

    def process_frame(self, frame):
        h, w, _ = frame.shape
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_rgb.flags.writeable = False
        result = self.hands.process(img_rgb)
        
        current_gestures = {}
        hand_landmark_counts = {}
        
        if result.multi_hand_landmarks:
            for idx, hand_landmarks in enumerate(result.multi_hand_landmarks):
                handedness = result.multi_handedness[idx].classification[0].label
                landmark_count = len(hand_landmarks.landmark)
                
                # 绘制手部
                self.mp_draw.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS,
                    self.draw_spec, self.draw_spec
                )
                
                fingers = self._get_finger_states(hand_landmarks)
                gesture = self._classify_gesture(fingers, hand_landmarks)
                
                current_gestures[handedness] = gesture
                hand_landmark_counts[handedness] = landmark_count
                
                wx, wy = int(hand_landmarks.landmark[0].x * w), int(hand_landmarks.landmark[0].y * h)
                cv2.putText(frame, f"{handedness}: {gesture} ({landmark_count})", (wx - 80, wy - 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        return frame, current_gestures, hand_landmark_counts


# ==========================================
# 2. 硬件控制层 (多线程异步高频版)
# ==========================================
class Go2RobotController:
    def __init__(self, network_interface=None):
        print(f"[硬件] 正在连接 Go2 (接口: {network_interface})...")
        try:
            network_interface = (
                network_interface or os.environ.get("GO2_NETWORK_INTERFACE", "eth0")
            ).strip() or "eth0"
            ChannelFactoryInitialize(0, network_interface)
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(10.0)
            self.sport_client.Init()
            print("[硬件] Go2 初始化成功！")
            self.is_connected = True
        except Exception as e:
            print(f"[警告] Go2 连接失败 (处于脱机测试模式)。错误信息: {e}")
            self.is_connected = False

        self.linear_speed = 0.45
        self.yaw_speed = 0.7

        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vyaw = 0.0

        self._running = True
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

    def _control_loop(self):
        while self._running:
            if self.is_connected:
                try:
                    if self.target_vx != 0 or self.target_vy != 0 or self.target_vyaw != 0:
                        self.sport_client.Move(self.target_vx, self.target_vy, self.target_vyaw)
                except:
                    pass
            time.sleep(0.05)

    def execute_command(self, cmd_text):
        if not self.is_connected:
            print(f"[脱机] 状态 -> {cmd_text}")

        if cmd_text == "停止":
            self.target_vx, self.target_vy, self.target_vyaw = 0.0, 0.0, 0.0
            if self.is_connected: self.sport_client.StopMove()
            
        elif cmd_text == "前进":
            self.target_vx, self.target_vy, self.target_vyaw = self.linear_speed, 0.0, 0.0
        elif cmd_text == "后退":
            self.target_vx, self.target_vy, self.target_vyaw = -self.linear_speed, 0.0, 0.0
            
        elif cmd_text == "向左转":
            self.target_vx, self.target_vy, self.target_vyaw = 0.0, 0.0, self.yaw_speed
        elif cmd_text == "向右转":
            self.target_vx, self.target_vy, self.target_vyaw = 0.0, 0.0, -self.yaw_speed
            
        elif cmd_text == "趴下":
            self.target_vx, self.target_vy, self.target_vyaw = 0.0, 0.0, 0.0
            if self.is_connected: self.sport_client.StandDown()
            
        elif cmd_text == "起立":
            self.target_vx, self.target_vy, self.target_vyaw = 0.0, 0.0, 0.0
            if self.is_connected: self.sport_client.StandUp()

    def shutdown(self):
        self._running = False
        self.execute_command("停止")
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)


# ==========================================
# 3. 核心应用控制逻辑
# ==========================================
class RobotDogApp:
    def __init__(self):
        self.vision = HandGestureRecognizer()
        self.robot = Go2RobotController()
        
        self.cap = cv2.VideoCapture(4)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) 

        self.is_system_active = False
        self.is_lying_down = False
        self.current_movement_state = "停止" 
        
        self.CONFIRM_FRAMES = 2
        self.last_toggle_time = 0
        self.cmd_candidate = None
        self.cmd_stable_frames = 0

        # 新增：手势锁定功能
        self.locked_hands = None          # None 表示未锁定，列表表示锁定的手 (如 ['Left', 'Right'])
        self.MIN_LANDMARKS = 18           # 单手时最小有效节点数（防止手部被裁切）

    def _determine_raw_command(self, current_gestures, landmark_counts):
        # 安全检查：单手且节点不足时强制停止
        if len(current_gestures) == 1:
            hand_label = list(current_gestures.keys())[0]
            count = landmark_counts.get(hand_label, 0)
            if count < self.MIN_LANDMARKS:
                print(f"[安全] 单手节点不足 ({count}/{21})，强制停止")
                return "停止"

        # 应用锁定逻辑
        if self.locked_hands is not None:
            filtered_gestures = {k: v for k, v in current_gestures.items() if k in self.locked_hands}
            if not filtered_gestures:
                return "停止" if self.is_system_active else None
            current_gestures = filtered_gestures

        if not current_gestures:
            return "停止" if self.is_system_active else None
            
        gesture_vals = list(current_gestures.values())
        
        if "ok" in gesture_vals:
            return "切换系统"

        if not self.is_system_active: return None
        if all(g == "unknown" for g in gesture_vals): return "停止"

        hand_count = len(current_gestures)

        if self.is_lying_down:
            if hand_count == 2 and all(g == "index_only" for g in gesture_vals): return "起立"
            return "停止"

        if hand_count == 2:
            if all(g == "fist" for g in gesture_vals): return "趴下"
            if all(g == "open_palm" for g in gesture_vals): return "后退"
            if all(g == "index_only" for g in gesture_vals): return "起立"
            return "停止"

        if hand_count == 1:
            hand_label = list(current_gestures.keys())[0]
            gesture = gesture_vals[0]
            if gesture == "open_palm": return "前进"
            if gesture == "fist": return "停止"
            if hand_label == "Left" and gesture == "index_only": return "向左转"
            if hand_label == "Right" and gesture == "index_only": return "向右转"

        return "停止"

    def run(self):
        try:
            while self.cap.isOpened():
                ret, frame = self.cap.read()
                if not ret: break

                frame = cv2.flip(frame, 1)
                current_time = time.time()
                
                frame, current_gestures, landmark_counts = self.vision.process_frame(frame)
                raw_command = self._determine_raw_command(current_gestures, landmark_counts)
                
                # 防抖逻辑
                if raw_command == self.cmd_candidate:
                    if raw_command is not None:
                        self.cmd_stable_frames += 1
                else:
                    self.cmd_candidate = raw_command
                    self.cmd_stable_frames = 1

                if self.cmd_candidate is not None:
                    # 系统开关
                    if self.cmd_stable_frames == self.CONFIRM_FRAMES and self.cmd_candidate == "切换系统":
                        if current_time - self.last_toggle_time > 1.0:
                            self.is_system_active = not self.is_system_active
                            self.last_toggle_time = current_time
                            
                            if not self.is_system_active:
                                self.robot.execute_command("停止")
                                self.current_movement_state = "停止"
                            else:
                                self.current_movement_state = "就绪"

                    # 运动指令
                    elif self.cmd_stable_frames >= self.CONFIRM_FRAMES and self.is_system_active and self.cmd_candidate != "切换系统":
                        if self.cmd_candidate != self.current_movement_state:
                            self.current_movement_state = self.cmd_candidate
                            
                            if self.cmd_candidate == "趴下": self.is_lying_down = True
                            elif self.cmd_candidate == "起立": self.is_lying_down = False
                            
                            self.robot.execute_command(self.cmd_candidate)

                # ==================== UI 渲染 ====================
                status = "STANDBY" if not self.is_system_active else ("LYING DOWN" if self.is_lying_down else "ACTIVE")
                color = (150, 150, 150) if not self.is_system_active else ((0, 0, 255) if self.is_lying_down else (0, 255, 0))
                cv2.putText(frame, f"SYSTEM: {status}", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

                if current_time - self.last_toggle_time < 0.8:
                    toggle_msg = "SYSTEM ON" if self.is_system_active else "SYSTEM OFF"
                    msg_color = (0, 255, 0) if self.is_system_active else (0, 0, 255)
                    cv2.putText(frame, toggle_msg, (180, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.5, msg_color, 4)

                if self.is_system_active:
                    cv2.putText(frame, f"ACTION: {self.current_movement_state}", (10, 80), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 100, 0), 3)

                # 锁定状态显示
                if self.locked_hands:
                    cv2.putText(frame, f"LOCKED: {self.locked_hands}", (10, frame.shape[0] - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                cv2.putText(frame, f"FPS: {int(self.cap.get(cv2.CAP_PROP_FPS))}", 
                            (frame.shape[1] - 100, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
                
                cv2.imshow("Robot Dog - Pro Controller", frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # q 或 ESC 退出
                    break
                elif key == ord(' '):  # 空格键 - 锁定当前手
                    if current_gestures:
                        self.locked_hands = list(current_gestures.keys())
                        print(f"[锁定] 已锁定手部: {self.locked_hands}")
                    else:
                        print("[锁定] 当前没有检测到手")
                elif key == ord('b') or key == ord('B'):  # B键 - 解锁
                    if self.locked_hands:
                        self.locked_hands = None
                        print("[锁定] 已解除手部锁定")
                    
        finally:
            print("\n[系统] 正在安全退出并停止狗的运动...")
            self.robot.shutdown()
            self.cap.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    app = RobotDogApp()
    app.run()
