"""
基于 MediaPipe 的手部 21 点关键点实时检测。
按 Q 退出。
"""

import importlib.util

import cv2

# MediaPipe 顶层导入会连带初始化音频/GPU 组件，在部分 Jetson 上耗时很长。
# 此处只检查是否安装，真正的 Hands 模型推迟到首次处理手势帧时加载，避免阻塞
# 邦邦的语音与导航 UI 启动。
HAS_MEDIAPIPE = importlib.util.find_spec("mediapipe") is not None


class HandDetector:
    """手部关键点检测器，封装 MediaPipe Hands。"""

    def __init__(self, max_num_hands: int = 2,
                 min_detection_confidence: float = 0.7,
                 min_tracking_confidence: float = 0.5):
        self.hands = None
        self._max_num_hands = max_num_hands
        self._min_detection_confidence = min_detection_confidence
        self._min_tracking_confidence = min_tracking_confidence
        self._load_attempted = False
        self._mp_hands = None
        self._mp_drawing = None
        self._mp_drawing_styles = None

    def _ensure_loaded(self) -> bool:
        if self.hands is not None:
            return True
        if self._load_attempted or not HAS_MEDIAPIPE:
            return False
        self._load_attempted = True
        try:
            import mediapipe as mp

            self._mp_hands = mp.solutions.hands
            self._mp_drawing = mp.solutions.drawing_utils
            self._mp_drawing_styles = mp.solutions.drawing_styles
            self.hands = self._mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=self._max_num_hands,
                min_detection_confidence=self._min_detection_confidence,
                min_tracking_confidence=self._min_tracking_confidence,
            )
            return True
        except (ImportError, AttributeError, RuntimeError):
            self.hands = None
            return False

    @property
    def available(self) -> bool:
        return HAS_MEDIAPIPE and (self.hands is not None or not self._load_attempted)

    def detect(self, bgr_frame):
        """
        处理一帧 BGR 图像，返回 (标注图像, 关键点列表, 左右手标签列表)。
        关键点列表: [[(x, y, z) * 21], ...]  每只手 21 个归一化坐标。
        左右手标签: ["Left", "Right", ...]  与关键点列表一一对应。
        """
        if not self._ensure_loaded():
            return bgr_frame, [], []

        rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = self.hands.process(rgb)
        rgb.flags.writeable = True
        annotated = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        all_landmarks = []
        all_handedness = []

        if results.multi_hand_landmarks:
            for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                # 收集 21 个关键点
                points = []
                for lm in hand_landmarks.landmark:
                    points.append((lm.x, lm.y, lm.z))
                all_landmarks.append(points)

                # 左右手标签
                handedness = results.multi_handedness[idx].classification[0].label
                all_handedness.append(handedness)

                # 绘制骨架
                self._mp_drawing.draw_landmarks(
                    annotated,
                    hand_landmarks,
                    self._mp_hands.HAND_CONNECTIONS,
                    self._mp_drawing_styles.get_default_hand_landmarks_style(),
                    self._mp_drawing_styles.get_default_hand_connections_style(),
                )

        return annotated, all_landmarks, all_handedness

    def close(self):
        if self.hands is not None:
            self.hands.close()


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ 无法打开摄像头")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    detector = HandDetector()
    if not detector.available:
        print("MediaPipe 不可用：请按教程安装兼容当前板卡架构的版本")
        cap.release()
        return
    print("🖐 手部关键点检测已启动 — 按 Q 退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        annotated, landmarks, _ = detector.detect(frame)

        # 顶部显示检测数量
        count = len(landmarks)
        cv2.putText(annotated, f"检测到 {count} 只手", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 240, 255), 2, cv2.LINE_AA)

        cv2.imshow("Hand Landmarks - 手部关键点", annotated)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    detector.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
