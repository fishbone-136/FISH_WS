import cv2


def main():
    # 打开默认摄像头（索引 0），如果有多个摄像头可改为 1, 2...
    cap = cv2.VideoCapture(1)

    if not cap.isOpened():
        print("❌ 无法打开摄像头，请检查设备连接")
        return

    # 设置分辨率（可选）
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("🎥 摄像头已启动 — 按 Q 退出，按 S 截图")
    screenshot_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ 无法读取画面帧")
            break

        # 显示画面
        cv2.imshow("Camera - 实时画面", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            print("👋 退出")
            break
        elif key == ord('s'):
            screenshot_count += 1
            filename = f"screenshot_{screenshot_count:03d}.png"
            cv2.imwrite(filename, frame)
            print(f"📸 截图已保存: {filename}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
