"""
摄像头测试工具 —— 同时显示 RGB 可见光 + 深度画面 (CAP_DSHOW 后端)
按 Q 退出，按 1-4 切换 RGB 摄像头索引
"""
import sys
import os
import cv2
import numpy as np

# Windows: 用 DSHOW 后端规避 MSMF 兼容性问题
_BACKEND = getattr(cv2, 'CAP_DSHOW', 0)

# ---- RealSense 可选 ----
HAS_RS = False
rs = None
try:
    import pyrealsense2 as rs
    HAS_RS = True
except ImportError:
    print("[INFO] pyrealsense2 not installed, depth stream unavailable")


def scan_cameras():
    available = []
    for idx in range(5):
        cap = cv2.VideoCapture(idx, _BACKEND)
        if cap.isOpened():
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            available.append((idx, w, h))
            cap.release()
    return available


def open_camera(index: int, width=1280, height=720):
    cap = cv2.VideoCapture(index, _BACKEND)
    if not cap.isOpened():
        return None, (0, 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    return cap, (actual_w, actual_h)


def main():
    print("=" * 55)
    print("  摄像头测试 — RGB (CAP_DSHOW) + Depth (RealSense)")
    print("=" * 55)

    cams = scan_cameras()
    print(f"\nDetected {len(cams)} RGB cameras:")
    for idx, w, h in cams:
        print(f"  camera_index={idx} -> {w}x{h}")

    if not cams:
        print("[FAIL] No RGB camera detected!")
        return

    # ---- RealSense ----
    depth_pipeline = None
    depth_running = False
    if HAS_RS:
        try:
            ctx = rs.context()
            devs = ctx.query_devices()
            if len(devs) > 0:
                for d in devs:
                    print(f"  RealSense: {d.get_info(rs.camera_info.name)}")
                depth_pipeline = rs.pipeline()
                config = rs.config()
                config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                depth_pipeline.start(config)
                depth_running = True
                print("[OK] RealSense depth stream started")
            else:
                print("[INFO] No RealSense device found")
        except Exception as e:
            print(f"[WARN] RealSense start failed: {e}")

    # ---- RGB ----
    rgb_index = cams[0][0]
    cap, (w, h) = open_camera(rgb_index)
    if cap is None:
        print("[FAIL] Cannot open RGB camera")
        if depth_pipeline:
            depth_pipeline.stop()
        return
    print(f"\n[OK] RGB camera opened: index={rgb_index}, actual={w}x{h}")

    print("\nPress Q to quit | 1-4 switch camera | R cycle resolution\n")
    cv2.namedWindow("Camera Test - Left:RGB | Right:Depth", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera Test - Left:RGB | Right:Depth", 1280, 480)

    resolutions = [(640, 480), (1280, 720), (1920, 1080)]
    res_idx = 1

    while True:
        # --- RGB ---
        ret, rgb = cap.read()
        if not ret:
            rgb = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(rgb, "RGB SIGNAL LOST", (80, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        else:
            cv2.putText(rgb, f"RGB cam={rgb_index} {rgb.shape[1]}x{rgb.shape[0]}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # --- Depth ---
        depth = np.zeros((480, 640, 3), dtype=np.uint8)
        if depth_running and depth_pipeline:
            try:
                frames = depth_pipeline.wait_for_frames(timeout_ms=1000)
                df = frames.get_depth_frame()
                if df:
                    depth_data = np.asanyarray(df.get_data())
                    depth = cv2.applyColorMap(
                        cv2.convertScaleAbs(depth_data, alpha=0.03),
                        cv2.COLORMAP_JET,
                    )
            except Exception:
                pass
            cv2.putText(depth, f"Depth {depth.shape[1]}x{depth.shape[0]}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        else:
            cv2.putText(depth, "DEPTH NOT AVAILABLE", (120, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (100, 100, 255), 2)

        # --- 拼接 ---
        h_target = 480
        if rgb.shape[0] != h_target:
            rgb = cv2.resize(rgb, (int(rgb.shape[1] * h_target / rgb.shape[0]), h_target))
        if depth.shape[0] != h_target:
            depth = cv2.resize(depth, (int(depth.shape[1] * h_target / depth.shape[0]), h_target))
        combined = np.hstack((rgb, depth))
        cv2.imshow("Camera Test - Left:RGB | Right:Depth", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif ord('1') <= key <= ord('4'):
            new_idx = key - ord('0')
            if new_idx != rgb_index:
                print(f"Switch to camera_index={new_idx}")
                cap.release()
                cap, (w, h) = open_camera(new_idx, *resolutions[res_idx])
                if cap is not None:
                    rgb_index = new_idx
                    print(f"  actual={w}x{h}")
                else:
                    cap, (w, h) = open_camera(rgb_index)
                    print(f"  failed, back to {rgb_index}")
        elif key == ord('r') or key == ord('R'):
            res_idx = (res_idx + 1) % len(resolutions)
            tw, th = resolutions[res_idx]
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, tw)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, th)
            aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"Resolution -> {tw}x{th}, actual={aw}x{ah}")

    cap.release()
    if depth_pipeline:
        depth_pipeline.stop()
    cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()
