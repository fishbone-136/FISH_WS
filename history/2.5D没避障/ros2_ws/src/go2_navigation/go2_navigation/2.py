import cv2
import time

# 打开默认摄像头（0=内置摄像头，1/2=外接摄像头）
cap = cv2.VideoCapture(4)

# 检查摄像头是否成功打开
if not cap.isOpened():
    print("错误：无法打开摄像头")
    exit()

# 初始化帧率计算变量
prev_time = 0  # 上一帧时间
curr_time = 0  # 当前帧时间

print("按 q 键退出程序")

# 循环读取视频帧
while True:
    # 读取一帧画面
    ret, frame = cap.read()
    
    # 如果读取失败，退出循环
    if not ret:
        print("警告：无法读取视频帧")
        break
    
    # 计算FPS
    curr_time = time.time()
    fps = 1 / (curr_time - prev_time)  # 帧率 = 1 / 帧间隔时间
    prev_time = curr_time

    # 在画面上显示FPS（整数格式，白色文字，红色背景）
    cv2.putText(
        frame, 
        f"FPS: {int(fps)}",  # 显示内容
        (10, 30),            # 文字位置
        cv2.FONT_HERSHEY_SIMPLEX,  # 字体
        1,                   # 字体大小
        (0, 0, 255),         # 文字颜色 (BGR格式)
        2                    # 文字粗细
    )

    # 显示视频窗口
    cv2.imshow("Camera Stream (Press q to quit)", frame)

    # 按 q 键退出程序
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()
