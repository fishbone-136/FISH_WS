import time
import math
import os
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

class Go2SportController:
    def __init__(self, interface: str | None = None):
        """
        初始化宇树 Go2 运动控制客户端
        :param interface: 绑定连接机器狗的网卡名称 (例如 eth0, wlan0)
        """
        self.interface = (
            interface or os.environ.get("GO2_NETWORK_INTERFACE", "eth0")
        ).strip() or "eth0"
        self.client = None
        self.is_ready = False

    def init_robot(self):
        try:
            # 1. 初始化 DDS 通道（默认 Domain ID 为 0）
            ChannelFactoryInitialize(0, self.interface)
            # 2. 创建高级运动控制客户端
            self.client = SportClient()
            
            # 【修复点 1】将心跳超时从 5.0s 严格降低至 0.5s。
            # 一旦发生断联或异常，底层主板会在 0.5 秒内由于无心跳自动驻车，防止“无限冲锋”撞墙。
            self.client.SetTimeout(0.5)
            
            self.client.Init()
            self.is_ready = True
            print("[底盘驱动] 宇树 Go2 SDK2 运动控制总线初始化成功！")
        except Exception as e:
            print(f"[底盘驱动] 初始化失败，请检查网卡名称或网络连接: {e}")
            self.is_ready = False

    def execute_commands(self, commands: list):
        """
        解析并物理执行大模型下发的动作序列
        """
        if not self.is_ready or not self.client:
            print("[底盘驱动] 驱动未就绪，拒绝执行物理动作！")
            return False, "Driver not initialized"

        print("[底盘驱动] 准备执行物理轨迹，切入站立姿态...")
        # 使用新版 SDK2 的 RecoveryStand API
        self.client.RecoveryStand() 
        time.sleep(1.0) # 等待姿态稳定

        try:
            for idx, cmd in enumerate(commands):
                action = cmd.get("action", "stop")
                value = cmd.get("value", 0)
                print(f"[底盘驱动] 正在物理执行步骤 {idx+1}: {action} | 参数: {value}")

                if action == "move_forward":
                    # 设定标准前行线速度 0.4 m/s，计算所需时间
                    vx = 0.4
                    duration = value / vx
                    # 【修复点 2】所有速度、角速度参数必须严格传入 0.0 浮点类型，杜绝 C++ 绑定库报错
                    self._send_velocity_loop(vx, 0.0, 0.0, duration)

                elif action == "move_backward":
                    # 设定标准后退线速度 -0.4 m/s
                    vx = -0.4
                    duration = value / abs(vx)
                    self._send_velocity_loop(vx, 0.0, 0.0, duration)

                elif action == "turn_left":
                    # 将大模型的角度转换为弧度，设定转向角速度 0.5 rad/s
                    omega = 0.5
                    rad = value * (math.pi / 180.0)
                    duration = rad / omega
                    self._send_velocity_loop(0.0, 0.0, omega, duration)

                elif action == "turn_right":
                    # 右转角速度为负
                    omega = -0.5
                    rad = value * (math.pi / 180.0)
                    duration = rad / abs(omega)
                    self._send_velocity_loop(0.0, 0.0, omega, duration)

                elif action == "wait":
                    time.sleep(value)

                elif action == "stop":
                    # 使用 StopMove 并下发零速度指令双重保险
                    self.client.StopMove()
                    # 【修复点 3】对抗 UDP 丢包，连续轰炸 5 帧零速度
                    for _ in range(5):
                        self.client.Move(0.0, 0.0, 0.0)
                        time.sleep(0.05)

            # 全部位移完成后，进入阻尼/停止状态，防止惯性漂移
            self.client.StopMove()
            # 【修复点 4】正常序列结束后的多帧刹车确保
            for _ in range(5):
                self.client.Move(0.0, 0.0, 0.0)
                time.sleep(0.05)
            return True, "All commands executed successfully"

        except Exception as e:
            # 异常时也要确保安全停下
            self.client.StopMove()
            # 【修复点 5】突发异常（如大模型数据格式错乱）触发的兜底多帧刹车
            for _ in range(5):
                self.client.Move(0.0, 0.0, 0.0)
                time.sleep(0.05)
            return False, f"Execution interrupted: {str(e)}"

    def _send_velocity_loop(self, vx: float, vy: float, omega: float, duration: float):
        """
        以 20Hz 频率循环发送速度指令，维持底盘心跳
        """
        rate = 20  # Hz
        dt = 1.0 / rate
        steps = int(duration * rate)
        
        # 【修复点 6】强制进行显式 float 转换，防止 Python 隐式类型将其识别为 int
        vx, vy, omega = float(vx), float(vy), float(omega)
        
        for _ in range(steps):
            # Move 参数分别为：线速度x(正前负后), 线速度y(正左负右), 角速度(正左负右)
            self.client.Move(vx, vy, omega)
            time.sleep(dt)
            
        # 【修复点 7】单次动作交替时，原先只发了一帧 (0,0,0) 极易因局域网抖动丢包。
        # 这里改为连续发送 5 帧零速度脉冲（持续约 0.25 秒），确保底盘切断动力，消除惯性漂移
        for _ in range(5):
            self.client.Move(0.0, 0.0, 0.0)
            time.sleep(0.05)
