# Unitree Go2：Ubuntu 22.04 + ROS 2 Humble 一体化使用教程

## 1. 工程说明

目录结构：

```text
Go2_Humble_Integrated/
├── go2_jiaohu/              # 原交互功能 + 已集成的导航 Qt 页面
├── ros2_ws/src/             # Go2、感知、SLAM、Nav2、Unitree 消息源码
├── maps/                    # 建议存放地图
├── scripts/                 # 安装、构建、启动、检查脚本
├── README.md
└── 使用教程.md
```

主要迁移内容：

- Nav2 参数切换到 Humble 的 AMCL 运动模型、`behavior_server`、
  `nav2_behaviors`、复数 `goal_checker_plugins`、新障碍物距离参数和速度平滑器。
- 删除 Nav2 行为树的硬编码用户路径，默认使用 Humble 自带行为树。
- 增加“在线建图 + Nav2”和“地图定位 + Nav2”两个完整启动入口。
- 导航点云切换为 Livox HAP `/livox/lidar`，建图/定位入口自动启动驱动并发布
  `base_link -> livox_frame` 外参；不再订阅 Go2 自带 UTLidar。
- 修复点云转激光节点反复重新订阅、TF 缓存参数未生效和数组越界风险。
- Go2 底盘桥增加 0.5 秒 `cmd_vel` 看门狗，导航或 UI 异常时自动 StopMove。
- Qt 导航页使用标准 `/map`、`/amcl_pose`、`/plan`、
  `NavigateToPose` 和 `SaveMap` 接口；ROS 回调在独立线程运行。
- 从首页切换到导航页时，交互运动会进入安全待机；离开导航页会取消目标并发零速度。
- 语音链路改为本地 faster-whisper + Ollama/Qwen，不再使用云端 API；机器狗名称为“邦邦”。
- 语音移动增加中英文固定口令、动作白名单与限幅；D435i 增加地面引导线、
  红绿灯和近障停车闭环，并提供本地/ROS 2 一键报警事件。
- MediaPipe、RealSense 在不支持其 pip 轮子的 ARM64 板卡上可缺省；主 UI 和导航仍可运行。
- 重写点云累计节点：HAP 默认只处理最新一帧，并在每个新 LiDAR 帧到达时发布一次，
  避免运动中的旧扫描重影和 CPU/TF 队列堆积。
- SLAM 地图分辨率调整为 0.04 m，地图发布约 4 Hz，并采用 Huber 鲁棒损失；
  Nav2 改用计算量更低的 Regulated Pure Pursuit 跟踪器。
- 里程计改用 Go2 `/lf/sportmodestate` 腿部/IMU 状态，2D TF 默认去除机身
  roll/pitch/z 起伏；不再依赖 UTLidar 位姿，并拒绝短时里程计跳变与乱序扫描。
- Qt 对高频位姿限制为 20 Hz，只转换最新 OccupancyGrid，并缓存地图缩放结果，
  避免位姿刷新反复处理整张地图。

## 2. 重要安全事项

1. 第一次运行时把机器狗架空或放在宽阔无人的区域，并随时准备使用遥控器急停。
2. 只有确认 `/scan`、`odom -> base_link`、地图和障碍层都正确后，才发送导航目标。
3. 自动导航时不要同时启用语音、手势或 VLM 的运动命令。Qt 已做模式切换保护，
   但不要在另一个终端同时运行独立交互脚本。
4. 语音移动只能使用“执行指令/执行……动作”或对应英文口令授权。停止和报警可随时
   直接说，但 D435i 识别仍是实验功能，不能代替遥控器急停或人工交通判断。

## 3. 新板子和网络准备

### 3.1 系统要求

- Ubuntu 22.04 64 位；ROS 2 Humble 官方支持 amd64 和 arm64。
- 建议至少 8 GB 内存；同时运行 Qt、Nav2、SLAM 和视觉模型时建议更高。
- 板子和 Go2 使用有线网口连接；关闭该接口的自动休眠。

先确认系统：

```bash
lsb_release -a
uname -m
python3 --version
```

应看到 Ubuntu 22.04 和 Python 3.10。

### 3.2 配置机器狗有线网卡

查网卡名：

```bash
ip -br link
nmcli connection show
```

假设机器狗连接对应的 NetworkManager 连接名是 `Wired connection 1`，网卡名是
`enp3s0`。按 Unitree 官方网络约定，把板子地址设为 `192.168.123.99/24`：

```bash
sudo nmcli connection modify "Wired connection 1" \
  ipv4.method manual ipv4.addresses 192.168.123.99/24 \
  ipv4.gateway "" ipv4.dns ""
sudo nmcli connection up "Wired connection 1"
ip -4 addr show enp3s0
```

连接名和网卡名必须替换为本机实际值。不要在 Wi-Fi 上误执行上述命令。

### 3.3 配置 HAP 有线网卡

HAP 使用独立网段。当前实机配置为 HAP `192.168.1.100`，板端接口
`enx00e04c3c2370` 使用 `192.168.1.102/24`。如接口尚未配置，可按实际连接名设置：

```bash
sudo nmcli connection modify "HAP" \
  ipv4.method manual ipv4.addresses 192.168.1.102/24 \
  ipv4.gateway "" ipv4.dns ""
sudo nmcli connection up "HAP"
ip -4 addr show enx00e04c3c2370
ping -c 3 192.168.1.100
```

机器狗网卡和 HAP 网卡用途不同：`GO2_NETWORK_INTERFACE` 用于 Unitree DDS，
`HAP_NETWORK_INTERFACE` 只用于检查雷达链路，不要把两者设成同一接口。

## 4. 安装依赖

进入工程根目录：

```bash
cd ~/Desktop/Go2_Humble_Integrated
bash scripts/install_dependencies.sh
```

脚本会完成：

- 检查 Ubuntu 22.04；
- 在缺少 ROS 时添加 ROS 2 软件源并安装 Humble Desktop；
- 安装 Nav2、SLAM Toolbox、CycloneDDS、Qt5、OpenCV、GStreamer 和构建工具；
- 用 `rosdep` 安装 ROS 包依赖；
- 创建带 `--system-site-packages` 的 `.venv`，保证 apt 安装的 rclpy/cv_bridge 可见；
- 安装交互模块的通用 Python 依赖。

不要用普通 venv 或 Conda 启动此工程，也不要在系统 Python 中执行
`pip install -U setuptools`；这会破坏 Humble 的 ament/ROS Python 环境。

## 5. 配置工程

复制环境模板：

```bash
cp go2_jiaohu/.env.example go2_jiaohu/.env
nano go2_jiaohu/.env
```

示例：

```dotenv
GO2_NETWORK_INTERFACE=enp3s0
ROS_DOMAIN_ID=0
LIVOX_WS=/home/fishbone/GO2/livox_ws
HAP_NETWORK_INTERFACE=enx00e04c3c2370
HAP_IP=192.168.1.100
GO2_ROBOT_NAME=邦邦
OLLAMA_HOST=http://127.0.0.1:11434
OLLAMA_MODEL=qwen2.5:3b
OLLAMA_WARMUP=1
WHISPER_MODEL=voice/models/faster-whisper-small
WHISPER_RESCUE_MODEL=voice/models/faster-whisper-large-v3-turbo
WHISPER_PRELOAD_RESCUE=0
GO2_OBSTACLE_STOP_DISTANCE_M=0.60
```

- `GO2_NETWORK_INTERFACE`：第 3 步查到的机器狗网卡名。
- `ROS_DOMAIN_ID`：机器狗默认通常为 0；同一 ROS 网络必须一致。
- `LIVOX_WS`：已经编译好 `livox_ros_driver2` 的工作区；`scripts/env.sh` 会先加载它。
- `HAP_NETWORK_INTERFACE`、`HAP_IP`：HAP 所在接口和雷达地址，用于系统检查。
- `OLLAMA_MODEL`：本机已安装的 Qwen 文本模型，用于中英文对话和复杂动作语义回退；
  常用固定动作由本地确定性语法直接解析。
- `WHISPER_MODEL`：低延迟主 ASR；清晰语音只运行 `small` 一次，边界结果才使用
  `WHISPER_RESCUE_MODEL` 配置的 `large-v3-turbo` 复核。

启动 UI 前确认本地模型：

```bash
ollama serve
ollama pull qwen2.5:3b
ollama list
```

每个脚本都会加载 `.env`，设置 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，并生成绑定
指定网卡的 `CYCLONEDDS_URI`。

## 6. 构建

```bash
cd ~/Desktop/Go2_Humble_Integrated
bash scripts/build.sh
```

构建结束后检查：

```bash
bash scripts/check_system.sh
```

如果修改过 C++、消息定义、CMake 或 ROS 参数，需要重新运行 `build.sh`。
只修改 Qt Python 文件时使用 `--symlink-install`，通常无需重构建。

## 7. 验证机器狗 ROS 数据

新开终端：

```bash
cd ~/Desktop/Go2_Humble_Integrated
source scripts/env.sh
ros2 topic list
```

先确认 Go2 腿部/IMU 里程计源存在：

```bash
ros2 topic hz /lf/sportmodestate
```

导航和建图入口会自动启动 HAP 驱动。也可以先单独验证雷达链路：

```bash
ros2 launch go2_perception livox_hap.launch.py
```

另开一个已执行 `source scripts/env.sh` 的终端，确认 HAP 点云、二维扫描和外参：

```bash
ros2 topic hz /livox/lidar
ros2 topic echo /livox/lidar --field header.frame_id --once
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base_link livox_frame
```

点云帧应为 `livox_frame`。默认安装外参是 HAP 相对 `base_link` 向前 0.14 m、
向上 0.13 m；若实际安装位置或朝向不同，必须在启动参数中修改
`hap_x/y/z/roll/pitch/yaw` 后再建图。

如果看不到话题：

```bash
echo "$GO2_NETWORK_INTERFACE"
echo "$RMW_IMPLEMENTATION"
ip link show "$GO2_NETWORK_INTERFACE"
ros2 daemon stop
ros2 daemon start
```

仍不通时检查静态 IP、网线、防火墙、Go2 固件，以及 `.env` 中的网卡名。

## 8. 启动一体化 Qt 页面

```bash
cd ~/Desktop/Go2_Humble_Integrated
bash scripts/run_ui.sh
```

界面底部：

- “首页”：原语音、手势、眼动、视觉交互功能。
- “导航”：新集成的 ROS 2 Humble 导航控制台。
- “协同 / 任务”：保留原占位页面。

导航页的 ROS 2 状态变绿并显示“Nav2、地图、位姿均正常”后，才可发送目标。

### 8.1 模式一：在线建图

1. 进入“导航”页。
2. 运行模式选择“在线建图 + Nav2”。
3. 点击“启动”。页面会启动底盘桥、点云处理、SLAM Toolbox、Nav2 和地图保存服务。
4. 用遥控器缓慢巡视环境，确认地图逐步出现。
5. 在“建图保存”中填写不带扩展名的目标，例如：

   ```text
   /home/你的用户名/maps/lab_map
   ```

6. 点击“保存 PGM + YAML”，应得到 `lab_map.pgm` 和 `lab_map.yaml`。

建图时也可以在已经生成的地图区域发送 Nav2 目标，但首次调试建议用遥控器完成地图。

### 8.2 模式二：载入地图定位导航

1. 运行模式选择“载入地图定位 + Nav2”。
2. 点击“选择地图”，选择保存的 `.yaml`。
3. 点击“启动”，等待地图出现。
4. 把 X/Y/Yaw 填成机器狗在地图中的实际初始位置，点击“设为初始位姿”。
5. 等待状态变绿。
6. 单击地图选目标；右侧 X/Y 会自动更新。点击“开始导航”即可，终点朝向不作限制。
   “初始 Yaw”只在设置初始位姿时使用。
7. “取消目标”会取消当前 Action；“急停”还会连续发布零速度。

### 8.3 模式三：连接已有节点

如果已经在别的终端启动了 ROS 栈，选择“连接已有 ROS 2 节点”。此模式不创建、
不终止外部节点，只连接标准话题、Action 和服务。

## 9. 命令行启动（不使用 Qt 的进程管理）

在线建图：

```bash
cd ~/Desktop/Go2_Humble_Integrated
bash scripts/run_mapping.sh
```

已有地图定位：

```bash
bash scripts/run_navigation.sh /home/你的用户名/maps/lab_map.yaml
```

随后可另开终端运行 UI，导航页选择“连接已有 ROS 2 节点”：

```bash
bash scripts/run_ui.sh
```

## 10. 交互硬件依赖

安装脚本会在 amd64 / arm64 上安装与本工程兼容的 RealSense、MediaPipe 和
Unitree SDK2 Python 依赖。若基础依赖已装好，也可单独执行：

```bash
bash scripts/install_interaction_dependencies.sh
bash scripts/check_system.sh
```

Jetson 使用 `pyrealsense2 2.58.3` 和仍提供 `mp.solutions.hands` 的 MediaPipe
`0.10.18`。脚本刻意保留 Ubuntu 自带、启用 GStreamer 的 OpenCV，避免 pip 的
OpenCV wheel 覆盖后导致 Go2 前置相机组播失效。

UI 的语音、手势和视觉动作统一发送给 ROS 2 `go2_base`，由它与 Nav2 仲裁并作为
唯一的 Unitree 运动请求发布者；UI 内不再另建直连 SDK 控制源。实机交互前必须先
启动或连接包含 `go2_base` 的建图/导航栈。RealSense 可用时首页显示其彩色流和深度流；
RealSense 无帧时，主画面会自动回退到 Go2 的 `230.1.1.1:1720` 前置相机组播。

`.env` 的 `GO2_UI_CAMERA_SOURCE` 可设为 `auto`（默认）、`go2` 或 `local`。
语音 ASR 与对话均在本地运行，不需要 DeepSeek、Moonshot 或 Google 服务。首次使用
`large-v3-turbo` 时 faster-whisper 会下载模型，之后可离线运行。若设置
`WHISPER_LOCAL_FILES_ONLY=1`，系统只使用已缓存模型。

语音授权示例：`执行指令，向前一米`、`执行左转动作`、
`execute command, move forward one meter`。裸说“前进/go forward”不会移动。
说`执行指令，沿地面引导线前进并识别红绿灯`会复用 UI 已打开的 D435i 彩色与原始
深度帧；红灯、近障、丢线、无帧和超时都会停车。说`停止/stop`可无条件制动。

说`一键报警/紧急报警/call for help`会先停车、写入
`log/bangbang_emergency_alerts.jsonl`，并发布 `/bangbang/emergency_alert`。现场声光、
短信或值班平台程序可通过 `.env` 的 `BANGBANG_ALARM_COMMAND` 接入；未接入时 UI
会明确提示只完成了本地记录。

## 11. 导航参数与实机标定

主要文件：

```text
ros2_ws/src/go2_navigation/config/nav2_params.yaml
ros2_ws/src/go2_slam/config/mapper_params_online_async.yaml
ros2_ws/src/go2_perception/launch/go2_pointcloud_process.launch.py
```

建议按以下顺序标定：

1. 检查 TF：

   ```bash
   source scripts/env.sh
   ros2 run tf2_ros tf2_echo odom base_link
   ros2 run tf2_tools view_frames
   ```

2. 检查激光：

   ```bash
   ros2 topic hz /scan
   ros2 topic echo /scan --once
   ```

3. 本工程默认使用适合传感器低延迟传输的 Best Effort QoS；它可同时匹配
   Best Effort 和 Reliable 发布者。如果现场网络必须使用 Reliable，可用：

   ```bash
   ros2 launch go2_navigation go2_mapping.launch.py use_best_effort:=false
   ```

4. 在 `nav2_params.yaml` 里从低到高调整 `max_vel_x`、`max_speed_xy` 和
   `velocity_smoother.max_velocity`。三者要保持一致或由平滑器设置更低上限。
5. 根据机身外接设备尺寸调整 `robot_radius`；宁可略大，不要小于真实外廓。
6. 根据场地调整 `inflation_radius`，走廊环境需同时验证全局和局部代价地图。

修改参数后重新 `bash scripts/build.sh`，再重启相关节点。

### 11.1 性能与刷新频率验收

启动建图后另开终端运行：

```bash
cd /home/fishbone/HAP_ws
bash scripts/benchmark_topics.sh 10
```

默认优化配置的健康表现：

| 指标 | 期望表现 |
|---|---|
| `/livox/lidar` | HAP 原始 PointCloud2，当前实机通常约 7 Hz，帧名为 `livox_frame` |
| `/trans_cloud` | 接近 `/livox/lidar` 的真实频率，不再固定重复发布 50 Hz |
| `/scan` | 完整建图栈下当前实测约 4–7 Hz；不能持续出现超过 2 秒的断流 |
| `/map` | 约 4 Hz；大地图也应持续更新而不是越建越慢 |
| 局部代价地图 | 约 10 Hz 消费最新 `/scan`，动态障碍出现/离开后应快速更新 |
| Qt 地图 | 只处理最新地图；机器人图标平滑、地图更新时界面无明显卡顿 |

还要确认 HAP 原始点云位于雷达坐标系：

```bash
ros2 topic echo /livox/lidar --field header.frame_id --once
```

默认应输出 `livox_frame`。点云节点的 `fixed_cloud_frame` 也应保持为
`livox_frame`，且 `accumulation_frames=1`，避免移动时把不同位姿的扫描直接拼接。
`target_frame=base_link` 会使用静态外参把点云转换到机身坐标系。

默认参数适合中等及以上板载算力。若实测 `/scan` 低于源点云频率或单核持续满载，
先检查 HAP 网卡、CPU 占用和 `base_link -> livox_frame` TF；不要通过伪造固定坐标系
或重复发布旧扫描来掩盖 TF、驱动或网络故障。

## 12. 常见故障

### Qt 显示“ROS 2 Python 环境不可用”

必须使用：

```bash
bash scripts/run_ui.sh
```

不要直接从未 source Humble 的 IDE/Conda 环境运行 `python main.py`。

### 地图一直为空

依次检查：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /trans_cloud
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /map
ros2 run tf2_ros tf2_echo odom base_link
```

在哪一步断开，就检查对应的 Go2 数据、点云节点、SLAM 或 TF。

### Nav2 等待位姿

- 建图模式应由 SLAM Toolbox 发布 `map -> odom`。
- 定位模式必须选择有效地图，并设置初始位姿。
- `base_frame_id` 和 `robot_base_frame` 已统一为 `base_link`，不要再配置
  不存在的 `base_footprint`。

### 点云没有进入转换节点

查看发布 QoS：

```bash
ros2 topic info -v /livox/lidar
```

默认已经使用 Best Effort。如果 `ros2 topic info -v` 明确显示现场只允许 Reliable，
按前文加 `use_best_effort:=false`。

### UI 能开，但语音/VLM 无法调用

依次运行 `ollama list`、`curl http://127.0.0.1:11434/api/tags`，确认 Ollama 服务和
`.env` 中的 `OLLAMA_MODEL`。若 ASR 初始化失败，检查 `faster_whisper`、模型缓存和
`WHISPER_DEVICE`；没有 CUDA 时可设 `WHISPER_DEVICE=cpu`、
`WHISPER_COMPUTE_TYPE=int8`。完整检查可运行 `bash scripts/check_system.sh`。

### 手势切换后仍持续运动

当前部署版使用短动作租约：相同运动手势仅定期续期，新手势、握拳、手部丢失、
关闭手势模式或进入导航页都会立即抢占并发送 `StopMove`。若仍持续运动，先使用
Unitree 遥控器急停，再确认没有单独运行 `hand/hand.py` 或其他 SDK 控制程序。

### 多次导航后 SLAM Toolbox 出现 `probability search`

回环搜索的 `loop_search_space_dimension / loop_search_space_resolution` 必须为偶整数。
本工程使用 `6.0 / 0.05 = 120`。修改参数时不要使用不能整除的组合，否则回环触发后
SLAM 可能退出，随后会连带出现大量 `map -> odom` TF 超时。

### 编译提示 `rosidl_generator_dds_idl` 缺失

```bash
sudo apt update
sudo apt install ros-humble-rosidl-generator-dds-idl
bash scripts/build.sh
```

### 关闭 UI 后机器狗仍有运动趋势

本工程有三层保护：取消 Nav2 Action、连续零速度、`go2_base` 0.5 秒看门狗。
实机仍应以 Unitree 遥控器急停为最高优先级；若看门狗未触发，检查是否有其他独立程序
正在直接通过 SDK 下发运动。

## 13. 快速命令清单

```bash
# 一次性安装
bash scripts/install_dependencies.sh

# 配置
cp go2_jiaohu/.env.example go2_jiaohu/.env
nano go2_jiaohu/.env

# 构建与检查
bash scripts/build.sh
bash scripts/check_system.sh

# 建图运行后的频率检查
bash scripts/benchmark_topics.sh 10

# 一体化界面
bash scripts/run_ui.sh

# 命令行在线建图
bash scripts/run_mapping.sh

# 命令行地图定位
bash scripts/run_navigation.sh /absolute/path/map.yaml
```
