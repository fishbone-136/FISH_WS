# Go2 Humble 一体化交互与导航工程

本目录把原 `go2_jiaohu` 交互界面与 `yasuo` 导航工作空间迁移到：

- Ubuntu 22.04（Jammy，amd64 / arm64）
- ROS 2 Humble
- Python 3.10 + PyQt5
- Nav2 + SLAM Toolbox + CycloneDDS

导航已集成到原 Qt 底栏的“导航”页，支持在线建图、载入地图定位、地图显示、
点击选点、初始位姿、目标发送/取消、代价地图清理、地图保存与急停。

导航激光雷达现已切换为外接 **Livox HAP**：启动建图或定位时会自动启动
`livox_ros_driver2`，使用 `/livox/lidar` 生成 `/scan`，并发布已标定的
`base_link -> livox_frame` 静态 TF。`/odom` 使用 `/lf/sportmodestate`，默认导航链
不再订阅 Go2 自带 UTLidar 的任何 `/utlidar/*` 话题。

当前地图分辨率为 4 cm；HAP 在 `livox_frame` 中保留 -0.33–0.37 m 高度切片，
经静态外参转换到 `base_link` 后生成二维扫描。SLAM 跟随有效扫描输入匹配、
以 4 Hz 发布完整地图。动态障碍由 10 Hz 局部代价地图即时处理，全局代价地图以
5 Hz 更新并以 2 Hz 重规划。Nav2 使用计算量更低的 RPP 局部规划器，进入目标点
0.15 m 范围即完成，不限制最终朝向。导航页可选“室内”和“室外（草地 / 石子路）”
两套参数，室外模式会降低速度并扩大局部避障视野。

2D 导航 TF 使用 Go2 腿部/IMU 状态估计，并滤除四足步态产生的 roll/pitch/z 起伏；
HAP 点云默认只使用最新一帧、历史限制为 0.12 秒，异常位姿跳变和乱序扫描会被拒绝，
防止动态障碍拖影进入 SLAM。

命令行可直接选择地形配置：

```bash
ros2 launch go2_navigation go2_mapping.launch.py terrain_mode:=outdoor
ros2 launch go2_navigation go2_mapping.launch.py terrain_mode:=indoor
```

HAP 驱动已经由上述入口自动启动。若已在其他终端单独运行驱动，可加
`start_hap_driver:=false`，避免重复占用雷达端口。

请从 [Go2_Humble_使用教程.md](Go2_Humble_使用教程.md) 开始；本轮参数变化和实机频率目标见
[性能优化说明.md](цАзшГ╜ф╝ШхМЦшп┤цШО.md)。常用入口：

```bash
bash scripts/install_dependencies.sh
cp go2_jiaohu/.env.example go2_jiaohu/.env
bash scripts/build.sh
# HAP + FAST-LIO2 + 在线可通行地图 + Nav2（必须通过该入口加载 Go2 DDS 网卡）
bash scripts/run_lio_navigation.sh
# 语音/视觉动作默认使用隔离的 unitree_sdk2_python 进程直连 Go2
bash scripts/run_ui.sh
```

原始压缩包未被覆盖。本工程不包含真实 API 密钥；请在
`go2_jiaohu/.env` 中配置本机 Ollama/Qwen、Whisper、Go2 网卡和报警接入。语音
对话与控制均在本地运行，不再需要云端 API 密钥。

语音/视觉动作默认通过 `unitree_sdk2_python` 的 `SportClient` 下发，并检查服务与 API
返回码；`GO2_MOTION_BACKEND=ros` 可切回 `go2_base` 兼容入口。SDK 直连模式下请勿同时
运行 Nav2 或其他底盘控制程序。
