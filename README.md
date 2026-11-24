# Go2 Humble 一体化交互与导航工程

本仓库保存当前工程的源码、配置模板、测试和构建脚本。模型权重、虚拟环境、CUDA/PyTorch 运行库、编译产物、实机地图和历史测试数据不上传。
重新部署前请先阅读 [模型与本地环境说明](docs/LOCAL_RESOURCES.md)；已保留的第三方源码及其版本见 [第三方源码来源](docs/THIRD_PARTY.md)。

上一级“历史版本”中的源码副本已单独收录，见 [历史版本索引](history/README.md)。

本目录把原 `go2_jiaohu` 交互界面与 `yasuo` 导航工作空间迁移到：

- Ubuntu 22.04（Jammy，amd64 / arm64）
- ROS 2 Humble
- Python 3.10 + PyQt5
- Nav2 + SLAM Toolbox + CycloneDDS

导航已集成到原 Qt 底栏的“导航”页，支持在线建图、载入地图定位、地图显示、
点击选点、初始位姿、目标发送/取消、代价地图清理、地图保存与急停。

导航页、2.5D 导航、二维建图和已有地图定位默认使用 **Go2 本体雷达**。
实时 2.5D 地图和避障使用 `/utlidar/cloud` 与 `/utlidar/cloud_base`，
并用 `/utlidar/robot_odom` 对点云去畸变和提供导航位姿。
二维入口使用同一套点云去畸变和地面投影，由 SLAM Toolbox 或 AMCL 提供地图定位。
需要外接 HAP 时，构建使用 `bash scripts/build.sh --hap`，启动加 `lidar_source:=hap`。

Nav2 默认使用稳定转向控制器与
DWB，局部代价地图分辨率 4 cm、更新 20 Hz。LIO 全局代价地图分辨率 4 cm、更新 10 Hz。
LIO 全局路径重规划设为 4 Hz。进入目标点 0.10 m 范围即可完成，最终朝向不限。
局部轨迹选择包含步态与近障碍速度约束，并根据双侧墙体和机身姿态处理窄道，
保留现有保护足迹及膨胀半径。传统二维建图、已有地图定位入口仍保留，
导航页可选室内与室外参数。

二维导航的底盘 TF 使用 Go2 腿部/IMU 状态估计。雷达逐点去畸变后生成水平扫描，
只把有实测回波支持的区域交给建图及避障。

命令行可直接选择地形配置：

```bash
ros2 launch go2_navigation go2_mapping.launch.py terrain_mode:=outdoor
ros2 launch go2_navigation go2_mapping.launch.py terrain_mode:=indoor
```

HAP 模式会自动启动驱动。若已在其他终端运行驱动，可加 `start_hap_driver:=false`。

使用教程与本地依赖见 [Go2_Humble_使用教程.md](Go2_Humble_使用教程.md) 和 [本地资源说明](docs/LOCAL_RESOURCES.md)。常用入口如下。

```bash
bash scripts/install_dependencies.sh
cp go2_jiaohu/.env.example go2_jiaohu/.env
bash scripts/build.sh
# 本体雷达 + 在线可通行地图 + Nav2
bash scripts/run_lio_navigation.sh
# 预览本体雷达地图，关闭底盘命令桥和自主导航
bash scripts/run_lio_navigation.sh enable_go2_base:=false enable_nav2:=false
# 二维建图与已有二维地图定位
bash scripts/run_mapping.sh
bash scripts/run_navigation.sh /绝对路径/地图.yaml
# 切回 HAP + FAST-LIO2，新开一个建图会话
bash scripts/run_lio_navigation.sh lidar_source:=hap
# 邦邦 UI 的“导航”页使用本体雷达启动 2.5D 导航，或接入已运行的导航
bash scripts/run_ui.sh
```

本机原始压缩包与历史工程副本未上传。请从 `.env.example` 创建不受 Git 跟踪的
`go2_jiaohu/.env`，配置本机 Ollama、Qwen ASR、Go2 网卡和报警接入。默认语音识别复用
上级 `yuyin/.venv-qwen` 和 `Qwen3-ASR-0.6B`，无需改动 ROS/UI 环境；
`GO2_ASR_BACKEND=whisper` 可切回旧识别器，详见 [交互说明](go2_jiaohu/README.md)。语音
对话与控制均在本地运行，不再需要云端 API 密钥。

语音/视觉动作默认通过 `unitree_sdk2_python` 的 `SportClient` 下发，并检查服务与 API
返回码；`GO2_MOTION_BACKEND=ros` 可切回 `go2_base` 兼容入口。SDK 直连模式下请勿同时
运行 Nav2 或其他底盘控制程序。

导航实现说明见 [LIO 导航包](ros2_ws/src/go2_lio_navigation/README.md)。
修改包可用 `bash scripts/build_navigation_fluency.sh` 构建，离线专项检查使用
`bash scripts/check_navigation_fluency.sh`，完整检查使用
`bash scripts/check_navigation_regressions.sh`。这些检查使用隔离 DDS 域和本机通信。
