# Go2 ROS2 Toolbox（中文说明）


> 本文件为[英文原版 README](./README.md)的中文翻译，若有疑问请参考原文。

[![ROS2](https://img.shields.io/badge/ROS2-Humble-green.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Ubuntu%2020.04-orange.svg)](https://ubuntu.com/)

本项目是为 Unitree Go2 EDU 机器人开发的 ROS2 工具箱，提供 SLAM 与导航能力，实现自主运行。

如果觉得本项目有用，请点个 Star ⭐️ 支持一下！

<div align="center">
  <img src="asset/demo.gif" alt="Go2 ROS2 Toolbox Demo" width="100%" style="background-color: #1a1a1a;">
</div>

## 🚀 功能特性

- **激光雷达集成**：实时点云处理与累积
- **相机支持**：基于 GStreamer 的相机采集与推流
- **SLAM 能力**：集成 SLAM Toolbox 进行建图
- **导航系统**：完整集成 Navigation2 实现自主导航
- **原生 ROS2**：已迁移至 ROS2 Humble 生态

## 📋 前置条件

> **⚠️ 注意：本仓库功能仅在 Go2 EDU 扩展坞自带计算机上测试通过，其他环境（如 PC 直连扩展坞）未验证。**

开发与测试环境：

- **操作系统**：Ubuntu 22.04
- **ROS2**：Humble
- **固件**：v1.1.7（已测试）

## 🛠️ 安装步骤

### 1. 安装官方 Unitree ROS2 包

请先安装官方 Unitree ROS2 包：

```bash
# 参考官方安装指南
# https://github.com/unitreerobotics/unitree_ros2
```

### 2. 安装依赖

#### ROS2 依赖包

```bash
sudo apt-get install ros-humble-navigation2 \
                     ros-humble-nav2-bringup \
                     ros-humble-pcl-ros \
                     ros-humble-tf-transformations \
                     ros-humble-slam-toolbox
```

#### Python 依赖包

```bash
pip3 install transforms3d
```

### 3. 编译工作空间

```bash
# 创建工作空间
mkdir -p go2_ros2_ws/src
cd go2_ros2_ws/src

# 克隆本仓库
git clone https://github.com/andy-zhuo-02/go2_ros2_toolbox.git

# 编译
cd ..
colcon build
```

## 🎯 使用方法

### 快速启动

```bash
# Source 环境
source install/setup.bash

# 启动机器人
ros2 launch go2_core go2_startup.launch.py
```

### SLAM 操作

- **地图保存**：将生成的地图序列化保存
- **地图加载**：加载之前保存的地图

### 导航

1. 打开 RViz2
2. 选择"Navigation2 Goal"按钮
3. 在地图上点击设置导航目标
4. 拖动调整目标朝向

## 🔧 开发说明

### 坐标系参考

| 坐标系         | 说明           | 来源                   |
| -------------- | -------------- | ---------------------- |
| `/odom`      | 里程计坐标系   | Go2 SportModeState 状态估计 |
| `/map`       | 地图坐标系     | SLAM Toolbox           |
| `/base_link` | 机器人基座坐标 | Unitree Go2 里程计话题 |

### ROS 话题

#### 发布者

| 组件             | 话题名                      | 类型        | 坐标系    |
| ---------------- | --------------------------- | ----------- | --------- |
| 机器人状态       | `/lf/sportmodestate` | SportModeState | `/odom` |
| 激光雷达（HAP）  | `/livox/lidar`       | PointCloud2 | `livox_frame` |
| 激光雷达（过滤） | `/trans_cloud`       | PointCloud2 | `livox_frame` |
| 相机图像         | `/camera/image_raw`       | Image       | -         |

#### 订阅者

| 组件         | 话题名      | 类型  | 坐标系        |
| ------------ | ----------- | ----- | ------------- |
| 速度控制命令 | `/cmd_vel`  | Twist | `/base_link` |

## 🤝 贡献

欢迎贡献代码！请随时提交 issue 或 pull request。

## 📄 许可证

本项目采用 MIT 许可证，详见 [LICENSE](LICENSE) 文件。

## 🙏 鸣谢

- Unitree Robotics 提供 Go2 EDU 平台
- ROS2 社区的导航与 SLAM 工具
- 本工具箱的贡献者与用户

## 📞 支持

如遇问题或有疑问，请：

1. 查看 [Issues](https://github.com/andy-zhuo-02/go2_ros2_toolbox/issues) 页面
2. 创建新 issue 并详细描述问题
3. 附上系统信息与错误日志

---

**注意**：本工具箱为非官方项目，与 Unitree Robotics 无直接关联。
