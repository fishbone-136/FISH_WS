# Go2 ROS2 Toolbox

[![ROS2](https://img.shields.io/badge/ROS2-Humble-green.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Ubuntu%2020.04-orange.svg)](https://ubuntu.com/)

[🇨🇳 中文版 README](./README_zh.md)

A comprehensive ROS2 toolbox for Unitree Go2 EDU robot, providing SLAM and navigation capabilities for autonomous operation.

If you find this project helpful, please give it a Star ⭐️ to support us!

<div align="center">
  <img src="asset/demo.gif" alt="Go2 ROS2 Toolbox Demo" width="100%" style="background-color: #1a1a1a;">
</div>

## 🚀 Features

- **LiDAR Integration**: Real-time point cloud processing and accumulation
- **Camera Support**: GStreamer-based camera capture and streaming
- **SLAM Capabilities**: Integration with SLAM Toolbox for mapping
- **Navigation Stack**: Full Navigation2 integration for autonomous navigation
- **ROS2 Native**: Migrated and tested statically for the ROS2 Humble ecosystem

## 📋 Prerequisites

> **⚠️ Note: The features of this repository have only been tested on the onboard expansion dock computer of Go2 EDU. Compatibility and functionality on other environments (like PC wired to Go2 dock computer) have not been verified.**

This toolbox is developed and tested on Unitree Go2 EDU with the expansion dock environment:

- **OS**: Ubuntu 22.04
- **ROS2**: Humble
- **Firmware**: v1.1.7 (tested)

## 🛠️ Installation

### 1. Install Official Unitree ROS2 Package

First, install the official Unitree ROS2 package:

```bash
# Follow the official installation guide
# https://github.com/unitreerobotics/unitree_ros2
```

### 2. Install Dependencies

#### ROS2 Packages

```bash
sudo apt-get install ros-humble-navigation2 \
                     ros-humble-nav2-bringup \
                     ros-humble-pcl-ros \
                     ros-humble-tf-transformations \
                     ros-humble-slam-toolbox
```

### 3. Build the Workspace

```bash
# Create workspace
mkdir -p go2_ros2_ws/src
cd go2_ros2_ws/src

# Clone repository
git clone https://github.com/andy-zhuo-02/go2_ros2_toolbox.git

# Build
cd ..
colcon build
```

## 🎯 Usage

### Quick Start

```bash
# Source the workspace
source install/setup.bash

# Launch the robot
ros2 launch go2_core go2_startup.launch.py
```

### SLAM Operations

- **Map Serialization**: Save generated maps for later use
- **Map Deserialization**: Load previously saved maps

### Navigation

1. Open RViz2
2. Select the 'Navigation2 Goal' button
3. Click on the map to set navigation goals
4. Drag to adjust the target orientation

## 🔧 Development

### Frame Reference

| Frame          | Description     | Source                      |
| -------------- | --------------- | --------------------------- |
| `/odom`      | Odometry frame  | Go2 SportModeState estimator |
| `/map`       | Map frame       | SLAM Toolbox                |
| `/base_link` | Base link frame | Unitree Go2 odometry topic |

### ROS Topics

#### Publishers

| Component           | Topic                       | Type        | Frame     |
| ------------------- | --------------------------- | ----------- | --------- |
| Robot State         | `/lf/sportmodestate` | SportModeState | `/odom` |
| LiDAR (Livox HAP)   | `/livox/lidar`       | PointCloud2 | `livox_frame` |
| LiDAR (Filtered)    | `/trans_cloud`       | PointCloud2 | `livox_frame` |
| Camera Image        | `/camera/image_raw`       | Image       | -         |

#### Subscribers

| Component        | Topic        | Type  | Frame        |
| ---------------- | ------------ | ----- | ------------ |
| Velocity Command | `/cmd_vel`   | Twist | `/base_link` |

## 🤝 Contributing

We welcome contributions! Please feel free to submit issues and pull requests.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- Unitree Robotics for the Go2 EDU platform
- ROS2 community for the excellent navigation and SLAM tools
- Contributors and users of this toolbox

## 📞 Support

If you encounter any issues or have questions, please:

1. Check the [Issues](https://github.com/andy-zhuo-02/go2_ros2_toolbox/issues) page
2. Create a new issue with detailed information
3. Include system information and error logs

---

**Note**: This is an unofficial toolbox and is not affiliated with Unitree Robotics.
