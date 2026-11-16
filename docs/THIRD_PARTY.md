# 第三方源码来源

仓库保留构建需要的第三方源码和已有许可证，不上传其 Git 历史、模型、编译库、安装目录及大体积演示媒体。部分源码含本地修改；下面的提交仅标识原始 checkout 基线，不代表未经修改的上游版本。

| 目录 | 上游 / 基线 | 备注 |
| --- | --- | --- |
| `ros2_ws/src/fast_lio/` | [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)，`2fffc570a25d0df172720bac034fbdb6a13d2162` | 保留本地前端与静止保护修改及测试；许可证见目录内 `LICENSE` |
| `ros2_ws/src/fast_lio/include/ikd-Tree/` | [hku-mars/ikd-Tree](https://github.com/hku-mars/ikd-Tree)，原 FAST-LIO 子模块的源码快照 | 直接收录源码，不需要递归拉取子模块 |
| `ros2_ws/src/livox_ros_driver2/` | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)，`4a1def929e5b59c7a8122d19fce6efba581ce9f7` | 已含本地修改，见 `LOCAL_CHANGES.md`；保留 `LICENSE.txt` |
| `third_party/livox_sdk2_system_spdlog/` | [Livox-SDK/Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)，SDK 0.0.2 的本地副本 | 使用系统 spdlog 和独立 SONAME；见 `LOCAL_CHANGES.md`、`LICENSE.txt` |
| `ros2_ws/src/` 既有 Go2 ROS 包 | 现有 Go2 ROS2 Toolbox 源码及其本地修改 | 保留 `ros2_ws/src/LICENSE` 与原始中英文说明 |

Livox SDK 内的 RapidJSON/FastCRC、ROS 包中的 nlohmann JSON 等随附源码保留原有版权和许可文件。各组件适用其各自许可，本仓库整理未替换其许可证。

`third_party/unitree_sdk2_python/` 不随仓库上传；交互依赖安装脚本从 [unitreerobotics/unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) 拉取。原设备 checkout 为 `65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5`，需要重现旧环境时可在单独安装目录固定到该版本。

FAST-LIO 的上游 PDF、GIF、结果图和 Toolbox 演示 GIF 已省略，因此部分原始第三方 README 的本地媒体链接不会显示；这些资料可在相应上游仓库查看。
