#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
source "${script_dir}/env.sh"

bash "${script_dir}/build_livox_sdk.sh"

cd "${project_root}/ros2_ws"
colcon build --symlink-install \
  --allow-overriding livox_ros_driver2 \
  --cmake-clean-cache \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "构建完成：${project_root}/ros2_ws/install"
