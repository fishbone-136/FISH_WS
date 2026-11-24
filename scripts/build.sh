#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
source "${script_dir}/env.sh"

sensor_packages=(--packages-skip fast_lio livox_ros_driver2)
case "${1:-}" in
  '') ;;
  --hap)
    bash "${script_dir}/build_livox_sdk.sh"
    sensor_packages=(--allow-overriding livox_ros_driver2)
    ;;
  *) echo "用法: $0 [--hap]" >&2; exit 2 ;;
esac

cd "${project_root}/ros2_ws"
colcon build --symlink-install \
  "${sensor_packages[@]}" \
  --cmake-clean-cache \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "构建完成：${project_root}/ros2_ws/install"
