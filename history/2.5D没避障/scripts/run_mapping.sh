#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/env.sh"
# HAP 的二维切片在 livox_frame 中使用已验收的 -0.33~0.37 m 高度范围。
# 命令行参数放在后面，HAP 重装或重新标定后仍可临时覆盖。
exec ros2 launch go2_navigation go2_mapping.launch.py \
  use_rviz:=true cloud_min_height:=-0.33 cloud_max_height:=0.37 "$@"
