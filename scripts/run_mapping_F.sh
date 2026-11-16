#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/env.sh"

# F 版是独立的实验建图链：Nav2/避障继续使用完整 /scan，只有 SLAM 使用
# 经过高速旋转与异常断帧门控的 /scan_slam_f。旧版 run_mapping.sh 不受影响。
exec ros2 launch go2_mapping_f go2_mapping_f.launch.py \
  use_rviz:=true cloud_min_height:=-0.33 cloud_max_height:=0.37 "$@"
