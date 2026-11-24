#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/env.sh"
exec ros2 launch go2_navigation go2_mapping.launch.py \
  use_rviz:=true "$@"
