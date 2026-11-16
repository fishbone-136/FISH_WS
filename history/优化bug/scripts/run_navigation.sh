#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "用法: $0 /绝对路径/地图.yaml [其他 launch 参数]" >&2
  exit 2
fi
map_yaml="$(realpath -- "$1")"
shift
if [[ ! -f "${map_yaml}" ]]; then
  echo "地图不存在: ${map_yaml}" >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/env.sh"
exec ros2 launch go2_navigation go2_localization.launch.py \
  map:="${map_yaml}" use_rviz:=true "$@"

