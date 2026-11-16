#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"

# 文件描述符会随最后的 exec 传给 Python；UI 退出后锁自动释放。
# 这样重复执行 run_ui.sh 不会再创建第二套摄像头、语音和 ROS 功能进程。
if command -v flock >/dev/null 2>&1; then
  ui_lock_dir="${XDG_RUNTIME_DIR:-/tmp}"
  ui_lock_file="${ui_lock_dir}/bangbang-ui-${UID:-1000}.lock"
  exec 9>"${ui_lock_file}"
  if ! flock -n 9; then
    echo "邦邦 UI 已在运行，已拒绝重复启动。" >&2
    exit 1
  fi
fi

source "${script_dir}/env.sh"

# PulseAudio 在部分 Jetson + USB UAC 组合上会保留一个 RUNNING 但全零的采集流。
# 启动 UI 前只重置配置的输入源，并保持系统输出路由不变。
bash "${script_dir}/prepare_audio.sh"
export ALSA_CONFIG_PATH="${project_root}/go2_jiaohu/config/alsa-pulse.conf"

if [[ ! -f "${project_root}/ros2_ws/install/setup.bash" ]]; then
  echo "ROS 2 工作空间尚未构建，请先运行 scripts/build.sh" >&2
  exit 1
fi

cd "${project_root}/go2_jiaohu"
exec python3 main.py "$@"
