#!/usr/bin/env bash
set -u

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/env.sh"

sample_seconds="${1:-8}"
if [[ ! "${sample_seconds}" =~ ^[1-9][0-9]*$ ]]; then
  echo "用法: bash scripts/benchmark_topics.sh [每个话题采样秒数]" >&2
  exit 2
fi

echo "每个话题采样 ${sample_seconds} 秒；Ctrl+C 可提前结束。"
echo "健康目标：/trans_cloud 与 /scan 接近 LiDAR 源频率，/map 约 4 Hz。"

echo
echo "===== 坐标系 ====="
for topic in /livox/lidar /trans_cloud /scan; do
  if ros2 topic list | grep -Fxq "${topic}"; then
    frame_id="$(timeout 3s ros2 topic echo "${topic}" --field header.frame_id --once 2>/dev/null || true)"
    printf '%-30s %s\n' "${topic}" "${frame_id:-[读取超时]}"
  fi
done

for topic in /livox/lidar /trans_cloud /scan /map; do
  echo
  echo "===== ${topic} ====="
  if ! ros2 topic list | grep -Fxq "${topic}"; then
    echo "[缺失] ${topic}"
    continue
  fi
  # timeout 正常结束会返回 124；这里保留 ros2 topic hz 的统计输出即可。
  timeout --signal=INT "${sample_seconds}s" ros2 topic hz "${topic}" --window 50 || true
done

echo
echo "若 /trans_cloud 固定显示 50 Hz，说明仍在运行旧 build/install，请重新 build 并 source。"
echo "若 /scan 明显低于源点云，先检查 CPU、TF 和 DDS QoS，再减少 accumulation_frames。"
echo "若日志出现 discontinuous/non-monotonic，先排查 LiDAR 时钟、网路乱序或里程计重置。"
