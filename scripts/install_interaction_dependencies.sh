#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
venv_python="${project_root}/.venv/bin/python"
export PYTHONNOUSERSITE=1

if [[ ! -x "${venv_python}" ]]; then
  echo "未找到 ${venv_python}；请先运行 scripts/install_dependencies.sh。" >&2
  exit 1
fi

# 保留 Ubuntu 自带的 OpenCV（含 GStreamer），否则 Go2 组播视频无法打开。
"${venv_python}" -m pip install \
  'pyrealsense2==2.58.3.10794' \
  'absl-py>=1' 'flatbuffers>=2' 'protobuf>=4.25.3,<5' \
  'sounddevice>=0.4.4' sentencepiece \
  'av>=11' 'huggingface-hub>=0.21' \
  'tokenizers>=0.13,<1' tqdm

# Jetson 部署包中包含按 Orin(sm_87)编译的 CUDA 版 CTranslate2。必须优先装它，
# 否则 PyPI 的 Linux aarch64 wheel 是 CPU 版，再跑本脚本会悄悄覆盖 GPU 能力。
shopt -s nullglob
cuda_ct2_wheels=(
  "${project_root}"/go2_jiaohu/third_party/ctranslate2-cuda/wheels/ctranslate2-*-linux_aarch64.whl
)
shopt -u nullglob
if (( ${#cuda_ct2_wheels[@]} > 0 )); then
  "${venv_python}" -m pip install --force-reinstall --no-deps "${cuda_ct2_wheels[0]}"
else
  "${venv_python}" -m pip install 'ctranslate2>=4,<5'
  echo "[WARN] 未找到本地 CUDA CTranslate2 wheel；已安装 CPU 版。" >&2
fi
"${venv_python}" -m pip install --no-deps 'mediapipe==0.10.18'
# 不安装可选 Silero VAD 的 onnxruntime：其通用 ARM64 wheel 在部分 Jetson 上会
# native assertion，且会强制升级 NumPy。录音层已经按静音切句，Whisper 关闭 VAD。
"${venv_python}" -m pip install --no-deps 'faster-whisper==1.2.1'

# YOLO runs in a separate process using the existing Jetson CUDA torch bundle.
# Keep its newer OpenCV out of the ROS/UI Python path (GStreamer must remain).
"${venv_python}" -m pip install --no-deps \
  --target "${project_root}/go2_jiaohu/third_party/person_detector_deps" \
  'opencv-python-headless==4.10.0.84' 'ultralytics==8.3.162'

third_party_dir="${project_root}/third_party"
cyclonedds_home=""
mkdir -p "${third_party_dir}"

if [[ -f /usr/local/lib/libddsc.so && -d /usr/local/include/dds ]]; then
  cyclonedds_home=/usr/local
else
  cyclone_src="${third_party_dir}/cyclonedds-0.10"
  if [[ ! -d "${cyclone_src}/.git" ]]; then
    git clone --depth 1 --branch releases/0.10.x \
      https://github.com/eclipse-cyclonedds/cyclonedds.git "${cyclone_src}"
  fi
  cmake -S "${cyclone_src}" -B "${cyclone_src}/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${cyclone_src}/install" \
    -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF
  cmake --build "${cyclone_src}/build" --parallel "$(nproc)"
  cmake --install "${cyclone_src}/build"
  cyclonedds_home="${cyclone_src}/install"
fi

CYCLONEDDS_HOME="${cyclonedds_home}" \
  "${venv_python}" -m pip install 'cyclonedds==0.10.2'

unitree_src="${third_party_dir}/unitree_sdk2_python"
if [[ ! -f "${unitree_src}/setup.py" ]]; then
  git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git \
    "${unitree_src}"
fi
"${venv_python}" -m pip install --no-deps -e "${unitree_src}"

"${venv_python}" - <<'PY'
import cv2
import ctranslate2
import faster_whisper
import mediapipe as mp
import pyrealsense2 as rs
import unitree_sdk2py

assert hasattr(mp.solutions, "hands")
assert "GStreamer:                   YES" in cv2.getBuildInformation()
print("交互依赖可用:")
print("  MediaPipe", mp.__version__)
print("  faster-whisper", faster_whisper.__version__)
print("  CTranslate2 CUDA devices", ctranslate2.get_cuda_device_count())
print("  pyrealsense2", getattr(rs, "__version__", "loaded"))
print("  unitree_sdk2py", unitree_sdk2py.__file__)
PY
