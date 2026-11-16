#!/usr/bin/env bash
set -u

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
failures=0

check() {
  local label="$1"
  shift
  if "$@" >/dev/null 2>&1; then
    printf '[OK]   %s\n' "${label}"
  else
    printf '[FAIL] %s\n' "${label}"
    failures=$((failures + 1))
  fi
}

check "Ubuntu 22.04" bash -c '. /etc/os-release && [[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]]'
check "ROS 2 Humble" test -f /opt/ros/humble/setup.bash
check "项目已构建" test -f "${project_root}/ros2_ws/install/setup.bash"
check "项目虚拟环境" test -x "${project_root}/.venv/bin/python3"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # 后续检查应使用与 build/run 脚本完全一致的 ROS、工作区和 venv 环境。
  source "${script_dir}/env.sh"
fi

check "Python 3.10" python3 -c 'import sys; raise SystemExit(sys.version_info[:2] != (3, 10))'
check "PyQt5" python3 -c 'import PyQt5'
check "OpenCV" python3 -c 'import cv2'
check "NumPy" python3 -c 'import numpy'
check "tf_transformations" python3 -c 'import tf_transformations'
check "交互 Python 依赖" python3 -c 'import faster_whisper, speech_recognition, pyttsx3, pyaudio'
check "CTranslate2 GPU" python3 -c 'import ctranslate2; assert ctranslate2.get_cuda_device_count() > 0; assert "float16" in ctranslate2.get_supported_compute_types("cuda")'
if [[ -d "${WHISPER_MODEL:-}" ]]; then
  check "faster-whisper 离线模型" test -s "${WHISPER_MODEL}/model.bin"
fi
if [[ -d "${WHISPER_RESCUE_MODEL:-}" ]]; then
  check "faster-whisper 二级复核模型" test -s "${WHISPER_RESCUE_MODEL}/model.bin"
fi
check "Kokoro 男声模型" bash -c \
  'test -s "$1/config.json" && test -s "$1/kokoro-v1_0.pth" && test -s "$1/voices/zm_yunxi.pt"' \
  _ "${TTS_MODEL_DIR:-${project_root}/go2_jiaohu/voice/models/kokoro-pytorch}"
kokoro_python="${project_root}/go2_jiaohu/third_party/kokoro-python"
cuda_sysroot="${project_root}/go2_jiaohu/third_party/cuda-sysroot"
check "Kokoro Jetson PyTorch sm_87" env -u PYTHONNOUSERSITE \
  PYTHONPATH="${kokoro_python}" \
  LD_LIBRARY_PATH="${kokoro_python}/torch/lib:${cuda_sysroot}/usr/lib/aarch64-linux-gnu:${cuda_sysroot}/usr/local/cuda-12.6/lib64:${HOME}/.local/lib/python3.10/site-packages/torch/lib" \
  /usr/bin/python3 -c 'import torch; assert torch.cuda.is_available(); assert "sm_87" in torch.cuda.get_arch_list()'
check "Ollama 命令" command -v ollama
check "Ollama 服务与 Qwen 模型" bash -c 'ollama list 2>/dev/null | grep -qi "qwen"'
check "MediaPipe Hands" timeout 30s python3 -c 'import mediapipe as mp; assert hasattr(mp.solutions, "hands")'
check "RealSense Python" python3 -c 'import pyrealsense2'
check "D435i 已连接" python3 -c 'import pyrealsense2 as rs; assert len(rs.context().query_devices()) > 0'
check "Unitree SDK2 Python" python3 -c 'import cyclonedds, unitree_sdk2py'
check "OpenCV GStreamer" python3 -c 'import cv2; assert "GStreamer:                   YES" in cv2.getBuildInformation()'

echo "[INFO] 本地模型: ${OLLAMA_MODEL:-qwen2.5:3b}；本地 ASR: ${WHISPER_MODEL:-large-v3-turbo}；TTS: ${TTS_VOICE:-zm_yunxi}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  check "go2_core 可发现" ros2 pkg prefix go2_core
  check "go2_navigation 可发现" ros2 pkg prefix go2_navigation
  check "Livox ROS Driver 2 可发现" ros2 pkg prefix livox_ros_driver2
  check "HAP 配置" test -s "${LIVOX_WS}/src/livox_ros_driver2/config/HAP_config.json"
  check "Nav2 RPP 控制器" ros2 pkg prefix nav2_regulated_pure_pursuit_controller
  check "CycloneDDS RMW" bash -c '[[ "$RMW_IMPLEMENTATION" == rmw_cyclonedds_cpp ]]'
  check "网卡 ${GO2_NETWORK_INTERFACE}" ip link show "${GO2_NETWORK_INTERFACE}"
  check "HAP 网卡 ${HAP_NETWORK_INTERFACE}" ip link show "${HAP_NETWORK_INTERFACE}"
  check "HAP ${HAP_IP} 可达" ping -c 1 -W 1 "${HAP_IP}"
fi

if [[ ${failures} -gt 0 ]]; then
  echo "发现 ${failures} 项未通过。请对照《使用教程.md》的排障章节。"
  exit 1
fi
echo "基础环境检查通过。连接机器狗后继续检查 ROS 话题与 TF。"
