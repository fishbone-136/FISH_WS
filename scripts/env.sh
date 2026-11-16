#!/usr/bin/env bash
# 用法：source scripts/env.sh

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "未找到 /opt/ros/humble/setup.bash，请先运行 install_dependencies.sh" >&2
  return 1 2>/dev/null || exit 1
fi

# ROS 2 和 virtualenv 的 setup 脚本会读取尚未定义的内部变量。调用方（如
# build.sh/check_system.sh）启用 `set -u` 时，直接 source 会在这里提前退出。
_go2_restore_nounset=false
case "$-" in
  *u*)
    _go2_restore_nounset=true
    set +u
    ;;
esac

source /opt/ros/humble/setup.bash

# HAP uses Livox ROS Driver 2 from a small underlay workspace.  Source it
# explicitly before this project so a clean rebuild does not depend on a stale
# prefix chain recorded in ros2_ws/install/setup.bash.
livox_ws="${LIVOX_WS:-${project_root}/../GO2/livox_ws}"
if [[ -f "${livox_ws}/install/setup.bash" ]]; then
  source "${livox_ws}/install/setup.bash"
  export LIVOX_WS="${livox_ws}"
else
  echo "未找到 Livox 工作区: ${livox_ws}/install/setup.bash" >&2
  echo "请设置 LIVOX_WS，或先构建 livox_ros_driver2。" >&2
  return 1 2>/dev/null || exit 1
fi
unset livox_ws

if [[ -f "${project_root}/ros2_ws/install/setup.bash" ]]; then
  source "${project_root}/ros2_ws/install/setup.bash"
fi

# The Livox component is loaded with dlopen. Relocated build artifacts may
# retain the old workspace RUNPATH even when every install symlink is valid.
# Resolve the private SDK from this workspace for both navigation and UI.
_go2_livox_sdk_lib="${project_root}/third_party/livox_sdk2_install/lib"
if [[ -f "${_go2_livox_sdk_lib}/liblivox_lidar_sdk_hap.so" ]]; then
  export LD_LIBRARY_PATH="${_go2_livox_sdk_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
unset _go2_livox_sdk_lib

if [[ -f "${project_root}/.venv/bin/activate" ]]; then
  # 隔离用户目录中可能与 Ubuntu/ROS ABI 不兼容的 pip 包（例如 NumPy 1.26
  # 会使 Humble 的 tf_transformations 在导入时失败）。
  export PYTHONNOUSERSITE=1
  source "${project_root}/.venv/bin/activate"
fi

env_file="${project_root}/go2_jiaohu/.env"
if [[ -f "${env_file}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${env_file}"
  set +a
fi

# 确保桌面代理不会截获本机 Ollama 回环请求。
export NO_PROXY="${NO_PROXY:+${NO_PROXY},}127.0.0.1,localhost,::1"
export no_proxy="${no_proxy:+${no_proxy},}127.0.0.1,localhost,::1"

if [[ "${_go2_restore_nounset}" == true ]]; then
  set -u
fi
unset _go2_restore_nounset

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GO2_NETWORK_INTERFACE="${GO2_NETWORK_INTERFACE:-eth0}"
export GO2_IP="${GO2_IP:-192.168.123.161}"
export HAP_NETWORK_INTERFACE="${HAP_NETWORK_INTERFACE:-enx00e04c3c2370}"
export HAP_IP="${HAP_IP:-192.168.1.100}"
if [[ ! "${GO2_NETWORK_INTERFACE}" =~ ^[A-Za-z0-9_.:-]+$ ]]; then
  echo "非法 GO2_NETWORK_INTERFACE: ${GO2_NETWORK_INTERFACE}" >&2
  return 1 2>/dev/null || exit 1
fi
if [[ ! "${HAP_NETWORK_INTERFACE}" =~ ^[A-Za-z0-9_.:-]+$ ]]; then
  echo "非法 HAP_NETWORK_INTERFACE: ${HAP_NETWORK_INTERFACE}" >&2
  return 1 2>/dev/null || exit 1
fi
if [[ ! "${HAP_IP}" =~ ^[0-9A-Fa-f:.]+$ ]]; then
  echo "非法 HAP_IP: ${HAP_IP}" >&2
  return 1 2>/dev/null || exit 1
fi
if [[ ! "${GO2_IP}" =~ ^[0-9A-Fa-f:.]+$ ]]; then
  echo "非法 GO2_IP: ${GO2_IP}" >&2
  return 1 2>/dev/null || exit 1
fi
# Go2 的 DDS 广播偶尔需要一个完整的发现周期才会被 ROS 2 看到。除了绑定
# 机器人网卡和保留组播，再把机器人的固定地址设为显式 peer，避免刚启动时
# Nav2 的第一批速度命令在运动服务订阅端出现之前被丢弃。
# HAP PointCloud2 bursts exceed Linux's usual 208 KiB UDP buffer ceiling.
# Request a large receive buffer for these DDS participants. The optional
# config/99-go2-lidar-buffers.conf raises only the kernel upper bounds; keep
# accepting the OS default on hosts where that tuning has not been installed.
export CYCLONEDDS_URI="<CycloneDDS><Domain Id='any'><General><Interfaces><NetworkInterface name='${GO2_NETWORK_INTERFACE}' priority='default' multicast='default'/></Interfaces></General><Discovery><Peers><Peer Address='${GO2_IP}'/></Peers></Discovery><Internal><SocketReceiveBufferSize min='default' max='8MiB'/><SocketSendBufferSize min='64KiB' max='1MiB'/></Internal></Domain></CycloneDDS>"
export GO2_HUMBLE_PROJECT_ROOT="${project_root}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/go2-matplotlib-${UID:-1000}}"
mkdir -p "${MPLCONFIGDIR}"

# 可随项目部署的 Jetson CUDA 12.6 与自编译 CTranslate2 GPU 运行库。
_go2_cuda_root="${project_root}/go2_jiaohu/third_party/cuda-sysroot/usr/local/cuda-12.6"
_go2_ct2_root="${project_root}/go2_jiaohu/third_party/ctranslate2-cuda"
if [[ -x "${_go2_cuda_root}/bin/nvcc" && -d "${_go2_ct2_root}/lib" ]]; then
  export PATH="${_go2_cuda_root}/bin:${PATH}"
  export LD_LIBRARY_PATH="${_go2_ct2_root}/lib:${_go2_cuda_root}/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
unset _go2_cuda_root _go2_ct2_root
