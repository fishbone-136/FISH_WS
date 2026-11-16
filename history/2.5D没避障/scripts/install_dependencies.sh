#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"

if [[ ! -r /etc/os-release ]]; then
  echo "无法识别系统；本脚本仅支持 Ubuntu 22.04。" >&2
  exit 1
fi
. /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  echo "检测到 ${PRETTY_NAME:-unknown}；需要 Ubuntu 22.04。" >&2
  exit 1
fi

sudo apt update
sudo apt install -y software-properties-common curl ca-certificates gnupg locales
sudo add-apt-repository -y universe

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  architecture="$(dpkg --print-architecture)"
  ubuntu_codename="${UBUNTU_CODENAME:-jammy}"
  echo "deb [arch=${architecture} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${ubuntu_codename} main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt update
  sudo apt install -y ros-humble-desktop ros-dev-tools
fi

sudo apt install -y \
  build-essential cmake git python3-colcon-common-extensions python3-rosdep \
  python3-pip python3-venv python3-pyqt5 python3-pyqt5.qtsvg \
  python3-opencv python3-numpy python3-pyaudio portaudio19-dev \
  fonts-noto-cjk espeak-ng libopencv-dev libyaml-cpp-dev libgl1 \
  gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-nav2-dwb-controller ros-humble-nav2-rotation-shim-controller \
  ros-humble-nav2-navfn-planner \
  ros-humble-pcl-ros ros-humble-tf-transformations ros-humble-cv-bridge \
  ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl \
  ros-humble-tf2-sensor-msgs ros-humble-laser-geometry

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update --rosdistro humble

# Humble 的 setup 脚本会读取未定义的内部变量，需暂时关闭 nounset。
set +u
source /opt/ros/humble/setup.bash
set -u
rosdep install \
  --from-paths "${project_root}/ros2_ws/src" \
  --ignore-src --rosdistro humble -r -y

python3 -m venv --system-site-packages "${project_root}/.venv"
source "${project_root}/.venv/bin/activate"
python -m pip install --upgrade pip
python -m pip install -r "${project_root}/go2_jiaohu/requirements.txt"
bash "${script_dir}/install_interaction_dependencies.sh"

echo
echo "依赖安装完成。下一步："
echo "  1) 编辑 ${project_root}/go2_jiaohu/.env"
echo "  2) 运行 ${project_root}/scripts/build.sh"
