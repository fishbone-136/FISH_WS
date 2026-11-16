#!/usr/bin/env bash
# Rebuild the changed packages locally; never reuse HAP_ws CMake caches.
set -eo pipefail
fluency_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "${fluency_root}/ros2_ws/install/setup.bash" ]]; then
  source "${fluency_root}/ros2_ws/install/setup.bash"
fi
export PYTHONNOUSERSITE=1
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
colcon --log-base "${fluency_root}/reports/navigation_fluency_build_logs" build \
  --base-paths "${fluency_root}/ros2_ws/src/go2_navigation" \
               "${fluency_root}/ros2_ws/src/go2_lio_navigation" \
  --packages-select go2_navigation go2_lio_navigation \
  --allow-overriding go2_navigation go2_lio_navigation \
  --executor sequential \
  --build-base "${fluency_root}/ros2_ws/build_fluency" \
  --install-base "${fluency_root}/ros2_ws/install" \
  --symlink-install --cmake-force-configure --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
