#!/usr/bin/env bash
# Offline tests only: localhost DDS, isolated domains, no robot/base launcher.
set -eo pipefail
fluency_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${fluency_root}"
source /opt/ros/humble/setup.bash
source "${fluency_root}/ros2_ws/install/setup.bash"
export GO2_HUMBLE_PROJECT_ROOT="${fluency_root}"
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=90 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOG_DIR="${fluency_root}/reports/navigation_fluency_ros_logs"
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
/usr/bin/python3 -m unittest discover -s ros2_ws/src/go2_lio_navigation/test -p 'test_*.py'
/usr/bin/python3 -m unittest discover -s ros2_ws/src/go2_navigation/test -p 'test_*.py'
ctest --test-dir ros2_ws/build_fluency/go2_navigation --output-on-failure
ROS_DOMAIN_ID=93 /usr/bin/python3 ros2_ws/src/go2_navigation/test/check_navigation_fluency.py
