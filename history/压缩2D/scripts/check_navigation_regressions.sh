#!/usr/bin/env bash
# Repeatable offline regression: no physical interface, no real navigation goal.
set -eo pipefail
audit_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${audit_root}"
source /opt/ros/humble/setup.bash
source "${audit_root}/ros2_ws/install/setup.bash"
export PYTHONNOUSERSITE=1 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
audit_logs="$(mktemp -d "${audit_root}/reports/navigation_regressions_XXXXXXXX")"
audit_failed=0
run_check() {
  local audit_label="$1"
  shift
  local audit_status=0
  "$@" > "${audit_logs}/${audit_label}.log" 2>&1 || audit_status=$?
  cat "${audit_logs}/${audit_label}.log"
  # Launch can return zero after a native child segfaults during shutdown.
  # A functional PASS must not hide that process failure.
  if [[ "${audit_status}" != 0 ]] || rg -q 'process has died.*exit code -[0-9]+,|failed to terminate.*seconds after receiving|^Traceback \(most recent call last\):' "${audit_logs}/${audit_label}.log"; then
    echo "FAIL ${audit_label}: command exit=${audit_status}; inspect ${audit_logs}/${audit_label}.log"
    audit_failed=1
  else
    echo "PASS CHECK ${audit_label}"
  fi
}
export ROS_DOMAIN_ID=90
run_check lio_unit /usr/bin/python3 -m unittest discover -s ros2_ws/src/go2_lio_navigation/test -p 'test_*.py'
run_check navigation_unit /usr/bin/python3 -m unittest discover -s ros2_ws/src/go2_navigation/test -p 'test_*.py'
run_check supervision /usr/bin/python3 scripts/test_process_supervision.py
run_check native_ground ros2_ws/build/go2_perception/test_ground_plane
run_check lio_recovery env ROS_DOMAIN_ID=85 /usr/bin/python3 ros2_ws/src/go2_lio_navigation/test/check_lio_bridge_freshness.py
run_check base_feedback env ROS_DOMAIN_ID=86 /usr/bin/python3 ros2_ws/src/go2_lio_navigation/test/check_base_feedback_ros.py
run_check fault_retry env ROS_DOMAIN_ID=91 /usr/bin/python3 ros2_ws/src/go2_lio_navigation/test/check_fault_retry_ros.py
run_check scan_configuration env ROS_DOMAIN_ID=87 /usr/bin/python3 ros2_ws/src/go2_perception/test/check_scan_configuration_ros.py
run_check scan_alignment env ROS_DOMAIN_ID=89 /usr/bin/python3 ros2_ws/src/go2_perception/test/check_scan_alignment_ros.py
run_check memory_freshness env ROS_DOMAIN_ID=89 /usr/bin/python3 ros2_ws/src/go2_lio_navigation/test/check_memory_freshness_ros.py
run_check obstacle_memory env ROS_DOMAIN_ID=87 /usr/bin/python3 ros2_ws/src/go2_lio_navigation/test/check_obstacle_memory_ros.py
run_check mixed_crossing env ROS_DOMAIN_ID=87 /usr/bin/python3 ros2_ws/src/go2_lio_navigation/test/check_obstacle_memory_ros.py --mixed-crossing --fast-crossing
run_check obstacle_pipeline env ROS_DOMAIN_ID=83 /usr/bin/python3 ros2_ws/src/go2_navigation/test/check_obstacle_pipeline.py --current-scan-layer
run_check rear_goals env ROS_DOMAIN_ID=88 /usr/bin/python3 ros2_ws/src/go2_lio_navigation/test/check_return_navigation_ros.py
echo "Regression logs: ${audit_logs}; result=${audit_failed}"
exit "${audit_failed}"
