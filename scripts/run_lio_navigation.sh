#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/env.sh"

/usr/bin/python3 - <<'PY'
from go2_navigation.runtime_guard import navigation_network_error
error = navigation_network_error()
if error:
    print('导航未启动：' + error, flush=True)
    raise SystemExit(2)
PY

# A low stance obscures useful floor and obstacle observations.
# Check actual feedback before starting any navigation/base nodes. Explicit
# sensor-only runs do not require the standing navigation precondition.
check_stance=true
for argument in "$@"; do
  if [[ "$argument" == "enable_go2_base:=false" ]]; then
    check_stance=false
  elif [[ "$argument" == "enable_go2_base:=true" ]]; then
    check_stance=true
  fi
done
if [[ "$check_stance" == true ]]; then
  /usr/bin/python3 "${script_dir}/check_navigation_stance.py"
fi

# This wrapper is the supported LIO navigation entry point.  env.sh selects
# CycloneDDS and binds discovery to the Go2 network; launching ros2 directly
# can leave /api/sport/request with no robot-side subscriber even while Nav2
# appears active and publishes velocity commands.
exec ros2 launch go2_lio_navigation go2_lio_navigation.launch.py \
  use_rviz:=true \
  enable_go2_base:=true \
  enable_loop_closure:=true \
  enable_perception:=true \
  enable_terrain:=true \
  enable_nav2:=true \
  nav2_use_composition:=true \
  controller_mode:=dwb \
  terrain_mode:=indoor \
  "$@"
