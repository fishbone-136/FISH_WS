#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/env.sh"
ros2 service call /mapping/save std_srvs/srv/Trigger '{}'
