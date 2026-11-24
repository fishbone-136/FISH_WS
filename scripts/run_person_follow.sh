#!/usr/bin/env bash
set -euo pipefail

follow_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
follow_project_root="$(cd -- "${follow_script_dir}/.." && pwd)"

# No ROS/Livox setup, nodes, or audio services are needed by this entry point.
if [[ -x "${follow_project_root}/.venv/bin/python3" ]]; then
  follow_python="${follow_project_root}/.venv/bin/python3"
else
  follow_python=python3
fi
export PYTHONNOUSERSITE=1
cd "${follow_project_root}/go2_jiaohu"
exec "${follow_python}" -m deploy.person_follow "$@"
