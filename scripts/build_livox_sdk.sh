#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
cmake -S "${project_root}/third_party/livox_sdk2_system_spdlog" \
  -B "${project_root}/third_party/livox_sdk2_build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${project_root}/third_party/livox_sdk2_install"
cmake --build "${project_root}/third_party/livox_sdk2_build" --parallel 2
cmake --install "${project_root}/third_party/livox_sdk2_build"
