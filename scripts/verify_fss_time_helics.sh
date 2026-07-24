#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 is not on PATH. Source your ROS 2 setup.bash first." >&2
  exit 2
fi

if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon is not installed. Install python3-colcon-common-extensions and rerun this script." >&2
  exit 2
fi

if [[ ! -d src/fss_time/third_party/HELICS ]]; then
  echo "Vendored HELICS source is missing at src/fss_time/third_party/HELICS." >&2
  exit 2
fi

echo "Building fss_time with vendored HELICS time coordination."

colcon build --symlink-install --packages-up-to fss_time
set +u
source install/setup.bash
set -u
colcon test --packages-select fss_time --event-handlers console_direct+
colcon test-result --verbose
