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

run_ecal_network_smoke()
{
  local test_exe="$1"
  if [[ ! -x "${test_exe}" ]]; then
    return 0
  fi

  local config_dir
  config_dir="$(mktemp -d /tmp/fss_time_ecal_network.XXXXXX)"
  printf 'communication_mode: "network"\n' > "${config_dir}/ecal.yaml"

  echo "Running eCAL network-mode loopback smoke test with ${config_dir}/ecal.yaml."
  (
    cd "${config_dir}"
    "${test_exe}"
  )

  if command -v strace >/dev/null 2>&1; then
    local trace_log="${config_dir}/strace.log"
    (
      cd "${config_dir}"
      strace -f -e trace=setsockopt -s 128 "${test_exe}" >"${trace_log}" 2>&1
    )
    if grep -q 'IP_ADD_MEMBERSHIP.*239\.0\.0\.1' "${trace_log}"; then
      echo "Observed eCAL joining UDP multicast group 239.0.0.1 in network mode."
    else
      echo "eCAL network-mode smoke passed, but strace did not show 239.0.0.1 multicast membership." >&2
      exit 1
    fi
  else
    echo "strace is not installed; skipped multicast socket observation."
  fi
}

has_ecal_sdk()
{
  if [[ -n "${CMAKE_PREFIX_PATH:-}" ]] &&
    find ${CMAKE_PREFIX_PATH//:/ } -path '*/cmake/eCAL/eCALConfig.cmake' -print -quit 2>/dev/null | grep -q .
  then
    return 0
  fi

  if find /usr /usr/local /opt -path '*/cmake/eCAL/eCALConfig.cmake' -print -quit 2>/dev/null | grep -q .; then
    return 0
  fi

  ldconfig -p 2>/dev/null | grep -q 'libecal_core'
}

if ! has_ecal_sdk; then
  echo "eCAL SDK is not installed or not discoverable. Install eCAL or set CMAKE_PREFIX_PATH to its prefix." >&2
  exit 2
fi

echo "eCAL detected: fss_time will build with the eCAL transport."

colcon build --symlink-install --packages-up-to fss_time
set +u
source install/setup.bash
set -u
colcon test --packages-select fss_time --event-handlers console_direct+
colcon test-result --verbose
ecal_test_exe="$(find "$(pwd)/build/fss_time" -type f -name test_ecal_time_transport -print -quit 2>/dev/null || true)"
run_ecal_network_smoke "${ecal_test_exe}"
