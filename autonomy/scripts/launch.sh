#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage: ./scripts/launch.sh [config_dir:=/absolute/path/config]

Starts RuntimeRunner. SensorManager opens RealSense through librealsense2 and
MID-360/MID-360S through Livox SDK2 itself; do not launch ROS sensor drivers.
SLAM, elevation, mapping, Hybrid A*, MPPI, and RGB-D detection run inside this
process; ROS remains only in MiddlewareManager's external I/O boundary.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd "${script_dir}/.." && pwd)"
config_dir=""
for argument in "$@"; do
  case "${argument}" in
    config_dir:=*) config_dir="${argument#config_dir:=}" ;;
    -h|--help) usage; exit 0 ;;
    *) echo "error: unknown argument: ${argument}" >&2; exit 1 ;;
  esac
done

if [[ -z "${config_dir}" ]]; then
  if [[ -d "${package_dir}/include/autonomy/parameter/config" ]]; then
    config_dir="${package_dir}/include/autonomy/parameter/config"
  else
    config_dir="$(ros2 pkg prefix autonomy)/share/autonomy/parameter/config"
  fi
fi
if [[ ! -f "${config_dir}/core.yaml" || ! -f "${config_dir}/sensor.yaml" ||
  ! -f "${config_dir}/algorithm.yaml" || ! -f "${config_dir}/middleware.yaml" ]]; then
  echo "error: required configuration files not found in: ${config_dir}" >&2
  exit 1
fi

AUTONOMY_CONFIG_DIR="${config_dir}" exec ros2 run autonomy autonomy_runtime_node
