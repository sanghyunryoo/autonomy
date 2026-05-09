#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./build.sh <x86_64|aarch64> [--clean] [-- <extra colcon args>]

Examples:
  ./build.sh x86_64
  ./build.sh aarch64
  ./build.sh x86_64 --clean
  CMAKE_TOOLCHAIN_FILE=/path/to/aarch64-toolchain.cmake ./build.sh aarch64
  ./build.sh x86_64 -- --event-handlers console_direct+

Environment:
  ROS_DISTRO             ROS 2 distro to source. Default: humble
  PYTHON_EXECUTABLE      Python executable passed to CMake when set.
  CMAKE_TOOLCHAIN_FILE   Optional CMake toolchain for cross compilation.
  BUILD_TYPE             CMake build type. Default: Release
  ONNXRUNTIME_VERSION    ONNX Runtime binary release. Default: 1.18.1
EOF
}

die() {
  echo "error: $*" >&2
  exit 1
}

if [[ $# -lt 1 ]]; then
  usage
  exit 2
fi

target_arch="$1"
shift

case "${target_arch}" in
  x86_64|aarch64) ;;
  -h|--help)
    usage
    exit 0
    ;;
  *)
    usage
    die "unsupported build target '${target_arch}'"
    ;;
esac

clean=false
extra_colcon_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --clean)
      clean=true
      shift
      ;;
    --)
      shift
      extra_colcon_args=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown argument '$1'. Put extra colcon args after --"
      ;;
  esac
done

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${script_dir}/../.." && pwd)"
package_name="height_map_ros2"

ros_distro="${ROS_DISTRO:-humble}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"
[[ -f "${ros_setup}" ]] || die "ROS setup file not found: ${ros_setup}"

ort_version="${ONNXRUNTIME_VERSION:-1.18.1}"
case "${target_arch}" in
  x86_64)
    ort_dir="${script_dir}/third_party/onnxruntime"
    ort_asset_arch="x64"
    ;;
  aarch64)
    ort_dir="${script_dir}/third_party/onnxruntime-aarch64"
    ort_asset_arch="aarch64"
    ;;
esac

ensure_onnxruntime() {
  if [[ -f "${ort_dir}/include/onnxruntime_cxx_api.h" && -f "${ort_dir}/lib/libonnxruntime.so" ]]; then
    return
  fi

  mkdir -p "${ort_dir}" /tmp/height_map_ros2_onnxruntime
  local archive="/tmp/height_map_ros2_onnxruntime/onnxruntime-linux-${ort_asset_arch}-${ort_version}.tgz"
  local url="https://github.com/microsoft/onnxruntime/releases/download/v${ort_version}/onnxruntime-linux-${ort_asset_arch}-${ort_version}.tgz"

  echo "Downloading ONNX Runtime ${ort_version} for ${target_arch}..."
  curl -L -o "${archive}" "${url}"
  tar --no-same-owner -xzf "${archive}" --strip-components=1 -C "${ort_dir}"
  touch "${ort_dir}/COLCON_IGNORE"

  [[ -f "${ort_dir}/include/onnxruntime_cxx_api.h" ]] ||
    die "ONNX Runtime headers were not extracted into ${ort_dir}"
  [[ -f "${ort_dir}/lib/libonnxruntime.so" ]] ||
    die "ONNX Runtime library was not extracted into ${ort_dir}"
}

host_arch="$(uname -m)"
if [[ "${host_arch}" != "${target_arch}" && -z "${CMAKE_TOOLCHAIN_FILE:-}" ]]; then
  echo "warning: host arch is ${host_arch}, target is ${target_arch}, and CMAKE_TOOLCHAIN_FILE is not set." >&2
  echo "warning: continuing as a native build; set CMAKE_TOOLCHAIN_FILE for cross compilation." >&2
fi

ensure_onnxruntime

if [[ "${clean}" == true ]]; then
  echo "Cleaning workspace build/install/log..."
  rm -rf "${workspace_dir}/build" "${workspace_dir}/install" "${workspace_dir}/log"
fi

set +u
source "${ros_setup}"
set -u

cmake_args=(
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE:-Release}"
  -DONNXRUNTIME_ROOT="${ort_dir}"
)

if [[ -n "${PYTHON_EXECUTABLE:-}" ]]; then
  cmake_args+=(
    -DPython3_EXECUTABLE="${PYTHON_EXECUTABLE}"
    -DPYTHON_EXECUTABLE="${PYTHON_EXECUTABLE}"
  )
fi

if [[ -n "${CMAKE_TOOLCHAIN_FILE:-}" ]]; then
  cmake_args+=(-DCMAKE_TOOLCHAIN_FILE="${CMAKE_TOOLCHAIN_FILE}")
fi

echo "Building ${package_name} for ${target_arch}"
echo "Workspace: ${workspace_dir}"
echo "ONNX Runtime: ${ort_dir}"

cd "${workspace_dir}"
colcon build \
  --symlink-install \
  --packages-select "${package_name}" \
  "${extra_colcon_args[@]}" \
  --cmake-args "${cmake_args[@]}"
