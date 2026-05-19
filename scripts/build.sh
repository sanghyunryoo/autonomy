#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/build.sh <x86_64|aarch64> [--clean] [-- <extra colcon args>]

Examples:
  ./scripts/build.sh x86_64
  ./scripts/build.sh aarch64
  ./scripts/build.sh x86_64 --clean
  CMAKE_TOOLCHAIN_FILE=/path/to/aarch64-toolchain.cmake ./scripts/build.sh aarch64
  ./scripts/build.sh x86_64 -- --event-handlers console_direct+

Environment:
  ROS_DISTRO             ROS 2 distro to source. Default: humble
  PYTHON_EXECUTABLE      Python executable passed to CMake when set.
  CMAKE_TOOLCHAIN_FILE   Optional CMake toolchain for cross compilation.
  BUILD_TYPE             CMake build type. Default: Release
  ONNXRUNTIME_VERSION    ONNX Runtime binary release. Default: 1.18.1
  OPENVINS_REPO          OpenVINS git repository. Default: https://github.com/rpng/open_vins.git
  OPENVINS_VERSION       OpenVINS git branch/tag/commit. Default: master
  SKIP_OPENVINS_CLONE    Set to 1 when OpenVINS is already provided in third_party/open_vins.
  ALLOW_CONDA_BUILD_ENV  Set to 1 to keep conda paths in the build environment.
EOF
}

die() {
  echo "error: $*" >&2
  exit 1
}

drop_path_prefix() {
  local value="${1:-}"
  local prefix="${2:-}"
  local cleaned=""
  local entry

  IFS=':' read -r -a entries <<< "${value}"
  for entry in "${entries[@]}"; do
    [[ -z "${entry}" ]] && continue
    if [[ -n "${prefix}" && "${entry}" == "${prefix}"* ]]; then
      continue
    fi
    if [[ -z "${cleaned}" ]]; then
      cleaned="${entry}"
    else
      cleaned="${cleaned}:${entry}"
    fi
  done
  printf '%s' "${cleaned}"
}

sanitize_conda_build_env() {
  if [[ "${ALLOW_CONDA_BUILD_ENV:-0}" == "1" ]]; then
    return
  fi

  local conda_root="${CONDA_PREFIX:-/root/miniconda3}"
  PATH="$(drop_path_prefix "${PATH:-}" "${conda_root}")"
  CMAKE_PREFIX_PATH="$(drop_path_prefix "${CMAKE_PREFIX_PATH:-}" "${conda_root}")"
  LD_LIBRARY_PATH="$(drop_path_prefix "${LD_LIBRARY_PATH:-}" "${conda_root}")"
  LIBRARY_PATH="$(drop_path_prefix "${LIBRARY_PATH:-}" "${conda_root}")"
  PKG_CONFIG_PATH="$(drop_path_prefix "${PKG_CONFIG_PATH:-}" "${conda_root}")"
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER
  export PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH LIBRARY_PATH PKG_CONFIG_PATH
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
package_dir="$(cd "${script_dir}/.." && pwd)"
workspace_dir="$(cd "${package_dir}/../.." && pwd)"
package_name="height_map_ros2"

ros_distro="${ROS_DISTRO:-humble}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"
[[ -f "${ros_setup}" ]] || die "ROS setup file not found: ${ros_setup}"

ort_version="${ONNXRUNTIME_VERSION:-1.18.1}"
case "${target_arch}" in
  x86_64)
    ort_dir="${package_dir}/third_party/onnxruntime"
    ort_asset_arch="x64"
    ;;
  aarch64)
    ort_dir="${package_dir}/third_party/onnxruntime-aarch64"
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

clean_third_party_build_artifacts() {
  echo "Cleaning third-party source-tree build artifacts..."
  rm -rf "${package_dir}/third_party/open_vins/build"
}

ensure_openvins() {
  local third_party_openvins="${package_dir}/third_party/open_vins"
  local workspace_openvins="${workspace_dir}/src/open_vins"

  if [[ ! -f "${third_party_openvins}/ov_msckf/package.xml" && -f "${workspace_openvins}/ov_msckf/package.xml" && ! -L "${workspace_openvins}" ]]; then
    echo "Moving existing workspace OpenVINS into ${third_party_openvins}..."
    mkdir -p "$(dirname "${third_party_openvins}")"
    mv "${workspace_openvins}" "${third_party_openvins}"
  fi

  if [[ ! -f "${third_party_openvins}/ov_msckf/package.xml" ]]; then
    if [[ "${SKIP_OPENVINS_CLONE:-0}" == "1" ]]; then
      die "OpenVINS not found at ${third_party_openvins}. Add it there or unset SKIP_OPENVINS_CLONE."
    fi

    local repo="${OPENVINS_REPO:-https://github.com/rpng/open_vins.git}"
    local version="${OPENVINS_VERSION:-master}"
    echo "Cloning OpenVINS ${version} into ${third_party_openvins}..."
    git clone --recursive --branch "${version}" "${repo}" "${third_party_openvins}"
  fi

  if [[ -L "${workspace_openvins}" ]]; then
    local linked_target
    linked_target="$(readlink "${workspace_openvins}")"
    if [[ "${linked_target}" != "${third_party_openvins}" ]]; then
      rm -f "${workspace_openvins}"
      ln -s "${third_party_openvins}" "${workspace_openvins}"
    fi
    return
  fi

  if [[ -e "${workspace_openvins}" ]]; then
    die "${workspace_openvins} exists but is not the managed OpenVINS symlink."
  fi

  ln -s "${third_party_openvins}" "${workspace_openvins}"
}

host_arch="$(uname -m)"
if [[ "${host_arch}" != "${target_arch}" && -z "${CMAKE_TOOLCHAIN_FILE:-}" ]]; then
  echo "warning: host arch is ${host_arch}, target is ${target_arch}, and CMAKE_TOOLCHAIN_FILE is not set." >&2
  echo "warning: continuing as a native build; set CMAKE_TOOLCHAIN_FILE for cross compilation." >&2
fi

ensure_onnxruntime
ensure_openvins
sanitize_conda_build_env

if [[ "${clean}" == true ]]; then
  echo "Cleaning workspace build/install/log..."
  rm -rf "${workspace_dir}/build" "${workspace_dir}/install" "${workspace_dir}/log"
  clean_third_party_build_artifacts
fi

set +u
source "${ros_setup}"
set -u

cmake_args=(
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE:-Release}"
  -DONNXRUNTIME_ROOT="${ort_dir}"
  -DENABLE_ARUCO_TAGS=OFF
  -DCMAKE_IGNORE_PREFIX_PATH="/root/miniconda3"
)

if [[ -n "${PYTHON_EXECUTABLE:-}" ]]; then
  cmake_args+=(
    -DPython3_EXECUTABLE="${PYTHON_EXECUTABLE}"
    -DPYTHON_EXECUTABLE="${PYTHON_EXECUTABLE}"
  )
else
  cmake_args+=(
    -DPython3_EXECUTABLE="/usr/bin/python3"
    -DPYTHON_EXECUTABLE="/usr/bin/python3"
  )
fi

if [[ -n "${CMAKE_TOOLCHAIN_FILE:-}" ]]; then
  cmake_args+=(-DCMAKE_TOOLCHAIN_FILE="${CMAKE_TOOLCHAIN_FILE}")
fi

echo "Building ${package_name} for ${target_arch}"
echo "Workspace: ${workspace_dir}"
echo "ONNX Runtime: ${ort_dir}"
echo "Python: ${PYTHON_EXECUTABLE:-/usr/bin/python3}"

cd "${workspace_dir}"
colcon build \
  --symlink-install \
  --packages-up-to ov_msckf "${package_name}" \
  "${extra_colcon_args[@]}" \
  --cmake-args "${cmake_args[@]}"
