#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage: ./scripts/build.sh [--workspace <path>] [--clean] [--without-external] [-- <colcon args>]

Builds autonomy plus the available core message package. RealSense is linked
directly through librealsense2 and Livox through Livox SDK2; neither ROS sensor
driver is built or launched.

The script never clones or vendors drivers. Put external repositories in the
same ROS workspace first, then source the target ROS distribution and run this
script. SLAM, elevation, mapping, Hybrid A*, MPPI, and detection are built
directly into autonomy. Super-LIO requires Eigen, PCL, GTSAM, glog, and TBB
to be installed before building.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd "${script_dir}/.." && pwd)"
workspace_dir="${WORKSPACE_DIR:-}"
clean=false
with_external=true
extra_args=()

while (($#)); do
  case "$1" in
    --workspace)
      workspace_dir="$2"
      shift 2
      ;;
    --clean)
      clean=true
      shift
      ;;
    --without-external)
      with_external=false
      shift
      ;;
    --)
      shift
      extra_args=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "error: unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "${workspace_dir}" ]]; then
  if [[ "$(basename "$(dirname "${package_dir}")")" == "src" ]]; then
    workspace_dir="$(cd "${package_dir}/../.." && pwd)"
  else
    workspace_dir="$(cd "${package_dir}/.." && pwd)"
  fi
fi

if ! command -v colcon >/dev/null 2>&1; then
  echo "error: colcon is not installed." >&2
  exit 1
fi
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
if [[ ! -d "${workspace_dir}" ]]; then
  echo "error: workspace does not exist: ${workspace_dir}" >&2
  exit 1
fi

mapfile -t available_packages < <(cd "${workspace_dir}" && colcon list -n)
contains_package() {
  local package="$1"
  local candidate
  for candidate in "${available_packages[@]}"; do
    [[ "${candidate}" == "${package}" ]] && return 0
  done
  return 1
}

packages=(autonomy)
if [[ "${with_external}" == true ]]; then
  for package in core; do
    if contains_package "${package}"; then
      packages+=("${package}")
    else
      echo "warning: ${package} is not a workspace source package; expecting an installed dependency." >&2
    fi
  done
fi
if ! contains_package autonomy; then
  echo "error: autonomy was not found under ${workspace_dir}/src. Use --workspace or add this package." >&2
  exit 1
fi

if [[ "${clean}" == true ]]; then
  for package in "${packages[@]}"; do
    rm -rf "${workspace_dir}/build/${package}" "${workspace_dir}/install/${package}" "${workspace_dir}/log/latest_build/${package}"
  done
fi

echo "Building: ${packages[*]}"
cd "${workspace_dir}"
colcon build --symlink-install --packages-select "${packages[@]}" "${extra_args[@]}" \
  --cmake-args "-DCMAKE_BUILD_TYPE=${BUILD_TYPE:-Release}"
