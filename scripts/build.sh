#!/usr/bin/env bash
set -Eeuo pipefail

# Single build/setup entrypoint for autonomy.
#
# Simulation host:
#   ./scripts/build.sh sim [--clean] [-- <extra colcon args>]
#
# Jetson hardware:
#   ./scripts/build.sh jetson [options] [-- <extra colcon args>]
#
# Build-only compatibility:
#   ./scripts/build.sh x86_64 [--clean] [-- <extra colcon args>]
#   ./scripts/build.sh aarch64 [--clean] [-- <extra colcon args>]

usage() {
  cat <<'EOF'
Usage:
  ./scripts/build.sh sim [--clean] [-- <extra colcon args>]
  ./scripts/build.sh jetson [options] [-- <extra colcon args>]
  ./scripts/build.sh <x86_64|aarch64> [--clean] [-- <extra colcon args>]

Modes:
  sim       Build autonomy for the current simulation/development machine.
  jetson    Configure Jetson dependencies, build RealSense, then build autonomy.
  x86_64    Build-only alias for sim target architecture.
  aarch64   Build-only alias for Jetson target architecture.

Common options:
  --clean                    Remove previous build/install/log before building.
  --setup-only               Run setup steps without building autonomy.
  --skip-autonomy-build      Do not build autonomy after setup.
  --workspace <path>         ROS workspace. Default: inferred workspace or $HOME/ros2_ws.
  --ros-distro <name>        ROS 2 distro. Default: humble.
  --jobs <n>                 Low-memory shortcut: set colcon, CMake, and librealsense jobs.
  --colcon-workers <n>       Maximum colcon packages built in parallel.
  --cmake-jobs <n>           Maximum compiler jobs inside CMake/Make/Ninja builds.
  --librealsense-jobs <n>    Maximum jobs for the librealsense source build.
  -- <args>                  Extra arguments passed to colcon build.

Jetson setup options:
  --with-desktop             Install ros-humble-desktop instead of ros-humble-ros-base.
  --headless                 Skip graphical RealSense examples/viewer.
  --no-cuda                  Build librealsense without CUDA acceleration.
  --python                   Build pyrealsense2 Python bindings.
  --skip-ros                 Do not install ROS 2 apt packages.
  --skip-librealsense        Do not build/install librealsense.
  --skip-realsense-ros       Do not clone/build realsense-ros.
  --librealsense-ref <ref>   librealsense tag/branch/commit. Default: v2.57.7.
  --librealsense-source-dir <path>
                             librealsense source directory. Default: $HOME/librealsense.
  --realsense-ros-ref <ref>  realsense-ros tag/branch/commit. Default: 4.57.7.

Environment:
  ROS_DISTRO                 ROS 2 distro to source. Default: humble.
  WORKSPACE_DIR              ROS workspace override.
  PYTHON_EXECUTABLE          Python executable passed to CMake when set.
  CMAKE_TOOLCHAIN_FILE       Optional CMake toolchain for cross compilation.
  BUILD_TYPE                 CMake build type. Default: Release.
  BUILD_JOBS                 Same as --jobs when set.
  COLCON_WORKERS             Same as --colcon-workers when set.
  CMAKE_BUILD_JOBS           Same as --cmake-jobs when set.
  LIBREALSENSE_JOBS          Same as --librealsense-jobs when set.
  RUN_ROSDEP                 Set to 1 to run rosdep before build-only flows.
  ONNXRUNTIME_VERSION        ONNX Runtime binary release. Default: 1.18.1.
  OPENVINS_REPO              OpenVINS git repository. Default: https://github.com/rpng/open_vins.git.
  OPENVINS_VERSION           OpenVINS git branch/tag/commit. Default: master.
  SKIP_OPENVINS_CLONE        Set to 1 when OpenVINS is already in third_party/open_vins.
  ALLOW_CONDA_BUILD_ENV      Set to 1 to keep conda paths in the build environment.
EOF
}

log() {
  printf '\n\033[1;32m[INFO]\033[0m %s\n' "$*"
}

warn() {
  printf '\n\033[1;33m[WARN]\033[0m %s\n' "$*" >&2
}

die() {
  printf '\n\033[1;31m[ERROR]\033[0m %s\n' "$*" >&2
  exit 1
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "Missing required command: $1"
}

apt_install() {
  sudo apt-get install -y "$@"
}

apt_package_installed() {
  dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q "install ok installed"
}

all_apt_packages_installed() {
  local package
  for package in "$@"; do
    if ! apt_package_installed "${package}"; then
      return 1
    fi
  done
  return 0
}

require_positive_int() {
  local name="$1"
  local value="$2"
  if [[ ! "${value}" =~ ^[1-9][0-9]*$ ]]; then
    die "${name} must be a positive integer"
  fi
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

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd "${script_dir}/.." && pwd)"
inferred_workspace_dir="$(cd "${package_dir}/../.." && pwd)"

mode="${1:-}"
if [[ -z "${mode}" || "${mode}" == "-h" || "${mode}" == "--help" ]]; then
  usage
  [[ -z "${mode}" ]] && exit 2 || exit 0
fi
shift

case "${mode}" in
  sim)
    target_arch="x86_64"
    run_jetson_setup="OFF"
    build_autonomy="ON"
    ;;
  jetson)
    target_arch="aarch64"
    run_jetson_setup="ON"
    build_autonomy="ON"
    ;;
  x86_64|aarch64)
    target_arch="${mode}"
    run_jetson_setup="OFF"
    build_autonomy="ON"
    ;;
  *)
    usage
    die "unsupported mode '${mode}'"
    ;;
esac

ros_distro="${ROS_DISTRO:-humble}"
workspace_dir="${WORKSPACE_DIR:-${inferred_workspace_dir}}"
if [[ "${mode}" == "jetson" && -z "${WORKSPACE_DIR:-}" ]]; then
  workspace_dir="${HOME}/ros2_ws"
fi

clean="OFF"
install_ros="ON"
install_ros_desktop="OFF"
build_librealsense="ON"
build_realsense_ros="ON"
use_cuda="ON"
build_graphical="ON"
build_python="OFF"
librealsense_ref="${LIBREALSENSE_REF:-v2.57.7}"
librealsense_src_dir="${LIBREALSENSE_SRC_DIR:-${HOME}/librealsense}"
realsense_ros_ref="${REALSENSE_ROS_REF:-4.57.7}"
realsense_ros_repo="${REALSENSE_ROS_REPO:-https://github.com/realsenseai/realsense-ros.git}"
build_jobs="${BUILD_JOBS:-}"
colcon_workers="${COLCON_WORKERS:-${BUILD_JOBS:-}}"
cmake_build_jobs="${CMAKE_BUILD_JOBS:-${BUILD_JOBS:-}}"
librealsense_jobs="${LIBREALSENSE_JOBS:-${BUILD_JOBS:-}}"
extra_colcon_args=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --clean)
      clean="ON"
      shift
      ;;
    --setup-only)
      run_jetson_setup="ON"
      build_autonomy="OFF"
      shift
      ;;
    --skip-autonomy-build)
      build_autonomy="OFF"
      shift
      ;;
    --workspace)
      [[ $# -ge 2 ]] || die "--workspace requires a value"
      workspace_dir="$2"
      shift 2
      ;;
    --ros-distro)
      [[ $# -ge 2 ]] || die "--ros-distro requires a value"
      ros_distro="$2"
      shift 2
      ;;
    --jobs)
      [[ $# -ge 2 ]] || die "--jobs requires a value"
      require_positive_int "--jobs" "$2"
      build_jobs="$2"
      colcon_workers="$2"
      cmake_build_jobs="$2"
      librealsense_jobs="$2"
      shift 2
      ;;
    --colcon-workers)
      [[ $# -ge 2 ]] || die "--colcon-workers requires a value"
      require_positive_int "--colcon-workers" "$2"
      colcon_workers="$2"
      shift 2
      ;;
    --cmake-jobs)
      [[ $# -ge 2 ]] || die "--cmake-jobs requires a value"
      require_positive_int "--cmake-jobs" "$2"
      cmake_build_jobs="$2"
      shift 2
      ;;
    --librealsense-jobs)
      [[ $# -ge 2 ]] || die "--librealsense-jobs requires a value"
      require_positive_int "--librealsense-jobs" "$2"
      librealsense_jobs="$2"
      shift 2
      ;;
    --with-desktop)
      install_ros_desktop="ON"
      shift
      ;;
    --headless)
      build_graphical="OFF"
      shift
      ;;
    --no-cuda)
      use_cuda="OFF"
      shift
      ;;
    --python)
      build_python="ON"
      shift
      ;;
    --skip-ros)
      install_ros="OFF"
      shift
      ;;
    --skip-librealsense)
      build_librealsense="OFF"
      shift
      ;;
    --skip-realsense-ros)
      build_realsense_ros="OFF"
      shift
      ;;
    --librealsense-ref|--ref)
      [[ $# -ge 2 ]] || die "$1 requires a value"
      librealsense_ref="$2"
      shift 2
      ;;
    --librealsense-source-dir|--source-dir)
      [[ $# -ge 2 ]] || die "$1 requires a value"
      librealsense_src_dir="$2"
      shift 2
      ;;
    --realsense-ros-ref)
      [[ $# -ge 2 ]] || die "--realsense-ros-ref requires a value"
      realsense_ros_ref="$2"
      shift 2
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

if [[ -n "${build_jobs}" ]]; then
  require_positive_int "BUILD_JOBS" "${build_jobs}"
fi
if [[ -n "${colcon_workers}" ]]; then
  require_positive_int "COLCON_WORKERS" "${colcon_workers}"
fi
if [[ -n "${cmake_build_jobs}" ]]; then
  require_positive_int "CMAKE_BUILD_JOBS" "${cmake_build_jobs}"
  export CMAKE_BUILD_PARALLEL_LEVEL="${cmake_build_jobs}"
  export MAKEFLAGS="-j${cmake_build_jobs} ${MAKEFLAGS:-}"
fi
if [[ -n "${librealsense_jobs}" ]]; then
  require_positive_int "LIBREALSENSE_JOBS" "${librealsense_jobs}"
fi

if [[ "${run_jetson_setup}" == "ON" && "${mode}" != "jetson" ]]; then
  warn "--setup-only is intended for Jetson setup; continuing with Jetson setup steps."
fi

ros_setup="/opt/ros/${ros_distro}/setup.bash"

check_jetson_platform() {
  log "Checking Jetson platform"
  echo "  uname -m: $(uname -m)"
  echo "  uname -r: $(uname -r)"

  if [[ "$(uname -m)" != "aarch64" ]]; then
    warn "Expected aarch64. Jetson setup is intended for Jetson boards."
  fi

  if [[ -r /proc/device-tree/model ]]; then
    local model
    model="$(tr -d '\0' < /proc/device-tree/model)"
    echo "  model: ${model}"
    if [[ "${model}" != *"Jetson"* ]]; then
      warn "Device model does not look like an NVIDIA Jetson board."
    fi
  else
    warn "Could not read /proc/device-tree/model"
  fi

  if [[ -r /etc/nv_tegra_release ]]; then
    log "Detected NVIDIA L4T information"
    cat /etc/nv_tegra_release
    if ! grep -q "R36" /etc/nv_tegra_release; then
      warn "This script is tuned for Jetson Linux R36 / JetPack 6 class systems."
    fi
  else
    warn "/etc/nv_tegra_release was not found."
  fi

  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    echo "  OS: ${PRETTY_NAME:-unknown}"
    if [[ "${VERSION_ID:-}" != "22.04" ]]; then
      warn "ROS 2 Humble on Jetson R36 normally uses Ubuntu 22.04."
    fi
  fi
}

setup_ros_apt_repo() {
  log "Configuring ROS 2 apt repository"
  apt_install software-properties-common curl gnupg lsb-release ca-certificates
  sudo add-apt-repository universe -y
  sudo mkdir -p /etc/apt/keyrings

  if [[ ! -f /etc/apt/keyrings/ros-archive-keyring.gpg ]]; then
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key |
      sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
  fi

  local codename
  codename="$(. /etc/os-release && printf '%s' "${UBUNTU_CODENAME:-${VERSION_CODENAME:-jammy}}")"
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${codename} main" |
    sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
}

install_system_packages() {
  local packages=(
    software-properties-common
    curl
    gnupg
    lsb-release
    ca-certificates
    build-essential
    cmake
    git
    wget
    pkg-config
    python3-dev
    python3-pip
    python3-setuptools
    python3-wheel
    python3-numpy
    python3-opencv
    python3-yaml
    libssl-dev
    libusb-1.0-0-dev
    libudev-dev
    libgtk-3-dev
    libglfw3-dev
    libgl1-mesa-dev
    libglu1-mesa-dev
    libxinerama-dev
    libxcursor-dev
    libxi-dev
    libxrandr-dev
    libudev1
    libtbb-dev
    libjpeg-dev
    libpng-dev
    libtiff-dev
    libdc1394-dev
    v4l-utils
    usbutils
  )

  if all_apt_packages_installed "${packages[@]}"; then
    log "System packages already installed; skipping apt install"
    return
  fi

  log "Installing missing system packages"
  sudo apt-get update

  apt_install software-properties-common curl gnupg lsb-release ca-certificates

  sudo add-apt-repository universe -y
  sudo apt-get update

  apt_install "${packages[@]}"
}

install_ros_packages() {
  if [[ "${install_ros}" != "ON" ]]; then
    return
  fi

  local ros_packages=(
    python3-rosdep
    python3-colcon-common-extensions
    "ros-${ros_distro}-ament-cmake"
    "ros-${ros_distro}-ament-lint-auto"
    "ros-${ros_distro}-ament-lint-common"
    "ros-${ros_distro}-cv-bridge"
    "ros-${ros_distro}-diagnostic-updater"
    "ros-${ros_distro}-geometry-msgs"
    "ros-${ros_distro}-image-transport"
    "ros-${ros_distro}-message-filters"
    "ros-${ros_distro}-nav-msgs"
    "ros-${ros_distro}-robot-state-publisher"
    "ros-${ros_distro}-rosidl-default-generators"
    "ros-${ros_distro}-rosidl-default-runtime"
    "ros-${ros_distro}-sensor-msgs"
    "ros-${ros_distro}-std-msgs"
    "ros-${ros_distro}-tf2"
    "ros-${ros_distro}-tf2-geometry-msgs"
    "ros-${ros_distro}-tf2-ros"
    "ros-${ros_distro}-xacro"
  )
  if [[ "${install_ros_desktop}" == "ON" ]]; then
    ros_packages=("ros-${ros_distro}-desktop" "${ros_packages[@]}")
  else
    ros_packages=("ros-${ros_distro}-ros-base" "${ros_packages[@]}")
  fi

  if [[ -f "${ros_setup}" ]] && all_apt_packages_installed "${ros_packages[@]}"; then
    log "ROS 2 ${ros_distro} packages already installed; skipping ROS apt install"
    if ! rosdep db >/dev/null 2>&1; then
      log "Initializing rosdep"
      sudo rosdep init 2>/dev/null || true
      rosdep update
    fi
    return
  fi

  setup_ros_apt_repo
  sudo apt-get update

  log "Installing ROS 2 ${ros_distro}"
  apt_install "${ros_packages[@]}"

  if ! rosdep db >/dev/null 2>&1; then
    log "Initializing rosdep"
    sudo rosdep init 2>/dev/null || true
    rosdep update
  fi
}

configure_cuda() {
  if [[ "${use_cuda}" != "ON" ]]; then
    return
  fi

  if command -v nvcc >/dev/null 2>&1; then
    log "CUDA compiler detected from PATH"
    nvcc --version | tail -n 4 || true
  elif [[ -x /usr/local/cuda/bin/nvcc ]]; then
    log "CUDA compiler detected at /usr/local/cuda/bin/nvcc"
    export PATH="/usr/local/cuda/bin:${PATH}"
    export LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}"
    nvcc --version | tail -n 4 || true
  else
    warn "nvcc was not found. Building librealsense without CUDA."
    use_cuda="OFF"
  fi
}

librealsense_installed() {
  command -v rs-enumerate-devices >/dev/null 2>&1 || return 1

  if command -v pkg-config >/dev/null 2>&1 && pkg-config --exists realsense2; then
    return 0
  fi

  if ldconfig -p 2>/dev/null | grep -q "librealsense2"; then
    return 0
  fi

  [[ -f /usr/local/lib/librealsense2.so || -f /usr/lib/aarch64-linux-gnu/librealsense2.so ]]
}

pyrealsense2_installed() {
  python3 -c 'import pyrealsense2' >/dev/null 2>&1
}

build_librealsense_from_source() {
  if [[ "${build_librealsense}" != "ON" ]]; then
    return
  fi

  if [[ "${clean}" != "ON" ]] && librealsense_installed; then
    if [[ "${build_python}" != "ON" ]] || pyrealsense2_installed; then
      log "librealsense is already installed; skipping librealsense build"
      return
    fi
    log "librealsense is installed, but pyrealsense2 is missing; rebuilding Python bindings"
  fi

  configure_cuda

  if dpkg -l 2>/dev/null | awk '{print $2}' | grep -qE '^librealsense2'; then
    warn "Existing librealsense2 Debian packages were found."
    warn "Mixed apt/source installations can cause conflicts."
    warn "If this build fails, remove old packages manually: sudo apt purge 'librealsense2*'"
  fi

  log "Preparing librealsense source: ${librealsense_src_dir}"
  if [[ ! -d "${librealsense_src_dir}/.git" ]]; then
    git clone https://github.com/realsenseai/librealsense.git "${librealsense_src_dir}"
  fi

  cd "${librealsense_src_dir}"
  git fetch --tags --prune
  git checkout "${librealsense_ref}"

  log "Installing RealSense udev rules"
  sudo ./scripts/setup_udev_rules.sh
  sudo udevadm control --reload-rules || true
  sudo udevadm trigger || true

  local build_dir="${librealsense_src_dir}/build"
  if [[ "${clean}" == "ON" ]]; then
    log "Removing previous librealsense build directory: ${build_dir}"
    rm -rf "${build_dir}"
  fi

  mkdir -p "${build_dir}"
  cd "${build_dir}"

  local cmake_args=(
    ..
    -DCMAKE_BUILD_TYPE=Release
    -DFORCE_RSUSB_BACKEND=ON
    -DBUILD_WITH_CUDA="${use_cuda}"
    -DBUILD_EXAMPLES=ON
    -DBUILD_GRAPHICAL_EXAMPLES="${build_graphical}"
    -DBUILD_TOOLS=ON
    -DBUILD_WITH_DDS=OFF
    -DCHECK_FOR_UPDATES=OFF
  )

  if [[ "${build_python}" == "ON" ]]; then
    cmake_args+=(
      -DBUILD_PYTHON_BINDINGS=ON
      -DPYTHON_EXECUTABLE="$(command -v python3)"
    )
  fi

  log "Configuring librealsense"
  cmake "${cmake_args[@]}"

  local jobs
  if [[ -n "${librealsense_jobs}" ]]; then
    jobs="${librealsense_jobs}"
  else
    jobs="$(nproc)"
    if [[ "${jobs}" -gt 1 ]]; then
      jobs="$((jobs - 1))"
    fi
  fi

  log "Building librealsense with ${jobs} job(s)"
  cmake --build . -j "${jobs}"

  log "Installing librealsense"
  sudo cmake --install .
  sudo ldconfig
}

source_ros() {
  [[ -f "${ros_setup}" ]] || die "ROS setup file not found: ${ros_setup}"
  set +u
  # shellcheck disable=SC1090
  source "${ros_setup}"
  set -u
}

build_realsense_ros_driver() {
  if [[ "${build_realsense_ros}" != "ON" ]]; then
    return
  fi

  source_ros
  if [[ -f "${workspace_dir}/install/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1091
    source "${workspace_dir}/install/setup.bash"
    set -u
  fi

  if [[ "${clean}" != "ON" ]] &&
    ros2 pkg prefix realsense2_camera >/dev/null 2>&1 &&
    ros2 pkg prefix realsense2_camera_msgs >/dev/null 2>&1; then
    log "realsense-ros is already installed; skipping realsense-ros build"
    return
  fi

  local src_dir="${workspace_dir}/src"
  local repo_dir="${src_dir}/realsense-ros"
  mkdir -p "${src_dir}"

  log "Preparing realsense-ros source: ${repo_dir}"
  if [[ ! -d "${repo_dir}/.git" ]]; then
    git clone "${realsense_ros_repo}" "${repo_dir}"
  fi

  cd "${repo_dir}"
  git fetch --tags --prune
  git checkout "${realsense_ros_ref}"

  if [[ "${clean}" == "ON" ]]; then
    log "Removing previous realsense-ros build/install/log for selected workspace"
    rm -rf "${workspace_dir}/build/realsense2_camera" \
      "${workspace_dir}/build/realsense2_camera_msgs" \
      "${workspace_dir}/install/realsense2_camera" \
      "${workspace_dir}/install/realsense2_camera_msgs"
  fi

  log "Installing rosdep dependencies for realsense-ros"
  cd "${workspace_dir}"
  rosdep install --from-paths src/realsense-ros -i -y -r --rosdistro "${ros_distro}" || true

  log "Building realsense-ros"
  local colcon_args=(
    --symlink-install
    --packages-up-to realsense2_camera
  )
  if [[ -n "${colcon_workers}" ]]; then
    colcon_args+=(--parallel-workers "${colcon_workers}")
  fi
  colcon build \
    "${colcon_args[@]}" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
}

ensure_workspace_link() {
  local expected_link="${workspace_dir}/src/autonomy"
  mkdir -p "${workspace_dir}/src"

  if [[ "${expected_link}" == "${package_dir}" ]]; then
    return
  fi

  if [[ ! -e "${expected_link}" ]]; then
    log "Linking autonomy into workspace"
    ln -s "${package_dir}" "${expected_link}"
  fi
}

ensure_core_interface_package() {
  local core_dir="${workspace_dir}/src/core"

  if [[ -f "${core_dir}/package.xml" ]]; then
    return
  fi

  log "Creating managed core interface package: ${core_dir}"
  mkdir -p "${core_dir}/msg"

  cat > "${core_dir}/package.xml" <<'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>core</name>
  <version>0.1.0</version>
  <description>Core ROS 2 interface package used by autonomy.</description>
  <maintainer email="todo@example.com">core maintainer</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

  cat > "${core_dir}/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.16)

project(core)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CommandFilter.msg"
  "msg/RobotReport.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
EOF

  cp "${package_dir}/msg/CommandFilter.msg" "${core_dir}/msg/CommandFilter.msg"
  cp "${package_dir}/msg/RobotReport.msg" "${core_dir}/msg/RobotReport.msg"
}

ensure_onnxruntime() {
  local ort_dir="$1"
  local ort_asset_arch="$2"
  local ort_version="${ONNXRUNTIME_VERSION:-1.18.1}"

  if [[ -f "${ort_dir}/include/onnxruntime_cxx_api.h" && -f "${ort_dir}/lib/libonnxruntime.so" ]]; then
    return
  fi

  mkdir -p "${ort_dir}" /tmp/autonomy_onnxruntime
  local archive="/tmp/autonomy_onnxruntime/onnxruntime-linux-${ort_asset_arch}-${ort_version}.tgz"
  local url="https://github.com/microsoft/onnxruntime/releases/download/v${ort_version}/onnxruntime-linux-${ort_asset_arch}-${ort_version}.tgz"

  log "Downloading ONNX Runtime ${ort_version} for ${target_arch}"
  curl -L -o "${archive}" "${url}"
  tar --no-same-owner -xzf "${archive}" --strip-components=1 -C "${ort_dir}"
  touch "${ort_dir}/COLCON_IGNORE"

  [[ -f "${ort_dir}/include/onnxruntime_cxx_api.h" ]] ||
    die "ONNX Runtime headers were not extracted into ${ort_dir}"
  [[ -f "${ort_dir}/lib/libonnxruntime.so" ]] ||
    die "ONNX Runtime library was not extracted into ${ort_dir}"
}

clean_third_party_build_artifacts() {
  log "Cleaning third-party source-tree build artifacts"
  rm -rf "${package_dir}/third_party/open_vins/build"
}

ensure_openvins() {
  local third_party_openvins="${package_dir}/third_party/open_vins"
  local workspace_openvins="${workspace_dir}/src/open_vins"

  if [[ ! -f "${third_party_openvins}/ov_msckf/package.xml" && -f "${workspace_openvins}/ov_msckf/package.xml" && ! -L "${workspace_openvins}" ]]; then
    log "Moving existing workspace OpenVINS into ${third_party_openvins}"
    mkdir -p "$(dirname "${third_party_openvins}")"
    mv "${workspace_openvins}" "${third_party_openvins}"
  fi

  if [[ ! -f "${third_party_openvins}/ov_msckf/package.xml" ]]; then
    if [[ "${SKIP_OPENVINS_CLONE:-0}" == "1" ]]; then
      die "OpenVINS not found at ${third_party_openvins}. Add it there or unset SKIP_OPENVINS_CLONE."
    fi

    local repo="${OPENVINS_REPO:-https://github.com/rpng/open_vins.git}"
    local version="${OPENVINS_VERSION:-master}"
    log "Cloning OpenVINS ${version} into ${third_party_openvins}"
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

build_autonomy_package() {
  if [[ "${build_autonomy}" != "ON" ]]; then
    return
  fi

  local host_arch
  host_arch="$(uname -m)"
  if [[ "${host_arch}" != "${target_arch}" && -z "${CMAKE_TOOLCHAIN_FILE:-}" ]]; then
    warn "host arch is ${host_arch}, target is ${target_arch}, and CMAKE_TOOLCHAIN_FILE is not set."
    warn "Continuing as a native build; set CMAKE_TOOLCHAIN_FILE for cross compilation."
  fi

  local ort_dir
  local ort_asset_arch
  case "${target_arch}" in
    x86_64)
      ort_dir="${package_dir}/third_party/onnxruntime"
      ort_asset_arch="x64"
      ;;
    aarch64)
      ort_dir="${package_dir}/third_party/onnxruntime-aarch64"
      ort_asset_arch="aarch64"
      ;;
    *)
      die "unsupported build target '${target_arch}'"
      ;;
  esac

  ensure_workspace_link
  ensure_core_interface_package
  ensure_onnxruntime "${ort_dir}" "${ort_asset_arch}"
  ensure_openvins
  sanitize_conda_build_env
  source_ros

  if [[ "${clean}" == "ON" ]]; then
    if [[ "${run_jetson_setup}" == "ON" ]]; then
      log "Cleaning autonomy/OpenVINS build artifacts without removing RealSense install"
      rm -rf "${workspace_dir}/build/autonomy" \
        "${workspace_dir}/build/ov_core" \
        "${workspace_dir}/build/ov_init" \
        "${workspace_dir}/build/ov_msckf" \
        "${workspace_dir}/install/autonomy" \
        "${workspace_dir}/install/ov_core" \
        "${workspace_dir}/install/ov_init" \
        "${workspace_dir}/install/ov_msckf"
    else
      log "Cleaning workspace build/install/log"
      rm -rf "${workspace_dir}/build" "${workspace_dir}/install" "${workspace_dir}/log"
    fi
    clean_third_party_build_artifacts
  fi

  local cmake_args=(
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

  log "Building autonomy for ${target_arch}"
  echo "Workspace: ${workspace_dir}"
  echo "ONNX Runtime: ${ort_dir}"
  echo "Python: ${PYTHON_EXECUTABLE:-/usr/bin/python3}"

  cd "${workspace_dir}"
  if [[ "${run_jetson_setup}" == "ON" || "${RUN_ROSDEP:-0}" == "1" ]]; then
    rosdep install --from-paths src -i -y -r --rosdistro "${ros_distro}" || true
  fi
  local colcon_args=(
    --symlink-install
    --packages-up-to ov_msckf autonomy
  )
  if [[ -n "${colcon_workers}" ]]; then
    colcon_args+=(--parallel-workers "${colcon_workers}")
  fi
  colcon build \
    "${colcon_args[@]}" \
    "${extra_colcon_args[@]}" \
    --cmake-args "${cmake_args[@]}"
}

run_jetson_setup_steps() {
  require_cmd sudo
  check_jetson_platform
  install_system_packages
  install_ros_packages
  build_librealsense_from_source
  build_realsense_ros_driver

  log "USB topology"
  lsusb || true
  lsusb -t || true

  log "Running RealSense device enumeration"
  if command -v rs-enumerate-devices >/dev/null 2>&1; then
    rs-enumerate-devices || true
  else
    warn "rs-enumerate-devices was not found in PATH."
  fi
}

print_summary() {
  if [[ "${mode}" != "jetson" && "${run_jetson_setup}" != "ON" ]]; then
    return
  fi

  cat <<EOF

Jetson flow finished.

Recommended checks:
  source /opt/ros/${ros_distro}/setup.bash
  source ${workspace_dir}/install/setup.bash
  rs-enumerate-devices
  ros2 launch realsense2_camera rs_launch.py

Build autonomy again:
  ${workspace_dir}/src/autonomy/scripts/build.sh jetson --skip-ros --skip-librealsense --skip-realsense-ros

If an older apt-based RealSense install conflicts:
  sudo apt purge 'librealsense2*'
  ${workspace_dir}/src/autonomy/scripts/build.sh jetson --clean
EOF

  if [[ "${build_python}" == "ON" ]]; then
    cat <<'EOF'

Python test:
  python3 - <<'PY'
import pyrealsense2 as rs
print(rs.__version__)
PY
EOF
  fi
}

main() {
  if [[ "${run_jetson_setup}" == "ON" ]]; then
    run_jetson_setup_steps
  fi

  build_autonomy_package
  print_summary
}

main "$@"
