#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
workspace_root="$(cd "${repo_root}/../.." && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
ros_prefix="${ROS_INSTALL_PREFIX:-/opt/ros/${ros_distro}}"
build_dir="${AUTONOMY_KEYBOARD_DDS_BUILD_DIR:-/tmp/autonomy_keyboard_filter_odom_dds_test}"

if [[ -f "${ros_prefix}/setup.bash" ]]; then
  # shellcheck source=/dev/null
  set +u
  source "${ros_prefix}/setup.bash"
  set -u
fi

if [[ -f "${workspace_root}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  set +u
  source "${workspace_root}/install/setup.bash"
  set -u
fi

command -v cmake >/dev/null 2>&1 || {
  echo "cmake was not found." >&2
  exit 1
}

command -v g++ >/dev/null 2>&1 || {
  echo "g++ was not found." >&2
  exit 1
}

mkdir -p "${build_dir}"

cat > "${build_dir}/CMakeLists.txt" <<EOF_CMAKE
cmake_minimum_required(VERSION 3.16)
project(autonomy_keyboard_filter_odom_dds_test LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(CycloneDDS REQUIRED CONFIG)
find_package(core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

idlc_generate(
  TARGET keyboard_dds_types
  FILES
    "${repo_root}/resources/idl/HeightMap.idl"
    "${repo_root}/resources/idl/LinearVelocity.idl"
)

add_executable(keyboard_filter_odom_dds_test
  "${script_dir}/keyboard_filter_odom_dds_test.cpp"
)

target_link_libraries(keyboard_filter_odom_dds_test
  keyboard_dds_types
  CycloneDDS::ddsc
)

ament_target_dependencies(keyboard_filter_odom_dds_test
  core
  geometry_msgs
  nav_msgs
  rclcpp
)
EOF_CMAKE

cmake -S "${build_dir}" -B "${build_dir}/build" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE:-RelWithDebInfo}"
cmake --build "${build_dir}/build" --target keyboard_filter_odom_dds_test -j "${CMAKE_BUILD_JOBS:-$(nproc)}"

exec "${build_dir}/build/keyboard_filter_odom_dds_test" "$@"
