#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
ros_prefix="${ROS_INSTALL_PREFIX:-/opt/ros/${ros_distro}}"
build_dir="${AUTONOMY_DDS_ECHO_BUILD_DIR:-/tmp/autonomy_dds_echo_heightmap_linear_velocity}"

if [[ -f "${ros_prefix}/setup.bash" ]]; then
  # shellcheck source=/dev/null
  set +u
  source "${ros_prefix}/setup.bash"
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
project(autonomy_dds_echo_heightmap_linear_velocity LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(CycloneDDS REQUIRED CONFIG)

idlc_generate(
  TARGET echo_dds_types
  FILES
    "${repo_root}/resources/idl/HeightMap.idl"
    "${repo_root}/resources/idl/LinearVelocity.idl"
)

add_executable(dds_echo_heightmap_linear_velocity
  "${script_dir}/dds_echo_heightmap_linear_velocity.cpp"
)

target_link_libraries(dds_echo_heightmap_linear_velocity
  echo_dds_types
  CycloneDDS::ddsc
)
EOF_CMAKE

cmake -S "${build_dir}" -B "${build_dir}/build" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE:-RelWithDebInfo}"
cmake --build "${build_dir}/build" --target dds_echo_heightmap_linear_velocity -j "${CMAKE_BUILD_JOBS:-$(nproc)}"

exec "${build_dir}/build/dds_echo_heightmap_linear_velocity" "$@"
