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

command -v idlc >/dev/null 2>&1 || {
  echo "idlc was not found. Source ROS 2 or set ROS_INSTALL_PREFIX." >&2
  exit 1
}

command -v gcc >/dev/null 2>&1 || {
  echo "gcc was not found." >&2
  exit 1
}

command -v g++ >/dev/null 2>&1 || {
  echo "g++ was not found." >&2
  exit 1
}

pkg_config_path="${ros_prefix}/lib/x86_64-linux-gnu/pkgconfig:${ros_prefix}/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
export PKG_CONFIG_PATH="${pkg_config_path}"

if ! pkg-config --exists CycloneDDS; then
  echo "CycloneDDS.pc was not found under ${ros_prefix}." >&2
  exit 1
fi

mkdir -p "${build_dir}"

idlc -o "${build_dir}" "${repo_root}/resources/idl/HeightMap.idl"
idlc -o "${build_dir}" "${repo_root}/resources/idl/LinearVelocity.idl"

gcc -std=c11 -O2 -Wall -Wextra \
  -c "${build_dir}/HeightMap.c" \
  -I"${build_dir}" \
  $(pkg-config --cflags CycloneDDS) \
  -o "${build_dir}/HeightMap.o"

gcc -std=c11 -O2 -Wall -Wextra \
  -c "${build_dir}/LinearVelocity.c" \
  -I"${build_dir}" \
  $(pkg-config --cflags CycloneDDS) \
  -o "${build_dir}/LinearVelocity.o"

g++ -std=c++17 -O2 -Wall -Wextra -pedantic \
  "${script_dir}/dds_echo_heightmap_linear_velocity.cpp" \
  "${build_dir}/HeightMap.o" \
  "${build_dir}/LinearVelocity.o" \
  -I"${build_dir}" \
  $(pkg-config --cflags --libs CycloneDDS) \
  -Wl,-rpath,"${ros_prefix}/lib/x86_64-linux-gnu" \
  -Wl,-rpath,"${ros_prefix}/lib" \
  -o "${build_dir}/dds_echo_heightmap_linear_velocity"

exec "${build_dir}/dds_echo_heightmap_linear_velocity" "$@"
