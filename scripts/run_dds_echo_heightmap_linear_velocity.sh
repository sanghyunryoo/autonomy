#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
ros_prefix="${ROS_INSTALL_PREFIX:-/opt/ros/${ros_distro}}"
build_dir="${AUTONOMY_DDS_ECHO_BUILD_DIR:-/tmp/autonomy_dds_echo_heightmap_linear_velocity}"
autonomy_config="${AUTONOMY_CONFIG:-${repo_root}/resources/config/autonomy.yaml}"

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

configure_cyclonedds_uri() {
  local config_file="$1"
  [[ -f "${config_file}" ]] || return 0

  local generated_uri
  generated_uri="$(
    /usr/bin/python3 - "${config_file}" <<'PY'
import tempfile
import sys
import yaml
from pathlib import Path
from xml.sax.saxutils import escape

with open(sys.argv[1], "r", encoding="utf-8") as stream:
    data = yaml.safe_load(stream) or {}

dds = data.get("dds_network") or {}
if not isinstance(dds, dict):
    dds = {}

mode = str(dds.get("mode", "wireless")).strip().lower()
if mode not in ("wired", "ethernet"):
    sys.exit(0)

local_ip = str(dds.get("local_ip", "")).strip().split("/", 1)[0]
peer_ip = str(dds.get("peer_ip", dds.get("remote_ip", ""))).strip().split("/", 1)[0]
if not local_ip or not peer_ip:
    sys.exit(0)

allow_multicast = str(dds.get("allow_multicast", "false")).strip().lower() in ("true", "1", "yes", "on")
multicast = "true" if allow_multicast else "false"
multicast_recv = "preferred" if allow_multicast else "none"
peers = []
for address in (peer_ip, local_ip):
    if address and address not in peers:
        peers.append(address)
peer_xml = "\n".join(f'        <Peer Address="{escape(address)}" />' for address in peers)
xml = f"""<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface address="{escape(local_ip)}" priority="default" multicast="{multicast}" />
      </Interfaces>
      <AllowMulticast>{multicast}</AllowMulticast>
      <MulticastRecvNetworkInterfaceAddresses>{multicast_recv}</MulticastRecvNetworkInterfaceAddresses>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
{peer_xml}
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
"""
with tempfile.NamedTemporaryFile(
    mode="w",
    encoding="utf-8",
    prefix="autonomy_cyclonedds_echo_",
    suffix=".xml",
    delete=False,
) as config:
    config.write(xml)
    print(f"summary=wired local_ip={local_ip} peer_ip={peer_ip} self_peer={local_ip} config={sys.argv[1]}", file=sys.stderr)
    print(f"file://{Path(config.name)}")
PY
  )" 2>"${build_dir}/dds_config_summary"

  if [[ -n "${generated_uri}" ]]; then
    export CYCLONEDDS_URI="${generated_uri}"
    if [[ -s "${build_dir}/dds_config_summary" ]]; then
      export AUTONOMY_DDS_ECHO_CONFIG_SUMMARY
      AUTONOMY_DDS_ECHO_CONFIG_SUMMARY="$(sed -n 's/^summary=//p' "${build_dir}/dds_config_summary" | tail -n 1)"
    fi
    if [[ -z "${AUTONOMY_DDS_ECHO_CONFIG_SUMMARY:-}" ]]; then
      export AUTONOMY_DDS_ECHO_CONFIG_SUMMARY="wired config=${config_file}"
    fi
  else
    export AUTONOMY_DDS_ECHO_CONFIG_SUMMARY="default DDS config=${config_file}"
  fi
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

build_log="${build_dir}/build.log"
if ! cmake -S "${build_dir}" -B "${build_dir}/build" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE:-RelWithDebInfo}" >"${build_log}" 2>&1; then
  cat "${build_log}" >&2
  exit 1
fi
if ! cmake --build "${build_dir}/build" --target dds_echo_heightmap_linear_velocity -j "${CMAKE_BUILD_JOBS:-$(nproc)}" >>"${build_log}" 2>&1; then
  cat "${build_log}" >&2
  exit 1
fi

configure_cyclonedds_uri "${autonomy_config}"

exec "${build_dir}/build/dds_echo_heightmap_linear_velocity" "$@"
