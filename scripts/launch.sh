#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  launch.sh [simulation:=true|false] [extra ros2 launch args]

Examples:
  launch.sh
  launch.sh simulation:=true
  launch.sh autonomy_config:=/path/to/autonomy.yaml
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd "${script_dir}/.." && pwd)"

setup_candidates=(
  "$(cd "${package_dir}/../.." && pwd)/install/setup.bash"
  "$(cd "${script_dir}/../../.." && pwd)/setup.bash"
  "$(pwd)/install/setup.bash"
)

autonomy_config_arg=""
for arg in "$@"; do
  if [[ "${arg}" == autonomy_config:=* ]]; then
    autonomy_config_arg="${arg#autonomy_config:=}"
    break
  fi
done

setup_file=""
for candidate in "${setup_candidates[@]}"; do
  if [[ -f "${candidate}" ]]; then
    setup_file="${candidate}"
    break
  fi
done

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ -z "${setup_file}" ]]; then
  echo "error: ROS 2 workspace setup file not found." >&2
  echo "Checked:" >&2
  printf '  %s\n' "${setup_candidates[@]}" >&2
  echo "Run colcon build first." >&2
  exit 1
fi

set +u
source "${setup_file}"
set -u

resolve_autonomy_config() {
  if [[ -n "${autonomy_config_arg}" ]]; then
    if [[ "${autonomy_config_arg}" = /* ]]; then
      printf '%s\n' "${autonomy_config_arg}"
    else
      printf '%s\n' "$(pwd)/${autonomy_config_arg}"
    fi
    return
  fi

  if [[ -f "${package_dir}/resources/config/autonomy.yaml" ]]; then
    printf '%s\n' "${package_dir}/resources/config/autonomy.yaml"
    return
  fi

  local prefix
  prefix="$(ros2 pkg prefix autonomy)"
  printf '%s\n' "${prefix}/share/autonomy/resources/config/autonomy.yaml"
}

read_dds_network_config() {
  local config_file="$1"
  /usr/bin/python3 - "${config_file}" <<'PY'
import sys

import yaml

with open(sys.argv[1], "r", encoding="utf-8") as stream:
    data = yaml.safe_load(stream) or {}

dds = data.get("dds_network") or {}
if not isinstance(dds, dict):
    dds = {}

def value(name, default=""):
    item = dds.get(name, default)
    return "" if item is None else str(item).strip()

print(value("mode", "wireless").lower())
print(value("interface"))
print(value("local_ip"))
print(value("peer_ip", value("remote_ip")))
PY
}

read_dds_thread_priority() {
  local config_file="$1"
  /usr/bin/python3 - "${config_file}" <<'PY'
import sys

import yaml

with open(sys.argv[1], "r", encoding="utf-8") as stream:
    data = yaml.safe_load(stream) or {}

params = data.get("elevation_mapping_node", {}).get("ros__parameters", {})
dds = params.get("dds") or {}
height_map = dds.get("height_map") or {}
print(int(height_map.get("thread_priority", 0) or 0))
PY
}

configure_realtime_permissions() {
  local config_file="$1"
  [[ -f "${config_file}" ]] || return 0

  local priority
  priority="$(read_dds_thread_priority "${config_file}")"
  if [[ "${priority}" -le 0 ]]; then
    return 0
  fi

  if ! command -v setcap >/dev/null 2>&1 || ! command -v getcap >/dev/null 2>&1; then
    echo "warning: setcap/getcap not found; DDS realtime priority may fail." >&2
    return 0
  fi

  local prefix node_path
  prefix="$(ros2 pkg prefix autonomy)"
  node_path="${prefix}/lib/autonomy/elevation_mapping_node"
  if [[ ! -x "${node_path}" ]]; then
    echo "warning: elevation_mapping_node not found at ${node_path}; build autonomy first." >&2
    return 0
  fi

  if getcap "${node_path}" | grep -q "cap_sys_nice"; then
    return 0
  fi

  echo "Configuring realtime permission for DDS height map thread: ${node_path}"
  sudo setcap cap_sys_nice+ep "${node_path}"
}

interface_has_ip() {
  local iface="$1"
  local local_host="$2"
  ip -o -4 addr show dev "${iface}" | awk '{print $4}' | cut -d/ -f1 | grep -Fxq "${local_host}"
}

find_wired_interface() {
  local local_host="$1"
  local candidates=()
  local wired_candidates=()
  local iface

  for iface_path in /sys/class/net/*; do
    iface="$(basename "${iface_path}")"
    [[ "${iface}" == "lo" ]] && continue
    [[ -d "/sys/class/net/${iface}/wireless" ]] && continue
    [[ -e "/sys/class/net/${iface}/bridge" ]] && continue
    [[ -e "/sys/class/net/${iface}/tun_flags" ]] && continue
    if interface_has_ip "${iface}" "${local_host}"; then
      printf '%s\n' "${iface}"
      return
    fi
    wired_candidates+=("${iface}")
    if [[ -r "/sys/class/net/${iface}/carrier" ]] && [[ "$(cat "/sys/class/net/${iface}/carrier" 2>/dev/null || true)" == "1" ]]; then
      candidates+=("${iface}")
    fi
  done

  if [[ "${#candidates[@]}" -eq 1 ]]; then
    printf '%s\n' "${candidates[0]}"
    return
  fi

  if [[ "${#candidates[@]}" -eq 0 && "${#wired_candidates[@]}" -eq 1 ]]; then
    printf '%s\n' "${wired_candidates[0]}"
    return
  fi

  if [[ "${#candidates[@]}" -eq 0 ]]; then
    echo "error: no connected wired network interface found for DDS wired mode." >&2
  else
    echo "error: multiple connected wired interfaces found: ${candidates[*]}" >&2
  fi
  echo "Set dds_network.interface in the autonomy config." >&2
  return 1
}

configure_wired_dds_network() {
  local config_file
  config_file="$(resolve_autonomy_config)"
  if [[ ! -f "${config_file}" ]]; then
    echo "error: autonomy config not found: ${config_file}" >&2
    exit 1
  fi

  local dds_mode dds_interface dds_local_ip dds_peer_ip
  mapfile -t dds_config < <(read_dds_network_config "${config_file}")
  dds_mode="${dds_config[0]:-wireless}"
  dds_interface="${dds_config[1]:-}"
  dds_local_ip="${dds_config[2]:-}"
  dds_peer_ip="${dds_config[3]:-}"

  if [[ "${dds_mode}" != "wired" && "${dds_mode}" != "ethernet" ]]; then
    return
  fi

  if [[ -z "${dds_local_ip}" || -z "${dds_peer_ip}" ]]; then
    echo "error: dds_network.local_ip and dds_network.peer_ip are required for wired mode." >&2
    exit 1
  fi

  if ! command -v ip >/dev/null 2>&1; then
    echo "error: 'ip' command not found; install iproute2." >&2
    exit 1
  fi

  local local_host="${dds_local_ip%%/*}"
  if [[ "${dds_local_ip}" != */* ]]; then
    dds_local_ip="${dds_local_ip}/24"
  fi

  if [[ -z "${dds_interface}" ]]; then
    dds_interface="$(find_wired_interface "${local_host}")"
  fi

  if [[ ! -d "/sys/class/net/${dds_interface}" ]]; then
    echo "error: dds_network.interface does not exist: ${dds_interface}" >&2
    exit 1
  fi

  if interface_has_ip "${dds_interface}" "${local_host}"; then
    echo "DDS wired network already configured: ${dds_interface} ${dds_local_ip}"
    return
  fi

  echo "Configuring DDS wired network: ${dds_interface} ${dds_local_ip} peer=${dds_peer_ip}"
  sudo ip link set "${dds_interface}" up
  sudo ip addr add "${dds_local_ip}" dev "${dds_interface}"
}

configure_wired_dds_network

resolved_autonomy_config="$(resolve_autonomy_config)"
configure_realtime_permissions "${resolved_autonomy_config}"

has_simulation_arg=false
for arg in "$@"; do
  if [[ "${arg}" == simulation:=* ]]; then
    has_simulation_arg=true
    break
  fi
done

launch_args=("$@")
if [[ "${has_simulation_arg}" == false ]]; then
  launch_args=("simulation:=false" "${launch_args[@]}")
fi

if [[ -z "${autonomy_config_arg}" ]]; then
  launch_args=("autonomy_config:=${resolved_autonomy_config}" "${launch_args[@]}")
fi

source_launch_file="${package_dir}/launch/autonomy.launch.py"
if [[ -f "${source_launch_file}" ]]; then
  exec ros2 launch "${source_launch_file}" "${launch_args[@]}"
fi

exec ros2 launch autonomy autonomy.launch.py "${launch_args[@]}"
