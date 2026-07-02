#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  launch.sh [simulation:=true|false] [save_data:=true|false] [extra ros2 launch args]

Examples:
  launch.sh
  launch.sh simulation:=true
  launch.sh save_data:=true
  launch.sh save_data:=true data_dir:=/tmp/autonomy_run_001
  launch.sh save_data:=true data_csv:=/tmp/autonomy_run_001/height_map.csv
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
save_data_arg="false"
data_dir_arg=""
data_csv_arg=""
raw_launch_args=()
for arg in "$@"; do
  case "${arg}" in
    autonomy_config:=*)
      autonomy_config_arg="${arg#autonomy_config:=}"
      raw_launch_args+=("${arg}")
      ;;
    save_data:=*)
      save_data_arg="${arg#save_data:=}"
      ;;
    data_dir:=*)
      data_dir_arg="${arg#data_dir:=}"
      ;;
    data_csv:=*)
      data_csv_arg="${arg#data_csv:=}"
      ;;
    *)
      raw_launch_args+=("${arg}")
      ;;
  esac
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

read_livox_network_config() {
  local config_file="$1"
  /usr/bin/python3 - "${config_file}" <<'PY'
import sys

import yaml

with open(sys.argv[1], "r", encoding="utf-8") as stream:
    data = yaml.safe_load(stream) or {}

network = data.get("livox_network") or {}
if not isinstance(network, dict):
    network = {}

driver = (((data.get("livox_driver") or {}).get("ros__parameters")) or {})
lidar = (((data.get("lidar") or {}).get("ros__parameters")) or {})

def value(source, name, default=""):
    item = source.get(name, default)
    return "" if item is None else str(item).strip()

print(value(network, "interface"))
print(value(network, "host_ip", "192.168.1.50/24"))
print(value(network, "scan_cidr", "192.168.1.0/24"))
print(value(network, "lidar_ip", "auto"))
print(value(lidar, "driver_frame", value(driver, "frame_id", "livox_frame")))
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

  local prefix node_path capability_path
  prefix="$(ros2 pkg prefix autonomy)"
  node_path="${prefix}/lib/autonomy/elevation_mapping_node"
  if [[ ! -x "${node_path}" ]]; then
    echo "warning: elevation_mapping_node not found at ${node_path}; build autonomy first." >&2
    return 0
  fi
  if command -v readlink >/dev/null 2>&1; then
    capability_path="$(readlink -f "${node_path}")"
  elif command -v realpath >/dev/null 2>&1; then
    capability_path="$(realpath "${node_path}")"
  else
    capability_path="${node_path}"
  fi
  if [[ ! -f "${capability_path}" ]]; then
    echo "warning: elevation_mapping_node capability target is not a regular file: ${capability_path}" >&2
    return 0
  fi

  if getcap "${capability_path}" | grep -q "cap_sys_nice"; then
    return 0
  fi

  echo "Configuring realtime permission for DDS height map thread: ${capability_path}"
  sudo setcap cap_sys_nice+ep "${capability_path}"
}

clear_realtime_permissions_when_disabled() {
  local config_file="$1"
  [[ -f "${config_file}" ]] || return 0

  local priority
  priority="$(read_dds_thread_priority "${config_file}")"
  if [[ "${priority}" -gt 0 ]]; then
    return 0
  fi

  if ! command -v setcap >/dev/null 2>&1 || ! command -v getcap >/dev/null 2>&1; then
    return 0
  fi

  local prefix node_path capability_path
  prefix="$(ros2 pkg prefix autonomy)"
  node_path="${prefix}/lib/autonomy/elevation_mapping_node"
  [[ -e "${node_path}" ]] || return 0
  if command -v readlink >/dev/null 2>&1; then
    capability_path="$(readlink -f "${node_path}")"
  elif command -v realpath >/dev/null 2>&1; then
    capability_path="$(realpath "${node_path}")"
  else
    capability_path="${node_path}"
  fi
  [[ -f "${capability_path}" ]] || return 0
  if getcap "${capability_path}" | grep -q "cap_sys_nice"; then
    echo "Clearing realtime permission for DDS height map thread: ${capability_path}"
    sudo setcap -r "${capability_path}"
  fi
}

interface_has_ip() {
  local iface="$1"
  local local_host="$2"
  ip -o -4 addr show dev "${iface}" | awk '{print $4}' | cut -d/ -f1 | grep -Fxq "${local_host}"
}

interface_has_cidr() {
  local iface="$1"
  local cidr="$2"
  ip -o -4 addr show dev "${iface}" | awk '{print $4}' | grep -Fxq "${cidr}"
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

find_livox_interface() {
  local requested_iface="$1"
  local local_host="$2"
  if [[ -n "${requested_iface}" ]]; then
    printf '%s\n' "${requested_iface}"
    return
  fi
  find_wired_interface "${local_host}"
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

is_simulation_launch() {
  local arg
  for arg in "${raw_launch_args[@]}"; do
    case "${arg}" in
      simulation:=true|simulation:=1|simulation:=yes|simulation:=on)
        return 0
        ;;
    esac
  done
  return 1
}

probe_host() {
  local ip_addr="$1"
  ping -c 1 -W 1 "${ip_addr}" >/dev/null 2>&1
}

discover_livox_neighbor_ip() {
  local iface="$1"
  local host_ip="$2"
  local candidates=()
  local ip_addr state

  while read -r ip_addr state; do
    [[ -n "${ip_addr}" ]] || continue
    [[ "${ip_addr}" == "${host_ip}" ]] && continue
    case "${state}" in
      FAILED|INCOMPLETE|"")
        continue
        ;;
    esac
    candidates+=("${ip_addr}")
  done < <(
    ip neigh show dev "${iface}" 2>/dev/null |
      awk '
        $1 ~ /^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$/ {
          state=$NF
          has_lladdr=0
          for (i = 1; i <= NF; ++i) {
            if ($i == "lladdr") {
              has_lladdr=1
            }
          }
          if (has_lladdr) {
            print $1, state
          }
        }'
  )

  if [[ "${#candidates[@]}" -eq 1 ]]; then
    printf '%s\n' "${candidates[0]}"
    return 0
  fi

  return 1
}

scan_livox_candidates() {
  local iface="$1"
  local cidr="$2"
  local host_ip="$3"
  local candidate
  local seen=" "

  for candidate in 192.168.1.12 192.168.1.102 192.168.1.3 192.168.1.1; do
    [[ -n "${candidate}" ]] || continue
    [[ "${seen}" == *" ${candidate} "* ]] && continue
    printf '%s\n' "${candidate}"
    seen+=" ${candidate} "
  done

  ip neigh show dev "${iface}" 2>/dev/null |
    awk '{print $1}' |
    grep -E '^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$' |
    while read -r candidate; do
      [[ "${candidate}" == "${host_ip}" ]] && continue
      [[ "${seen}" == *" ${candidate} "* ]] && continue
      printf '%s\n' "${candidate}"
      seen+=" ${candidate} "
    done

  if command -v nmap >/dev/null 2>&1; then
    nmap -n -sn -e "${iface}" "${cidr}" 2>/dev/null |
      awk '/Nmap scan report for / {print $NF}' |
      while read -r candidate; do
        [[ "${candidate}" == "${host_ip}" ]] && continue
        [[ "${seen}" == *" ${candidate} "* ]] && continue
        printf '%s\n' "${candidate}"
        seen+=" ${candidate} "
      done
  fi
}

discover_livox_ip() {
  local iface="$1"
  local cidr="$2"
  local host_ip="$3"
  local explicit_ip="$4"

  if [[ -n "${explicit_ip}" && "${explicit_ip}" != "auto" ]]; then
    printf '%s\n' "${explicit_ip}"
    return
  fi

  local neighbor_ip
  if neighbor_ip="$(discover_livox_neighbor_ip "${iface}" "${host_ip}")"; then
    printf '%s\n' "${neighbor_ip}"
    return
  fi

  local candidate
  while read -r candidate; do
    [[ -n "${candidate}" ]] || continue
    if probe_host "${candidate}"; then
      printf '%s\n' "${candidate}"
      return
    fi
  done < <(scan_livox_candidates "${iface}" "${cidr}" "${host_ip}")

  return 1
}

write_livox_mid360_config() {
  local output_path="$1"
  local host_ip="$2"
  local lidar_ip="$3"
  local frame_id="$4"
  mkdir -p "$(dirname "${output_path}")"
  /usr/bin/python3 - "${output_path}" "${host_ip}" "${lidar_ip}" "${frame_id}" <<'PY'
import json
import sys

output_path, host_ip, lidar_ip, frame_id = sys.argv[1:5]
config = {
    "lidar_summary_info": {"lidar_type": 8},
    "MID360": {
        "lidar_net_info": {
            "cmd_data_port": 56100,
            "push_msg_port": 56200,
            "point_data_port": 56300,
            "imu_data_port": 56400,
            "log_data_port": 56500,
        },
        "host_net_info": {
            "cmd_data_ip": host_ip,
            "cmd_data_port": 56101,
            "push_msg_ip": host_ip,
            "push_msg_port": 56201,
            "point_data_ip": host_ip,
            "point_data_port": 56301,
            "imu_data_ip": host_ip,
            "imu_data_port": 56401,
            "log_data_ip": "",
            "log_data_port": 56501,
        },
    },
    "lidar_configs": [
        {
            "ip": lidar_ip,
            "pcl_data_type": 1,
            "pattern_mode": 0,
            "extrinsic_parameter": {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "x": 0,
                "y": 0,
                "z": 0,
            },
        }
    ],
}
with open(output_path, "w", encoding="utf-8") as stream:
    json.dump(config, stream, indent=2)
    stream.write("\n")
print(output_path)
PY
}

configure_livox_network() {
  if is_simulation_launch; then
    return
  fi

  local config_file
  config_file="$(resolve_autonomy_config)"
  [[ -f "${config_file}" ]] || return 0

  local iface host_cidr scan_cidr lidar_ip output_path frame_id
  mapfile -t livox_config < <(read_livox_network_config "${config_file}")
  iface="${livox_config[0]:-}"
  host_cidr="${livox_config[1]:-192.168.1.50/24}"
  scan_cidr="${livox_config[2]:-192.168.1.0/24}"
  lidar_ip="${livox_config[3]:-auto}"
  frame_id="${livox_config[4]:-livox_frame}"
  output_path="/tmp/autonomy_livox_mid360_config.json"

  if ! command -v ip >/dev/null 2>&1; then
    echo "warning: 'ip' command not found; skipping Livox network auto setup." >&2
    return
  fi

  local host_ip="${host_cidr%%/*}"
  [[ "${host_cidr}" == */* ]] || host_cidr="${host_cidr}/24"
  iface="$(find_livox_interface "${iface}" "${host_ip}")"
  if [[ ! -d "/sys/class/net/${iface}" ]]; then
    echo "warning: Livox network interface does not exist: ${iface}" >&2
    return
  fi

  sudo ip link set "${iface}" up
  if ! interface_has_cidr "${iface}" "${host_cidr}"; then
    if ! interface_has_ip "${iface}" "${host_ip}"; then
      echo "Configuring Livox MID360 host network: ${iface} ${host_cidr}"
      sudo ip addr add "${host_cidr}" dev "${iface}"
    fi
  fi

  local detected_lidar_ip
  if detected_lidar_ip="$(discover_livox_ip "${iface}" "${scan_cidr}" "${host_ip}" "${lidar_ip}")"; then
    lidar_ip="${detected_lidar_ip}"
  else
    echo "warning: could not auto-detect Livox MID360 IP on ${iface} ${scan_cidr}; skipping Livox auto config." >&2
    echo "warning: set livox_network.lidar_ip explicitly if auto detection cannot see the sensor." >&2
    return
  fi

  write_livox_mid360_config "${output_path}" "${host_ip}" "${lidar_ip}" "${frame_id}" >/dev/null
  export AUTONOMY_LIVOX_CONFIG_PATH="${output_path}"
  export AUTONOMY_LIVOX_HOST_IP="${host_ip}"
  export AUTONOMY_LIVOX_LIDAR_IP="${lidar_ip}"
  echo "Livox MID360 network: iface=${iface} host=${host_ip} lidar=${lidar_ip} config=${output_path}"
}

configure_wired_dds_network
configure_livox_network

resolved_autonomy_config="$(resolve_autonomy_config)"
clear_realtime_permissions_when_disabled "${resolved_autonomy_config}"

is_true() {
  case "${1,,}" in
    true|1|yes|y|on)
      return 0
      ;;
    false|0|no|n|off|"")
      return 1
      ;;
    *)
      echo "error: expected boolean value, got '${1}'" >&2
      exit 1
      ;;
  esac
}

absolute_path() {
  local path="$1"
  if [[ "${path}" = /* ]]; then
    printf '%s\n' "${path}"
  else
    printf '%s\n' "$(pwd)/${path}"
  fi
}

has_simulation_arg=false
for arg in "${raw_launch_args[@]}"; do
  if [[ "${arg}" == simulation:=* ]]; then
    has_simulation_arg=true
    break
  fi
done

launch_args=("${raw_launch_args[@]}")
if [[ "${has_simulation_arg}" == false ]]; then
  launch_args=("simulation:=false" "${launch_args[@]}")
fi

if [[ -z "${autonomy_config_arg}" ]]; then
  launch_args=("autonomy_config:=${resolved_autonomy_config}" "${launch_args[@]}")
fi

source_launch_file="${package_dir}/launch/autonomy.launch.py"

run_autonomy_launch() {
  if [[ -f "${source_launch_file}" ]]; then
    ros2 launch "${source_launch_file}" "${launch_args[@]}"
  else
    ros2 launch autonomy autonomy.launch.py "${launch_args[@]}"
  fi
}

cleanup_stale_autonomy_processes() {
  if is_simulation_launch; then
    return 0
  fi
  if ! command -v pgrep >/dev/null 2>&1 || ! command -v ps >/dev/null 2>&1; then
    return 0
  fi

  local patterns=(
    "realsense2_camera_node"
    "livox_ros_driver2_node"
    "drive_mapper_node"
    "pointcloud_merge_node"
    "elevation_mapping_node"
    "autonomy_manager_node"
    "point_lio_lidar_adapter_node"
    "point_lio_monitor_node"
    "global_planner_node"
    "local_planner_node"
    "pointlio_mapping"
  )
  local current_pgid
  current_pgid="$(ps -o pgid= -p "$$" | tr -d ' ' || true)"

  local pids=()
  local pattern pid command_line
  for pattern in "${patterns[@]}"; do
    while IFS= read -r line; do
      pid="${line%% *}"
      command_line="${line#* }"
      [[ -n "${pid}" && "${pid}" =~ ^[0-9]+$ ]] || continue
      [[ "${pid}" == "$$" || "${pid}" == "${BASHPID}" ]] && continue
      [[ "${command_line}" == *"scripts/launch.sh"* ]] && continue
      pids+=("${pid}")
    done < <(pgrep -af "${pattern}" 2>/dev/null || true)
  done

  if [[ "${#pids[@]}" -eq 0 ]]; then
    return 0
  fi

  local unique_pids=()
  local seen=" "
  for pid in "${pids[@]}"; do
    if [[ "${seen}" != *" ${pid} "* ]]; then
      unique_pids+=("${pid}")
      seen+=" ${pid} "
    fi
  done

  echo "Stopping stale autonomy/sensor processes before launch: ${unique_pids[*]}"
  local pgid target
  for pid in "${unique_pids[@]}"; do
    pgid="$(ps -o pgid= -p "${pid}" | tr -d ' ' || true)"
    if [[ -n "${pgid}" && "${pgid}" != "${current_pgid}" ]]; then
      target="-${pgid}"
    else
      target="${pid}"
    fi
    kill -INT "${target}" >/dev/null 2>&1 || kill -TERM "${target}" >/dev/null 2>&1 || true
  done

  local waited
  for waited in 1 2 3 4 5 6 7 8; do
    local any_alive=false
    for pid in "${unique_pids[@]}"; do
      if kill -0 "${pid}" >/dev/null 2>&1; then
        any_alive=true
        break
      fi
    done
    [[ "${any_alive}" == false ]] && return 0
    sleep 1
  done

  for pid in "${unique_pids[@]}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      pgid="$(ps -o pgid= -p "${pid}" | tr -d ' ' || true)"
      if [[ -n "${pgid}" && "${pgid}" != "${current_pgid}" ]]; then
        kill -KILL "-${pgid}" >/dev/null 2>&1 || true
      else
        kill -KILL "${pid}" >/dev/null 2>&1 || true
      fi
    fi
  done
}

read_expected_sensor_topics() {
  local config_file="$1"
  /usr/bin/python3 - "${config_file}" <<'PY'
import sys

import yaml

with open(sys.argv[1], "r", encoding="utf-8") as stream:
    data = yaml.safe_load(stream) or {}


def parse_bool(value, default=True):
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("true", "1", "yes", "y", "on"):
        return True
    if text in ("false", "0", "no", "n", "off"):
        return False
    return bool(value)


def robot_model():
    robot = data.get("robot") or {}
    return str(robot.get("model", "f4")).strip() or "f4"


def robot_namespace():
    robot = data.get("robot") or {}
    return str(robot.get("namespace", robot_model())).strip("/") or robot_model()


def topic_prefix():
    robot = data.get("robot") or {}
    return str(robot.get("topic_prefix", f"/{robot_namespace()}")).rstrip("/")


def camera_bindings():
    bindings = data.get("camera_bindings", [])
    if isinstance(bindings, dict):
        bindings = bindings.get("real", [])
    return bindings if isinstance(bindings, list) else []


def emit(label, topic, msg_type):
    topic = str(topic or "").strip()
    if topic:
        print(f"{label}|{topic}|{msg_type}")


for binding in camera_bindings():
    if not isinstance(binding, dict) or not parse_bool(binding.get("enabled", True), True):
        continue
    role = str(binding.get("role", "camera")).strip() or "camera"
    name = str(binding.get("camera_name", f"{role}_camera")).strip("/")
    base = f"{topic_prefix()}/{name}"
    emit(f"camera:{role}:depth", binding.get("depth_topic", f"{base}/depth/image_rect_raw"), "sensor_msgs/msg/Image")
    emit(
        f"camera:{role}:camera_info",
        binding.get("camera_info_topic", f"{base}/depth/camera_info"),
        "sensor_msgs/msg/CameraInfo",
    )
    model = str(binding.get("model", "")).strip().lower()
    has_imu = model.endswith("i") or parse_bool(binding.get("enable_imu", False), False)
    if has_imu:
        emit(f"camera:{role}:imu", binding.get("imu_topic", f"{base}/imu"), "sensor_msgs/msg/Imu")

params = ((data.get("pointcloud_merge_node") or {}).get("ros__parameters") or {})
lidar_names = params.get("lidar_names", [])
lidars = params.get("lidars", {})
if isinstance(lidar_names, list):
    if not isinstance(lidars, dict):
        lidars = {}
    for raw_name in lidar_names:
        name = str(raw_name).strip()
        if not name:
            continue
        lidar = lidars.get(name, {})
        if not isinstance(lidar, dict):
            lidar = {}
        if not parse_bool(lidar.get("enabled", True), True):
            continue
        emit(f"lidar:{name}:cloud", lidar.get("cloud_topic", f"{topic_prefix()}/{name}/lidar"), "sensor_msgs/msg/PointCloud2")
PY
}

wait_for_expected_sensors() {
  if is_simulation_launch; then
    return 0
  fi

  local config_file="$1"
  local timeout_sec="${AUTONOMY_SENSOR_TIMEOUT_SEC:-45}"
  local expected=()
  mapfile -t expected < <(read_expected_sensor_topics "${config_file}")
  if [[ "${#expected[@]}" -eq 0 ]]; then
    echo "No expected real sensor topics found in autonomy config; skipping sensor guard."
    return 0
  fi

  echo "Checking expected real sensors before keeping autonomy launch alive."
  echo "Sensor timeout: ${timeout_sec}s per topic (override with AUTONOMY_SENSOR_TIMEOUT_SEC)."

  local item label topic msg_type
  for item in "${expected[@]}"; do
    IFS='|' read -r label topic msg_type <<<"${item}"
    printf 'Waiting for %s: %s\n' "${label}" "${topic}"
  done

  /usr/bin/python3 - "${timeout_sec}" "${expected[@]}" <<'PY'
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

timeout_sec = float(sys.argv[1])
expected = []
for item in sys.argv[2:]:
    label, topic, msg_type = item.split("|", 2)
    expected.append((label, topic, msg_type))

rclpy.init()
node = rclpy.create_node("autonomy_expected_sensor_guard")
received = set()
subscriptions = []
qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

try:
    for label, topic, msg_type in expected:
        msg_cls = get_message(msg_type)

        def callback(_msg, sensor_label=label):
            if sensor_label not in received:
                print(f"Sensor OK: {sensor_label}", flush=True)
            received.add(sensor_label)

        subscriptions.append(node.create_subscription(msg_cls, topic, callback, qos))

    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and time.monotonic() < deadline and len(received) < len(expected):
        rclpy.spin_once(node, timeout_sec=0.1)

    missing = [(label, topic, msg_type) for label, topic, msg_type in expected if label not in received]
    if missing:
        print("ERROR: one or more expected sensors did not publish data:", file=sys.stderr)
        for label, topic, msg_type in missing:
            print(f"  - {label}: {topic} ({msg_type})", file=sys.stderr)
            print(f"    inspect: ros2 topic info -v {topic}", file=sys.stderr)
        sys.exit(1)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
}

launch_pid=""

start_autonomy_launch() {
  if [[ -f "${source_launch_file}" ]]; then
    setsid ros2 launch "${source_launch_file}" "${launch_args[@]}" &
  else
    setsid ros2 launch autonomy autonomy.launch.py "${launch_args[@]}" &
  fi
  launch_pid="$!"
}

stop_autonomy_launch() {
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" >/dev/null 2>&1; then
    echo "Stopping autonomy launch process group pid=${launch_pid}"
    kill -INT "-${launch_pid}" >/dev/null 2>&1 || kill -INT "${launch_pid}" >/dev/null 2>&1 || true
    local waited
    for waited in 1 2 3 4 5 6 7 8 9 10; do
      if ! kill -0 "${launch_pid}" >/dev/null 2>&1; then
        wait "${launch_pid}" >/dev/null 2>&1 || true
        return 0
      fi
      sleep 1
    done
    kill -TERM "-${launch_pid}" >/dev/null 2>&1 || kill -TERM "${launch_pid}" >/dev/null 2>&1 || true
    sleep 3
    if kill -0 "${launch_pid}" >/dev/null 2>&1; then
      kill -KILL "-${launch_pid}" >/dev/null 2>&1 || kill -KILL "${launch_pid}" >/dev/null 2>&1 || true
    fi
    wait "${launch_pid}" >/dev/null 2>&1 || true
  fi
}

run_autonomy_launch_guarded() {
  cleanup_stale_autonomy_processes
  start_autonomy_launch
  if ! wait_for_expected_sensors "${resolved_autonomy_config}"; then
    stop_autonomy_launch
    return 1
  fi
  wait "${launch_pid}"
}

data_logger_pid=""

cleanup_data_logger() {
  if [[ -n "${data_logger_pid}" ]] && kill -0 "${data_logger_pid}" >/dev/null 2>&1; then
    echo "Stopping height-map data logger pid=${data_logger_pid}"
    kill "${data_logger_pid}" >/dev/null 2>&1 || true
    wait "${data_logger_pid}" >/dev/null 2>&1 || true
  fi
}

start_data_logger() {
  local data_dir data_csv data_log
  if [[ -n "${data_csv_arg}" ]]; then
    data_csv="$(absolute_path "${data_csv_arg}")"
    data_dir="$(dirname "${data_csv}")"
  else
    if [[ -n "${data_dir_arg}" ]]; then
      data_dir="$(absolute_path "${data_dir_arg}")"
    else
      data_dir="${package_dir}/data/$(date +%Y%m%d_%H%M%S)"
    fi
    data_csv="${data_dir}/height_map.csv"
  fi

  mkdir -p "${data_dir}"
  data_log="${data_dir}/height_map_logger.log"

  echo "Saving DDS height_map CSV: ${data_csv}"
  echo "Height-map logger log: ${data_log}"
  AUTONOMY_CONFIG="${resolved_autonomy_config}" \
    "${script_dir}/run_dds_echo_heightmap_linear_velocity.sh" \
      --type height_map \
      --csv "${data_csv}" \
      --no-render \
      >"${data_log}" 2>&1 &
  data_logger_pid="$!"
}

if is_true "${save_data_arg}"; then
  trap 'stop_autonomy_launch; cleanup_data_logger' EXIT INT TERM
  start_data_logger
  run_autonomy_launch_guarded
  launch_status="$?"
  cleanup_data_logger
  exit "${launch_status}"
fi

trap stop_autonomy_launch EXIT INT TERM
run_autonomy_launch_guarded
