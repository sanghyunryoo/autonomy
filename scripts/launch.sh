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

exec ros2 launch height_map_ros2 autonomy.launch.py "${launch_args[@]}"
