from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


NODE_SHUTDOWN_TIMEOUT = "1.0"
QUIET_WORKER_ROS_ARGS = ["--ros-args", "--log-level", "fatal"]


def _parse_bool(value, default=True):
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "1", "yes", "y", "on"):
            return True
        if normalized in ("false", "0", "no", "n", "off"):
            return False
    return bool(value)


def _load_config(config_file):
    with open(config_file, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _load_node_parameters(config_file, node_name):
    data = _load_config(config_file)
    return _sanitize_launch_parameters(
        data.get(node_name, {})
        .get("ros__parameters", {})
        .copy()
    )


def _sanitize_launch_parameters(value):
    if isinstance(value, dict):
        cleaned = {}
        for key, item in value.items():
            sanitized = _sanitize_launch_parameters(item)
            if sanitized is not None:
                cleaned[key] = sanitized
        return cleaned
    if isinstance(value, list):
        if not value:
            return None
        return [_sanitize_launch_parameters(item) for item in value]
    return value


def _binding_space_from_simulation(simulation):
    return "simulation" if _parse_bool(simulation, default=False) else "real"


def _select_camera_bindings(data, mode):
    bindings = data.get("camera_bindings", [])
    if isinstance(bindings, dict):
        bindings = bindings.get(mode, [])
    if not isinstance(bindings, list):
        raise ValueError(f"'camera_bindings.{mode}' must be a list")
    return bindings


def _find_camera_binding(data, binding_space, role):
    for binding in _select_camera_bindings(data, binding_space):
        if not isinstance(binding, dict):
            continue
        if str(binding.get("role", "")).strip().lower() != role:
            continue
        if not _parse_bool(binding.get("enabled", True), default=True):
            raise RuntimeError(
                f"camera_bindings.{binding_space}.{role} exists but is disabled"
            )
        return binding
    raise RuntimeError(f"camera_bindings.{binding_space} has no '{role}' camera binding")


def _topic_from_binding(binding, keys, default):
    for key in keys:
        value = binding.get(key)
        if value:
            return str(value)
    return default


def _camera_topic_base(binding, binding_space):
    camera_name = str(binding.get("camera_name", "adas_camera")).strip("/")
    if binding_space == "simulation":
        return f"/robot_4w4l/{camera_name}"
    return f"/{camera_name}"


def _default_operation_modes():
    return {
        "drive": {"camera_roles": ["front", "rear"], "require_roles": []},
        "adas": {"camera_roles": ["front", "rear", "adas"], "require_roles": ["adas"]},
        "fsd": {"camera_roles": ["front", "rear", "adas"], "require_roles": ["adas"]},
    }


def _select_operation_mode(data, operation_mode):
    mode_name = str(operation_mode or "drive").strip().lower()
    modes = data.get("operation_modes") or _default_operation_modes()
    if mode_name not in modes:
        valid = ", ".join(sorted(modes.keys()))
        raise ValueError(f"Unsupported operation_mode '{mode_name}'. Valid modes: {valid}")

    mode = modes[mode_name] or {}
    camera_roles = [str(role) for role in mode.get("camera_roles", [])]
    require_roles = [str(role) for role in mode.get("require_roles", [])]
    return mode_name, camera_roles, require_roles


def _filter_bindings_for_operation_mode(data, binding_space, operation_mode):
    mode_name, camera_roles, require_roles = _select_operation_mode(data, operation_mode)
    role_allowlist = set(camera_roles)
    selected = []
    role_to_binding = {}

    for binding in _select_camera_bindings(data, binding_space):
        if not isinstance(binding, dict):
            continue
        role = str(binding.get("role", ""))
        if camera_roles and role not in role_allowlist:
            continue
        role_to_binding[role] = binding
        if _parse_bool(binding.get("enabled", True), default=True):
            selected.append(binding)

    enabled_roles = {str(binding.get("role", "")) for binding in selected}
    missing = [
        role for role in require_roles
        if role not in role_to_binding or role not in enabled_roles
    ]
    if missing:
        raise RuntimeError(
            f"operation_mode '{mode_name}' requires enabled camera role(s): "
            + ", ".join(missing)
        )

    return selected


def _frame_prefix_from_target(target_frame):
    suffix = "base_link"
    if target_frame == suffix:
        return ""
    if target_frame.endswith("/" + suffix):
        return target_frame[: -len(suffix)]
    return ""


def _merge_frame(frame, frame_prefix):
    text = str(frame or "").strip().lstrip("/")
    if not text:
        return ""
    if "/" in text or not frame_prefix:
        return text
    return frame_prefix + text


def _load_operation_mode_from_config(config_file):
    data = _load_config(config_file)
    return str(
        data.get("elevation_mapping_node", {})
        .get("ros__parameters", {})
        .get("operation_mode", "drive")
    )


def _normalize_mode(mode):
    text = str(mode or "").strip().lower()
    return text or "idle"


def _resolve_operation_mode(context):
    operation_mode = LaunchConfiguration("operation_mode").perform(context).strip()
    if operation_mode:
        return _normalize_mode(operation_mode)
    config_file = LaunchConfiguration("autonomy_config").perform(context)
    return _normalize_mode(_load_operation_mode_from_config(config_file))


def _validate_launch_mode(mode):
    valid_modes = {"idle", "drive", "adas", "fsd", "mapping", "error", "estop"}
    if mode not in valid_modes:
        valid = ", ".join(sorted(valid_modes))
        raise RuntimeError(f"Unsupported operation_mode '{mode}'. Valid modes: {valid}")


def _map_dir_has_map(map_dir):
    path = Path(map_dir).expanduser()
    if not path.exists() or not path.is_dir():
        return False
    return any(child.is_file() and not child.name.startswith(".") for child in path.iterdir())


def _require_map_for_fsd(map_dir):
    if not _map_dir_has_map(map_dir):
        raise RuntimeError(
            "FSD mode requires at least one map file in map_dir. "
            f"Checked: {Path(map_dir).expanduser()}"
        )


def _load_merge_parameters(config_file, simulation, operation_mode):
    data = _load_config(config_file)
    binding_space = _binding_space_from_simulation(simulation)
    node_params = (
        data.get("pointcloud_merge_node", {})
        .get("ros__parameters", {})
        .copy()
    )
    target_frame = str(node_params.get("target_frame", "base_link"))
    frame_prefix = str(
        node_params.pop("static_tf_frame_prefix", _frame_prefix_from_target(target_frame))
    )

    camera_names = []
    cameras = {}
    for binding in _filter_bindings_for_operation_mode(data, binding_space, operation_mode):
        role = str(binding["role"])
        camera_names.append(role)
        cameras[role] = {
            "enabled": True,
            "depth_topic": str(binding.get("depth_topic", f"/{role}/depth/image_rect")),
            "camera_info_topic": str(
                binding.get("camera_info_topic", f"/{role}/depth/camera_info")
            ),
            "mount_frame": _merge_frame(binding.get("mount_frame", ""), frame_prefix),
            "optical_frame": _merge_frame(binding.get("optical_frame", ""), frame_prefix),
            "mount_to_optical_xyz": binding.get("mount_to_optical_xyz", [0.0, 0.0, 0.0]),
            "mount_to_optical_rpy": binding.get(
                "mount_to_optical_rpy",
                [-1.5707963267948966, 0.0, -1.5707963267948966],
            ),
        }

    node_params["camera_names"] = camera_names
    node_params["cameras"] = cameras
    node_params["static_tf_frame_prefix"] = frame_prefix
    return node_params


def _frame_prefix_from_config(data):
    node_params = data.get("pointcloud_merge_node", {}).get("ros__parameters", {})
    target_frame = str(node_params.get("target_frame", "base_link"))
    return str(node_params.get("static_tf_frame_prefix", _frame_prefix_from_target(target_frame)))


def _load_adas_binding_parameters(config_file, simulation):
    data = _load_config(config_file)
    binding_space = _binding_space_from_simulation(simulation)
    binding = _find_camera_binding(data, binding_space, "adas")
    return data, binding_space, binding


def _load_slam_topic_parameters(config_file, simulation):
    data, binding_space, binding = _load_adas_binding_parameters(config_file, simulation)
    base = _camera_topic_base(binding, binding_space)
    frame_prefix = _frame_prefix_from_config(data)
    imu_topic = str(binding.get("imu_topic", ""))
    if not imu_topic:
        raise RuntimeError(
            f"camera_bindings.{binding_space}.adas must define imu_topic for orbslam3_node"
        )
    
    return {
        "left_image_topic": _topic_from_binding(
            binding,
            ("left_image_topic", "infra1_topic"),
            f"{base}/left/image_raw",
        ),
        "right_image_topic": _topic_from_binding(
            binding,
            ("right_image_topic", "infra2_topic"),
            f"{base}/right/image_raw",
        ),
        "imu_topic": imu_topic,
        "camera_frame": _merge_frame(binding.get("optical_frame", "A_camera_link"), frame_prefix),
    }


def _load_ai_topic_parameters(config_file, simulation):
    _data, binding_space, binding = _load_adas_binding_parameters(config_file, simulation)
    base = _camera_topic_base(binding, binding_space)
    return {
        "image_topic": _topic_from_binding(
            binding,
            ("rgb_topic", "color_topic", "image_topic"),
            f"{base}/rgb/image_raw" if binding_space == "simulation" else f"{base}/color/image_raw",
        ),
        "camera_info_topic": _topic_from_binding(
            binding,
            ("rgb_camera_info_topic", "color_camera_info_topic", "image_camera_info_topic"),
            f"{base}/rgb/camera_info" if binding_space == "simulation" else f"{base}/color/camera_info",
        ),
        "depth_topic": _topic_from_binding(
            binding,
            ("depth_topic",),
            f"{base}/depth/image_rect_raw",
        ),
    }


def _make_usb_mapper_node(context, *args, **kwargs):
    config_file = LaunchConfiguration("autonomy_config").perform(context)
    return [
        Node(
            package="height_map_ros2",
            executable="realsense_usb_mapper.py",
            name="realsense_usb_mapper",
            output="log",
            arguments=QUIET_WORKER_ROS_ARGS,
            sigterm_timeout=NODE_SHUTDOWN_TIMEOUT,
            sigkill_timeout=NODE_SHUTDOWN_TIMEOUT,
            parameters=[
                {
                    "mapping_file": config_file,
                    "operation_mode": _resolve_operation_mode(context),
                },
            ],
        )
    ]


def _make_usb_mapper_node_action(context):
    return _make_usb_mapper_node(context)[0]


def _make_merge_node_action(context):
    config_file = LaunchConfiguration("autonomy_config").perform(context)
    simulation = LaunchConfiguration("simulation").perform(context)
    operation_mode = _resolve_operation_mode(context)
    merge_parameters = _load_merge_parameters(config_file, simulation, operation_mode)

    return Node(
        package="height_map_ros2",
        executable="pointcloud_merge_node",
        name="pointcloud_merge_node",
        output="log",
        arguments=QUIET_WORKER_ROS_ARGS,
        sigterm_timeout=NODE_SHUTDOWN_TIMEOUT,
        sigkill_timeout=NODE_SHUTDOWN_TIMEOUT,
        parameters=[
            merge_parameters,
            {"use_sim_time": LaunchConfiguration("simulation")},
        ],
    )


def _make_elevation_node_action(context):
    config_file = LaunchConfiguration("autonomy_config").perform(context)
    return Node(
        package="height_map_ros2",
        executable="elevation_mapping_node",
        name="elevation_mapping_node",
        output="log",
        arguments=QUIET_WORKER_ROS_ARGS,
        sigterm_timeout=NODE_SHUTDOWN_TIMEOUT,
        sigkill_timeout=NODE_SHUTDOWN_TIMEOUT,
        parameters=[
            _load_node_parameters(config_file, "elevation_mapping_node"),
            {"use_sim_time": LaunchConfiguration("simulation")},
            {"operation_mode": _resolve_operation_mode(context)},
        ],
    )


def _make_ai_detection_node_action(context):
    config_file = LaunchConfiguration("autonomy_config").perform(context)
    simulation = LaunchConfiguration("simulation").perform(context)
    ai_topic_parameters = _load_ai_topic_parameters(config_file, simulation)

    return Node(
        package="height_map_ros2",
        executable="ai_detection_node",
        name="ai_detection_node",
        output="log",
        arguments=QUIET_WORKER_ROS_ARGS,
        sigterm_timeout=NODE_SHUTDOWN_TIMEOUT,
        sigkill_timeout=NODE_SHUTDOWN_TIMEOUT,
        parameters=[
            _load_node_parameters(config_file, "ai_detection_node"),
            ai_topic_parameters,
            {"use_sim_time": LaunchConfiguration("simulation")},
        ],
    )


def _make_slam_node_action(context):
    config_file = LaunchConfiguration("autonomy_config").perform(context)
    simulation = LaunchConfiguration("simulation").perform(context)
    slam_topic_parameters = _load_slam_topic_parameters(config_file, simulation)

    return Node(
        package="height_map_ros2",
        executable="orbslam3_node",
        name="orbslam3_node",
        output="log",
        arguments=QUIET_WORKER_ROS_ARGS,
        sigterm_timeout=NODE_SHUTDOWN_TIMEOUT,
        sigkill_timeout=NODE_SHUTDOWN_TIMEOUT,
        parameters=[
            _load_node_parameters(config_file, "orbslam3_node"),
            slam_topic_parameters,
            {"use_sim_time": LaunchConfiguration("simulation")},
        ],
    )


def _make_rl_local_planner_node_action(context):
    config_file = LaunchConfiguration("autonomy_config").perform(context)
    return Node(
        package="height_map_ros2",
        executable="rl_local_planner_node",
        name="rl_local_planner_node",
        output="log",
        arguments=QUIET_WORKER_ROS_ARGS,
        sigterm_timeout=NODE_SHUTDOWN_TIMEOUT,
        sigkill_timeout=NODE_SHUTDOWN_TIMEOUT,
        parameters=[
            _load_node_parameters(config_file, "rl_local_planner_node"),
            {"use_sim_time": LaunchConfiguration("simulation")},
            {"enabled": True},
        ],
    )


def _make_global_planner_node_action(context):
    config_file = LaunchConfiguration("autonomy_config").perform(context)
    return Node(
        package="height_map_ros2",
        executable="global_planner_node",
        name="global_planner_node",
        output="log",
        arguments=QUIET_WORKER_ROS_ARGS,
        sigterm_timeout=NODE_SHUTDOWN_TIMEOUT,
        sigkill_timeout=NODE_SHUTDOWN_TIMEOUT,
        parameters=[
            _load_node_parameters(config_file, "global_planner_node"),
            {"use_sim_time": LaunchConfiguration("simulation")},
            {"enabled": True},
        ],
    )


def _make_static_stack(context, *args, **kwargs):
    simulation = _parse_bool(LaunchConfiguration("simulation").perform(context), default=False)
    map_dir = LaunchConfiguration("map_dir").perform(context)

    actions = [
        LogInfo(msg="Autonomy stack starting in IDLE. Use /autonomy_manager/set_mode to change modes."),
    ]

    if not simulation:
        # Keep all camera roles available so runtime mode changes do not need a relaunch.
        config_file = LaunchConfiguration("autonomy_config").perform(context)
        actions.append(Node(
            package="height_map_ros2",
            executable="realsense_usb_mapper.py",
            name="realsense_usb_mapper",
            output="log",
            arguments=QUIET_WORKER_ROS_ARGS,
            sigterm_timeout=NODE_SHUTDOWN_TIMEOUT,
            sigkill_timeout=NODE_SHUTDOWN_TIMEOUT,
            parameters=[
                {
                    "mapping_file": config_file,
                    "operation_mode": "fsd",
                },
            ],
        ))

    config_file = LaunchConfiguration("autonomy_config").perform(context)
    simulation_text = LaunchConfiguration("simulation").perform(context)
    merge_parameters = _load_merge_parameters(config_file, simulation_text, "fsd")

    actions.extend([
        Node(
            package="height_map_ros2",
            executable="pointcloud_merge_node",
            name="pointcloud_merge_node",
           output="log",
            arguments=QUIET_WORKER_ROS_ARGS,
            sigterm_timeout=NODE_SHUTDOWN_TIMEOUT,
            sigkill_timeout=NODE_SHUTDOWN_TIMEOUT,
            parameters=[
                merge_parameters,
                {"use_sim_time": LaunchConfiguration("simulation")},
            ],
        ),
        Node(
            package="height_map_ros2",
            executable="elevation_mapping_node",
            name="elevation_mapping_node",
            output="log",
            arguments=QUIET_WORKER_ROS_ARGS,
            sigterm_timeout=NODE_SHUTDOWN_TIMEOUT,
            sigkill_timeout=NODE_SHUTDOWN_TIMEOUT,
            parameters=[
                _load_node_parameters(config_file, "elevation_mapping_node"),
                {"use_sim_time": LaunchConfiguration("simulation")},
                {"operation_mode": "fsd"},
            ],
        ),
        _make_slam_node_action(context),
        _make_rl_local_planner_node_action(context),
        _make_ai_detection_node_action(context),
        _make_global_planner_node_action(context),
        Node(
            package="height_map_ros2",
            executable="autonomy_manager_node",
            name="autonomy_manager",
            output="screen",
            sigterm_timeout=NODE_SHUTDOWN_TIMEOUT,
            sigkill_timeout=NODE_SHUTDOWN_TIMEOUT,
            parameters=[
                _load_node_parameters(config_file, "autonomy_manager"),
                {
                    "startup_mode": "IDLE",
                    "speed_limit": 0.0,
                    "enable_ai": False,
                    "map_dir": map_dir,
                    "managed_nodes.drive": [
                        "realsense_usb_mapper",
                        "pointcloud_merge_node",
                        "elevation_mapping_node",
                    ] if not simulation else [
                        "pointcloud_merge_node",
                        "elevation_mapping_node",
                    ],
                    "managed_nodes.adas": [
                        "realsense_usb_mapper",
                        "pointcloud_merge_node",
                        "elevation_mapping_node",
                        "orbslam3_node",
                        "rl_local_planner_node",
                    ] if not simulation else [
                        "pointcloud_merge_node",
                        "elevation_mapping_node",
                        "orbslam3_node",
                        "rl_local_planner_node",
                    ],
                    "managed_nodes.fsd": [
                        "realsense_usb_mapper",
                        "pointcloud_merge_node",
                        "elevation_mapping_node",
                        "orbslam3_node",
                        "rl_local_planner_node",
                        "global_planner_node",
                    ] if not simulation else [
                        "pointcloud_merge_node",
                        "elevation_mapping_node",
                        "orbslam3_node",
                        "rl_local_planner_node",
                        "global_planner_node",
                    ],
                },
            ],
        ),
    ])
    return actions


def generate_launch_description():
    package_share = Path(get_package_share_directory("height_map_ros2"))
    autonomy_config = package_share / "config" / "autonomy.yaml"
    default_map_dir = package_share / "map"

    autonomy_config_arg = DeclareLaunchArgument(
        "autonomy_config",
        default_value=str(autonomy_config),
        description="Single autonomy stack parameter file.",
    )
    simulation_arg = DeclareLaunchArgument("simulation", default_value="false")
    map_dir_arg = DeclareLaunchArgument(
        "map_dir",
        default_value=str(default_map_dir),
        description="Directory containing the global map required by FSD mode.",
    )

    return LaunchDescription(
        [
            autonomy_config_arg,
            simulation_arg,
            map_dir_arg,
            OpaqueFunction(function=_make_static_stack),
        ]
    )
