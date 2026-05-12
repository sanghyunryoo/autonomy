from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = "height_map_ros2"
NODE_SHUTDOWN_TIMEOUT = "1.0"
QUIET_WORKER_ROS_ARGS = ["--ros-args", "--log-level", "fatal"]
STACK_MODE = "fsd"  # Keep every camera/node needed for runtime mode switching available.


def _parse_bool(value, default=True):
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in ("true", "1", "yes", "y", "on"):
        return True
    if normalized in ("false", "0", "no", "n", "off"):
        return False
    return bool(value)


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


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


def _node_params(data, node_name):
    return _sanitize_launch_parameters(
        data.get(node_name, {}).get("ros__parameters", {}).copy()
    )


def _binding_space(simulation):
    return "simulation" if _parse_bool(simulation, default=False) else "real"


def _camera_bindings(data, space):
    bindings = data.get("camera_bindings", [])
    if isinstance(bindings, dict):
        bindings = bindings.get(space, [])
    if not isinstance(bindings, list):
        raise RuntimeError(f"camera_bindings.{space} must be a list")
    return bindings


def _find_camera_binding(data, space, role):
    for binding in _camera_bindings(data, space):
        if not isinstance(binding, dict):
            continue
        if str(binding.get("role", "")).strip().lower() != role:
            continue
        if not _parse_bool(binding.get("enabled", True), default=True):
            raise RuntimeError(f"camera_bindings.{space}.{role} exists but is disabled")
        return binding
    raise RuntimeError(f"camera_bindings.{space} has no '{role}' camera binding")


def _operation_modes(data):
    return data.get("operation_modes") or {
        "drive": {"camera_roles": ["front", "rear"], "require_roles": []},
        "adas": {"camera_roles": ["front", "rear", "adas"], "require_roles": ["adas"]},
        "fsd": {"camera_roles": ["front", "rear", "adas"], "require_roles": ["adas"]},
    }


def _bindings_for_mode(data, space, mode_name):
    modes = _operation_modes(data)
    mode = modes.get(mode_name)
    if not isinstance(mode, dict):
        valid = ", ".join(sorted(str(key) for key in modes.keys()))
        raise RuntimeError(f"Unsupported operation mode '{mode_name}'. Valid modes: {valid}")

    allowed_roles = {str(role) for role in mode.get("camera_roles", [])}
    required_roles = {str(role) for role in mode.get("require_roles", [])}
    selected = []
    seen_roles = set()
    enabled_roles = set()

    for binding in _camera_bindings(data, space):
        if not isinstance(binding, dict):
            continue
        role = str(binding.get("role", ""))
        if allowed_roles and role not in allowed_roles:
            continue
        seen_roles.add(role)
        if _parse_bool(binding.get("enabled", True), default=True):
            enabled_roles.add(role)
            selected.append(binding)

    missing = sorted(role for role in required_roles if role not in seen_roles or role not in enabled_roles)
    if missing:
        raise RuntimeError(
            f"operation mode '{mode_name}' requires enabled camera role(s): "
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


def _frame_prefix(data):
    params = data.get("pointcloud_merge_node", {}).get("ros__parameters", {})
    target_frame = str(params.get("target_frame", "base_link"))
    return str(params.get("static_tf_frame_prefix", _frame_prefix_from_target(target_frame)))


def _merge_frame(frame, prefix):
    text = str(frame or "").strip().lstrip("/")
    if not text:
        return ""
    if "/" in text or not prefix:
        return text
    return prefix + text


def _topic_from_binding(binding, keys, default):
    for key in keys:
        value = binding.get(key)
        if value:
            return str(value)
    return default


def _camera_base_topic(binding, space):
    camera_name = str(binding.get("camera_name", "adas_camera")).strip("/")
    return f"/robot_4w4l/{camera_name}" if space == "simulation" else f"/{camera_name}"


def _merge_params(data, space):
    params = data.get("pointcloud_merge_node", {}).get("ros__parameters", {}).copy()
    prefix = str(params.pop("static_tf_frame_prefix", _frame_prefix(data)))

    camera_names = []
    cameras = {}
    for binding in _bindings_for_mode(data, space, STACK_MODE):
        role = str(binding["role"])
        camera_names.append(role)
        cameras[role] = {
            "enabled": True,
            "depth_topic": str(binding.get("depth_topic", f"/{role}/depth/image_rect")),
            "camera_info_topic": str(binding.get("camera_info_topic", f"/{role}/depth/camera_info")),
            "mount_frame": _merge_frame(binding.get("mount_frame", ""), prefix),
            "optical_frame": _merge_frame(binding.get("optical_frame", ""), prefix),
            "mount_to_optical_xyz": binding.get("mount_to_optical_xyz", [0.0, 0.0, 0.0]),
            "mount_to_optical_rpy": binding.get(
                "mount_to_optical_rpy",
                [-1.5707963267948966, 0.0, -1.5707963267948966],
            ),
        }

    params["camera_names"] = camera_names
    params["cameras"] = cameras
    params["static_tf_frame_prefix"] = prefix
    return params


def _adas_binding(data, space):
    return _find_camera_binding(data, space, "adas")


def _ai_topic_params(data, space):
    binding = _adas_binding(data, space)
    base = _camera_base_topic(binding, space)
    return {
        "image_topic": _topic_from_binding(
            binding,
            ("rgb_topic", "color_topic", "image_topic"),
            f"{base}/rgb/image_raw" if space == "simulation" else f"{base}/color/image_raw",
        ),
        "camera_info_topic": _topic_from_binding(
            binding,
            ("rgb_camera_info_topic", "color_camera_info_topic", "image_camera_info_topic"),
            f"{base}/rgb/camera_info" if space == "simulation" else f"{base}/color/camera_info",
        ),
        "depth_topic": _topic_from_binding(binding, ("depth_topic",), f"{base}/depth/image_rect_raw"),
    }


def _slam_topic_params(data, space):
    binding = _adas_binding(data, space)
    base = _camera_base_topic(binding, space)
    imu_topic = str(binding.get("imu_topic", ""))
    if not imu_topic:
        raise RuntimeError(f"camera_bindings.{space}.adas must define imu_topic for orbslam3_node")

    return {
        "left_image_topic": _topic_from_binding(
            binding, ("left_image_topic", "infra1_topic"), f"{base}/left/image_raw"
        ),
        "right_image_topic": _topic_from_binding(
            binding, ("right_image_topic", "infra2_topic"), f"{base}/right/image_raw"
        ),
        "imu_topic": imu_topic,
        "camera_frame": _merge_frame(binding.get("optical_frame", "A_camera_link"), _frame_prefix(data)),
    }


def _format_opencv_scalar(value):
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, str):
        escaped = value.replace("\\", "\\\\").replace('"', '\\"')
        return f'"{escaped}"'
    return str(value)


def _is_opencv_matrix(value):
    return (
        isinstance(value, dict)
        and {"rows", "cols", "dt", "data"}.issubset(value.keys())
        and isinstance(value.get("data"), list)
    )


def _write_orbslam_settings(settings_config, profile_name, profile):
    settings = profile.get("settings", {})
    if not isinstance(settings, dict):
        raise RuntimeError(f"orbslam3 profile '{profile_name}' must contain a settings dictionary")

    output_path = Path("/tmp") / f"height_map_ros2_orbslam3_{profile_name}.yaml"
    lines = [
        "%YAML:1.0",
        "",
        f"# Generated from {settings_config} profile '{profile_name}'.",
        "# Edit the source config, not this file.",
        "",
    ]

    for key, value in settings.items():
        if _is_opencv_matrix(value):
            data = ", ".join(_format_opencv_scalar(item) for item in value["data"])
            lines.extend([
                f"{key}: !!opencv-matrix",
                f"  rows: {int(value['rows'])}",
                f"  cols: {int(value['cols'])}",
                f"  dt: {value['dt']}",
                f"  data: [{data}]",
                "",
            ])
        else:
            lines.append(f"{key}: {_format_opencv_scalar(value)}")

    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return str(output_path)


def _orbslam_settings_params(config_file, data, space):
    params = _node_params(data, "orbslam3_node")
    settings_config = str(params.get("settings_config") or Path(config_file).with_name("orbslam3.yaml"))
    profiles = _load_yaml(settings_config).get("profiles", {})
    if not isinstance(profiles, dict):
        raise RuntimeError(f"{settings_config} must define a 'profiles' dictionary")

    profile_key = "settings_profile_simulation" if space == "simulation" else "settings_profile_real"
    profile_name = str(params.get(profile_key) or space)
    profile = profiles.get(profile_name)
    if not isinstance(profile, dict):
        valid = ", ".join(sorted(str(key) for key in profiles.keys()))
        raise RuntimeError(f"ORB-SLAM3 profile '{profile_name}' not found in {settings_config}. Valid profiles: {valid}")

    return {
        "settings_path": _write_orbslam_settings(settings_config, profile_name, profile),
        "sensor_type": str(profile.get("sensor_type", params.get("sensor_type", "stereo_inertial"))),
    }


def _orbslam_node_params(data):
    params = _node_params(data, "orbslam3_node")
    for key in (
        "settings_config",
        "settings_profile_real",
        "settings_profile_simulation",
        "settings_path_real",
        "settings_path_simulation",
        "settings_path",
    ):
        params.pop(key, None)
    return params


def _managed_nodes(simulation):
    base = ["pointcloud_merge_node", "elevation_mapping_node"]
    if not simulation:
        base = ["realsense_usb_mapper"] + base

    return {
        "managed_nodes.drive": base,
        "managed_nodes.adas": base + ["orbslam3_node", "rl_local_planner_node"],
        "managed_nodes.fsd": base + ["orbslam3_node", "rl_local_planner_node", "global_planner_node"],
    }


def _worker_node(executable, parameters, name=None, output="log"):
    return Node(
        package=PACKAGE_NAME,
        executable=executable,
        name=name or executable,
        output=output,
        arguments=QUIET_WORKER_ROS_ARGS if output != "screen" else [],
        sigterm_timeout=NODE_SHUTDOWN_TIMEOUT,
        sigkill_timeout=NODE_SHUTDOWN_TIMEOUT,
        parameters=parameters,
    )


def _make_stack(context, *args, **kwargs):
    config_file = LaunchConfiguration("autonomy_config").perform(context)
    simulation_text = LaunchConfiguration("simulation").perform(context)
    simulation = _parse_bool(simulation_text, default=False)
    map_dir = LaunchConfiguration("map_dir").perform(context)
    data = _load_yaml(config_file)
    space = _binding_space(simulation_text)
    use_sim_time = {"use_sim_time": LaunchConfiguration("simulation")}

    actions = [
        LogInfo(msg="Autonomy stack starting in IDLE. Use /autonomy_manager/set_mode to change modes."),
    ]

    if not simulation:
        actions.append(_worker_node(
            "realsense_usb_mapper.py",
            [{"mapping_file": config_file, "operation_mode": STACK_MODE}],
            name="realsense_usb_mapper",
        ))

    actions.extend([
        _worker_node(
            "pointcloud_merge_node",
            [_merge_params(data, space), use_sim_time],
        ),
        _worker_node(
            "elevation_mapping_node",
            [_node_params(data, "elevation_mapping_node"), use_sim_time, {"operation_mode": STACK_MODE}],
        ),
        _worker_node(
            "orbslam3_node",
            [
                _orbslam_node_params(data),
                _orbslam_settings_params(config_file, data, space),
                _slam_topic_params(data, space),
                use_sim_time,
            ],
        ),
        _worker_node(
            "rl_local_planner_node",
            [_node_params(data, "rl_local_planner_node"), use_sim_time, {"enabled": True}],
        ),
        _worker_node(
            "ai_detection_node",
            [_node_params(data, "ai_detection_node"), _ai_topic_params(data, space), use_sim_time],
        ),
        _worker_node(
            "global_planner_node",
            [_node_params(data, "global_planner_node"), use_sim_time, {"enabled": True}],
        ),
        _worker_node(
            "autonomy_manager_node",
            [
                _node_params(data, "autonomy_manager"),
                {
                    "startup_mode": "IDLE",
                    "speed_limit": 0.0,
                    "enable_ai": False,
                    "map_dir": map_dir,
                    **_managed_nodes(simulation),
                },
            ],
            name="autonomy_manager",
            output="screen",
        ),
    ])
    return actions


def generate_launch_description():
    package_share = Path(get_package_share_directory(PACKAGE_NAME))

    return LaunchDescription([
        DeclareLaunchArgument(
            "autonomy_config",
            default_value=str(package_share / "config" / "autonomy.yaml"),
            description="Single autonomy stack parameter file.",
        ),
        DeclareLaunchArgument("simulation", default_value="false"),
        DeclareLaunchArgument(
            "map_dir",
            default_value=str(package_share / "map"),
            description="Directory containing the global map required by FSD mode.",
        ),
        OpaqueFunction(function=_make_stack),
    ])
