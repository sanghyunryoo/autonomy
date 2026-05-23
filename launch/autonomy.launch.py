from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = "autonomy"
NODE_SIGTERM_TIMEOUT = "0.5"
NODE_SIGKILL_TIMEOUT = "1.0"
QUIET_WORKER_ROS_ARGS = ["--ros-args", "--log-level", "fatal"]
STACK_MODE = "fsd"  # Keep every camera/node needed for runtime mode switching available.


def _package_share_path():
    return Path(get_package_share_directory(PACKAGE_NAME))


def _resolve_package_path(value):
    if not value:
        return value
    path = Path(str(value)).expanduser()
    if path.is_absolute():
        return str(path)

    text = str(value)
    source_prefix = f"src/{PACKAGE_NAME}/"
    if text.startswith(source_prefix):
        return str(_package_share_path() / text[len(source_prefix):])
    if text.startswith("resources/"):
        return str(_package_share_path() / text)
    return text


def _resolve_node_paths(params, keys):
    if not isinstance(params, dict):
        return params
    resolved = params.copy()
    for key in keys:
        if key in resolved:
            resolved[key] = _resolve_package_path(resolved[key])
    return resolved


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


def _external_launch(data, node_name, default_package, default_launch_file, parameters=None):
    params = _node_params(data, node_name) or {}
    if not _parse_bool(params.get("enabled", True), default=True):
        return []
    package = str(params.get("package", default_package))
    launch_file = str(params.get("launch_file", default_launch_file))
    try:
        package_share = Path(get_package_share_directory(package))
    except Exception as exc:
        return [LogInfo(msg=f"{node_name}: package '{package}' not found, skipping external launch ({exc})")]
    launch_path = package_share / "launch" / launch_file
    if not launch_path.exists():
        return [LogInfo(msg=f"{node_name}: launch file not found: {launch_path}")]

    launch_arguments = params.get("launch_arguments", {})
    if not isinstance(launch_arguments, dict):
        launch_arguments = {}
    if parameters:
        launch_arguments = {**launch_arguments, **parameters}
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_path)),
            launch_arguments={str(k): str(v) for k, v in launch_arguments.items()}.items(),
        )
    ]


def _point_lio_launch_arguments(data, simulation):
    params = _node_params(data, "point_lio") or {}
    launch_arguments = params.get("launch_arguments", {})
    if not isinstance(launch_arguments, dict):
        launch_arguments = {}
    launch_arguments = launch_arguments.copy()

    sim_lidar_topic = params.get("lidar_topic_simulation")
    real_lidar_topic = params.get("lidar_topic_real")
    if simulation and sim_lidar_topic:
        launch_arguments["lid_topic"] = sim_lidar_topic
    elif not simulation and real_lidar_topic:
        launch_arguments["lid_topic"] = real_lidar_topic
    return launch_arguments


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


def _enabled_camera_binding(data, space, role):
    for binding in _camera_bindings(data, space):
        if not isinstance(binding, dict):
            continue
        if str(binding.get("role", "")).strip().lower() != role:
            continue
        return binding if _parse_bool(binding.get("enabled", True), default=True) else None
    return None


def _operation_modes(data):
    return data.get("operation_modes") or {
        "drive": {"camera_roles": ["front", "rear"], "require_roles": []},
        "adas": {"camera_roles": ["front", "rear", "adas"], "require_roles": ["adas"]},
        "fsd": {"camera_roles": ["front", "rear", "adas"], "require_roles": ["adas"]},
        "tracking": {"camera_roles": ["front", "rear", "adas"], "require_roles": ["adas"]},
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
    for binding in _camera_bindings(data, space):
        if not isinstance(binding, dict):
            continue
        if not _parse_bool(binding.get("enabled", True), default=True):
            continue
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
    if "urdf_path" in params:
        params["urdf_path"] = _resolve_package_path(params["urdf_path"])
    return params


def _adas_binding(data, space):
    return _find_camera_binding(data, space, "adas")


def _adas_enabled(data, space):
    return _enabled_camera_binding(data, space, "adas") is not None


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


def _format_opencv_scalar(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        escaped = value.replace("\\", "\\\\").replace('"', '\\"')
        return f'"{escaped}"'
    return str(value)


def _format_opencv_value(value, indent=0):
    pad = " " * indent
    if isinstance(value, dict):
        lines = []
        for key, item in value.items():
            if isinstance(item, (dict, list)):
                lines.append(f"{pad}{key}:")
                lines.extend(_format_opencv_value(item, indent + 2))
            else:
                lines.append(f"{pad}{key}: {_format_opencv_scalar(item)}")
        return lines
    if isinstance(value, list):
        if value and all(isinstance(item, list) for item in value):
            return [
                f"{pad}- [{', '.join(_format_opencv_scalar(elem) for elem in row)}]"
                for row in value
            ]
        return [f"{pad}[{', '.join(_format_opencv_scalar(item) for item in value)}]"]
    return [f"{pad}{_format_opencv_scalar(value)}"]


def _write_openvins_yaml(path, data):
    lines = ["%YAML:1.0", ""]
    lines.extend(_format_opencv_value(data))
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
    return str(path)


def _openvins_profile(config_file, data, space):
    params = _node_params(data, "openvins_vio_node")
    settings_config = _resolve_package_path(
        str(params.get("settings_config") or Path(config_file).with_name("openvins.yaml"))
    )
    profiles = _load_yaml(settings_config).get("profiles", {})
    if not isinstance(profiles, dict):
        raise RuntimeError(f"{settings_config} must define a 'profiles' dictionary")

    profile_key = "settings_profile_simulation" if space == "simulation" else "settings_profile_real"
    profile_name = str(params.get(profile_key) or space)
    profile = profiles.get(profile_name)
    if not isinstance(profile, dict):
        valid = ", ".join(sorted(str(key) for key in profiles.keys()))
        raise RuntimeError(f"OpenVINS profile '{profile_name}' not found in {settings_config}. Valid profiles: {valid}")
    return settings_config, profile_name, profile


def _openvins_params(config_file, data, space):
    settings_config, profile_name, profile = _openvins_profile(config_file, data, space)
    binding = _adas_binding(data, space)
    base = _camera_base_topic(binding, space)
    imu_topic = str(binding.get("imu_topic", ""))
    if not imu_topic:
        raise RuntimeError(f"camera_bindings.{space}.adas must define imu_topic for openvins_vio_node")

    output_dir = Path("/tmp") / f"autonomy_openvins_{profile_name}"
    output_dir.mkdir(parents=True, exist_ok=True)
    estimator_path = output_dir / "estimator_config.yaml"
    imu_path = output_dir / "kalibr_imu_chain.yaml"
    imucam_path = output_dir / "kalibr_imucam_chain.yaml"

    estimator = dict(profile.get("estimator", {}))
    estimator.update({
        "verbosity": profile.get("verbosity", "INFO"),
        "use_stereo": profile.get("use_stereo", True),
        "max_cameras": profile.get("max_cameras", 2),
        "relative_config_imu": imu_path.name,
        "relative_config_imucam": imucam_path.name,
    })

    imu = dict(profile.get("imu", {}))
    imu.setdefault("T_i_b", [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
    imu["rostopic"] = imu_topic
    imu.setdefault("time_offset", 0.0)
    imu.setdefault("model", "kalibr")
    imu.setdefault("Tw", [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    imu.setdefault("R_IMUtoGYRO", [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    imu.setdefault("Ta", [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    imu.setdefault("R_IMUtoACC", [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    imu.setdefault("Tg", [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

    cameras = {}
    for camera_name, camera_config in (profile.get("cameras", {}) or {}).items():
        camera = dict(camera_config)
        binding_key = str(camera.pop("binding_key", "left_image_topic"))
        camera["rostopic"] = _topic_from_binding(
            binding,
            (binding_key,),
            f"{base}/left/image_raw" if camera_name == "cam0" else f"{base}/right/image_raw",
        )
        cameras[camera_name] = camera

    _write_openvins_yaml(estimator_path, estimator)
    _write_openvins_yaml(imu_path, {"imu0": imu})
    _write_openvins_yaml(imucam_path, cameras)

    params = _node_params(data, "openvins_vio_node")
    return {
        "verbosity": str(params.get("verbosity", profile.get("verbosity", "INFO"))),
        "config_path": str(estimator_path),
        "use_stereo": bool(params.get("use_stereo", profile.get("use_stereo", True))),
        "max_cameras": int(params.get("max_cameras", profile.get("max_cameras", 2))),
    }


def _managed_nodes(simulation, enable_vio):
    base = ["pointcloud_merge_node", "elevation_mapping_node"]
    if not simulation:
        base = ["realsense_usb_mapper"] + base
    adas_stack = base + (["openvins_vio_node", "rl_local_planner_node"] if enable_vio else [])
    fsd_stack = adas_stack + ["point_lio_monitor_node", "global_planner_node"]
    if not simulation:
        fsd_stack = ["livox_monitor_node"] + fsd_stack
    mapping_stack = ["point_lio_monitor_node"]
    if not simulation:
        mapping_stack = ["livox_monitor_node"] + mapping_stack
    tracking_stack = base + (["ai_detection_node", "tracking_follower_node"] if enable_vio else [])

    return {
        "managed_nodes.drive": base,
        "managed_nodes.adas": adas_stack,
        "managed_nodes.fsd": fsd_stack,
        "managed_nodes.tracking": tracking_stack,
        "managed_nodes.mapping": mapping_stack,
    }


def _worker_node(executable, parameters, name=None, output="log", package=PACKAGE_NAME):
    return Node(
        package=package,
        executable=executable,
        name=name or executable,
        output=output,
        arguments=QUIET_WORKER_ROS_ARGS if output != "screen" else [],
        sigterm_timeout=NODE_SIGTERM_TIMEOUT,
        sigkill_timeout=NODE_SIGKILL_TIMEOUT,
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
    enable_adas_stack = _adas_enabled(data, space)

    actions = [
        LogInfo(msg="Autonomy stack starting in IDLE. Use /autonomy_manager/set_mode to change modes."),
    ]

    if not simulation:
        actions.append(_worker_node(
            "realsense_usb_mapper.py",
            [{
                "mapping_file": config_file,
                "operation_mode": STACK_MODE,
                "respect_autonomy_mode": True,
                "autonomy_status_topic": "/autonomy_manager/status",
            }],
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
    ])

    if not simulation:
        actions.append(_worker_node(
            "livox_monitor_node",
            [_node_params(data, "livox_monitor_node"), use_sim_time, {"simulation": False}],
        ))
        actions.extend(_external_launch(data, "livox_driver", "livox_ros_driver2", "rviz_MID360_launch.py"))

    actions.extend([
        _worker_node(
            "point_lio_monitor_node",
            [_node_params(data, "point_lio_monitor_node"), use_sim_time, {"simulation": False}],
        ),
    ])
    point_lio_params = _node_params(data, "point_lio") or {}
    profile_key = "simulation_launch_file" if simulation else "real_launch_file"
    default_point_lio_launch = str(point_lio_params.get(profile_key, "mapping_avia.launch.py"))
    actions.extend(_external_launch(
        data,
        "point_lio",
        "point_lio",
        default_point_lio_launch,
        _point_lio_launch_arguments(data, simulation),
    ))

    if enable_adas_stack:
        actions.extend([
            _worker_node(
                "run_subscribe_msckf",
                [_openvins_params(config_file, data, space), use_sim_time],
                name="openvins_vio_node",
                package="ov_msckf",
            ),
            _worker_node(
                "vio_pose_adapter_node",
                [_node_params(data, "vio_pose_adapter_node"), use_sim_time],
            ),
            _worker_node(
                "ai_detection_node",
                [_resolve_node_paths(_node_params(data, "ai_detection_node"), ("model_path",)),
                 _ai_topic_params(data, space),
                 use_sim_time],
            ),
            _worker_node(
                "rl_local_planner_node",
                [_node_params(data, "rl_local_planner_node"), use_sim_time, {"enabled": True}],
            ),
            _worker_node(
                "global_planner_node",
                [_node_params(data, "global_planner_node"), use_sim_time, {"enabled": True}],
            ),
            _worker_node(
                "tracking_follower_node",
                [_node_params(data, "tracking_follower_node"), use_sim_time, {"enabled": True}],
            ),
        ])
    else:
        actions.append(LogInfo(msg="ADAS camera is disabled; ADAS/FSD/TRACKING worker nodes will not start."))

    actions.append(
        _worker_node(
            "autonomy_manager_node",
            [
                _node_params(data, "autonomy_manager"),
                {
                    "startup_mode": "IDLE",
                    "speed_limit": 0.0,
                    "enable_ai": False,
                    "segmentation": False,
                    "map_dir": map_dir,
                    **_managed_nodes(simulation, enable_adas_stack),
                },
            ],
            name="autonomy_manager",
            output="screen",
        ),
    )
    return actions


def generate_launch_description():
    package_share = Path(get_package_share_directory(PACKAGE_NAME))

    return LaunchDescription([
        DeclareLaunchArgument(
            "autonomy_config",
            default_value=str(package_share / "resources" / "config" / "autonomy.yaml"),
            description="Single autonomy stack parameter file.",
        ),
        DeclareLaunchArgument("simulation", default_value="false"),
        DeclareLaunchArgument(
            "map_dir",
            default_value=str(package_share / "resources" / "map"),
            description="Directory containing the global map required by FSD mode.",
        ),
        OpaqueFunction(function=_make_stack),
    ])
