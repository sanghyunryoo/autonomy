from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = "autonomy"
NODE_SIGTERM_TIMEOUT = "3.0"
NODE_SIGKILL_TIMEOUT = "6.0"
QUIET_WORKER_ROS_ARGS = ["--ros-args", "--log-level", "fatal"]


def _package_share_path():
    return Path(get_package_share_directory(PACKAGE_NAME))


def _package_source_path():
    launch_path = Path(__file__).resolve()
    if launch_path.parent.name == "launch":
        return launch_path.parent.parent
    return None


def _first_existing_path(*paths):
    for path in paths:
        if path is not None and path.exists():
            return path
    for path in paths:
        if path is not None:
            return path
    return None


def _resolve_package_path(value):
    if not value:
        return value
    path = Path(str(value)).expanduser()
    if path.is_absolute():
        return str(path)

    text = str(value)
    source_path = _package_source_path()
    source_prefix = f"src/{PACKAGE_NAME}/"
    if text.startswith(source_prefix):
        relative_path = text[len(source_prefix):]
        return str(_first_existing_path(
            _package_share_path() / relative_path,
            source_path / relative_path if source_path is not None else None,
        ))
    if text.startswith("resources/"):
        return str(_first_existing_path(
            _package_share_path() / text,
            source_path / text if source_path is not None else None,
        ))
    return text


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _node_params(data, node_name):
    return data.get(node_name, {}).get("ros__parameters", {}).copy()


def _robot_params(data):
    params = data.get("robot", {})
    return params if isinstance(params, dict) else {}


def _robot_model(data):
    return str(_robot_params(data).get("model", "f4")).strip("/") or "f4"


def _robot_namespace(data):
    params = _robot_params(data)
    return str(params.get("namespace", params.get("name", _robot_model(data)))).strip("/")


def _robot_frame_prefix(data):
    params = _robot_params(data)
    prefix = str(params.get("frame_prefix", "")).strip("/")
    if not prefix:
        prefix = _robot_namespace(data)
    return f"{prefix}/" if prefix else ""


def _robot_link(data, key, default):
    return str(_robot_params(data).get(key, default)).strip().lstrip("/")


def _robot_frame(data, link_key, default_link):
    link = _robot_link(data, link_key, default_link)
    if "/" in link:
        return link
    return _robot_frame_prefix(data) + link


def _robot_topic_prefix(data):
    params = _robot_params(data)
    return str(params.get("topic_prefix", f"/{_robot_namespace(data)}")).rstrip("/")


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


def _binding_space(simulation):
    return "simulation" if _parse_bool(simulation, default=False) else "real"


def _camera_bindings(data, space):
    bindings = data.get("camera_bindings", [])
    if isinstance(bindings, dict):
        bindings = bindings.get(space, [])
    if not isinstance(bindings, list):
        raise RuntimeError("camera_bindings must be a list")
    return bindings


def _frame_prefix_from_target(target_frame):
    for suffix in ("base_link", "base_stabilized", "base_footprint"):
        if target_frame == suffix:
            return ""
        if target_frame.endswith("/" + suffix):
            return target_frame[: -len(suffix)]
    return ""


def _frame_prefix(data):
    params = data.get("pointcloud_merge_node", {}).get("ros__parameters", {})
    target_frame = str(params.get("target_frame", _robot_frame(data, "base_stabilized_link", "base_stabilized")))
    default_prefix = _frame_prefix_from_target(target_frame) or _robot_frame_prefix(data)
    return str(params.get("frame_prefix", default_prefix))


def _merge_frame(frame, prefix):
    text = str(frame or "").strip().lstrip("/")
    if not text:
        return ""
    if "/" in text or not prefix:
        return text
    return prefix + text


def _camera_base_topic(data, binding, space):
    camera_name = str(binding.get("camera_name", "front_camera")).strip("/")
    return f"{_robot_topic_prefix(data)}/{camera_name}"


def _merge_params(data, space):
    params = _node_params(data, "pointcloud_merge_node")
    prefix = str(params.pop("frame_prefix", _frame_prefix(data)))
    params["target_frame"] = str(
        params.get("target_frame", _robot_frame(data, "base_stabilized_link", "base_stabilized")))

    camera_names = []
    cameras = {}
    for binding in _camera_bindings(data, space):
        if not isinstance(binding, dict) or not _parse_bool(binding.get("enabled", True), default=True):
            continue
        role = str(binding["role"])
        base = _camera_base_topic(data, binding, space)
        camera_names.append(role)
        frame = binding.get("frame") or binding.get("optical_frame") or binding.get("mount_frame") or ""
        cameras[role] = {
            "enabled": True,
            "depth_topic": str(binding.get("depth_topic", f"{base}/depth/image_rect_raw")),
            "camera_info_topic": str(binding.get("camera_info_topic", f"{base}/depth/camera_info")),
            "frame": _merge_frame(frame, prefix),
        }

    params["camera_names"] = camera_names
    params["cameras"] = cameras

    lidar_names = params.get("lidar_names", [])
    lidars = params.get("lidars", {})
    if isinstance(lidar_names, list):
        if not isinstance(lidars, dict):
            lidars = {}
        for lidar_name in lidar_names:
            name = str(lidar_name)
            lidar = lidars.get(name, {})
            if not isinstance(lidar, dict):
                lidar = {}
            lidar.setdefault("enabled", True)
            lidar.setdefault("cloud_topic", f"{_robot_topic_prefix(data)}/{name}/lidar")
            lidar.setdefault("frame_id", _robot_frame(data, "lidar_link", "lidar_link"))
            lidars[name] = lidar
        params["lidars"] = lidars

    return params


def _mapper_params(data, space):
    params = _node_params(data, "drive_mapper_node")
    prefix = _robot_frame_prefix(data)
    topic_prefix = _robot_topic_prefix(data)
    model = _robot_model(data)

    roles = []
    names = []
    frames = []
    depth_topics = []
    info_topics = []
    imu_topics = []
    front_imu_topic = ""
    front_imu_frame = ""

    for binding in _camera_bindings(data, space):
        if not isinstance(binding, dict) or not _parse_bool(binding.get("enabled", True), default=True):
            continue
        role = str(binding["role"])
        camera_name = str(binding.get("camera_name", f"{role}_camera")).strip("/")
        base = f"{topic_prefix}/{camera_name}"
        frame = binding.get("frame") or binding.get("optical_frame") or binding.get("mount_frame") or camera_name
        merged_frame = _merge_frame(frame, prefix)
        imu_topic = str(binding.get("imu_topic", f"{base}/imu"))

        roles.append(role)
        names.append(camera_name)
        frames.append(merged_frame)
        depth_topics.append(str(binding.get("depth_topic", f"{base}/depth/image_rect_raw")))
        info_topics.append(str(binding.get("camera_info_topic", f"{base}/depth/camera_info")))
        imu_topics.append(imu_topic)

        if role == "front":
            front_imu_topic = imu_topic
            front_imu_frame = merged_frame

    params.update({
        "model": model,
        "frame_prefix": prefix,
        "topic_prefix": topic_prefix,
        "urdf_path": _resolve_package_path(
            _robot_params(data).get("urdf_path", f"resources/urdf/{model}.urdf")),
        "base_frame_id": _robot_frame(data, "base_link", "base_link"),
        "stabilized_frame_id": _robot_frame(data, "base_stabilized_link", "base_stabilized"),
        "front_imu_topic": front_imu_topic,
        "front_imu_frame_id": front_imu_frame,
        "camera_roles": roles,
        "camera_names": names,
        "camera_frames": frames,
        "camera_depth_topics": depth_topics,
        "camera_info_topics": info_topics,
        "camera_imu_topics": imu_topics,
    })
    return params


def _dds_network_log_message(data):
    mode = str((data.get("dds_network", {}) or {}).get("mode", "wireless")).strip().lower()
    return f"DDS network mode: {mode or 'wireless'}"


def _worker_node(
    executable,
    parameters,
    name=None,
    output="log",
    package=PACKAGE_NAME,
    arguments=None,
    additional_env=None,
):
    return Node(
        package=package,
        executable=executable,
        name=name or executable,
        output=output,
        arguments=arguments if arguments is not None else (QUIET_WORKER_ROS_ARGS if output != "screen" else []),
        sigterm_timeout=NODE_SIGTERM_TIMEOUT,
        sigkill_timeout=NODE_SIGKILL_TIMEOUT,
        parameters=parameters,
        additional_env=additional_env,
    )


def _managed_nodes():
    return {
        "managed_nodes.drive": ["drive_mapper_node", "pointcloud_merge_node", "elevation_mapping_node"],
    }


def _rviz_action(data, use_sim_time, launch_rviz):
    params = _node_params(data, "rviz2")
    if not (_parse_bool(params.get("enabled", False), default=False) or launch_rviz):
        return None
    config_path = _resolve_package_path(params.get("config_path", ""))
    arguments = ["-d", str(config_path)] if config_path else []
    return _worker_node("rviz2", [use_sim_time], name="rviz2", package="rviz2", output="screen", arguments=arguments)


def _make_stack(context, *args, **kwargs):
    config_file = LaunchConfiguration("autonomy_config").perform(context)
    simulation_text = LaunchConfiguration("simulation").perform(context)
    simulation = _parse_bool(simulation_text, default=False)
    launch_rviz = _parse_bool(LaunchConfiguration("rviz").perform(context), default=False)
    map_dir = LaunchConfiguration("map_dir").perform(context)
    data = _load_yaml(config_file)
    space = _binding_space(simulation_text)
    use_sim_time = {"use_sim_time": LaunchConfiguration("simulation")}

    actions = [
        LogInfo(msg="Autonomy DRIVE stack: mapper -> pointcloud merge -> elevation mapping."),
        LogInfo(msg="ADAS, FSD, TRACKING, and MAPPING are currently not supported."),
        LogInfo(msg=_dds_network_log_message(data)),
        _worker_node(
            "drive_mapper_node",
            [_mapper_params(data, space), {
                "simulation": simulation,
            }, use_sim_time],
            name="drive_mapper_node",
            output="screen",
        ),
        _worker_node(
            "pointcloud_merge_node",
            [_merge_params(data, space), use_sim_time],
        ),
        _worker_node(
            "elevation_mapping_node",
            [_node_params(data, "elevation_mapping_node"), use_sim_time, {"operation_mode": "drive"}],
            output="screen",
        ),
        _worker_node(
            "autonomy_manager_node",
            [
                _node_params(data, "autonomy_manager"),
                {
                    "startup_mode": "DRIVE",
                    "speed_limit": 0.0,
                    "enable_ai": False,
                    "segmentation": False,
                    "map_dir": map_dir,
                    **_managed_nodes(),
                },
            ],
            name="autonomy_manager",
            output="screen",
        ),
    ]

    rviz = _rviz_action(data, use_sim_time, launch_rviz)
    if rviz is not None:
        actions.append(rviz)
    return actions


def generate_launch_description():
    package_share = Path(get_package_share_directory(PACKAGE_NAME))
    return LaunchDescription([
        DeclareLaunchArgument(
            "autonomy_config",
            default_value=str(package_share / "resources" / "config" / "autonomy.yaml"),
            description="Single autonomy DRIVE stack parameter file.",
        ),
        DeclareLaunchArgument("simulation", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "map_dir",
            default_value=str(package_share / "resources" / "map"),
            description="Reserved map directory; non-DRIVE modes are currently not supported.",
        ),
        OpaqueFunction(function=_make_stack),
    ])
