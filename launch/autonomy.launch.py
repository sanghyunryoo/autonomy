from pathlib import Path
import os
import tempfile
from xml.sax.saxutils import escape

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


def _robot_state_publisher_node(data, use_sim_time):
    params = _node_params(data, "robot_state_publisher") or {}
    if not _parse_bool(params.pop("enabled", True), default=True):
        return None
    urdf_path = _resolve_package_path(params.pop("urdf_path", "src/autonomy/resources/urdf/f16.urdf"))
    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()
    return _worker_node(
        "robot_state_publisher",
        [params, use_sim_time, {"robot_description": robot_description}],
        name="robot_state_publisher",
        package="robot_state_publisher",
    )


def _static_tf_node(parent_frame, child_frame, use_sim_time):
    return _worker_node(
        "static_transform_publisher",
        [use_sim_time],
        name=f"static_tf_alias_{child_frame.replace('/', '_')}",
        package="tf2_ros",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            str(parent_frame),
            str(child_frame),
        ],
    )


def _identity_static_tf_node(parent_frame, child_frame, use_sim_time, name_prefix="static_tf_alias"):
    parent = str(parent_frame or "").strip().lstrip("/")
    child = str(child_frame or "").strip().lstrip("/")
    if not parent or not child or parent == child:
        return None
    return _worker_node(
        "static_transform_publisher",
        [use_sim_time],
        name=f"{name_prefix}_{child.replace('/', '_')}",
        package="tf2_ros",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            parent,
            child,
        ],
    )


def _lidar_params(data):
    params = _node_params(data, "lidar") or {}
    return params if isinstance(params, dict) else {}


def _lidar_tf_nodes(data, use_sim_time):
    params = _lidar_params(data)
    if not _parse_bool(params.get("publish_driver_frame_alias", True), default=True):
        return []
    parent = params.get("robot_lidar_frame", "4w4l/lidar_link")
    child = params.get("driver_frame", "")
    node = _identity_static_tf_node(parent, child, use_sim_time, name_prefix="static_tf_lidar")
    return [] if node is None else [node]


def _frame_alias_static_tf_nodes(data, space, use_sim_time):
    prefix = _frame_prefix(data)
    if not prefix:
        return []

    aliases = []
    seen = set()
    bindings = _camera_bindings(data, space)
    for binding in bindings:
        if not isinstance(binding, dict) or not _parse_bool(binding.get("enabled", True), default=True):
            continue
        for key in ("optical_frame", "rgb_optical_frame", "color_optical_frame"):
            child = str(binding.get(key, "")).strip().lstrip("/")
            if not child or "/" in child:
                continue
            parent = prefix + child
            if parent == child or (parent, child) in seen:
                continue
            seen.add((parent, child))
            aliases.append(_static_tf_node(parent, child, use_sim_time))
    return aliases


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


def _dds_network_params(data):
    params = data.get("dds_network", {})
    return params if isinstance(params, dict) else {}


def _dds_network_mode(data):
    params = _dds_network_params(data)
    return str(params.get("mode", params.get("transport", "wireless"))).strip().lower()


def _ip_without_prefix(value):
    return str(value).strip().split("/", 1)[0]


def _dds_network_env(data):
    params = _dds_network_params(data)
    mode = _dds_network_mode(data)
    if mode in ("", "auto", "default", "wireless", "wifi"):
        return None
    if mode not in ("wired", "ethernet"):
        raise RuntimeError("dds_network.mode must be one of: wireless, wired")

    local_ip = _ip_without_prefix(params.get("local_ip", ""))
    peer_ip = _ip_without_prefix(params.get("peer_ip", params.get("remote_ip", "")))
    if not local_ip:
        raise RuntimeError("dds_network.local_ip is required when mode is wired")
    if not peer_ip:
        raise RuntimeError("dds_network.peer_ip is required when mode is wired")

    allow_multicast = _parse_bool(params.get("allow_multicast", False), default=False)
    multicast = "true" if allow_multicast else "false"
    multicast_recv = "preferred" if allow_multicast else "none"
    allow_multicast_text = "true" if allow_multicast else "false"
    peers = []
    for address in (peer_ip, local_ip):
        if address and address not in peers:
            peers.append(address)
    peer_xml = "\n".join(f'        <Peer address="{escape(address)}" />' for address in peers)
    xml = f"""<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface address="{escape(local_ip)}" priority="default" multicast="{multicast}" />
      </Interfaces>
      <AllowMulticast>{allow_multicast_text}</AllowMulticast>
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
        prefix="autonomy_cyclonedds_",
        suffix=".xml",
        delete=False,
    ) as config_file:
        config_file.write(xml)
        config_path = Path(config_file.name)
    return {
        "CYCLONEDDS_URI": f"file://{config_path}",
    }


def _dds_network_log_message(data, env):
    mode = _dds_network_mode(data)
    if not env:
        return f"DDS network mode: {mode or 'wireless'}"
    params = _dds_network_params(data)
    return (
        "DDS network mode: wired "
        f"local_ip={params.get('local_ip')} peer_ip={params.get('peer_ip', params.get('remote_ip'))} "
        f"CYCLONEDDS_URI={env['CYCLONEDDS_URI']}"
    )


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
    if "rviz" in launch_file.lower() and not _parse_bool(params.get("allow_rviz_launch", False), default=False):
        return [LogInfo(msg=f"{node_name}: skipping RViz launch file: {launch_file}")]
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
    launch_arguments.setdefault("rviz", "false")
    launch_arguments.setdefault("use_rviz", "false")
    launch_arguments.setdefault("launch_rviz", "false")

    sim_lidar_topic = params.get("lidar_topic_simulation")
    real_lidar_topic = params.get("lidar_topic_real")
    if simulation and sim_lidar_topic:
        launch_arguments["lid_topic"] = sim_lidar_topic
    elif not simulation and real_lidar_topic:
        launch_arguments["lid_topic"] = real_lidar_topic
    return launch_arguments


def _point_lio_actions(data, simulation, use_sim_time):
    params = _node_params(data, "point_lio") or {}
    if not _parse_bool(params.get("enabled", True), default=True):
        return []
    try:
        package_share = Path(get_package_share_directory(str(params.get("package", "point_lio"))))
    except Exception as exc:
        return [LogInfo(msg=f"point_lio: package not found, skipping Point-LIO ({exc})")]

    config_key = "simulation_config_file" if simulation else "real_config_file"
    default_config = "velody16.yaml" if simulation else "mid360.yaml"
    config_file = _resolve_package_path(params.get(config_key, ""))
    if not config_file:
        config_file = str(package_share / "config" / str(params.get("config_file", default_config)))
    elif not Path(config_file).is_absolute() and "/" not in str(config_file):
        config_file = str(package_share / "config" / str(config_file))

    lidar_params = _lidar_params(data)
    lidar_topic = params.get("lidar_topic_simulation" if simulation else "lidar_topic_real")
    if not lidar_topic:
        lidar_topic = lidar_params.get("pointcloud_topic_simulation" if simulation else "pointcloud_topic")
    imu_topic = params.get("imu_topic_simulation" if simulation else "imu_topic_real")
    if not imu_topic:
        imu_topic = lidar_params.get("imu_topic_simulation" if simulation else "imu_topic")
    overrides = {
        "odom_header_frame_id": str(params.get("odom_frame_id", "map")),
        "odom_child_frame_id": str(params.get("base_frame_id", "4w4l/base_footprint")),
        "use_sim_time": simulation,
    }
    if lidar_topic:
        overrides["common.lid_topic"] = str(lidar_topic)
    if imu_topic:
        overrides["common.imu_topic"] = str(imu_topic)

    return [
        _worker_node(
            "pointlio_mapping",
            [config_file, overrides, use_sim_time],
            name="laserMapping",
            package=str(params.get("package", "point_lio")),
            output="screen",
        )
    ]


def _nav2_actions(data, simulation):
    params = _node_params(data, "nav2") or {}
    if not _parse_bool(params.get("enabled", True), default=True):
        return []
    try:
        package_share = Path(get_package_share_directory(str(params.get("package", "nav2_bringup"))))
    except Exception as exc:
        return [LogInfo(msg=f"nav2: package not found, skipping Nav2 ({exc})")]

    launch_file = str(params.get("launch_file", "navigation_launch.py"))
    launch_path = package_share / "launch" / launch_file
    if not launch_path.exists():
        return [LogInfo(msg=f"nav2: launch file not found: {launch_path}")]

    params_file = _resolve_package_path(params.get("params_file", "resources/config/nav2_point_lio.yaml"))
    launch_arguments = {
        "use_sim_time": "true" if simulation else "false",
        "params_file": params_file,
        "autostart": str(params.get("autostart", True)).lower(),
    }
    for key, value in (params.get("launch_arguments", {}) or {}).items():
        launch_arguments[str(key)] = str(value)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_path)),
            launch_arguments=launch_arguments.items(),
        )
    ]


def _livox_driver_actions(data, use_sim_time):
    params = _node_params(data, "livox_driver") or {}
    if not _parse_bool(params.get("enabled", True), default=True):
        return []
    try:
        get_package_share_directory(str(params.get("package", "livox_ros_driver2")))
    except Exception as exc:
        return [LogInfo(msg=f"livox_driver: package not found, skipping Livox driver ({exc})")]

    user_config_path = (
        os.environ.get("AUTONOMY_LIVOX_CONFIG_PATH")
        or _resolve_package_path(params.get("user_config_path", ""))
    )
    if not user_config_path:
        network_params = data.get("livox_network", {}) or {}
        candidate = str(network_params.get("config_output", "")).strip()
        if candidate and Path(candidate).expanduser().exists():
            user_config_path = str(Path(candidate).expanduser())
    if not user_config_path:
        return [LogInfo(msg="livox_driver: user_config_path is empty; run scripts/launch.sh or set AUTONOMY_LIVOX_CONFIG_PATH")]

    driver_params = {
        "xfer_format": int(params.get("xfer_format", 0)),
        "multi_topic": int(params.get("multi_topic", 0)),
        "data_src": int(params.get("data_src", 0)),
        "publish_freq": float(params.get("publish_freq", 10.0)),
        "output_data_type": int(params.get("output_data_type", 0)),
        "frame_id": str(params.get("frame_id", "livox_frame")),
        "lvx_file_path": str(params.get("lvx_file_path", "/tmp/autonomy_mid360.lvx")),
        "user_config_path": str(user_config_path),
        "cmdline_input_bd_code": str(params.get("cmdline_input_bd_code", "livox0000000001")),
    }
    return [
        _worker_node(
            str(params.get("executable", "livox_ros_driver2_node")),
            [driver_params, use_sim_time],
            name="livox_lidar_publisher",
            package=str(params.get("package", "livox_ros_driver2")),
            output="screen",
        )
    ]


def _rviz_actions(data, use_sim_time, launch_rviz=False):
    params = _node_params(data, "rviz2") or {}
    if not (_parse_bool(params.get("enabled", False), default=False) or launch_rviz):
        return []
    arguments = []
    config_path = _resolve_package_path(params.get("config_path", ""))
    if not config_path:
        try:
            config_path = str(Path(get_package_share_directory("nav2_bringup")) / "rviz" / "nav2_default_view.rviz")
        except Exception:
            config_path = ""
    if config_path:
        arguments = ["-d", str(config_path)]
    return [
        _worker_node(
            "rviz2",
            [use_sim_time],
            name="rviz2",
            package="rviz2",
            output="screen",
            arguments=arguments,
        )
    ]


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
    default_prefix = _frame_prefix_from_target(target_frame)
    return str(params.get("frame_prefix", params.get("static_tf_frame_prefix", default_prefix)))


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
    params.pop("static_tf_frame_prefix", None)
    params.pop("urdf_path", None)
    prefix = str(params.pop("frame_prefix", _frame_prefix(data)))

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
        }

    params["camera_names"] = camera_names
    params["cameras"] = cameras
    return params


def _adas_binding(data, space):
    return _find_camera_binding(data, space, "adas")


def _front_binding(data, space):
    return _find_camera_binding(data, space, "front")


def _adas_enabled(data, space):
    return _enabled_camera_binding(data, space, "adas") is not None


def _front_enabled(data, space):
    return _enabled_camera_binding(data, space, "front") is not None


def _node_enabled(data, node_name, default=True):
    return _parse_bool(_node_params(data, node_name).get("enabled", default), default=default)


def _imu_stabilized_params(data, space):
    params = _node_params(data, "imu_stabilized_tf_node").copy()
    try:
        binding = _front_binding(data, space)
    except RuntimeError:
        binding = {}
    imu_topic = str(binding.get("imu_topic", params.get("imu_topic", "")))
    if imu_topic:
        params["imu_topic"] = imu_topic
    return params


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


def _managed_nodes(simulation, enable_nav2, enable_tracking):
    base = ["imu_stabilized_tf_node", "pointcloud_merge_node", "elevation_mapping_node"]
    if not simulation:
        base = ["realsense_usb_mapper", "livox_monitor_node"] + base
    point_lio_stack = ["point_lio_monitor_node", "localization_pose_adapter_node"]
    adas_stack = base + point_lio_stack
    if enable_nav2:
        adas_stack += ["global_planner_node", "goal_pose_to_nav2_action_node", "cmd_vel_to_command_user_node"]
    else:
        adas_stack += ["rl_local_planner_node"]
    fsd_stack = list(adas_stack)
    if "global_planner_node" not in fsd_stack:
        fsd_stack += ["global_planner_node"]
    if enable_nav2:
        for node_name in ("goal_pose_to_nav2_action_node", "cmd_vel_to_command_user_node"):
            if node_name not in fsd_stack:
                fsd_stack.append(node_name)
    mapping_stack = ["point_lio_monitor_node", "point_lio_map_saver_node"]
    if not simulation:
        mapping_stack = ["livox_monitor_node"] + mapping_stack
    tracking_stack = base + (["ai_detection_node", "tracking_follower_node"] if enable_tracking else [])

    return {
        "managed_nodes.drive": base,
        "managed_nodes.adas": adas_stack,
        "managed_nodes.fsd": fsd_stack,
        "managed_nodes.tracking": tracking_stack,
        "managed_nodes.mapping": mapping_stack,
    }


def _worker_node(
    executable,
    parameters,
    name=None,
    output="log",
    package=PACKAGE_NAME,
    remappings=None,
    arguments=None,
    additional_env=None,
):
    return Node(
        package=package,
        executable=executable,
        name=name or executable,
        output=output,
        arguments=arguments if arguments is not None else (QUIET_WORKER_ROS_ARGS if output != "screen" else []),
        remappings=remappings or [],
        sigterm_timeout=NODE_SIGTERM_TIMEOUT,
        sigkill_timeout=NODE_SIGKILL_TIMEOUT,
        parameters=parameters,
        additional_env=additional_env,
    )


def _make_stack(context, *args, **kwargs):
    config_file = LaunchConfiguration("autonomy_config").perform(context)
    simulation_text = LaunchConfiguration("simulation").perform(context)
    simulation = _parse_bool(simulation_text, default=False)
    launch_rviz = _parse_bool(LaunchConfiguration("rviz").perform(context), default=False)
    map_dir = LaunchConfiguration("map_dir").perform(context)
    data = _load_yaml(config_file)
    dds_env = _dds_network_env(data)
    space = _binding_space(simulation_text)
    use_sim_time = {"use_sim_time": LaunchConfiguration("simulation")}
    enable_local_stack = True
    enable_nav2 = _node_enabled(data, "nav2")
    enable_rl_local_planner = enable_local_stack and _node_enabled(data, "rl_local_planner_node") and not enable_nav2
    enable_ai_detection = _adas_enabled(data, space) and _node_enabled(data, "ai_detection_node")
    enable_global_planner = enable_local_stack and _node_enabled(data, "global_planner_node")
    enable_tracking_follower = enable_ai_detection and _node_enabled(data, "tracking_follower_node")

    actions = [
        LogInfo(msg="Autonomy stack starting in DRIVE. Use /autonomy_manager/set_mode to change modes."),
        LogInfo(msg=_dds_network_log_message(data, dds_env)),
    ]

    robot_state_publisher = _robot_state_publisher_node(data, use_sim_time)
    if robot_state_publisher is not None:
        actions.append(robot_state_publisher)
    actions.extend(_frame_alias_static_tf_nodes(data, space, use_sim_time))
    actions.extend(_lidar_tf_nodes(data, use_sim_time))

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
            output="screen",
        ))

    actions.extend([
        _worker_node(
            "imu_stabilized_tf_node",
            [_imu_stabilized_params(data, space), use_sim_time],
        ),
        _worker_node(
            "pointcloud_merge_node",
            [_merge_params(data, space), use_sim_time],
        ),
        _worker_node(
            "elevation_mapping_node",
            [_node_params(data, "elevation_mapping_node"), use_sim_time, {"operation_mode": STACK_MODE}],
            output="screen",
            additional_env=dds_env,
        ),
    ])

    if not simulation:
        actions.append(_worker_node(
            "livox_monitor_node",
            [_node_params(data, "livox_monitor_node"), use_sim_time, {"simulation": False}],
        ))
        actions.extend(_livox_driver_actions(data, use_sim_time))

    actions.extend([
        _worker_node(
            "point_lio_monitor_node",
            [_node_params(data, "point_lio_monitor_node"), use_sim_time, {"simulation": simulation}],
        ),
        _worker_node(
            "point_lio_map_saver_node",
            [_node_params(data, "point_lio_map_saver_node"), use_sim_time, {"map_dir": map_dir}],
        ),
    ])
    actions.extend(_point_lio_actions(data, simulation, use_sim_time))
    actions.append(_worker_node(
        "localization_pose_adapter_node",
        [_node_params(data, "localization_pose_adapter_node"), use_sim_time],
        additional_env=dds_env,
    ))

    if enable_ai_detection:
        actions.append(
            _worker_node(
                "ai_detection_node",
                [_resolve_node_paths(_node_params(data, "ai_detection_node"), ("model_path",)),
                 _ai_topic_params(data, space),
                 use_sim_time],
            )
        )

    if enable_rl_local_planner:
        actions.append(
            _worker_node(
                "rl_local_planner_node",
                [_node_params(data, "rl_local_planner_node"), use_sim_time, {"enabled": True}],
            )
        )

    if enable_global_planner:
        actions.append(
            _worker_node(
                "global_planner_node",
                [_resolve_node_paths(_node_params(data, "global_planner_node"), ("map_file",)),
                 use_sim_time,
                 {"enabled": True}],
            )
        )

    if enable_nav2:
        actions.extend(_nav2_actions(data, simulation))
        actions.append(
            _worker_node(
                "goal_pose_to_nav2_action_node",
                [_node_params(data, "goal_pose_to_nav2_action_node"), use_sim_time],
            )
        )
        actions.append(
            _worker_node(
                "cmd_vel_to_command_user_node",
                [_node_params(data, "cmd_vel_to_command_user_node"), use_sim_time],
            )
        )
        actions.extend(_rviz_actions(data, use_sim_time, launch_rviz))

    if enable_tracking_follower:
        actions.append(
            _worker_node(
                "tracking_follower_node",
                [_node_params(data, "tracking_follower_node"), use_sim_time, {"enabled": True}],
            )
        )

    if not _adas_enabled(data, space):
        actions.append(LogInfo(msg="ADAS camera is disabled; AI detection/tracking worker nodes will not start."))

    actions.append(
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
                    **_managed_nodes(simulation, enable_nav2, enable_tracking_follower),
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
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "map_dir",
            default_value=str(package_share / "resources" / "map"),
            description="Directory containing the global map required by FSD mode.",
        ),
        OpaqueFunction(function=_make_stack),
    ])
