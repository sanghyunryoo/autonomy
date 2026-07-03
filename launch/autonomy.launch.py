import os
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
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


def _mat_mul(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def _mat_vec_mul(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def _rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _flatten_matrix(matrix):
    return [matrix[i][j] for i in range(3) for j in range(3)]


def _robot_urdf_path(data):
    params = _robot_params(data)
    default = f"resources/urdf/{_robot_model(data)}.urdf"
    return _resolve_package_path(params.get("urdf_path", default))


def _fixed_transform_from_urdf(data, parent_link, child_link):
    urdf_path = _robot_urdf_path(data)
    if not urdf_path or not Path(urdf_path).exists():
        return None

    root = ET.parse(urdf_path).getroot()
    children = {}
    for joint in root.findall("joint"):
        if joint.get("type") != "fixed":
            continue
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        origin = joint.find("origin")
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
            rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
        children.setdefault(parent.get("link"), []).append((child.get("link"), xyz, _rpy_to_matrix(*rpy)))

    stack = [(parent_link, [0.0, 0.0, 0.0], [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])]
    visited = set()
    while stack:
        link, pos, rot = stack.pop()
        if link == child_link:
            return pos, rot
        if link in visited:
            continue
        visited.add(link)
        for next_link, xyz, next_rot in children.get(link, []):
            stack.append((
                next_link,
                [pos[i] + _mat_vec_mul(rot, xyz)[i] for i in range(3)],
                _mat_mul(rot, next_rot),
            ))
    return None


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


def _robot_base_height(data):
    value = _robot_params(data).get("base_height", 0.0)
    try:
        return max(0.0, float(value))
    except (TypeError, ValueError):
        return 0.0


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
    target_frame = str(params.get("target_frame", _robot_frame(data, "base_link", "base_link")))
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


def _stream_profile(data, prefix):
    stream = data.get("stream", {})
    if not isinstance(stream, dict):
        stream = {}
    width = int(stream.get(f"{prefix}_width", 0) or 0)
    height = int(stream.get(f"{prefix}_height", 0) or 0)
    fps = int(stream.get(f"{prefix}_fps", 0) or 0)
    if width > 0 and height > 0 and fps > 0:
        return f"{width}x{height}x{fps}"
    return ""


def _stream_bool(data, key, default):
    stream = data.get("stream", {})
    if not isinstance(stream, dict):
        stream = {}
    return _parse_bool(stream.get(key, default), default=default)


def _realsense_actions(data, simulation):
    if simulation:
        return []

    try:
        get_package_share_directory("realsense2_camera")
    except PackageNotFoundError:
        return [
            LogInfo(msg="realsense2_camera package not found; RealSense cameras will not be launched."),
        ]

    namespace = _robot_namespace(data)
    depth_profile = _stream_profile(data, "depth")
    color_profile = _stream_profile(data, "color")
    enable_color = _stream_bool(data, "enable_color", True)
    actions = []

    for binding in _camera_bindings(data, "real"):
        if not isinstance(binding, dict) or not _parse_bool(binding.get("enabled", True), default=True):
            continue
        camera_name = str(binding.get("camera_name", f"{binding.get('role', 'front')}_camera")).strip("/")
        model = str(binding.get("model", "")).lower()
        has_imu = model.endswith("i") or _parse_bool(binding.get("enable_imu", False), default=False)

        parameters = {
            "camera_namespace": namespace,
            "camera_name": camera_name,
            "enable_depth": True,
            "enable_color": enable_color,
            "enable_infra1": False,
            "enable_infra2": False,
            "enable_gyro": has_imu,
            "enable_accel": has_imu,
            "enable_motion": has_imu,
            "unite_imu_method": 2 if has_imu else 0,
        }
        usb_port_id = str(binding.get("usb_port_id", "")).strip()
        serial_no = str(binding.get("serial_no", "")).strip()
        if usb_port_id:
            parameters["usb_port_id"] = usb_port_id
        if serial_no:
            parameters["serial_no"] = serial_no
        if depth_profile:
            parameters["depth_module.depth_profile"] = depth_profile
        if enable_color and color_profile:
            parameters["rgb_camera.color_profile"] = color_profile

        actions.append(LogInfo(msg=f"Launching RealSense camera {namespace}/{camera_name}."))
        actions.append(Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace=namespace,
            name=camera_name,
            output="screen",
            parameters=[parameters],
            sigterm_timeout=NODE_SIGTERM_TIMEOUT,
            sigkill_timeout=NODE_SIGKILL_TIMEOUT,
        ))

    return actions


def _merge_params(data, space):
    params = _node_params(data, "pointcloud_merge_node")
    prefix = str(params.pop("frame_prefix", _frame_prefix(data)))
    default_target = _robot_frame(data, "base_link", "base_link")
    params["target_frame"] = _merge_frame(params.get("target_frame", default_target), prefix)

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


def _point_lio_config_path(data):
    params = _node_params(data, "point_lio")
    value = params.get("config_path", "")
    if value:
        return _resolve_package_path(value)
    return str(Path(get_package_share_directory("point_lio")) / "config" / "mid360.yaml")


def _point_lio_adapter_params(data):
    params = _node_params(data, "point_lio_lidar_adapter_node")
    params.setdefault("input_topic", f"{_robot_topic_prefix(data)}/livox/lidar")
    params.setdefault("output_topic", "/point_lio/lidar")
    params.setdefault("ring", 0)
    params.setdefault("synthetic_scan_period", 0.1)
    params.setdefault("estimate_scan_period", True)
    params.setdefault("min_scan_period", 0.02)
    params.setdefault("max_scan_period", 0.25)
    params.setdefault("period_filter_alpha", 0.2)
    return params


def _lidar_enabled(data):
    params = _node_params(data, "pointcloud_merge_node")
    lidar_names = params.get("lidar_names", [])
    lidars = params.get("lidars", {})
    if not isinstance(lidar_names, list):
        return False
    if not isinstance(lidars, dict):
        lidars = {}
    for raw_name in lidar_names:
        name = str(raw_name)
        lidar = lidars.get(name, {})
        if not isinstance(lidar, dict):
            lidar = {}
        if _parse_bool(lidar.get("enabled", True), default=True):
            return True
    return False


def _livox_driver_action(data, simulation):
    if simulation or not _lidar_enabled(data):
        return []

    try:
        get_package_share_directory("livox_ros_driver2")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "livox_ros_driver2 package not found. Run scripts/build.sh jetson with Livox support "
            "so MID-360 can be launched automatically."
        ) from exc

    namespace = _robot_namespace(data)
    frame_id = _robot_frame(data, "lidar_link", "lidar_link")
    config_path = os.environ.get("AUTONOMY_LIVOX_CONFIG_PATH", "").strip()
    if not config_path:
        config_path = str(Path(get_package_share_directory("livox_ros_driver2")) / "config" / "MID360_config.json")

    parameters = {
        "xfer_format": 0,
        "multi_topic": 0,
        "data_src": 0,
        "publish_freq": 10.0,
        "output_data_type": 0,
        "frame_id": frame_id,
        "lvx_file_path": "/tmp/autonomy_livox.lvx",
        "user_config_path": config_path,
        "cmdline_input_bd_code": "livox0000000001",
    }

    return [
        LogInfo(msg=f"Launching Livox MID-360 driver in namespace {namespace} using {config_path}."),
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            namespace=namespace,
            name="livox_lidar_publisher",
            output="screen",
            parameters=[parameters],
            remappings=[
                ("livox/lidar", f"/{namespace}/livox/lidar"),
                ("/livox/lidar", f"/{namespace}/livox/lidar"),
                ("livox/imu", f"/{namespace}/livox/imu"),
                ("/livox/imu", f"/{namespace}/livox/imu"),
            ],
            sigterm_timeout=NODE_SIGTERM_TIMEOUT,
            sigkill_timeout=NODE_SIGKILL_TIMEOUT,
        ),
    ]


def _point_lio_params(data):
    params = _node_params(data, "point_lio")
    adapter = _point_lio_adapter_params(data)
    child_to_body = _fixed_transform_from_urdf(
        data,
        _robot_link(data, "base_link", "base_link"),
        _robot_link(data, "lidar_link", "lidar_link"),
    )
    flat = {
        "common.lid_topic": adapter.get("output_topic", "/point_lio/lidar"),
        "common.imu_topic": f"{_robot_topic_prefix(data)}/livox/imu",
        "use_imu_as_input": False,
        "prop_at_freq_of_imu": True,
        "check_satu": True,
        "init_map_size": 10,
        "space_down_sample": True,
        "filter_size_surf": 0.5,
        "filter_size_map": 0.5,
        "cube_side_length": 1000.0,
        "preprocess.lidar_type": 5,
        "preprocess.scan_line": 40,
        "preprocess.scan_rate": 10,
        "preprocess.timestamp_unit": 0,
        "preprocess.blind": 0.1,
        "point_filter_num": 4,
        "mapping.imu_en": True,
        "mapping.imu_time_inte": 0.02,
        "mapping.acc_norm": 9.81,
        "mapping.extrinsic_est_en": False,
        "mapping.satu_acc": 30.0,
        "mapping.satu_gyro": 35.0,
        "mapping.imu_meas_acc_cov": 0.1,
        "mapping.imu_meas_omg_cov": 0.1,
        "mapping.gyr_cov_input": 0.01,
        "mapping.acc_cov_input": 0.1,
        "mapping.lidar_meas_cov": 0.05,
        "odom_header_frame_id": _robot_frame(data, "map_frame", "map"),
        "odom_child_frame_id": _robot_frame(data, "base_link", "base_link"),
        "odom.footprint_frame_id": _robot_frame(data, "base_footprint_link", "base_footprint"),
        "odom.publish_footprint_tf": True,
        "odom.footprint_base_height": _robot_base_height(data),
        "publish.scan_bodyframe_pub_en": False,
        "runtime_pos_log_enable": False,
    }
    if child_to_body is not None:
        child_to_body_t, child_to_body_r = child_to_body
        flat["odom.child_to_body_T"] = child_to_body_t
        flat["odom.child_to_body_R"] = _flatten_matrix(child_to_body_r)
    for key, value in params.items():
        if key != "config_path":
            flat[key] = value
    return flat


def _point_lio_monitor_params(data):
    params = _node_params(data, "point_lio_monitor_node")
    params.setdefault("odom_topic", "/aft_mapped_to_init")
    params.setdefault("cloud_topic", "/cloud_registered")
    params.setdefault("timeout_sec", 3.0)
    return params


def _elevation_mapping_params(data):
    params = _node_params(data, "elevation_mapping_node")
    params["algorithm.base_height"] = _robot_base_height(data)
    return params


def _global_costmap_params(data):
    params = _node_params(data, "global_costmap_node")
    params.setdefault("cloud_topic", "/cloud_registered")
    params.setdefault("frame_id", _robot_frame(data, "map_frame", "map"))
    params.setdefault("footprint_frame_id", _robot_frame(data, "base_footprint_link", "base_footprint"))
    return params


def _local_costmap_params(data):
    params = _node_params(data, "local_costmap_node")
    params.setdefault("height_scan_topic", "/elevation_mapping_node/local_terrain_map")
    params.setdefault("frame_id", _robot_frame(data, "map_frame", "map"))
    params.setdefault("footprint_frame_id", _robot_frame(data, "base_footprint_link", "base_footprint"))
    return params


def _planner_params(data, node_name, enable_planning):
    params = _node_params(data, node_name)
    if node_name == "global_planner_node":
        params.setdefault("target_frame", _robot_frame(data, "base_footprint_link", "base_footprint"))
    if node_name == "local_planner_node":
        params["enabled"] = enable_planning
    return params


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
        "managed_nodes.auto": [
            "drive_mapper_node",
            "pointcloud_merge_node",
            "elevation_mapping_node",
            "point_lio_lidar_adapter_node",
            "point_lio_monitor_node",
            "global_costmap_node",
            "local_costmap_node",
            "global_planner_node",
            "local_planner_node",
        ],
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
    requested_operation_mode = str(LaunchConfiguration("operation_mode").perform(context)).strip().lower() or "drive"
    operation_mode = "auto" if requested_operation_mode in ("adas", "fsd") else requested_operation_mode
    enable_map = _parse_bool(LaunchConfiguration("enable_map").perform(context), default=False)
    enable_planning = _parse_bool(LaunchConfiguration("enable_planning").perform(context), default=False)
    map_dir = LaunchConfiguration("map_dir").perform(context)
    data = _load_yaml(config_file)
    space = _binding_space(simulation_text)
    use_sim_time = {"use_sim_time": LaunchConfiguration("simulation")}
    slam_enabled = operation_mode == "auto"
    if operation_mode not in ("drive", "auto"):
        return [
            LogInfo(msg=f"{requested_operation_mode.upper()} currently not supported. Supported modes: DRIVE, AUTO."),
        ]
    if operation_mode == "auto" and enable_map:
        return [
            LogInfo(msg="AUTO enable_map=true is not implemented yet. Current AUTO supports mapless SLAM/local-terrain planning only."),
        ]

    actions = [
        LogInfo(msg=f"Autonomy {operation_mode.upper()} stack: mapper -> pointcloud merge -> elevation mapping."),
        LogInfo(msg="Point-LIO SLAM and mapless planners enabled for AUTO." if slam_enabled else "Point-LIO SLAM and planners disabled for DRIVE."),
        LogInfo(msg="Mapless planning enabled."),
        LogInfo(msg="Local planner command output enabled." if enable_planning else "Local planner command output disabled."),
        LogInfo(msg=_dds_network_log_message(data)),
        *_realsense_actions(data, simulation),
        *_livox_driver_action(data, simulation),
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
            [_elevation_mapping_params(data), use_sim_time, {"operation_mode": "drive"}],
            output="screen",
        ),
        _worker_node(
            "autonomy_manager_node",
            [
                _node_params(data, "autonomy_manager"),
                {
                    "startup_mode": operation_mode.upper(),
                    "speed_limit": 0.0,
                    "enable_ai": slam_enabled,
                    "segmentation": False,
                    "map_dir": map_dir,
                    **_managed_nodes(),
                },
            ],
            name="autonomy_manager",
            output="screen",
        ),
    ]
    if requested_operation_mode in ("adas", "fsd"):
        actions.insert(1, LogInfo(msg=f"{requested_operation_mode.upper()} requested as AUTO alias."))

    if slam_enabled:
        actions.extend([
            _worker_node(
                "point_lio_lidar_adapter_node",
                [_point_lio_adapter_params(data), use_sim_time],
                name="point_lio_lidar_adapter_node",
                output="screen",
            ),
            _worker_node(
                "pointlio_mapping",
                [_point_lio_config_path(data), _point_lio_params(data), use_sim_time],
                name="laserMapping",
                package="point_lio",
                output="screen",
                arguments=[],
            ),
            _worker_node(
                "point_lio_monitor_node",
                [_point_lio_monitor_params(data), use_sim_time],
                name="point_lio_monitor_node",
                output="screen",
            ),
            _worker_node(
                "global_costmap_node",
                [_global_costmap_params(data), use_sim_time],
                name="global_costmap_node",
                output="screen",
            ),
            _worker_node(
                "local_costmap_node",
                [_local_costmap_params(data), use_sim_time],
                name="local_costmap_node",
                output="screen",
            ),
            _worker_node(
                "global_planner_node",
                [_planner_params(data, "global_planner_node", enable_planning), use_sim_time],
                name="global_planner_node",
                output="screen",
            ),
            _worker_node(
                "local_planner_node",
                [_planner_params(data, "local_planner_node", enable_planning), use_sim_time],
                name="local_planner_node",
                output="screen",
            ),
        ])

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
        DeclareLaunchArgument("operation_mode", default_value="drive"),
        DeclareLaunchArgument("enable_map", default_value="false"),
        DeclareLaunchArgument("enable_planning", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "map_dir",
            default_value=str(package_share / "resources" / "map"),
            description="Map directory for AUTO SLAM consumers.",
        ),
        OpaqueFunction(function=_make_stack),
    ])
