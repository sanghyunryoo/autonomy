from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


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


def _select_camera_bindings(data, mode):
    bindings = data.get("camera_bindings", [])
    if isinstance(bindings, dict):
        bindings = bindings.get(mode, [])
    if not isinstance(bindings, list):
        raise ValueError(f"'camera_bindings.{mode}' must be a list")
    return bindings


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


def _load_merge_parameters(mapping_file, simulation, operation_mode):
    with open(mapping_file, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    binding_space = "simulation" if _parse_bool(simulation, default=False) else "real"
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
        if not isinstance(binding, dict):
            continue

        role = str(binding["role"])
        camera_names.append(role)
        cameras[role] = {
            "enabled": True,
            "depth_topic": str(binding.get("depth_topic", f"/{role}/depth/image_rect")),
            "camera_info_topic": str(
                binding.get("camera_info_topic", f"/{role}/depth/camera_info")
            ),
            "mount_frame": _merge_frame(
                binding.get("mount_frame", ""),
                frame_prefix,
            ),
            "optical_frame": _merge_frame(
                binding.get("optical_frame", ""),
                frame_prefix,
            ),
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


def _load_operation_mode_from_config(elevation_config):
    with open(elevation_config, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    return str(
        data.get("elevation_mapping_node", {})
        .get("ros__parameters", {})
        .get("operation_mode", "drive")
    )


def _resolve_operation_mode(context):
    operation_mode = LaunchConfiguration("operation_mode").perform(context).strip()
    if operation_mode:
        return operation_mode
    elevation_config = LaunchConfiguration("elevation_config").perform(context)
    return _load_operation_mode_from_config(elevation_config)


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


def _make_merge_node(context, *args, **kwargs):
    mapping_file = LaunchConfiguration("camera_mapping").perform(context)
    simulation = LaunchConfiguration("simulation").perform(context)
    operation_mode = _resolve_operation_mode(context)
    merge_parameters = _load_merge_parameters(mapping_file, simulation, operation_mode)

    return [
        Node(
            package="height_map_ros2",
            executable="pointcloud_merge_node",
            name="pointcloud_merge_node",
            output="screen",
            parameters=[
                merge_parameters,
                {"urdf_path": LaunchConfiguration("urdf_path")},
                {"use_sim_time": LaunchConfiguration("simulation")},
            ],
        )
    ]


def _make_usb_mapper_node(context, *args, **kwargs):
    return [
        Node(
            package="height_map_ros2",
            executable="realsense_usb_mapper.py",
            name="realsense_usb_mapper",
            output="screen",
            parameters=[
                {
                    "mapping_file": LaunchConfiguration("camera_mapping"),
                    "operation_mode": _resolve_operation_mode(context),
                }
            ],
        )
    ]


def _make_elevation_node(context, *args, **kwargs):
    return [
        Node(
            package="height_map_ros2",
            executable="elevation_mapping_node",
            name="elevation_mapping_node",
            output="screen",
            parameters=[
                LaunchConfiguration("elevation_config"),
                {"use_sim_time": LaunchConfiguration("simulation")},
                {"operation_mode": _resolve_operation_mode(context)},
            ],
        )
    ]


def _make_slam_node(context, *args, **kwargs):
    return [
        Node(
            package="height_map_ros2",
            executable="orbslam3_node",
            name="orbslam3_node",
            output="screen",
            parameters=[
                LaunchConfiguration("slam_config"),
                {"use_sim_time": LaunchConfiguration("simulation")},
            ],
        )
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("height_map_ros2"))
    default_elevation_config = package_share / "config" / "elevation_mapping.yaml"
    default_camera_mapping = package_share / "config" / "realsense_usb_mapping.yaml"
    default_slam_config = package_share / "config" / "autonomy.yaml"
    default_urdf = package_share / "urdf" / "f16.urdf"

    elevation_config_arg = DeclareLaunchArgument(
        "elevation_config",
        default_value=str(default_elevation_config),
        description="Path to the elevation mapping parameter file.",
    )
    camera_mapping_arg = DeclareLaunchArgument(
        "camera_mapping",
        default_value=str(default_camera_mapping),
        description="Path to the RealSense camera mapping and merge parameter file.",
    )
    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=str(default_slam_config),
        description="Path to the parameter file containing orbslam3_node settings.",
    )
    urdf_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=str(default_urdf),
        description="Path to the URDF used by the merge node for static TF.",
    )
    simulation_arg = DeclareLaunchArgument(
        "simulation",
        default_value="false",
        description=(
            "If true, skip hardware USB-port mapping, consume simulation topics "
            "directly, and enable use_sim_time on processing nodes."
        ),
    )
    operation_mode_arg = DeclareLaunchArgument(
        "operation_mode",
        default_value="",
        description=(
            "Operation mode: drive, adas, or fsd. If empty, use elevation_config "
            "elevation_mapping_node.ros__parameters.operation_mode."
        ),
    )
    enable_slam_arg = DeclareLaunchArgument(
        "enable_slam",
        default_value="true",
        description="If true, start orbslam3_node together with the elevation stack.",
    )

    usb_mapper_node = OpaqueFunction(
        function=_make_usb_mapper_node,
        condition=UnlessCondition(LaunchConfiguration("simulation")),
    )

    merge_node = OpaqueFunction(function=_make_merge_node)
    elevation_node = OpaqueFunction(function=_make_elevation_node)
    slam_node = OpaqueFunction(
        function=_make_slam_node,
        condition=IfCondition(LaunchConfiguration("enable_slam")),
    )

    return LaunchDescription(
        [
            elevation_config_arg,
            camera_mapping_arg,
            slam_config_arg,
            urdf_arg,
            simulation_arg,
            operation_mode_arg,
            enable_slam_arg,
            usb_mapper_node,
            merge_node,
            elevation_node,
            slam_node,
        ]
    )
