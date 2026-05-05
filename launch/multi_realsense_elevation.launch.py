from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import UnlessCondition
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


def _load_merge_parameters(mapping_file):
    with open(mapping_file, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    node_params = (
        data.get("pointcloud_merge_node", {})
        .get("ros__parameters", {})
        .copy()
    )
    target_frame = str(node_params.get("target_frame", "base_link"))
    frame_prefix = _frame_prefix_from_target(target_frame)

    camera_names = []
    cameras = {}
    for binding in data.get("camera_bindings", []) or []:
        if not isinstance(binding, dict):
            continue
        if not _parse_bool(binding.get("enabled", True), default=True):
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
                binding.get("optical_frame", binding.get("publish_frame", "")),
                frame_prefix,
            ),
        }

    node_params["camera_names"] = camera_names
    node_params["cameras"] = cameras
    node_params.setdefault("static_tf_frame_prefix", frame_prefix)
    return node_params


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
    mapping_file = LaunchConfiguration("serial_mapping").perform(context)
    merge_parameters = _load_merge_parameters(mapping_file)

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


def generate_launch_description():
    package_share = Path(get_package_share_directory("height_map_ros2"))
    default_elevation_config = package_share / "config" / "elevation_mapping.yaml"
    default_serial_mapping = package_share / "config" / "realsense_serial_mapping.yaml"
    default_urdf = package_share / "urdf" / "f16.urdf"

    elevation_config_arg = DeclareLaunchArgument(
        "elevation_config",
        default_value=str(default_elevation_config),
        description="Path to the elevation mapping parameter file.",
    )
    serial_mapping_arg = DeclareLaunchArgument(
        "serial_mapping",
        default_value=str(default_serial_mapping),
        description="Path to the RealSense camera mapping and merge parameter file.",
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
            "If true, skip hardware serial mapping, consume simulation topics "
            "directly, and enable use_sim_time on processing nodes."
        ),
    )

    serial_mapper_node = Node(
        package="height_map_ros2",
        executable="realsense_serial_mapper.py",
        name="realsense_serial_mapper",
        output="screen",
        parameters=[{"mapping_file": LaunchConfiguration("serial_mapping")}],
        condition=UnlessCondition(LaunchConfiguration("simulation")),
    )

    merge_node = OpaqueFunction(function=_make_merge_node)

    elevation_node = Node(
        package="height_map_ros2",
        executable="elevation_mapping_node",
        name="elevation_mapping_node",
        output="screen",
        parameters=[
            LaunchConfiguration("elevation_config"),
            {"use_sim_time": LaunchConfiguration("simulation")},
        ],
    )

    return LaunchDescription(
        [
            elevation_config_arg,
            serial_mapping_arg,
            urdf_arg,
            simulation_arg,
            serial_mapper_node,
            merge_node,
            elevation_node,
        ]
    )
