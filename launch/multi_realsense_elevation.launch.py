from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("height_map_ros2"))
    default_merge_config = package_share / "config" / "multi_realsense_elevation.yaml"
    default_elevation_config = package_share / "config" / "elevation_mapping.yaml"
    default_serial_mapping = package_share / "config" / "realsense_serial_mapping.yaml"
    default_urdf = package_share / "urdf" / "f16.urdf"

    merge_config_arg = DeclareLaunchArgument(
        "merge_config",
        default_value=str(default_merge_config),
        description="Path to the point cloud merge parameter file.",
    )
    elevation_config_arg = DeclareLaunchArgument(
        "elevation_config",
        default_value=str(default_elevation_config),
        description="Path to the elevation mapping parameter file.",
    )
    serial_mapping_arg = DeclareLaunchArgument(
        "serial_mapping",
        default_value=str(default_serial_mapping),
        description="Path to the hardware RealSense serial mapping file.",
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

    merge_node = Node(
        package="height_map_ros2",
        executable="pointcloud_merge_node",
        name="pointcloud_merge_node",
        output="screen",
        parameters=[
            LaunchConfiguration("merge_config"),
            {"urdf_path": LaunchConfiguration("urdf_path")},
            {"use_sim_time": LaunchConfiguration("simulation")},
        ],
    )

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
            merge_config_arg,
            elevation_config_arg,
            serial_mapping_arg,
            urdf_arg,
            simulation_arg,
            serial_mapper_node,
            merge_node,
            elevation_node,
        ]
    )
