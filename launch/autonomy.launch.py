from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _load_operation_mode_from_config(elevation_config):
    with open(elevation_config, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    return str(
        data.get("elevation_mapping_node", {})
        .get("ros__parameters", {})
        .get("operation_mode", "DRIVE")
    )


def _resolve_operation_mode(context):
    operation_mode = LaunchConfiguration("operation_mode").perform(context).strip()
    if operation_mode:
        return operation_mode
    elevation_config = LaunchConfiguration("elevation_config").perform(context)
    return _load_operation_mode_from_config(elevation_config)


def _make_autonomy_manager_node(context, *args, **kwargs):
    parameters = [LaunchConfiguration("autonomy_config")]
    operation_mode = LaunchConfiguration("operation_mode").perform(context).strip()
    if operation_mode:
        parameters.append({"startup_mode": operation_mode})
    return [
        Node(
            package="height_map_ros2",
            executable="autonomy_manager_node",
            name="autonomy_manager",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("height_map_ros2"))
    autonomy_config = package_share / "config" / "autonomy.yaml"
    elevation_config = package_share / "config" / "elevation_mapping.yaml"
    elevation_launch = package_share / "launch" / "multi_realsense_elevation.launch.py"

    autonomy_config_arg = DeclareLaunchArgument(
        "autonomy_config",
        default_value=str(autonomy_config),
        description="Autonomy stack parameter file.",
    )
    elevation_config_arg = DeclareLaunchArgument(
        "elevation_config",
        default_value=str(elevation_config),
        description="Elevation stack parameter file.",
    )
    operation_mode_arg = DeclareLaunchArgument(
        "operation_mode",
        default_value="",
        description="Optional mode override: DRIVE, ADAS, FSD, MAPPING, ERROR, ESTOP.",
    )
    simulation_arg = DeclareLaunchArgument("simulation", default_value="false")

    elevation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(elevation_launch)),
        launch_arguments={
            "elevation_config": LaunchConfiguration("elevation_config"),
            "simulation": LaunchConfiguration("simulation"),
            "operation_mode": LaunchConfiguration("operation_mode"),
        }.items(),
    )

    common_params = [LaunchConfiguration("autonomy_config")]
    return LaunchDescription(
        [
            autonomy_config_arg,
            elevation_config_arg,
            operation_mode_arg,
            simulation_arg,
            elevation_stack,
            OpaqueFunction(function=_make_autonomy_manager_node),
            Node(
                package="height_map_ros2",
                executable="ai_detection_node",
                name="ai_detection_node",
                output="screen",
                parameters=common_params,
            ),
            Node(
                package="height_map_ros2",
                executable="rl_local_planner_node",
                name="rl_local_planner_node",
                output="screen",
                parameters=common_params,
            ),
            Node(
                package="height_map_ros2",
                executable="global_planner_node",
                name="global_planner_node",
                output="screen",
                parameters=common_params,
            ),
            Node(
                package="height_map_ros2",
                executable="orbslam3_node",
                name="orbslam3_node",
                output="screen",
                parameters=common_params,
            ),
        ]
    )
