cmake_minimum_required(VERSION 3.16)

project(autonomy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(core REQUIRED)
find_package(CycloneDDS REQUIRED CONFIG)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutonomyState.msg"
  "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg"
  "msg/MaskedHeightScan.msg"
  "srv/SetAutonomyMode.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

idlc_generate(
  TARGET autonomy_dds_types
  FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/resources/idl/HeightMap.idl"
    "${CMAKE_CURRENT_SOURCE_DIR}/resources/idl/LinearVelocity.idl"
)

add_executable(pointcloud_merge_node
  src/pointcloud_merge_main.cpp
  src/pointcloud_merger_node.cpp
)

target_include_directories(pointcloud_merge_node
  PRIVATE
    include
)

ament_target_dependencies(pointcloud_merge_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(elevation_mapping_node
  src/dds_height_map_publisher.cpp
  src/elevation_grid.cpp
  src/elevation_mapping_main.cpp
  src/elevation_mapping_node.cpp
  src/height_map_model.cpp
  src/min_z_elevation_backend.cpp
)

target_include_directories(elevation_mapping_node
  PRIVATE
    include
)

ament_target_dependencies(elevation_mapping_node
  core
  rclcpp
  sensor_msgs
  std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(pointcloud_merge_node "${cpp_typesupport_target}")
target_link_libraries(elevation_mapping_node "${cpp_typesupport_target}")
target_link_libraries(elevation_mapping_node autonomy_dds_types CycloneDDS::ddsc)

add_executable(drive_mapper_node
  src/drive_mapper_node.cpp
)
ament_target_dependencies(drive_mapper_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(autonomy_manager_node
  src/autonomy_manager_node.cpp
)
target_include_directories(autonomy_manager_node PRIVATE include)
ament_target_dependencies(autonomy_manager_node core geometry_msgs nav_msgs rclcpp std_msgs)
target_link_libraries(autonomy_manager_node "${cpp_typesupport_target}")

add_executable(global_planner_node
  src/global_planner_node.cpp
)
ament_target_dependencies(global_planner_node geometry_msgs nav_msgs rclcpp std_msgs)

add_executable(global_costmap_node
  src/global_costmap_node.cpp
)
ament_target_dependencies(global_costmap_node nav_msgs rclcpp sensor_msgs std_msgs)

add_executable(local_costmap_node
  src/local_costmap_node.cpp
)
ament_target_dependencies(local_costmap_node nav_msgs rclcpp std_msgs tf2 tf2_geometry_msgs tf2_ros)
target_link_libraries(local_costmap_node "${cpp_typesupport_target}")

add_executable(local_planner_node
  src/local_planner_node.cpp
  src/dds_linear_velocity_publisher.cpp
)
target_include_directories(local_planner_node PRIVATE include)
ament_target_dependencies(local_planner_node core geometry_msgs nav_msgs rclcpp std_msgs)
target_link_libraries(local_planner_node "${cpp_typesupport_target}" autonomy_dds_types CycloneDDS::ddsc)

add_executable(point_lio_lidar_adapter_node
  src/point_lio_lidar_adapter_node.cpp
)
ament_target_dependencies(point_lio_lidar_adapter_node rclcpp sensor_msgs std_msgs)

add_executable(point_lio_monitor_node
  src/point_lio_monitor_node.cpp
)
ament_target_dependencies(point_lio_monitor_node nav_msgs rclcpp sensor_msgs std_msgs)

install(TARGETS
  autonomy_manager_node
  drive_mapper_node
  global_costmap_node
  global_planner_node
  local_costmap_node
  local_planner_node
  point_lio_lidar_adapter_node
  point_lio_monitor_node
  pointcloud_merge_node
  elevation_mapping_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch resources
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/build.sh
  scripts/launch.sh
  scripts/run_keyboard_filter_odom_dds_test.sh
  scripts/run_dds_echo_heightmap_linear_velocity.sh
  scripts/generate_realsense_mapping_from_urdf.py
  scripts/generate_height_filter_from_urdf.py
  scripts/show_realsense_usb_ports.py
  scripts/show_realsense_serials.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/dds_echo_heightmap_linear_velocity.cpp
  scripts/dds_heightmap_linear_velocity_test.cpp
  scripts/keyboard_filter_odom_dds_test.cpp
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
