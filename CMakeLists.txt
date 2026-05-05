cmake_minimum_required(VERSION 3.16)
project(height_map_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MaskedHeightScan.msg"
  DEPENDENCIES std_msgs
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
  src/elevation_grid.cpp
  src/elevation_mapping_main.cpp
  src/elevation_mapping_node.cpp
  src/min_z_elevation_backend.cpp
)

target_include_directories(elevation_mapping_node
  PRIVATE
    include
)

ament_target_dependencies(elevation_mapping_node
  rclcpp
  sensor_msgs
  std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(elevation_mapping_node "${cpp_typesupport_target}")

install(TARGETS
  pointcloud_merge_node
  elevation_mapping_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/generate_realsense_mapping_from_urdf.py
  scripts/realsense_serial_mapper.py
  scripts/show_realsense_serials.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
