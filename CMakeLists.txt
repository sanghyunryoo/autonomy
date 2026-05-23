cmake_minimum_required(VERSION 3.16)

project(autonomy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party" CACHE PATH "Third-party source root")
set(ONNXRUNTIME_ROOT "${THIRD_PARTY_DIR}/onnxruntime" CACHE PATH "ONNX Runtime binary distribution root")

if(NOT EXISTS "${ONNXRUNTIME_ROOT}/include/onnxruntime_cxx_api.h")
  message(FATAL_ERROR "ONNX Runtime headers not found at ${ONNXRUNTIME_ROOT}/include")
endif()

if(NOT EXISTS "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")
  message(FATAL_ERROR "ONNX Runtime library not found at ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")
endif()

find_package(ament_cmake REQUIRED)
find_package(core REQUIRED)
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
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
target_link_libraries(elevation_mapping_node fastcdr fastrtps)

add_executable(autonomy_manager_node
  src/autonomy_manager_node.cpp
)
target_include_directories(autonomy_manager_node PRIVATE include)
ament_target_dependencies(autonomy_manager_node core geometry_msgs nav_msgs rclcpp std_msgs)
target_link_libraries(autonomy_manager_node "${cpp_typesupport_target}")

add_executable(rl_local_planner_node
  src/rl_local_planner_node.cpp
)
target_include_directories(rl_local_planner_node PRIVATE include)
ament_target_dependencies(rl_local_planner_node core geometry_msgs nav_msgs rclcpp std_msgs)
target_link_libraries(rl_local_planner_node "${cpp_typesupport_target}")

add_executable(ai_detection_node
  src/ai_detection_node.cpp
)
target_include_directories(ai_detection_node PRIVATE include ${ONNXRUNTIME_ROOT}/include)
ament_target_dependencies(ai_detection_node cv_bridge rclcpp sensor_msgs std_msgs)
target_link_libraries(
  ai_detection_node
  "${cpp_typesupport_target}"
  ${OpenCV_LIBS}
  ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so
)
set_target_properties(ai_detection_node PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib"
  INSTALL_RPATH "${ONNXRUNTIME_ROOT}/lib"
)

add_executable(vio_pose_adapter_node
  src/vio_pose_adapter_node.cpp
)
target_include_directories(vio_pose_adapter_node PRIVATE include)
ament_target_dependencies(vio_pose_adapter_node geometry_msgs nav_msgs rclcpp std_msgs)
target_link_libraries(vio_pose_adapter_node "${cpp_typesupport_target}")

add_executable(global_planner_node
  src/global_planner_node.cpp
)
target_include_directories(global_planner_node PRIVATE include)
ament_target_dependencies(global_planner_node geometry_msgs nav_msgs rclcpp std_msgs)

add_executable(livox_monitor_node
  src/livox_monitor_node.cpp
)
target_include_directories(livox_monitor_node PRIVATE include)
ament_target_dependencies(livox_monitor_node rclcpp sensor_msgs std_msgs)

add_executable(point_lio_monitor_node
  src/point_lio_monitor_node.cpp
)
target_include_directories(point_lio_monitor_node PRIVATE include)
ament_target_dependencies(point_lio_monitor_node nav_msgs rclcpp std_msgs)

add_executable(point_lio_map_saver_node
  src/point_lio_map_saver_node.cpp
)
target_include_directories(point_lio_map_saver_node PRIVATE include)
ament_target_dependencies(point_lio_map_saver_node nav_msgs rclcpp sensor_msgs std_msgs)
target_link_libraries(point_lio_map_saver_node "${cpp_typesupport_target}")

add_executable(tracking_follower_node
  src/tracking_follower_node.cpp
)
target_include_directories(tracking_follower_node PRIVATE include)
ament_target_dependencies(tracking_follower_node geometry_msgs rclcpp std_msgs)
target_link_libraries(tracking_follower_node "${cpp_typesupport_target}")

install(TARGETS
  ai_detection_node
  autonomy_manager_node
  pointcloud_merge_node
  elevation_mapping_node
  global_planner_node
  livox_monitor_node
  rl_local_planner_node
  point_lio_monitor_node
  point_lio_map_saver_node
  tracking_follower_node
  vio_pose_adapter_node
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
  scripts/generate_realsense_mapping_from_urdf.py
  scripts/generate_height_filter_from_urdf.py
  scripts/realsense_usb_mapper.py
  scripts/realsense_serial_mapper.py
  scripts/show_realsense_usb_ports.py
  scripts/show_realsense_serials.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
