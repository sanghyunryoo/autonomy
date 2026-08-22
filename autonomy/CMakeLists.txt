cmake_minimum_required(VERSION 3.16)

project(autonomy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(core REQUIRED)
find_package(CycloneDDS REQUIRED CONFIG)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Glog REQUIRED)
find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED)
find_package(TBB REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(realsense2 REQUIRED)
find_library(TINYXML2_LIBRARY NAMES tinyxml2 REQUIRED)
find_path(LIVOX_SDK2_INCLUDE_DIR NAMES livox_lidar_api.h REQUIRED)
find_library(LIVOX_SDK2_LIBRARY NAMES
  livox_lidar_sdk_shared
  livox_lidar_sdk_static
  livox_lidar_sdk
  REQUIRED)

if(TARGET realsense2::realsense2)
  set(REALSENSE2_TARGET realsense2::realsense2)
elseif(TARGET realsense2)
  set(REALSENSE2_TARGET realsense2)
elseif(DEFINED realsense2_LIBRARY)
  set(REALSENSE2_TARGET ${realsense2_LIBRARY})
else()
  message(FATAL_ERROR "realsense2 was found without an exported CMake library target")
endif()

# Local ROS interfaces intentionally live inside src/middleware: they are
# transport contracts, not part of the ROS-free manager core.
rosidl_generate_interfaces(${PROJECT_NAME}
  "src/middleware/msg/AutonomyState.msg"
  "src/middleware/srv/SetAutonomyMode.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

idlc_generate(
  TARGET autonomy_dds_types
  FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/middleware/idl/HeightMap.idl"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/middleware/idl/LinearVelocity.idl"
)

# Manager core: deliberately no rclcpp, ROS messages, core-msg, or DDS types.
add_library(autonomy_runtime_core
  src/algorithm/slam/super_lio.cpp
  src/algorithm/detection/rgbd_detector.cpp
  src/algorithm/elevation/elevation_mapper.cpp
  src/algorithm/elevation/occupancy_mapper.cpp
  src/algorithm/planner/hybrid_astar.cpp
  src/algorithm/planner/mppi.cpp
  src/algorithm/algorithm_manager.cpp
  src/parameter/parameter_manager.cpp
  src/sensor/livox_driver.cpp
  src/sensor/realsense_driver.cpp
  src/sensor/sensor_manager.cpp
  src/sensor/urdf_model.cpp
  src/core/loop_timer.cpp
  src/core/state_manager.cpp
  src/core/types.cpp
)
target_include_directories(autonomy_runtime_core PUBLIC include ${PCL_INCLUDE_DIRS}
  PRIVATE ${LIVOX_SDK2_INCLUDE_DIR})
target_link_libraries(autonomy_runtime_core PUBLIC
  yaml-cpp
  ${TINYXML2_LIBRARY}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  gtsam
  glog
  TBB::tbb
  ${REALSENSE2_TARGET}
  ${LIVOX_SDK2_LIBRARY}
)

# MiddlewareManager is the one and only ROS/CycloneDDS/core-msg boundary.
add_executable(autonomy_runtime_node
  src/main.cpp
  src/middleware/dds_height_map_publisher.cpp
  src/middleware/dds_linear_velocity_publisher.cpp
  src/middleware/middleware_manager.cpp
  src/core/runtime_runner.cpp
)
target_include_directories(autonomy_runtime_node PRIVATE include)
ament_target_dependencies(autonomy_runtime_node
  builtin_interfaces
  core
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_msgs
)
target_link_libraries(autonomy_runtime_node autonomy_runtime_core autonomy_dds_types CycloneDDS::ddsc)

# rosidl_get_typesupport_target was added after Foxy. Keep the package source
# buildable on both Foxy and Humble while Jetson deployment targets Humble.
if(COMMAND rosidl_get_typesupport_target)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(autonomy_runtime_node "${cpp_typesupport_target}")
else()
  rosidl_target_interfaces(autonomy_runtime_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

install(TARGETS autonomy_runtime_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS autonomy_runtime_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY
  include/autonomy/parameter/config
  include/autonomy/parameter/urdf
  DESTINATION share/${PROJECT_NAME}/parameter
)
install(PROGRAMS
  scripts/build.sh
  scripts/launch.sh
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES LICENSE NOTICE
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
