cmake_minimum_required(VERSION 3.16)

project(height_map_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(ExternalProject)

set(THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party" CACHE PATH "Third-party source root")
set(ORB_SLAM3_ROOT "${THIRD_PARTY_DIR}/orb_slam3" CACHE PATH "ORB-SLAM3 source/build root")
set(PANGOLIN_ROOT "${THIRD_PARTY_DIR}/pangolin" CACHE PATH "Pangolin source root")
set(ONNXRUNTIME_ROOT "${THIRD_PARTY_DIR}/onnxruntime" CACHE PATH "ONNX Runtime binary distribution root")
set(THIRD_PARTY_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/third_party_build")
set(THIRD_PARTY_INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/third_party_install")
set(PANGOLIN_INSTALL_DIR "${THIRD_PARTY_INSTALL_DIR}/pangolin")
set(PANGOLIN_CMAKE_DIR "${PANGOLIN_INSTALL_DIR}/lib/cmake/Pangolin")

if(NOT EXISTS "${ORB_SLAM3_ROOT}/CMakeLists.txt")
  message(FATAL_ERROR "ORB-SLAM3 source not found at ${ORB_SLAM3_ROOT}")
endif()

if(NOT EXISTS "${PANGOLIN_ROOT}/CMakeLists.txt")
  message(FATAL_ERROR "Pangolin source not found at ${PANGOLIN_ROOT}")
endif()

if(NOT EXISTS "${ONNXRUNTIME_ROOT}/include/onnxruntime_cxx_api.h")
  message(FATAL_ERROR "ONNX Runtime headers not found at ${ONNXRUNTIME_ROOT}/include")
endif()

if(NOT EXISTS "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")
  message(FATAL_ERROR "ONNX Runtime library not found at ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")
endif()

find_package(ament_cmake REQUIRED)
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS OpenGL)
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
  "srv/CommandFilter.srv"
  "srv/SetAutonomyMode.srv"
  "srv/SetEstop.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ExternalProject_Add(pangolin_external
  SOURCE_DIR "${PANGOLIN_ROOT}"
  BINARY_DIR "${THIRD_PARTY_BUILD_DIR}/pangolin"
  INSTALL_DIR "${PANGOLIN_INSTALL_DIR}"
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_INSTALL_PREFIX=${PANGOLIN_INSTALL_DIR}
  CMAKE_CACHE_ARGS
    -DBUILD_EXAMPLES:BOOL=OFF
    -DBUILD_TESTS:BOOL=OFF
    -DBUILD_PANGOLIN_PYTHON:BOOL=OFF
)

ExternalProject_Add(dbow2_external
  SOURCE_DIR "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2"
  BINARY_DIR "${THIRD_PARTY_BUILD_DIR}/dbow2"
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=Release
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -j4
  INSTALL_COMMAND ""
)

ExternalProject_Add(orb_slam3_external
  DEPENDS pangolin_external dbow2_external
  SOURCE_DIR "${ORB_SLAM3_ROOT}"
  BINARY_DIR "${THIRD_PARTY_BUILD_DIR}/orb_slam3"
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=Release
    -DPangolin_DIR=${PANGOLIN_CMAKE_DIR}
    -DCMAKE_PREFIX_PATH=${PANGOLIN_INSTALL_DIR}
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --target ORB_SLAM3 -j4
  INSTALL_COMMAND ""
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
  rclcpp
  sensor_msgs
  std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(elevation_mapping_node "${cpp_typesupport_target}")
target_link_libraries(elevation_mapping_node fastcdr fastrtps)

add_executable(autonomy_manager_node
  src/autonomy_manager_node.cpp
)
target_include_directories(autonomy_manager_node PRIVATE include)
ament_target_dependencies(autonomy_manager_node geometry_msgs nav_msgs rclcpp std_msgs)
target_link_libraries(autonomy_manager_node "${cpp_typesupport_target}")

add_executable(rl_local_planner_node
  src/rl_local_planner_node.cpp
)
target_include_directories(rl_local_planner_node PRIVATE include)
ament_target_dependencies(rl_local_planner_node geometry_msgs rclcpp std_msgs)
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

add_executable(orbslam3_node
  src/orbslam3_node.cpp
)
target_include_directories(orbslam3_node
  PRIVATE
    include
    ${ORB_SLAM3_ROOT}
    ${ORB_SLAM3_ROOT}/include
    ${ORB_SLAM3_ROOT}/include/CameraModels
    ${ORB_SLAM3_ROOT}/Thirdparty/Sophus
    ${PANGOLIN_INSTALL_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)
target_compile_definitions(orbslam3_node
  PRIVATE
    COMPILEDWITHC11
    HAVE_EIGEN
    HAVE_EPOXY
    PANGO_DEFAULT_WIN_URI="x11"
    _LINUX_
)
ament_target_dependencies(orbslam3_node
  cv_bridge
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
)
target_link_libraries(orbslam3_node
  ${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so
  ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so
  ${OpenCV_LIBS}
  pango_core
  pango_display
  pango_geometry
  pango_glgeometry
  pango_image
  pango_opengl
  pango_packetstream
  pango_plot
  pango_scene
  pango_tools
  pango_vars
  pango_video
  pango_windowing
  tinyobj
  OpenGL::OpenGL
  epoxy
)
target_link_directories(orbslam3_node PRIVATE ${PANGOLIN_INSTALL_DIR}/lib)
add_dependencies(orbslam3_node orb_slam3_external)
target_link_options(orbslam3_node PRIVATE "-Wl,--disable-new-dtags")
set_target_properties(orbslam3_node PROPERTIES
  BUILD_RPATH "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib;${PANGOLIN_INSTALL_DIR}/lib"
  INSTALL_RPATH "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib;${PANGOLIN_INSTALL_DIR}/lib"
)

add_executable(global_planner_node
  src/global_planner_node.cpp
)
target_include_directories(global_planner_node PRIVATE include)
ament_target_dependencies(global_planner_node geometry_msgs nav_msgs rclcpp std_msgs)

install(TARGETS
  ai_detection_node
  autonomy_manager_node
  pointcloud_merge_node
  elevation_mapping_node
  global_planner_node
  orbslam3_node
  rl_local_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY weights
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY map
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY idl
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
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
