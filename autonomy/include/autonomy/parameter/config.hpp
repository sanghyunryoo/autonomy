#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "autonomy/algorithm/detection/rgbd_detector.hpp"
#include "autonomy/algorithm/elevation/elevation_mapper.hpp"
#include "autonomy/algorithm/elevation/occupancy_mapper.hpp"
#include "autonomy/algorithm/planner/types.hpp"
#include "autonomy/algorithm/slam/super_lio.hpp"

namespace autonomy {

/** One USB 3 RealSense device, acquired directly through librealsense2. */
struct CameraConfig {
  std::string id{};
  std::string model{};
  std::string serial{};
  std::string port{};
  std::string role{};
  std::string frame{};
  std::uint32_t width{640};
  std::uint32_t height{480};
  std::uint32_t fps{30};
  bool enable_imu{false};
  bool enabled{true};
  std::chrono::milliseconds timeout{500};
};

/** One Livox MID-360/MID-360S acquired directly through Livox SDK2. */
struct LidarConfig {
  std::string id{};
  std::string model{};
  std::string ip{};
  std::string role{};
  std::string frame{};
  std::string sdk_config_path{};
  bool enabled{true};
  std::chrono::milliseconds timeout{500};
};

struct StateManagerConfig {
  std::chrono::milliseconds robot_report_timeout{500};
  bool require_robot_report{true};
};

struct DdsConfig {
  bool enabled{true};
  std::uint32_t domain_id{0};
  std::string height_map_topic{"autonomy/height_map"};
  std::string linear_velocity_topic{"autonomy/linear_velocity"};
};

struct Ros2DebugConfig {
  bool enabled{false};
  double hz{2.0};
  std::uint32_t max_points_per_cloud{2000};
  std::uint32_t max_path_poses{200};
  std::uint32_t max_image_width{320};
  std::uint32_t max_image_height{240};
  std::string elevation_points_topic{"/autonomy/debug/elevation_points"};
  std::string sensor_tf_topic{"/autonomy/debug/sensor_tf"};
  std::string slam_odom_topic{"/autonomy/debug/slam/odom"};
  std::string slam_path_topic{"/autonomy/debug/slam/path"};
  std::string livox_points_topic{"/autonomy/debug/livox_points"};
  std::string ai_rgb_topic{"/autonomy/debug/ai_realsense/rgb"};
  std::string ai_bbox_topic{"/autonomy/debug/ai_realsense/bbox"};
};

struct Ros2Config {
  std::string robot_report_topic{"/robot_report"};
  std::string command_topic{"/control_command/autopilot"};
  double autopilot_hz{10.0};
  std::string command_filter_topic{"/command_filter"};
  std::string navigation_goal_topic{"/autonomy/navigation_goal"};
  Ros2DebugConfig debug{};
};

struct MiddlewareConfig {
  Ros2Config ros2{};
  DdsConfig dds{};
};

struct NetworkConfig {
  std::string lidar_local_ip{"192.168.1.50"};
  std::string sbc_local_ip{"192.168.20.2"};
  std::string sbc_peer_ip{"192.168.20.1"};
};

struct RuntimeConfig {
  double loop_hz{50.0};
  std::string base_frame{"base_link"};
  std::string target_frame{"base_link"};
  std::string urdf_path{};
  StateManagerConfig state{};
  ElevationConfig elevation{};
  SlamConfig slam{};
  MappingConfig mapping{};
  PlannerConfig planner{};
  DetectionConfig detection{};
  MiddlewareConfig middleware{};
  NetworkConfig network{};
  std::vector<CameraConfig> cameras{};
  std::vector<LidarConfig> lidars{};
};

}  // namespace autonomy
