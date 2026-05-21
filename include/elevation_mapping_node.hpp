#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include "dds_height_map_publisher.hpp"
#include "elevation_map_backend.hpp"
#include "height_map_model.hpp"
#include "autonomy/msg/autonomy_state.hpp"
#include "core/msg/command_filter.hpp"
#include "autonomy/msg/masked_height_scan.hpp"

namespace autonomy
{

class ElevationMappingNode final : public rclcpp::Node
{
public:
  explicit ElevationMappingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void loadParameters();
  void createIo();
  void publishHeartbeat();
  void onAutonomyState(autonomy::msg::AutonomyState::SharedPtr msg);
  void onCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publishCommandFilter();
  [[nodiscard]] bool processingActive() const;
  [[nodiscard]] core::msg::CommandFilter evaluateCommandFilter(
    double move_forward,
    double move_right) const;
  void fillDebugGrid(ElevationGrid & grid) const;
  [[nodiscard]] sensor_msgs::msg::PointCloud2 gridToPointCloud(const ElevationGrid & grid) const;
  [[nodiscard]] bool isPathClear(
    const HeightMapFrame & frame,
    double x_min,
    double x_max,
    double y_min,
    double y_max) const;

  std::string input_cloud_topic_{"pointcloud_merge_node/merged_points"};
  std::string output_image_topic_{"~/elevation_image"};
  std::string output_cloud_topic_{"~/elevation_points"};
  std::string output_masked_height_scan_topic_{"~/masked_height_scan"};
  std::string operation_mode_{"drive"};
  std::string command_filter_topic_{"/command_filter"};
  std::string output_local_terrain_image_topic_{"~/local_terrain_image"};
  std::string output_local_terrain_cloud_topic_{"~/local_terrain_points"};
  std::string output_local_terrain_scan_topic_{"~/local_terrain_map"};
  bool local_terrain_map_config_enabled_{true};
  bool local_terrain_map_enabled_{false};
  bool respect_autonomy_mode_{false};
  bool has_autonomy_state_{false};
  int8_t autonomy_mode_{autonomy::msg::AutonomyState::IDLE};
  std::string autonomy_status_topic_{"/autonomy_manager/status"};
  bool dds_height_map_enabled_{true};
  int dds_domain_id_{0};
  std::string dds_height_map_topic_{"height_map"};
  std::string dds_height_map_type_{"core_dds::HeightMap"};
  GridSpec grid_spec_;
  GridSpec local_terrain_grid_spec_;
  double height_scan_offset_{0.5};
  double base_height_{0.5};
  double obstacle_floor_z_{-0.47957};
  double obstacle_height_threshold_{0.25};
  double forward_lateral_half_width_{0.25};
  double right_longitudinal_half_width_{0.25};
  double command_filter_forward_distance_{0.0};
  double command_filter_lateral_distance_{0.0};
  double command_filter_publish_rate_hz_{10.0};
  bool fill_debug_outputs_{true};
  double debug_fill_z_{0.0};
  std::chrono::steady_clock::time_point fps_window_start_;
  std::size_t fps_frame_count_{0};
  mutable std::mutex latest_height_map_mutex_;
  HeightMapFrame latest_height_map_;
  bool has_latest_height_map_{false};

  rclcpp::Subscription<autonomy::msg::AutonomyState>::SharedPtr autonomy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr elevation_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr elevation_cloud_pub_;
  rclcpp::Publisher<autonomy::msg::MaskedHeightScan>::SharedPtr masked_height_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr local_terrain_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_terrain_cloud_pub_;
  rclcpp::Publisher<autonomy::msg::MaskedHeightScan>::SharedPtr local_terrain_scan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<core::msg::CommandFilter>::SharedPtr command_filter_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr command_filter_timer_;
  std::unique_ptr<ElevationMapBackend> elevation_backend_;
  std::unique_ptr<ElevationMapBackend> local_terrain_backend_;
  std::unique_ptr<DdsHeightMapPublisher> dds_height_map_pub_;
};

}  // namespace autonomy
