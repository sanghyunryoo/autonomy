#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "elevation_map_backend.hpp"
#include "height_map_ros2/msg/masked_height_scan.hpp"

namespace height_map_ros2
{

class ElevationMappingNode final : public rclcpp::Node
{
public:
  explicit ElevationMappingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void loadParameters();
  void createIo();
  void onCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void fillDebugGrid(ElevationGrid & grid) const;
  [[nodiscard]] height_map_ros2::msg::MaskedHeightScan gridToMaskedHeightScan(
    const ElevationGrid & grid) const;
  [[nodiscard]] sensor_msgs::msg::PointCloud2 gridToPointCloud(const ElevationGrid & grid) const;

  std::string input_cloud_topic_{"pointcloud_merge_node/merged_points"};
  std::string output_image_topic_{"~/elevation_image"};
  std::string output_cloud_topic_{"~/elevation_points"};
  std::string output_masked_height_scan_topic_{"~/masked_height_scan"};
  GridSpec grid_spec_;
  double height_scan_offset_{0.5};
  double base_height_{0.5};
  bool fill_debug_outputs_{true};
  double debug_fill_z_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr elevation_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr elevation_cloud_pub_;
  rclcpp::Publisher<height_map_ros2::msg::MaskedHeightScan>::SharedPtr masked_height_scan_pub_;
  std::unique_ptr<ElevationMapBackend> elevation_backend_;
};

}  // namespace height_map_ros2
