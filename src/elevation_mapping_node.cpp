#include "height_map_ros2/elevation_mapping_node.hpp"

#include <cmath>
#include <utility>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "height_map_ros2/min_z_elevation_backend.hpp"

namespace height_map_ros2
{

ElevationMappingNode::ElevationMappingNode(const rclcpp::NodeOptions & options)
: Node("elevation_mapping_node", options)
{
  loadParameters();
  elevation_backend_ = std::make_unique<MinZElevationBackend>(grid_spec_);
  createIo();
}

void ElevationMappingNode::loadParameters()
{
  input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", input_cloud_topic_);
  output_image_topic_ = declare_parameter<std::string>("output_image_topic", output_image_topic_);
  output_cloud_topic_ = declare_parameter<std::string>("output_cloud_topic", output_cloud_topic_);

  grid_spec_.resolution = declare_parameter<double>("grid.resolution", grid_spec_.resolution);
  grid_spec_.x_min = declare_parameter<double>("grid.x_min", grid_spec_.x_min);
  grid_spec_.x_max = declare_parameter<double>("grid.x_max", grid_spec_.x_max);
  grid_spec_.y_min = declare_parameter<double>("grid.y_min", grid_spec_.y_min);
  grid_spec_.y_max = declare_parameter<double>("grid.y_max", grid_spec_.y_max);
  grid_spec_.min_z = declare_parameter<double>("grid.min_z", grid_spec_.min_z);
  grid_spec_.max_z = declare_parameter<double>("grid.max_z", grid_spec_.max_z);

  // Declared here as the stable ROS2 parameter surface for the production
  // backend that will wrap the existing realsense_points_test algorithm.
  declare_parameter<int>("algorithm.point_stride", 1);
  declare_parameter<double>("algorithm.max_range", 2.5);
  declare_parameter<bool>("algorithm.print_frame_info", false);
  declare_parameter<double>("algorithm.height_offset_base", 0.0);

  declare_parameter<double>("algorithm.uncertainty.noise_alpha", 0.001);
  declare_parameter<double>("algorithm.uncertainty.min_meas_var", 0.0004);
  declare_parameter<double>("algorithm.uncertainty.init_cell_var", 0.01);
  declare_parameter<double>("algorithm.uncertainty.min_cell_var", 0.0004);
  declare_parameter<double>("algorithm.uncertainty.max_cell_var", 0.25);
  declare_parameter<double>("algorithm.uncertainty.time_var_rate", 0.2);
  declare_parameter<double>("algorithm.uncertainty.mahalanobis_thresh", 3.0);
  declare_parameter<double>("algorithm.uncertainty.dyn_env_var_bump", 0.0225);
  declare_parameter<double>("algorithm.uncertainty.dyn_reset_delta", 0.1);

  declare_parameter<double>("algorithm.frame_aggregation.robust_height_gate", 0.04);
  declare_parameter<double>("algorithm.frame_aggregation.intra_cell_min_support_gap", 0.025);
  declare_parameter<int>("algorithm.frame_aggregation.intra_cell_min_support_count", 3);
  declare_parameter<double>("algorithm.frame_aggregation.edge_mix_height_diff", 0.035);
  declare_parameter<int>("algorithm.frame_aggregation.edge_prefer_prev_support_count", 2);

  declare_parameter<int>("algorithm.isolated_filter.radius", 1);
  declare_parameter<int>("algorithm.isolated_filter.min_support_neighbors", 2);
  declare_parameter<double>("algorithm.isolated_filter.support_height_diff", 0.025);
  declare_parameter<double>("algorithm.isolated_filter.outlier_height_diff", 0.05);
  declare_parameter<int>("algorithm.isolated_filter.every_n_frames", 2);

  declare_parameter<int>("algorithm.hole_fill.radius", 1);
  declare_parameter<int>("algorithm.hole_fill.min_neighbors", 3);
  declare_parameter<double>("algorithm.hole_fill.max_height_diff", 0.03);

  declare_parameter<int>("algorithm.bilateral.radius", 1);
  declare_parameter<double>("algorithm.bilateral.sigma_spatial", 1.1);
  declare_parameter<double>("algorithm.bilateral.sigma_height", 0.025);
  declare_parameter<double>("algorithm.bilateral.max_height_diff", 0.04);
  declare_parameter<int>("algorithm.bilateral.passes", 2);
  declare_parameter<int>("algorithm.bilateral.every_n_frames", 2);

  declare_parameter<int>("algorithm.runtime.min_runtime_fps", 60);
}

void ElevationMappingNode::createIo()
{
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_,
    rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      onCloud(std::move(msg));
    });

  elevation_image_pub_ = create_publisher<sensor_msgs::msg::Image>(output_image_topic_, 10);
  elevation_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    output_cloud_topic_, rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile());

  RCLCPP_INFO(get_logger(), "Subscribing merged cloud: %s", input_cloud_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Publishing elevation image: %s", output_image_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Publishing elevation points: %s", output_cloud_topic_.c_str());
}

void ElevationMappingNode::onCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto grid = elevation_backend_->build(*msg, msg->header);
  elevation_image_pub_->publish(grid.toImageMsg());
  elevation_cloud_pub_->publish(gridToPointCloud(grid));
}

sensor_msgs::msg::PointCloud2 ElevationMappingNode::gridToPointCloud(const ElevationGrid & grid) const
{
  std::size_t valid_count = 0;
  for (const auto height : grid.height) {
    if (std::isfinite(height)) {
      ++valid_count;
    }
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = grid.header;
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(valid_count);

  sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y_it(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z_it(cloud, "z");

  const auto width = grid.spec.width();
  for (std::uint32_t row = 0; row < grid.spec.height(); ++row) {
    for (std::uint32_t col = 0; col < width; ++col) {
      const auto index = static_cast<std::size_t>(row) * width + col;
      const auto z = grid.height[index];
      if (!std::isfinite(z)) {
        continue;
      }

      *x_it = static_cast<float>(grid.spec.x_min + (col + 0.5) * grid.spec.resolution);
      *y_it = static_cast<float>(grid.spec.y_min + (row + 0.5) * grid.spec.resolution);
      *z_it = z;
      ++x_it;
      ++y_it;
      ++z_it;
    }
  }

  return cloud;
}

}  // namespace height_map_ros2
