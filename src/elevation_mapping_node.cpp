#include "elevation_mapping_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "min_z_elevation_backend.hpp"

namespace autonomy
{

ElevationMappingNode::ElevationMappingNode(const rclcpp::NodeOptions & options)
: Node("elevation_mapping_node", options)
{
  loadParameters();
  elevation_backend_ = std::make_unique<MinZElevationBackend>(grid_spec_);
  if (local_terrain_map_enabled_) {
    local_terrain_backend_ = std::make_unique<MinZElevationBackend>(local_terrain_grid_spec_);
  }
  createIo();
}

void ElevationMappingNode::loadParameters()
{
  input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", input_cloud_topic_);
  output_image_topic_ = declare_parameter<std::string>("output_image_topic", output_image_topic_);
  output_cloud_topic_ = declare_parameter<std::string>("output_cloud_topic", output_cloud_topic_);
  output_masked_height_scan_topic_ = declare_parameter<std::string>(
    "output_masked_height_scan_topic", output_masked_height_scan_topic_);
  operation_mode_ = declare_parameter<std::string>("operation_mode", operation_mode_);
  std::transform(operation_mode_.begin(), operation_mode_.end(), operation_mode_.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (operation_mode_ != "drive" && operation_mode_ != "adas" && operation_mode_ != "fsd") {
    throw std::invalid_argument("operation_mode must be one of: drive, adas, fsd");
  }

  command_filter_topic_ = declare_parameter<std::string>(
    "command_filter_topic", command_filter_topic_);
  output_local_terrain_image_topic_ = declare_parameter<std::string>(
    "local_terrain_map.output_image_topic", output_local_terrain_image_topic_);
  output_local_terrain_cloud_topic_ = declare_parameter<std::string>(
    "local_terrain_map.output_cloud_topic", output_local_terrain_cloud_topic_);
  output_local_terrain_scan_topic_ = declare_parameter<std::string>(
    "local_terrain_map.output_scan_topic", output_local_terrain_scan_topic_);
  local_terrain_map_config_enabled_ = declare_parameter<bool>(
    "local_terrain_map.enabled", local_terrain_map_config_enabled_);
  respect_autonomy_mode_ = declare_parameter<bool>(
    "respect_autonomy_mode", respect_autonomy_mode_);
  autonomy_status_topic_ = declare_parameter<std::string>(
    "autonomy_status_topic", autonomy_status_topic_);

  dds_height_map_enabled_ = declare_parameter<bool>("dds.height_map.enabled", dds_height_map_enabled_);
  dds_domain_id_ = declare_parameter<int>("dds.height_map.domain_id", dds_domain_id_);
  dds_height_map_topic_ = declare_parameter<std::string>(
    "dds.height_map.topic", dds_height_map_topic_);
  dds_height_map_type_ = declare_parameter<std::string>(
    "dds.height_map.type", dds_height_map_type_);

  grid_spec_.resolution = declare_parameter<double>("grid.resolution", grid_spec_.resolution);
  grid_spec_.x_min = declare_parameter<double>("grid.x_min", grid_spec_.x_min);
  grid_spec_.x_max = declare_parameter<double>("grid.x_max", grid_spec_.x_max);
  grid_spec_.y_min = declare_parameter<double>("grid.y_min", grid_spec_.y_min);
  grid_spec_.y_max = declare_parameter<double>("grid.y_max", grid_spec_.y_max);
  grid_spec_.min_z = declare_parameter<double>("grid.min_z", grid_spec_.min_z);
  grid_spec_.max_z = declare_parameter<double>("grid.max_z", grid_spec_.max_z);

  local_terrain_grid_spec_ = grid_spec_;
  local_terrain_grid_spec_.x_max = std::max(grid_spec_.x_max, 2.0);
  local_terrain_grid_spec_.resolution = declare_parameter<double>(
    "local_terrain_map.grid.resolution", local_terrain_grid_spec_.resolution);
  local_terrain_grid_spec_.x_min = declare_parameter<double>(
    "local_terrain_map.grid.x_min", local_terrain_grid_spec_.x_min);
  local_terrain_grid_spec_.x_max = declare_parameter<double>(
    "local_terrain_map.grid.x_max", local_terrain_grid_spec_.x_max);
  local_terrain_grid_spec_.y_min = declare_parameter<double>(
    "local_terrain_map.grid.y_min", local_terrain_grid_spec_.y_min);
  local_terrain_grid_spec_.y_max = declare_parameter<double>(
    "local_terrain_map.grid.y_max", local_terrain_grid_spec_.y_max);
  local_terrain_grid_spec_.min_z = declare_parameter<double>(
    "local_terrain_map.grid.min_z", local_terrain_grid_spec_.min_z);
  local_terrain_grid_spec_.max_z = declare_parameter<double>(
    "local_terrain_map.grid.max_z", local_terrain_grid_spec_.max_z);
  local_terrain_map_enabled_ =
    local_terrain_map_config_enabled_ && (operation_mode_ == "adas" || operation_mode_ == "fsd");

  // Declared here as the stable ROS2 parameter surface for the production
  // backend that will wrap the existing realsense_points_test algorithm.
  declare_parameter<int>("algorithm.point_stride", 1);
  declare_parameter<double>("algorithm.max_range", 2.5);
  declare_parameter<bool>("algorithm.print_frame_info", false);
  height_scan_offset_ = declare_parameter<double>("algorithm.height_scan_offset", height_scan_offset_);
  base_height_ = declare_parameter<double>("algorithm.base_height", base_height_);
  obstacle_floor_z_ = declare_parameter<double>("command_filter.obstacle_floor_z", obstacle_floor_z_);
  obstacle_height_threshold_ = declare_parameter<double>(
    "command_filter.obstacle_height_threshold", obstacle_height_threshold_);
  forward_lateral_half_width_ = declare_parameter<double>(
    "command_filter.forward_lateral_half_width", forward_lateral_half_width_);
  right_longitudinal_half_width_ = declare_parameter<double>(
    "command_filter.right_longitudinal_half_width", right_longitudinal_half_width_);
  command_filter_forward_distance_ = declare_parameter<double>(
    "command_filter.forward_distance", command_filter_forward_distance_);
  command_filter_lateral_distance_ = declare_parameter<double>(
    "command_filter.lateral_distance", command_filter_lateral_distance_);
  command_filter_publish_rate_hz_ = declare_parameter<double>(
    "command_filter.publish_rate_hz", command_filter_publish_rate_hz_);
  fill_debug_outputs_ = declare_parameter<bool>("algorithm.fill_debug_outputs", fill_debug_outputs_);
  debug_fill_z_ = declare_parameter<double>("algorithm.debug_fill_z", debug_fill_z_);

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
  fps_window_start_ = std::chrono::steady_clock::now();

  if (respect_autonomy_mode_) {
    autonomy_sub_ = create_subscription<autonomy::msg::AutonomyState>(
      autonomy_status_topic_,
      rclcpp::QoS(10),
      [this](autonomy::msg::AutonomyState::SharedPtr msg) {
        onAutonomyState(std::move(msg));
      });
  }

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_,
    rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      onCloud(std::move(msg));
    });

  elevation_image_pub_ = create_publisher<sensor_msgs::msg::Image>(output_image_topic_, 10);
  elevation_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    output_cloud_topic_, rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile());
  masked_height_scan_pub_ = create_publisher<autonomy::msg::MaskedHeightScan>(
    output_masked_height_scan_topic_, rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile());
  heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
    "/autonomy/heartbeat/elevation_mapping_node", 10);
  command_filter_pub_ = create_publisher<core::msg::CommandFilter>(
    command_filter_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
  heartbeat_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    [this]() { publishHeartbeat(); });
  const auto command_filter_period = std::chrono::duration<double>(
    1.0 / std::max(1.0, command_filter_publish_rate_hz_));
  command_filter_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(command_filter_period),
    [this]() { publishCommandFilter(); });
  if (local_terrain_map_enabled_) {
    local_terrain_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      output_local_terrain_image_topic_, 10);
    local_terrain_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_local_terrain_cloud_topic_, rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile());
    local_terrain_scan_pub_ = create_publisher<autonomy::msg::MaskedHeightScan>(
      output_local_terrain_scan_topic_, rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile());
  }
  if (dds_height_map_enabled_) {
    dds_height_map_pub_ = std::make_unique<DdsHeightMapPublisher>(
      static_cast<std::uint32_t>(std::max(0, dds_domain_id_)),
      dds_height_map_topic_,
      dds_height_map_type_);
    if (!dds_height_map_pub_->isReady()) {
      RCLCPP_ERROR(
        get_logger(),
        "DDS height map writer disabled after init failure: %s",
        dds_height_map_pub_->error().c_str());
      dds_height_map_pub_.reset();
    }
  }

  RCLCPP_INFO(get_logger(), "Subscribing merged cloud: %s", input_cloud_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Publishing elevation image: %s", output_image_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Publishing elevation points: %s", output_cloud_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Publishing masked height scan: %s",
    output_masked_height_scan_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Publishing command filter: %s", command_filter_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Operation mode: %s", operation_mode_.c_str());
  if (local_terrain_map_enabled_) {
    RCLCPP_INFO(
      get_logger(),
      "Publishing local terrain map: image=%s points=%s scan=%s grid=%ux%u",
      output_local_terrain_image_topic_.c_str(),
      output_local_terrain_cloud_topic_.c_str(),
      output_local_terrain_scan_topic_.c_str(),
      local_terrain_grid_spec_.width(),
      local_terrain_grid_spec_.height());
  }
  if (dds_height_map_pub_) {
    RCLCPP_INFO(
      get_logger(),
      "Publishing DDS height map: domain_id=%d topic=%s type=%s",
      dds_domain_id_,
      dds_height_map_topic_.c_str(),
      dds_height_map_type_.c_str());
  }
}

void ElevationMappingNode::publishHeartbeat()
{
  std_msgs::msg::String msg;
  if (!processingActive()) {
    msg.data = "standby";
  } else {
    msg.data = has_latest_height_map_ ? "ready" : "waiting_for_cloud";
  }
  heartbeat_pub_->publish(msg);
}

void ElevationMappingNode::onAutonomyState(autonomy::msg::AutonomyState::SharedPtr msg)
{
  autonomy_mode_ = msg->mode;
  has_autonomy_state_ = true;
}

bool ElevationMappingNode::processingActive() const
{
  if (!respect_autonomy_mode_) {
    return true;
  }
  if (!has_autonomy_state_) {
    return false;
  }
  return autonomy_mode_ == autonomy::msg::AutonomyState::DRIVE ||
    autonomy_mode_ == autonomy::msg::AutonomyState::ADAS ||
    autonomy_mode_ == autonomy::msg::AutonomyState::FSD ||
    autonomy_mode_ == autonomy::msg::AutonomyState::MAPPING ||
    autonomy_mode_ == autonomy::msg::AutonomyState::TRACKING;
}

void ElevationMappingNode::onCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!processingActive()) {
    std::lock_guard<std::mutex> lock(latest_height_map_mutex_);
    has_latest_height_map_ = false;
    return;
  }

  auto grid = elevation_backend_->build(*msg, msg->header);
  auto height_map = gridToHeightMapFrame(grid, height_scan_offset_, base_height_);

  {
    std::lock_guard<std::mutex> lock(latest_height_map_mutex_);
    latest_height_map_ = height_map;
    has_latest_height_map_ = true;
  }

  masked_height_scan_pub_->publish(toRosMaskedHeightScan(height_map));
  if (dds_height_map_pub_) {
    dds_height_map_pub_->publish(toDdsHeightMap(height_map));
  }

  if (local_terrain_backend_) {
    auto local_grid = local_terrain_backend_->build(*msg, msg->header);
    local_terrain_scan_pub_->publish(toRosMaskedHeightScan(
      gridToHeightMapFrame(local_grid, height_scan_offset_, base_height_)));
    fillDebugGrid(local_grid);
    local_terrain_image_pub_->publish(local_grid.toImageMsg());
    local_terrain_cloud_pub_->publish(gridToPointCloud(local_grid));
  }

  auto debug_grid = grid;
  fillDebugGrid(debug_grid);
  elevation_image_pub_->publish(debug_grid.toImageMsg());
  elevation_cloud_pub_->publish(gridToPointCloud(debug_grid));

  ++fps_frame_count_;
  const auto now = std::chrono::steady_clock::now();
  const std::chrono::duration<double> elapsed = now - fps_window_start_;
  if (elapsed.count() >= 1.0) {
    const double fps = static_cast<double>(fps_frame_count_) / elapsed.count();
    RCLCPP_INFO(
      get_logger(),
      "Elevation mapping FPS: %.1f input_points=%zu output_cells=%zu",
      fps,
      static_cast<std::size_t>(msg->width) * msg->height,
      static_cast<std::size_t>(grid.spec.width()) * grid.spec.height());
    fps_window_start_ = now;
    fps_frame_count_ = 0;
  }
}

void ElevationMappingNode::publishCommandFilter()
{
  const double forward_distance = command_filter_forward_distance_ > 0.0 ?
    command_filter_forward_distance_ : std::max(0.0, grid_spec_.x_max);
  const double lateral_distance = command_filter_lateral_distance_ > 0.0 ?
    command_filter_lateral_distance_ : std::max(std::abs(grid_spec_.y_min), std::abs(grid_spec_.y_max));
  command_filter_pub_->publish(evaluateCommandFilter(forward_distance, lateral_distance));
}

core::msg::CommandFilter ElevationMappingNode::evaluateCommandFilter(
  const double move_forward,
  const double move_right) const
{
  core::msg::CommandFilter result;
  result.allow_linear_vel_x = true;
  result.allow_linear_vel_y = true;

  HeightMapFrame frame;
  {
    std::lock_guard<std::mutex> lock(latest_height_map_mutex_);
    if (!has_latest_height_map_) {
      return result;
    }
    frame = latest_height_map_;
  }

  const auto forward = move_forward;
  const auto right = move_right;
  result.allow_linear_vel_x = isPathClear(
    frame,
    std::min(0.0, forward),
    std::max(0.0, forward),
    -forward_lateral_half_width_,
    forward_lateral_half_width_);
  result.allow_linear_vel_y = isPathClear(
    frame,
    -right_longitudinal_half_width_,
    right_longitudinal_half_width_,
    std::min(0.0, right),
    std::max(0.0, right));
  return result;
}

void ElevationMappingNode::fillDebugGrid(ElevationGrid & grid) const
{
  if (!fill_debug_outputs_ || !std::isfinite(debug_fill_z_)) {
    return;
  }

  const auto fill_value = static_cast<float>(debug_fill_z_);
  for (auto & height : grid.height) {
    if (!std::isfinite(height)) {
      height = fill_value;
    }
  }
}

bool ElevationMappingNode::isPathClear(
  const HeightMapFrame & frame,
  const double x_min,
  const double x_max,
  const double y_min,
  const double y_max) const
{
  if (x_min == x_max && y_min == y_max) {
    return true;
  }

  const auto width = frame.spec.width();
  const auto height = frame.spec.height();
  const auto obstacle_z = obstacle_floor_z_ + obstacle_height_threshold_;

  for (std::uint32_t row = 0; row < height; ++row) {
    const auto y = frame.spec.y_min + (row + 0.5) * frame.spec.resolution;
    if (y < y_min || y > y_max) {
      continue;
    }

    for (std::uint32_t col = 0; col < width; ++col) {
      const auto x = frame.spec.x_min + (col + 0.5) * frame.spec.resolution;
      if (x < x_min || x > x_max) {
        continue;
      }

      const auto index = static_cast<std::size_t>(row) * width + col;
      if (index >= frame.data.size() || index >= frame.valid_mask.size() || frame.valid_mask[index] == 0) {
        continue;
      }

      const auto z = -static_cast<double>(frame.data[index]) - height_scan_offset_;
      if (std::isfinite(z) && z >= obstacle_z) {
        return false;
      }
    }
  }

  return true;
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

}  // namespace autonomy
