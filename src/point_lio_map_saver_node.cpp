#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

#include "autonomy/msg/autonomy_state.hpp"

namespace autonomy
{
namespace
{

int clampInt(const int value, const int min_value, const int max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

}  // namespace

class PointLioMapSaverNode final : public rclcpp::Node
{
public:
  PointLioMapSaverNode()
  : Node("point_lio_map_saver_node")
  {
    declare_parameter<bool>("enabled", true);
    declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    declare_parameter<std::string>("map_topic", "/planning/global_costmap");
    declare_parameter<std::string>("map_dir", "src/autonomy/resources/map");
    declare_parameter<std::string>("map_basename", "global_costmap");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<bool>("respect_autonomy_mode", true);
    declare_parameter<std::string>("autonomy_status_topic", "/autonomy_manager/status");
    declare_parameter<std::vector<std::string>>("active_modes", {"MAPPING"});
    declare_parameter<double>("resolution", 0.1);
    declare_parameter<int>("width", 400);
    declare_parameter<int>("height", 400);
    declare_parameter<double>("origin_x", -20.0);
    declare_parameter<double>("origin_y", -20.0);
    declare_parameter<double>("z_min", -0.4);
    declare_parameter<double>("z_max", 2.0);
    declare_parameter<int>("occupied_count_threshold", 2);
    declare_parameter<double>("publish_rate_hz", 1.0);
    declare_parameter<double>("save_period_sec", 10.0);

    enabled_ = get_parameter("enabled").as_bool();
    map_dir_ = get_parameter("map_dir").as_string();
    map_basename_ = get_parameter("map_basename").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    respect_autonomy_mode_ = get_parameter("respect_autonomy_mode").as_bool();
    active_modes_ = get_parameter("active_modes").as_string_array();
    resolution_ = std::max(0.01, get_parameter("resolution").as_double());
    width_ = std::max(1, static_cast<int>(get_parameter("width").as_int()));
    height_ = std::max(1, static_cast<int>(get_parameter("height").as_int()));
    origin_x_ = get_parameter("origin_x").as_double();
    origin_y_ = get_parameter("origin_y").as_double();
    z_min_ = get_parameter("z_min").as_double();
    z_max_ = get_parameter("z_max").as_double();
    occupied_count_threshold_ = std::max(1, static_cast<int>(get_parameter("occupied_count_threshold").as_int()));
    save_period_sec_ = std::max(1.0, get_parameter("save_period_sec").as_double());
    hit_counts_.assign(static_cast<std::size_t>(width_) * height_, 0);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("cloud_topic").as_string(),
      10,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        onCloud(*msg);
      });
    autonomy_sub_ = create_subscription<autonomy::msg::AutonomyState>(
      get_parameter("autonomy_status_topic").as_string(),
      10,
      [this](autonomy::msg::AutonomyState::SharedPtr msg) {
        autonomy_mode_name_ = msg->mode_name;
        has_autonomy_state_ = true;
      });
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      get_parameter("map_topic").as_string(),
      rclcpp::QoS(1).transient_local());
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/point_lio_map_saver_node",
      10);

    const auto period = std::chrono::duration<double>(
      1.0 / std::max(0.1, get_parameter("publish_rate_hz").as_double()));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { tick(); });
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    if (!enabled_ || !modeAllowsMapping()) {
      return;
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const double x = *iter_x;
      const double y = *iter_y;
      const double z = *iter_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (z < z_min_ || z > z_max_) {
        continue;
      }
      const int col = static_cast<int>((x - origin_x_) / resolution_);
      const int row = static_cast<int>((y - origin_y_) / resolution_);
      if (col < 0 || row < 0 || col >= width_ || row >= height_) {
        continue;
      }
      auto & count = hit_counts_[static_cast<std::size_t>(row) * width_ + static_cast<std::size_t>(col)];
      count = static_cast<std::uint16_t>(std::min<int>(count + 1, 65535));
    }
    received_clouds_ = true;
    dirty_ = true;
  }

  void tick()
  {
    std_msgs::msg::String heartbeat;
    if (!enabled_) {
      heartbeat.data = "disabled";
      heartbeat_pub_->publish(heartbeat);
      return;
    }
    if (!modeAllowsMapping()) {
      heartbeat.data = "standby:mode=" + autonomy_mode_name_;
      heartbeat_pub_->publish(heartbeat);
      return;
    }
    if (!received_clouds_) {
      heartbeat.data = "waiting_for_point_lio_map_cloud";
      heartbeat_pub_->publish(heartbeat);
      return;
    }

    const auto grid = buildGrid();
    map_pub_->publish(grid);
    maybeSave(grid);
    heartbeat.data = "mapping:saved_period_sec=" + std::to_string(static_cast<int>(save_period_sec_));
    heartbeat_pub_->publish(heartbeat);
  }

  [[nodiscard]] bool modeAllowsMapping() const
  {
    if (!respect_autonomy_mode_) {
      return true;
    }
    if (!has_autonomy_state_) {
      return false;
    }
    return std::find(active_modes_.begin(), active_modes_.end(), autonomy_mode_name_) != active_modes_.end();
  }

  [[nodiscard]] nav_msgs::msg::OccupancyGrid buildGrid() const
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = frame_id_;
    grid.info.resolution = static_cast<float>(resolution_);
    grid.info.width = static_cast<std::uint32_t>(width_);
    grid.info.height = static_cast<std::uint32_t>(height_);
    grid.info.origin.position.x = origin_x_;
    grid.info.origin.position.y = origin_y_;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(hit_counts_.size(), 0);
    for (std::size_t i = 0; i < hit_counts_.size(); ++i) {
      if (hit_counts_[i] >= occupied_count_threshold_) {
        grid.data[i] = 100;
      }
    }
    return grid;
  }

  void maybeSave(const nav_msgs::msg::OccupancyGrid & grid)
  {
    if (!dirty_) {
      return;
    }
    const auto time_now = now();
    if (last_save_.nanoseconds() != 0 &&
      (time_now - last_save_) < rclcpp::Duration::from_seconds(save_period_sec_))
    {
      return;
    }
    saveMap(grid);
    last_save_ = time_now;
    dirty_ = false;
  }

  void saveMap(const nav_msgs::msg::OccupancyGrid & grid) const
  {
    const std::filesystem::path dir(map_dir_);
    std::filesystem::create_directories(dir);
    const auto pgm_path = dir / (map_basename_ + ".pgm");
    const auto yaml_path = dir / (map_basename_ + ".yaml");
    const auto tmp_pgm_path = dir / (map_basename_ + ".pgm.tmp");
    const auto tmp_yaml_path = dir / (map_basename_ + ".yaml.tmp");

    {
      std::ofstream pgm(tmp_pgm_path, std::ios::binary);
      pgm << "P5\n" << grid.info.width << " " << grid.info.height << "\n255\n";
      for (int row = static_cast<int>(grid.info.height) - 1; row >= 0; --row) {
        for (int col = 0; col < static_cast<int>(grid.info.width); ++col) {
          const auto index = static_cast<std::size_t>(row) * grid.info.width + static_cast<std::size_t>(col);
          const int value = grid.data[index];
          const auto pixel = static_cast<unsigned char>(value >= 100 ? 0 : 254);
          pgm.write(reinterpret_cast<const char *>(&pixel), 1);
        }
      }
    }

    {
      std::ofstream yaml(tmp_yaml_path);
      yaml << "image: " << pgm_path.filename().string() << "\n";
      yaml << "mode: trinary\n";
      yaml << "resolution: " << grid.info.resolution << "\n";
      yaml << "origin: [" << grid.info.origin.position.x << ", "
           << grid.info.origin.position.y << ", 0.0]\n";
      yaml << "negate: 0\n";
      yaml << "occupied_thresh: 0.65\n";
      yaml << "free_thresh: 0.25\n";
    }

    std::filesystem::rename(tmp_pgm_path, pgm_path);
    std::filesystem::rename(tmp_yaml_path, yaml_path);
  }

  bool enabled_{true};
  bool respect_autonomy_mode_{true};
  bool has_autonomy_state_{false};
  bool received_clouds_{false};
  bool dirty_{false};
  int width_{400};
  int height_{400};
  int occupied_count_threshold_{2};
  double resolution_{0.1};
  double origin_x_{-20.0};
  double origin_y_{-20.0};
  double z_min_{-0.4};
  double z_max_{2.0};
  double save_period_sec_{10.0};
  std::string map_dir_;
  std::string map_basename_;
  std::string frame_id_;
  std::string autonomy_mode_name_{"unknown"};
  std::vector<std::string> active_modes_;
  std::vector<std::uint16_t> hit_counts_;
  rclcpp::Time last_save_{0, 0u, RCL_SYSTEM_TIME};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<autonomy::msg::AutonomyState>::SharedPtr autonomy_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::PointLioMapSaverNode>());
  rclcpp::shutdown();
  return 0;
}
