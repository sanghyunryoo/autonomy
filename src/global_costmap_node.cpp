#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

namespace autonomy
{
namespace
{

class GlobalCostmapNode final : public rclcpp::Node
{
public:
  GlobalCostmapNode()
  : Node("global_costmap_node")
  {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    costmap_topic_ = declare_parameter<std::string>("costmap_topic", "~/global_costmap");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    resolution_ = std::max(0.02, declare_parameter<double>("resolution", 0.10));
    x_min_ = declare_parameter<double>("x_min", -20.0);
    x_max_ = declare_parameter<double>("x_max", 20.0);
    y_min_ = declare_parameter<double>("y_min", -20.0);
    y_max_ = declare_parameter<double>("y_max", 20.0);
    min_obstacle_z_ = declare_parameter<double>("min_obstacle_z", 0.05);
    max_obstacle_z_ = declare_parameter<double>("max_obstacle_z", 1.50);
    hit_increment_ = static_cast<int>(
      std::clamp<long>(declare_parameter<int>("hit_increment", 8), 1L, 100L));
    decay_per_publish_ = static_cast<int>(
      std::clamp<long>(declare_parameter<int>("decay_per_publish", 0), 0L, 100L));
    publish_rate_hz_ = std::max(1.0, declare_parameter<double>("publish_rate_hz", 2.0));

    if (x_max_ <= x_min_ || y_max_ <= y_min_) {
      throw std::invalid_argument("Invalid global costmap bounds");
    }

    width_ = static_cast<std::uint32_t>(std::ceil((x_max_ - x_min_) / resolution_));
    height_ = static_cast<std::uint32_t>(std::ceil((y_max_ - y_min_) / resolution_));
    costs_.assign(static_cast<std::size_t>(width_) * height_, -1);

    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/global_costmap_node", 10);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { onCloud(std::move(msg)); });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publishCostmap(); });

    RCLCPP_INFO(
      get_logger(),
      "Global costmap listening cloud=%s publishing %s frame=%s size=%ux%u res=%.3f",
      cloud_topic_.c_str(),
      costmap_topic_.c_str(),
      frame_id_.c_str(),
      width_,
      height_,
      resolution_);
  }

private:
  void onCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_stamp_ = msg->header.stamp;
    if (!msg->header.frame_id.empty()) {
      frame_id_ = msg->header.frame_id;
    }

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x_it(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_it(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_it(*msg, "z");
      const auto count = static_cast<std::size_t>(msg->width) * msg->height;
      for (std::size_t i = 0; i < count; ++i, ++x_it, ++y_it, ++z_it) {
        addPoint(*x_it, *y_it, *z_it);
      }
      received_clouds_++;
    } catch (const std::runtime_error & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Global costmap input cloud missing xyz fields: %s",
        ex.what());
    }
  }

  void addPoint(const float x, const float y, const float z)
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return;
    }
    if (z < min_obstacle_z_ || z > max_obstacle_z_) {
      return;
    }
    if (x < x_min_ || x >= x_max_ || y < y_min_ || y >= y_max_) {
      return;
    }

    const auto col = static_cast<std::uint32_t>((x - x_min_) / resolution_);
    const auto row = static_cast<std::uint32_t>((y - y_min_) / resolution_);
    const auto index = static_cast<std::size_t>(row) * width_ + col;
    if (index >= costs_.size()) {
      return;
    }
    const int current = costs_[index] < 0 ? 0 : costs_[index];
    costs_[index] = static_cast<std::int8_t>(std::min(100, current + hit_increment_));
  }

  void publishCostmap()
  {
    nav_msgs::msg::OccupancyGrid msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (decay_per_publish_ > 0) {
        for (auto & cost : costs_) {
          if (cost > 0) {
            cost = static_cast<std::int8_t>(std::max(0, static_cast<int>(cost) - decay_per_publish_));
          }
        }
      }
      if (last_stamp_.sec == 0 && last_stamp_.nanosec == 0) {
        msg.header.stamp = get_clock()->now();
      } else {
        msg.header.stamp = last_stamp_;
      }
      msg.header.frame_id = frame_id_;
      msg.info.resolution = static_cast<float>(resolution_);
      msg.info.width = width_;
      msg.info.height = height_;
      msg.info.origin.position.x = x_min_;
      msg.info.origin.position.y = y_min_;
      msg.info.origin.orientation.w = 1.0;
      msg.data = costs_;
    }
    costmap_pub_->publish(msg);

    std_msgs::msg::String heartbeat;
    heartbeat.data = received_clouds_ == 0 ? "waiting_for_cloud" :
      "ready:clouds=" + std::to_string(received_clouds_);
    heartbeat_pub_->publish(heartbeat);
  }

  std::string cloud_topic_;
  std::string costmap_topic_;
  std::string frame_id_;
  double resolution_{0.10};
  double x_min_{-20.0};
  double x_max_{20.0};
  double y_min_{-20.0};
  double y_max_{20.0};
  double min_obstacle_z_{0.05};
  double max_obstacle_z_{1.50};
  int hit_increment_{8};
  int decay_per_publish_{0};
  double publish_rate_hz_{2.0};
  std::uint32_t width_{0};
  std::uint32_t height_{0};
  std::vector<std::int8_t> costs_;
  builtin_interfaces::msg::Time last_stamp_;
  std::uint64_t received_clouds_{0};
  std::mutex mutex_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::GlobalCostmapNode>());
  rclcpp::shutdown();
  return 0;
}
