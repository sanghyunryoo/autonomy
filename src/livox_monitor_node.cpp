#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

namespace autonomy
{

class LivoxMonitorNode final : public rclcpp::Node
{
public:
  LivoxMonitorNode()
  : Node("livox_monitor_node")
  {
    declare_parameter<bool>("enabled", true);
    declare_parameter<bool>("simulation", false);
    declare_parameter<std::string>("pointcloud_topic", "/livox/lidar");
    declare_parameter<std::string>("required_frame_contains", "");
    declare_parameter<double>("detection_timeout_sec", 1.0);
    declare_parameter<double>("startup_grace_sec", 12.0);
    declare_parameter<bool>("fatal_on_timeout", true);
    declare_parameter<int>("min_points", 1);
    declare_parameter<double>("publish_rate_hz", 5.0);

    enabled_ = get_parameter("enabled").as_bool();
    simulation_ = get_parameter("simulation").as_bool();
    required_frame_contains_ = get_parameter("required_frame_contains").as_string();
    min_points_ = std::max(1, static_cast<int>(get_parameter("min_points").as_int()));
    timeout_sec_ = std::max(0.1, get_parameter("detection_timeout_sec").as_double());
    startup_grace_sec_ = std::max(0.0, get_parameter("startup_grace_sec").as_double());
    fatal_on_timeout_ = get_parameter("fatal_on_timeout").as_bool();
    start_time_ = now();

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("pointcloud_topic").as_string(),
      10,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        const auto points = static_cast<std::size_t>(msg->width) * msg->height;
        const bool enough_points = points >= static_cast<std::size_t>(min_points_);
        const bool frame_ok = required_frame_contains_.empty() ||
          msg->header.frame_id.find(required_frame_contains_) != std::string::npos;
        if (enough_points && frame_ok) {
          last_seen_ = now();
          seen_ = true;
          last_frame_id_ = msg->header.frame_id;
          last_points_ = points;
        }
      });
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/livox_monitor_node",
      10);

    const auto period = std::chrono::duration<double>(
      1.0 / std::max(1.0, get_parameter("publish_rate_hz").as_double()));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publishHeartbeat(); });
  }

private:
  void publishHeartbeat()
  {
    std_msgs::msg::String heartbeat;
    if (!enabled_) {
      heartbeat.data = "disabled";
    } else if (simulation_) {
      heartbeat.data = "simulated";
    } else if (detected()) {
      heartbeat.data = "detected:frame=" + last_frame_id_ + ":points=" + std::to_string(last_points_);
    } else {
      heartbeat.data = "error:livox_mid360_not_detected";
    }
    heartbeat_pub_->publish(heartbeat);
    maybeFailFast(heartbeat.data);
  }

  void maybeFailFast(const std::string & heartbeat)
  {
    if (!enabled_ || simulation_ || !fatal_on_timeout_ || detected()) {
      return;
    }
    const auto elapsed = (now() - start_time_).seconds();
    if (elapsed < startup_grace_sec_) {
      return;
    }
    RCLCPP_FATAL(
      get_logger(),
      "LiDAR point cloud is not available after %.1f sec: topic timeout on /livox/lidar "
      "(last_status=%s). Shutting down autonomy launch.",
      elapsed,
      heartbeat.c_str());
    rclcpp::shutdown();
  }

  [[nodiscard]] bool detected() const
  {
    if (!seen_) {
      return false;
    }
    return (now() - last_seen_) <= rclcpp::Duration::from_seconds(timeout_sec_);
  }

  bool enabled_{true};
  bool simulation_{false};
  bool seen_{false};
  int min_points_{1};
  double timeout_sec_{1.0};
  double startup_grace_sec_{12.0};
  bool fatal_on_timeout_{true};
  std::size_t last_points_{0};
  std::string required_frame_contains_;
  std::string last_frame_id_;
  rclcpp::Time start_time_{0, 0u, RCL_SYSTEM_TIME};
  rclcpp::Time last_seen_{0, 0u, RCL_SYSTEM_TIME};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::LivoxMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
