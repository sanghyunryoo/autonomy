#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace autonomy
{

class PointLioMonitorNode final : public rclcpp::Node
{
public:
  PointLioMonitorNode()
  : Node("point_lio_monitor_node")
  {
    declare_parameter<bool>("enabled", true);
    declare_parameter<bool>("simulation", false);
    declare_parameter<std::string>("odometry_topic", "/aft_mapped_to_init");
    declare_parameter<double>("detection_timeout_sec", 1.0);
    declare_parameter<double>("publish_rate_hz", 5.0);

    enabled_ = get_parameter("enabled").as_bool();
    simulation_ = get_parameter("simulation").as_bool();
    timeout_sec_ = std::max(0.1, get_parameter("detection_timeout_sec").as_double());

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odometry_topic").as_string(),
      10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        last_seen_ = now();
        seen_ = true;
        last_frame_id_ = msg->header.frame_id;
      });
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/point_lio_monitor_node",
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
      heartbeat.data = "tracking:frame=" + last_frame_id_;
    } else {
      heartbeat.data = "error:point_lio_odometry_not_detected";
    }
    heartbeat_pub_->publish(heartbeat);
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
  double timeout_sec_{1.0};
  std::string last_frame_id_;
  rclcpp::Time last_seen_{0, 0u, RCL_SYSTEM_TIME};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::PointLioMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
