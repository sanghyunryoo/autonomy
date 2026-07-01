#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

namespace autonomy
{
namespace
{

class PointLioMonitorNode final : public rclcpp::Node
{
public:
  PointLioMonitorNode()
  : Node("point_lio_monitor_node")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    timeout_sec_ = std::max(0.1, declare_parameter<double>("timeout_sec", 3.0));

    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/point_lio_monitor_node", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      10,
      [this](nav_msgs::msg::Odometry::SharedPtr) {
        last_odom_time_ = now();
        odom_count_++;
      });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr) {
        last_cloud_time_ = now();
        cloud_count_++;
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() { publishHeartbeat(); });

    RCLCPP_INFO(
      get_logger(),
      "Point-LIO monitor watching odom=%s cloud=%s timeout=%.2fs",
      odom_topic_.c_str(),
      cloud_topic_.c_str(),
      timeout_sec_);
  }

private:
  bool fresh(const rclcpp::Time & stamp) const
  {
    return stamp.nanoseconds() > 0 &&
      (now() - stamp) <= rclcpp::Duration::from_seconds(timeout_sec_);
  }

  void publishHeartbeat()
  {
    std_msgs::msg::String msg;
    const bool odom_fresh = fresh(last_odom_time_);
    const bool cloud_fresh = fresh(last_cloud_time_);
    if (odom_fresh && cloud_fresh) {
      msg.data = "ready:odom=" + std::to_string(odom_count_) +
        ":cloud=" + std::to_string(cloud_count_);
    } else if (odom_count_ == 0 && cloud_count_ == 0) {
      msg.data = "waiting_for_point_lio";
    } else if (!odom_fresh && !cloud_fresh) {
      msg.data = "degraded:stale_odom_and_cloud";
    } else if (!odom_fresh) {
      msg.data = "degraded:stale_odom";
    } else {
      msg.data = "degraded:stale_cloud";
    }
    heartbeat_pub_->publish(msg);
  }

  std::string odom_topic_;
  std::string cloud_topic_;
  double timeout_sec_{3.0};
  std::uint64_t odom_count_{0};
  std::uint64_t cloud_count_{0};
  rclcpp::Time last_odom_time_{0, 0u, RCL_SYSTEM_TIME};
  rclcpp::Time last_cloud_time_{0, 0u, RCL_SYSTEM_TIME};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::PointLioMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
