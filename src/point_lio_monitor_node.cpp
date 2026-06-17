#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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
    declare_parameter<std::string>("pointcloud_topic", "/livox/lidar");
    declare_parameter<std::string>("imu_topic", "/livox/imu");
    declare_parameter<double>("detection_timeout_sec", 1.0);
    declare_parameter<double>("publish_rate_hz", 5.0);

    enabled_ = get_parameter("enabled").as_bool();
    simulation_ = get_parameter("simulation").as_bool();
    timeout_sec_ = std::max(0.1, get_parameter("detection_timeout_sec").as_double());

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odometry_topic").as_string(),
      10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_seen_ = now();
        seen_odom_ = true;
        last_odom_frame_id_ = msg->header.frame_id;
      });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("pointcloud_topic").as_string(),
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        last_cloud_seen_ = now();
        seen_cloud_ = true;
        last_cloud_frame_id_ = msg->header.frame_id;
        last_cloud_points_ = static_cast<std::size_t>(msg->width) * msg->height;
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      get_parameter("imu_topic").as_string(),
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        last_imu_seen_ = now();
        seen_imu_ = true;
        last_imu_frame_id_ = msg->header.frame_id;
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
    } else if (odomDetected()) {
      heartbeat.data = "tracking:odom_frame=" + last_odom_frame_id_;
    } else if (!cloudDetected() && !imuDetected()) {
      heartbeat.data = "error:point_lio_waiting_for_lidar_and_imu";
    } else if (!cloudDetected()) {
      heartbeat.data = "error:point_lio_waiting_for_lidar";
    } else if (!imuDetected()) {
      heartbeat.data = "error:point_lio_waiting_for_imu";
    } else {
      heartbeat.data = "error:point_lio_odometry_not_detected:lidar_frame=" +
        last_cloud_frame_id_ + ":points=" + std::to_string(last_cloud_points_) +
        ":imu_frame=" + last_imu_frame_id_;
    }
    heartbeat_pub_->publish(heartbeat);
  }

  [[nodiscard]] bool fresh(const bool seen, const rclcpp::Time & stamp) const
  {
    if (!seen) {
      return false;
    }
    return (now() - stamp) <= rclcpp::Duration::from_seconds(timeout_sec_);
  }

  [[nodiscard]] bool odomDetected() const { return fresh(seen_odom_, last_odom_seen_); }
  [[nodiscard]] bool cloudDetected() const { return fresh(seen_cloud_, last_cloud_seen_); }
  [[nodiscard]] bool imuDetected() const { return fresh(seen_imu_, last_imu_seen_); }

  bool enabled_{true};
  bool simulation_{false};
  bool seen_odom_{false};
  bool seen_cloud_{false};
  bool seen_imu_{false};
  double timeout_sec_{1.0};
  std::size_t last_cloud_points_{0};
  std::string last_odom_frame_id_;
  std::string last_cloud_frame_id_;
  std::string last_imu_frame_id_;
  rclcpp::Time last_odom_seen_{0, 0u, RCL_SYSTEM_TIME};
  rclcpp::Time last_cloud_seen_{0, 0u, RCL_SYSTEM_TIME};
  rclcpp::Time last_imu_seen_{0, 0u, RCL_SYSTEM_TIME};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
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
