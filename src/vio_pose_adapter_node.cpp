#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace autonomy
{

class VioPoseAdapterNode final : public rclcpp::Node
{
public:
  VioPoseAdapterNode()
  : Node("vio_pose_adapter_node")
  {
    declare_parameter<std::string>("odom_topic", "/odomimu");
    declare_parameter<std::string>("pose2d_topic", "/localization/current_pose");
    declare_parameter<std::string>("heartbeat_name", "vio_pose_adapter_node");
    declare_parameter<double>("publish_rate_hz", 10.0);

    heartbeat_name_ = get_parameter("heartbeat_name").as_string();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(),
      20,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = std::move(msg);
        publishPose2d(*latest_odom_);
      });

    pose2d_pub_ = create_publisher<geometry_msgs::msg::Pose2D>(
      get_parameter("pose2d_topic").as_string(),
      10);
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/" + heartbeat_name_,
      10);

    const auto period = std::chrono::duration<double>(
      1.0 / std::max(1.0, get_parameter("publish_rate_hz").as_double()));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publishHeartbeat(); });
  }

private:
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  void publishPose2d(const nav_msgs::msg::Odometry & odom)
  {
    geometry_msgs::msg::Pose2D pose;
    pose.x = odom.pose.pose.position.x;
    pose.y = odom.pose.pose.position.y;
    pose.theta = yawFromQuaternion(odom.pose.pose.orientation);
    pose2d_pub_->publish(pose);
  }

  void publishHeartbeat()
  {
    std_msgs::msg::String heartbeat;
    heartbeat.data = latest_odom_ ? "tracking" : "waiting_for_vio";
    heartbeat_pub_->publish(heartbeat);
  }

  std::string heartbeat_name_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose2d_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::VioPoseAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
