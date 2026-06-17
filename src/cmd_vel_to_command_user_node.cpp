#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "core/msg/command_user.hpp"

namespace autonomy
{

class CmdVelToCommandUserNode final : public rclcpp::Node
{
public:
  CmdVelToCommandUserNode()
  : Node("cmd_vel_to_command_user_node")
  {
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_nav2");
    declare_parameter<std::string>("command_user_topic", "/command_user");
    declare_parameter<std::string>("odom_frame_id", "odom");
    declare_parameter<std::string>("base_frame_id", "4w4l/base_footprint");
    declare_parameter<double>("publish_rate_hz", 50.0);
    declare_parameter<double>("cmd_timeout_sec", 0.25);
    declare_parameter<bool>("integrate_commanded_pose", true);

    odom_frame_id_ = get_parameter("odom_frame_id").as_string();
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    cmd_timeout_sec_ = std::max(0.0, get_parameter("cmd_timeout_sec").as_double());
    integrate_pose_ = get_parameter("integrate_commanded_pose").as_bool();

    command_pub_ = create_publisher<core::msg::CommandUser>(
      get_parameter("command_user_topic").as_string(),
      10);
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/cmd_vel_to_command_user_node",
      10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_topic").as_string(),
      10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        latest_cmd_ = *msg;
        last_cmd_time_ = now();
        has_cmd_ = true;
      });

    last_tick_time_ = now();
    const auto period = std::chrono::duration<double>(
      1.0 / std::max(1.0, get_parameter("publish_rate_hz").as_double()));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { tick(); });
  }

private:
  void tick()
  {
    const auto time_now = now();
    const double dt = std::max(0.0, (time_now - last_tick_time_).seconds());
    last_tick_time_ = time_now;

    auto cmd = latest_cmd_;
    if (!has_cmd_ || (cmd_timeout_sec_ > 0.0 && (time_now - last_cmd_time_).seconds() > cmd_timeout_sec_)) {
      cmd = geometry_msgs::msg::Twist{};
    }

    if (integrate_pose_) {
      integrate(cmd, dt);
    }

    core::msg::CommandUser command;
    command.odom.header.stamp = time_now;
    command.odom.header.frame_id = odom_frame_id_;
    command.odom.child_frame_id = base_frame_id_;
    command.odom.pose.pose.position.x = x_;
    command.odom.pose.pose.position.y = y_;
    command.odom.pose.pose.position.z = 0.0;
    command.odom.pose.pose.orientation = yawToQuaternion(yaw_);
    command.odom.twist.twist = cmd;
    command_pub_->publish(command);

    std_msgs::msg::String heartbeat;
    heartbeat.data = isTimedOut(time_now) ? "holding_zero_cmd" : "tracking";
    heartbeat_pub_->publish(heartbeat);
  }

  void integrate(const geometry_msgs::msg::Twist & cmd, const double dt)
  {
    if (dt <= 0.0) {
      return;
    }
    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);
    x_ += (cmd.linear.x * cos_yaw - cmd.linear.y * sin_yaw) * dt;
    y_ += (cmd.linear.x * sin_yaw + cmd.linear.y * cos_yaw) * dt;
    yaw_ = normalizeAngle(yaw_ + cmd.angular.z * dt);
  }

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw)
  {
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(quat);
  }

  bool isTimedOut(const rclcpp::Time & time_now) const
  {
    return !has_cmd_ || (cmd_timeout_sec_ > 0.0 && (time_now - last_cmd_time_).seconds() > cmd_timeout_sec_);
  }

  std::string odom_frame_id_;
  std::string base_frame_id_;
  double cmd_timeout_sec_{0.25};
  bool integrate_pose_{true};
  bool has_cmd_{false};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Twist latest_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<core::msg::CommandUser>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::CmdVelToCommandUserNode>());
  rclcpp::shutdown();
  return 0;
}
