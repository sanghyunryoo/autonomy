#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "autonomy/msg/autonomy_state.hpp"
#include "autonomy/msg/detected_object.hpp"
#include "autonomy/msg/detected_object_array.hpp"

namespace autonomy
{

class TrackingFollowerNode final : public rclcpp::Node
{
public:
  TrackingFollowerNode()
  : Node("tracking_follower_node")
  {
    declare_parameter<bool>("enabled", true);
    declare_parameter<std::string>("detections_topic", "/ai_detection_node/detections");
    declare_parameter<std::string>("autonomy_status_topic", "/autonomy_manager/status");
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    declare_parameter<std::string>("target_label", "");
    declare_parameter<double>("target_distance_m", 1.0);
    declare_parameter<double>("linear_gain", 0.6);
    declare_parameter<double>("angular_gain", 1.2);
    declare_parameter<double>("max_linear_speed", 0.4);
    declare_parameter<double>("max_angular_speed", 0.8);
    declare_parameter<double>("target_timeout_sec", 0.5);
    declare_parameter<double>("publish_rate_hz", 20.0);

    enabled_ = get_parameter("enabled").as_bool();
    target_label_ = get_parameter("target_label").as_string();
    target_distance_m_ = get_parameter("target_distance_m").as_double();
    linear_gain_ = get_parameter("linear_gain").as_double();
    angular_gain_ = get_parameter("angular_gain").as_double();
    max_linear_speed_ = get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = get_parameter("max_angular_speed").as_double();
    target_timeout_sec_ = get_parameter("target_timeout_sec").as_double();

    detections_sub_ = create_subscription<autonomy::msg::DetectedObjectArray>(
      get_parameter("detections_topic").as_string(),
      10,
      [this](autonomy::msg::DetectedObjectArray::SharedPtr msg) {
        onDetections(std::move(msg));
      });
    autonomy_sub_ = create_subscription<autonomy::msg::AutonomyState>(
      get_parameter("autonomy_status_topic").as_string(),
      10,
      [this](autonomy::msg::AutonomyState::SharedPtr msg) {
        autonomy_allows_command_ =
          msg->mode == autonomy::msg::AutonomyState::TRACKING &&
          msg->ai_enabled &&
          !msg->estop_active &&
          !msg->error_active;
        has_autonomy_state_ = true;
      });
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_topic").as_string(),
      10);
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/tracking_follower_node",
      10);

    const auto period = std::chrono::duration<double>(
      1.0 / std::max(1.0, get_parameter("publish_rate_hz").as_double()));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { tick(); });

    RCLCPP_INFO(
      get_logger(),
      "Tracking follower started: detections=%s cmd_vel=%s target_label='%s'",
      get_parameter("detections_topic").as_string().c_str(),
      get_parameter("cmd_vel_topic").as_string().c_str(),
      target_label_.c_str());
  }

private:
  void onDetections(autonomy::msg::DetectedObjectArray::SharedPtr msg)
  {
    const autonomy::msg::DetectedObject * best = nullptr;
    float best_confidence = -std::numeric_limits<float>::infinity();
    for (const auto & object : msg->objects) {
      if (!object.has_3d_position) {
        continue;
      }
      if (!target_label_.empty() && object.label != target_label_) {
        continue;
      }
      if (object.confidence > best_confidence) {
        best = &object;
        best_confidence = object.confidence;
      }
    }

    if (!best) {
      has_target_ = false;
      return;
    }

    target_ = *best;
    target_stamp_ = now();
    has_target_ = true;
  }

  void tick()
  {
    std_msgs::msg::String heartbeat;
    if (!enabled_) {
      heartbeat.data = "disabled";
      publishStop();
    } else if (!has_autonomy_state_ || !autonomy_allows_command_) {
      heartbeat.data = "standby:autonomy_blocked";
      publishStop();
    } else if (!freshTarget()) {
      heartbeat.data = "waiting_for_target";
      publishStop();
    } else {
      heartbeat.data = "tracking:" + target_.label;
      publishTrackingCommand();
    }
    heartbeat_pub_->publish(heartbeat);
  }

  bool freshTarget() const
  {
    if (!has_target_) {
      return false;
    }
    const auto timeout = rclcpp::Duration::from_seconds(target_timeout_sec_);
    return (now() - target_stamp_) <= timeout;
  }

  void publishTrackingCommand()
  {
    geometry_msgs::msg::Twist cmd;
    const double lateral_error = target_.position.x;
    const double forward_error = target_.position.z - target_distance_m_;
    cmd.linear.x = std::clamp(linear_gain_ * forward_error, -max_linear_speed_, max_linear_speed_);
    cmd.angular.z = std::clamp(-angular_gain_ * lateral_error, -max_angular_speed_, max_angular_speed_);
    cmd_pub_->publish(cmd);
  }

  void publishStop()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  bool enabled_{true};
  bool has_autonomy_state_{false};
  bool autonomy_allows_command_{false};
  bool has_target_{false};
  std::string target_label_;
  double target_distance_m_{1.0};
  double linear_gain_{0.6};
  double angular_gain_{1.2};
  double max_linear_speed_{0.4};
  double max_angular_speed_{0.8};
  double target_timeout_sec_{0.5};
  rclcpp::Time target_stamp_{0, 0u, RCL_SYSTEM_TIME};
  autonomy::msg::DetectedObject target_;
  rclcpp::Subscription<autonomy::msg::DetectedObjectArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<autonomy::msg::AutonomyState>::SharedPtr autonomy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::TrackingFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
