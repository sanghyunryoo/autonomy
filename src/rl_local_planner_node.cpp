#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "height_map_ros2/msg/masked_height_scan.hpp"

namespace height_map_ros2
{

class RlLocalPlannerNode final : public rclcpp::Node
{
public:
  RlLocalPlannerNode()
  : Node("rl_local_planner_node")
  {
    declare_parameter<bool>("enabled", true);
    declare_parameter<std::string>("model_path", "");
    declare_parameter<std::string>("current_pose_topic", "/localization/current_pose");
    declare_parameter<std::string>("target_pose_topic", "/planning/target_pose");
    declare_parameter<std::string>("height_scan_topic", "/elevation_mapping_node/local_terrain_map");
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    declare_parameter<double>("publish_rate_hz", 20.0);

    enabled_ = get_parameter("enabled").as_bool();
    model_path_ = get_parameter("model_path").as_string();

    current_pose_sub_ = create_subscription<geometry_msgs::msg::Pose2D>(
      get_parameter("current_pose_topic").as_string(),
      10,
      [this](geometry_msgs::msg::Pose2D::SharedPtr msg) {
        current_pose_ = *msg;
        has_current_pose_ = true;
      });
    target_pose_sub_ = create_subscription<geometry_msgs::msg::Pose2D>(
      get_parameter("target_pose_topic").as_string(),
      10,
      [this](geometry_msgs::msg::Pose2D::SharedPtr msg) {
        target_pose_ = *msg;
        has_target_pose_ = true;
      });
    height_scan_sub_ = create_subscription<height_map_ros2::msg::MaskedHeightScan>(
      get_parameter("height_scan_topic").as_string(),
      10,
      [this](height_map_ros2::msg::MaskedHeightScan::SharedPtr msg) {
        latest_scan_ = std::move(msg);
      });
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_topic").as_string(),
      10);
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/rl_local_planner_node",
      10);

    const auto period = std::chrono::duration<double>(
      1.0 / std::max(1.0, get_parameter("publish_rate_hz").as_double()));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "C++ RL local planner skeleton started");
  }

private:
  void tick()
  {
    std_msgs::msg::String heartbeat;
    if (!enabled_) {
      heartbeat.data = "disabled";
    } else if (has_current_pose_ && has_target_pose_ && latest_scan_) {
      heartbeat.data = model_path_.empty() ? "ready:no_model" : "ready";
      // TODO: Load ONNX Runtime session and publish action from current pose,
      // target pose, and local height scan.
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
    } else {
      heartbeat.data = "waiting_for_inputs";
    }
    heartbeat_pub_->publish(heartbeat);
  }

  bool enabled_{true};
  bool has_current_pose_{false};
  bool has_target_pose_{false};
  std::string model_path_;
  geometry_msgs::msg::Pose2D current_pose_;
  geometry_msgs::msg::Pose2D target_pose_;
  height_map_ros2::msg::MaskedHeightScan::SharedPtr latest_scan_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<height_map_ros2::msg::MaskedHeightScan>::SharedPtr height_scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace height_map_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<height_map_ros2::RlLocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
