#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "autonomy/msg/autonomy_state.hpp"
#include "autonomy/msg/masked_height_scan.hpp"
#include "core/msg/command_user.hpp"

namespace autonomy
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
    declare_parameter<std::string>("command_user_topic", "/command_user");
    declare_parameter<std::string>("autonomy_status_topic", "/autonomy_manager/status");
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
    height_scan_sub_ = create_subscription<autonomy::msg::MaskedHeightScan>(
      get_parameter("height_scan_topic").as_string(),
      10,
      [this](autonomy::msg::MaskedHeightScan::SharedPtr msg) {
        latest_scan_ = std::move(msg);
      });
    autonomy_sub_ = create_subscription<autonomy::msg::AutonomyState>(
      get_parameter("autonomy_status_topic").as_string(),
      10,
      [this](autonomy::msg::AutonomyState::SharedPtr msg) {
        autonomy_allows_command_ =
          !msg->estop_active &&
          !msg->error_active &&
          (msg->mode == autonomy::msg::AutonomyState::ADAS ||
           msg->mode == autonomy::msg::AutonomyState::FSD);
        has_autonomy_state_ = true;
      });
    command_user_pub_ = create_publisher<core::msg::CommandUser>(
      get_parameter("command_user_topic").as_string(),
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
    } else if (!has_autonomy_state_ || !autonomy_allows_command_) {
      heartbeat.data = "standby:autonomy_blocked";
    } else if (has_current_pose_ && has_target_pose_ && latest_scan_) {
      heartbeat.data = model_path_.empty() ? "ready:no_model" : "ready";
      // TODO: Load ONNX Runtime session and publish action from current pose,
      // target pose, and local height scan.
      command_user_pub_->publish(makeStopCommand());
    } else {
      heartbeat.data = "waiting_for_inputs";
    }
    heartbeat_pub_->publish(heartbeat);
  }

  [[nodiscard]] core::msg::CommandUser makeStopCommand() const
  {
    core::msg::CommandUser command;
    command.event.estop = false;
    command.event.wake = false;
    command.event.sleep = false;
    command.event.rough_drive_toggle = false;
    return command;
  }

  bool enabled_{true};
  bool has_autonomy_state_{false};
  bool autonomy_allows_command_{false};
  bool has_current_pose_{false};
  bool has_target_pose_{false};
  std::string model_path_;
  geometry_msgs::msg::Pose2D current_pose_;
  geometry_msgs::msg::Pose2D target_pose_;
  autonomy::msg::MaskedHeightScan::SharedPtr latest_scan_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<autonomy::msg::MaskedHeightScan>::SharedPtr height_scan_sub_;
  rclcpp::Subscription<autonomy::msg::AutonomyState>::SharedPtr autonomy_sub_;
  rclcpp::Publisher<core::msg::CommandUser>::SharedPtr command_user_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::RlLocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
