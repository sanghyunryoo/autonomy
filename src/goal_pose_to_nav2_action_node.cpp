#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include "autonomy/msg/autonomy_state.hpp"

namespace autonomy
{

class GoalPoseToNav2ActionNode final : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  GoalPoseToNav2ActionNode()
  : Node("goal_pose_to_nav2_action_node")
  {
    declare_parameter<bool>("enabled", true);
    declare_parameter<std::string>("goal_pose_topic", "/goal_pose");
    declare_parameter<std::string>("navigate_to_pose_action", "navigate_to_pose");
    declare_parameter<std::string>("behavior_tree", "");
    declare_parameter<double>("server_wait_timeout_sec", 0.5);
    declare_parameter<double>("publish_rate_hz", 2.0);
    declare_parameter<bool>("respect_autonomy_mode", true);
    declare_parameter<std::string>("autonomy_status_topic", "/autonomy_manager/status");
    declare_parameter<std::vector<std::string>>("active_modes", {"ADAS", "FSD"});

    enabled_ = get_parameter("enabled").as_bool();
    behavior_tree_ = get_parameter("behavior_tree").as_string();
    server_wait_timeout_sec_ = std::max(0.0, get_parameter("server_wait_timeout_sec").as_double());
    respect_autonomy_mode_ = get_parameter("respect_autonomy_mode").as_bool();
    active_modes_ = get_parameter("active_modes").as_string_array();

    action_client_ = rclcpp_action::create_client<NavigateToPose>(
      this,
      get_parameter("navigate_to_pose_action").as_string());
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("goal_pose_topic").as_string(),
      10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        sendGoal(*msg);
      });
    autonomy_sub_ = create_subscription<autonomy::msg::AutonomyState>(
      get_parameter("autonomy_status_topic").as_string(),
      10,
      [this](autonomy::msg::AutonomyState::SharedPtr msg) {
        autonomy_allows_goal_ =
          !msg->estop_active &&
          !msg->error_active &&
          std::find(active_modes_.begin(), active_modes_.end(), msg->mode_name) != active_modes_.end();
        has_autonomy_state_ = true;
      });
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/goal_pose_to_nav2_action_node",
      10);

    const auto period = std::chrono::duration<double>(
      1.0 / std::max(0.2, get_parameter("publish_rate_hz").as_double()));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publishHeartbeat(); });
  }

private:
  void sendGoal(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!enabled_) {
      last_status_ = "disabled";
      return;
    }
    if (!modeAllowsGoal()) {
      last_status_ = "standby:autonomy_blocked";
      return;
    }
    if (!action_client_->wait_for_action_server(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(server_wait_timeout_sec_))))
    {
      last_status_ = "waiting_for_nav2_action_server";
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Nav2 NavigateToPose action server is not ready");
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose = pose;
    if (goal.pose.header.frame_id.empty()) {
      goal.pose.header.frame_id = "map";
    }
    if (goal.pose.header.stamp.sec == 0 && goal.pose.header.stamp.nanosec == 0) {
      goal.pose.header.stamp = now();
    }
    goal.behavior_tree = behavior_tree_;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const GoalHandle::SharedPtr & handle) {
        last_status_ = handle ? "goal_accepted" : "goal_rejected";
      };
    options.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            last_status_ = "goal_succeeded";
            break;
          case rclcpp_action::ResultCode::ABORTED:
            last_status_ = "goal_aborted";
            break;
          case rclcpp_action::ResultCode::CANCELED:
            last_status_ = "goal_canceled";
            break;
          default:
            last_status_ = "goal_unknown_result";
            break;
        }
      };

    action_client_->async_send_goal(goal, options);
    ++sent_goals_;
    last_goal_time_ = now();
    last_status_ = "goal_sent";
  }

  void publishHeartbeat()
  {
    std_msgs::msg::String heartbeat;
    if (!enabled_) {
      heartbeat.data = "disabled";
    } else {
      heartbeat.data = last_status_ + ":sent_goals=" + std::to_string(sent_goals_);
    }
    heartbeat_pub_->publish(heartbeat);
  }

  bool modeAllowsGoal() const
  {
    if (!respect_autonomy_mode_) {
      return true;
    }
    return has_autonomy_state_ && autonomy_allows_goal_;
  }

  bool enabled_{true};
  bool respect_autonomy_mode_{true};
  bool has_autonomy_state_{false};
  bool autonomy_allows_goal_{false};
  double server_wait_timeout_sec_{0.5};
  std::string behavior_tree_;
  std::string last_status_{"waiting_for_goal_pose"};
  std::vector<std::string> active_modes_;
  std::size_t sent_goals_{0};
  rclcpp::Time last_goal_time_{0, 0, RCL_ROS_TIME};
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<autonomy::msg::AutonomyState>::SharedPtr autonomy_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::GoalPoseToNav2ActionNode>());
  rclcpp::shutdown();
  return 0;
}
