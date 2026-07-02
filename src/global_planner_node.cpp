#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace autonomy
{
namespace
{

class GlobalPlannerNode final : public rclcpp::Node
{
public:
  GlobalPlannerNode()
  : Node("global_planner_node")
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "base_footprint");
    local_goal_topic_ = declare_parameter<std::string>("local_goal_topic", "/global_planner_node/local_goal");
    path_topic_ = declare_parameter<std::string>("path_topic", "/global_planner_node/path");
    lookahead_distance_ = std::max(0.1, declare_parameter<double>("lookahead_distance", 1.2));
    lateral_offset_ = declare_parameter<double>("lateral_offset", 0.0);
    publish_rate_hz_ = std::max(1.0, declare_parameter<double>("publish_rate_hz", 10.0));

    heartbeat_pub_ = create_publisher<std_msgs::msg::String>("/autonomy/heartbeat/global_planner_node", 10);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(local_goal_topic_, 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { tick(); });

    RCLCPP_INFO(
      get_logger(),
      "Mapless global planner publishing local goal %.2fm ahead in %s.",
      lookahead_distance_,
      target_frame_.c_str());
  }

private:
  void tick()
  {
    std_msgs::msg::String heartbeat;
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = target_frame_;
    goal.pose.position.x = lookahead_distance_;
    goal.pose.position.y = lateral_offset_;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.w = 1.0;
    goal_pub_->publish(goal);

    nav_msgs::msg::Path path;
    path.header = goal.header;
    geometry_msgs::msg::PoseStamped start;
    start.header = path.header;
    start.pose.orientation.w = 1.0;
    path.poses.push_back(start);
    path.poses.push_back(goal);
    path_pub_->publish(path);

    heartbeat.data = "ready:mapless_goal";
    heartbeat_pub_->publish(heartbeat);
  }

  std::string target_frame_{"base_footprint"};
  std::string local_goal_topic_{"/global_planner_node/local_goal"};
  std::string path_topic_{"/global_planner_node/path"};
  double lookahead_distance_{1.2};
  double lateral_offset_{0.0};
  double publish_rate_hz_{10.0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
