#include <chrono>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace height_map_ros2
{

class GlobalPlannerNode final : public rclcpp::Node
{
public:
  GlobalPlannerNode()
  : Node("global_planner_node")
  {
    declare_parameter<bool>("enabled", false);
    declare_parameter<std::string>("goal_topic", "/goal_pose");
    declare_parameter<std::string>("path_topic", "/planning/global_path");
    enabled_ = get_parameter("enabled").as_bool();
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("goal_topic").as_string(),
      10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { latest_goal_ = std::move(msg); });
    path_pub_ = create_publisher<nav_msgs::msg::Path>(get_parameter("path_topic").as_string(), 10);
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>("/autonomy/heartbeat/global_planner_node", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { tick(); });
  }

private:
  void tick()
  {
    std_msgs::msg::String heartbeat;
    heartbeat.data = enabled_ ? "ready" : "disabled";
    heartbeat_pub_->publish(heartbeat);
    if (enabled_ && latest_goal_) {
      nav_msgs::msg::Path path;
      path.header = latest_goal_->header;
      path.poses.push_back(*latest_goal_);
      path_pub_->publish(path);
    }
  }

  bool enabled_{false};
  geometry_msgs::msg::PoseStamped::SharedPtr latest_goal_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace height_map_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<height_map_ros2::GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
