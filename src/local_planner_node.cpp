#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace autonomy
{
namespace
{

class LocalPlannerNode : public rclcpp::Node
{
public:
  LocalPlannerNode()
  : Node("local_planner_node")
  {
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>("/autonomy/heartbeat/local_planner_node", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
      std_msgs::msg::String msg;
      msg.data = "currently_not_supported:skeleton";
      heartbeat_pub_->publish(msg);
    });

    RCLCPP_WARN(get_logger(), "Local planner skeleton loaded; planner behavior is not implemented yet.");
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::LocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
