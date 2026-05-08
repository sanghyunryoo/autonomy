#include <algorithm>
#include <chrono>
#include <cctype>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "height_map_ros2/msg/autonomy_state.hpp"
#include "height_map_ros2/srv/set_autonomy_mode.hpp"
#include "height_map_ros2/srv/set_estop.hpp"

namespace height_map_ros2
{
namespace
{

using AutonomyStateMsg = height_map_ros2::msg::AutonomyState;

std::string upper(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return text;
}

std::string modeName(const std::uint8_t mode)
{
  switch (mode) {
    case AutonomyStateMsg::IDLE:
      return "IDLE";
    case AutonomyStateMsg::DRIVE:
      return "DRIVE";
    case AutonomyStateMsg::ADAS:
      return "ADAS";
    case AutonomyStateMsg::FSD:
      return "FSD";
    case AutonomyStateMsg::MAPPING:
      return "MAPPING";
    case AutonomyStateMsg::ESTOP:
      return "ESTOP";
    case AutonomyStateMsg::ERROR:
    default:
      return "ERROR";
  }
}

std::uint8_t parseMode(const std::string & text)
{
  const auto value = upper(text);
  if (value == "IDLE") {
    return AutonomyStateMsg::IDLE;
  }
  if (value == "DRIVE") {
    return AutonomyStateMsg::DRIVE;
  }
  if (value == "ADAS") {
    return AutonomyStateMsg::ADAS;
  }
  if (value == "FSD") {
    return AutonomyStateMsg::FSD;
  }
  if (value == "MAPPING") {
    return AutonomyStateMsg::MAPPING;
  }
  if (value == "ESTOP") {
    return AutonomyStateMsg::ESTOP;
  }
  return AutonomyStateMsg::ERROR;
}

bool validMode(const std::uint8_t mode)
{
  return mode <= AutonomyStateMsg::ESTOP;
}

struct NodeStatus
{
  bool seen{false};
  std::string status{"unknown"};
  rclcpp::Time last_seen;
};

}  // namespace

class AutonomyManagerNode final : public rclcpp::Node
{
public:
  AutonomyManagerNode()
  : Node("autonomy_manager")
  {
    declare_parameter<std::string>("startup_mode", "IDLE");
    declare_parameter<double>("heartbeat_timeout_sec", 1.0);
    declare_parameter<double>("publish_rate_hz", 10.0);
    declare_parameter<std::vector<std::string>>("managed_nodes.idle", {});
    declare_parameter<std::vector<std::string>>(
      "managed_nodes.drive",
      {"realsense_usb_mapper", "pointcloud_merge_node", "elevation_mapping_node"});
    declare_parameter<std::vector<std::string>>(
      "managed_nodes.adas",
      {"realsense_usb_mapper", "pointcloud_merge_node", "elevation_mapping_node", "ai_detection_node"});
    declare_parameter<std::vector<std::string>>(
      "managed_nodes.fsd",
      {
        "realsense_usb_mapper",
        "pointcloud_merge_node",
        "elevation_mapping_node",
        "ai_detection_node",
        "rl_local_planner_node",
        "global_planner_node",
        "orbslam3_node",
      });
    declare_parameter<std::vector<std::string>>(
      "managed_nodes.mapping",
      {"realsense_usb_mapper", "pointcloud_merge_node", "elevation_mapping_node", "orbslam3_node"});

    mode_ = parseMode(get_parameter("startup_mode").as_string());
    managed_nodes_[AutonomyStateMsg::IDLE] = getStringArray("managed_nodes.idle");
    managed_nodes_[AutonomyStateMsg::DRIVE] = getStringArray("managed_nodes.drive");
    managed_nodes_[AutonomyStateMsg::ADAS] = getStringArray("managed_nodes.adas");
    managed_nodes_[AutonomyStateMsg::FSD] = getStringArray("managed_nodes.fsd");
    managed_nodes_[AutonomyStateMsg::MAPPING] = getStringArray("managed_nodes.mapping");
    managed_nodes_[AutonomyStateMsg::ERROR] = {};
    managed_nodes_[AutonomyStateMsg::ESTOP] = {};

    std::vector<std::string> all_nodes;
    for (const auto & item : managed_nodes_) {
      all_nodes.insert(all_nodes.end(), item.second.begin(), item.second.end());
    }
    std::sort(all_nodes.begin(), all_nodes.end());
    all_nodes.erase(std::unique(all_nodes.begin(), all_nodes.end()), all_nodes.end());

    for (const auto & node_name : all_nodes) {
      node_status_[node_name] = NodeStatus{};
      heartbeat_subs_.push_back(create_subscription<std_msgs::msg::String>(
        "/autonomy/heartbeat/" + node_name,
        10,
        [this, node_name](std_msgs::msg::String::SharedPtr msg) {
          auto & status = node_status_[node_name];
          status.seen = true;
          status.status = msg->data;
          status.last_seen = now();
        }));
    }

    state_pub_ = create_publisher<AutonomyStateMsg>("~/state", 10);
    set_mode_srv_ = create_service<height_map_ros2::srv::SetAutonomyMode>(
      "~/set_mode",
      std::bind(&AutonomyManagerNode::onSetMode, this, std::placeholders::_1, std::placeholders::_2));
    set_estop_srv_ = create_service<height_map_ros2::srv::SetEstop>(
      "~/set_estop",
      std::bind(&AutonomyManagerNode::onSetEstop, this, std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::duration<double>(
      1.0 / std::max(1.0, get_parameter("publish_rate_hz").as_double()));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publishState(); });

    RCLCPP_INFO(get_logger(), "Autonomy manager started in %s", modeName(mode_).c_str());
  }

private:
  std::vector<std::string> getStringArray(const std::string & name)
  {
    return get_parameter(name).as_string_array();
  }

  void onSetMode(
    const std::shared_ptr<height_map_ros2::srv::SetAutonomyMode::Request> request,
    std::shared_ptr<height_map_ros2::srv::SetAutonomyMode::Response> response)
  {
    if (!validMode(request->mode)) {
      response->accepted = false;
      response->current_mode = mode_;
      response->message = "Invalid mode";
      return;
    }
    if (estop_active_ && request->mode != AutonomyStateMsg::ESTOP) {
      response->accepted = false;
      response->current_mode = mode_;
      response->message = "ESTOP is active";
      return;
    }
    mode_ = request->mode;
    error_active_ = mode_ == AutonomyStateMsg::ERROR;
    response->accepted = true;
    response->current_mode = mode_;
    response->message = "Mode changed to " + modeName(mode_);
    RCLCPP_INFO(
      get_logger(),
      "Mode changed to %s reason='%s'",
      modeName(mode_).c_str(),
      request->reason.c_str());
  }

  void onSetEstop(
    const std::shared_ptr<height_map_ros2::srv::SetEstop::Request> request,
    std::shared_ptr<height_map_ros2::srv::SetEstop::Response> response)
  {
    estop_active_ = request->active;
    mode_ = estop_active_ ? AutonomyStateMsg::ESTOP : AutonomyStateMsg::IDLE;
    response->accepted = true;
    response->current_mode = mode_;
    response->message = estop_active_ ? "ESTOP active" : "ESTOP cleared";
    RCLCPP_WARN(
      get_logger(),
      "ESTOP %s reason='%s'",
      estop_active_ ? "active" : "cleared",
      request->reason.c_str());
  }

  void classifyNodes(
    std::vector<std::string> & active,
    std::vector<std::string> & inactive,
    std::vector<std::string> & degraded) const
  {
    const auto expected_it = managed_nodes_.find(mode_);
    const std::vector<std::string> expected =
      expected_it == managed_nodes_.end() ? std::vector<std::string>{} : expected_it->second;
    const auto timeout = rclcpp::Duration::from_seconds(get_parameter("heartbeat_timeout_sec").as_double());
    const auto time_now = now();

    for (const auto & item : node_status_) {
      const auto & node_name = item.first;
      const auto & status = item.second;
      const bool expected_active =
        std::find(expected.begin(), expected.end(), node_name) != expected.end();
      const bool fresh = status.seen && ((time_now - status.last_seen) <= timeout);
      if (expected_active && fresh && status.status != "disabled") {
        active.push_back(node_name);
      } else if (expected_active) {
        degraded.push_back(node_name);
      } else {
        inactive.push_back(node_name);
      }
    }
  }

  void publishState()
  {
    std::vector<std::string> active;
    std::vector<std::string> inactive;
    std::vector<std::string> degraded;
    classifyNodes(active, inactive, degraded);

    AutonomyStateMsg msg;
    msg.header.stamp = now();
    msg.mode = mode_;
    msg.mode_name = modeName(mode_);
    msg.autonomy_enabled = mode_ == AutonomyStateMsg::ADAS || mode_ == AutonomyStateMsg::FSD;
    msg.estop_active = estop_active_;
    msg.error_active = error_active_ || !degraded.empty();
    msg.status = degraded.empty() ? "ok" : "degraded";
    msg.active_nodes = active;
    msg.inactive_nodes = inactive;
    msg.degraded_nodes = degraded;
    state_pub_->publish(msg);
  }

  std::uint8_t mode_{AutonomyStateMsg::IDLE};
  bool estop_active_{false};
  bool error_active_{false};
  std::unordered_map<std::uint8_t, std::vector<std::string>> managed_nodes_;
  std::unordered_map<std::string, NodeStatus> node_status_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> heartbeat_subs_;
  rclcpp::Publisher<AutonomyStateMsg>::SharedPtr state_pub_;
  rclcpp::Service<height_map_ros2::srv::SetAutonomyMode>::SharedPtr set_mode_srv_;
  rclcpp::Service<height_map_ros2::srv::SetEstop>::SharedPtr set_estop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace height_map_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<height_map_ros2::AutonomyManagerNode>());
  rclcpp::shutdown();
  return 0;
}
