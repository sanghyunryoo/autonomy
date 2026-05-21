#include <algorithm>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

#include "height_map_ros2/msg/autonomy_state.hpp"
#include "core/msg/robot_report.hpp"
#include "height_map_ros2/srv/set_autonomy_mode.hpp"

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

std::string modeName(const std::int8_t mode)
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
    case AutonomyStateMsg::TRACKING:
      return "TRACKING";
    case AutonomyStateMsg::ERROR:
    default:
      return "ERROR";
  }
}

std::int8_t parseMode(const std::string & text)
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
  if (value == "TRACKING") {
    return AutonomyStateMsg::TRACKING;
  }
  return AutonomyStateMsg::ERROR;
}

bool validModeText(const std::string & text)
{
  const auto value = upper(text);
  return value == "IDLE" || value == "DRIVE" || value == "ADAS" || value == "FSD" ||
         value == "MAPPING" || value == "TRACKING";
}

bool reportRequestsDrive(const std::uint8_t state, const std::string & name)
{
  const auto value = upper(name);
  return (state >= 2 && state <= 6) ||
         value == "READY" ||
         value == "STAND" ||
         value == "FLAT_DRIVE" ||
         value == "ROUGH_DRIVE" ||
         value == "CUSTOM_DRIVE";
}

bool reportRequestsIdle(const std::uint8_t state, const std::string & name)
{
  const auto value = upper(name);
  return state == 0 || state == 1 || state == 7 || state == 8 || state == 9 ||
         value == "IDLE" ||
         value == "INIT" ||
         value == "FREEZE" ||
         value == "SIT" ||
         value == "LIE";
}

bool startsWith(const std::string & text, const std::string & prefix)
{
  return text.rfind(prefix, 0) == 0;
}

bool isNodeFailureStatus(const std::string & status)
{
  return startsWith(status, "error:") ||
         startsWith(status, "fatal:") ||
         startsWith(status, "exception:") ||
         startsWith(status, "degraded:");
}

struct NodeStatus
{
  NodeStatus()
  : seen(false),
    status("unknown"),
    last_seen(0, 0u, RCL_SYSTEM_TIME)
  {
  }

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
    declare_parameter<std::string>("pose_topic", "/odomimu");
    declare_parameter<std::string>("robot_report_topic", "/robot_report");
    declare_parameter<std::string>("map_dir", "");
    declare_parameter<double>("speed_limit", 0.0);
    declare_parameter<bool>("enable_ai", false);
    declare_parameter<bool>("segmentation", false);
    declare_parameter<double>("heartbeat_timeout_sec", 1.0);
    declare_parameter<double>("publish_rate_hz", 10.0);
    declare_parameter<std::vector<std::string>>(
      "managed_nodes.idle",
      std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>(
      "managed_nodes.drive",
      {"realsense_usb_mapper", "pointcloud_merge_node", "elevation_mapping_node"});
    declare_parameter<std::vector<std::string>>(
      "managed_nodes.adas",
      {"realsense_usb_mapper", "pointcloud_merge_node", "elevation_mapping_node", "openvins_vio_node", "ai_detection_node"});
    declare_parameter<std::vector<std::string>>(
      "managed_nodes.fsd",
      {
        "realsense_usb_mapper",
        "pointcloud_merge_node",
        "elevation_mapping_node",
        "ai_detection_node",
        "rl_local_planner_node",
        "global_planner_node",
        "openvins_vio_node",
      });
    declare_parameter<std::vector<std::string>>(
      "managed_nodes.mapping",
      std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>(
      "managed_nodes.tracking",
      {
        "realsense_usb_mapper",
        "pointcloud_merge_node",
        "elevation_mapping_node",
        "ai_detection_node",
        "tracking_follower_node",
      });

    mode_ = parseMode(get_parameter("startup_mode").as_string());
    speed_limit_ = static_cast<float>(get_parameter("speed_limit").as_double());
    ai_enabled_ = get_parameter("enable_ai").as_bool();
    segmentation_enabled_ = get_parameter("segmentation").as_bool();
    requested_mode_ = mode_;
    requested_speed_limit_ = speed_limit_;
    requested_ai_enabled_ = ai_enabled_;
    requested_segmentation_enabled_ = segmentation_enabled_;
    map_dir_ = get_parameter("map_dir").as_string();
    managed_nodes_[AutonomyStateMsg::IDLE] = getStringArray("managed_nodes.idle");
    managed_nodes_[AutonomyStateMsg::DRIVE] = getStringArray("managed_nodes.drive");
    managed_nodes_[AutonomyStateMsg::ADAS] = getStringArray("managed_nodes.adas");
    managed_nodes_[AutonomyStateMsg::FSD] = getStringArray("managed_nodes.fsd");
    managed_nodes_[AutonomyStateMsg::MAPPING] = getStringArray("managed_nodes.mapping");
    managed_nodes_[AutonomyStateMsg::TRACKING] = getStringArray("managed_nodes.tracking");
    managed_nodes_[AutonomyStateMsg::ERROR] = {};

    std::vector<std::string> all_nodes;
    for (const auto & item : managed_nodes_) {
      all_nodes.insert(all_nodes.end(), item.second.begin(), item.second.end());
    }
    std::sort(all_nodes.begin(), all_nodes.end());
    all_nodes.erase(std::unique(all_nodes.begin(), all_nodes.end()), all_nodes.end());

    for (const auto & node_name : all_nodes) {
      node_status_[node_name] = NodeStatus();
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

    pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("pose_topic").as_string(),
      10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_pose_ = msg->pose.pose;
        latest_velocity_ = msg->twist.twist;
        has_pose_ = true;
      });
    robot_report_sub_ = create_subscription<core::msg::RobotReport>(
      get_parameter("robot_report_topic").as_string(),
      10,
      [this](core::msg::RobotReport::SharedPtr msg) {
        onRobotReport(std::move(msg));
      });

    state_pub_ = create_publisher<AutonomyStateMsg>("~/state", 10);
    status_pub_ = create_publisher<AutonomyStateMsg>("~/status", 10);
    set_mode_srv_ = create_service<height_map_ros2::srv::SetAutonomyMode>(
      "~/set_mode",
      std::bind(&AutonomyManagerNode::onSetMode, this, std::placeholders::_1, std::placeholders::_2));

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
    response->current_mode = mode_;
    response->enable_ai = ai_enabled_;
    response->segmentation = segmentation_enabled_;

    if (!validModeText(request->operation_mode)) {
      response->accepted = false;
      response->message = "Invalid operation_mode";
      return;
    }

    const auto requested_mode = parseMode(request->operation_mode);
    const bool report_controlled_mode =
      requested_mode == AutonomyStateMsg::IDLE || requested_mode == AutonomyStateMsg::DRIVE;

    if (requested_mode == AutonomyStateMsg::FSD && !mapReady()) {
      response->accepted = false;
      response->message = "FSD requires at least one map file in map_dir: " + map_dir_;
      error_code_ = 1001;
      return;
    }
    if (requested_mode == AutonomyStateMsg::TRACKING && !request->enable_ai) {
      response->accepted = false;
      response->message = "TRACKING requires enable_ai=true";
      error_code_ = 1002;
      return;
    }

    requested_mode_ = requested_mode;
    requested_speed_limit_ = request->speed_limit;
    requested_ai_enabled_ = request->enable_ai;
    requested_segmentation_enabled_ = request->segmentation;

    if (!estop_active_) {
      applyRequestedMode(report_controlled_mode);
      response->message = "Mode changed to " + modeName(mode_);
      RCLCPP_INFO(
        get_logger(),
        "Mode changed to %s ai=%s segmentation=%s",
        modeName(mode_).c_str(),
        boolText(ai_enabled_).c_str(),
        boolText(segmentation_enabled_).c_str());
    } else {
      response->message = "Mode queued until ESTOP is cleared: " + modeName(requested_mode_);
      RCLCPP_INFO(
        get_logger(),
        "Mode queued during ESTOP: %s ai=%s segmentation=%s",
        modeName(requested_mode_).c_str(),
        boolText(requested_ai_enabled_).c_str(),
        boolText(requested_segmentation_enabled_).c_str());
    }

    error_code_ = estop_active_ ? 2001 : (comm_fault_ ? 3001 : 0);
    error_active_ = !estop_active_ && (mode_ == AutonomyStateMsg::ERROR || comm_fault_);
    response->accepted = true;
    response->current_mode = mode_;
    response->enable_ai = ai_enabled_;
    response->segmentation = segmentation_enabled_;
  }

  void onRobotReport(core::msg::RobotReport::SharedPtr msg)
  {
    robot_state_ = msg->robot_state;
    robot_state_name_ = msg->robot_state_name;
    has_robot_report_ = true;
    physical_estop_ = msg->physical_estop;
    comm_estop_ = msg->comm_estop;
    comm_fault_ = msg->comm_fault;
    error_reason_ = msg->error_reason;

    const bool next_robot_estop = physical_estop_ || comm_estop_;
    if (next_robot_estop && !estop_active_) {
      forceIdleForEstop();
      RCLCPP_WARN(
        get_logger(),
        "Robot ESTOP active from /robot_report physical_estop=%s comm_estop=%s",
        boolText(physical_estop_).c_str(),
        boolText(comm_estop_).c_str());
    } else if (!next_robot_estop && estop_active_) {
      restoreRequestedModeAfterEstop();
      RCLCPP_WARN(
        get_logger(),
        "Robot ESTOP cleared from /robot_report; restored mode %s",
        modeName(mode_).c_str());
    }
    estop_active_ = next_robot_estop;

    if (comm_fault_ && error_reason_.empty()) {
      error_reason_ = "robot communication fault";
    }

    if (!estop_active_ && reportControlsMode()) {
      applyRobotReportMode();
    }
  }

  void classifyNodes(
    std::vector<std::string> & active,
    std::vector<std::string> & inactive,
    std::vector<std::string> & degraded) const
  {
    std::vector<std::string> expected = expectedNodesForCurrentMode();
    const auto timeout = rclcpp::Duration::from_seconds(get_parameter("heartbeat_timeout_sec").as_double());
    const auto time_now = now();

    for (const auto & item : node_status_) {
      const auto & node_name = item.first;
      const auto & status = item.second;
      const bool expected_active =
        std::find(expected.begin(), expected.end(), node_name) != expected.end();
      const bool fresh = status.seen && ((time_now - status.last_seen) <= timeout);
      const bool node_failure = fresh && isNodeFailureStatus(status.status);
      if (expected_active && (!fresh || status.status == "disabled" || node_failure)) {
        degraded.push_back(node_name);
      } else if (expected_active) {
        active.push_back(node_name);
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
    const bool internal_error_active = error_active_ || !degraded.empty() || comm_fault_;
    const auto reported_mode = internal_error_active ? AutonomyStateMsg::ERROR : mode_;

    AutonomyStateMsg msg;
    msg.header.stamp = now();
    msg.mode = reported_mode;
    msg.mode_name = modeName(reported_mode);
    msg.autonomy_enabled = mode_ == AutonomyStateMsg::ADAS || mode_ == AutonomyStateMsg::FSD ||
      mode_ == AutonomyStateMsg::TRACKING;
    msg.estop_active = estop_active_;
    msg.error_active = internal_error_active;
    msg.ai_enabled = ai_enabled_;
    msg.segmentation_enabled = segmentation_enabled_;
    msg.robot_state = robot_state_;
    msg.robot_state_name = robot_state_name_;
    msg.physical_estop = physical_estop_;
    msg.comm_estop = comm_estop_;
    msg.comm_fault = comm_fault_;
    msg.error_reason = formatErrorReason(degraded);
    msg.speed_limit = speed_limit_;
    msg.error_code = msg.error_active ? std::max(error_code_, comm_fault_ ? 3001 : 1) : error_code_;
    if (has_pose_) {
      msg.pose = latest_pose_;
      msg.velocity = latest_velocity_;
    }
    msg.status = internal_error_active ? "error" : "ok";
    msg.active_nodes = active;
    msg.inactive_nodes = inactive;
    msg.degraded_nodes = degraded;
    for (const auto & item : node_status_) {
      msg.node_names.push_back(item.first);
      msg.node_statuses.push_back(nodeStatusText(item.first));
    }
    state_pub_->publish(msg);
    status_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "%s",
      formatStatusLog(msg, active, inactive, degraded).c_str());
  }

  std::string formatStatusLog(
    const AutonomyStateMsg & msg,
    const std::vector<std::string> & active,
    const std::vector<std::string> & inactive,
    const std::vector<std::string> & degraded) const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "\n"
        << "========== AUTONOMY STATUS ==========\n"
        << "mode=" << msg.mode_name
        << " status=" << msg.status
        << " error_code=" << msg.error_code
        << " estop=" << boolText(msg.estop_active)
        << " physical_estop=" << boolText(msg.physical_estop)
        << " comm_estop=" << boolText(msg.comm_estop)
        << " comm_fault=" << boolText(msg.comm_fault)
        << " ai=" << boolText(msg.ai_enabled)
        << " segmentation=" << boolText(msg.segmentation_enabled)
        << " speed_limit=" << std::setprecision(2) << msg.speed_limit << "\n";
    out << "robot_state=" << static_cast<int>(msg.robot_state)
        << " name=" << msg.robot_state_name
        << " error_reason=" << msg.error_reason << "\n";

    out << std::setprecision(3)
        << "robot pose xyz=(" << msg.pose.position.x << ", " << msg.pose.position.y << ", "
        << msg.pose.position.z << ")"
        << " quat=(" << msg.pose.orientation.x << ", " << msg.pose.orientation.y << ", "
        << msg.pose.orientation.z << ", " << msg.pose.orientation.w << ")"
        << "\n";
    out << "robot velocity linear=(" << msg.velocity.linear.x << ", " << msg.velocity.linear.y
        << ", " << msg.velocity.linear.z << ")"
        << " angular=(" << msg.velocity.angular.x << ", " << msg.velocity.angular.y
        << ", " << msg.velocity.angular.z << ")\n";

    out << "nodes active=" << active.size()
        << " degraded=" << degraded.size()
        << " inactive=" << inactive.size() << "\n";
    appendNodeGroup(out, "ACTIVE  ", active);
    appendNodeGroup(out, "DEGRADED", degraded);
    appendNodeGroup(out, "INACTIVE", inactive);
    out << "=====================================";
    return out.str();
  }

  std::string boolText(const bool value) const
  {
    return value ? "true" : "false";
  }

  void appendNodeGroup(
    std::ostringstream & out,
    const std::string & label,
    const std::vector<std::string> & nodes) const
  {
    if (nodes.empty()) {
      out << label << ": none\n";
      return;
    }
    out << label << ":\n";
    for (const auto & node : nodes) {
      out << "  - " << node << ": " << nodeStatusText(node) << "\n";
    }
  }

  std::string formatErrorReason(const std::vector<std::string> & degraded) const
  {
    std::ostringstream out;
    if (!error_reason_.empty()) {
      out << error_reason_;
    }
    if (!degraded.empty()) {
      if (!out.str().empty()) {
        out << "; ";
      }
      out << "degraded nodes: ";
      for (std::size_t i = 0; i < degraded.size(); ++i) {
        if (i > 0) {
          out << ", ";
        }
        out << degraded[i] << "(" << nodeStatusText(degraded[i]) << ")";
      }
    }
    if (comm_fault_ && out.str().empty()) {
      out << "robot communication fault";
    }
    return out.str();
  }

  std::string nodeStatusText(const std::string & node_name) const
  {
    const auto status_it = node_status_.find(node_name);
    if (status_it == node_status_.end()) {
      return "unknown";
    }

    const auto & status = status_it->second;
    if (!status.seen) {
      return "no_heartbeat";
    }

    const auto timeout = rclcpp::Duration::from_seconds(get_parameter("heartbeat_timeout_sec").as_double());
    const bool fresh = (now() - status.last_seen) <= timeout;
    if (!fresh) {
      return "heartbeat_timeout:last_status=" + status.status;
    }
    return status.status;
  }

  std::vector<std::string> expectedNodesForCurrentMode() const
  {
    const auto expected_it = managed_nodes_.find(mode_);
    std::vector<std::string> expected =
      expected_it == managed_nodes_.end() ? std::vector<std::string>{} : expected_it->second;

    if ((mode_ == AutonomyStateMsg::ADAS || mode_ == AutonomyStateMsg::FSD ||
      mode_ == AutonomyStateMsg::TRACKING) && ai_enabled_)
    {
      if (std::find(expected.begin(), expected.end(), "ai_detection_node") == expected.end()) {
        expected.push_back("ai_detection_node");
      }
    }
    if (!ai_enabled_) {
      expected.erase(
        std::remove(expected.begin(), expected.end(), "ai_detection_node"),
        expected.end());
    }
    return expected;
  }

  bool reportControlsMode() const
  {
    return requested_mode_ == AutonomyStateMsg::IDLE || requested_mode_ == AutonomyStateMsg::DRIVE;
  }

  void applyRobotReportMode()
  {
    if (!has_robot_report_) {
      mode_ = AutonomyStateMsg::IDLE;
      speed_limit_ = 0.0F;
      ai_enabled_ = false;
      segmentation_enabled_ = false;
      return;
    }

    if (reportRequestsDrive(robot_state_, robot_state_name_)) {
      mode_ = AutonomyStateMsg::DRIVE;
      speed_limit_ = requested_speed_limit_;
      ai_enabled_ = false;
      segmentation_enabled_ = requested_segmentation_enabled_;
      return;
    }

    if (reportRequestsIdle(robot_state_, robot_state_name_)) {
      mode_ = AutonomyStateMsg::IDLE;
      speed_limit_ = 0.0F;
      ai_enabled_ = false;
      segmentation_enabled_ = false;
      return;
    }
  }

  void applyRequestedMode(const bool use_robot_report_for_drive_idle = true)
  {
    if (use_robot_report_for_drive_idle && reportControlsMode()) {
      applyRobotReportMode();
      return;
    }

    mode_ = requested_mode_;
    speed_limit_ = requested_speed_limit_;
    ai_enabled_ = requested_ai_enabled_;
    segmentation_enabled_ = requested_segmentation_enabled_;
  }

  void forceIdleForEstop()
  {
    if (!estop_active_ && mode_ != AutonomyStateMsg::IDLE) {
      requested_mode_ = mode_;
      requested_speed_limit_ = speed_limit_;
      requested_ai_enabled_ = ai_enabled_;
      requested_segmentation_enabled_ = segmentation_enabled_;
    }
    estop_active_ = true;
    mode_ = AutonomyStateMsg::IDLE;
    speed_limit_ = 0.0F;
    ai_enabled_ = false;
    segmentation_enabled_ = false;
    error_code_ = 2001;
    error_active_ = false;
  }

  void restoreRequestedModeAfterEstop()
  {
    estop_active_ = false;
    applyRequestedMode();
    error_code_ = comm_fault_ ? 3001 : 0;
    error_active_ = mode_ == AutonomyStateMsg::ERROR || comm_fault_;
  }

  bool mapReady() const
  {
    if (map_dir_.empty()) {
      return false;
    }
    const std::filesystem::path path(map_dir_);
    if (!std::filesystem::exists(path) || !std::filesystem::is_directory(path)) {
      return false;
    }
    for (const auto & entry : std::filesystem::directory_iterator(path)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      const auto name = entry.path().filename().string();
      if (!name.empty() && name.front() != '.') {
        return true;
      }
    }
    return false;
  }

  std::int8_t mode_{AutonomyStateMsg::IDLE};
  std::int8_t requested_mode_{AutonomyStateMsg::IDLE};
  bool estop_active_{false};
  bool error_active_{false};
  bool ai_enabled_{false};
  bool requested_ai_enabled_{false};
  bool segmentation_enabled_{false};
  bool requested_segmentation_enabled_{false};
  bool has_pose_{false};
  bool has_robot_report_{false};
  float speed_limit_{0.0F};
  float requested_speed_limit_{0.0F};
  int error_code_{0};
  std::uint8_t robot_state_{0};
  std::string robot_state_name_;
  bool physical_estop_{false};
  bool comm_estop_{false};
  bool comm_fault_{false};
  std::string error_reason_;
  std::string map_dir_;
  geometry_msgs::msg::Pose latest_pose_;
  geometry_msgs::msg::Twist latest_velocity_;
  std::unordered_map<std::int8_t, std::vector<std::string>> managed_nodes_;
  std::unordered_map<std::string, NodeStatus> node_status_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> heartbeat_subs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<core::msg::RobotReport>::SharedPtr robot_report_sub_;
  rclcpp::Publisher<AutonomyStateMsg>::SharedPtr state_pub_;
  rclcpp::Publisher<AutonomyStateMsg>::SharedPtr status_pub_;
  rclcpp::Service<height_map_ros2::srv::SetAutonomyMode>::SharedPtr set_mode_srv_;
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
