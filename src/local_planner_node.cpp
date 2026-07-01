#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <core/msg/command_filter.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "autonomy/msg/masked_height_scan.hpp"
#include "dds_linear_velocity_publisher.hpp"

namespace autonomy
{
namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

struct Candidate
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  double score{-kInf};
  double min_clearance{kInf};
};

class LocalPlannerNode final : public rclcpp::Node
{
public:
  LocalPlannerNode()
  : Node("local_planner_node")
  {
    enable_map_ = declare_parameter<bool>("enable_map", false);
    height_scan_topic_ = declare_parameter<std::string>(
      "height_scan_topic", "/elevation_mapping_node/local_terrain_map");
    command_filter_topic_ = declare_parameter<std::string>("command_filter_topic", "/command_filter");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/global_planner_node/local_goal");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/local_planner_node/cmd_vel");
    publish_rate_hz_ = std::max(1.0, declare_parameter<double>("publish_rate_hz", 20.0));
    max_vx_ = std::max(0.0, declare_parameter<double>("max_vx", 0.35));
    max_vy_ = std::max(0.0, declare_parameter<double>("max_vy", 0.22));
    max_wz_ = std::max(0.0, declare_parameter<double>("max_wz", 0.7));
    min_forward_vx_ = std::clamp(declare_parameter<double>("min_forward_vx", 0.05), 0.0, max_vx_);
    robot_radius_ = std::max(0.05, declare_parameter<double>("robot_radius", 0.32));
    rollout_time_ = std::max(0.2, declare_parameter<double>("rollout_time", 1.2));
    rollout_dt_ = std::max(0.05, declare_parameter<double>("rollout_dt", 0.15));
    obstacle_height_threshold_ = declare_parameter<double>("obstacle_height_threshold", 0.18);
    step_height_threshold_ = declare_parameter<double>("step_height_threshold", 0.10);
    unknown_is_blocked_ = declare_parameter<bool>("unknown_is_blocked", false);
    stale_timeout_sec_ = std::max(0.1, declare_parameter<double>("stale_timeout_sec", 1.0));
    dds_enabled_ = declare_parameter<bool>("dds.enabled", true);
    dds_domain_id_ = declare_parameter<int>("dds.domain_id", 1);
    dds_topic_ = declare_parameter<std::string>("dds.lin_vel.topic", "lin_vel");
    dds_type_ = declare_parameter<std::string>("dds.lin_vel.type", "core_dds::LinearVelocity");

    heartbeat_pub_ = create_publisher<std_msgs::msg::String>("/autonomy/heartbeat/local_planner_node", 10);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    scan_sub_ = create_subscription<autonomy::msg::MaskedHeightScan>(
      height_scan_topic_,
      rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile(),
      [this](autonomy::msg::MaskedHeightScan::SharedPtr msg) { onScan(std::move(msg)); });
    filter_sub_ = create_subscription<core::msg::CommandFilter>(
      command_filter_topic_,
      10,
      [this](core::msg::CommandFilter::SharedPtr msg) { onFilter(std::move(msg)); });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_,
      10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { onGoal(std::move(msg)); });

    if (dds_enabled_) {
      dds_pub_ = std::make_unique<DdsLinearVelocityPublisher>(
        static_cast<std::uint32_t>(std::max(0, dds_domain_id_)),
        dds_topic_,
        dds_type_);
      if (!dds_pub_->isReady()) {
        RCLCPP_WARN(
          get_logger(),
          "DDS linear velocity publisher disabled: %s",
          dds_pub_->error().c_str());
        dds_pub_.reset();
      }
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { tick(); });

    RCLCPP_INFO(
      get_logger(),
      "Mapless terrain DWA local planner listening scan=%s goal=%s cmd=%s.",
      height_scan_topic_.c_str(),
      goal_topic_.c_str(),
      cmd_vel_topic_.c_str());
  }

private:
  void onScan(autonomy::msg::MaskedHeightScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    scan_ = std::move(msg);
    last_scan_time_ = now();
  }

  void onFilter(core::msg::CommandFilter::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    filter_ = *msg;
    has_filter_ = true;
  }

  void onGoal(geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_ = *msg;
    has_goal_ = true;
  }

  void tick()
  {
    std_msgs::msg::String heartbeat;
    if (enable_map_) {
      publishStop();
      heartbeat.data = "error:map_local_planning_not_supported";
      heartbeat_pub_->publish(heartbeat);
      return;
    }

    autonomy::msg::MaskedHeightScan::SharedPtr scan;
    core::msg::CommandFilter filter;
    geometry_msgs::msg::PoseStamped goal;
    bool has_filter = false;
    bool has_goal = false;
    rclcpp::Time last_scan;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      scan = scan_;
      filter = filter_;
      goal = goal_;
      has_filter = has_filter_;
      has_goal = has_goal_;
      last_scan = last_scan_time_;
    }

    if (!scan) {
      publishStop();
      heartbeat.data = "waiting_for_height_scan";
      heartbeat_pub_->publish(heartbeat);
      return;
    }
    if ((now() - last_scan) > rclcpp::Duration::from_seconds(stale_timeout_sec_)) {
      publishStop();
      heartbeat.data = "degraded:stale_height_scan";
      heartbeat_pub_->publish(heartbeat);
      return;
    }

    const Candidate best = chooseBest(*scan, has_filter ? &filter : nullptr, has_goal ? &goal : nullptr);
    publishCommand(best);
    heartbeat.data = "ready:vx=" + shortText(best.vx) +
      ":vy=" + shortText(best.vy) +
      ":wz=" + shortText(best.wz) +
      ":clearance=" + shortText(best.min_clearance);
    heartbeat_pub_->publish(heartbeat);
  }

  Candidate chooseBest(
    const autonomy::msg::MaskedHeightScan & scan,
    const core::msg::CommandFilter * filter,
    const geometry_msgs::msg::PoseStamped * goal) const
  {
    Candidate best;
    const std::vector<double> vx_samples{
      0.0,
      min_forward_vx_,
      0.5 * max_vx_,
      max_vx_};
    const std::vector<double> vy_samples{
      -max_vy_,
      -0.5 * max_vy_,
      0.0,
      0.5 * max_vy_,
      max_vy_};
    const std::vector<double> wz_samples{
      -max_wz_,
      -0.5 * max_wz_,
      0.0,
      0.5 * max_wz_,
      max_wz_};

    for (const double vx : vx_samples) {
      for (const double vy : vy_samples) {
        if (!allowedByFilter(vx, vy, filter)) {
          continue;
        }
        for (const double wz : wz_samples) {
          Candidate candidate;
          candidate.vx = vx;
          candidate.vy = vy;
          candidate.wz = wz;
          scoreCandidate(scan, goal, candidate);
          if (candidate.score > best.score) {
            best = candidate;
          }
        }
      }
    }

    if (best.score == -kInf) {
      return Candidate{};
    }
    return best;
  }

  bool allowedByFilter(const double vx, const double vy, const core::msg::CommandFilter * filter) const
  {
    if (filter == nullptr) {
      return true;
    }
    if (vx > 0.01 && !filter->allow_linear_vel_forward_x) {
      return false;
    }
    if (vx < -0.01 && !filter->allow_linear_vel_backward_x) {
      return false;
    }
    if (vy > 0.01 && !filter->allow_linear_vel_forward_y) {
      return false;
    }
    if (vy < -0.01 && !filter->allow_linear_vel_backward_y) {
      return false;
    }
    return true;
  }

  void scoreCandidate(
    const autonomy::msg::MaskedHeightScan & scan,
    const geometry_msgs::msg::PoseStamped * goal,
    Candidate & candidate) const
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double terrain_cost = 0.0;
    double min_clearance = kInf;
    int samples = 0;

    for (double t = 0.0; t <= rollout_time_; t += rollout_dt_) {
      x += (std::cos(yaw) * candidate.vx - std::sin(yaw) * candidate.vy) * rollout_dt_;
      y += (std::sin(yaw) * candidate.vx + std::cos(yaw) * candidate.vy) * rollout_dt_;
      yaw += candidate.wz * rollout_dt_;

      double clearance = 0.0;
      double local_cost = 0.0;
      if (!footprintCost(scan, x, y, clearance, local_cost)) {
        candidate.score = -kInf;
        return;
      }
      terrain_cost += local_cost;
      min_clearance = std::min(min_clearance, clearance);
      samples++;
    }

    const double speed_reward = candidate.vx;
    const double lateral_penalty = std::abs(candidate.vy);
    const double yaw_penalty = std::abs(candidate.wz);
    const double goal_reward = goalProgressReward(x, y, goal);
    const double clearance_reward = std::min(min_clearance, 1.0);
    const double normalized_terrain_cost = samples > 0 ? terrain_cost / static_cast<double>(samples) : 10.0;

    candidate.min_clearance = min_clearance;
    candidate.score =
      2.0 * goal_reward +
      1.5 * speed_reward +
      0.8 * clearance_reward -
      2.5 * normalized_terrain_cost -
      0.4 * lateral_penalty -
      0.25 * yaw_penalty;
  }

  double goalProgressReward(
    const double x,
    const double y,
    const geometry_msgs::msg::PoseStamped * goal) const
  {
    if (goal == nullptr) {
      return x;
    }
    const double gx = goal->pose.position.x;
    const double gy = goal->pose.position.y;
    const double norm = std::hypot(gx, gy);
    if (norm < 1.0e-3) {
      return 0.0;
    }
    return (x * gx + y * gy) / norm;
  }

  bool footprintCost(
    const autonomy::msg::MaskedHeightScan & scan,
    const double x,
    const double y,
    double & clearance,
    double & cost) const
  {
    if (scan.width == 0 || scan.height == 0 || scan.resolution <= 0.0F) {
      return false;
    }

    clearance = kInf;
    cost = 0.0;
    bool saw_valid = false;
    const double radius = robot_radius_;
    const std::uint32_t width = scan.width;
    const std::uint32_t height = scan.height;

    for (std::uint32_t row = 0; row < height; ++row) {
      const double cy = scan.y_min + (static_cast<double>(row) + 0.5) * scan.resolution;
      if (std::abs(cy - y) > radius + scan.resolution) {
        continue;
      }
      for (std::uint32_t col = 0; col < width; ++col) {
        const double cx = scan.x_min + (static_cast<double>(col) + 0.5) * scan.resolution;
        const double dist = std::hypot(cx - x, cy - y);
        if (dist > radius) {
          continue;
        }

        const std::size_t index = static_cast<std::size_t>(row) * width + col;
        if (index >= scan.data.size()) {
          continue;
        }
        const bool valid = index < scan.valid_mask.size() && scan.valid_mask[index] != 0U;
        if (!valid) {
          if (unknown_is_blocked_) {
            return false;
          }
          cost += 0.2;
          continue;
        }

        saw_valid = true;
        const double terrain_z = -static_cast<double>(scan.data[index]);
        if (terrain_z > obstacle_height_threshold_) {
          return false;
        }
        const double roughness = localRoughness(scan, row, col);
        if (roughness > step_height_threshold_) {
          return false;
        }
        cost += std::max(0.0, terrain_z) + 2.0 * roughness;
        clearance = std::min(clearance, std::max(0.0, obstacle_height_threshold_ - terrain_z));
      }
    }

    if (!saw_valid && unknown_is_blocked_) {
      return false;
    }
    if (clearance == kInf) {
      clearance = obstacle_height_threshold_;
    }
    return true;
  }

  double localRoughness(
    const autonomy::msg::MaskedHeightScan & scan,
    const std::uint32_t row,
    const std::uint32_t col) const
  {
    const std::size_t center_index = static_cast<std::size_t>(row) * scan.width + col;
    if (center_index >= scan.data.size()) {
      return 0.0;
    }
    const double center = -static_cast<double>(scan.data[center_index]);
    double roughness = 0.0;
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        const int rr = static_cast<int>(row) + dr;
        const int cc = static_cast<int>(col) + dc;
        if (rr < 0 || cc < 0 || rr >= static_cast<int>(scan.height) || cc >= static_cast<int>(scan.width)) {
          continue;
        }
        const std::size_t index = static_cast<std::size_t>(rr) * scan.width + static_cast<std::size_t>(cc);
        if (index >= scan.data.size() || index >= scan.valid_mask.size() || scan.valid_mask[index] == 0U) {
          continue;
        }
        roughness = std::max(roughness, std::abs(center + static_cast<double>(scan.data[index])));
      }
    }
    return roughness;
  }

  void publishCommand(const Candidate & candidate)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = candidate.vx;
    cmd.linear.y = candidate.vy;
    cmd.angular.z = candidate.wz;
    cmd_pub_->publish(cmd);
    if (dds_pub_) {
      dds_pub_->publish({
        static_cast<float>(cmd.linear.x),
        static_cast<float>(cmd.linear.y),
        static_cast<float>(cmd.angular.z)});
    }
  }

  void publishStop()
  {
    publishCommand(Candidate{});
  }

  static std::string shortText(const double value)
  {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.2f", value);
    return std::string(buffer);
  }

  bool enable_map_{false};
  std::string height_scan_topic_;
  std::string command_filter_topic_;
  std::string goal_topic_;
  std::string cmd_vel_topic_;
  double publish_rate_hz_{20.0};
  double max_vx_{0.35};
  double max_vy_{0.22};
  double max_wz_{0.7};
  double min_forward_vx_{0.05};
  double robot_radius_{0.32};
  double rollout_time_{1.2};
  double rollout_dt_{0.15};
  double obstacle_height_threshold_{0.18};
  double step_height_threshold_{0.10};
  bool unknown_is_blocked_{false};
  double stale_timeout_sec_{1.0};
  bool dds_enabled_{true};
  int dds_domain_id_{1};
  std::string dds_topic_{"lin_vel"};
  std::string dds_type_{"core_dds::LinearVelocity"};
  std::mutex mutex_;
  autonomy::msg::MaskedHeightScan::SharedPtr scan_;
  core::msg::CommandFilter filter_;
  geometry_msgs::msg::PoseStamped goal_;
  bool has_filter_{false};
  bool has_goal_{false};
  rclcpp::Time last_scan_time_{0, 0u, RCL_SYSTEM_TIME};
  std::unique_ptr<DdsLinearVelocityPublisher> dds_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<autonomy::msg::MaskedHeightScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<core::msg::CommandFilter>::SharedPtr filter_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
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
