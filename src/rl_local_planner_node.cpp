#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "autonomy/msg/autonomy_state.hpp"
#include "autonomy/msg/masked_height_scan.hpp"
#include "core/msg/command_filter.hpp"
#include "core/msg/command_user.hpp"

namespace autonomy
{
namespace
{

constexpr double kEpsilon = 1e-6;

double clamp(const double value, const double min_value, const double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double square(const double value)
{
  return value * value;
}

struct VelocityCommand
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct FootprintPoint
{
  double x{0.0};
  double y{0.0};
};

}  // namespace

class LocalPlannerNode final : public rclcpp::Node
{
public:
  LocalPlannerNode()
  : Node("rl_local_planner_node"), rng_(42)
  {
    declare_parameter<bool>("enabled", true);
    declare_parameter<std::string>("planner_backend", "dwb_lite");
    declare_parameter<std::string>("model_path", "");
    declare_parameter<std::string>("current_pose_topic", "/localization/current_pose");
    declare_parameter<std::string>("target_pose_topic", "/planning/target_pose");
    declare_parameter<std::string>("height_scan_topic", "/elevation_mapping_node/local_terrain_map");
    declare_parameter<std::string>("command_filter_topic", "/command_filter");
    declare_parameter<std::string>("command_user_topic", "/command_user");
    declare_parameter<std::string>("local_costmap_topic", "~/local_costmap");
    declare_parameter<std::string>("footprint_topic", "~/footprint");
    declare_parameter<std::string>("costmap_frame_id", "base_link");
    declare_parameter<std::vector<double>>(
      "footprint",
      std::vector<double>{0.35, 0.25, 0.35, -0.25, -0.35, -0.25, -0.35, 0.25});
    declare_parameter<std::string>("autonomy_status_topic", "/autonomy_manager/status");
    declare_parameter<double>("publish_rate_hz", 20.0);
    declare_parameter<double>("goal_tolerance_xy", 0.15);
    declare_parameter<double>("goal_tolerance_yaw", 0.2);
    declare_parameter<double>("max_linear_x", 0.45);
    declare_parameter<double>("max_linear_y", 0.30);
    declare_parameter<double>("max_angular_z", 0.9);
    declare_parameter<double>("max_linear_accel", 0.6);
    declare_parameter<double>("max_angular_accel", 1.2);
    declare_parameter<double>("rollout_dt", 0.1);
    declare_parameter<int>("rollout_steps", 10);
    declare_parameter<double>("traversability_obstacle_floor_z", -0.47957);
    declare_parameter<double>("traversability_obstacle_height", 0.25);
    declare_parameter<double>("traversability_unknown_cost", 0.45);
    declare_parameter<double>("traversability_obstacle_cost", 1000.0);
    declare_parameter<double>("inflation_radius", 0.45);
    declare_parameter<double>("inflation_cost_scaling_factor", 3.0);
    declare_parameter<double>("footprint_padding", 0.02);
    declare_parameter<int>("dwb_samples_x", 7);
    declare_parameter<int>("dwb_samples_y", 5);
    declare_parameter<int>("dwb_samples_wz", 7);
    declare_parameter<double>("weight_goal_distance", 4.0);
    declare_parameter<double>("weight_goal_heading", 1.0);
    declare_parameter<double>("weight_terrain", 8.0);
    declare_parameter<double>("weight_smoothness", 0.4);
    declare_parameter<double>("weight_speed", 0.2);
    declare_parameter<int>("mppi_samples", 64);
    declare_parameter<double>("mppi_noise_x", 0.18);
    declare_parameter<double>("mppi_noise_y", 0.12);
    declare_parameter<double>("mppi_noise_wz", 0.35);

    loadParameters();
    createIo();

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { tick(); });

    RCLCPP_INFO(
      get_logger(),
      "Local planner started: backend=%s command_user=%s",
      planner_backend_.c_str(),
      command_user_topic_.c_str());
  }

private:
  void loadParameters()
  {
    enabled_ = get_parameter("enabled").as_bool();
    planner_backend_ = get_parameter("planner_backend").as_string();
    model_path_ = get_parameter("model_path").as_string();
    command_user_topic_ = get_parameter("command_user_topic").as_string();
    local_costmap_topic_ = get_parameter("local_costmap_topic").as_string();
    footprint_topic_ = get_parameter("footprint_topic").as_string();
    costmap_frame_id_ = get_parameter("costmap_frame_id").as_string();
    footprint_ = parseFootprint(get_parameter("footprint").as_double_array());
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    goal_tolerance_xy_ = get_parameter("goal_tolerance_xy").as_double();
    goal_tolerance_yaw_ = get_parameter("goal_tolerance_yaw").as_double();
    max_linear_x_ = std::abs(get_parameter("max_linear_x").as_double());
    max_linear_y_ = std::abs(get_parameter("max_linear_y").as_double());
    max_angular_z_ = std::abs(get_parameter("max_angular_z").as_double());
    max_linear_accel_ = std::abs(get_parameter("max_linear_accel").as_double());
    max_angular_accel_ = std::abs(get_parameter("max_angular_accel").as_double());
    rollout_dt_ = std::max(0.02, get_parameter("rollout_dt").as_double());
    rollout_steps_ = std::max(1, static_cast<int>(get_parameter("rollout_steps").as_int()));
    traversability_obstacle_floor_z_ = get_parameter("traversability_obstacle_floor_z").as_double();
    traversability_obstacle_height_ = get_parameter("traversability_obstacle_height").as_double();
    traversability_unknown_cost_ = get_parameter("traversability_unknown_cost").as_double();
    traversability_obstacle_cost_ = get_parameter("traversability_obstacle_cost").as_double();
    inflation_radius_ = std::max(0.0, get_parameter("inflation_radius").as_double());
    inflation_cost_scaling_factor_ = std::max(0.0, get_parameter("inflation_cost_scaling_factor").as_double());
    footprint_padding_ = std::max(0.0, get_parameter("footprint_padding").as_double());
    dwb_samples_x_ = std::max(1, static_cast<int>(get_parameter("dwb_samples_x").as_int()));
    dwb_samples_y_ = std::max(1, static_cast<int>(get_parameter("dwb_samples_y").as_int()));
    dwb_samples_wz_ = std::max(1, static_cast<int>(get_parameter("dwb_samples_wz").as_int()));
    weight_goal_distance_ = get_parameter("weight_goal_distance").as_double();
    weight_goal_heading_ = get_parameter("weight_goal_heading").as_double();
    weight_terrain_ = get_parameter("weight_terrain").as_double();
    weight_smoothness_ = get_parameter("weight_smoothness").as_double();
    weight_speed_ = get_parameter("weight_speed").as_double();
    mppi_samples_ = std::max(1, static_cast<int>(get_parameter("mppi_samples").as_int()));
    mppi_noise_x_ = std::abs(get_parameter("mppi_noise_x").as_double());
    mppi_noise_y_ = std::abs(get_parameter("mppi_noise_y").as_double());
    mppi_noise_wz_ = std::abs(get_parameter("mppi_noise_wz").as_double());
  }

  void createIo()
  {
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
        publishLocalCostmap();
        publishFootprint();
      });
    command_filter_sub_ = create_subscription<core::msg::CommandFilter>(
      get_parameter("command_filter_topic").as_string(),
      10,
      [this](core::msg::CommandFilter::SharedPtr msg) {
        command_filter_ = *msg;
        has_command_filter_ = true;
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
    command_user_pub_ = create_publisher<core::msg::CommandUser>(command_user_topic_, 10);
    local_costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(local_costmap_topic_, 1);
    footprint_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(footprint_topic_, 1);
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/rl_local_planner_node",
      10);
  }

  void tick()
  {
    std_msgs::msg::String heartbeat;
    if (!enabled_) {
      heartbeat.data = "disabled";
      publishHeartbeat(heartbeat);
      return;
    }
    if (!has_autonomy_state_ || !autonomy_allows_command_) {
      heartbeat.data = "standby:autonomy_blocked";
      publishHeartbeat(heartbeat);
      return;
    }
    if (!has_current_pose_ || !has_target_pose_ || !latest_scan_) {
      heartbeat.data = "waiting_for_inputs";
      publishHeartbeat(heartbeat);
      return;
    }

    const auto target = targetInRobotFrame();
    VelocityCommand command;
    if (reachedGoal(target)) {
      heartbeat.data = "goal_reached";
    } else if (planner_backend_ == "rl") {
      heartbeat.data = model_path_.empty() ? "rl:skeleton_no_model" : "rl:skeleton";
      command = computeRlSkeletonCommand();
    } else if (planner_backend_ == "mppi_lite") {
      heartbeat.data = "mppi_lite";
      command = computeMppiLiteCommand(target);
    } else {
      heartbeat.data = planner_backend_ == "dwb_lite" ? "dwb_lite" : "dwb_lite:fallback_backend";
      command = computeDwbLiteCommand(target);
    }

    command = applyAccelerationLimit(applyCommandFilter(command));
    previous_command_ = command;
    command_user_pub_->publish(toCommandUser(command));
    publishHeartbeat(heartbeat);
  }

  void publishHeartbeat(const std_msgs::msg::String & heartbeat)
  {
    heartbeat_pub_->publish(heartbeat);
  }

  [[nodiscard]] VelocityCommand computeRlSkeletonCommand() const
  {
    return {};
  }

  [[nodiscard]] VelocityCommand computeDwbLiteCommand(const geometry_msgs::msg::Pose2D & target) const
  {
    VelocityCommand best;
    double best_cost = std::numeric_limits<double>::infinity();

    for (int ix = 0; ix < dwb_samples_x_; ++ix) {
      const double vx = sampleLinear(ix, dwb_samples_x_, -max_linear_x_, max_linear_x_);
      for (int iy = 0; iy < dwb_samples_y_; ++iy) {
        const double vy = sampleLinear(iy, dwb_samples_y_, -max_linear_y_, max_linear_y_);
        for (int iw = 0; iw < dwb_samples_wz_; ++iw) {
          const double wz = sampleLinear(iw, dwb_samples_wz_, -max_angular_z_, max_angular_z_);
          const VelocityCommand candidate{vx, vy, wz};
          const double cost = scoreConstantVelocity(candidate, target);
          if (cost < best_cost) {
            best_cost = cost;
            best = candidate;
          }
        }
      }
    }

    return best;
  }

  [[nodiscard]] VelocityCommand computeMppiLiteCommand(const geometry_msgs::msg::Pose2D & target)
  {
    VelocityCommand best = computeGoalSeekingSeed(target);
    double best_cost = scoreNoisySequence(best, target, false);

    std::normal_distribution<double> noise_x(0.0, mppi_noise_x_);
    std::normal_distribution<double> noise_y(0.0, mppi_noise_y_);
    std::normal_distribution<double> noise_wz(0.0, mppi_noise_wz_);
    for (int sample = 0; sample < mppi_samples_; ++sample) {
      VelocityCommand seed = previous_command_;
      seed.vx = clamp(seed.vx + noise_x(rng_), -max_linear_x_, max_linear_x_);
      seed.vy = clamp(seed.vy + noise_y(rng_), -max_linear_y_, max_linear_y_);
      seed.wz = clamp(seed.wz + noise_wz(rng_), -max_angular_z_, max_angular_z_);
      const double cost = scoreNoisySequence(seed, target, true);
      if (cost < best_cost) {
        best_cost = cost;
        best = seed;
      }
    }

    return best;
  }

  [[nodiscard]] double scoreNoisySequence(
    const VelocityCommand & seed,
    const geometry_msgs::msg::Pose2D & target,
    const bool noisy)
  {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double terrain_cost = 0.0;
    double smoothness_cost = 0.0;
    VelocityCommand command = seed;
    std::normal_distribution<double> noise_x(0.0, mppi_noise_x_ * 0.35);
    std::normal_distribution<double> noise_y(0.0, mppi_noise_y_ * 0.35);
    std::normal_distribution<double> noise_wz(0.0, mppi_noise_wz_ * 0.35);

    for (int step = 0; step < rollout_steps_; ++step) {
      if (noisy && step > 0) {
        command.vx = clamp(command.vx + noise_x(rng_), -max_linear_x_, max_linear_x_);
        command.vy = clamp(command.vy + noise_y(rng_), -max_linear_y_, max_linear_y_);
        command.wz = clamp(command.wz + noise_wz(rng_), -max_angular_z_, max_angular_z_);
      }
      rolloutStep(command, x, y, theta);
      terrain_cost += traversabilityCostAt(x, y);
      smoothness_cost += commandDeltaCost(command, previous_command_);
    }

    const double distance = std::hypot(target.x - x, target.y - y);
    const double heading = std::abs(normalizeAngle(target.theta - theta));
    const double speed_reward = std::hypot(seed.vx, seed.vy);
    return weight_goal_distance_ * distance +
           weight_goal_heading_ * heading +
           weight_terrain_ * terrain_cost / rollout_steps_ +
           weight_smoothness_ * smoothness_cost / rollout_steps_ -
           weight_speed_ * speed_reward;
  }

  [[nodiscard]] double scoreConstantVelocity(
    const VelocityCommand & command,
    const geometry_msgs::msg::Pose2D & target) const
  {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double terrain_cost = 0.0;
    for (int step = 0; step < rollout_steps_; ++step) {
      rolloutStep(command, x, y, theta);
      terrain_cost += traversabilityCostAt(x, y);
    }

    const double distance = std::hypot(target.x - x, target.y - y);
    const double heading = std::abs(normalizeAngle(target.theta - theta));
    const double smoothness = commandDeltaCost(command, previous_command_);
    const double speed_reward = std::hypot(command.vx, command.vy);
    return weight_goal_distance_ * distance +
           weight_goal_heading_ * heading +
           weight_terrain_ * terrain_cost / rollout_steps_ +
           weight_smoothness_ * smoothness -
           weight_speed_ * speed_reward;
  }

  void rolloutStep(const VelocityCommand & command, double & x, double & y, double & theta) const
  {
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);
    x += (command.vx * cos_theta - command.vy * sin_theta) * rollout_dt_;
    y += (command.vx * sin_theta + command.vy * cos_theta) * rollout_dt_;
    theta = normalizeAngle(theta + command.wz * rollout_dt_);
  }

  [[nodiscard]] double traversabilityCostAt(const double x, const double y) const
  {
    const auto & scan = *latest_scan_;
    if (scan.width == 0 || scan.height == 0 || scan.resolution <= 0.0F || scan.data.empty()) {
      return traversability_unknown_cost_;
    }
    if (x < scan.x_min || x > scan.x_max || y < scan.y_min || y > scan.y_max) {
      return traversability_unknown_cost_;
    }

    const auto col = static_cast<int>((x - scan.x_min) / scan.resolution);
    const auto row = static_cast<int>((y - scan.y_min) / scan.resolution);
    if (col < 0 || row < 0 || col >= static_cast<int>(scan.width) || row >= static_cast<int>(scan.height)) {
      return traversability_unknown_cost_;
    }

    const auto index = static_cast<std::size_t>(row) * scan.width + static_cast<std::size_t>(col);
    if (index >= scan.data.size()) {
      return traversability_unknown_cost_;
    }
    if (index < scan.valid_mask.size() && scan.valid_mask[index] == 0U) {
      return traversability_unknown_cost_;
    }

    const double height_scan_value = scan.data[index];
    if (!std::isfinite(height_scan_value)) {
      return traversability_unknown_cost_;
    }
    const double z = -height_scan_value - static_cast<double>(scan.height_scan_offset);
    const double obstacle_height = z - traversability_obstacle_floor_z_;
    if (obstacle_height >= traversability_obstacle_height_) {
      return traversability_obstacle_cost_;
    }
    return clamp(obstacle_height / std::max(kEpsilon, traversability_obstacle_height_), 0.0, 1.0);
  }

  void publishLocalCostmap() const
  {
    if (!latest_scan_ || !local_costmap_pub_) {
      return;
    }

    const auto & scan = *latest_scan_;
    if (scan.width == 0 || scan.height == 0 || scan.resolution <= 0.0F) {
      return;
    }

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = costmap_frame_id_;
    grid.info.resolution = scan.resolution;
    grid.info.width = scan.width;
    grid.info.height = scan.height;
    grid.info.origin.position.x = scan.x_min;
    grid.info.origin.position.y = scan.y_min;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(static_cast<std::size_t>(scan.width) * scan.height, 0);

    std::vector<std::pair<int, int>> obstacle_cells;
    for (int row = 0; row < static_cast<int>(scan.height); ++row) {
      for (int col = 0; col < static_cast<int>(scan.width); ++col) {
        const auto index = static_cast<std::size_t>(row) * scan.width + static_cast<std::size_t>(col);
        const double cost = traversabilityCostAtIndex(scan, index);
        if (cost < 0.0) {
          grid.data[index] = -1;
        } else if (cost >= traversability_obstacle_cost_) {
          grid.data[index] = 100;
          obstacle_cells.emplace_back(row, col);
        } else {
          grid.data[index] = static_cast<int8_t>(std::lround(clamp(cost, 0.0, 1.0) * 99.0));
        }
      }
    }

    applyInflation(obstacle_cells, scan.resolution, grid);
    local_costmap_pub_->publish(grid);
  }

  void applyInflation(
    const std::vector<std::pair<int, int>> & obstacle_cells,
    const double resolution,
    nav_msgs::msg::OccupancyGrid & grid) const
  {
    if (inflation_radius_ <= 0.0 || resolution <= 0.0 || obstacle_cells.empty()) {
      return;
    }

    const int radius_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution));
    const double inscribed_radius = footprintInscribedRadius() + footprint_padding_;
    for (const auto & [obstacle_row, obstacle_col] : obstacle_cells) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          const int row = obstacle_row + dy;
          const int col = obstacle_col + dx;
          if (row < 0 || col < 0 ||
            row >= static_cast<int>(grid.info.height) || col >= static_cast<int>(grid.info.width))
          {
            continue;
          }
          const double distance = std::hypot(dx * resolution, dy * resolution);
          if (distance > inflation_radius_) {
            continue;
          }

          const auto index = static_cast<std::size_t>(row) * grid.info.width + static_cast<std::size_t>(col);
          if (grid.data[index] == 100) {
            continue;
          }
          const int8_t inflated_cost = inflationCost(distance, inscribed_radius);
          grid.data[index] = std::max(grid.data[index], inflated_cost);
        }
      }
    }
  }

  [[nodiscard]] int8_t inflationCost(const double distance, const double inscribed_radius) const
  {
    if (distance <= inscribed_radius) {
      return 99;
    }
    const double decay_distance = distance - inscribed_radius;
    const double cost = 99.0 * std::exp(-inflation_cost_scaling_factor_ * decay_distance);
    return static_cast<int8_t>(clamp(std::lround(cost), 1.0, 99.0));
  }

  [[nodiscard]] double traversabilityCostAtIndex(
    const autonomy::msg::MaskedHeightScan & scan,
    const std::size_t index) const
  {
    if (index >= scan.data.size()) {
      return -1.0;
    }
    if (index < scan.valid_mask.size() && scan.valid_mask[index] == 0U) {
      return -1.0;
    }

    const double height_scan_value = scan.data[index];
    if (!std::isfinite(height_scan_value)) {
      return -1.0;
    }
    const double z = -height_scan_value - static_cast<double>(scan.height_scan_offset);
    const double obstacle_height = z - traversability_obstacle_floor_z_;
    if (obstacle_height >= traversability_obstacle_height_) {
      return traversability_obstacle_cost_;
    }
    return clamp(obstacle_height / std::max(kEpsilon, traversability_obstacle_height_), 0.0, 1.0);
  }

  void publishFootprint() const
  {
    if (!footprint_pub_) {
      return;
    }
    geometry_msgs::msg::PolygonStamped footprint_msg;
    footprint_msg.header.stamp = now();
    footprint_msg.header.frame_id = costmap_frame_id_;
    for (const auto & point : footprint_) {
      geometry_msgs::msg::Point32 msg_point;
      msg_point.x = static_cast<float>(point.x);
      msg_point.y = static_cast<float>(point.y);
      msg_point.z = 0.0F;
      footprint_msg.polygon.points.push_back(msg_point);
    }
    footprint_pub_->publish(footprint_msg);
  }

  [[nodiscard]] std::vector<FootprintPoint> parseFootprint(
    const std::vector<double> & values) const
  {
    std::vector<FootprintPoint> footprint;
    if (values.size() < 6 || values.size() % 2 != 0) {
      RCLCPP_WARN(
        get_logger(),
        "Invalid footprint parameter. Expected flat [x0, y0, ...], using default rectangle.");
      return {{0.35, 0.25}, {0.35, -0.25}, {-0.35, -0.25}, {-0.35, 0.25}};
    }
    footprint.reserve(values.size() / 2);
    for (std::size_t i = 0; i + 1 < values.size(); i += 2) {
      footprint.push_back({values[i], values[i + 1]});
    }
    return footprint;
  }

  [[nodiscard]] double footprintInscribedRadius() const
  {
    if (footprint_.empty()) {
      return 0.0;
    }
    double radius = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < footprint_.size(); ++i) {
      const auto & a = footprint_[i];
      const auto & b = footprint_[(i + 1) % footprint_.size()];
      const double segment_length = std::hypot(b.x - a.x, b.y - a.y);
      if (segment_length <= kEpsilon) {
        continue;
      }
      const double distance = std::abs(a.x * b.y - b.x * a.y) / segment_length;
      radius = std::min(radius, distance);
    }
    if (!std::isfinite(radius)) {
      return 0.0;
    }
    return radius;
  }

  [[nodiscard]] VelocityCommand computeGoalSeekingSeed(const geometry_msgs::msg::Pose2D & target) const
  {
    VelocityCommand command;
    command.vx = clamp(target.x, -max_linear_x_, max_linear_x_);
    command.vy = clamp(target.y, -max_linear_y_, max_linear_y_);
    command.wz = clamp(normalizeAngle(target.theta), -max_angular_z_, max_angular_z_);
    return command;
  }

  [[nodiscard]] VelocityCommand applyCommandFilter(VelocityCommand command) const
  {
    if (!has_command_filter_) {
      return command;
    }
    if (command.vx > 0.0 && !command_filter_.allow_linear_vel_forward_x) {
      command.vx = 0.0;
    } else if (command.vx < 0.0 && !command_filter_.allow_linear_vel_backward_x) {
      command.vx = 0.0;
    }
    if (command.vy > 0.0 && !command_filter_.allow_linear_vel_forward_y) {
      command.vy = 0.0;
    } else if (command.vy < 0.0 && !command_filter_.allow_linear_vel_backward_y) {
      command.vy = 0.0;
    }
    return command;
  }

  [[nodiscard]] VelocityCommand applyAccelerationLimit(VelocityCommand command) const
  {
    const double max_linear_delta = max_linear_accel_ / std::max(1.0, publish_rate_hz_);
    const double max_angular_delta = max_angular_accel_ / std::max(1.0, publish_rate_hz_);
    command.vx = previous_command_.vx +
      clamp(command.vx - previous_command_.vx, -max_linear_delta, max_linear_delta);
    command.vy = previous_command_.vy +
      clamp(command.vy - previous_command_.vy, -max_linear_delta, max_linear_delta);
    command.wz = previous_command_.wz +
      clamp(command.wz - previous_command_.wz, -max_angular_delta, max_angular_delta);
    return command;
  }

  [[nodiscard]] core::msg::CommandUser toCommandUser(const VelocityCommand & velocity) const
  {
    core::msg::CommandUser command;
    command.odom.header.stamp = now();
    command.odom.twist.twist.linear.x = velocity.vx;
    command.odom.twist.twist.linear.y = velocity.vy;
    command.odom.twist.twist.angular.z = velocity.wz;
    command.event.estop = false;
    command.event.wake = false;
    command.event.sleep = false;
    command.event.rough_drive_toggle = false;
    return command;
  }

  [[nodiscard]] geometry_msgs::msg::Pose2D targetInRobotFrame() const
  {
    const double dx = target_pose_.x - current_pose_.x;
    const double dy = target_pose_.y - current_pose_.y;
    const double cos_theta = std::cos(current_pose_.theta);
    const double sin_theta = std::sin(current_pose_.theta);
    geometry_msgs::msg::Pose2D target;
    target.x = cos_theta * dx + sin_theta * dy;
    target.y = -sin_theta * dx + cos_theta * dy;
    target.theta = normalizeAngle(target_pose_.theta - current_pose_.theta);
    return target;
  }

  [[nodiscard]] bool reachedGoal(const geometry_msgs::msg::Pose2D & target) const
  {
    return std::hypot(target.x, target.y) <= goal_tolerance_xy_ &&
           std::abs(normalizeAngle(target.theta)) <= goal_tolerance_yaw_;
  }

  [[nodiscard]] double sampleLinear(
    const int index,
    const int count,
    const double min_value,
    const double max_value) const
  {
    if (count <= 1) {
      return 0.5 * (min_value + max_value);
    }
    const double ratio = static_cast<double>(index) / static_cast<double>(count - 1);
    return min_value + ratio * (max_value - min_value);
  }

  [[nodiscard]] double commandDeltaCost(
    const VelocityCommand & lhs,
    const VelocityCommand & rhs) const
  {
    return square(lhs.vx - rhs.vx) + square(lhs.vy - rhs.vy) + square(lhs.wz - rhs.wz);
  }

  bool enabled_{true};
  bool has_autonomy_state_{false};
  bool autonomy_allows_command_{false};
  bool has_current_pose_{false};
  bool has_target_pose_{false};
  bool has_command_filter_{false};
  std::string planner_backend_{"dwb_lite"};
  std::string model_path_;
  std::string command_user_topic_;
  std::string local_costmap_topic_;
  std::string footprint_topic_;
  std::string costmap_frame_id_{"base_link"};
  double publish_rate_hz_{20.0};
  double goal_tolerance_xy_{0.15};
  double goal_tolerance_yaw_{0.2};
  double max_linear_x_{0.45};
  double max_linear_y_{0.30};
  double max_angular_z_{0.9};
  double max_linear_accel_{0.6};
  double max_angular_accel_{1.2};
  double rollout_dt_{0.1};
  int rollout_steps_{10};
  double traversability_obstacle_floor_z_{-0.47957};
  double traversability_obstacle_height_{0.25};
  double traversability_unknown_cost_{0.45};
  double traversability_obstacle_cost_{1000.0};
  double inflation_radius_{0.45};
  double inflation_cost_scaling_factor_{3.0};
  double footprint_padding_{0.02};
  int dwb_samples_x_{7};
  int dwb_samples_y_{5};
  int dwb_samples_wz_{7};
  double weight_goal_distance_{4.0};
  double weight_goal_heading_{1.0};
  double weight_terrain_{8.0};
  double weight_smoothness_{0.4};
  double weight_speed_{0.2};
  int mppi_samples_{64};
  double mppi_noise_x_{0.18};
  double mppi_noise_y_{0.12};
  double mppi_noise_wz_{0.35};

  geometry_msgs::msg::Pose2D current_pose_;
  geometry_msgs::msg::Pose2D target_pose_;
  autonomy::msg::MaskedHeightScan::SharedPtr latest_scan_;
  core::msg::CommandFilter command_filter_;
  std::vector<FootprintPoint> footprint_;
  VelocityCommand previous_command_;
  std::mt19937 rng_;

  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<autonomy::msg::MaskedHeightScan>::SharedPtr height_scan_sub_;
  rclcpp::Subscription<core::msg::CommandFilter>::SharedPtr command_filter_sub_;
  rclcpp::Subscription<autonomy::msg::AutonomyState>::SharedPtr autonomy_sub_;
  rclcpp::Publisher<core::msg::CommandUser>::SharedPtr command_user_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::LocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
