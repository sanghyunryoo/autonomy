#include "autonomy/algorithm/planner/mppi.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <utility>
#include <vector>

namespace autonomy {
namespace {

constexpr double kCollisionCost = 1.0e9;
constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle <= -kPi) angle += 2.0 * kPi;
  return angle;
}

bool collisionFree(const OccupancyGrid & map, const double x, const double y,
  const double radius, const std::int8_t threshold, const bool unknown_is_obstacle)
{
  const int center_column = static_cast<int>(std::floor((x - map.origin_x) / map.resolution));
  const int center_row = static_cast<int>(std::floor((y - map.origin_y) / map.resolution));
  const int radius_cells = static_cast<int>(std::ceil(radius / map.resolution));
  for (int row_offset = -radius_cells; row_offset <= radius_cells; ++row_offset) {
    for (int column_offset = -radius_cells; column_offset <= radius_cells; ++column_offset) {
      if (std::hypot(column_offset, row_offset) * map.resolution > radius) continue;
      const int column = center_column + column_offset;
      const int row = center_row + row_offset;
      if (column < 0 || row < 0 || column >= static_cast<int>(map.width) || row >= static_cast<int>(map.height)) {
        return false;
      }
      const std::int8_t value = map.cells[static_cast<std::size_t>(row) * map.width + column];
      if (value < 0 ? unknown_is_obstacle : value >= threshold) return false;
    }
  }
  return true;
}

double nearestPathCost(const Pose2d & pose, const std::vector<Pose2d> & path)
{
  double best = std::numeric_limits<double>::infinity();
  for (const Pose2d & waypoint : path) {
    best = std::min(best, std::hypot(pose.x - waypoint.x, pose.y - waypoint.y));
  }
  return best;
}

struct Sample {
  double linear{0.0};
  double angular{0.0};
  double cost{kCollisionCost};
};

bool rolloutCollisionFree(
  const OccupancyGrid & map,
  const Pose2d & current_pose,
  const double linear_velocity,
  const MppiConfig & config)
{
  if (!collisionFree(
      map, current_pose.x, current_pose.y, config.vehicle_radius,
      config.occupied_threshold, config.unknown_is_obstacle)) {
    return false;
  }

  Pose2d simulated = current_pose;
  for (std::uint32_t step = 0U; step < config.horizon_steps; ++step) {
    simulated.x += linear_velocity * config.dt * std::cos(simulated.yaw);
    simulated.y += linear_velocity * config.dt * std::sin(simulated.yaw);
    if (!collisionFree(
        map, simulated.x, simulated.y, config.vehicle_radius,
        config.occupied_threshold, config.unknown_is_obstacle)) {
      return false;
    }
  }
  return true;
}

}  // namespace

MppiPlanner::MppiPlanner(MppiConfig config)
: config_(std::move(config))
{
}

LocalPlanResult MppiPlanner::solve(
  const OccupancyGrid & map,
  const Pose2d & current_pose,
  const std::vector<Pose2d> & global_path)
{
  LocalPlanResult result;
  if (!map.valid() || global_path.empty() || config_.sample_count == 0U || config_.horizon_steps == 0U ||
    config_.dt <= 0.0 || config_.temperature <= 0.0) {
    result.failure = PlannerFailure::EmptyPath;
    result.message = "MPPI requires a valid map, non-empty global path, and positive configuration";
    return result;
  }
  const Pose2d & goal = global_path.back();
  if (std::hypot(goal.x - current_pose.x, goal.y - current_pose.y) <= 0.15) {
    result.message = "MPPI goal reached; zero command";
    return result;
  }
  std::mt19937 random_engine(sequence_++);
  std::normal_distribution<double> linear_noise(0.0, config_.linear_noise);
  std::normal_distribution<double> angular_noise(0.0, config_.angular_noise);
  const double desired_heading = std::atan2(goal.y - current_pose.y, goal.x - current_pose.x);
  const double nominal_linear = std::clamp(
    std::hypot(goal.x - current_pose.x, goal.y - current_pose.y), 0.0, config_.max_linear_velocity);
  const double nominal_angular = std::clamp(normalizeAngle(desired_heading - current_pose.yaw),
    -config_.max_angular_velocity, config_.max_angular_velocity);

  std::vector<Sample> samples;
  samples.reserve(config_.sample_count);
  double best_cost = std::numeric_limits<double>::infinity();
  for (std::uint32_t index = 0; index < config_.sample_count; ++index) {
    Sample sample;
    sample.linear = std::clamp(nominal_linear + linear_noise(random_engine),
      -0.20, config_.max_linear_velocity);
    sample.angular = std::clamp(nominal_angular + angular_noise(random_engine),
      -config_.max_angular_velocity, config_.max_angular_velocity);
    Pose2d simulated = current_pose;
    sample.cost = 0.0;
    for (std::uint32_t step = 0; step < config_.horizon_steps; ++step) {
      simulated.x += sample.linear * config_.dt * std::cos(simulated.yaw);
      simulated.y += sample.linear * config_.dt * std::sin(simulated.yaw);
      simulated.yaw = normalizeAngle(simulated.yaw + sample.angular * config_.dt);
      if (!collisionFree(map, simulated.x, simulated.y, config_.vehicle_radius,
          config_.occupied_threshold, config_.unknown_is_obstacle)) {
        sample.cost = kCollisionCost;
        break;
      }
      const double path_cost = nearestPathCost(simulated, global_path);
      const double goal_cost = std::hypot(simulated.x - goal.x, simulated.y - goal.y);
      sample.cost += 3.0 * path_cost * path_cost + 0.4 * goal_cost +
        0.05 * sample.linear * sample.linear + 0.03 * sample.angular * sample.angular;
    }
    best_cost = std::min(best_cost, sample.cost);
    samples.push_back(sample);
  }
  if (!std::isfinite(best_cost) || best_cost >= kCollisionCost) {
    result.failure = PlannerFailure::NoFeasibleTrajectory;
    result.message = "MPPI found no collision-free trajectory";
    return result;
  }
  double weighted_linear = 0.0;
  double weighted_angular = 0.0;
  double weight_sum = 0.0;
  for (const Sample & sample : samples) {
    if (sample.cost >= kCollisionCost) continue;
    const double weight = std::exp(-(sample.cost - best_cost) / config_.temperature);
    weighted_linear += weight * sample.linear;
    weighted_angular += weight * sample.angular;
    weight_sum += weight;
  }
  if (weight_sum <= std::numeric_limits<double>::epsilon()) {
    result.failure = PlannerFailure::NoFeasibleTrajectory;
    result.message = "MPPI importance weights collapsed";
    return result;
  }
  result.linear_x = weighted_linear / weight_sum;
  result.angular_z = weighted_angular / weight_sum;
  result.message = "MPPI command ready";
  return result;
}

DirectionalMotionFilter MppiPlanner::evaluateDirectionalFilter(
  const OccupancyGrid & map,
  const Pose2d & current_pose) const
{
  DirectionalMotionFilter result;
  // Lateral robot motion is explicitly not terrain-filtered at this stage.
  result.allow_linear_vel_forward_y = true;
  result.allow_linear_vel_backward_y = true;

  if (!map.valid() || current_pose.frame_id != map.frame_id || config_.horizon_steps == 0U ||
    config_.dt <= 0.0 || config_.max_linear_velocity <= 0.0 || config_.vehicle_radius <= 0.0) {
    return result;
  }

  // Evaluate straight, maximum-speed rollouts. Permitting a direction only
  // when this full horizon is clear is intentionally more conservative than
  // the goal-seeking MPPI sample selection.
  result.allow_linear_vel_forward_x = rolloutCollisionFree(
    map, current_pose, config_.max_linear_velocity, config_);
  const double reverse_velocity = -std::min(0.20, config_.max_linear_velocity);
  result.allow_linear_vel_backward_x = rolloutCollisionFree(
    map, current_pose, reverse_velocity, config_);
  return result;
}

}  // namespace autonomy
