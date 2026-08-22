#include "autonomy/algorithm/planner/hybrid_astar.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autonomy {
namespace {

constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle <= -kPi) angle += 2.0 * kPi;
  return angle;
}

double distance(const Pose2d & left, const Pose2d & right)
{
  return std::hypot(left.x - right.x, left.y - right.y);
}

bool worldToCell(const OccupancyGrid & map, const double x, const double y, int & column, int & row)
{
  column = static_cast<int>(std::floor((x - map.origin_x) / map.resolution));
  row = static_cast<int>(std::floor((y - map.origin_y) / map.resolution));
  return column >= 0 && row >= 0 && column < static_cast<int>(map.width) && row < static_cast<int>(map.height);
}

bool occupied(
  const OccupancyGrid & map,
  const int column,
  const int row,
  const std::int8_t threshold,
  const bool unknown_is_obstacle)
{
  if (column < 0 || row < 0 || column >= static_cast<int>(map.width) || row >= static_cast<int>(map.height)) {
    return true;
  }
  const std::int8_t value = map.cells[static_cast<std::size_t>(row) * map.width + column];
  return value < 0 ? unknown_is_obstacle : value >= threshold;
}

bool collisionFree(
  const OccupancyGrid & map,
  const double x,
  const double y,
  const double radius,
  const std::int8_t threshold,
  const bool unknown_is_obstacle)
{
  int center_column = 0;
  int center_row = 0;
  if (!worldToCell(map, x, y, center_column, center_row)) {
    return false;
  }
  const int radius_cells = static_cast<int>(std::ceil(radius / map.resolution));
  for (int row_offset = -radius_cells; row_offset <= radius_cells; ++row_offset) {
    for (int column_offset = -radius_cells; column_offset <= radius_cells; ++column_offset) {
      if (std::hypot(column_offset, row_offset) * map.resolution > radius) {
        continue;
      }
      if (occupied(map, center_column + column_offset, center_row + row_offset, threshold, unknown_is_obstacle)) {
        return false;
      }
    }
  }
  return true;
}

std::uint32_t headingBin(const double yaw, const std::uint32_t bins)
{
  const double wrapped = normalizeAngle(yaw) + kPi;
  const double width = 2.0 * kPi / bins;
  const auto result = static_cast<std::int64_t>(std::floor(wrapped / width));
  return static_cast<std::uint32_t>((result % static_cast<std::int64_t>(bins) + bins) % bins);
}

std::uint64_t key(const int column, const int row, const std::uint32_t heading)
{
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(column)) << 40U) |
    (static_cast<std::uint64_t>(static_cast<std::uint32_t>(row)) << 16U) | heading;
}

struct SearchNode {
  Pose2d pose{};
  int column{0};
  int row{0};
  std::uint32_t heading{0};
  double cost{0.0};
  int parent{-1};
  int direction{1};
};

struct QueueEntry {
  double priority{0.0};
  int node_index{-1};
  bool operator<(const QueueEntry & other) const { return priority > other.priority; }
};

}  // namespace

const char * plannerFailureName(const PlannerFailure failure)
{
  switch (failure) {
    case PlannerFailure::None: return "none";
    case PlannerFailure::InvalidMap: return "invalid_map";
    case PlannerFailure::FrameMismatch: return "frame_mismatch";
    case PlannerFailure::StartOutsideMap: return "start_outside_map";
    case PlannerFailure::GoalOutsideMap: return "goal_outside_map";
    case PlannerFailure::StartOccupied: return "start_occupied";
    case PlannerFailure::GoalOccupied: return "goal_occupied";
    case PlannerFailure::SearchExhausted: return "search_exhausted";
    case PlannerFailure::SearchLimit: return "search_limit";
    case PlannerFailure::EmptyPath: return "empty_path";
    case PlannerFailure::NoFeasibleTrajectory: return "no_feasible_trajectory";
    default: return "unknown";
  }
}

HybridAStarPlanner::HybridAStarPlanner(HybridAStarConfig config)
: config_(std::move(config))
{
}

GlobalPlanResult HybridAStarPlanner::plan(
  const OccupancyGrid & map,
  const Pose2d & start,
  const Pose2d & goal) const
{
  GlobalPlanResult result;
  if (!map.valid() || config_.heading_bins == 0U || config_.step_size <= 0.0 || config_.wheelbase <= 0.0) {
    result.failure = PlannerFailure::InvalidMap;
    result.message = "Hybrid A* received an invalid map or configuration";
    return result;
  }
  if (start.frame_id != map.frame_id || goal.frame_id != map.frame_id) {
    result.failure = PlannerFailure::FrameMismatch;
    result.message = "map, robot pose, and goal must use one frame";
    return result;
  }
  int start_column = 0;
  int start_row = 0;
  if (!worldToCell(map, start.x, start.y, start_column, start_row)) {
    result.failure = PlannerFailure::StartOutsideMap;
    result.message = "robot pose is outside the planning map";
    return result;
  }
  int goal_column = 0;
  int goal_row = 0;
  if (!worldToCell(map, goal.x, goal.y, goal_column, goal_row)) {
    result.failure = PlannerFailure::GoalOutsideMap;
    result.message = "goal is outside the planning map";
    return result;
  }
  if (!collisionFree(map, start.x, start.y, config_.vehicle_radius, config_.occupied_threshold,
      config_.unknown_is_obstacle)) {
    result.failure = PlannerFailure::StartOccupied;
    result.message = "robot footprint intersects an occupied cell";
    return result;
  }
  if (!collisionFree(map, goal.x, goal.y, config_.vehicle_radius, config_.occupied_threshold,
      config_.unknown_is_obstacle)) {
    result.failure = PlannerFailure::GoalOccupied;
    result.message = "goal footprint intersects an occupied cell";
    return result;
  }

  std::vector<SearchNode> nodes;
  nodes.push_back(SearchNode{start, start_column, start_row, headingBin(start.yaw, config_.heading_bins), 0.0, -1, 1});
  std::priority_queue<QueueEntry> open;
  open.push({distance(start, goal), 0});
  std::unordered_map<std::uint64_t, double> best_cost;
  best_cost.emplace(key(start_column, start_row, nodes.front().heading), 0.0);

  const double steering[] = {-config_.max_steering_radians, 0.0, config_.max_steering_radians};
  const int directions[] = {1, -1};
  std::uint32_t expansions = 0U;
  int terminal = -1;
  while (!open.empty()) {
    const QueueEntry entry = open.top();
    open.pop();
    const SearchNode current = nodes[entry.node_index];
    if (++expansions > config_.max_expansions) {
      result.failure = PlannerFailure::SearchLimit;
      result.message = "Hybrid A* exceeded max_expansions";
      return result;
    }
    if (distance(current.pose, goal) <= config_.goal_tolerance) {
      terminal = entry.node_index;
      break;
    }
    for (const int direction : directions) {
      if (direction < 0 && !config_.allow_reverse) {
        continue;
      }
      for (const double steer : steering) {
        const double signed_step = direction * config_.step_size;
        Pose2d next = current.pose;
        next.x += signed_step * std::cos(current.pose.yaw);
        next.y += signed_step * std::sin(current.pose.yaw);
        next.yaw = normalizeAngle(current.pose.yaw + signed_step * std::tan(steer) / config_.wheelbase);
        next.frame_id = map.frame_id;
        int column = 0;
        int row = 0;
        if (!worldToCell(map, next.x, next.y, column, row) ||
          !collisionFree(map, next.x, next.y, config_.vehicle_radius, config_.occupied_threshold,
          config_.unknown_is_obstacle)) {
          continue;
        }
        const std::uint32_t heading = headingBin(next.yaw, config_.heading_bins);
        const double direction_penalty = direction < 0 ? 1.8 : 1.0;
        const double switch_penalty = direction != current.direction ? 0.25 : 0.0;
        const double steering_penalty = std::abs(steer) * 0.12;
        const double next_cost = current.cost + config_.step_size * direction_penalty + switch_penalty + steering_penalty;
        const std::uint64_t next_key = key(column, row, heading);
        const auto previous = best_cost.find(next_key);
        if (previous != best_cost.end() && previous->second <= next_cost) {
          continue;
        }
        best_cost[next_key] = next_cost;
        nodes.push_back(SearchNode{next, column, row, heading, next_cost, entry.node_index, direction});
        const double heading_cost = std::abs(normalizeAngle(goal.yaw - next.yaw)) * 0.15;
        open.push({next_cost + distance(next, goal) + heading_cost, static_cast<int>(nodes.size() - 1U)});
      }
    }
  }
  if (terminal < 0) {
    result.failure = PlannerFailure::SearchExhausted;
    result.message = "Hybrid A* could not reach the goal";
    return result;
  }
  for (int index = terminal; index >= 0; index = nodes[index].parent) {
    result.path.push_back(nodes[index].pose);
  }
  std::reverse(result.path.begin(), result.path.end());
  if (result.path.empty()) {
    result.failure = PlannerFailure::EmptyPath;
    result.message = "Hybrid A* reconstructed an empty path";
    return result;
  }
  result.path.push_back(goal);
  result.message = "Hybrid A* path ready";
  return result;
}

}  // namespace autonomy
