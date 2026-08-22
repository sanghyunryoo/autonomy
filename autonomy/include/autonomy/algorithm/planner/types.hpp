#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace autonomy {

/** Shared, transport-neutral data types for the in-process planners. */
struct Pose2d {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::string frame_id{};
  std::chrono::steady_clock::time_point received_at{};
};

/** Transport-neutral nav map. Cells follow ROS OccupancyGrid semantics. */
struct OccupancyGrid {
  std::string frame_id{};
  double resolution{0.1};
  double origin_x{0.0};
  double origin_y{0.0};
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::vector<std::int8_t> cells{};
  std::chrono::steady_clock::time_point received_at{};

  [[nodiscard]] bool valid() const
  {
    return resolution > 0.0 && width > 0U && height > 0U &&
      cells.size() == static_cast<std::size_t>(width) * height;
  }
};

enum class PlannerFailure : std::uint8_t {
  None,
  InvalidMap,
  FrameMismatch,
  StartOutsideMap,
  GoalOutsideMap,
  StartOccupied,
  GoalOccupied,
  SearchExhausted,
  SearchLimit,
  EmptyPath,
  NoFeasibleTrajectory,
};

[[nodiscard]] const char * plannerFailureName(PlannerFailure failure);

struct HybridAStarConfig {
  double wheelbase{0.45};
  double vehicle_radius{0.35};
  double step_size{0.25};
  double max_steering_radians{0.55};
  double goal_tolerance{0.30};
  std::uint32_t heading_bins{72};
  std::uint32_t max_expansions{20000};
  std::int8_t occupied_threshold{50};
  bool unknown_is_obstacle{true};
  bool allow_reverse{true};
};

struct MppiConfig {
  std::uint32_t sample_count{128};
  std::uint32_t horizon_steps{20};
  double dt{0.10};
  double max_linear_velocity{0.45};
  double max_angular_velocity{0.80};
  double linear_noise{0.18};
  double angular_noise{0.35};
  double temperature{1.0};
  double vehicle_radius{0.35};
  std::int8_t occupied_threshold{50};
  bool unknown_is_obstacle{true};
};

/**
 * Directional motion permission consumed by core/CommandFilter.
 *
 * X directions are produced from conservative MPPI footprint rollouts. Y is
 * intentionally left open for the robot's lateral-motion controller.
 */
struct DirectionalMotionFilter {
  bool allow_linear_vel_forward_x{false};
  bool allow_linear_vel_backward_x{false};
  bool allow_linear_vel_forward_y{true};
  bool allow_linear_vel_backward_y{true};
};

struct PlannerConfig {
  double hz{10.0};
  std::chrono::milliseconds input_timeout{500};
  HybridAStarConfig hybrid_astar{};
  MppiConfig mppi{};
};

struct GlobalPlanResult {
  PlannerFailure failure{PlannerFailure::None};
  std::string message{};
  std::vector<Pose2d> path{};

  [[nodiscard]] bool succeeded() const { return failure == PlannerFailure::None; }
};

struct LocalPlanResult {
  PlannerFailure failure{PlannerFailure::None};
  std::string message{};
  double linear_x{0.0};
  double angular_z{0.0};

  [[nodiscard]] bool succeeded() const { return failure == PlannerFailure::None; }
};

}  // namespace autonomy
