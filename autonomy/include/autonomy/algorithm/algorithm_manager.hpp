#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "autonomy/algorithm/detection/rgbd_detector.hpp"
#include "autonomy/algorithm/detection/types.hpp"
#include "autonomy/algorithm/elevation/elevation_mapper.hpp"
#include "autonomy/algorithm/elevation/occupancy_mapper.hpp"
#include "autonomy/algorithm/planner/hybrid_astar.hpp"
#include "autonomy/algorithm/planner/mppi.hpp"
#include "autonomy/algorithm/slam/super_lio.hpp"
#include "autonomy/core/types.hpp"
#include "autonomy/parameter/config.hpp"
#include "autonomy/sensor/types.hpp"

namespace autonomy {

enum class AlgorithmNode : std::uint8_t {
  Elevation,
  Slam,
  GlobalPlanner,
  LocalPlanner,
  AiDetection,
  MapWriter,
  ObjectTracking,
};

enum class AlgorithmStatus : std::uint8_t {
  Idle,
  WaitingForData,
  Running,
  Warning,
  Failed,
};

enum class AlgorithmErrorCode : std::int32_t {
  None = 0,
  ElevationFailure = 1001,
  SlamFailure = 1101,
  GlobalPlannerFailure = 1201,
  LocalPlannerFailure = 1202,
  AiDetectionFailure = 1301,
  MappingFailure = 1401,
  TrackingFailure = 1501,
};

[[nodiscard]] const char * algorithmNodeName(AlgorithmNode node);
[[nodiscard]] const char * algorithmStatusName(AlgorithmStatus status);

struct AlgorithmReport {
  AlgorithmNode node{AlgorithmNode::Elevation};
  AlgorithmStatus status{AlgorithmStatus::Idle};
  AlgorithmErrorCode error_code{AlgorithmErrorCode::None};
  std::string message{};
  std::chrono::steady_clock::time_point reported_at{};
};

struct VelocityCommand {
  float linear_x{0.0F};
  float linear_y{0.0F};
  float angular_z{0.0F};
};

struct AlgorithmOutput {
  ExecutionPlan plan{};
  std::optional<ElevationMap> elevation{};
  std::optional<Pose2d> slam_pose{};
  std::optional<SlamLinearVelocity> slam_linear_velocity{};
  std::optional<OccupancyGrid> occupancy_map{};
  std::optional<std::vector<Pose2d>> global_path{};
  std::optional<Detection> detection{};
  std::optional<DirectionalMotionFilter> directional_motion_filter{};
  std::vector<AlgorithmReport> reports{};
  std::optional<VelocityCommand> command{};
  std::optional<AlgorithmReport> fatal_error{};
};

/**
 * Owns all autonomy algorithms. Each algorithm's configured hz controls both
 * its update schedule and its execution deadline. ROS and DDS are never part
 * of this manager.
 */
class AlgorithmManager final {
public:
  explicit AlgorithmManager(const RuntimeConfig & config);

  void updateNavigationGoal(Pose2d goal);
  [[nodiscard]] AlgorithmOutput tick(
    const StateSnapshot & state,
    const SensorSnapshot & sensors,
    std::chrono::steady_clock::time_point now);

private:
  [[nodiscard]] ExecutionPlan schedule(AutonomyMode mode, std::chrono::steady_clock::time_point now);
  void runSlamAndMapping(
    AlgorithmOutput & output,
    const SensorSnapshot & sensors,
    std::chrono::steady_clock::time_point now);
  void runPlanner(AlgorithmOutput & output, std::chrono::steady_clock::time_point now);
  void runDirectionalMotionFilter(
    AlgorithmOutput & output,
    std::chrono::steady_clock::time_point now) const;
  void runDetection(
    AlgorithmOutput & output,
    const SensorSnapshot & sensors,
    std::chrono::steady_clock::time_point now);
  [[nodiscard]] static std::vector<Workload> workloadsFor(AutonomyMode mode);

  ElevationMapper elevation_mapper_;
  PlannerConfig planner_config_{};
  SlamConfig slam_config_{};
  MappingConfig mapping_config_{};
  DetectionConfig detection_config_{};
  SuperLioSlam slam_;
  OccupancyMapper mapper_;
  HybridAStarPlanner global_planner_;
  MppiPlanner local_planner_;
  RgbdDetector detector_;
  std::chrono::nanoseconds elevation_period_{};
  std::chrono::nanoseconds slam_period_{};
  std::chrono::nanoseconds planner_period_{};
  std::chrono::nanoseconds detection_period_{};
  std::chrono::steady_clock::time_point next_elevation_tick_{};
  std::chrono::steady_clock::time_point next_slam_tick_{};
  std::chrono::steady_clock::time_point next_planner_tick_{};
  std::chrono::steady_clock::time_point next_detection_tick_{};
  std::optional<Pose2d> slam_pose_{};
  std::optional<OccupancyGrid> occupancy_map_{};
  std::optional<Detection> detection_{};
  std::optional<Pose2d> navigation_goal_{};
};

}  // namespace autonomy
