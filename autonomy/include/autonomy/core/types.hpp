#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace autonomy {

/**
 * Core operating modes. These are deliberately independent from ROS messages
 * so the control policy can also run without a ROS executor.
 */
enum class AutonomyMode : std::int8_t {
  Error = -1,
  Idle = 0,
  Adas = 1,
  Fsd = 2,
  Mapping = 3,
  Tracking = 4,
};

[[nodiscard]] const char * autonomyModeName(AutonomyMode mode);
[[nodiscard]] std::optional<AutonomyMode> parseAutonomyMode(const std::string & text);
[[nodiscard]] bool isExternalAutonomyMode(AutonomyMode mode);

/** The robot states supplied by core/RobotReport. */
enum class RobotMode : std::uint8_t {
  Idle = 0,
  Init = 1,
  Ready = 2,
  Stand = 3,
  FlatDrive = 4,
  RoughDrive = 5,
  CustomDrive = 6,
  Sdk = 7,
  Freeze = 8,
  Sit = 9,
  Lie = 10,
  Unknown = 255,
};

[[nodiscard]] RobotMode robotModeFromReport(std::uint8_t value, const std::string & name);
[[nodiscard]] bool robotModeRequiresIdleAutonomy(RobotMode mode);

/** ROS-free projection of the fields that affect autonomy policy. */
struct RobotReport {
  RobotMode mode{RobotMode::Unknown};
  std::uint8_t raw_mode{255};
  std::string mode_name{};
  bool physical_estop{false};
  bool comm_estop{false};
  bool comm_fault{false};
  std::string error_reason{};
  std::chrono::steady_clock::time_point received_at{};
};

struct StateRequestResult {
  bool accepted{false};
  std::string message{};
};

struct StateSnapshot {
  AutonomyMode mode{AutonomyMode::Idle};
  AutonomyMode requested_mode{AutonomyMode::Adas};
  bool has_external_request{false};
  bool estop_active{false};
  std::int32_t algorithm_error_code{0};
  std::string algorithm_error_reason{};
  bool robot_report_fresh{false};
  RobotReport robot_report{};
  std::uint64_t transition_sequence{0};
  std::chrono::steady_clock::time_point changed_at{};
};

enum class Workload : std::uint8_t {
  Sensors,
  Elevation,
  Slam,
  GlobalPlanner,
  LocalPlanner,
  AiDetection,
  MapWriter,
  ObjectTracking,
  CommandOutput,
};

[[nodiscard]] const char * workloadName(Workload workload);

struct ExecutionPlan {
  AutonomyMode mode{AutonomyMode::Idle};
  bool sensors_enabled{false};
  bool elevation_due{false};
  bool slam_due{false};
  bool planner_due{false};
  bool detection_due{false};
  std::vector<Workload> active_workloads{};
};

}  // namespace autonomy
