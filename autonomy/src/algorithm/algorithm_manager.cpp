#include "autonomy/algorithm/algorithm_manager.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace autonomy {
namespace {

bool hasWorkload(const ExecutionPlan & plan, const Workload workload)
{
  return std::find(plan.active_workloads.begin(), plan.active_workloads.end(), workload) !=
    plan.active_workloads.end();
}

bool fresh(
  const std::chrono::steady_clock::time_point received_at,
  const std::chrono::milliseconds timeout,
  const std::chrono::steady_clock::time_point now)
{
  return received_at != std::chrono::steady_clock::time_point{} && now >= received_at &&
    now - received_at <= timeout;
}

AlgorithmReport report(
  const AlgorithmNode node,
  const AlgorithmStatus status,
  std::string message,
  const std::chrono::steady_clock::time_point now)
{
  AlgorithmReport result;
  result.node = node;
  result.status = status;
  result.message = std::move(message);
  result.reported_at = now;
  return result;
}

std::chrono::nanoseconds periodFromHz(const double hz)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / hz));
}

bool consumeDue(
  const bool enabled,
  const std::chrono::nanoseconds period,
  std::chrono::steady_clock::time_point & next_tick,
  const std::chrono::steady_clock::time_point now)
{
  if (!enabled) {
    next_tick = {};
    return false;
  }
  if (next_tick == std::chrono::steady_clock::time_point{} || now >= next_tick) {
    if (next_tick == std::chrono::steady_clock::time_point{}) {
      next_tick = now + period;
    } else {
      do {
        next_tick += period;
      } while (next_tick <= now);
    }
    return true;
  }
  return false;
}

void appendDeadlineWarning(
  AlgorithmOutput & output,
  const AlgorithmNode node,
  const char * name,
  const std::chrono::steady_clock::time_point started_at,
  const std::chrono::nanoseconds deadline)
{
  const auto elapsed = std::chrono::steady_clock::now() - started_at;
  if (elapsed <= deadline) return;

  const double elapsed_ms = std::chrono::duration<double, std::milli>(elapsed).count();
  const double deadline_ms = std::chrono::duration<double, std::milli>(deadline).count();
  std::cerr << "[AlgorithmManager][WARN] " << name << " deadline missed: processing=" <<
    elapsed_ms << " ms, target=" << deadline_ms << " ms\n";
  output.reports.push_back(report(
    node, AlgorithmStatus::Warning,
    std::string(name) + " deadline missed: processing=" + std::to_string(elapsed_ms) +
    " ms, target=" + std::to_string(deadline_ms) + " ms",
    std::chrono::steady_clock::now()));
}

}  // namespace

const char * algorithmNodeName(const AlgorithmNode node)
{
  switch (node) {
    case AlgorithmNode::Elevation: return "ELEVATION";
    case AlgorithmNode::Slam: return "SLAM";
    case AlgorithmNode::GlobalPlanner: return "GLOBAL_PLANNER";
    case AlgorithmNode::LocalPlanner: return "LOCAL_PLANNER";
    case AlgorithmNode::AiDetection: return "AI_DETECTION";
    case AlgorithmNode::MapWriter: return "MAP_WRITER";
    case AlgorithmNode::ObjectTracking: return "OBJECT_TRACKING";
    default: return "UNKNOWN";
  }
}

const char * algorithmStatusName(const AlgorithmStatus status)
{
  switch (status) {
    case AlgorithmStatus::Idle: return "idle";
    case AlgorithmStatus::WaitingForData: return "waiting_for_data";
    case AlgorithmStatus::Running: return "running";
    case AlgorithmStatus::Warning: return "warning";
    case AlgorithmStatus::Failed: return "failed";
    default: return "unknown";
  }
}

AlgorithmManager::AlgorithmManager(const RuntimeConfig & config)
: elevation_mapper_(config.elevation),
  planner_config_(config.planner),
  slam_config_(config.slam),
  mapping_config_(config.mapping),
  detection_config_(config.detection),
  slam_(config.slam),
  mapper_(config.mapping),
  global_planner_(config.planner.hybrid_astar),
  local_planner_(config.planner.mppi),
  detector_(config.detection),
  elevation_period_(periodFromHz(config.elevation.hz)),
  slam_period_(periodFromHz(config.slam.hz)),
  planner_period_(periodFromHz(config.planner.hz)),
  detection_period_(periodFromHz(config.detection.hz))
{
  if (config.loop_hz <= 0.0 || elevation_period_ <= std::chrono::nanoseconds::zero() ||
    slam_period_ <= std::chrono::nanoseconds::zero() || planner_period_ <= std::chrono::nanoseconds::zero() ||
    detection_period_ <= std::chrono::nanoseconds::zero() ||
    config.slam.map_frame != config.mapping.map_frame) {
    throw std::invalid_argument("runtime algorithm configuration is invalid");
  }
}

void AlgorithmManager::updateNavigationGoal(Pose2d goal)
{
  if (goal.received_at == std::chrono::steady_clock::time_point{}) {
    goal.received_at = std::chrono::steady_clock::now();
  }
  navigation_goal_ = std::move(goal);
}

AlgorithmOutput AlgorithmManager::tick(
  const StateSnapshot & state,
  const SensorSnapshot & sensors,
  const std::chrono::steady_clock::time_point now)
{
  AlgorithmOutput output;
  output.plan = schedule(state.mode, now);
  if (state.mode == AutonomyMode::Idle || state.mode == AutonomyMode::Error) return output;

  if (output.plan.elevation_due) {
    const auto started_at = std::chrono::steady_clock::now();
    output.elevation = elevation_mapper_.build(sensors.merged, now);
    output.reports.push_back(report(
      AlgorithmNode::Elevation,
      sensors.merged.points.empty() ? AlgorithmStatus::WaitingForData : AlgorithmStatus::Running,
      sensors.merged.points.empty() ? "waiting for transformed camera/lidar points" : "elevation grid updated",
      now));
    appendDeadlineWarning(output, AlgorithmNode::Elevation, "elevation", started_at, elevation_period_);
  }

  if (output.plan.slam_due) {
    const auto started_at = std::chrono::steady_clock::now();
    runSlamAndMapping(output, sensors, now);
    appendDeadlineWarning(output, AlgorithmNode::Slam, "SLAM/mapping", started_at, slam_period_);
    if (output.fatal_error) return output;
  }
  if (output.plan.planner_due) {
    const auto started_at = std::chrono::steady_clock::now();
    runDirectionalMotionFilter(output, now);
    runPlanner(output, now);
    appendDeadlineWarning(output, AlgorithmNode::GlobalPlanner, "planner", started_at, planner_period_);
    if (output.fatal_error) return output;
  }
  if (output.plan.detection_due) {
    const auto started_at = std::chrono::steady_clock::now();
    runDetection(output, sensors, now);
    appendDeadlineWarning(output, AlgorithmNode::AiDetection, "detection", started_at, detection_period_);
    if (output.fatal_error) return output;
  }

  if (hasWorkload(output.plan, Workload::ObjectTracking) && output.plan.detection_due) {
    const bool target_fresh = detection_ &&
      fresh(detection_->target.received_at, std::chrono::milliseconds(500), now);
    if (!target_fresh) {
      output.reports.push_back(report(
        AlgorithmNode::ObjectTracking, AlgorithmStatus::WaitingForData,
        "waiting for internal RGB-D detection target", now));
      return output;
    }
    const float distance = std::hypot(detection_->target.x, detection_->target.y);
    VelocityCommand command;
    command.linear_x = std::clamp((distance - 0.8F) * 0.7F, -0.35F, 0.45F);
    command.angular_z = std::clamp(
      std::atan2(detection_->target.y, detection_->target.x), -0.8F, 0.8F);
    output.command = command;
    output.reports.push_back(report(
      AlgorithmNode::ObjectTracking, AlgorithmStatus::Running, "tracking command ready", now));
  }
  return output;
}

void AlgorithmManager::runDirectionalMotionFilter(
  AlgorithmOutput & output,
  const std::chrono::steady_clock::time_point now) const
{
  DirectionalMotionFilter filter;
  const bool inputs_fresh = occupancy_map_ && slam_pose_ &&
    fresh(occupancy_map_->received_at, planner_config_.input_timeout, now) &&
    fresh(slam_pose_->received_at, planner_config_.input_timeout, now);
  if (inputs_fresh) {
    filter = local_planner_.evaluateDirectionalFilter(*occupancy_map_, *slam_pose_);
  }
  output.directional_motion_filter = filter;
}

ExecutionPlan AlgorithmManager::schedule(
  const AutonomyMode mode,
  const std::chrono::steady_clock::time_point now)
{
  ExecutionPlan plan;
  plan.mode = mode;
  plan.active_workloads = workloadsFor(mode);
  plan.sensors_enabled = hasWorkload(plan, Workload::Sensors);
  plan.elevation_due = consumeDue(
    hasWorkload(plan, Workload::Elevation), elevation_period_, next_elevation_tick_, now);
  plan.slam_due = consumeDue(
    hasWorkload(plan, Workload::Slam), slam_period_, next_slam_tick_, now);
  plan.planner_due = consumeDue(
    hasWorkload(plan, Workload::GlobalPlanner) && hasWorkload(plan, Workload::LocalPlanner),
    planner_period_, next_planner_tick_, now);
  plan.detection_due = consumeDue(
    hasWorkload(plan, Workload::AiDetection), detection_period_, next_detection_tick_, now);
  return plan;
}

void AlgorithmManager::runSlamAndMapping(
  AlgorithmOutput & output,
  const SensorSnapshot & sensors,
  const std::chrono::steady_clock::time_point now)
{
  const SlamResult slam_result = slam_.process(sensors, now);
  if (slam_result.status == SlamStatus::Failed) {
    AlgorithmReport failure = report(AlgorithmNode::Slam, AlgorithmStatus::Failed, slam_result.message, now);
    failure.error_code = AlgorithmErrorCode::SlamFailure;
    output.reports.push_back(failure);
    output.fatal_error = std::move(failure);
    return;
  }
  if (slam_result.status == SlamStatus::WaitingForData) {
    output.reports.push_back(report(
      AlgorithmNode::Slam, AlgorithmStatus::WaitingForData, slam_result.message, now));
    return;
  }
  slam_pose_ = slam_result.pose;
  output.slam_pose = slam_result.pose;
  output.slam_linear_velocity = slam_result.linear_velocity;
  output.reports.push_back(report(AlgorithmNode::Slam, AlgorithmStatus::Running, slam_result.message, now));

  const bool mapping_required = hasWorkload(output.plan, Workload::MapWriter) ||
    hasWorkload(output.plan, Workload::GlobalPlanner);
  if (!mapping_required) return;
  const MappingResult mapping_result = mapper_.update(slam_result.pose, sensors.merged, now);
  if (mapping_result.status == MappingStatus::Failed) {
    AlgorithmReport failure = report(AlgorithmNode::MapWriter, AlgorithmStatus::Failed, mapping_result.message, now);
    failure.error_code = AlgorithmErrorCode::MappingFailure;
    output.reports.push_back(failure);
    output.fatal_error = std::move(failure);
    return;
  }
  if (mapping_result.status == MappingStatus::WaitingForData) {
    output.reports.push_back(report(
      AlgorithmNode::MapWriter, AlgorithmStatus::WaitingForData, mapping_result.message, now));
    return;
  }
  occupancy_map_ = mapping_result.map;
  output.occupancy_map = mapping_result.map;
  output.reports.push_back(report(
    AlgorithmNode::MapWriter, AlgorithmStatus::Running, mapping_result.message, now));
}

void AlgorithmManager::runPlanner(
  AlgorithmOutput & output,
  const std::chrono::steady_clock::time_point now)
{
  if (!hasWorkload(output.plan, Workload::GlobalPlanner) ||
    !hasWorkload(output.plan, Workload::LocalPlanner)) {
    return;
  }
  const bool inputs_present = occupancy_map_ && slam_pose_ && navigation_goal_;
  const bool inputs_fresh = inputs_present &&
    fresh(occupancy_map_->received_at, planner_config_.input_timeout, now) &&
    fresh(slam_pose_->received_at, planner_config_.input_timeout, now) &&
    fresh(navigation_goal_->received_at, planner_config_.input_timeout, now);
  if (!inputs_fresh) {
    output.reports.push_back(report(
      AlgorithmNode::GlobalPlanner, AlgorithmStatus::WaitingForData,
      "waiting for internal occupancy map, internal SLAM pose, and goal", now));
    output.reports.push_back(report(
      AlgorithmNode::LocalPlanner, AlgorithmStatus::WaitingForData,
      "waiting for Hybrid A* global path", now));
    return;
  }
  const GlobalPlanResult global = global_planner_.plan(*occupancy_map_, *slam_pose_, *navigation_goal_);
  if (!global.succeeded()) {
    AlgorithmReport failure = report(
      AlgorithmNode::GlobalPlanner, AlgorithmStatus::Failed,
      std::string("Hybrid A* ") + plannerFailureName(global.failure) + ": " + global.message, now);
    failure.error_code = AlgorithmErrorCode::GlobalPlannerFailure;
    output.reports.push_back(failure);
    output.fatal_error = std::move(failure);
    return;
  }
  output.global_path = global.path;
  output.reports.push_back(report(
    AlgorithmNode::GlobalPlanner, AlgorithmStatus::Running, global.message, now));

  const LocalPlanResult local = local_planner_.solve(*occupancy_map_, *slam_pose_, *output.global_path);
  if (!local.succeeded()) {
    AlgorithmReport failure = report(
      AlgorithmNode::LocalPlanner, AlgorithmStatus::Failed,
      std::string("MPPI ") + plannerFailureName(local.failure) + ": " + local.message, now);
    failure.error_code = AlgorithmErrorCode::LocalPlannerFailure;
    output.reports.push_back(failure);
    output.fatal_error = std::move(failure);
    return;
  }
  output.command = VelocityCommand{
    static_cast<float>(local.linear_x), 0.0F, static_cast<float>(local.angular_z)};
  output.reports.push_back(report(
    AlgorithmNode::LocalPlanner, AlgorithmStatus::Running, local.message, now));
}

void AlgorithmManager::runDetection(
  AlgorithmOutput & output,
  const SensorSnapshot & sensors,
  const std::chrono::steady_clock::time_point now)
{
  if (!hasWorkload(output.plan, Workload::AiDetection)) return;
  const DetectionResult detection_result = detector_.infer(sensors, now);
  if (detection_result.status == DetectionStatus::Failed) {
    AlgorithmReport failure = report(
      AlgorithmNode::AiDetection, AlgorithmStatus::Failed, detection_result.message, now);
    failure.error_code = AlgorithmErrorCode::AiDetectionFailure;
    output.reports.push_back(failure);
    output.fatal_error = std::move(failure);
    return;
  }
  if (detection_result.status == DetectionStatus::WaitingForData) {
    output.reports.push_back(report(
      AlgorithmNode::AiDetection, AlgorithmStatus::WaitingForData, detection_result.message, now));
    return;
  }
  detection_ = *detection_result.detection;
  output.detection = detection_;
  output.reports.push_back(report(
    AlgorithmNode::AiDetection, AlgorithmStatus::Running, detection_result.message, now));
}

std::vector<Workload> AlgorithmManager::workloadsFor(const AutonomyMode mode)
{
  switch (mode) {
    case AutonomyMode::Adas:
      return {Workload::Sensors, Workload::Elevation, Workload::Slam,
        Workload::GlobalPlanner, Workload::LocalPlanner, Workload::AiDetection,
        Workload::CommandOutput};
    case AutonomyMode::Fsd:
      return {Workload::Sensors, Workload::Elevation, Workload::Slam,
        Workload::GlobalPlanner, Workload::LocalPlanner, Workload::CommandOutput};
    case AutonomyMode::Mapping:
      return {Workload::Sensors, Workload::Elevation, Workload::Slam, Workload::MapWriter};
    case AutonomyMode::Tracking:
      return {Workload::Sensors, Workload::Elevation, Workload::AiDetection,
        Workload::ObjectTracking, Workload::CommandOutput};
    case AutonomyMode::Idle:
    case AutonomyMode::Error:
    default:
      return {};
  }
}

}  // namespace autonomy
