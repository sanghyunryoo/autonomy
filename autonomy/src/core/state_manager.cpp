#include "autonomy/core/state_manager.hpp"

#include <utility>

namespace autonomy {

StateManager::StateManager(StateManagerConfig config)
: config_(std::move(config))
{
  state_.mode = AutonomyMode::Idle;
  state_.requested_mode = AutonomyMode::Adas;
}

void StateManager::onRobotReport(RobotReport report)
{
  report.mode = robotModeFromReport(report.raw_mode, report.mode_name);
  latest_robot_report_ = std::move(report);
}

void StateManager::raiseAlgorithmError(const std::int32_t error_code, std::string reason)
{
  algorithm_error_ = std::make_pair(error_code, std::move(reason));
}

void StateManager::clearAlgorithmError()
{
  algorithm_error_.reset();
}

StateRequestResult StateManager::requestExternalMode(const AutonomyMode mode)
{
  if (!isExternalAutonomyMode(mode)) {
    return {false, "Only FSD, MAPPING, and TRACKING are requested externally"};
  }

  external_request_ = mode;
  state_.requested_mode = mode;
  state_.has_external_request = true;
  return {true, std::string("Requested ") + autonomyModeName(mode)};
}

void StateManager::clearExternalRequest()
{
  external_request_.reset();
  state_.requested_mode = AutonomyMode::Adas;
  state_.has_external_request = false;
}

const StateSnapshot & StateManager::tick(const std::chrono::steady_clock::time_point now)
{
  bool external_rearm_required = algorithm_error_.has_value();
  if (latest_robot_report_) {
    state_.robot_report = *latest_robot_report_;
    state_.robot_report_fresh = now - latest_robot_report_->received_at <= config_.robot_report_timeout;
    state_.estop_active = latest_robot_report_->physical_estop || latest_robot_report_->comm_estop;
    external_rearm_required = external_rearm_required || !state_.robot_report_fresh ||
      latest_robot_report_->physical_estop || latest_robot_report_->comm_estop ||
      latest_robot_report_->comm_fault || robotModeRequiresIdleAutonomy(latest_robot_report_->mode);
  } else {
    state_.robot_report = RobotReport{};
    state_.robot_report_fresh = false;
    state_.estop_active = false;
    external_rearm_required = external_rearm_required || config_.require_robot_report;
  }
  if (external_rearm_required && external_request_) {
    // A safe robot state, communication fault, stale report, or algorithm
    // error must be followed by a new explicit external FSD/Mapping/Tracking
    // request; an old request must never resume motion after recovery.
    clearExternalRequest();
  }
  if (algorithm_error_) {
    state_.algorithm_error_code = algorithm_error_->first;
    state_.algorithm_error_reason = algorithm_error_->second;
  } else {
    state_.algorithm_error_code = 0;
    state_.algorithm_error_reason.clear();
  }

  setMode(decide(now), now);
  return state_;
}

const StateSnapshot & StateManager::state() const
{
  return state_;
}

AutonomyMode StateManager::decide(const std::chrono::steady_clock::time_point now) const
{
  if (algorithm_error_) {
    return AutonomyMode::Error;
  }
  if (!latest_robot_report_) {
    return config_.require_robot_report ? AutonomyMode::Idle :
      (external_request_.value_or(AutonomyMode::Adas));
  }

  const RobotReport & report = *latest_robot_report_;
  const bool fresh = now - report.received_at <= config_.robot_report_timeout;
  if (!fresh || report.physical_estop || report.comm_estop || report.comm_fault ||
      robotModeRequiresIdleAutonomy(report.mode)) {
    return AutonomyMode::Idle;
  }

  return external_request_.value_or(AutonomyMode::Adas);
}

void StateManager::setMode(const AutonomyMode mode, const std::chrono::steady_clock::time_point now)
{
  if (state_.mode == mode) {
    return;
  }

  state_.mode = mode;
  state_.changed_at = now;
  ++state_.transition_sequence;
}

}  // namespace autonomy
