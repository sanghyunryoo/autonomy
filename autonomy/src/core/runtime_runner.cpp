#include "autonomy/core/runtime_runner.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace autonomy {

RuntimeRunner::RuntimeRunner() = default;

RuntimeRunner::RuntimeRunner(std::string config_directory)
: parameter_manager_(std::move(config_directory))
{
}

RuntimeRunner::~RuntimeRunner()
{
  finish();
}

bool RuntimeRunner::init()
{
  if (initialized_) return true;

  parameter_manager_.load();
  const RuntimeConfig & config = parameter_manager_.runtime();

  // SensorManager owns direct SDK driver inventory and URDF transforms before
  // MiddlewareManager creates ROS/DDS endpoints.
  sensor_manager_ = std::make_unique<SensorManager>(config);
  state_manager_ = std::make_unique<StateManager>(config.state);
  algorithm_manager_ = std::make_unique<AlgorithmManager>(config);
  const auto loop_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / config.loop_hz));
  loop_timer_ = std::make_unique<LoopTimer>(loop_period);

  middleware_manager_.init(config, *sensor_manager_);
  initialized_ = true;
  running_ = true;
  return true;
}

void RuntimeRunner::run()
{
  requireInitialized();
  loop_timer_->init();
  while (running_ && middleware_manager_.isRunning()) {
    processMiddlewareInputs();
    const RuntimeCycle cycle = runCycle(std::chrono::steady_clock::now());
    middleware_manager_.publish(cycle);
    if (!cycle.keep_running) {
      running_ = false;
      break;
    }

    loop_timer_->rest();
  }
}

void RuntimeRunner::finish()
{
  running_ = false;
  middleware_manager_.finish();
  loop_timer_.reset();
  algorithm_manager_.reset();
  sensor_manager_.reset();
  state_manager_.reset();
  initialized_ = false;
}

RuntimeCycle RuntimeRunner::runCycle(const std::chrono::steady_clock::time_point now)
{
  requireInitialized();
  RuntimeCycle cycle;
  cycle.state = state_manager_->tick(now);
  const bool acquire = cycle.state.mode != AutonomyMode::Idle && cycle.state.mode != AutonomyMode::Error;
  sensor_manager_->setAcquisitionEnabled(acquire);
  if (!acquire) {
    cycle.algorithms.plan.mode = cycle.state.mode;
    return cycle;
  }

  cycle.sensors = sensor_manager_->snapshot(now);
  cycle.algorithms = algorithm_manager_->tick(cycle.state, cycle.sensors, now);
  
  if (cycle.algorithms.fatal_error) {
    const AlgorithmReport & error = *cycle.algorithms.fatal_error;
    state_manager_->raiseAlgorithmError(static_cast<std::int32_t>(error.error_code), error.message);
    cycle.state = state_manager_->tick(now);
    sensor_manager_->setAcquisitionEnabled(false);
    cycle.sensors = {};
    cycle.algorithms.plan = {};
    cycle.algorithms.plan.mode = AutonomyMode::Error;
    cycle.keep_running = false;
    return cycle;
  }
  return cycle;
}

void RuntimeRunner::processMiddlewareInputs()
{
  middleware_manager_.poll();
  while (const std::optional<RobotReport> report = middleware_manager_.popRobotReport()) {
    state_manager_->onRobotReport(*report);
  }
  while (const std::optional<MiddlewareModeRequest> request = middleware_manager_.popModeRequest()) {
    const StateRequestResult result = handleModeRequest(*request);
    if (!result.accepted) {
      std::cerr << "[RuntimeRunner][WARN] rejected queued mode request: " << result.message << '\n';
    }
  }
  while (const std::optional<Pose2d> goal = middleware_manager_.popNavigationGoal()) {
    algorithm_manager_->updateNavigationGoal(*goal);
  }
}

StateRequestResult RuntimeRunner::handleModeRequest(const MiddlewareModeRequest & request)
{
  requireInitialized();
  const std::optional<AutonomyMode> requested = parseAutonomyMode(request.operation_mode);
  StateRequestResult result;
  if (!requested) {
    result = {false, "operation_mode must be ADAS, FSD, MAPPING, or TRACKING"};
  } else if (*requested == AutonomyMode::Adas) {
    state_manager_->clearExternalRequest();
    state_manager_->clearAlgorithmError();
    result = {true, "Released explicit mode request and cleared the latched algorithm error"};
  } else {
    result = state_manager_->requestExternalMode(*requested);
  }

  return result;
}

void RuntimeRunner::requireInitialized() const
{
  if (!initialized_) {
    throw std::logic_error("RuntimeRunner::init must be called first");
  }
}

}  // namespace autonomy
