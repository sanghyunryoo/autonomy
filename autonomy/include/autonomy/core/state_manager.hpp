#pragma once

#include <chrono>
#include <optional>
#include <utility>

#include "autonomy/core/types.hpp"
#include "autonomy/parameter/config.hpp"

namespace autonomy {

/**
 * Core FSM: decides the effective autonomy state.
 *
 * RobotReport selects the safe default (IDLE or ADAS). FSD, MAPPING, and
 * TRACKING are explicit requests and are retained only while the robot is in
 * an autonomy-capable state and no E-stop is active.
 */
class StateManager final {
public:
  explicit StateManager(StateManagerConfig config);

  void onRobotReport(RobotReport report);
  void raiseAlgorithmError(std::int32_t error_code, std::string reason);
  void clearAlgorithmError();
  [[nodiscard]] StateRequestResult requestExternalMode(AutonomyMode mode);
  void clearExternalRequest();
  [[nodiscard]] const StateSnapshot & tick(std::chrono::steady_clock::time_point now);
  [[nodiscard]] const StateSnapshot & state() const;

private:
  [[nodiscard]] AutonomyMode decide(std::chrono::steady_clock::time_point now) const;
  void setMode(AutonomyMode mode, std::chrono::steady_clock::time_point now);

  StateManagerConfig config_;
  std::optional<RobotReport> latest_robot_report_{};
  std::optional<AutonomyMode> external_request_{};
  std::optional<std::pair<std::int32_t, std::string>> algorithm_error_{};
  StateSnapshot state_{};
};

}  // namespace autonomy
