#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "autonomy/core/runtime_cycle.hpp"
#include "autonomy/parameter/config.hpp"
#include "autonomy/core/types.hpp"

namespace autonomy {

class SensorManager;

struct MiddlewareModeRequest {
  std::string operation_mode{};
  float speed_limit{0.0F};
  bool enable_ai{false};
  bool segmentation{false};
};

/**
 * ROS 2, core-msg, and Cyclone DDS adapter. It never owns the control loop:
 * RuntimeRunner calls poll() to drain ready callbacks and publish() at the
 * end of each control cycle. Debug and data outputs follow their configured
 * algorithm rates; the safety-gated ROS control command is refreshed at its
 * configured rate (10 Hz by default), with an immediate zero on unsafe edges.
 */
class MiddlewareManager final {
public:
  MiddlewareManager();
  ~MiddlewareManager();

  MiddlewareManager(const MiddlewareManager &) = delete;
  MiddlewareManager & operator=(const MiddlewareManager &) = delete;

  void init(const RuntimeConfig & config, SensorManager & sensor_manager);
  void poll();
  [[nodiscard]] bool isRunning() const;
  [[nodiscard]] std::optional<RobotReport> popRobotReport();
  [[nodiscard]] std::optional<MiddlewareModeRequest> popModeRequest();
  [[nodiscard]] std::optional<Pose2d> popNavigationGoal();
  void publish(const RuntimeCycle & cycle);
  void finish();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_{};
};

}  // namespace autonomy
