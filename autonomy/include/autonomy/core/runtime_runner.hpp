#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "autonomy/algorithm/algorithm_manager.hpp"
#include "autonomy/core/loop_timer.hpp"
#include "autonomy/middleware/middleware_manager.hpp"
#include "autonomy/parameter/parameter_manager.hpp"
#include "autonomy/sensor/sensor_manager.hpp"
#include "autonomy/core/state_manager.hpp"

namespace autonomy {

/**
 * Core composition root: owns startup, the deterministic 50 Hz control loop,
 * and the five managers.
 * Each loop explicitly drains Middleware input, decides State, builds a
 * non-IDLE Sensor snapshot, runs due algorithms, then returns one cycle for
 * Middleware output. ROS callbacks never drive this loop.
 */
class RuntimeRunner final {
public:
  RuntimeRunner();
  explicit RuntimeRunner(std::string config_directory);
  ~RuntimeRunner();

  RuntimeRunner(const RuntimeRunner &) = delete;
  RuntimeRunner & operator=(const RuntimeRunner &) = delete;

  [[nodiscard]] bool init();
  void run();
  void finish();

private:
  [[nodiscard]] RuntimeCycle runCycle(std::chrono::steady_clock::time_point now);
  void processMiddlewareInputs();
  [[nodiscard]] StateRequestResult handleModeRequest(const MiddlewareModeRequest & request);
  void requireInitialized() const;

  ParameterManager parameter_manager_;
  MiddlewareManager middleware_manager_{};
  std::unique_ptr<StateManager> state_manager_{};
  std::unique_ptr<SensorManager> sensor_manager_{};
  std::unique_ptr<AlgorithmManager> algorithm_manager_{};
  std::unique_ptr<LoopTimer> loop_timer_{};
  bool initialized_{false};
  bool running_{false};
};

}  // namespace autonomy
