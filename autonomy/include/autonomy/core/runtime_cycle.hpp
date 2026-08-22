#pragma once

#include "autonomy/algorithm/algorithm_manager.hpp"
#include "autonomy/sensor/types.hpp"
#include "autonomy/core/types.hpp"

namespace autonomy {

/** One control pass: Middleware input → State → Sensor → Algorithm → Middleware output. */
struct RuntimeCycle {
  StateSnapshot state{};
  SensorSnapshot sensors{};
  AlgorithmOutput algorithms{};
  bool keep_running{true};
};

}  // namespace autonomy
