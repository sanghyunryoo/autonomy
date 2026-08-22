#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "autonomy/algorithm/planner/types.hpp"
#include "autonomy/sensor/types.hpp"

namespace autonomy {

struct MappingConfig {
  std::string map_frame{"map"};
  double resolution{0.10};
  double x_min{-20.0};
  double x_max{20.0};
  double y_min{-20.0};
  double y_max{20.0};
  double obstacle_min_z{0.10};
  double obstacle_max_z{1.20};
  double maximum_range{20.0};
};

enum class MappingStatus : std::uint8_t {
  WaitingForData,
  Ready,
  Failed,
};

struct MappingResult {
  MappingStatus status{MappingStatus::WaitingForData};
  OccupancyGrid map{};
  std::string message{};
};

/** Builds the persistent map-frame OccupancyGrid consumed directly by Hybrid A*. */
class OccupancyMapper final {
public:
  explicit OccupancyMapper(MappingConfig config);

  [[nodiscard]] MappingResult update(
    const Pose2d & pose,
    const MergedSensorData & sensor_data,
    std::chrono::steady_clock::time_point now);
  void reset();

private:
  [[nodiscard]] bool inBounds(int column, int row) const;
  void setFree(int column, int row);
  void setOccupied(int column, int row);
  void markRay(int start_column, int start_row, int end_column, int end_row);
  void ensureGrid();

  MappingConfig config_{};
  OccupancyGrid map_{};
};

}  // namespace autonomy
