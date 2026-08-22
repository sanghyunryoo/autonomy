#include "autonomy/algorithm/elevation/occupancy_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace autonomy {

OccupancyMapper::OccupancyMapper(MappingConfig config)
: config_(std::move(config))
{
  if (config_.map_frame.empty() || config_.resolution <= 0.0 || config_.x_max <= config_.x_min || config_.y_max <= config_.y_min ||
    config_.obstacle_max_z <= config_.obstacle_min_z || config_.maximum_range <= 0.0) {
    throw std::invalid_argument("occupancy mapping configuration is invalid");
  }
  ensureGrid();
}

MappingResult OccupancyMapper::update(
  const Pose2d & pose,
  const MergedSensorData & sensor_data,
  const std::chrono::steady_clock::time_point now)
{
  MappingResult result;
  if (pose.frame_id != map_.frame_id) {
    result.status = MappingStatus::Failed;
    result.message = "SLAM pose frame does not match occupancy-map frame";
    return result;
  }
  if (sensor_data.points.empty()) {
    result.message = "waiting for merged points to update occupancy map";
    return result;
  }
  const int robot_column = static_cast<int>(std::floor((pose.x - map_.origin_x) / map_.resolution));
  const int robot_row = static_cast<int>(std::floor((pose.y - map_.origin_y) / map_.resolution));
  if (!inBounds(robot_column, robot_row)) {
    result.status = MappingStatus::Failed;
    result.message = "SLAM pose left the configured occupancy-map bounds";
    return result;
  }
  setFree(robot_column, robot_row);
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  std::size_t accepted = 0U;
  for (const Point3f & point : sensor_data.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    if (std::hypot(point.x, point.y) > config_.maximum_range) continue;
    const double map_x = pose.x + cosine * point.x - sine * point.y;
    const double map_y = pose.y + sine * point.x + cosine * point.y;
    const int column = static_cast<int>(std::floor((map_x - map_.origin_x) / map_.resolution));
    const int row = static_cast<int>(std::floor((map_y - map_.origin_y) / map_.resolution));
    if (!inBounds(column, row)) continue;
    markRay(robot_column, robot_row, column, row);
    if (point.z >= config_.obstacle_min_z && point.z <= config_.obstacle_max_z) {
      setOccupied(column, row);
    } else {
      setFree(column, row);
    }
    ++accepted;
  }
  if (accepted == 0U) {
    result.message = "waiting for points inside occupancy-map bounds";
    return result;
  }
  map_.received_at = now;
  result.status = MappingStatus::Ready;
  result.map = map_;
  result.message = "occupancy map updated";
  return result;
}

void OccupancyMapper::reset()
{
  map_ = OccupancyGrid{};
  ensureGrid();
}

bool OccupancyMapper::inBounds(const int column, const int row) const
{
  return column >= 0 && row >= 0 && column < static_cast<int>(map_.width) &&
    row < static_cast<int>(map_.height);
}

void OccupancyMapper::setFree(const int column, const int row)
{
  if (!inBounds(column, row)) return;
  std::int8_t & cell = map_.cells[static_cast<std::size_t>(row) * map_.width + column];
  if (cell < 50) cell = 0;
}

void OccupancyMapper::setOccupied(const int column, const int row)
{
  if (!inBounds(column, row)) return;
  map_.cells[static_cast<std::size_t>(row) * map_.width + column] = 100;
}

void OccupancyMapper::markRay(int start_column, int start_row, const int end_column, const int end_row)
{
  int delta_column = std::abs(end_column - start_column);
  const int step_column = start_column < end_column ? 1 : -1;
  int delta_row = -std::abs(end_row - start_row);
  const int step_row = start_row < end_row ? 1 : -1;
  int error = delta_column + delta_row;
  while (start_column != end_column || start_row != end_row) {
    setFree(start_column, start_row);
    const int twice_error = 2 * error;
    if (twice_error >= delta_row) {
      error += delta_row;
      start_column += step_column;
    }
    if (twice_error <= delta_column) {
      error += delta_column;
      start_row += step_row;
    }
    if (!inBounds(start_column, start_row)) return;
  }
}

void OccupancyMapper::ensureGrid()
{
  map_.frame_id = config_.map_frame;
  map_.resolution = config_.resolution;
  map_.origin_x = config_.x_min;
  map_.origin_y = config_.y_min;
  map_.width = static_cast<std::uint32_t>(std::ceil((config_.x_max - config_.x_min) / config_.resolution));
  map_.height = static_cast<std::uint32_t>(std::ceil((config_.y_max - config_.y_min) / config_.resolution));
  map_.cells.assign(static_cast<std::size_t>(map_.width) * map_.height, -1);
}

}  // namespace autonomy
