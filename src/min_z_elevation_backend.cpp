#include "height_map_ros2/min_z_elevation_backend.hpp"

#include <cmath>
#include <limits>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace height_map_ros2
{

MinZElevationBackend::MinZElevationBackend(GridSpec spec)
: spec_(std::move(spec))
{
}

ElevationGrid MinZElevationBackend::build(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std_msgs::msg::Header & output_header)
{
  ElevationGrid grid(spec_);
  grid.header = output_header;

  sensor_msgs::PointCloud2ConstIterator<float> x_it(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it(cloud, "z");

  const auto width = grid.spec.width();
  const auto height = grid.spec.height();

  for (; x_it != x_it.end(); ++x_it, ++y_it, ++z_it) {
    const float x = *x_it;
    const float y = *y_it;
    const float z = *z_it;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    if (x < spec_.x_min || x >= spec_.x_max || y < spec_.y_min || y >= spec_.y_max) {
      continue;
    }
    if (z < spec_.min_z || z > spec_.max_z) {
      continue;
    }

    const auto col = static_cast<std::uint32_t>((x - spec_.x_min) / spec_.resolution);
    const auto row = static_cast<std::uint32_t>((y - spec_.y_min) / spec_.resolution);
    if (col >= width || row >= height) {
      continue;
    }

    const auto index = static_cast<std::size_t>(row) * width + col;
    float & cell = grid.height[index];
    if (!std::isfinite(cell) || z < cell) {
      cell = z;
    }
  }

  return grid;
}

}  // namespace height_map_ros2
