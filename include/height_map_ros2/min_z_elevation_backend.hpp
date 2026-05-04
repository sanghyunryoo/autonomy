#pragma once

#include "height_map_ros2/elevation_map_backend.hpp"

namespace height_map_ros2
{

class MinZElevationBackend final : public ElevationMapBackend
{
public:
  explicit MinZElevationBackend(GridSpec spec);

  ElevationGrid build(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const std_msgs::msg::Header & output_header) override;

private:
  GridSpec spec_;
};

}  // namespace height_map_ros2
