#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "elevation_grid.hpp"

namespace height_map_ros2
{

class ElevationMapBackend
{
public:
  virtual ~ElevationMapBackend() = default;

  virtual ElevationGrid build(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const std_msgs::msg::Header & output_header) = 0;
};

}  // namespace height_map_ros2
