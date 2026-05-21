#pragma once

#include "elevation_map_backend.hpp"

namespace autonomy
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

}  // namespace autonomy
