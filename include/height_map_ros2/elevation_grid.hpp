#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace height_map_ros2
{

struct GridSpec
{
  double resolution{0.01};
  double x_min{-0.25};
  double x_max{0.75};
  double y_min{-0.50};
  double y_max{0.50};
  double min_z{-1.0};
  double max_z{1.0};

  [[nodiscard]] std::uint32_t width() const;
  [[nodiscard]] std::uint32_t height() const;
};

struct ElevationGrid
{
  GridSpec spec;
  std_msgs::msg::Header header;
  std::vector<float> height;

  explicit ElevationGrid(GridSpec grid_spec = {});

  void reset(float value = std::numeric_limits<float>::quiet_NaN());
  [[nodiscard]] sensor_msgs::msg::Image toImageMsg(const std::string & encoding = "32FC1") const;
};

}  // namespace height_map_ros2
