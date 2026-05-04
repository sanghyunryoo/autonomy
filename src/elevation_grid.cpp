#include "height_map_ros2/elevation_grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace height_map_ros2
{

std::uint32_t GridSpec::width() const
{
  return static_cast<std::uint32_t>(std::ceil((x_max - x_min) / resolution));
}

std::uint32_t GridSpec::height() const
{
  return static_cast<std::uint32_t>(std::ceil((y_max - y_min) / resolution));
}

ElevationGrid::ElevationGrid(GridSpec grid_spec)
: spec(std::move(grid_spec))
{
  if (spec.resolution <= 0.0 || spec.x_max <= spec.x_min || spec.y_max <= spec.y_min) {
    throw std::invalid_argument("Invalid elevation grid geometry");
  }
  reset();
}

void ElevationGrid::reset(float value)
{
  height.assign(static_cast<std::size_t>(spec.width()) * spec.height(), value);
}

sensor_msgs::msg::Image ElevationGrid::toImageMsg(const std::string & encoding) const
{
  sensor_msgs::msg::Image msg;
  msg.header = header;
  msg.height = spec.height();
  msg.width = spec.width();
  msg.encoding = encoding;
  msg.is_bigendian = false;
  msg.step = msg.width * sizeof(float);
  msg.data.resize(static_cast<std::size_t>(msg.step) * msg.height);

  if (!height.empty()) {
    std::memcpy(msg.data.data(), height.data(), msg.data.size());
  }

  return msg;
}

}  // namespace height_map_ros2
