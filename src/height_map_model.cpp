#include "height_map_model.hpp"

#include <cmath>

namespace height_map_ros2
{

HeightMapFrame gridToHeightMapFrame(
  const ElevationGrid & grid,
  const double height_scan_offset,
  const double base_height)
{
  HeightMapFrame frame;
  frame.header = grid.header;
  frame.spec = grid.spec;
  frame.height_scan_offset = static_cast<float>(height_scan_offset);
  frame.base_height = static_cast<float>(base_height);
  frame.fill_value = static_cast<float>(base_height - height_scan_offset);

  const auto count = static_cast<std::size_t>(grid.spec.width()) * grid.spec.height();
  frame.data.resize(count, frame.fill_value);
  frame.valid_mask.resize(count, 0);

  for (std::size_t index = 0; index < count; ++index) {
    const auto z = grid.height[index];
    if (std::isfinite(z)) {
      frame.data[index] = static_cast<float>(-static_cast<double>(z) - height_scan_offset);
      frame.valid_mask[index] = 1;
    }
  }

  return frame;
}

height_map_ros2::msg::MaskedHeightScan toRosMaskedHeightScan(const HeightMapFrame & frame)
{
  height_map_ros2::msg::MaskedHeightScan msg;
  msg.header = frame.header;
  msg.width = frame.spec.width();
  msg.height = frame.spec.height();
  msg.resolution = static_cast<float>(frame.spec.resolution);
  msg.x_min = static_cast<float>(frame.spec.x_min);
  msg.x_max = static_cast<float>(frame.spec.x_max);
  msg.y_min = static_cast<float>(frame.spec.y_min);
  msg.y_max = static_cast<float>(frame.spec.y_max);
  msg.height_scan_offset = frame.height_scan_offset;
  msg.base_height = frame.base_height;
  msg.fill_value = frame.fill_value;
  msg.data = frame.data;
  msg.valid_mask = frame.valid_mask;
  return msg;
}

DdsHeightMap toDdsHeightMap(const HeightMapFrame & frame)
{
  DdsHeightMap msg;
  msg.data = frame.data;
  return msg;
}

}  // namespace height_map_ros2
