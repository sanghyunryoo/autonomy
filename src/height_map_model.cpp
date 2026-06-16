#include "height_map_model.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy
{

HeightMapFrame gridToHeightMapFrame(
  const ElevationGrid & grid,
  const double base_height,
  const double clipping_min,
  const double clipping_max)
{
  const auto clip_min = std::min(clipping_min, clipping_max);
  const auto clip_max = std::max(clipping_min, clipping_max);

  HeightMapFrame frame;
  frame.header = grid.header;
  frame.spec = grid.spec;
  frame.base_height = static_cast<float>(base_height);
  frame.clipping_min = static_cast<float>(clip_min);
  frame.clipping_max = static_cast<float>(clip_max);
  frame.fill_value = static_cast<float>(std::clamp(base_height, clip_min, clip_max));

  const auto count = static_cast<std::size_t>(grid.spec.width()) * grid.spec.height();
  frame.data.resize(count, frame.fill_value);
  frame.valid_mask.resize(count, 0);
  frame.fov_mask.resize(count, 1);

  for (std::size_t index = 0; index < count; ++index) {
    const auto z = grid.height[index];
    if (std::isfinite(z)) {
      const auto distance = std::max(0.0, -static_cast<double>(z));
      frame.data[index] = static_cast<float>(std::clamp(distance, clip_min, clip_max));
      frame.valid_mask[index] = 1;
    }
  }
  
  return frame;
}

autonomy::msg::MaskedHeightScan toRosMaskedHeightScan(const HeightMapFrame & frame)
{
  autonomy::msg::MaskedHeightScan msg;
  msg.header = frame.header;
  msg.width = frame.spec.width();
  msg.height = frame.spec.height();
  msg.resolution = static_cast<float>(frame.spec.resolution);
  msg.x_min = static_cast<float>(frame.spec.x_min);
  msg.x_max = static_cast<float>(frame.spec.x_max);
  msg.y_min = static_cast<float>(frame.spec.y_min);
  msg.y_max = static_cast<float>(frame.spec.y_max);
  msg.height_scan_offset = 0.0F;
  msg.base_height = frame.base_height;
  msg.clipping_min = frame.clipping_min;
  msg.clipping_max = frame.clipping_max;
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

}  // namespace autonomy
