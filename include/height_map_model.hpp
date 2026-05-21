#pragma once

#include <cstdint>
#include <vector>

#include <std_msgs/msg/header.hpp>

#include "elevation_grid.hpp"
#include "autonomy/msg/masked_height_scan.hpp"

namespace autonomy
{

struct HeightMapFrame
{
  std_msgs::msg::Header header;
  GridSpec spec;
  float height_scan_offset{0.0F};
  float base_height{0.0F};
  float fill_value{0.0F};
  std::vector<float> data;
  std::vector<std::uint8_t> valid_mask;
};

struct DdsHeightMap
{
  std::vector<float> data;
};

[[nodiscard]] HeightMapFrame gridToHeightMapFrame(
  const ElevationGrid & grid,
  double height_scan_offset,
  double base_height);

[[nodiscard]] autonomy::msg::MaskedHeightScan toRosMaskedHeightScan(
  const HeightMapFrame & frame);

[[nodiscard]] DdsHeightMap toDdsHeightMap(const HeightMapFrame & frame);

}  // namespace autonomy
