#pragma once

#include <chrono>
#include <cstdint>

namespace autonomy {

/** Position target produced by an internal detection model. */
struct TrackingTarget {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  std::chrono::steady_clock::time_point received_at{};
};

struct BoundingBox {
  std::uint32_t x{0};
  std::uint32_t y{0};
  std::uint32_t width{0};
  std::uint32_t height{0};
};

}  // namespace autonomy
