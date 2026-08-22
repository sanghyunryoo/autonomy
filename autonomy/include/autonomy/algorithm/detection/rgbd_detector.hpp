#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "autonomy/algorithm/detection/types.hpp"
#include "autonomy/sensor/types.hpp"

namespace autonomy {

struct DetectionConfig {
  double hz{30.0};
  std::string camera_id{"ai_realsense"};
  double minimum_depth{0.30};
  double maximum_depth{6.00};
  std::uint32_t sample_stride{4};
  std::uint32_t minimum_valid_samples{64};
};

struct Detection {
  std::string class_name{"foreground"};
  std::string source_camera_id{};
  BoundingBox bbox{};
  float confidence{0.0F};
  TrackingTarget target{};
};

enum class DetectionStatus : std::uint8_t {
  WaitingForData,
  Ready,
  Failed,
};

struct DetectionResult {
  DetectionStatus status{DetectionStatus::WaitingForData};
  std::optional<Detection> detection{};
  std::string message{};
};

/**
 * ROS-free RGB-D inference front-end. It extracts a robust foreground target
 * from aligned depth and camera calibration, then expresses it in target_frame.
 */
class RgbdDetector final {
public:
  explicit RgbdDetector(DetectionConfig config);

  [[nodiscard]] DetectionResult infer(
    const SensorSnapshot & sensors,
    std::chrono::steady_clock::time_point now) const;

private:
  DetectionConfig config_{};
};

}  // namespace autonomy
