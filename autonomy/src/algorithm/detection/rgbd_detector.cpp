#include "autonomy/algorithm/detection/rgbd_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autonomy {
namespace {

bool isDepth16(const std::string & encoding)
{
  return encoding == "16UC1" || encoding == "mono16";
}

bool isDepth32(const std::string & encoding)
{
  return encoding == "32FC1";
}

double depthAt(const SensorPacket & packet, const std::uint32_t row, const std::uint32_t column)
{
  const std::size_t bytes_per_pixel = isDepth16(packet.encoding) ? sizeof(std::uint16_t) : sizeof(float);
  const std::size_t offset = static_cast<std::size_t>(row) * packet.row_step +
    static_cast<std::size_t>(column) * bytes_per_pixel;
  if (!packet.bytes || offset + bytes_per_pixel > packet.bytes->size()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (isDepth16(packet.encoding)) {
    std::uint16_t millimetres = 0U;
    std::memcpy(&millimetres, packet.bytes->data() + offset, sizeof(millimetres));
    return static_cast<double>(millimetres) * 0.001;
  }
  float metres = 0.0F;
  std::memcpy(&metres, packet.bytes->data() + offset, sizeof(metres));
  return static_cast<double>(metres);
}

std::array<double, 3> transformPoint(const Transform & transform, const std::array<double, 3> & point)
{
  std::array<double, 3> result{};
  for (std::size_t row = 0; row < 3U; ++row) {
    result[row] = transform.translation[row];
    for (std::size_t column = 0; column < 3U; ++column) {
      result[row] += transform.rotation[row * 3U + column] * point[column];
    }
  }
  return result;
}

bool fresh(
  const std::chrono::steady_clock::time_point received_at,
  const std::chrono::milliseconds timeout,
  const std::chrono::steady_clock::time_point now)
{
  return received_at != std::chrono::steady_clock::time_point{} && now >= received_at &&
    now - received_at <= timeout;
}

}  // namespace

RgbdDetector::RgbdDetector(DetectionConfig config)
: config_(std::move(config))
{
  if (config_.camera_id.empty() || config_.minimum_depth <= 0.0 ||
    config_.maximum_depth <= config_.minimum_depth ||
    config_.sample_stride == 0U || config_.minimum_valid_samples == 0U) {
    throw std::invalid_argument("RGB-D detection configuration is invalid");
  }
}

DetectionResult RgbdDetector::infer(
  const SensorSnapshot & sensors,
  const std::chrono::steady_clock::time_point now) const
{
  DetectionResult result;
  std::optional<Detection> nearest;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (const CameraSnapshot & camera : sensors.cameras) {
    if (camera.id != config_.camera_id) continue;
    const SensorPacket & depth = camera.data.depth;
    const CameraIntrinsics & intrinsics = camera.data.intrinsics;
    if (!depth.available() || !intrinsics.valid ||
      !fresh(camera.depth_received_at, camera.timeout, now)) {
      continue;
    }
    if ((!isDepth16(depth.encoding) && !isDepth32(depth.encoding)) || depth.width == 0U || depth.height == 0U ||
      depth.row_step == 0U) {
      result.status = DetectionStatus::Failed;
      result.message = "RGB-D detector received malformed depth input from " + camera.id;
      return result;
    }
    const double fx = intrinsics.matrix[0];
    const double fy = intrinsics.matrix[4];
    const double cx = intrinsics.matrix[2];
    const double cy = intrinsics.matrix[5];
    if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0.0 || fy <= 0.0) {
      result.status = DetectionStatus::Failed;
      result.message = "RGB-D detector received invalid intrinsics from " + camera.id;
      return result;
    }
    const std::uint32_t row_begin = depth.height / 4U;
    const std::uint32_t row_end = depth.height - row_begin;
    const std::uint32_t column_begin = depth.width / 4U;
    const std::uint32_t column_end = depth.width - column_begin;
    std::vector<double> candidates;
    std::uint32_t minimum_row = depth.height;
    std::uint32_t maximum_row = 0U;
    std::uint32_t minimum_column = depth.width;
    std::uint32_t maximum_column = 0U;
    for (std::uint32_t row = row_begin; row < row_end; row += config_.sample_stride) {
      for (std::uint32_t column = column_begin; column < column_end; column += config_.sample_stride) {
        const double value = depthAt(depth, row, column);
        if (std::isfinite(value) && value >= config_.minimum_depth && value <= config_.maximum_depth) {
          candidates.push_back(value);
          minimum_row = std::min(minimum_row, row);
          maximum_row = std::max(maximum_row, row);
          minimum_column = std::min(minimum_column, column);
          maximum_column = std::max(maximum_column, column);
        }
      }
    }
    if (candidates.size() < config_.minimum_valid_samples) continue;
    const std::size_t median_index = candidates.size() / 2U;
    std::nth_element(candidates.begin(), candidates.begin() + median_index, candidates.end());
    const double distance = candidates[median_index];
    const std::array<double, 3> camera_point{
      (static_cast<double>(depth.width) * 0.5 - cx) * distance / fx,
      (static_cast<double>(depth.height) * 0.5 - cy) * distance / fy,
      distance};
    const std::array<double, 3> target_point = transformPoint(camera.target_from_sensor, camera_point);
    Detection candidate;
    candidate.source_camera_id = camera.id;
    const SensorPacket & rgb = camera.data.rgb;
    const std::uint32_t bbox_width = rgb.available() && rgb.width > 0U ? rgb.width : depth.width;
    const std::uint32_t bbox_height = rgb.available() && rgb.height > 0U ? rgb.height : depth.height;
    const double x_scale = static_cast<double>(bbox_width) / static_cast<double>(depth.width);
    const double y_scale = static_cast<double>(bbox_height) / static_cast<double>(depth.height);
    candidate.bbox.x = static_cast<std::uint32_t>(minimum_column * x_scale);
    candidate.bbox.y = static_cast<std::uint32_t>(minimum_row * y_scale);
    const std::uint32_t right = std::min(
      bbox_width, static_cast<std::uint32_t>((maximum_column + config_.sample_stride) * x_scale));
    const std::uint32_t bottom = std::min(
      bbox_height, static_cast<std::uint32_t>((maximum_row + config_.sample_stride) * y_scale));
    candidate.bbox.width = right > candidate.bbox.x ? right - candidate.bbox.x : 1U;
    candidate.bbox.height = bottom > candidate.bbox.y ? bottom - candidate.bbox.y : 1U;
    candidate.confidence = std::min(1.0F,
      static_cast<float>(candidates.size()) / static_cast<float>(config_.minimum_valid_samples * 4U));
    candidate.target.x = static_cast<float>(target_point[0]);
    candidate.target.y = static_cast<float>(target_point[1]);
    candidate.target.z = static_cast<float>(target_point[2]);
    candidate.target.received_at = now;
    if (!nearest || distance < nearest_distance) {
      nearest_distance = distance;
      nearest = std::move(candidate);
    }
  }
  if (!nearest) {
    result.message = "waiting for calibrated RGB-D depth samples from " + config_.camera_id;
    return result;
  }
  result.status = DetectionStatus::Ready;
  result.detection = std::move(nearest);
  result.message = "RGB-D foreground detection updated";
  return result;
}

}  // namespace autonomy
