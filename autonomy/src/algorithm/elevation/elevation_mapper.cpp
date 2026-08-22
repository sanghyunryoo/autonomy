#include "autonomy/algorithm/elevation/elevation_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autonomy {
namespace {

constexpr float kMinimumVariance = 1.0e-6F;

[[nodiscard]] std::size_t cellIndex(
  const std::uint32_t row,
  const std::uint32_t column,
  const std::uint32_t width)
{
  return static_cast<std::size_t>(row) * width + column;
}

[[nodiscard]] bool inBounds(
  const int row,
  const int column,
  const std::uint32_t width,
  const std::uint32_t height)
{
  return row >= 0 && column >= 0 && row < static_cast<int>(height) &&
    column < static_cast<int>(width);
}

void updateCellEstimate(
  const float measurement,
  const float measurement_variance,
  float & height,
  float & variance,
  const ElevationConfig & config)
{
  const float minimum_variance = static_cast<float>(config.minimum_cell_variance);
  const float maximum_variance = static_cast<float>(config.maximum_cell_variance);
  if (!std::isfinite(height)) {
    height = measurement;
    variance = std::clamp(measurement_variance, minimum_variance, maximum_variance);
    return;
  }

  const float delta = measurement - height;
  const float combined_variance = std::max(variance + measurement_variance, kMinimumVariance);
  const float mahalanobis_squared = delta * delta / combined_variance;
  const float threshold_squared = static_cast<float>(
    config.mahalanobis_threshold * config.mahalanobis_threshold);

  if (mahalanobis_squared > threshold_squared &&
    std::fabs(delta) > static_cast<float>(config.dynamic_reset_delta)) {
    const float blend = measurement < height ? 0.8F : 0.5F;
    height = blend * measurement + (1.0F - blend) * height;
    variance = std::clamp(
      variance + static_cast<float>(config.dynamic_environment_variance_bump),
      minimum_variance, maximum_variance);
    return;
  }

  const float kalman_gain = std::clamp(variance / combined_variance, 0.2F, 0.8F);
  height += kalman_gain * delta;
  variance = std::clamp((1.0F - kalman_gain) * variance, minimum_variance, maximum_variance);
}

void fillSmallHolesEdgeAware(
  std::vector<float> & heights,
  std::vector<float> & variances,
  const std::uint32_t width,
  const std::uint32_t height,
  const ElevationConfig & config)
{
  if (config.hole_fill_radius == 0U) return;
  const int radius = static_cast<int>(config.hole_fill_radius);
  std::vector<float> filled_heights = heights;
  std::vector<float> filled_variances = variances;
  const float maximum_difference = static_cast<float>(config.hole_fill_max_height_difference);

  for (std::uint32_t row = 0U; row < height; ++row) {
    for (std::uint32_t column = 0U; column < width; ++column) {
      const std::size_t index = cellIndex(row, column, width);
      if (std::isfinite(heights[index])) continue;

      float weighted_sum = 0.0F;
      float weight_sum = 0.0F;
      float variance_sum = 0.0F;
      float minimum = std::numeric_limits<float>::infinity();
      float maximum = -std::numeric_limits<float>::infinity();
      std::uint32_t count = 0U;
      for (int row_offset = -radius; row_offset <= radius; ++row_offset) {
        for (int column_offset = -radius; column_offset <= radius; ++column_offset) {
          if (row_offset == 0 && column_offset == 0) continue;
          const int neighbor_row = static_cast<int>(row) + row_offset;
          const int neighbor_column = static_cast<int>(column) + column_offset;
          if (!inBounds(neighbor_row, neighbor_column, width, height)) continue;
          const float value = heights[cellIndex(
            static_cast<std::uint32_t>(neighbor_row), static_cast<std::uint32_t>(neighbor_column), width)];
          if (!std::isfinite(value)) continue;
          const float distance_squared = static_cast<float>(
            row_offset * row_offset + column_offset * column_offset);
          const float weight = 1.0F / (1.0F + distance_squared);
          weighted_sum += weight * value;
          weight_sum += weight;
          variance_sum += variances[cellIndex(
            static_cast<std::uint32_t>(neighbor_row), static_cast<std::uint32_t>(neighbor_column), width)];
          minimum = std::min(minimum, value);
          maximum = std::max(maximum, value);
          ++count;
        }
      }
      if (count < config.hole_fill_min_neighbors || weight_sum <= kMinimumVariance ||
        maximum - minimum > maximum_difference) {
        continue;
      }
      filled_heights[index] = weighted_sum / weight_sum;
      filled_variances[index] = std::min(
        static_cast<float>(config.maximum_cell_variance),
        variance_sum / static_cast<float>(count) + maximum_difference * maximum_difference);
    }
  }
  heights.swap(filled_heights);
  variances.swap(filled_variances);
}

void removeIsolatedCellsEdgeAware(
  std::vector<float> & heights,
  std::vector<float> & variances,
  const std::uint32_t width,
  const std::uint32_t height,
  const ElevationConfig & config)
{
  if (config.isolated_radius == 0U) return;
  const int radius = static_cast<int>(config.isolated_radius);
  const float support_difference = static_cast<float>(config.isolated_support_height_difference);
  const float outlier_difference = static_cast<float>(config.isolated_outlier_height_difference);
  std::vector<float> filtered_heights = heights;
  std::vector<float> filtered_variances = variances;
  std::vector<float> neighbor_heights;
  neighbor_heights.reserve(static_cast<std::size_t>((2 * radius + 1) * (2 * radius + 1)));

  for (std::uint32_t row = 0U; row < height; ++row) {
    for (std::uint32_t column = 0U; column < width; ++column) {
      const std::size_t index = cellIndex(row, column, width);
      const float center = heights[index];
      if (!std::isfinite(center)) continue;

      std::uint32_t support_count = 0U;
      neighbor_heights.clear();
      for (int row_offset = -radius; row_offset <= radius; ++row_offset) {
        for (int column_offset = -radius; column_offset <= radius; ++column_offset) {
          if (row_offset == 0 && column_offset == 0) continue;
          const int neighbor_row = static_cast<int>(row) + row_offset;
          const int neighbor_column = static_cast<int>(column) + column_offset;
          if (!inBounds(neighbor_row, neighbor_column, width, height)) continue;
          const float neighbor = heights[cellIndex(
            static_cast<std::uint32_t>(neighbor_row), static_cast<std::uint32_t>(neighbor_column), width)];
          if (!std::isfinite(neighbor)) continue;
          neighbor_heights.push_back(neighbor);
          if (std::fabs(neighbor - center) <= support_difference) ++support_count;
        }
      }
      if (neighbor_heights.size() < config.isolated_min_support_neighbors ||
        support_count >= config.isolated_min_support_neighbors) {
        continue;
      }
      const auto median = neighbor_heights.begin() + neighbor_heights.size() / 2U;
      std::nth_element(neighbor_heights.begin(), median, neighbor_heights.end());
      if (std::fabs(center - *median) > outlier_difference) {
        filtered_heights[index] = std::numeric_limits<float>::quiet_NaN();
        filtered_variances[index] = static_cast<float>(config.initial_cell_variance);
      }
    }
  }
  heights.swap(filtered_heights);
  variances.swap(filtered_variances);
}

void bilateralFillMissing(
  std::vector<float> & heights,
  const std::uint32_t width,
  const std::uint32_t height,
  const ElevationConfig & config)
{
  if (config.bilateral_radius == 0U || config.bilateral_passes == 0U) return;
  const int radius = static_cast<int>(config.bilateral_radius);
  const float maximum_difference = static_cast<float>(config.bilateral_max_height_difference);
  const float inverse_spatial = 1.0F / static_cast<float>(
    2.0 * config.bilateral_sigma_spatial * config.bilateral_sigma_spatial);
  const float inverse_height = 1.0F / static_cast<float>(
    2.0 * config.bilateral_sigma_height * config.bilateral_sigma_height);
  std::vector<float> source = heights;
  std::vector<float> destination = heights;
  std::vector<float> neighbors;
  neighbors.reserve(static_cast<std::size_t>((2 * radius + 1) * (2 * radius + 1)));

  for (std::uint32_t pass = 0U; pass < config.bilateral_passes; ++pass) {
    destination = source;
    for (std::uint32_t row = 0U; row < height; ++row) {
      for (std::uint32_t column = 0U; column < width; ++column) {
        const std::size_t index = cellIndex(row, column, width);
        if (std::isfinite(source[index])) continue;
        neighbors.clear();
        float minimum = std::numeric_limits<float>::infinity();
        float maximum = -std::numeric_limits<float>::infinity();
        for (int row_offset = -radius; row_offset <= radius; ++row_offset) {
          for (int column_offset = -radius; column_offset <= radius; ++column_offset) {
            if (row_offset == 0 && column_offset == 0) continue;
            const int neighbor_row = static_cast<int>(row) + row_offset;
            const int neighbor_column = static_cast<int>(column) + column_offset;
            if (!inBounds(neighbor_row, neighbor_column, width, height)) continue;
            const float neighbor = source[cellIndex(
              static_cast<std::uint32_t>(neighbor_row), static_cast<std::uint32_t>(neighbor_column), width)];
            if (!std::isfinite(neighbor)) continue;
            neighbors.push_back(neighbor);
            minimum = std::min(minimum, neighbor);
            maximum = std::max(maximum, neighbor);
          }
        }
        if (neighbors.empty() || maximum - minimum > maximum_difference) continue;
        const auto median = neighbors.begin() + neighbors.size() / 2U;
        std::nth_element(neighbors.begin(), median, neighbors.end());
        const float reference = *median;
        float weighted_sum = 0.0F;
        float weight_sum = 0.0F;
        for (int row_offset = -radius; row_offset <= radius; ++row_offset) {
          for (int column_offset = -radius; column_offset <= radius; ++column_offset) {
            if (row_offset == 0 && column_offset == 0) continue;
            const int neighbor_row = static_cast<int>(row) + row_offset;
            const int neighbor_column = static_cast<int>(column) + column_offset;
            if (!inBounds(neighbor_row, neighbor_column, width, height)) continue;
            const float neighbor = source[cellIndex(
              static_cast<std::uint32_t>(neighbor_row), static_cast<std::uint32_t>(neighbor_column), width)];
            if (!std::isfinite(neighbor)) continue;
            const float height_delta = neighbor - reference;
            if (std::fabs(height_delta) > maximum_difference) continue;
            const float spatial_distance_squared = static_cast<float>(
              row_offset * row_offset + column_offset * column_offset);
            const float weight = std::exp(-spatial_distance_squared * inverse_spatial) *
              std::exp(-height_delta * height_delta * inverse_height);
            weighted_sum += weight * neighbor;
            weight_sum += weight;
          }
        }
        if (weight_sum > kMinimumVariance) destination[index] = weighted_sum / weight_sum;
      }
    }
    source.swap(destination);
  }
  heights.swap(source);
}

void fillRemainingHoles(
  std::vector<float> & heights,
  std::vector<float> & variances,
  const std::uint32_t width,
  const std::uint32_t height,
  const float initial_variance)
{
  std::vector<float> source = heights;
  std::vector<float> destination = heights;
  std::vector<float> source_variances = variances;
  std::vector<float> destination_variances = variances;
  const std::uint32_t maximum_iterations = std::max(width, height);
  for (std::uint32_t iteration = 0U; iteration < maximum_iterations; ++iteration) {
    bool filled_any = false;
    destination = source;
    destination_variances = source_variances;
    for (std::uint32_t row = 0U; row < height; ++row) {
      for (std::uint32_t column = 0U; column < width; ++column) {
        const std::size_t index = cellIndex(row, column, width);
        if (std::isfinite(source[index])) continue;
        float weighted_sum = 0.0F;
        float weight_sum = 0.0F;
        float variance_sum = 0.0F;
        std::uint32_t count = 0U;
        for (int row_offset = -1; row_offset <= 1; ++row_offset) {
          for (int column_offset = -1; column_offset <= 1; ++column_offset) {
            if (row_offset == 0 && column_offset == 0) continue;
            const int neighbor_row = static_cast<int>(row) + row_offset;
            const int neighbor_column = static_cast<int>(column) + column_offset;
            if (!inBounds(neighbor_row, neighbor_column, width, height)) continue;
            const std::size_t neighbor_index = cellIndex(
              static_cast<std::uint32_t>(neighbor_row), static_cast<std::uint32_t>(neighbor_column), width);
            const float neighbor = source[neighbor_index];
            if (!std::isfinite(neighbor)) continue;
            const float distance_squared = static_cast<float>(
              row_offset * row_offset + column_offset * column_offset);
            const float weight = 1.0F / (1.0F + distance_squared);
            weighted_sum += weight * neighbor;
            weight_sum += weight;
            variance_sum += source_variances[neighbor_index];
            ++count;
          }
        }
        if (weight_sum > kMinimumVariance) {
          destination[index] = weighted_sum / weight_sum;
          destination_variances[index] = count > 0U ? variance_sum / static_cast<float>(count) : initial_variance;
          filled_any = true;
        }
      }
    }
    source.swap(destination);
    source_variances.swap(destination_variances);
    if (!filled_any) break;
  }

  std::vector<float> finite_heights;
  finite_heights.reserve(source.size());
  for (const float value : source) {
    if (std::isfinite(value)) finite_heights.push_back(value);
  }
  float fallback = 0.0F;
  if (!finite_heights.empty()) {
    const auto median = finite_heights.begin() + finite_heights.size() / 2U;
    std::nth_element(finite_heights.begin(), median, finite_heights.end());
    fallback = *median;
  }
  for (std::size_t index = 0U; index < source.size(); ++index) {
    if (!std::isfinite(source[index])) {
      source[index] = fallback;
      source_variances[index] = initial_variance;
    }
  }
  heights.swap(source);
  variances.swap(source_variances);
}

}  // namespace

ElevationMapper::ElevationMapper(ElevationConfig config)
: config_(std::move(config))
{
  if (!std::isfinite(config_.resolution) || config_.resolution <= 0.0 ||
    config_.x_max <= config_.x_min || config_.y_max <= config_.y_min ||
    config_.max_z <= config_.min_z || config_.maximum_range <= 0.0 ||
    config_.point_stride == 0U || config_.minimum_measurement_variance <= 0.0 ||
    config_.minimum_cell_variance <= 0.0 ||
    config_.maximum_cell_variance < config_.minimum_cell_variance ||
    config_.initial_cell_variance < config_.minimum_cell_variance ||
    config_.initial_cell_variance > config_.maximum_cell_variance ||
    config_.mahalanobis_threshold <= 0.0) {
    throw std::invalid_argument("elevation mapping configuration is invalid");
  }
}

ElevationMap ElevationMapper::build(
  const MergedSensorData & sensor_data,
  const std::chrono::steady_clock::time_point now)
{
  const double x_extent = config_.x_max - config_.x_min;
  const double y_extent = config_.y_max - config_.y_min;
  const std::uint32_t width = static_cast<std::uint32_t>(std::ceil(x_extent / config_.resolution));
  const std::uint32_t height = static_cast<std::uint32_t>(std::ceil(y_extent / config_.resolution));
  if (width == 0U || height == 0U) {
    throw std::runtime_error("elevation grid dimensions are invalid");
  }
  if (width != width_ || height != height_) resetGrid(width, height);

  if (last_updated_at_ != std::chrono::steady_clock::time_point{} && now >= last_updated_at_) {
    const float elapsed_seconds = std::chrono::duration<float>(now - last_updated_at_).count();
    const float variance_increment = static_cast<float>(config_.time_variance_rate) * elapsed_seconds;
    if (variance_increment > 0.0F) {
      for (std::size_t index = 0U; index < heights_.size(); ++index) {
        if (std::isfinite(heights_[index])) {
          variances_[index] = std::clamp(
            variances_[index] + variance_increment,
            static_cast<float>(config_.minimum_cell_variance),
            static_cast<float>(config_.maximum_cell_variance));
        }
      }
    }
  }

  const std::size_t cells = heights_.size();
  std::vector<std::uint32_t> hit_count(cells, 0U);
  std::vector<float> weighted_height_sum(cells, 0.0F);
  std::vector<float> weight_sum(cells, 0.0F);
  std::vector<float> frame_min(cells, std::numeric_limits<float>::infinity());
  std::vector<float> frame_max(cells, -std::numeric_limits<float>::infinity());
  std::vector<float> minimum_cluster_sum(cells, 0.0F);
  std::vector<float> minimum_cluster_variance_sum(cells, 0.0F);
  std::vector<std::uint32_t> minimum_cluster_count(cells, 0U);
  const float maximum_range_squared = static_cast<float>(config_.maximum_range * config_.maximum_range);
  const float resolution = static_cast<float>(config_.resolution);
  const float x_min = static_cast<float>(config_.x_min);
  const float y_min = static_cast<float>(config_.y_min);
  const float robust_gate = static_cast<float>(config_.robust_height_gate);
  const float support_gap = static_cast<float>(config_.intra_cell_min_support_gap);

  for (std::size_t point_index = 0U; point_index < sensor_data.points.size(); point_index += config_.point_stride) {
    const Point3f & point = sensor_data.points[point_index];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
      point.x < config_.x_min || point.x >= config_.x_max ||
      point.y < config_.y_min || point.y >= config_.y_max ||
      point.z < config_.min_z || point.z > config_.max_z) {
      continue;
    }
    const float range_squared = point.x * point.x + point.y * point.y + point.z * point.z;
    if (range_squared <= kMinimumVariance || range_squared > maximum_range_squared) continue;
    const std::uint32_t column = static_cast<std::uint32_t>((point.x - x_min) / resolution);
    const std::uint32_t row = static_cast<std::uint32_t>((point.y - y_min) / resolution);
    if (column >= width || row >= height) continue;
    const std::size_t index = cellIndex(row, column, width);
    const float measurement = point.z;
    const float previous_minimum = frame_min[index];
    if (measurement < frame_min[index]) frame_min[index] = measurement;
    if (measurement > frame_max[index]) frame_max[index] = measurement;
    ++hit_count[index];

    const float measurement_variance = std::max(
      static_cast<float>(config_.minimum_measurement_variance),
      static_cast<float>(config_.noise_alpha) * range_squared);
    float robust_weight = 1.0F / std::max(measurement_variance, kMinimumVariance);
    if (std::isfinite(heights_[index])) {
      const float normalized_difference = (measurement - heights_[index]) / robust_gate;
      robust_weight *= 1.0F / (1.0F + normalized_difference * normalized_difference);
    }
    weighted_height_sum[index] += robust_weight * measurement;
    weight_sum[index] += robust_weight;

    if (measurement + support_gap < previous_minimum) {
      minimum_cluster_sum[index] = measurement;
      minimum_cluster_variance_sum[index] = measurement_variance;
      minimum_cluster_count[index] = 1U;
    } else if (std::fabs(measurement - frame_min[index]) <= support_gap) {
      minimum_cluster_sum[index] += measurement;
      minimum_cluster_variance_sum[index] += measurement_variance;
      ++minimum_cluster_count[index];
    }
  }

  for (std::size_t index = 0U; index < cells; ++index) {
    if (hit_count[index] == 0U || weight_sum[index] <= kMinimumVariance) continue;
    float measurement = weighted_height_sum[index] / weight_sum[index];
    float measurement_variance = std::max(
      static_cast<float>(config_.minimum_measurement_variance), 1.0F / weight_sum[index]);
    const bool edge_mixed = std::isfinite(frame_max[index] - frame_min[index]) &&
      frame_max[index] - frame_min[index] > static_cast<float>(config_.edge_mix_height_difference);
    const std::uint32_t support_count = minimum_cluster_count[index];
    if (support_count >= config_.intra_cell_min_support_count) {
      const float supported_minimum = minimum_cluster_sum[index] / static_cast<float>(support_count);
      const float supported_variance = std::max(
        static_cast<float>(config_.minimum_measurement_variance),
        minimum_cluster_variance_sum[index] / static_cast<float>(support_count));
      if (!std::isfinite(heights_[index]) ||
        (edge_mixed && support_count >= config_.edge_prefer_previous_support_count) ||
        supported_minimum < heights_[index] ||
        std::fabs(supported_minimum - heights_[index]) < robust_gate) {
        measurement = supported_minimum;
        measurement_variance = supported_variance;
      }
    }
    updateCellEstimate(measurement, measurement_variance, heights_[index], variances_[index], config_);
  }

  fillSmallHolesEdgeAware(heights_, variances_, width_, height_, config_);
  ++update_sequence_;
  if (update_sequence_ % config_.isolated_filter_every_n_frames == 0U) {
    removeIsolatedCellsEdgeAware(heights_, variances_, width_, height_, config_);
  }
  if (update_sequence_ % config_.bilateral_every_n_frames == 0U) {
    bilateralFillMissing(heights_, width_, height_, config_);
  }
  fillRemainingHoles(
    heights_, variances_, width_, height_, static_cast<float>(config_.initial_cell_variance));
  last_updated_at_ = now;

  ElevationMap map;
  map.frame_id = sensor_data.target_frame;
  map.stamp_nanoseconds = sensor_data.stamp_nanoseconds;
  map.resolution = config_.resolution;
  map.x_min = config_.x_min;
  map.x_max = config_.x_max;
  map.y_min = config_.y_min;
  map.y_max = config_.y_max;
  map.width = width_;
  map.height = height_;
  map.data = heights_;
  return map;
}

void ElevationMapper::resetGrid(const std::uint32_t width, const std::uint32_t height)
{
  width_ = width;
  height_ = height;
  const std::size_t cells = static_cast<std::size_t>(width_) * height_;
  heights_.assign(cells, std::numeric_limits<float>::quiet_NaN());
  variances_.assign(cells, static_cast<float>(config_.initial_cell_variance));
  last_updated_at_ = {};
  update_sequence_ = 0U;
}

}  // namespace autonomy
