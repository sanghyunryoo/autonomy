#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "autonomy/sensor/types.hpp"

namespace autonomy {

struct ElevationConfig {
  double hz{50.0};
  double resolution{0.05};
  double x_min{-2.0};
  double x_max{2.0};
  double y_min{-2.0};
  double y_max{2.0};
  double min_z{-2.0};
  double max_z{1.0};
  double maximum_range{2.5};
  std::uint32_t point_stride{1};

  // Cell-wise robust temporal fusion. Values are variances in m² unless
  // otherwise noted.
  double noise_alpha{0.001};
  double minimum_measurement_variance{0.0004};
  double initial_cell_variance{0.01};
  double minimum_cell_variance{0.0004};
  double maximum_cell_variance{0.25};
  double time_variance_rate{0.20};
  double mahalanobis_threshold{3.0};
  double dynamic_environment_variance_bump{0.0225};
  double dynamic_reset_delta{0.10};
  double robust_height_gate{0.04};

  // Per-frame representative-height and edge-aware grid cleanup.
  double intra_cell_min_support_gap{0.025};
  std::uint32_t intra_cell_min_support_count{3};
  double edge_mix_height_difference{0.035};
  std::uint32_t edge_prefer_previous_support_count{2};
  std::uint32_t isolated_radius{1};
  std::uint32_t isolated_min_support_neighbors{2};
  double isolated_support_height_difference{0.025};
  double isolated_outlier_height_difference{0.05};
  std::uint32_t isolated_filter_every_n_frames{2};
  std::uint32_t hole_fill_radius{1};
  std::uint32_t hole_fill_min_neighbors{3};
  double hole_fill_max_height_difference{0.03};
  std::uint32_t bilateral_radius{1};
  double bilateral_sigma_spatial{1.1};
  double bilateral_sigma_height{0.025};
  double bilateral_max_height_difference{0.04};
  std::uint32_t bilateral_passes{2};
  std::uint32_t bilateral_every_n_frames{2};
};

/** Terrain-height grid generated from target-frame merged sensor points. */
struct ElevationMap {
  std::string frame_id{};
  std::uint64_t stamp_nanoseconds{0};
  double resolution{0.05};
  double x_min{-2.0};
  double x_max{2.0};
  double y_min{-2.0};
  double y_max{2.0};
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::vector<float> data{};
};

/**
 * ROS-free, robot-centric elevation grid with robust per-cell temporal fusion.
 * Input points must already be merged and expressed in target_frame.
 */
class ElevationMapper final {
public:
  explicit ElevationMapper(ElevationConfig config);

  [[nodiscard]] ElevationMap build(
    const MergedSensorData & sensor_data,
    std::chrono::steady_clock::time_point now);

private:
  void resetGrid(std::uint32_t width, std::uint32_t height);

  ElevationConfig config_{};
  std::uint32_t width_{0};
  std::uint32_t height_{0};
  std::vector<float> heights_{};
  std::vector<float> variances_{};
  std::chrono::steady_clock::time_point last_updated_at_{};
  std::uint64_t update_sequence_{0};
};

}  // namespace autonomy
