#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "autonomy/algorithm/planner/types.hpp"
#include "autonomy/sensor/types.hpp"

namespace autonomy {

struct SlamConfig {
  double hz{10.0};
  std::string map_frame{"map"};
  std::uint32_t minimum_points{64};
  std::uint32_t minimum_imu_samples{50};
  double minimum_range{0.30};
  double maximum_range{30.0};
  double maximum_scan_translation{1.50};
  double maximum_yaw_rate{3.14};
  double scan_voxel_size{0.20};
  double map_voxel_size{0.30};
  std::uint32_t maximum_map_points{200000};
  std::uint32_t registration_iterations{4};
  double keyframe_distance{1.0};
  std::uint32_t loop_min_keyframes{30};
  double loop_descriptor_threshold{0.25};
  double loop_fitness_threshold{0.35};
  double loop_max_correspondence{2.5};
};

enum class SlamStatus : std::uint8_t {
  WaitingForData,
  Ready,
  Failed,
};

/**
 * ESKF linear-velocity estimate expressed in the SLAM map frame.
 *
 * This is an estimate of robot motion, not an autonomy control command.
 */
struct SlamLinearVelocity {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  std::string frame_id{};
  std::chrono::steady_clock::time_point received_at{};
};

struct SlamResult {
  SlamStatus status{SlamStatus::WaitingForData};
  Pose2d pose{};
  SlamLinearVelocity linear_velocity{};
  std::string message{};
};

/**
 * Stateful ROS-free Super-LIO core.
 *
 * Adapted from Super-LIO's ESKF + voxel-map + point-to-plane registration
 * pipeline, but fed directly from SensorManager rather than a ROS node. It
 * owns IMU initialization, scan undistortion prediction, registration, and
 * the map-frame robot pose.
 */
class SuperLioSlam final {
public:
  explicit SuperLioSlam(SlamConfig config);
  ~SuperLioSlam();

  SuperLioSlam(const SuperLioSlam &) = delete;
  SuperLioSlam & operator=(const SuperLioSlam &) = delete;

  [[nodiscard]] SlamResult process(
    const SensorSnapshot & sensors,
    std::chrono::steady_clock::time_point now);
  [[nodiscard]] const Pose2d & pose() const;
  void reset();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace autonomy
