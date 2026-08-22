#pragma once

#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "autonomy/parameter/config.hpp"
#include "autonomy/sensor/types.hpp"
#include "autonomy/sensor/urdf_model.hpp"

namespace autonomy {

class RealSenseDriver;
class LivoxDriver;

/**
 * Owns the physical sensor inventory and its latest transport-neutral data.
 * The manager resolves every sensor both against base_link and the configured
 * target link, then merges ready point clouds in the target frame.
 */
class SensorManager final {
public:
  explicit SensorManager(const RuntimeConfig & config);
  ~SensorManager();

  void setAcquisitionEnabled(bool enabled);
  void updateCameraRgb(const std::string & id, SensorPacket data);
  void updateCameraDepth(const std::string & id, SensorPacket data);
  void updateCameraPoints(const std::string & id, PointCloudData data);
  void updateCameraImu(const std::string & id, ImuData data);
  void updateCameraIntrinsics(const std::string & id, CameraIntrinsics data);
  void updateLidarPoints(const std::string & id, PointCloudData data);
  void updateLidarImu(const std::string & id, ImuData data);
  [[nodiscard]] SensorSnapshot snapshot(std::chrono::steady_clock::time_point now) const;
  [[nodiscard]] std::vector<Transform> sensorTransforms() const;

private:
  struct CameraRecord {
    CameraConfig config{};
    CameraData data{};
    std::chrono::steady_clock::time_point received_at{};
    std::chrono::steady_clock::time_point points_received_at{};
    std::chrono::steady_clock::time_point depth_received_at{};
    std::chrono::steady_clock::time_point imu_received_at{};
    std::optional<Transform> base_from_sensor{};
    std::optional<Transform> target_from_sensor{};
  };

  struct LidarRecord {
    LidarConfig config{};
    LidarData data{};
    std::chrono::steady_clock::time_point received_at{};
    std::chrono::steady_clock::time_point points_received_at{};
    std::chrono::steady_clock::time_point imu_received_at{};
    std::optional<Transform> base_from_sensor{};
    std::optional<Transform> target_from_sensor{};
  };

  [[nodiscard]] SensorHealth healthFor(
    bool enabled,
    bool has_points,
    bool has_transform,
    std::chrono::steady_clock::time_point points_received_at,
    std::chrono::milliseconds timeout,
    std::chrono::steady_clock::time_point now) const;
  void updateReceivedAt(std::chrono::steady_clock::time_point & received_at);
  [[nodiscard]] MergedSensorData merge(const std::vector<CameraSnapshot> & cameras,
    const std::vector<LidarSnapshot> & lidars) const;
  [[nodiscard]] MergedSensorData mergeLidars(const std::vector<LidarSnapshot> & lidars) const;

  mutable std::mutex mutex_;
  bool acquisition_enabled_{false};
  std::string base_frame_{};
  std::string target_frame_{};
  std::unordered_map<std::string, CameraRecord> cameras_{};
  std::unordered_map<std::string, LidarRecord> lidars_{};
  std::unique_ptr<RealSenseDriver> realsense_driver_{};
  std::unique_ptr<LivoxDriver> livox_driver_{};
};

}  // namespace autonomy
