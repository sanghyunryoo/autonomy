#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace autonomy {

/** Transport-neutral sensor payload created by direct sensor drivers. */
using SensorBytes = std::shared_ptr<const std::vector<std::uint8_t>>;

struct SensorPacket {
  SensorBytes bytes{};
  std::string encoding{};
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t row_step{0};
  std::uint32_t point_step{0};
  std::uint64_t stamp_nanoseconds{0};
  std::string frame_id{};

  [[nodiscard]] bool available() const { return bytes != nullptr && !bytes->empty(); }
};

struct Point3f {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
  /** Seconds from the beginning of the LiDAR scan; zero when unavailable. */
  float time_offset_seconds{0.0F};
};

using PointCloud = std::shared_ptr<const std::vector<Point3f>>;

struct PointCloudData {
  SensorPacket raw{};
  PointCloud points{};

  [[nodiscard]] bool available() const { return raw.available() || (points && !points->empty()); }
};

struct ImuData {
  bool valid{false};
  std::array<double, 3> angular_velocity{};
  std::array<double, 3> linear_acceleration{};
  std::uint64_t stamp_nanoseconds{0};
  std::string frame_id{};
};

struct CameraIntrinsics {
  bool valid{false};
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::array<double, 9> matrix{};
  std::uint64_t stamp_nanoseconds{0};
  std::string frame_id{};
};

struct CameraData {
  SensorPacket rgb{};
  SensorPacket depth{};
  PointCloudData points{};
  ImuData imu{};
  CameraIntrinsics intrinsics{};

  [[nodiscard]] bool available() const
  {
    return rgb.available() || depth.available() || points.available();
  }
};

struct LidarData {
  PointCloudData points{};
  ImuData imu{};
  /** Ordered IMU samples retained across the current LiDAR scan interval. */
  std::vector<ImuData> imu_samples{};

  [[nodiscard]] bool available() const { return points.available(); }
};

struct Transform {
  std::string parent_frame{};
  std::string child_frame{};
  std::array<double, 9> rotation{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> translation{0.0, 0.0, 0.0};
};

enum class SensorHealth : std::uint8_t {
  Disabled,
  Idle,
  Ready,
  Stale,
  MissingTransform,
};

[[nodiscard]] const char * sensorHealthName(SensorHealth health);

struct CameraSnapshot {
  std::string id{};
  std::string port{};
  std::string role{};
  std::string frame{};
  CameraData data{};
  std::chrono::milliseconds timeout{500};
  std::chrono::steady_clock::time_point received_at{};
  std::chrono::steady_clock::time_point points_received_at{};
  std::chrono::steady_clock::time_point depth_received_at{};
  SensorHealth health{SensorHealth::Idle};
  Transform base_from_sensor{};
  Transform target_from_sensor{};
};

struct LidarSnapshot {
  std::string id{};
  std::string ip{};
  std::string role{};
  std::string frame{};
  LidarData data{};
  std::chrono::milliseconds timeout{500};
  std::chrono::steady_clock::time_point received_at{};
  std::chrono::steady_clock::time_point points_received_at{};
  std::chrono::steady_clock::time_point imu_received_at{};
  SensorHealth health{SensorHealth::Idle};
  Transform base_from_sensor{};
  Transform target_from_sensor{};
};

/** Points from every ready camera/lidar expressed in one target link. */
struct MergedSensorData {
  std::string target_frame{};
  std::uint64_t stamp_nanoseconds{0};
  std::vector<Point3f> points{};
  std::vector<std::string> source_sensor_ids{};
};

struct SensorSnapshot {
  bool acquisition_enabled{false};
  std::string base_frame{};
  std::string target_frame{};
  std::vector<CameraSnapshot> cameras{};
  std::vector<LidarSnapshot> lidars{};
  MergedSensorData merged{};
  /** Ready LiDAR points only, also expressed in target_frame, for LIO/SLAM. */
  MergedSensorData lidar_merged{};
};

}  // namespace autonomy
