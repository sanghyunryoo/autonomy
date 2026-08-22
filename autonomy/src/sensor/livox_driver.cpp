#include "autonomy/sensor/livox_driver.hpp"

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <livox_lidar_api.h>

#include "autonomy/sensor/sensor_manager.hpp"
#include "autonomy/sensor/types.hpp"

namespace autonomy {
namespace {

std::uint64_t nowNanoseconds()
{
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count());
}

std::uint64_t packetTimestamp(const LivoxLidarEthernetPacket & packet)
{
  std::uint64_t timestamp = 0U;
  std::memcpy(&timestamp, packet.timestamp, sizeof(timestamp));
  return timestamp == 0U ? nowNanoseconds() : timestamp;
}

}  // namespace

struct LivoxDriver::Impl {
  Impl(
    const NetworkConfig & network,
    const std::vector<LidarConfig> & configured_lidars,
    SensorManager & manager)
  : sensor_manager(manager)
  {
    for (const LidarConfig & lidar : configured_lidars) {
      if (!lidar.enabled) continue;
      lidars_by_ip.emplace(lidar.ip, lidar);
      if (sdk_config_path.empty()) {
        sdk_config_path = lidar.sdk_config_path;
      } else if (sdk_config_path != lidar.sdk_config_path) {
        throw std::invalid_argument(
                "all enabled Livox sensors must be described by the same SDK JSON configuration");
      }
    }
    if (lidars_by_ip.empty()) return;

    // The JSON file owns all command, point, and IMU UDP port assignment.
    if (!LivoxLidarSdkInit(sdk_config_path.c_str(), network.lidar_local_ip.c_str())) {
      throw std::runtime_error("Livox SDK2 initialization failed: " + sdk_config_path);
    }
    initialized = true;
    SetLivoxLidarPointCloudCallBack(&Impl::onPointCloud, this);
    SetLivoxLidarImuDataCallback(&Impl::onImu, this);
    SetLivoxLidarInfoChangeCallback(&Impl::onInfoChange, this);
    if (!LivoxLidarSdkStart()) {
      shutdown();
      throw std::runtime_error("Livox SDK2 start failed");
    }
    started = true;
  }

  ~Impl()
  {
    shutdown();
  }

  void shutdown()
  {
    accepting.store(false, std::memory_order_release);
    if (!initialized) return;
    SetLivoxLidarPointCloudCallBack(nullptr, nullptr);
    SetLivoxLidarImuDataCallback(nullptr, nullptr);
    SetLivoxLidarInfoChangeCallback(nullptr, nullptr);
    LivoxLidarSdkUninit();
    started = false;
    initialized = false;
  }

  void setAcquisitionEnabled(const bool enabled)
  {
    accepting.store(enabled, std::memory_order_release);
    std::vector<std::uint32_t> handles;
    {
      std::lock_guard<std::mutex> lock(mutex);
      for (const auto & item : sensor_id_by_handle) handles.push_back(item.first);
    }
    for (const std::uint32_t handle : handles) {
      if (enabled) {
        EnableLivoxLidarPointSend(handle, nullptr, nullptr);
        EnableLivoxLidarImuData(handle, nullptr, nullptr);
      } else {
        DisableLivoxLidarPointSend(handle, nullptr, nullptr);
        DisableLivoxLidarImuData(handle, nullptr, nullptr);
      }
    }
  }

  [[nodiscard]] std::optional<LidarConfig> lidarFor(const std::uint32_t handle) const
  {
    std::lock_guard<std::mutex> lock(mutex);
    const auto id = sensor_id_by_handle.find(handle);
    if (id != sensor_id_by_handle.end()) {
      const auto lidar = lidar_by_id.find(id->second);
      if (lidar != lidar_by_id.end()) return lidar->second;
    }
    if (lidars_by_ip.size() == 1U) return lidars_by_ip.begin()->second;
    return std::nullopt;
  }

  void handleInfoChange(const std::uint32_t handle, const LivoxLidarInfo & info)
  {
    const std::string ip(info.lidar_ip);
    {
      std::lock_guard<std::mutex> lock(mutex);
      const auto configured = lidars_by_ip.find(ip);
      if (configured == lidars_by_ip.end()) {
        std::cerr << "[LivoxDriver][WARN] ignoring SDK device at unconfigured IP " << ip << '\n';
        return;
      }
      sensor_id_by_handle[handle] = configured->second.id;
      lidar_by_id[configured->second.id] = configured->second;
    }
    if (accepting.load(std::memory_order_acquire)) {
      EnableLivoxLidarPointSend(handle, nullptr, nullptr);
      EnableLivoxLidarImuData(handle, nullptr, nullptr);
    }
  }

  void handlePointCloud(const std::uint32_t handle, const LivoxLidarEthernetPacket & packet)
  {
    if (!accepting.load(std::memory_order_acquire)) return;
    const std::optional<LidarConfig> lidar = lidarFor(handle);
    if (!lidar || packet.data_type != kLivoxLidarCartesianCoordinateHighData || packet.dot_num == 0U) {
      return;
    }

    const auto * source = reinterpret_cast<const LivoxLidarCartesianHighRawPoint *>(packet.data);
    auto points = std::make_shared<std::vector<Point3f>>();
    points->reserve(packet.dot_num);
    const float point_interval_seconds = static_cast<float>(packet.time_interval) * 1.0e-7F;
    for (std::uint16_t index = 0U; index < packet.dot_num; ++index) {
      const LivoxLidarCartesianHighRawPoint & raw = source[index];
      Point3f point;
      point.x = static_cast<float>(raw.x) * 0.001F;
      point.y = static_cast<float>(raw.y) * 0.001F;
      point.z = static_cast<float>(raw.z) * 0.001F;
      point.intensity = static_cast<float>(raw.reflectivity);
      point.time_offset_seconds = static_cast<float>(index) * point_interval_seconds;
      points->push_back(point);
    }

    PointCloudData cloud;
    cloud.raw.encoding = "livox_sdk2/cartesian_high";
    cloud.raw.width = packet.dot_num;
    cloud.raw.height = 1U;
    cloud.raw.point_step = sizeof(LivoxLidarCartesianHighRawPoint);
    cloud.raw.row_step = cloud.raw.width * cloud.raw.point_step;
    cloud.raw.stamp_nanoseconds = packetTimestamp(packet);
    cloud.raw.frame_id = lidar->frame;
    cloud.points = std::move(points);
    sensor_manager.updateLidarPoints(lidar->id, std::move(cloud));
  }

  void handleImu(const std::uint32_t handle, const LivoxLidarEthernetPacket & packet)
  {
    if (!accepting.load(std::memory_order_acquire)) return;
    const std::optional<LidarConfig> lidar = lidarFor(handle);
    if (!lidar || packet.data_type != kLivoxLidarImuData) return;
    const auto * raw = reinterpret_cast<const LivoxLidarImuRawPoint *>(packet.data);
    ImuData imu;
    imu.valid = true;
    imu.angular_velocity = {raw->gyro_x, raw->gyro_y, raw->gyro_z};
    imu.linear_acceleration = {raw->acc_x, raw->acc_y, raw->acc_z};
    imu.stamp_nanoseconds = packetTimestamp(packet);
    imu.frame_id = lidar->frame;
    sensor_manager.updateLidarImu(lidar->id, std::move(imu));
  }

  static void onPointCloud(
    const std::uint32_t handle,
    const std::uint8_t,
    LivoxLidarEthernetPacket * packet,
    void * client_data)
  {
    if (packet != nullptr && client_data != nullptr) {
      static_cast<Impl *>(client_data)->handlePointCloud(handle, *packet);
    }
  }

  static void onImu(
    const std::uint32_t handle,
    const std::uint8_t,
    LivoxLidarEthernetPacket * packet,
    void * client_data)
  {
    if (packet != nullptr && client_data != nullptr) {
      static_cast<Impl *>(client_data)->handleImu(handle, *packet);
    }
  }

  static void onInfoChange(
    const std::uint32_t handle,
    const LivoxLidarInfo * info,
    void * client_data)
  {
    if (info != nullptr && client_data != nullptr) {
      static_cast<Impl *>(client_data)->handleInfoChange(handle, *info);
    }
  }

  SensorManager & sensor_manager;
  std::string sdk_config_path{};
  std::unordered_map<std::string, LidarConfig> lidars_by_ip{};
  mutable std::mutex mutex{};
  std::unordered_map<std::uint32_t, std::string> sensor_id_by_handle{};
  std::unordered_map<std::string, LidarConfig> lidar_by_id{};
  std::atomic_bool accepting{false};
  bool initialized{false};
  bool started{false};
};

LivoxDriver::LivoxDriver(
  const NetworkConfig & network,
  const std::vector<LidarConfig> & lidars,
  SensorManager & sensor_manager)
: impl_(std::make_unique<Impl>(network, lidars, sensor_manager))
{
}

LivoxDriver::~LivoxDriver() = default;

void LivoxDriver::setAcquisitionEnabled(const bool enabled)
{
  impl_->setAcquisitionEnabled(enabled);
}

}  // namespace autonomy
