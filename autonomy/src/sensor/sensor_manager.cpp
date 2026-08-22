#include "autonomy/sensor/sensor_manager.hpp"

#include "autonomy/sensor/livox_driver.hpp"
#include "autonomy/sensor/realsense_driver.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <utility>

namespace autonomy {
namespace {

Point3f transformPoint(const Transform & transform, const Point3f & point)
{
  Point3f result;
  result.x = static_cast<float>(
    transform.rotation[0] * point.x + transform.rotation[1] * point.y +
    transform.rotation[2] * point.z + transform.translation[0]);
  result.y = static_cast<float>(
    transform.rotation[3] * point.x + transform.rotation[4] * point.y +
    transform.rotation[5] * point.z + transform.translation[1]);
  result.z = static_cast<float>(
    transform.rotation[6] * point.x + transform.rotation[7] * point.y +
    transform.rotation[8] * point.z + transform.translation[2]);
  result.intensity = point.intensity;
  result.time_offset_seconds = point.time_offset_seconds;
  return result;
}

template<typename RecordT>
void clearRecord(RecordT & record)
{
  record.data = {};
  record.received_at = {};
  record.points_received_at = {};
  record.imu_received_at = {};
}

}  // namespace

const char * sensorHealthName(const SensorHealth health)
{
  switch (health) {
    case SensorHealth::Disabled:
      return "disabled";
    case SensorHealth::Idle:
      return "idle";
    case SensorHealth::Ready:
      return "ready";
    case SensorHealth::Stale:
      return "stale";
    case SensorHealth::MissingTransform:
      return "missing_transform";
    default:
      return "unknown";
  }
}

SensorManager::SensorManager(const RuntimeConfig & config)
: base_frame_(config.base_frame), target_frame_(config.target_frame)
{
  UrdfModel urdf;
  urdf.load(config.urdf_path);

  for (const CameraConfig & camera : config.cameras) {
    CameraRecord record;
    record.config = camera;
    record.base_from_sensor = urdf.findTransform(base_frame_, camera.frame);
    record.target_from_sensor = urdf.findTransform(target_frame_, camera.frame);
    cameras_.emplace(camera.id, std::move(record));
  }
  for (const LidarConfig & lidar : config.lidars) {
    LidarRecord record;
    record.config = lidar;
    record.base_from_sensor = urdf.findTransform(base_frame_, lidar.frame);
    record.target_from_sensor = urdf.findTransform(target_frame_, lidar.frame);
    lidars_.emplace(lidar.id, std::move(record));
  }

  // Both drivers are transport-neutral and call the update* methods below.
  // They are created only after the URDF-backed inventory is complete.
  realsense_driver_ = std::make_unique<RealSenseDriver>(config.cameras, *this);
  livox_driver_ = std::make_unique<LivoxDriver>(config.network, config.lidars, *this);
}

SensorManager::~SensorManager()
{
  setAcquisitionEnabled(false);
  // Stop SDK callbacks before records and their mutex are destroyed.
  realsense_driver_.reset();
  livox_driver_.reset();
}

void SensorManager::setAcquisitionEnabled(const bool enabled)
{
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (acquisition_enabled_ == enabled) {
      return;
    }
    acquisition_enabled_ = enabled;
    changed = true;
    if (!enabled) {
      for (auto & item : cameras_) {
        clearRecord(item.second);
      }
      for (auto & item : lidars_) {
        clearRecord(item.second);
      }
    }
  }
  if (changed) {
    if (realsense_driver_) realsense_driver_->setAcquisitionEnabled(enabled);
    if (livox_driver_) livox_driver_->setAcquisitionEnabled(enabled);
  }
}

void SensorManager::updateCameraRgb(const std::string & id, SensorPacket data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto item = cameras_.find(id);
  if (item == cameras_.end() || !acquisition_enabled_ || !item->second.config.enabled) {
    return;
  }
  item->second.data.rgb = std::move(data);
  updateReceivedAt(item->second.received_at);
}

void SensorManager::updateCameraDepth(const std::string & id, SensorPacket data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto item = cameras_.find(id);
  if (item == cameras_.end() || !acquisition_enabled_ || !item->second.config.enabled) {
    return;
  }
  item->second.data.depth = std::move(data);
  updateReceivedAt(item->second.received_at);
  item->second.depth_received_at = item->second.received_at;
}

void SensorManager::updateCameraPoints(const std::string & id, PointCloudData data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto item = cameras_.find(id);
  if (item == cameras_.end() || !acquisition_enabled_ || !item->second.config.enabled) {
    return;
  }
  item->second.data.points = std::move(data);
  updateReceivedAt(item->second.received_at);
  item->second.points_received_at = item->second.received_at;
}

void SensorManager::updateCameraImu(const std::string & id, ImuData data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto item = cameras_.find(id);
  if (item == cameras_.end() || !acquisition_enabled_ || !item->second.config.enabled) {
    return;
  }
  item->second.data.imu = std::move(data);
  updateReceivedAt(item->second.received_at);
  item->second.imu_received_at = item->second.received_at;
}

void SensorManager::updateCameraIntrinsics(const std::string & id, CameraIntrinsics data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto item = cameras_.find(id);
  if (item == cameras_.end() || !acquisition_enabled_ || !item->second.config.enabled) {
    return;
  }
  item->second.data.intrinsics = std::move(data);
  updateReceivedAt(item->second.received_at);
}

void SensorManager::updateLidarPoints(const std::string & id, PointCloudData data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto item = lidars_.find(id);
  if (item == lidars_.end() || !acquisition_enabled_ || !item->second.config.enabled) {
    return;
  }
  item->second.data.points = std::move(data);
  updateReceivedAt(item->second.received_at);
  item->second.points_received_at = item->second.received_at;
}

void SensorManager::updateLidarImu(const std::string & id, ImuData data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto item = lidars_.find(id);
  if (item == lidars_.end() || !acquisition_enabled_ || !item->second.config.enabled) {
    return;
  }
  item->second.data.imu = data;
  item->second.data.imu_samples.push_back(std::move(data));
  constexpr std::size_t kMaximumImuSamples = 1000U;
  if (item->second.data.imu_samples.size() > kMaximumImuSamples) {
    const std::size_t excess = item->second.data.imu_samples.size() - kMaximumImuSamples;
    item->second.data.imu_samples.erase(
      item->second.data.imu_samples.begin(),
      item->second.data.imu_samples.begin() + static_cast<std::ptrdiff_t>(excess));
  }
  updateReceivedAt(item->second.received_at);
  item->second.imu_received_at = item->second.received_at;
}

SensorSnapshot SensorManager::snapshot(const std::chrono::steady_clock::time_point now) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  SensorSnapshot result;
  result.acquisition_enabled = acquisition_enabled_;
  result.base_frame = base_frame_;
  result.target_frame = target_frame_;
  result.cameras.reserve(cameras_.size());
  for (const auto & item : cameras_) {
    const CameraRecord & record = item.second;
    CameraSnapshot sensor;
    sensor.id = record.config.id;
    sensor.port = record.config.port;
    sensor.role = record.config.role;
    sensor.frame = record.config.frame;
    sensor.data = record.data;
    sensor.timeout = record.config.timeout;
    sensor.received_at = record.received_at;
    sensor.points_received_at = record.points_received_at;
    sensor.depth_received_at = record.depth_received_at;
    if (record.base_from_sensor) {
      sensor.base_from_sensor = *record.base_from_sensor;
    }
    if (record.target_from_sensor) {
      sensor.target_from_sensor = *record.target_from_sensor;
    }
    sensor.health = healthFor(
      record.config.enabled, record.data.points.available(),
      record.base_from_sensor.has_value() && record.target_from_sensor.has_value(),
      record.points_received_at, record.config.timeout, now);
    result.cameras.push_back(std::move(sensor));
  }
  result.lidars.reserve(lidars_.size());
  for (const auto & item : lidars_) {
    const LidarRecord & record = item.second;
    LidarSnapshot sensor;
    sensor.id = record.config.id;
    sensor.ip = record.config.ip;
    sensor.role = record.config.role;
    sensor.frame = record.config.frame;
    sensor.data = record.data;
    sensor.timeout = record.config.timeout;
    sensor.received_at = record.received_at;
    sensor.points_received_at = record.points_received_at;
    sensor.imu_received_at = record.imu_received_at;
    if (record.base_from_sensor) {
      sensor.base_from_sensor = *record.base_from_sensor;
    }
    if (record.target_from_sensor) {
      sensor.target_from_sensor = *record.target_from_sensor;
    }
    sensor.health = healthFor(
      record.config.enabled, record.data.points.available(),
      record.base_from_sensor.has_value() && record.target_from_sensor.has_value(),
      record.points_received_at, record.config.timeout, now);
    result.lidars.push_back(std::move(sensor));
  }
  std::sort(result.cameras.begin(), result.cameras.end(), [](const CameraSnapshot & left, const CameraSnapshot & right) {
    return left.id < right.id;
  });
  std::sort(result.lidars.begin(), result.lidars.end(), [](const LidarSnapshot & left, const LidarSnapshot & right) {
    return left.id < right.id;
  });
  result.merged = merge(result.cameras, result.lidars);
  result.lidar_merged = mergeLidars(result.lidars);
  return result;
}

std::vector<Transform> SensorManager::sensorTransforms() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Transform> transforms;
  transforms.reserve(cameras_.size() + lidars_.size());
  for (const auto & item : cameras_) {
    if (item.second.base_from_sensor) transforms.push_back(*item.second.base_from_sensor);
  }
  for (const auto & item : lidars_) {
    if (item.second.base_from_sensor) transforms.push_back(*item.second.base_from_sensor);
  }
  std::sort(transforms.begin(), transforms.end(), [](const Transform & left, const Transform & right) {
    return left.child_frame < right.child_frame;
  });
  return transforms;
}

SensorHealth SensorManager::healthFor(
  const bool enabled,
  const bool has_points,
  const bool has_transform,
  const std::chrono::steady_clock::time_point points_received_at,
  const std::chrono::milliseconds timeout,
  const std::chrono::steady_clock::time_point now) const
{
  if (!enabled) {
    return SensorHealth::Disabled;
  }
  if (!acquisition_enabled_) {
    return SensorHealth::Idle;
  }
  if (!has_transform) {
    return SensorHealth::MissingTransform;
  }
  if (!has_points || points_received_at == std::chrono::steady_clock::time_point{} ||
    now - points_received_at > timeout) {
    return SensorHealth::Stale;
  }
  return SensorHealth::Ready;
}

void SensorManager::updateReceivedAt(std::chrono::steady_clock::time_point & received_at)
{
  received_at = std::chrono::steady_clock::now();
}

MergedSensorData SensorManager::merge(
  const std::vector<CameraSnapshot> & cameras,
  const std::vector<LidarSnapshot> & lidars) const
{
  MergedSensorData result;
  result.target_frame = target_frame_;

  const auto append = [&result](const std::string & id, const PointCloudData & cloud,
      const Transform & target_from_sensor) {
      if (!cloud.points || cloud.points->empty()) {
        return;
      }
      result.source_sensor_ids.push_back(id);
      result.stamp_nanoseconds = std::max(result.stamp_nanoseconds, cloud.raw.stamp_nanoseconds);
      result.points.reserve(result.points.size() + cloud.points->size());
      for (const Point3f & point : *cloud.points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
          result.points.push_back(transformPoint(target_from_sensor, point));
        }
      }
    };

  for (const CameraSnapshot & camera : cameras) {
    if (camera.health == SensorHealth::Ready) {
      append(camera.id, camera.data.points, camera.target_from_sensor);
    }
  }
  for (const LidarSnapshot & lidar : lidars) {
    if (lidar.health == SensorHealth::Ready) {
      append(lidar.id, lidar.data.points, lidar.target_from_sensor);
    }
  }
  return result;
}

MergedSensorData SensorManager::mergeLidars(const std::vector<LidarSnapshot> & lidars) const
{
  MergedSensorData result;
  result.target_frame = target_frame_;
  for (const LidarSnapshot & lidar : lidars) {
    if (lidar.health != SensorHealth::Ready || !lidar.data.points.points ||
      lidar.data.points.points->empty()) {
      continue;
    }
    result.source_sensor_ids.push_back(lidar.id);
    result.stamp_nanoseconds = std::max(
      result.stamp_nanoseconds, lidar.data.points.raw.stamp_nanoseconds);
    result.points.reserve(result.points.size() + lidar.data.points.points->size());
    for (const Point3f & point : *lidar.data.points.points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        result.points.push_back(transformPoint(lidar.target_from_sensor, point));
      }
    }
  }
  return result;
}

}  // namespace autonomy
