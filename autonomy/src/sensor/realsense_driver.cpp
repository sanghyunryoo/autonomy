#include "autonomy/sensor/realsense_driver.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <librealsense2/rs.hpp>

#include "autonomy/sensor/sensor_manager.hpp"
#include "autonomy/sensor/types.hpp"

namespace autonomy {
namespace {

std::uint64_t timestampNanoseconds(const rs2::frame & frame)
{
  const double milliseconds = frame.get_timestamp();
  if (std::isfinite(milliseconds) && milliseconds >= 0.0) {
    return static_cast<std::uint64_t>(milliseconds * 1000000.0);
  }
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count());
}

SensorPacket packetFrom(const rs2::video_frame & frame, const std::string & encoding)
{
  SensorPacket packet;
  const auto * bytes = static_cast<const std::uint8_t *>(frame.get_data());
  const std::size_t size = static_cast<std::size_t>(frame.get_data_size());
  packet.bytes = std::make_shared<const std::vector<std::uint8_t>>(bytes, bytes + size);
  packet.encoding = encoding;
  packet.width = static_cast<std::uint32_t>(frame.get_width());
  packet.height = static_cast<std::uint32_t>(frame.get_height());
  packet.row_step = static_cast<std::uint32_t>(frame.get_stride_in_bytes());
  packet.stamp_nanoseconds = timestampNanoseconds(frame);
  return packet;
}

CameraIntrinsics intrinsicsFrom(const rs2::video_frame & frame)
{
  const rs2::video_stream_profile profile = frame.get_profile().as<rs2::video_stream_profile>();
  const auto intrinsics = profile.get_intrinsics();
  CameraIntrinsics result;
  result.valid = intrinsics.fx > 0.0F && intrinsics.fy > 0.0F;
  result.width = static_cast<std::uint32_t>(intrinsics.width);
  result.height = static_cast<std::uint32_t>(intrinsics.height);
  result.matrix = {
    intrinsics.fx, 0.0, intrinsics.ppx,
    0.0, intrinsics.fy, intrinsics.ppy,
    0.0, 0.0, 1.0};
  result.stamp_nanoseconds = timestampNanoseconds(frame);
  return result;
}

}  // namespace

struct RealSenseDriver::Impl {
  struct CameraSession {
    explicit CameraSession(CameraConfig value)
    : config(std::move(value))
    {
    }

    CameraConfig config{};
    std::string serial{};
    rs2::pipeline pipeline{};
    rs2::pointcloud pointcloud{};
    std::mutex imu_mutex{};
    ImuData latest_imu{};
    bool running{false};
  };

  Impl(const std::vector<CameraConfig> & configured_cameras, SensorManager & manager)
  : sensor_manager(manager)
  {
    for (const CameraConfig & camera : configured_cameras) {
      if (camera.enabled) cameras.push_back(std::make_unique<CameraSession>(camera));
    }
  }

  ~Impl()
  {
    stop();
  }

  void setAcquisitionEnabled(const bool enabled)
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex);
    if (active.load(std::memory_order_acquire) == enabled) return;
    if (!enabled) {
      active.store(false, std::memory_order_release);
      stopLocked();
      return;
    }

    try {
      active.store(true, std::memory_order_release);
      for (const std::unique_ptr<CameraSession> & session : cameras) start(*session);
    } catch (...) {
      active.store(false, std::memory_order_release);
      stopLocked();
      throw;
    }
  }

  void stop()
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex);
    active.store(false, std::memory_order_release);
    stopLocked();
  }

  [[nodiscard]] std::string selectSerial(const CameraConfig & camera) const
  {
    for (rs2::device device : context.query_devices()) {
      if (!device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) continue;
      const std::string serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      if (!camera.serial.empty() && serial == camera.serial) return serial;
      if (camera.serial.empty() && device.supports(RS2_CAMERA_INFO_PHYSICAL_PORT)) {
        const std::string physical_port = device.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
        if (physical_port == camera.port || physical_port.rfind(camera.port, 0U) == 0U) {
          return serial;
        }
      }
    }
    const std::string selector = !camera.serial.empty() ? "serial " + camera.serial :
      "physical port " + camera.port;
    throw std::runtime_error("RealSense " + camera.id + " was not found at " + selector);
  }

  void start(CameraSession & session)
  {
    if (session.running) return;
    session.serial = selectSerial(session.config);
    rs2::config config;
    config.enable_device(session.serial);
    config.enable_stream(
      RS2_STREAM_COLOR, static_cast<int>(session.config.width), static_cast<int>(session.config.height),
      RS2_FORMAT_RGB8, static_cast<int>(session.config.fps));
    config.enable_stream(
      RS2_STREAM_DEPTH, static_cast<int>(session.config.width), static_cast<int>(session.config.height),
      RS2_FORMAT_Z16, static_cast<int>(session.config.fps));
    if (session.config.enable_imu) {
      config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);
      config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 63);
    }
    session.pipeline.start(config, [this, session_ptr = &session](const rs2::frame frame) {
      onFrame(*session_ptr, frame);
    });
    session.running = true;
  }

  void stopLocked()
  {
    for (const std::unique_ptr<CameraSession> & session : cameras) {
      if (!session->running) continue;
      try {
        session->pipeline.stop();
      } catch (const rs2::error &) {
        // A detached USB device cannot be stopped cleanly; it is already idle.
      }
      session->running = false;
    }
  }

  void onFrame(CameraSession & session, const rs2::frame & frame)
  {
    if (!active.load(std::memory_order_acquire)) return;
    if (const rs2::motion_frame motion = frame.as<rs2::motion_frame>()) {
      onMotion(session, motion);
      return;
    }
    const rs2::frameset frames = frame.as<rs2::frameset>();
    if (!frames) return;
    const rs2::video_frame color = frames.get_color_frame();
    const rs2::depth_frame depth = frames.get_depth_frame();
    if (!color || !depth) return;

    SensorPacket rgb = packetFrom(color, "rgb8");
    rgb.frame_id = session.config.frame;
    SensorPacket depth_packet = packetFrom(depth, "16UC1");
    depth_packet.frame_id = session.config.frame;
    CameraIntrinsics intrinsics = intrinsicsFrom(depth);
    intrinsics.frame_id = session.config.frame;

    session.pointcloud.map_to(color);
    const rs2::points sdk_points = session.pointcloud.calculate(depth);
    const auto * vertices = sdk_points.get_vertices();
    auto points = std::make_shared<std::vector<Point3f>>();
    points->reserve(static_cast<std::size_t>(sdk_points.size()));
    for (std::size_t index = 0U; index < static_cast<std::size_t>(sdk_points.size()); ++index) {
      Point3f point;
      point.x = vertices[index].x;
      point.y = vertices[index].y;
      point.z = vertices[index].z;
      points->push_back(point);
    }
    PointCloudData cloud;
    cloud.raw = depth_packet;
    cloud.raw.encoding = "librealsense2/vertices";
    cloud.points = std::move(points);

    sensor_manager.updateCameraRgb(session.config.id, std::move(rgb));
    sensor_manager.updateCameraDepth(session.config.id, std::move(depth_packet));
    sensor_manager.updateCameraIntrinsics(session.config.id, std::move(intrinsics));
    sensor_manager.updateCameraPoints(session.config.id, std::move(cloud));
  }

  void onMotion(CameraSession & session, const rs2::motion_frame & frame)
  {
    if (!session.config.enable_imu) return;
    const auto value = frame.get_motion_data();
    std::lock_guard<std::mutex> lock(session.imu_mutex);
    ImuData & imu = session.latest_imu;
    imu.valid = true;
    imu.stamp_nanoseconds = timestampNanoseconds(frame);
    imu.frame_id = session.config.frame;
    const auto stream = frame.get_profile().stream_type();
    if (stream == RS2_STREAM_GYRO) {
      imu.angular_velocity = {value.x, value.y, value.z};
    } else if (stream == RS2_STREAM_ACCEL) {
      imu.linear_acceleration = {value.x, value.y, value.z};
    } else {
      return;
    }
    sensor_manager.updateCameraImu(session.config.id, imu);
  }

  SensorManager & sensor_manager;
  rs2::context context{};
  std::vector<std::unique_ptr<CameraSession>> cameras{};
  std::mutex lifecycle_mutex{};
  std::atomic_bool active{false};
};

RealSenseDriver::RealSenseDriver(
  const std::vector<CameraConfig> & cameras,
  SensorManager & sensor_manager)
: impl_(std::make_unique<Impl>(cameras, sensor_manager))
{
}

RealSenseDriver::~RealSenseDriver() = default;

void RealSenseDriver::setAcquisitionEnabled(const bool enabled)
{
  impl_->setAcquisitionEnabled(enabled);
}

}  // namespace autonomy
