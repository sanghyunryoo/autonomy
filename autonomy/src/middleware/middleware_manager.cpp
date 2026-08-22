#include "autonomy/middleware/middleware_manager.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <core/msg/command_filter.hpp>
#include <core/msg/robot_report.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "autonomy/middleware/dds_height_map_publisher.hpp"
#include "autonomy/middleware/dds_linear_velocity_publisher.hpp"
#include "autonomy/msg/autonomy_state.hpp"
#include "autonomy/sensor/sensor_manager.hpp"
#include "autonomy/srv/set_autonomy_mode.hpp"

namespace autonomy {
namespace {

using AutonomyStateMsg = autonomy::msg::AutonomyState;

constexpr std::size_t kMaximumQueuedInputs = 32U;
constexpr auto kMiddlewarePollBudget = std::chrono::milliseconds(1);

builtin_interfaces::msg::Time toRosTime(const std::uint64_t nanoseconds)
{
  builtin_interfaces::msg::Time time;
  time.sec = static_cast<std::int32_t>(nanoseconds / 1000000000ULL);
  time.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000ULL);
  return time;
}

sensor_msgs::msg::PointCloud2 toPointCloud(
  const MergedSensorData & sensor_data,
  const std::uint32_t maximum_points)
{
  const std::size_t source_count = sensor_data.points.size();
  const std::size_t stride = source_count <= maximum_points ? 1U :
    (source_count + maximum_points - 1U) / maximum_points;
  const std::size_t output_count = source_count == 0U ? 0U : (source_count + stride - 1U) / stride;
  sensor_msgs::msg::PointCloud2 message;
  message.header.frame_id = sensor_data.target_frame;
  message.header.stamp = toRosTime(sensor_data.stamp_nanoseconds);
  message.height = 1;
  message.width = static_cast<std::uint32_t>(output_count);
  message.is_bigendian = false;
  message.is_dense = false;
  message.point_step = 4U * sizeof(float);
  message.row_step = message.width * message.point_step;
  message.fields.resize(4);
  const char * names[] = {"x", "y", "z", "intensity"};
  for (std::size_t index = 0; index < message.fields.size(); ++index) {
    message.fields[index].name = names[index];
    message.fields[index].offset = static_cast<std::uint32_t>(index * sizeof(float));
    message.fields[index].datatype = sensor_msgs::msg::PointField::FLOAT32;
    message.fields[index].count = 1;
  }
  message.data.resize(static_cast<std::size_t>(message.row_step));
  std::size_t output_index = 0U;
  for (std::size_t index = 0; index < source_count; index += stride) {
    std::uint8_t * destination = message.data.data() + output_index * message.point_step;
    const Point3f & source = sensor_data.points[index];
    std::memcpy(destination, &source.x, sizeof(float));
    std::memcpy(destination + sizeof(float), &source.y, sizeof(float));
    std::memcpy(destination + 2U * sizeof(float), &source.z, sizeof(float));
    std::memcpy(destination + 3U * sizeof(float), &source.intensity, sizeof(float));
    ++output_index;
  }
  return message;
}

sensor_msgs::msg::PointCloud2 toElevationPointCloud(
  const ElevationMap & map,
  const std::uint32_t maximum_points)
{
  std::size_t finite_cells = 0U;
  for (const float height : map.data) {
    if (std::isfinite(height)) ++finite_cells;
  }
  const std::size_t stride = finite_cells <= maximum_points ? 1U :
    (finite_cells + maximum_points - 1U) / maximum_points;

  sensor_msgs::msg::PointCloud2 message;
  message.header.frame_id = map.frame_id;
  message.header.stamp = toRosTime(map.stamp_nanoseconds);
  message.height = 1;
  message.width = static_cast<std::uint32_t>(
    finite_cells == 0U ? 0U : (finite_cells + stride - 1U) / stride);
  message.is_bigendian = false;
  message.is_dense = true;
  message.point_step = 4U * sizeof(float);
  message.row_step = message.width * message.point_step;
  message.fields.resize(4);
  const char * names[] = {"x", "y", "z", "intensity"};
  for (std::size_t index = 0; index < message.fields.size(); ++index) {
    message.fields[index].name = names[index];
    message.fields[index].offset = static_cast<std::uint32_t>(index * sizeof(float));
    message.fields[index].datatype = sensor_msgs::msg::PointField::FLOAT32;
    message.fields[index].count = 1;
  }
  message.data.resize(static_cast<std::size_t>(message.row_step));

  std::size_t finite_index = 0U;
  std::size_t output_index = 0U;
  for (std::size_t index = 0; index < map.data.size(); ++index) {
    const float height = map.data[index];
    if (!std::isfinite(height)) continue;
    if (finite_index++ % stride != 0U) continue;
    const std::uint32_t column = static_cast<std::uint32_t>(index % map.width);
    const std::uint32_t row = static_cast<std::uint32_t>(index / map.width);
    const float values[] = {
      static_cast<float>(map.x_min + (static_cast<double>(column) + 0.5) * map.resolution),
      static_cast<float>(map.y_min + (static_cast<double>(row) + 0.5) * map.resolution),
      height,
      0.0F,
    };
    std::uint8_t * destination = message.data.data() + output_index * message.point_step;
    std::memcpy(destination, values, sizeof(values));
    ++output_index;
  }
  return message;
}

geometry_msgs::msg::Quaternion quaternionFromRotation(const std::array<double, 9> & rotation)
{
  geometry_msgs::msg::Quaternion quaternion;
  const double trace = rotation[0] + rotation[4] + rotation[8];
  if (trace > 0.0) {
    const double scale = 2.0 * std::sqrt(trace + 1.0);
    quaternion.w = 0.25 * scale;
    quaternion.x = (rotation[7] - rotation[5]) / scale;
    quaternion.y = (rotation[2] - rotation[6]) / scale;
    quaternion.z = (rotation[3] - rotation[1]) / scale;
  } else if (rotation[0] > rotation[4] && rotation[0] > rotation[8]) {
    const double scale = 2.0 * std::sqrt(1.0 + rotation[0] - rotation[4] - rotation[8]);
    quaternion.w = (rotation[7] - rotation[5]) / scale;
    quaternion.x = 0.25 * scale;
    quaternion.y = (rotation[1] + rotation[3]) / scale;
    quaternion.z = (rotation[2] + rotation[6]) / scale;
  } else if (rotation[4] > rotation[8]) {
    const double scale = 2.0 * std::sqrt(1.0 + rotation[4] - rotation[0] - rotation[8]);
    quaternion.w = (rotation[2] - rotation[6]) / scale;
    quaternion.x = (rotation[1] + rotation[3]) / scale;
    quaternion.y = 0.25 * scale;
    quaternion.z = (rotation[5] + rotation[7]) / scale;
  } else {
    const double scale = 2.0 * std::sqrt(1.0 + rotation[8] - rotation[0] - rotation[4]);
    quaternion.w = (rotation[3] - rotation[1]) / scale;
    quaternion.x = (rotation[2] + rotation[6]) / scale;
    quaternion.y = (rotation[5] + rotation[7]) / scale;
    quaternion.z = 0.25 * scale;
  }
  return quaternion;
}

tf2_msgs::msg::TFMessage toTfMessage(
  const std::vector<Transform> & transforms,
  const rclcpp::Time & stamp)
{
  tf2_msgs::msg::TFMessage message;
  message.transforms.reserve(transforms.size());
  for (const Transform & transform : transforms) {
    geometry_msgs::msg::TransformStamped stamped;
    stamped.header.stamp = stamp;
    stamped.header.frame_id = transform.parent_frame;
    stamped.child_frame_id = transform.child_frame;
    stamped.transform.translation.x = transform.translation[0];
    stamped.transform.translation.y = transform.translation[1];
    stamped.transform.translation.z = transform.translation[2];
    stamped.transform.rotation = quaternionFromRotation(transform.rotation);
    message.transforms.push_back(std::move(stamped));
  }
  return message;
}

sensor_msgs::msg::Image toImage(const SensorPacket & packet)
{
  sensor_msgs::msg::Image message;
  message.header.frame_id = packet.frame_id;
  message.header.stamp = toRosTime(packet.stamp_nanoseconds);
  message.height = packet.height;
  message.width = packet.width;
  message.encoding = packet.encoding;
  message.is_bigendian = false;
  message.step = packet.row_step;
  if (packet.bytes) message.data = *packet.bytes;
  return message;
}

std::size_t bytesPerPixel(const std::string & encoding)
{
  if (encoding == "rgb8" || encoding == "bgr8") return 3U;
  if (encoding == "rgba8" || encoding == "bgra8") return 4U;
  if (encoding == "mono8") return 1U;
  return 0U;
}

void paintBoundingBoxPixel(
  sensor_msgs::msg::Image & image,
  const std::uint32_t column,
  const std::uint32_t row,
  const std::size_t bytes_per_pixel)
{
  const std::size_t offset = static_cast<std::size_t>(row) * image.step +
    static_cast<std::size_t>(column) * bytes_per_pixel;
  if (offset + bytes_per_pixel > image.data.size()) return;
  if (bytes_per_pixel == 1U) {
    image.data[offset] = 255U;
    return;
  }
  const bool bgr = image.encoding == "bgr8" || image.encoding == "bgra8";
  image.data[offset] = bgr ? 0U : 255U;
  image.data[offset + 1U] = 0U;
  image.data[offset + 2U] = bgr ? 255U : 0U;
}

sensor_msgs::msg::Image toBoundingBoxImage(const SensorPacket & packet, const BoundingBox & bbox)
{
  sensor_msgs::msg::Image message = toImage(packet);
  const std::size_t bytes_per_pixel = bytesPerPixel(message.encoding);
  if (bytes_per_pixel == 0U || message.width == 0U || message.height == 0U ||
    message.step < message.width * bytes_per_pixel || bbox.width == 0U || bbox.height == 0U ||
    bbox.x >= message.width || bbox.y >= message.height) {
    return message;
  }
  const std::uint32_t right = std::min(message.width - 1U, bbox.x + bbox.width - 1U);
  const std::uint32_t bottom = std::min(message.height - 1U, bbox.y + bbox.height - 1U);
  for (std::uint32_t column = bbox.x; column <= right; ++column) {
    paintBoundingBoxPixel(message, column, bbox.y, bytes_per_pixel);
    paintBoundingBoxPixel(message, column, bottom, bytes_per_pixel);
  }
  for (std::uint32_t row = bbox.y; row <= bottom; ++row) {
    paintBoundingBoxPixel(message, bbox.x, row, bytes_per_pixel);
    paintBoundingBoxPixel(message, right, row, bytes_per_pixel);
  }
  return message;
}

sensor_msgs::msg::Image downsampleImage(
  const sensor_msgs::msg::Image & image,
  const std::uint32_t maximum_width,
  const std::uint32_t maximum_height)
{
  const std::size_t bytes_per_pixel = bytesPerPixel(image.encoding);
  if (bytes_per_pixel == 0U || image.width == 0U || image.height == 0U ||
    (image.width <= maximum_width && image.height <= maximum_height) ||
    image.step < image.width * bytes_per_pixel || image.data.size() < image.step * image.height) {
    return image;
  }
  const double scale = std::min(
    static_cast<double>(maximum_width) / static_cast<double>(image.width),
    static_cast<double>(maximum_height) / static_cast<double>(image.height));
  sensor_msgs::msg::Image result = image;
  result.width = std::max(1U, static_cast<std::uint32_t>(std::floor(image.width * scale)));
  result.height = std::max(1U, static_cast<std::uint32_t>(std::floor(image.height * scale)));
  result.step = static_cast<std::uint32_t>(result.width * bytes_per_pixel);
  result.data.resize(static_cast<std::size_t>(result.step) * result.height);
  for (std::uint32_t row = 0U; row < result.height; ++row) {
    const std::uint32_t source_row = std::min(
      image.height - 1U, static_cast<std::uint32_t>(static_cast<double>(row) / scale));
    for (std::uint32_t column = 0U; column < result.width; ++column) {
      const std::uint32_t source_column = std::min(
        image.width - 1U, static_cast<std::uint32_t>(static_cast<double>(column) / scale));
      const std::size_t source_offset = static_cast<std::size_t>(source_row) * image.step +
        static_cast<std::size_t>(source_column) * bytes_per_pixel;
      const std::size_t destination_offset = static_cast<std::size_t>(row) * result.step +
        static_cast<std::size_t>(column) * bytes_per_pixel;
      std::memcpy(result.data.data() + destination_offset, image.data.data() + source_offset, bytes_per_pixel);
    }
  }
  return result;
}

Pose2d poseFrom(const geometry_msgs::msg::PoseStamped & message)
{
  const auto & orientation = message.pose.orientation;
  const double yaw = std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
  Pose2d pose;
  pose.x = message.pose.position.x;
  pose.y = message.pose.position.y;
  pose.yaw = yaw;
  pose.frame_id = message.header.frame_id;
  pose.received_at = std::chrono::steady_clock::now();
  return pose;
}

nav_msgs::msg::Path toPath(const std::vector<Pose2d> & path, const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path message;
  if (path.empty()) {
    return message;
  }
  message.header.stamp = stamp;
  message.header.frame_id = path.front().frame_id;
  message.poses.reserve(path.size());
  for (const Pose2d & waypoint : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = message.header;
    pose.pose.position.x = waypoint.x;
    pose.pose.position.y = waypoint.y;
    pose.pose.orientation.w = std::cos(waypoint.yaw * 0.5);
    pose.pose.orientation.z = std::sin(waypoint.yaw * 0.5);
    message.poses.push_back(std::move(pose));
  }
  return message;
}

nav_msgs::msg::Odometry toOdometry(
  const Pose2d & pose,
  const std::string & child_frame,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry message;
  message.header.stamp = stamp;
  message.header.frame_id = pose.frame_id;
  message.child_frame_id = child_frame;
  message.pose.pose.position.x = pose.x;
  message.pose.pose.position.y = pose.y;
  message.pose.pose.orientation.w = std::cos(pose.yaw * 0.5);
  message.pose.pose.orientation.z = std::sin(pose.yaw * 0.5);
  return message;
}

}  // namespace

struct MiddlewareManager::Impl {
  Impl(const RuntimeConfig & config, SensorManager & sensor_manager)
  : config_(config),
    node_(std::make_shared<rclcpp::Node>("autonomy_middleware_manager")),
    executor_(std::make_unique<rclcpp::executors::SingleThreadedExecutor>())
  {
    executor_->add_node(node_);
    robot_report_sub_ = node_->create_subscription<core::msg::RobotReport>(
      config_.middleware.ros2.robot_report_topic, 10,
      [this](const core::msg::RobotReport::SharedPtr message) { onRobotReport(*message); });
    navigation_goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      config_.middleware.ros2.navigation_goal_topic, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) { onNavigationGoal(*message); });

    state_pub_ = node_->create_publisher<AutonomyStateMsg>("~/state", 10);
    status_pub_ = node_->create_publisher<AutonomyStateMsg>("~/status", 10);
    command_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      config_.middleware.ros2.command_topic, 10);
    command_filter_pub_ = node_->create_publisher<core::msg::CommandFilter>(
      config_.middleware.ros2.command_filter_topic, 10);
    set_mode_srv_ = node_->create_service<autonomy::srv::SetAutonomyMode>(
      "~/set_mode",
      [this](const std::shared_ptr<autonomy::srv::SetAutonomyMode::Request> request,
      std::shared_ptr<autonomy::srv::SetAutonomyMode::Response> response) {
        onSetMode(*request, *response);
      });
    autopilot_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / config_.middleware.ros2.autopilot_hz));

    const Ros2DebugConfig & debug = config_.middleware.ros2.debug;
    if (debug.enabled) {
      rclcpp::QoS debug_qos(rclcpp::KeepLast(1));
      debug_qos.best_effort();
      debug_elevation_points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        debug.elevation_points_topic, debug_qos);
      debug_livox_points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        debug.livox_points_topic, debug_qos);
      debug_slam_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
        debug.slam_odom_topic, debug_qos);
      debug_slam_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
        debug.slam_path_topic, debug_qos);
      debug_ai_rgb_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
        debug.ai_rgb_topic, debug_qos);
      debug_ai_bbox_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
        debug.ai_bbox_topic, debug_qos);
      rclcpp::QoS tf_qos(rclcpp::KeepLast(1));
      tf_qos.reliable();
      tf_qos.transient_local();
      debug_sensor_tf_pub_ = node_->create_publisher<tf2_msgs::msg::TFMessage>(
        debug.sensor_tf_topic, tf_qos);
      const std::vector<Transform> transforms = sensor_manager.sensorTransforms();
      if (!transforms.empty()) {
        debug_sensor_tf_pub_->publish(toTfMessage(transforms, node_->now()));
      }
      debug_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / debug.hz));
    }

    if (config_.middleware.dds.enabled) {
      dds_height_map_ = std::make_unique<DdsHeightMapPublisher>(
        config_.middleware.dds.domain_id, config_.middleware.dds.height_map_topic, "core_dds::HeightMap");
      dds_slam_velocity_ = std::make_unique<DdsLinearVelocityPublisher>(
        config_.middleware.dds.domain_id, config_.middleware.dds.linear_velocity_topic,
        "core_dds::LinearVelocity");
      if (!dds_height_map_->isReady()) {
        std::cerr << "[MiddlewareManager][WARN] DDS height-map publisher unavailable: " <<
          dds_height_map_->error() << '\n';
      }
      if (!dds_slam_velocity_->isReady()) {
        std::cerr << "[MiddlewareManager][WARN] DDS SLAM velocity publisher unavailable: " <<
          dds_slam_velocity_->error() << '\n';
      }
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Middleware manager ready: loop %.1f Hz; elevation %.1f Hz, SLAM %.1f Hz, planner %.1f Hz, detection %.1f Hz; lidar Jetson IP %s, SBC %s -> %s",
      config_.loop_hz, config_.elevation.hz, config_.slam.hz, config_.planner.hz, config_.detection.hz,
      config_.network.lidar_local_ip.c_str(),
      config_.network.sbc_local_ip.c_str(), config_.network.sbc_peer_ip.c_str());
  }

  ~Impl()
  {
    shutdown();
  }

  void poll()
  {
    // ROS callbacks are only an input edge. Reserve the rest of the 20 ms
    // control period for state, sensor, algorithm, and output work.
    executor_->spin_some(kMiddlewarePollBudget);
  }

  bool autopilotDue(const std::chrono::steady_clock::time_point now)
  {
    if (next_autopilot_tick_ != std::chrono::steady_clock::time_point{} && now < next_autopilot_tick_) {
      return false;
    }
    next_autopilot_tick_ = now + autopilot_period_;
    return true;
  }

  std::optional<RobotReport> popRobotReport()
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (robot_reports_.empty()) return std::nullopt;
    RobotReport report = std::move(robot_reports_.front());
    robot_reports_.pop_front();
    return report;
  }

  std::optional<Pose2d> popNavigationGoal()
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (navigation_goals_.empty()) return std::nullopt;
    Pose2d goal = std::move(navigation_goals_.front());
    navigation_goals_.pop_front();
    return goal;
  }

  std::optional<MiddlewareModeRequest> popModeRequest()
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (mode_requests_.empty()) return std::nullopt;
    MiddlewareModeRequest request = std::move(mode_requests_.front());
    mode_requests_.pop_front();
    return request;
  }

  void publish(const RuntimeCycle & cycle)
  {
    const rclcpp::Time stamp = node_->now();
    const auto control_now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      last_published_mode_ = static_cast<std::int8_t>(cycle.state.mode);
    }
    AutonomyStateMsg message;
    message.header.stamp = stamp;
    message.mode = static_cast<std::int8_t>(cycle.state.mode);
    message.mode_name = autonomyModeName(cycle.state.mode);
    message.requested_mode = autonomyModeName(cycle.state.requested_mode);
    message.autonomy_enabled = cycle.state.mode != AutonomyMode::Idle && cycle.state.mode != AutonomyMode::Error;
    message.estop_active = cycle.state.estop_active;
    message.error_active = cycle.state.robot_report.comm_fault || cycle.state.algorithm_error_code != 0;
    message.ai_enabled = hasWorkload(cycle.algorithms.plan, Workload::AiDetection);
    message.segmentation_enabled = false;
    message.robot_state = cycle.state.robot_report.raw_mode;
    message.robot_state_name = cycle.state.robot_report.mode_name;
    message.physical_estop = cycle.state.robot_report.physical_estop;
    message.comm_estop = cycle.state.robot_report.comm_estop;
    message.comm_fault = cycle.state.robot_report.comm_fault;
    message.error_reason = cycle.state.algorithm_error_reason.empty() ?
      cycle.state.robot_report.error_reason : cycle.state.algorithm_error_reason;
    message.error_code = cycle.state.algorithm_error_code != 0 ?
      cycle.state.algorithm_error_code : (cycle.state.robot_report.comm_fault ? 3001 : 0);
    message.speed_limit = 0.0F;
    message.sensors_enabled = cycle.algorithms.plan.sensors_enabled;
    message.sensors_ready = message.sensors_enabled;
    message.elevation_due = cycle.algorithms.plan.elevation_due;
    message.slam_due = cycle.algorithms.plan.slam_due;
    message.planner_due = cycle.algorithms.plan.planner_due;
    message.detection_due = cycle.algorithms.plan.detection_due;

    for (const Workload workload : cycle.algorithms.plan.active_workloads) {
      message.active_workloads.emplace_back(workloadName(workload));
      message.active_nodes.emplace_back(workloadName(workload));
    }
    appendSensorStatus(cycle.sensors.cameras, message);
    appendSensorStatus(cycle.sensors.lidars, message);
    for (const AlgorithmReport & report : cycle.algorithms.reports) {
      message.node_names.emplace_back(algorithmNodeName(report.node));
      message.node_statuses.emplace_back(algorithmStatusName(report.status));
      if (report.status == AlgorithmStatus::Warning || report.status == AlgorithmStatus::Failed) {
        message.degraded_nodes.emplace_back(algorithmNodeName(report.node));
      }
    }
    for (const std::string & status : message.sensor_statuses) {
      if (status != "ready" && status != "disabled" && status != "idle") {
        message.sensors_ready = false;
      }
    }
    if (message.error_active) {
      message.status = "error";
    } else if (!message.sensors_enabled) {
      message.status = "idle";
    } else if (!message.sensors_ready) {
      message.status = "degraded";
    } else {
      message.status = "ok";
    }
    state_pub_->publish(message);
    status_pub_->publish(message);

    const bool control_mode = cycle.state.mode == AutonomyMode::Fsd ||
      cycle.state.mode == AutonomyMode::Tracking;
    const bool control_permitted = control_mode && !message.error_active && message.sensors_ready;
    if (control_permitted && cycle.algorithms.command) {
      const VelocityCommand & candidate = *cycle.algorithms.command;
      if (std::isfinite(candidate.linear_x) && std::isfinite(candidate.linear_y) &&
        std::isfinite(candidate.angular_z)) {
        last_control_command_ = candidate;
        last_control_command_at_ = control_now;
      } else {
        last_control_command_.reset();
        last_control_command_at_ = {};
      }
    }

    const bool command_fresh = control_permitted && last_control_command_ &&
      last_control_command_at_ != std::chrono::steady_clock::time_point{} &&
      control_now - last_control_command_at_ <= config_.planner.input_timeout;
    if (!command_fresh) {
      last_control_command_.reset();
      last_control_command_at_ = {};
    }

    if (cycle.algorithms.directional_motion_filter) {
      last_directional_motion_filter_ = *cycle.algorithms.directional_motion_filter;
    }
    DirectionalMotionFilter directional_filter = last_directional_motion_filter_;
    if (message.error_active || !message.sensors_ready) {
      // Without current trusted terrain data, fail closed in X. Lateral Y is
      // intentionally left open by the requested CommandFilter policy.
      directional_filter = {};
    }
    core::msg::CommandFilter filter;
    filter.allow_linear_vel_forward_x = directional_filter.allow_linear_vel_forward_x;
    filter.allow_linear_vel_backward_x = directional_filter.allow_linear_vel_backward_x;
    filter.allow_linear_vel_forward_y = directional_filter.allow_linear_vel_forward_y;
    filter.allow_linear_vel_backward_y = directional_filter.allow_linear_vel_backward_y;
    command_filter_pub_->publish(filter);

    if (cycle.algorithms.elevation) {
      if (dds_height_map_ && dds_height_map_->isReady()) {
        dds_height_map_->publish(cycle.algorithms.elevation->data);
      }
    }
    if (cycle.algorithms.slam_linear_velocity && dds_slam_velocity_ && dds_slam_velocity_->isReady()) {
      const SlamLinearVelocity & velocity = *cycle.algorithms.slam_linear_velocity;
      dds_slam_velocity_->publish({
        static_cast<float>(velocity.x), static_cast<float>(velocity.y), static_cast<float>(velocity.z)});
    }
    updateDebugData(cycle);
    if (debugDue()) publishDebug(cycle, stamp);

    const bool immediate_zero = last_autopilot_motion_allowed_ && !command_fresh;
    if (autopilotDue(control_now) || immediate_zero) {
      geometry_msgs::msg::Twist command;
      if (command_fresh) {
        command.linear.x = last_control_command_->linear_x;
        command.linear.y = last_control_command_->linear_y;
        command.angular.z = last_control_command_->angular_z;
      }
      // Normal command traffic is rate-limited to autopilot_hz. A transition
      // from a valid command to an unsafe condition bypasses that rate limit
      // once to actively stop the lower controller.
      command_pub_->publish(command);
    }
    last_autopilot_motion_allowed_ = command_fresh;
  }

  void shutdown()
  {
    if (!node_) return;
    dds_height_map_.reset();
    dds_slam_velocity_.reset();
    if (executor_) executor_->remove_node(node_);
    executor_.reset();
    node_.reset();
  }

  bool owns_ros_context_{false};

private:
  void updateDebugData(const RuntimeCycle & cycle)
  {
    if (!debug_elevation_points_pub_) return;
    if (cycle.algorithms.elevation) latest_elevation_ = *cycle.algorithms.elevation;
    if (cycle.algorithms.detection) latest_detection_ = *cycle.algorithms.detection;
    if (!cycle.algorithms.slam_pose) return;
    latest_slam_pose_ = *cycle.algorithms.slam_pose;
    slam_path_.push_back(*cycle.algorithms.slam_pose);
    const std::size_t maximum_path_poses = config_.middleware.ros2.debug.max_path_poses;
    while (slam_path_.size() > maximum_path_poses) {
      slam_path_.pop_front();
    }
  }

  bool debugDue()
  {
    if (!debug_elevation_points_pub_) return false;
    const auto now = std::chrono::steady_clock::now();
    if (next_debug_tick_ != std::chrono::steady_clock::time_point{} && now < next_debug_tick_) {
      return false;
    }
    next_debug_tick_ = now + debug_period_;
    return true;
  }

  void publishDebug(const RuntimeCycle & cycle, const rclcpp::Time & stamp)
  {
    const Ros2DebugConfig & debug = config_.middleware.ros2.debug;
    if (latest_elevation_) {
      debug_elevation_points_pub_->publish(
        toElevationPointCloud(*latest_elevation_, debug.max_points_per_cloud));
    }
    if (!cycle.sensors.lidar_merged.points.empty()) {
      debug_livox_points_pub_->publish(
        toPointCloud(cycle.sensors.lidar_merged, debug.max_points_per_cloud));
    }
    if (latest_slam_pose_) {
      debug_slam_odom_pub_->publish(toOdometry(*latest_slam_pose_, config_.target_frame, stamp));
    }
    if (!slam_path_.empty()) {
      const std::vector<Pose2d> path(slam_path_.begin(), slam_path_.end());
      debug_slam_path_pub_->publish(toPath(path, stamp));
    }
    const CameraSnapshot * const ai_camera = findCamera(cycle.sensors, config_.detection.camera_id);
    if (ai_camera == nullptr || !ai_camera->data.rgb.available()) return;
    const sensor_msgs::msg::Image raw_rgb = downsampleImage(
      toImage(ai_camera->data.rgb), debug.max_image_width, debug.max_image_height);
    debug_ai_rgb_pub_->publish(raw_rgb);
    if (latest_detection_ && latest_detection_->source_camera_id == ai_camera->id) {
      debug_ai_bbox_pub_->publish(downsampleImage(
        toBoundingBoxImage(ai_camera->data.rgb, latest_detection_->bbox),
        debug.max_image_width, debug.max_image_height));
    }
  }

  static const CameraSnapshot * findCamera(const SensorSnapshot & sensors, const std::string & id)
  {
    const auto found = std::find_if(sensors.cameras.begin(), sensors.cameras.end(), [&id](const auto & camera) {
      return camera.id == id;
    });
    return found == sensors.cameras.end() ? nullptr : &*found;
  }

  void onRobotReport(const core::msg::RobotReport & message)
  {
    RobotReport report;
    report.raw_mode = message.robot_state;
    report.mode_name = message.robot_state_name;
    report.physical_estop = message.physical_estop;
    report.comm_estop = message.comm_estop;
    report.comm_fault = message.comm_fault;
    report.error_reason = message.error_reason;
    report.received_at = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (robot_reports_.size() >= kMaximumQueuedInputs) robot_reports_.pop_front();
    robot_reports_.push_back(std::move(report));
  }

  void onNavigationGoal(const geometry_msgs::msg::PoseStamped & message)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (navigation_goals_.size() >= kMaximumQueuedInputs) navigation_goals_.pop_front();
    navigation_goals_.push_back(poseFrom(message));
  }

  void onSetMode(
    const autonomy::srv::SetAutonomyMode::Request & request,
    autonomy::srv::SetAutonomyMode::Response & response)
  {
    MiddlewareModeRequest input;
    input.operation_mode = request.operation_mode;
    input.speed_limit = request.speed_limit;
    input.enable_ai = request.enable_ai;
    input.segmentation = request.segmentation;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if (mode_requests_.size() >= kMaximumQueuedInputs) {
        response.accepted = false;
        response.current_mode = last_published_mode_;
        response.enable_ai = request.enable_ai;
        response.segmentation = request.segmentation;
        response.message = "Mode request queue is full";
        return;
      }
      mode_requests_.push_back(std::move(input));
      response.current_mode = last_published_mode_;
    }
    response.accepted = true;
    response.enable_ai = request.enable_ai;
    response.segmentation = request.segmentation;
    response.message = "Mode request queued for the next 50 Hz control cycle";
  }

  static bool hasWorkload(const ExecutionPlan & plan, const Workload workload)
  {
    return std::find(plan.active_workloads.begin(), plan.active_workloads.end(), workload) !=
      plan.active_workloads.end();
  }

  template<typename SnapshotT>
  static void appendSensorStatus(const std::vector<SnapshotT> & sensors, AutonomyStateMsg & message)
  {
    for (const SnapshotT & sensor : sensors) {
      const std::string status = sensorHealthName(sensor.health);
      message.sensor_names.push_back(sensor.id);
      message.sensor_statuses.push_back(status);
      message.node_names.push_back(sensor.id);
      message.node_statuses.push_back(status);
      if (status != "ready" && status != "disabled" && status != "idle") {
        message.degraded_nodes.push_back(sensor.id);
      }
    }
  }

  const RuntimeConfig & config_;
  std::shared_ptr<rclcpp::Node> node_{};
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_{};
  std::mutex input_mutex_{};
  std::deque<RobotReport> robot_reports_{};
  std::deque<MiddlewareModeRequest> mode_requests_{};
  std::deque<Pose2d> navigation_goals_{};
  std::int8_t last_published_mode_{static_cast<std::int8_t>(AutonomyMode::Idle)};
  std::unique_ptr<DdsHeightMapPublisher> dds_height_map_{};
  std::unique_ptr<DdsLinearVelocityPublisher> dds_slam_velocity_{};
  rclcpp::Subscription<core::msg::RobotReport>::SharedPtr robot_report_sub_{};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr navigation_goal_sub_{};
  rclcpp::Publisher<AutonomyStateMsg>::SharedPtr state_pub_{};
  rclcpp::Publisher<AutonomyStateMsg>::SharedPtr status_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_elevation_points_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_livox_points_pub_{};
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr debug_sensor_tf_pub_{};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr debug_slam_odom_pub_{};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_slam_path_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_ai_rgb_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_ai_bbox_pub_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_{};
  rclcpp::Publisher<core::msg::CommandFilter>::SharedPtr command_filter_pub_{};
  rclcpp::Service<autonomy::srv::SetAutonomyMode>::SharedPtr set_mode_srv_{};
  std::chrono::nanoseconds debug_period_{};
  std::chrono::steady_clock::time_point next_debug_tick_{};
  std::chrono::nanoseconds autopilot_period_{std::chrono::milliseconds(100)};
  std::chrono::steady_clock::time_point next_autopilot_tick_{};
  bool last_autopilot_motion_allowed_{false};
  std::optional<VelocityCommand> last_control_command_{};
  std::chrono::steady_clock::time_point last_control_command_at_{};
  DirectionalMotionFilter last_directional_motion_filter_{};
  std::optional<ElevationMap> latest_elevation_{};
  std::optional<Pose2d> latest_slam_pose_{};
  std::optional<Detection> latest_detection_{};
  std::deque<Pose2d> slam_path_{};
};

MiddlewareManager::MiddlewareManager() = default;

MiddlewareManager::~MiddlewareManager()
{
  finish();
}

void MiddlewareManager::init(const RuntimeConfig & config, SensorManager & sensor_manager)
{
  if (impl_) throw std::logic_error("MiddlewareManager is already initialized");
  const bool owns_ros_context = !rclcpp::ok();
  if (owns_ros_context) {
    int argc = 1;
    char executable_name[] = "autonomy_middleware";
    char * argv[] = {executable_name};
    rclcpp::init(argc, argv);
  }
  try {
    impl_ = std::make_unique<Impl>(config, sensor_manager);
    impl_->owns_ros_context_ = owns_ros_context;
  } catch (...) {
    if (owns_ros_context && rclcpp::ok()) rclcpp::shutdown();
    throw;
  }
}

void MiddlewareManager::poll()
{
  if (impl_) impl_->poll();
}

bool MiddlewareManager::isRunning() const
{
  return impl_ && rclcpp::ok();
}

std::optional<RobotReport> MiddlewareManager::popRobotReport()
{
  return impl_ ? impl_->popRobotReport() : std::nullopt;
}

std::optional<MiddlewareModeRequest> MiddlewareManager::popModeRequest()
{
  return impl_ ? impl_->popModeRequest() : std::nullopt;
}

std::optional<Pose2d> MiddlewareManager::popNavigationGoal()
{
  return impl_ ? impl_->popNavigationGoal() : std::nullopt;
}

void MiddlewareManager::publish(const RuntimeCycle & cycle)
{
  if (!impl_) throw std::logic_error("MiddlewareManager::init must be called first");
  impl_->publish(cycle);
}

void MiddlewareManager::finish()
{
  if (!impl_) return;
  const bool owns_ros_context = impl_->owns_ros_context_;
  impl_.reset();
  if (owns_ros_context && rclcpp::ok()) rclcpp::shutdown();
}

}  // namespace autonomy
