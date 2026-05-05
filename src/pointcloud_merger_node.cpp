#include "height_map_ros2/pointcloud_merger_node.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <cmath>
#include <limits>
#include <regex>
#include <stdexcept>
#include <sstream>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace height_map_ros2
{

PointCloudMergerNode::PointCloudMergerNode(const rclcpp::NodeOptions & options)
: Node("pointcloud_merge_node", options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  static_tf_broadcaster_(this)
{
  loadParameters();
  publishStaticTransformsFromUrdf();
  createIo();
}

void PointCloudMergerNode::loadParameters()
{
  target_frame_ = declare_parameter<std::string>("target_frame", target_frame_);
  urdf_path_ = declare_parameter<std::string>("urdf_path", urdf_path_);
  publish_static_tf_ = declare_parameter<bool>("publish_static_tf", publish_static_tf_);
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", publish_rate_hz_);
  max_cloud_age_sec_ = declare_parameter<double>("max_cloud_age_sec", max_cloud_age_sec_);
  min_range_ = declare_parameter<double>("depth_filter.min_range", min_range_);
  max_range_ = declare_parameter<double>("depth_filter.max_range", max_range_);
  pixel_stride_ = declare_parameter<int>("depth_filter.pixel_stride", pixel_stride_);

  const std::vector<std::string> default_camera_names;
  const auto camera_names =
    declare_parameter<std::vector<std::string>>("camera_names", default_camera_names);

  cameras_.clear();
  cameras_.reserve(camera_names.size());

  for (const auto & name : camera_names) {
    CameraSource camera;
    camera.name = name;
    camera.enabled = declare_parameter<bool>("cameras." + name + ".enabled", true);
    camera.depth_topic = declare_parameter<std::string>(
      "cameras." + name + ".depth_topic", "/" + name + "/depth/image_rect");
    camera.camera_info_topic = declare_parameter<std::string>(
      "cameras." + name + ".camera_info_topic", "/" + name + "/depth/camera_info");
    camera.mount_frame = declare_parameter<std::string>(
      "cameras." + name + ".mount_frame", "");
    camera.optical_frame = declare_parameter<std::string>(
      "cameras." + name + ".optical_frame", "");

    if (!camera.enabled) {
      RCLCPP_INFO(get_logger(), "Camera '%s' is disabled; skipping subscriptions and merge", name.c_str());
      continue;
    }

    cameras_.push_back(std::move(camera));
  }

  if (cameras_.empty()) {
    throw std::runtime_error("No enabled cameras. Check camera_names and cameras.<name>.enabled parameters");
  }
}

namespace
{

bool isUsableStamp(const rclcpp::Time & stamp)
{
  return stamp.nanoseconds() > 0;
}

std::string readFile(const std::string & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open URDF: " + path);
  }

  std::ostringstream stream;
  stream << file.rdbuf();
  return stream.str();
}

std::vector<double> parseDoubles(const std::string & text, std::size_t expected_count)
{
  std::istringstream stream(text);
  std::vector<double> values;
  double value = 0.0;

  while (stream >> value) {
    values.push_back(value);
  }

  if (values.size() != expected_count) {
    throw std::runtime_error("Invalid numeric vector in URDF: " + text);
  }

  return values;
}

}  // namespace

void PointCloudMergerNode::publishStaticTransformsFromUrdf()
{
  if (!publish_static_tf_ || urdf_path_.empty()) {
    return;
  }

  const auto urdf = readFile(urdf_path_);

  const std::regex fixed_joint_re(
    R"(<joint[^>]*type\s*=\s*["']fixed["'][^>]*>([\s\S]*?)</joint>)",
    std::regex::icase);
  const std::regex origin_re(
    R"(<origin[^>]*xyz\s*=\s*["']([^"']+)["'][^>]*rpy\s*=\s*["']([^"']+)["'][^>]*/?>)",
    std::regex::icase);
  const std::regex parent_re(
    R"(<parent[^>]*link\s*=\s*["']([^"']+)["'][^>]*/?>)",
    std::regex::icase);
  const std::regex child_re(
    R"(<child[^>]*link\s*=\s*["']([^"']+)["'][^>]*/?>)",
    std::regex::icase);

  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  auto begin = std::sregex_iterator(urdf.begin(), urdf.end(), fixed_joint_re);
  auto end = std::sregex_iterator();

  for (auto it = begin; it != end; ++it) {
    const auto joint_xml = (*it)[1].str();

    std::smatch origin_match;
    std::smatch parent_match;
    std::smatch child_match;

    if (!std::regex_search(joint_xml, origin_match, origin_re) ||
      !std::regex_search(joint_xml, parent_match, parent_re) ||
      !std::regex_search(joint_xml, child_match, child_re))
    {
      continue;
    }

    const auto xyz = parseDoubles(origin_match[1].str(), 3);
    const auto rpy = parseDoubles(origin_match[2].str(), 3);

    tf2::Quaternion quaternion;
    quaternion.setRPY(rpy[0], rpy[1], rpy[2]);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = parent_match[1].str();
    transform.child_frame_id = child_match[1].str();
    transform.transform.translation.x = xyz[0];
    transform.transform.translation.y = xyz[1];
    transform.transform.translation.z = xyz[2];
    transform.transform.rotation = tf2::toMsg(quaternion);

    transforms.push_back(transform);
  }

  if (!transforms.empty()) {
    static_tf_broadcaster_.sendTransform(transforms);
    RCLCPP_INFO(
      get_logger(),
      "Published %zu static transforms from %s",
      transforms.size(),
      urdf_path_.c_str());
  }
}

void PointCloudMergerNode::createIo()
{
  const auto input_qos = rclcpp::SensorDataQoS();

  for (const auto & camera : cameras_) {
    depth_subscriptions_.push_back(create_subscription<ImageMsg>(
      camera.depth_topic,
      input_qos,
      [this, camera_name = camera.name](ImageMsgPtr msg) {
        onDepth(camera_name, std::move(msg));
      }));

    camera_info_subscriptions_.push_back(create_subscription<CameraInfoMsg>(
      camera.camera_info_topic,
      input_qos,
      [this, camera_name = camera.name](CameraInfoMsgPtr msg) {
        onCameraInfo(camera_name, std::move(msg));
      }));

    RCLCPP_INFO(
      get_logger(),
      "Subscribing to camera '%s' depth: %s camera_info: %s mount_frame: %s optical_frame: %s",
      camera.name.c_str(),
      camera.depth_topic.c_str(),
      camera.camera_info_topic.c_str(),
      camera.mount_frame.c_str(),
      camera.optical_frame.c_str());
  }

  auto output_qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile();
  merged_cloud_pub_ = create_publisher<PointCloudMsg>("~/merged_points", output_qos);

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { onPublishTimer(); });
}

void PointCloudMergerNode::onCameraInfo(const std::string & camera_name, CameraInfoMsgPtr msg)
{
  latest_camera_infos_[camera_name] = std::move(msg);
}

void PointCloudMergerNode::onDepth(const std::string & camera_name, ImageMsgPtr msg)
{
  RCLCPP_DEBUG(
    get_logger(),
    "Received depth camera='%s' frame='%s' size=%ux%u encoding=%s",
    camera_name.c_str(),
    msg->header.frame_id.c_str(),
    msg->width,
    msg->height,
    msg->encoding.c_str());

  latest_depths_[camera_name] = std::move(msg);
}

void PointCloudMergerNode::onPublishTimer()
{
  const auto now = get_clock()->now();

  auto merged_cloud = mergeLatestClouds(now);
  if (merged_cloud.width == 0) {
    return;
  }

  merged_cloud_pub_->publish(merged_cloud);
}

PointCloudMergerNode::PointCloudMsg PointCloudMergerNode::mergeLatestClouds(
  const rclcpp::Time & now)
{
  PointCloudMsg output;
  output.header.stamp = now;
  output.header.frame_id = target_frame_;
  output.height = 1;
  output.is_bigendian = false;
  output.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(output);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(0);

  std::size_t valid_cloud_count = 0;

  for (const auto & camera : cameras_) {
    const auto depth_it = latest_depths_.find(camera.name);
    const auto info_it = latest_camera_infos_.find(camera.name);

    if (depth_it == latest_depths_.end() || !depth_it->second) {
      continue;
    }

    if (info_it == latest_camera_infos_.end() || !info_it->second) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Waiting for camera_info camera='%s'",
        camera.name.c_str());
      continue;
    }

    const auto & depth = *depth_it->second;
    const auto & camera_info = *info_it->second;

    const auto input_stamp = rclcpp::Time(depth.header.stamp);
    const auto age_sec = isUsableStamp(input_stamp) ? (now - input_stamp).seconds() : 0.0;

    if (max_cloud_age_sec_ > 0.0 && age_sec > max_cloud_age_sec_) {
      if (age_sec > 60.0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "Depth stamp is far from node clock camera='%s' frame='%s' age=%.3fs. "
          "Treating this as a clock mismatch; set max_cloud_age_sec: 0.0 or enable use_sim_time.",
          camera.name.c_str(),
          depth.header.frame_id.c_str(),
          age_sec);
      }

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Dropping stale depth camera='%s' frame='%s' age=%.3fs max_age=%.3fs",
        camera.name.c_str(),
        depth.header.frame_id.c_str(),
        age_sec,
        max_cloud_age_sec_);

      continue;
    }

    if (appendDepthAsTransformedCloud(camera, depth, camera_info, output, now)) {
      ++valid_cloud_count;
    }
  }

  if (valid_cloud_count == 0) {
    modifier.resize(0);
  }

  return output;
}

bool PointCloudMergerNode::appendDepthAsTransformedCloud(
  const CameraSource & camera,
  const ImageMsg & depth,
  const CameraInfoMsg & camera_info,
  PointCloudMsg & output,
  const rclcpp::Time & now)
{
  if (depth.encoding != sensor_msgs::image_encodings::TYPE_32FC1) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Unsupported depth encoding camera='%s': %s (expected 32FC1 meters)",
      camera.name.c_str(),
      depth.encoding.c_str());
    return false;
  }

  if (depth.step < depth.width * sizeof(float)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Invalid depth step camera='%s': step=%u width=%u",
      camera.name.c_str(),
      depth.step,
      depth.width);
    return false;
  }

  if (camera_info.k[0] == 0.0 || camera_info.k[4] == 0.0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Invalid camera intrinsics camera='%s'",
      camera.name.c_str());
    return false;
  }

  const std::string source_frame =
    camera.optical_frame.empty() ? depth.header.frame_id : camera.optical_frame;

  if (!camera.optical_frame.empty() && camera.optical_frame != depth.header.frame_id) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Depth frame_id camera='%s' is '%s', but using configured optical_frame '%s'",
      camera.name.c_str(),
      depth.header.frame_id.c_str(),
      camera.optical_frame.c_str());
  }

  geometry_msgs::msg::TransformStamped transform_msg;

  try {
    transform_msg = tf_buffer_.lookupTransform(
      target_frame_,
      source_frame,
      depth.header.stamp,
      std::chrono::milliseconds(20));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "TF unavailable from '%s' to '%s': %s",
      source_frame.c_str(),
      target_frame_.c_str(),
      ex.what());
    return false;
  }

  tf2::Transform transform;
  tf2::fromMsg(transform_msg.transform, transform);

  const auto stride = static_cast<std::uint32_t>(std::max(1, pixel_stride_));

  const auto max_candidate_count =
    static_cast<std::size_t>((depth.width + stride - 1) / stride) *
    static_cast<std::size_t>((depth.height + stride - 1) / stride);

  const auto old_point_count = static_cast<std::size_t>(output.width) * output.height;

  sensor_msgs::PointCloud2Modifier modifier(output);
  modifier.resize(old_point_count + max_candidate_count);

  sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");

  for (std::size_t i = 0; i < old_point_count; ++i) {
    ++out_x;
    ++out_y;
    ++out_z;
  }

  std::size_t written = 0;

  const auto * depth_data = reinterpret_cast<const float *>(depth.data.data());
  const auto row_step = static_cast<std::size_t>(depth.step / sizeof(float));

  const double fx = camera_info.k[0];
  const double fy = camera_info.k[4];
  const double cx = camera_info.k[2];
  const double cy = camera_info.k[5];

  for (std::uint32_t v = 0; v < depth.height; v += stride) {
    for (std::uint32_t u = 0; u < depth.width; u += stride) {
      const auto index = static_cast<std::size_t>(v) * row_step + u;
      const float z = depth_data[index];

      if (!std::isfinite(z) || z < min_range_ || z > max_range_) {
        continue;
      }

      const double x = (static_cast<double>(u) - cx) * z / fx;
      const double y = (static_cast<double>(v) - cy) * z / fy;

      const auto transformed = transform * tf2::Vector3(x, y, z);

      *out_x = static_cast<float>(transformed.x());
      *out_y = static_cast<float>(transformed.y());
      *out_z = static_cast<float>(transformed.z());

      ++out_x;
      ++out_y;
      ++out_z;
      ++written;
    }
  }

  modifier.resize(old_point_count + written);

  output.header.stamp = now;
  output.header.frame_id = target_frame_;
  output.height = 1;
  output.width = static_cast<std::uint32_t>(old_point_count + written);

  return written > 0;
}

}  // namespace height_map_ros2
