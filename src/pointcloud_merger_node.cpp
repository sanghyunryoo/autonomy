#include "pointcloud_merger_node.hpp"

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
#include <tf2/LinearMath/Quaternion.h>
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
  static_tf_frame_prefix_ =
    declare_parameter<std::string>("static_tf_frame_prefix", static_tf_frame_prefix_);
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
    camera.mount_to_optical_xyz = declare_parameter<std::vector<double>>(
      "cameras." + name + ".mount_to_optical_xyz", camera.mount_to_optical_xyz);
    camera.mount_to_optical_rpy = declare_parameter<std::vector<double>>(
      "cameras." + name + ".mount_to_optical_rpy", camera.mount_to_optical_rpy);

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

std::string addFramePrefix(const std::string & prefix, const std::string & frame)
{
  if (prefix.empty() || frame.empty() || frame.find('/') != std::string::npos) {
    return frame;
  }
  return prefix + frame;
}

std::string unprefixedFrame(const std::string & frame)
{
  const auto slash = frame.rfind('/');
  if (slash == std::string::npos) {
    return frame;
  }
  return frame.substr(slash + 1);
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

bool sameFrame(const std::string & lhs, const std::string & rhs)
{
  return unprefixedFrame(lhs) == unprefixedFrame(rhs);
}

tf2::Transform transformFromXyzRpy(
  const std::vector<double> & xyz,
  const std::vector<double> & rpy)
{
  if (xyz.size() != 3 || rpy.size() != 3) {
    throw std::runtime_error("mount_to_optical_xyz/rpy must each contain exactly 3 values");
  }

  tf2::Quaternion quaternion;
  quaternion.setRPY(rpy[0], rpy[1], rpy[2]);
  return tf2::Transform(quaternion, tf2::Vector3(xyz[0], xyz[1], xyz[2]));
}

geometry_msgs::msg::TransformStamped makeTransformStamped(
  const std::string & parent,
  const std::string & child,
  const tf2::Transform & transform,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = parent;
  msg.child_frame_id = child;
  msg.transform = tf2::toMsg(transform);
  return msg;
}

struct FixedJointTransform
{
  std::string parent_frame;
  std::string child_frame;
  tf2::Transform transform;
};

std::vector<FixedJointTransform> parseFixedJointTransformsFromUrdf(const std::string & urdf)
{
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

  std::vector<FixedJointTransform> joints;
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
    joints.push_back({
      parent_match[1].str(),
      child_match[1].str(),
      tf2::Transform(quaternion, tf2::Vector3(xyz[0], xyz[1], xyz[2])),
    });
  }

  return joints;
}

void appendPrefixedTransform(
  const geometry_msgs::msg::TransformStamped & transform,
  const std::string & prefix,
  std::vector<geometry_msgs::msg::TransformStamped> & transforms)
{
  if (prefix.empty()) {
    transforms.push_back(transform);
    return;
  }

  auto prefixed_transform = transform;
  prefixed_transform.header.frame_id = addFramePrefix(prefix, transform.header.frame_id);
  prefixed_transform.child_frame_id = addFramePrefix(prefix, transform.child_frame_id);
  transforms.push_back(prefixed_transform);
}

}  // namespace

void PointCloudMergerNode::publishStaticTransformsFromUrdf()
{
  if (urdf_path_.empty()) {
    RCLCPP_WARN(get_logger(), "No urdf_path provided; static camera TF will not be published");
    return;
  }

  const auto joints = parseFixedJointTransformsFromUrdf(readFile(urdf_path_));
  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  for (const auto & joint : joints) {
    std::vector<geometry_msgs::msg::TransformStamped> transforms_for_joint;
    bool replaced_by_mount_frame = false;

    for (const auto & camera : cameras_) {
      if (camera.mount_frame.empty() || camera.optical_frame.empty() ||
        sameFrame(camera.mount_frame, camera.optical_frame) ||
        !sameFrame(camera.optical_frame, joint.child_frame))
      {
        continue;
      }

      const auto mount_to_optical =
        transformFromXyzRpy(camera.mount_to_optical_xyz, camera.mount_to_optical_rpy);
      const auto parent_to_mount = joint.transform * mount_to_optical.inverse();

      transforms_for_joint.push_back(makeTransformStamped(
        joint.parent_frame, camera.mount_frame, parent_to_mount, now()));
      transforms_for_joint.push_back(makeTransformStamped(
        camera.mount_frame, camera.optical_frame, mount_to_optical, now()));
      replaced_by_mount_frame = true;
      break;
    }

    if (!replaced_by_mount_frame) {
      transforms_for_joint.push_back(makeTransformStamped(
        joint.parent_frame,
        joint.child_frame,
        joint.transform,
        now()));
    }

    for (const auto & transform : transforms_for_joint) {
      appendPrefixedTransform(transform, static_tf_frame_prefix_, transforms);
    }
  }

  if (!transforms.empty()) {
    const std::string prefix_log = static_tf_frame_prefix_.empty() ?
      "" : " with frame prefix '" + static_tf_frame_prefix_ + "'";
    static_tf_broadcaster_.sendTransform(transforms);
    RCLCPP_INFO(
      get_logger(),
      "Published %zu static transforms from %s%s",
      transforms.size(),
      urdf_path_.c_str(),
      prefix_log.c_str());
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
  heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
    "/autonomy/heartbeat/pointcloud_merge_node", 10);

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { onPublishTimer(); });
  heartbeat_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    [this]() { publishHeartbeat(); });
}

void PointCloudMergerNode::publishHeartbeat()
{
  std_msgs::msg::String msg;
  msg.data = cameras_.empty() ? "degraded:no_cameras" : "ready";
  heartbeat_pub_->publish(msg);
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
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Waiting for depth image camera='%s' topic='%s'",
        camera.name.c_str(),
        camera.depth_topic.c_str());
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

  if (written == 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Depth image produced no valid points camera='%s' frame='%s' encoding='%s' "
      "range=[%.3f, %.3f] stride=%d",
      camera.name.c_str(),
      depth.header.frame_id.c_str(),
      depth.encoding.c_str(),
      min_range_,
      max_range_,
      pixel_stride_);
  }

  output.header.stamp = now;
  output.header.frame_id = target_frame_;
  output.height = 1;
  output.width = static_cast<std::uint32_t>(old_point_count + written);

  return written > 0;
}

}  // namespace height_map_ros2
