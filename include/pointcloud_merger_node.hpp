#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "autonomy/msg/autonomy_state.hpp"

namespace autonomy
{

struct CameraSource
{
  std::string name;
  bool enabled{true};
  std::string depth_topic;
  std::string camera_info_topic;
  std::string frame;
};

struct LidarSource
{
  std::string name;
  bool enabled{true};
  std::string cloud_topic;
  std::string frame_id;
};

class PointCloudMergerNode final : public rclcpp::Node
{
public:
  explicit PointCloudMergerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;
  using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
  using CameraInfoMsgPtr = sensor_msgs::msg::CameraInfo::SharedPtr;
  using ImageMsg = sensor_msgs::msg::Image;
  using ImageMsgPtr = sensor_msgs::msg::Image::SharedPtr;

  void loadParameters();
  void createIo();
  void publishHeartbeat();
  void onAutonomyState(autonomy::msg::AutonomyState::SharedPtr msg);
  void onCameraInfo(const std::string & camera_name, CameraInfoMsgPtr msg);
  void onDepth(const std::string & camera_name, ImageMsgPtr msg);
  void onLidarCloud(const std::string & lidar_name, PointCloudMsg::SharedPtr msg);
  void onAttitudeCorrection(geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void onPublishTimer();
  [[nodiscard]] bool processingActive() const;

  [[nodiscard]] PointCloudMsg mergeLatestClouds(const rclcpp::Time & now);
  [[nodiscard]] bool appendDepthAsTransformedCloud(
    const CameraSource & camera,
    const ImageMsg & depth,
    const CameraInfoMsg & camera_info,
    PointCloudMsg & output,
    const rclcpp::Time & now);
  [[nodiscard]] bool appendLidarAsTransformedCloud(
    const LidarSource & lidar,
    const PointCloudMsg & cloud,
    PointCloudMsg & output,
    const rclcpp::Time & now);
  [[nodiscard]] bool lookupTransformWithLatestFallback(
    const std::string & source_frame,
    const builtin_interfaces::msg::Time & stamp,
    const char * source_label,
    geometry_msgs::msg::TransformStamped & transform_msg);

  std::string target_frame_{"base_link"};
  double publish_rate_hz_{20.0};
  double max_cloud_age_sec_{0.20};
  double min_range_{0.05};
  double max_range_{2.5};
  int pixel_stride_{2};
  bool respect_autonomy_mode_{false};
  bool attitude_correction_enabled_{true};
  bool has_attitude_correction_{false};
  double attitude_roll_{0.0};
  double attitude_pitch_{0.0};
  std::string attitude_correction_topic_{"/drive_mapper_node/attitude_correction"};
  bool has_autonomy_state_{false};
  int8_t autonomy_mode_{autonomy::msg::AutonomyState::IDLE};
  std::string autonomy_status_topic_{"/autonomy_manager/status"};
  std::vector<CameraSource> cameras_;
  std::vector<LidarSource> lidars_;

  std::unordered_map<std::string, ImageMsgPtr> latest_depths_;
  std::unordered_map<std::string, CameraInfoMsgPtr> latest_camera_infos_;
  std::unordered_map<std::string, PointCloudMsg::SharedPtr> latest_lidar_clouds_;
  rclcpp::Subscription<autonomy::msg::AutonomyState>::SharedPtr autonomy_sub_;
  std::vector<rclcpp::Subscription<ImageMsg>::SharedPtr> depth_subscriptions_;
  std::vector<rclcpp::Subscription<CameraInfoMsg>::SharedPtr> camera_info_subscriptions_;
  std::vector<rclcpp::Subscription<PointCloudMsg>::SharedPtr> lidar_subscriptions_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_correction_sub_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr merged_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace autonomy
