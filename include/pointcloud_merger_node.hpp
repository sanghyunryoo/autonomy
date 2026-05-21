#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
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
  std::string mount_frame;
  std::string optical_frame;
  std::vector<double> mount_to_optical_xyz{0.0, 0.0, 0.0};
  std::vector<double> mount_to_optical_rpy{-1.5707963267948966, 0.0, -1.5707963267948966};
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
  void publishStaticTransformsFromUrdf();
  void publishHeartbeat();
  void onAutonomyState(autonomy::msg::AutonomyState::SharedPtr msg);
  void onCameraInfo(const std::string & camera_name, CameraInfoMsgPtr msg);
  void onDepth(const std::string & camera_name, ImageMsgPtr msg);
  void onPublishTimer();
  [[nodiscard]] bool processingActive() const;

  [[nodiscard]] PointCloudMsg mergeLatestClouds(const rclcpp::Time & now);
  [[nodiscard]] bool appendDepthAsTransformedCloud(
    const CameraSource & camera,
    const ImageMsg & depth,
    const CameraInfoMsg & camera_info,
    PointCloudMsg & output,
    const rclcpp::Time & now);

  std::string target_frame_{"base_link"};
  std::string urdf_path_;
  std::string static_tf_frame_prefix_;
  double publish_rate_hz_{20.0};
  double max_cloud_age_sec_{0.20};
  double min_range_{0.05};
  double max_range_{2.5};
  int pixel_stride_{2};
  bool respect_autonomy_mode_{false};
  bool has_autonomy_state_{false};
  int8_t autonomy_mode_{autonomy::msg::AutonomyState::IDLE};
  std::string autonomy_status_topic_{"/autonomy_manager/status"};
  std::vector<CameraSource> cameras_;

  std::unordered_map<std::string, ImageMsgPtr> latest_depths_;
  std::unordered_map<std::string, CameraInfoMsgPtr> latest_camera_infos_;
  rclcpp::Subscription<autonomy::msg::AutonomyState>::SharedPtr autonomy_sub_;
  std::vector<rclcpp::Subscription<ImageMsg>::SharedPtr> depth_subscriptions_;
  std::vector<rclcpp::Subscription<CameraInfoMsg>::SharedPtr> camera_info_subscriptions_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr merged_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
};

}  // namespace autonomy
