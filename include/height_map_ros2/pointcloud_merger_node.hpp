#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace height_map_ros2
{

struct CameraSource
{
  std::string name;
  std::string depth_topic;
  std::string camera_info_topic;
  std::string mount_frame;
  std::string optical_frame;
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
  void onCameraInfo(const std::string & camera_name, CameraInfoMsgPtr msg);
  void onDepth(const std::string & camera_name, ImageMsgPtr msg);
  void onPublishTimer();

  [[nodiscard]] PointCloudMsg mergeLatestClouds(const rclcpp::Time & now);
  [[nodiscard]] bool appendDepthAsTransformedCloud(
    const CameraSource & camera,
    const ImageMsg & depth,
    const CameraInfoMsg & camera_info,
    PointCloudMsg & output,
    const rclcpp::Time & now);

  std::string target_frame_{"base_link"};
  std::string urdf_path_;
  bool publish_static_tf_{true};
  double publish_rate_hz_{20.0};
  double max_cloud_age_sec_{0.20};
  double min_range_{0.05};
  double max_range_{2.5};
  int pixel_stride_{2};
  std::vector<CameraSource> cameras_;

  std::unordered_map<std::string, ImageMsgPtr> latest_depths_;
  std::unordered_map<std::string, CameraInfoMsgPtr> latest_camera_infos_;
  std::vector<rclcpp::Subscription<ImageMsg>::SharedPtr> depth_subscriptions_;
  std::vector<rclcpp::Subscription<CameraInfoMsg>::SharedPtr> camera_info_subscriptions_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr merged_cloud_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
};

}  // namespace height_map_ros2
