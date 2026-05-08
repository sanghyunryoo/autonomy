#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <System.h>

namespace height_map_ros2
{

namespace
{

double stampToSec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

double yawFromRotation(const Eigen::Matrix3f & rotation)
{
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

Sophus::SE3f transformToSophus(const geometry_msgs::msg::Transform & transform)
{
  Eigen::Quaternionf rotation(
    static_cast<float>(transform.rotation.w),
    static_cast<float>(transform.rotation.x),
    static_cast<float>(transform.rotation.y),
    static_cast<float>(transform.rotation.z));
  rotation.normalize();

  Eigen::Vector3f translation(
    static_cast<float>(transform.translation.x),
    static_cast<float>(transform.translation.y),
    static_cast<float>(transform.translation.z));

  return Sophus::SE3f(rotation, translation);
}

geometry_msgs::msg::Transform transformFromSophus(const Sophus::SE3f & transform)
{
  const auto translation = transform.translation();
  const Eigen::Quaternionf rotation = transform.unit_quaternion();

  geometry_msgs::msg::Transform msg;
  msg.translation.x = translation.x();
  msg.translation.y = translation.y();
  msg.translation.z = translation.z();
  msg.rotation.x = rotation.x();
  msg.rotation.y = rotation.y();
  msg.rotation.z = rotation.z();
  msg.rotation.w = rotation.w();
  return msg;
}

}  // namespace

class Orbslam3Node final : public rclcpp::Node
{
public:
  Orbslam3Node()
  : Node("orbslam3_node")
  {
    declare_parameter<bool>("enabled", true);
    declare_parameter<std::string>(
      "vocabulary_path",
      "/root/ros2_ws/src/height_map_ros2/third_party/orb_slam3/Vocabulary/ORBvoc.txt");
    declare_parameter<std::string>("settings_path", "");
    declare_parameter<std::string>("left_image_topic", "/stereo/left/image_rect");
    declare_parameter<std::string>("right_image_topic", "/stereo/right/image_rect");
    declare_parameter<std::string>("imu_topic", "/imu/data");
    declare_parameter<std::string>("pose_topic", "/localization/orbslam3_pose");
    declare_parameter<std::string>("pose2d_topic", "/localization/current_pose");
    declare_parameter<std::string>("path_topic", "/localization/orbslam3_path");
    declare_parameter<int>("max_path_poses", 2000);
    declare_parameter<bool>("publish_tf", true);
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("world_frame", "map");
    declare_parameter<std::string>("camera_frame", "orbslam3_camera");
    declare_parameter<bool>("use_viewer", false);
    declare_parameter<int>("sync_queue_size", 10);
    declare_parameter<double>("max_imu_buffer_sec", 2.0);

    enabled_ = get_parameter("enabled").as_bool();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    world_frame_ = get_parameter("world_frame").as_string();
    camera_frame_ = get_parameter("camera_frame").as_string();
    max_imu_buffer_sec_ = get_parameter("max_imu_buffer_sec").as_double();
    max_path_poses_ = std::max<int>(1, static_cast<int>(get_parameter("max_path_poses").as_int()));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    if (publish_tf_) {
      publishStaticMapToOdom();
    }

    pose_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(get_parameter("pose_topic").as_string(), 10);
    pose2d_pub_ =
      create_publisher<geometry_msgs::msg::Pose2D>(get_parameter("pose2d_topic").as_string(), 10);
    path_pub_ =
      create_publisher<nav_msgs::msg::Path>(get_parameter("path_topic").as_string(), 10);
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>("/autonomy/heartbeat/orbslam3_node", 10);

    if (enabled_ && startSlam()) {
      setupSubscriptions();
    }

    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { publishHeartbeat(); });
  }

  ~Orbslam3Node() override
  {
    if (slam_) {
      slam_->Shutdown();
    }
  }

private:
  using Image = sensor_msgs::msg::Image;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  bool startSlam()
  {
    const auto vocabulary_path = get_parameter("vocabulary_path").as_string();
    const auto settings_path = get_parameter("settings_path").as_string();
    const auto use_viewer = get_parameter("use_viewer").as_bool();

    if (vocabulary_path.empty() || settings_path.empty()) {
      status_ = "error: missing vocabulary_path or settings_path";
      RCLCPP_ERROR(get_logger(), "%s", status_.c_str());
      return false;
    }
    if (!std::filesystem::exists(vocabulary_path)) {
      status_ = "error: vocabulary file not found";
      RCLCPP_ERROR(get_logger(), "%s: %s", status_.c_str(), vocabulary_path.c_str());
      return false;
    }
    if (!std::filesystem::exists(settings_path)) {
      status_ = "error: settings file not found";
      RCLCPP_ERROR(get_logger(), "%s: %s", status_.c_str(), settings_path.c_str());
      return false;
    }

    try {
      slam_ = std::make_unique<ORB_SLAM3::System>(
        vocabulary_path,
        settings_path,
        ORB_SLAM3::System::IMU_STEREO,
        use_viewer);
    } catch (const std::exception & error) {
      status_ = std::string("error: failed to start ORB-SLAM3: ") + error.what();
      RCLCPP_ERROR(get_logger(), "%s", status_.c_str());
      return false;
    }

    status_ = "ready";
    return true;
  }

  void setupSubscriptions()
  {
    const auto left_topic = get_parameter("left_image_topic").as_string();
    const auto right_topic = get_parameter("right_image_topic").as_string();
    const auto imu_topic = get_parameter("imu_topic").as_string();
    const int queue_size = std::max<int>(1, static_cast<int>(get_parameter("sync_queue_size").as_int()));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr msg) { onImu(std::move(msg)); });

    left_sub_.subscribe(this, left_topic, rmw_qos_profile_sensor_data);
    right_sub_.subscribe(this, right_topic, rmw_qos_profile_sensor_data);
    sync_ = std::make_unique<Sync>(SyncPolicy(queue_size), left_sub_, right_sub_);
    sync_->registerCallback(
      std::bind(&Orbslam3Node::onStereo, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "ORB-SLAM3 stereo-IMU subscriptions: left=%s right=%s imu=%s",
      left_topic.c_str(),
      right_topic.c_str(),
      imu_topic.c_str());
  }

  void onImu(sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const double stamp = stampToSec(msg->header.stamp);
    ORB_SLAM3::IMU::Point point(
      static_cast<float>(msg->linear_acceleration.x),
      static_cast<float>(msg->linear_acceleration.y),
      static_cast<float>(msg->linear_acceleration.z),
      static_cast<float>(msg->angular_velocity.x),
      static_cast<float>(msg->angular_velocity.y),
      static_cast<float>(msg->angular_velocity.z),
      stamp);

    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_buffer_.push_back(point);
    while (!imu_buffer_.empty() && stamp - imu_buffer_.front().t > max_imu_buffer_sec_) {
      imu_buffer_.pop_front();
    }
  }

  void onStereo(const Image::ConstSharedPtr & left_msg, const Image::ConstSharedPtr & right_msg)
  {
    if (!slam_) {
      return;
    }

    cv_bridge::CvImageConstPtr left_bridge;
    cv_bridge::CvImageConstPtr right_bridge;
    try {
      left_bridge = cv_bridge::toCvShare(left_msg);
      right_bridge = cv_bridge::toCvShare(right_msg);
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge failed: %s", error.what());
      return;
    }

    const double image_stamp = stampToSec(left_msg->header.stamp);
    auto imu_measurements = takeImuUntil(image_stamp);

    Sophus::SE3f t_cw;
    try {
      t_cw = slam_->TrackStereo(left_bridge->image, right_bridge->image, image_stamp, imu_measurements);
    } catch (const std::exception & error) {
      status_ = std::string("error: tracking failed: ") + error.what();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", status_.c_str());
      return;
    }

    const int tracking_state = slam_->GetTrackingState();
    if (tracking_state != ORB_SLAM3::Tracking::OK && tracking_state != ORB_SLAM3::Tracking::OK_KLT) {
      status_ = "tracking_not_ok";
      return;
    }

    publishPose(t_cw.inverse(), left_msg->header.stamp);
    status_ = "tracking";
  }

  std::vector<ORB_SLAM3::IMU::Point> takeImuUntil(double stamp)
  {
    std::vector<ORB_SLAM3::IMU::Point> measurements;
    std::lock_guard<std::mutex> lock(imu_mutex_);
    while (!imu_buffer_.empty() && imu_buffer_.front().t <= stamp) {
      measurements.push_back(imu_buffer_.front());
      imu_buffer_.pop_front();
    }
    return measurements;
  }

  void publishPose(const Sophus::SE3f & t_wc, const builtin_interfaces::msg::Time & stamp)
  {
    const Sophus::SE3f t_wb_raw = cameraPoseToBasePose(t_wc);
    const Sophus::SE3f t_wb = alignToInitialBasePose(t_wb_raw);
    const auto translation = t_wb.translation();
    const Eigen::Quaternionf quaternion = t_wb.unit_quaternion();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.position.x = translation.x();
    pose.pose.position.y = translation.y();
    pose.pose.position.z = translation.z();
    pose.pose.orientation.x = quaternion.x();
    pose.pose.orientation.y = quaternion.y();
    pose.pose.orientation.z = quaternion.z();
    pose.pose.orientation.w = quaternion.w();

    nav_msgs::msg::Odometry odometry;
    odometry.header = pose.header;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose = pose.pose;
    fillVelocity(t_wb, stamp, odometry);
    pose_pub_->publish(odometry);

    appendAndPublishPath(pose);

    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = translation.x();
    pose2d.y = translation.y();
    pose2d.theta = yawFromRotation(t_wb.rotationMatrix());
    pose2d_pub_->publish(pose2d);

    if (publish_tf_) {
      publishOdomToBase(t_wb, stamp);
    }
  }

  void fillVelocity(
    const Sophus::SE3f & t_wb,
    const builtin_interfaces::msg::Time & stamp,
    nav_msgs::msg::Odometry & odometry)
  {
    const double current_stamp = stampToSec(stamp);
    if (!has_previous_pose_for_velocity_) {
      previous_pose_for_velocity_ = t_wb;
      previous_pose_stamp_ = current_stamp;
      has_previous_pose_for_velocity_ = true;
      return;
    }

    const double dt = current_stamp - previous_pose_stamp_;
    if (dt <= 1e-6) {
      return;
    }

    const Sophus::SE3f delta_body = previous_pose_for_velocity_.inverse() * t_wb;
    const Eigen::Vector3f linear_velocity = delta_body.translation() / static_cast<float>(dt);
    const Eigen::Vector3f angular_velocity = delta_body.so3().log() / static_cast<float>(dt);

    odometry.twist.twist.linear.x = linear_velocity.x();
    odometry.twist.twist.linear.y = linear_velocity.y();
    odometry.twist.twist.linear.z = linear_velocity.z();
    odometry.twist.twist.angular.x = angular_velocity.x();
    odometry.twist.twist.angular.y = angular_velocity.y();
    odometry.twist.twist.angular.z = angular_velocity.z();

    previous_pose_for_velocity_ = t_wb;
    previous_pose_stamp_ = current_stamp;
  }

  void appendAndPublishPath(const geometry_msgs::msg::PoseStamped & pose)
  {
    path_msg_.header = pose.header;
    path_msg_.poses.push_back(pose);
    if (path_msg_.poses.size() > static_cast<std::size_t>(max_path_poses_)) {
      path_msg_.poses.erase(path_msg_.poses.begin());
    }
    path_pub_->publish(path_msg_);
  }

  Sophus::SE3f cameraPoseToBasePose(const Sophus::SE3f & t_wc)
  {
    try {
      const auto base_to_camera = tf_buffer_->lookupTransform(
        base_frame_,
        camera_frame_,
        tf2::TimePointZero);
      const Sophus::SE3f t_bc = transformToSophus(base_to_camera.transform);
      return t_wc * t_bc.inverse();
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Cannot lookup %s -> %s, publishing camera pose as base pose: %s",
        base_frame_.c_str(),
        camera_frame_.c_str(),
        error.what());
      return t_wc;
    }
  }

  Sophus::SE3f alignToInitialBasePose(const Sophus::SE3f & t_wb)
  {
    if (!has_initial_base_pose_) {
      initial_base_pose_inverse_ = t_wb.inverse();
      has_initial_base_pose_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Aligned %s/%s to initial %s pose",
        map_frame_.c_str(),
        odom_frame_.c_str(),
        base_frame_.c_str());
    }
    return initial_base_pose_inverse_ * t_wb;
  }

  void publishStaticMapToOdom()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(transform);
  }

  void publishOdomToBase(const Sophus::SE3f & t_wb, const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform = transformFromSophus(t_wb);
    tf_broadcaster_->sendTransform(transform);
  }

  void publishHeartbeat()
  {
    std_msgs::msg::String msg;
    msg.data = enabled_ ? status_ : "disabled";
    heartbeat_pub_->publish(msg);
  }

  bool enabled_{true};
  bool publish_tf_{true};
  bool has_initial_base_pose_{false};
  bool has_previous_pose_for_velocity_{false};
  int max_path_poses_{2000};
  double max_imu_buffer_sec_{2.0};
  double previous_pose_stamp_{0.0};
  std::string status_{"disabled"};
  std::string map_frame_{"map"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string world_frame_{"map"};
  std::string camera_frame_{"orbslam3_camera"};
  Sophus::SE3f initial_base_pose_inverse_;
  Sophus::SE3f previous_pose_for_velocity_;
  nav_msgs::msg::Path path_msg_;

  std::unique_ptr<ORB_SLAM3::System> slam_;
  std::deque<ORB_SLAM3::IMU::Point> imu_buffer_;
  std::mutex imu_mutex_;

  message_filters::Subscriber<Image> left_sub_;
  message_filters::Subscriber<Image> right_sub_;
  std::unique_ptr<Sync> sync_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose2d_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

}  // namespace height_map_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<height_map_ros2::Orbslam3Node>());
  rclcpp::shutdown();
  return 0;
}
