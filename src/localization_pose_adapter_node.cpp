#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace autonomy
{

class LocalizationPoseAdapterNode final : public rclcpp::Node
{
public:
  LocalizationPoseAdapterNode()
  : Node("localization_pose_adapter_node")
  {
    declare_parameter<std::string>("odom_topic", "/rtabmap/odom");
    declare_parameter<std::string>("pose2d_topic", "/localization/current_pose");
    declare_parameter<std::string>("path_topic", "/localization/trajectory");
    declare_parameter<std::string>("heartbeat_name", "rtabmap_localization_node");
    declare_parameter<std::string>("odom_frame_id", "odom");
    declare_parameter<std::string>("base_frame_id", "4w4l/base_footprint");
    declare_parameter<std::string>("reset_odom_service", "/reset_odom");
    declare_parameter<double>("publish_rate_hz", 10.0);
    declare_parameter<double>("odom_timeout_sec", 0.5);
    declare_parameter<double>("recovery_timeout_sec", 1.0);
    declare_parameter<double>("recovery_cooldown_sec", 3.0);
    declare_parameter<double>("path_min_distance", 0.03);
    declare_parameter<int>("path_max_poses", 2000);
    declare_parameter<int>("invalid_odom_recovery_threshold", 3);

    heartbeat_name_ = get_parameter("heartbeat_name").as_string();
    odom_frame_id_ = get_parameter("odom_frame_id").as_string();
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    reset_odom_service_ = get_parameter("reset_odom_service").as_string();
    odom_timeout_sec_ = std::max(0.0, get_parameter("odom_timeout_sec").as_double());
    recovery_timeout_sec_ = std::max(0.0, get_parameter("recovery_timeout_sec").as_double());
    recovery_cooldown_sec_ = std::max(0.0, get_parameter("recovery_cooldown_sec").as_double());
    path_min_distance_ = std::max(0.0, get_parameter("path_min_distance").as_double());
    path_max_poses_ = std::max(1, static_cast<int>(get_parameter("path_max_poses").as_int()));
    invalid_odom_recovery_threshold_ =
      std::max(1, static_cast<int>(get_parameter("invalid_odom_recovery_threshold").as_int()));
    current_transform_.transform.rotation.w = 1.0;
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    reset_odom_client_ = create_client<std_srvs::srv::Empty>(reset_odom_service_);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(),
      20,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        updateFromOdom(*msg);
      });

    pose2d_pub_ = create_publisher<geometry_msgs::msg::Pose2D>(
      get_parameter("pose2d_topic").as_string(),
      10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      get_parameter("path_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local());
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/" + heartbeat_name_,
      10);

    const auto period = std::chrono::duration<double>(
      1.0 / std::max(1.0, get_parameter("publish_rate_hz").as_double()));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { tick(); });
  }

private:
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  void updateFromOdom(const nav_msgs::msg::Odometry & odom)
  {
    if (!isValidPose(odom)) {
      has_invalid_odom_ = true;
      rejected_odom_count_++;
      consecutive_invalid_odom_count_++;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Ignoring invalid odometry pose from RTAB-Map: rejected=%zu",
        rejected_odom_count_);
      return;
    }

    consecutive_invalid_odom_count_ = 0;
    current_pose_.x = odom.pose.pose.position.x;
    current_pose_.y = odom.pose.pose.position.y;
    current_pose_.theta = yawFromQuaternion(odom.pose.pose.orientation);
    current_transform_.transform.translation.x = odom.pose.pose.position.x;
    current_transform_.transform.translation.y = odom.pose.pose.position.y;
    current_transform_.transform.translation.z = odom.pose.pose.position.z;
    current_transform_.transform.rotation = normalizedQuaternion(odom.pose.pose.orientation);
    latest_odom_time_ = now();
    has_odom_ = true;
    has_invalid_odom_ = false;
    appendPathPose(odom);
  }

  void tick()
  {
    publishPose2d();
    publishTf();
    publishPath();
    maybeRecoverOdometry();
    publishHeartbeat();
  }

  void publishPose2d()
  {
    pose2d_pub_->publish(current_pose_);
  }

  void publishTf()
  {
    current_transform_.header.stamp = now();
    current_transform_.header.frame_id = odom_frame_id_;
    current_transform_.child_frame_id = base_frame_id_;
    tf_broadcaster_->sendTransform(current_transform_);
  }

  void appendPathPose(const nav_msgs::msg::Odometry & odom)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = odom.header.stamp;
    if (pose.header.stamp.sec == 0 && pose.header.stamp.nanosec == 0) {
      pose.header.stamp = now();
    }
    pose.header.frame_id = odom_frame_id_;
    pose.pose = odom.pose.pose;
    pose.pose.orientation = normalizedQuaternion(pose.pose.orientation);

    if (!trajectory_.poses.empty()) {
      const auto & previous = trajectory_.poses.back().pose.position;
      const auto & current = pose.pose.position;
      const double dx = current.x - previous.x;
      const double dy = current.y - previous.y;
      const double dz = current.z - previous.z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) < path_min_distance_) {
        return;
      }
    }

    trajectory_.header.frame_id = odom_frame_id_;
    trajectory_.poses.push_back(pose);
    while (static_cast<int>(trajectory_.poses.size()) > path_max_poses_) {
      trajectory_.poses.erase(trajectory_.poses.begin());
    }
  }

  void publishPath()
  {
    trajectory_.header.stamp = now();
    path_pub_->publish(trajectory_);
  }

  bool odomTimedOut() const
  {
    if (!has_odom_ || odom_timeout_sec_ <= 0.0) {
      return false;
    }
    return (now() - latest_odom_time_).seconds() > odom_timeout_sec_;
  }

  bool recoveryTimedOut() const
  {
    if (!has_odom_ || recovery_timeout_sec_ <= 0.0) {
      return false;
    }
    return (now() - latest_odom_time_).seconds() > recovery_timeout_sec_;
  }

  bool recoveryCooldownActive() const
  {
    if (!has_recovery_request_) {
      return false;
    }
    return (now() - last_recovery_request_time_).seconds() < recovery_cooldown_sec_;
  }

  void maybeRecoverOdometry()
  {
    const bool invalid_recovery =
      consecutive_invalid_odom_count_ >= invalid_odom_recovery_threshold_;
    const bool timeout_recovery = recoveryTimedOut();
    if (!invalid_recovery && !timeout_recovery) {
      return;
    }
    if (recoveryCooldownActive()) {
      return;
    }
    if (!reset_odom_client_->service_is_ready()) {
      recovery_waiting_for_service_ = true;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Odometry recovery requested, but reset service is not ready: %s",
        reset_odom_service_.c_str());
      return;
    }

    recovery_waiting_for_service_ = false;
    has_recovery_request_ = true;
    last_recovery_request_time_ = now();
    recovery_request_count_++;
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    reset_odom_client_->async_send_request(request);
    RCLCPP_WARN(
      get_logger(),
      "Requested RTAB-Map odometry recovery via %s (count=%zu reason=%s)",
      reset_odom_service_.c_str(),
      recovery_request_count_,
      timeout_recovery ? "timeout" : "invalid_odom");
  }

  static bool isFinite(const double value)
  {
    return std::isfinite(value);
  }

  static bool isValidPose(const nav_msgs::msg::Odometry & odom)
  {
    const auto & p = odom.pose.pose.position;
    const auto & q = odom.pose.pose.orientation;
    if (!isFinite(p.x) || !isFinite(p.y) || !isFinite(p.z) ||
      !isFinite(q.x) || !isFinite(q.y) || !isFinite(q.z) || !isFinite(q.w))
    {
      return false;
    }

    const double norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(norm_sq) && norm_sq > 1.0e-12;
  }

  static geometry_msgs::msg::Quaternion normalizedQuaternion(
    const geometry_msgs::msg::Quaternion & input)
  {
    auto output = input;
    const double norm = std::sqrt(
      output.x * output.x + output.y * output.y + output.z * output.z + output.w * output.w);
    if (norm <= 1.0e-12 || !std::isfinite(norm)) {
      output.x = 0.0;
      output.y = 0.0;
      output.z = 0.0;
      output.w = 1.0;
      return output;
    }
    output.x /= norm;
    output.y /= norm;
    output.z /= norm;
    output.w /= norm;
    return output;
  }

  void publishHeartbeat()
  {
    std_msgs::msg::String heartbeat;
    if (!has_odom_) {
      heartbeat.data = "holding_identity";
    } else if (has_invalid_odom_) {
      heartbeat.data = "holding_last_odom:invalid_odom";
    } else if (odomTimedOut()) {
      heartbeat.data = "holding_last_odom";
    } else {
      heartbeat.data = "tracking";
    }
    if (recovery_waiting_for_service_) {
      heartbeat.data += ":recovery_service_unavailable";
    } else if (recoveryCooldownActive()) {
      heartbeat.data += ":recovery_requested";
    }
    heartbeat_pub_->publish(heartbeat);
  }

  std::string heartbeat_name_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string reset_odom_service_;
  double odom_timeout_sec_{0.5};
  double recovery_timeout_sec_{1.0};
  double recovery_cooldown_sec_{3.0};
  double path_min_distance_{0.03};
  int path_max_poses_{2000};
  int invalid_odom_recovery_threshold_{3};
  bool has_odom_{false};
  bool has_invalid_odom_{false};
  bool has_recovery_request_{false};
  bool recovery_waiting_for_service_{false};
  std::size_t rejected_odom_count_{0};
  std::size_t consecutive_invalid_odom_count_{0};
  std::size_t recovery_request_count_{0};
  rclcpp::Time latest_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_recovery_request_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Pose2D current_pose_;
  geometry_msgs::msg::TransformStamped current_transform_;
  nav_msgs::msg::Path trajectory_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose2d_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_odom_client_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::LocalizationPoseAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
