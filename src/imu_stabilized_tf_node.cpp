#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace
{

double clamp(const double value, const double min_value, const double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

bool finiteVector(const geometry_msgs::msg::Vector3 & vector)
{
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

}  // namespace

class ImuStabilizedTfNode final : public rclcpp::Node
{
public:
  ImuStabilizedTfNode()
  : Node("imu_stabilized_tf_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/robot_4w4l/imu");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "4w4l/base_link");
    stabilized_frame_id_ =
      declare_parameter<std::string>("stabilized_frame_id", "4w4l/base_stabilized");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);
    low_pass_alpha_ = clamp(declare_parameter<double>("low_pass_alpha", 0.08), 0.0, 1.0);
    min_accel_norm_ = std::max(0.1, declare_parameter<double>("min_accel_norm", 3.0));
    max_accel_norm_ = std::max(min_accel_norm_, declare_parameter<double>("max_accel_norm", 20.0));
    roll_sign_ = declare_parameter<double>("roll_sign", -1.0);
    pitch_sign_ = declare_parameter<double>("pitch_sign", -1.0);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr msg) { onImu(std::move(msg)); });

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publishTransform(); });
    heartbeat_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() { publishHeartbeat(); });
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/imu_stabilized_tf_node", 10);

    RCLCPP_INFO(
      get_logger(),
      "IMU stabilized TF started: imu=%s base=%s stabilized=%s",
      imu_topic_.c_str(),
      base_frame_id_.c_str(),
      stabilized_frame_id_.c_str());
  }

private:
  void onImu(sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto accel = accelerationInBaseFrame(*msg);
    if (!finiteVector(accel)) {
      return;
    }

    const double norm = std::hypot(accel.x, accel.y, accel.z);
    if (norm < min_accel_norm_ || norm > max_accel_norm_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Ignoring IMU acceleration norm %.3f outside [%.3f, %.3f]",
        norm,
        min_accel_norm_,
        max_accel_norm_);
      return;
    }

    const double roll = std::atan2(accel.y, accel.z);
    const double pitch = std::atan2(-accel.x, std::hypot(accel.y, accel.z));

    if (!has_estimate_) {
      roll_ = roll;
      pitch_ = pitch;
      has_estimate_ = true;
    } else {
      roll_ = (1.0 - low_pass_alpha_) * roll_ + low_pass_alpha_ * roll;
      pitch_ = (1.0 - low_pass_alpha_) * pitch_ + low_pass_alpha_ * pitch;
    }

    last_imu_stamp_ = msg->header.stamp;
    last_imu_frame_ = msg->header.frame_id;
    received_imu_ = true;
  }

  geometry_msgs::msg::Vector3 accelerationInBaseFrame(const sensor_msgs::msg::Imu & msg)
  {
    if (msg.header.frame_id.empty() || msg.header.frame_id == base_frame_id_) {
      return msg.linear_acceleration;
    }

    geometry_msgs::msg::Vector3Stamped source;
    source.header = msg.header;
    source.vector = msg.linear_acceleration;

    try {
      const auto transform = tf_buffer_.lookupTransform(
        base_frame_id_,
        msg.header.frame_id,
        rclcpp::Time(0),
        std::chrono::milliseconds(10));
      geometry_msgs::msg::Vector3Stamped target;
      tf2::doTransform(source, target, transform);
      return target.vector;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Using raw IMU acceleration; TF unavailable from '%s' to '%s': %s",
        msg.header.frame_id.c_str(),
        base_frame_id_.c_str(),
        ex.what());
      return msg.linear_acceleration;
    }
  }

  void publishTransform()
  {
    if (!has_estimate_) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = base_frame_id_;
    transform.child_frame_id = stabilized_frame_id_;
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(roll_sign_ * roll_, pitch_sign_ * pitch_, 0.0);
    q.normalize();
    transform.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(transform);
  }

  void publishHeartbeat()
  {
    std_msgs::msg::String heartbeat;
    if (!received_imu_) {
      heartbeat.data = "waiting_for_imu";
    } else if (!has_estimate_) {
      heartbeat.data = "waiting_for_valid_accel";
    } else {
      heartbeat.data = "ready:imu_frame=" + last_imu_frame_;
    }
    heartbeat_pub_->publish(heartbeat);
  }

  std::string imu_topic_;
  std::string base_frame_id_;
  std::string stabilized_frame_id_;
  std::string last_imu_frame_;
  double publish_rate_hz_{100.0};
  double low_pass_alpha_{0.08};
  double min_accel_norm_{3.0};
  double max_accel_norm_{20.0};
  double roll_sign_{-1.0};
  double pitch_sign_{-1.0};
  double roll_{0.0};
  double pitch_{0.0};
  bool received_imu_{false};
  bool has_estimate_{false};
  builtin_interfaces::msg::Time last_imu_stamp_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuStabilizedTfNode>());
  rclcpp::shutdown();
  return 0;
}
