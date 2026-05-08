#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include "height_map_ros2/msg/detected_object_array.hpp"

namespace height_map_ros2
{

class AiDetectionNode final : public rclcpp::Node
{
public:
  AiDetectionNode()
  : Node("ai_detection_node")
  {
    declare_parameter<bool>("enabled", true);
    declare_parameter<std::string>("model_path", "");
    declare_parameter<std::string>("image_topic", "/adas_camera/color/image_raw");
    declare_parameter<std::string>("camera_info_topic", "/adas_camera/color/camera_info");
    declare_parameter<std::string>("depth_topic", "/adas_camera/depth/image_rect_raw");
    declare_parameter<std::string>("annotated_image_topic", "~/annotated_image");
    declare_parameter<std::string>("detections_topic", "~/detections");
    declare_parameter<double>("confidence_threshold", 0.35);

    enabled_ = get_parameter("enabled").as_bool();
    model_path_ = get_parameter("model_path").as_string();

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      get_parameter("image_topic").as_string(),
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr msg) { onImage(std::move(msg)); });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      get_parameter("camera_info_topic").as_string(),
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) { latest_info_ = std::move(msg); });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      get_parameter("depth_topic").as_string(),
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr msg) { latest_depth_ = std::move(msg); });

    annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(
      get_parameter("annotated_image_topic").as_string(),
      10);
    detections_pub_ = create_publisher<height_map_ros2::msg::DetectedObjectArray>(
      get_parameter("detections_topic").as_string(),
      10);
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/ai_detection_node",
      10);
    heartbeat_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() { publishHeartbeat(); });
    RCLCPP_INFO(get_logger(), "C++ AI detection skeleton started");
  }

private:
  void onImage(sensor_msgs::msg::Image::SharedPtr msg)
  {
    height_map_ros2::msg::DetectedObjectArray detections;
    detections.header = msg->header;
    // TODO: Run C++ detector, draw bbox/label into annotated image, and project
    // depth to 3D object coordinates.
    detections_pub_->publish(detections);
    annotated_pub_->publish(*msg);
  }

  void publishHeartbeat()
  {
    std_msgs::msg::String msg;
    msg.data = enabled_ ? (model_path_.empty() ? "ready:no_model" : "ready") : "disabled";
    heartbeat_pub_->publish(msg);
  }

  bool enabled_{true};
  std::string model_path_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_info_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
  rclcpp::Publisher<height_map_ros2::msg::DetectedObjectArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace height_map_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<height_map_ros2::AiDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
