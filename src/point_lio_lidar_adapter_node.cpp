#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

namespace autonomy
{
namespace
{

class PointLioLidarAdapterNode final : public rclcpp::Node
{
public:
  PointLioLidarAdapterNode()
  : Node("point_lio_lidar_adapter_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/f16/livox/lidar");
    output_topic_ = declare_parameter<std::string>("output_topic", "/point_lio/lidar");
    const int ring_param = static_cast<int>(declare_parameter<int>("ring", 0));
    ring_ = static_cast<std::uint16_t>(std::clamp(ring_param, 0, static_cast<int>(UINT16_MAX)));
    synthetic_scan_period_ = std::max(0.0, declare_parameter<double>("synthetic_scan_period", 0.1));
    estimate_scan_period_ = declare_parameter<bool>("estimate_scan_period", true);
    min_scan_period_ = std::max(0.001, declare_parameter<double>("min_scan_period", 0.02));
    max_scan_period_ = std::max(min_scan_period_, declare_parameter<double>("max_scan_period", 0.25));
    period_filter_alpha_ = std::clamp(declare_parameter<double>("period_filter_alpha", 0.2), 0.0, 1.0);

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(4)).reliable().durability_volatile());
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/point_lio_lidar_adapter_node", 10);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { onCloud(std::move(msg)); });
    heartbeat_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() { publishHeartbeat(); });

    RCLCPP_INFO(
      get_logger(),
      "Point-LIO lidar adapter: %s -> %s ring=%u scan_period=%.3f",
      input_topic_.c_str(),
      output_topic_.c_str(),
      ring_,
      synthetic_scan_period_);
  }

private:
  static bool hasField(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
  {
    return std::any_of(
      cloud.fields.begin(),
      cloud.fields.end(),
      [&name](const sensor_msgs::msg::PointField & field) { return field.name == name; });
  }

  void onCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto point_count = static_cast<std::size_t>(msg->width) * msg->height;
    const double scan_period = scanPeriodFromStamp(msg->header.stamp);
    const bool has_intensity = hasField(*msg, "intensity");
    const bool has_time = hasField(*msg, "time");
    const bool has_ring = hasField(*msg, "ring");
    const bool has_line = hasField(*msg, "line");
    sensor_msgs::msg::PointCloud2 output;
    output.header = msg->header;
    output.height = 1;
    output.width = static_cast<std::uint32_t>(point_count);
    output.is_bigendian = false;
    output.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2Fields(
      6,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "time", 1, sensor_msgs::msg::PointField::FLOAT32,
      "ring", 1, sensor_msgs::msg::PointField::UINT16);
    modifier.resize(point_count);

    try {
      sensor_msgs::PointCloud2ConstIterator<float> in_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> in_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> in_z(*msg, "z");
      std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> in_intensity;
      std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> in_time;
      std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<std::uint16_t>> in_ring;
      std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<std::uint8_t>> in_line;
      if (has_intensity) {
        in_intensity = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*msg, "intensity");
      }
      if (has_time) {
        in_time = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*msg, "time");
      }
      if (has_ring) {
        in_ring = std::make_unique<sensor_msgs::PointCloud2ConstIterator<std::uint16_t>>(*msg, "ring");
      } else if (has_line) {
        in_line = std::make_unique<sensor_msgs::PointCloud2ConstIterator<std::uint8_t>>(*msg, "line");
      }
      sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
      sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
      sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");
      sensor_msgs::PointCloud2Iterator<float> out_intensity(output, "intensity");
      sensor_msgs::PointCloud2Iterator<float> out_time(output, "time");
      sensor_msgs::PointCloud2Iterator<std::uint16_t> out_ring(output, "ring");

      for (std::size_t i = 0; i < point_count; ++i) {
        *out_x = *in_x;
        *out_y = *in_y;
        *out_z = *in_z;
        *out_intensity = in_intensity ? **in_intensity : 0.0F;
        *out_time = in_time ? **in_time : point_count > 1 ?
          static_cast<float>(scan_period * static_cast<double>(i) /
            static_cast<double>(point_count - 1)) :
          0.0F;
        *out_ring = in_ring ? **in_ring :
          static_cast<std::uint16_t>(in_line ? **in_line : ring_);

        ++in_x;
        ++in_y;
        ++in_z;
        if (in_intensity) {
          ++(*in_intensity);
        }
        if (in_time) {
          ++(*in_time);
        }
        if (in_ring) {
          ++(*in_ring);
        }
        if (in_line) {
          ++(*in_line);
        }
        ++out_x;
        ++out_y;
        ++out_z;
        ++out_intensity;
        ++out_time;
        ++out_ring;
      }
    } catch (const std::runtime_error & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Input cloud is missing xyz fields: %s",
        ex.what());
      return;
    }

    pub_->publish(output);
    last_input_time_ = now();
    converted_count_++;
  }

  void publishHeartbeat()
  {
    std_msgs::msg::String msg;
    if (converted_count_ == 0) {
      msg.data = "waiting_for_lidar";
    } else if ((now() - last_input_time_) > rclcpp::Duration::from_seconds(2.0)) {
      msg.data = "degraded:stale_lidar";
    } else {
      msg.data = "ready:converted=" + std::to_string(converted_count_) +
        ":period=" + shortText(filtered_scan_period_);
    }
    heartbeat_pub_->publish(msg);
  }

  double scanPeriodFromStamp(const builtin_interfaces::msg::Time & stamp)
  {
    if (!estimate_scan_period_) {
      return synthetic_scan_period_;
    }

    const rclcpp::Time current(stamp);
    double period = filtered_scan_period_;
    if (last_cloud_stamp_.nanoseconds() > 0 && current > last_cloud_stamp_) {
      const double observed = (current - last_cloud_stamp_).seconds();
      if (std::isfinite(observed) && observed >= min_scan_period_ && observed <= max_scan_period_) {
        period = period_filter_alpha_ * observed + (1.0 - period_filter_alpha_) * filtered_scan_period_;
        filtered_scan_period_ = period;
      }
    }
    last_cloud_stamp_ = current;
    return std::clamp(period, min_scan_period_, max_scan_period_);
  }

  static std::string shortText(const double value)
  {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.3f", value);
    return std::string(buffer);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::uint16_t ring_{0};
  double synthetic_scan_period_{0.1};
  bool estimate_scan_period_{true};
  double min_scan_period_{0.02};
  double max_scan_period_{0.25};
  double period_filter_alpha_{0.2};
  double filtered_scan_period_{0.1};
  std::uint64_t converted_count_{0};
  rclcpp::Time last_input_time_{0, 0u, RCL_SYSTEM_TIME};
  rclcpp::Time last_cloud_stamp_{0, 0u, RCL_SYSTEM_TIME};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace
}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::PointLioLidarAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
