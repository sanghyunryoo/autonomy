#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

namespace autonomy
{

namespace
{

bool hasField(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  return std::any_of(
    cloud.fields.begin(),
    cloud.fields.end(),
    [&name](const sensor_msgs::msg::PointField & field) { return field.name == name; });
}

std::size_t pointCount(const sensor_msgs::msg::PointCloud2 & cloud)
{
  return static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
}

double toSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec + stamp.nanosec * 1.0e-9;
}

double normalizeTimestamp(const double raw_timestamp, const double header_time)
{
  if (!std::isfinite(raw_timestamp) || raw_timestamp <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double candidates[] = {
    raw_timestamp,
    raw_timestamp * 1.0e-3,
    raw_timestamp * 1.0e-6,
    raw_timestamp * 1.0e-9,
  };
  double best = std::numeric_limits<double>::quiet_NaN();
  double best_error = std::numeric_limits<double>::infinity();
  for (const auto candidate : candidates) {
    const auto error = std::abs(candidate - header_time);
    if (std::isfinite(candidate) && error < best_error) {
      best = candidate;
      best_error = error;
    }
  }
  return best_error < 60.0 ? best : std::numeric_limits<double>::quiet_NaN();
}

bool saneTimestamp(const double timestamp, const double reference_time)
{
  return std::isfinite(timestamp) && timestamp > 0.0 && std::abs(timestamp - reference_time) < 60.0;
}

}  // namespace

class LivoxPointcloudAdapterNode final : public rclcpp::Node
{
public:
  LivoxPointcloudAdapterNode()
  : Node("livox_pointcloud_adapter_node")
  {
    declare_parameter<bool>("enabled", true);
    declare_parameter<std::string>("input_topic", "/livox/lidar");
    declare_parameter<std::string>("output_topic", "/point_lio/lidar");
    declare_parameter<std::string>("output_frame_id", "");
    declare_parameter<int>("scan_lines", 4);
    declare_parameter<bool>("filter_invalid_returns", true);
    declare_parameter<double>("publish_rate_hz", 5.0);

    enabled_ = get_parameter("enabled").as_bool();
    output_frame_id_ = get_parameter("output_frame_id").as_string();
    scan_lines_ = std::max(1, static_cast<int>(get_parameter("scan_lines").as_int()));
    filter_invalid_returns_ = get_parameter("filter_invalid_returns").as_bool();

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, rclcpp::SensorDataQoS());
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/livox_pointcloud_adapter_node",
      10);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { onCloud(msg); });

    const auto period = std::chrono::duration<double>(
      1.0 / std::max(1.0, get_parameter("publish_rate_hz").as_double()));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publishHeartbeat(); });
  }

private:
  struct InputPoint
  {
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
    float intensity{0.0F};
    float time{0.0F};
    std::uint16_t ring{0U};
  };

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    if (!enabled_) {
      return;
    }
    if (!validateInput(*msg)) {
      return;
    }

    std::vector<InputPoint> points;
    points.reserve(pointCount(*msg));
    const double receive_time = now().seconds();
    const double message_header_time = toSeconds(msg->header.stamp);
    const double reference_time =
      saneTimestamp(message_header_time, receive_time) ? message_header_time : receive_time;
    double scan_start_time = std::numeric_limits<double>::infinity();

    {
      sensor_msgs::PointCloud2ConstIterator<double> timestamp_it(*msg, "timestamp");
      const auto total = pointCount(*msg);
      for (std::size_t i = 0; i < total; ++i, ++timestamp_it) {
        const auto timestamp = normalizeTimestamp(*timestamp_it, reference_time);
        if (std::isfinite(timestamp) && timestamp > 0.0) {
          scan_start_time = std::min(scan_start_time, timestamp);
        }
      }
    }
    if (!std::isfinite(scan_start_time)) {
      scan_start_time = reference_time;
    }

    sensor_msgs::PointCloud2ConstIterator<float> x_it(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_it(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_it(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> intensity_it(*msg, "intensity");
    sensor_msgs::PointCloud2ConstIterator<double> timestamp_it(*msg, "timestamp");
    sensor_msgs::PointCloud2ConstIterator<std::uint8_t> line_it(*msg, "line");

    const bool has_tag = hasField(*msg, "tag");
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<std::uint8_t>> tag_it;
    if (has_tag) {
      tag_it = std::make_unique<sensor_msgs::PointCloud2ConstIterator<std::uint8_t>>(*msg, "tag");
    }

    const auto total = pointCount(*msg);
    for (std::size_t i = 0; i < total; ++i, ++x_it, ++y_it, ++z_it, ++intensity_it, ++timestamp_it, ++line_it) {
      if (tag_it) {
        const auto tag = static_cast<std::uint8_t>(**tag_it);
        ++(*tag_it);
        const auto return_type = tag & 0x30U;
        if (filter_invalid_returns_ && return_type != 0x00U && return_type != 0x10U) {
          continue;
        }
      }

      const auto line = static_cast<std::uint16_t>(*line_it);
      if (line >= static_cast<std::uint16_t>(scan_lines_)) {
        continue;
      }

      InputPoint point;
      point.x = *x_it;
      point.y = *y_it;
      point.z = *z_it;
      point.intensity = *intensity_it;
      const auto timestamp = normalizeTimestamp(*timestamp_it, reference_time);
      const auto offset = timestamp - scan_start_time;
      point.time = static_cast<float>(std::isfinite(offset) && offset >= 0.0 ? offset : 0.0);
      point.ring = line;
      points.push_back(point);
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    const auto scan_start_sec = static_cast<std::int32_t>(
      std::clamp(std::floor(scan_start_time), 0.0, static_cast<double>(std::numeric_limits<std::int32_t>::max())));
    const auto scan_start_nsec = static_cast<std::uint32_t>(
      std::clamp((scan_start_time - static_cast<double>(scan_start_sec)) * 1.0e9, 0.0, 999999999.0));
    out.header.stamp.sec = scan_start_sec;
    out.header.stamp.nanosec = scan_start_nsec;
    if (!output_frame_id_.empty()) {
      out.header.frame_id = output_frame_id_;
    }
    out.height = 1;
    out.width = static_cast<std::uint32_t>(points.size());
    out.is_bigendian = msg->is_bigendian;
    out.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(out);
    modifier.setPointCloud2Fields(
      6,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "time", 1, sensor_msgs::msg::PointField::FLOAT32,
      "ring", 1, sensor_msgs::msg::PointField::UINT16);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(out, "z");
    sensor_msgs::PointCloud2Iterator<float> out_intensity(out, "intensity");
    sensor_msgs::PointCloud2Iterator<float> out_time(out, "time");
    sensor_msgs::PointCloud2Iterator<std::uint16_t> out_ring(out, "ring");

    for (const auto & point : points) {
      *out_x = point.x;
      *out_y = point.y;
      *out_z = point.z;
      *out_intensity = point.intensity;
      *out_time = point.time;
      *out_ring = point.ring;
      ++out_x;
      ++out_y;
      ++out_z;
      ++out_intensity;
      ++out_time;
      ++out_ring;
    }

    last_input_points_ = total;
    last_output_points_ = points.size();
    last_frame_id_ = out.header.frame_id;
    last_seen_ = now();
    seen_ = true;
    pub_->publish(out);
  }

  bool validateInput(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    static constexpr const char * required_fields[] = {
      "x", "y", "z", "intensity", "line", "timestamp"};
    for (const auto * field : required_fields) {
      if (!hasField(cloud, field)) {
        last_error_ = std::string("missing_field:") + field;
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Livox PointCloud2 adapter input is missing required field '%s'",
          field);
        return false;
      }
    }
    last_error_.clear();
    return true;
  }

  void publishHeartbeat()
  {
    std_msgs::msg::String heartbeat;
    if (!enabled_) {
      heartbeat.data = "disabled";
    } else if (!last_error_.empty()) {
      heartbeat.data = "error:" + last_error_;
    } else if (!seen_ || (now() - last_seen_) > rclcpp::Duration::from_seconds(1.0)) {
      heartbeat.data = "waiting_for_livox_pointcloud";
    } else {
      heartbeat.data = "ready:frame=" + last_frame_id_ +
        ":input_points=" + std::to_string(last_input_points_) +
        ":output_points=" + std::to_string(last_output_points_);
    }
    heartbeat_pub_->publish(heartbeat);
  }

  bool enabled_{true};
  bool seen_{false};
  bool filter_invalid_returns_{true};
  int scan_lines_{4};
  std::size_t last_input_points_{0};
  std::size_t last_output_points_{0};
  std::string output_frame_id_;
  std::string last_frame_id_;
  std::string last_error_;
  rclcpp::Time last_seen_{0, 0u, RCL_SYSTEM_TIME};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::LivoxPointcloudAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
