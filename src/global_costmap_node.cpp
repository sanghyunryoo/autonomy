#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace autonomy
{
namespace
{

struct ObservedPoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

class GlobalCostmapNode final : public rclcpp::Node
{
public:
  GlobalCostmapNode()
  : Node("global_costmap_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    costmap_topic_ = declare_parameter<std::string>("costmap_topic", "~/global_costmap");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    footprint_frame_id_ = declare_parameter<std::string>("footprint_frame_id", "");
    sensor_frame_id_ = declare_parameter<std::string>("sensor_frame_id", footprint_frame_id_);
    resolution_ = std::max(0.02, declare_parameter<double>("resolution", 0.10));
    x_min_ = declare_parameter<double>("x_min", -20.0);
    x_max_ = declare_parameter<double>("x_max", 20.0);
    y_min_ = declare_parameter<double>("y_min", -20.0);
    y_max_ = declare_parameter<double>("y_max", 20.0);
    min_obstacle_z_ = declare_parameter<double>("min_obstacle_z", 0.05);
    max_obstacle_z_ = declare_parameter<double>("max_obstacle_z", 1.50);
    mark_free_space_ = declare_parameter<bool>("mark_free_space", true);
    max_free_z_ = std::min(declare_parameter<double>("max_free_z", min_obstacle_z_), min_obstacle_z_);
    auto_expand_ = declare_parameter<bool>("auto_expand", true);
    expand_margin_ = std::max(0.0, declare_parameter<double>("expand_margin", 1.0));
    max_cells_ = static_cast<std::size_t>(
      std::max<int>(1, declare_parameter<int>("max_cells", 4000000)));
    free_clear_observations_ = static_cast<std::uint8_t>(
      std::clamp<int>(declare_parameter<int>("free_clear_observations", 4), 1, 100));
    inflation_radius_ = std::max(0.0, declare_parameter<double>("inflation_radius", 0.6));
    inscribed_radius_ = std::max(0.0, declare_parameter<double>("inscribed_radius", 0.25));
    inflation_cost_scaling_ = std::max(1.0e-3, declare_parameter<double>("inflation_cost_scaling", 4.0));
    min_inflation_cost_ = static_cast<int>(
      std::clamp<long>(declare_parameter<int>("min_inflation_cost", 1), 0L, 99L));
    hit_increment_ = static_cast<int>(
      std::clamp<long>(declare_parameter<int>("hit_increment", 8), 1L, 100L));
    decay_per_publish_ = static_cast<int>(
      std::clamp<long>(declare_parameter<int>("decay_per_publish", 0), 0L, 100L));
    publish_rate_hz_ = std::max(1.0, declare_parameter<double>("publish_rate_hz", 2.0));

    if (x_max_ <= x_min_ || y_max_ <= y_min_) {
      throw std::invalid_argument("Invalid global costmap bounds");
    }

    width_ = static_cast<std::uint32_t>(std::ceil((x_max_ - x_min_) / resolution_));
    height_ = static_cast<std::uint32_t>(std::ceil((y_max_ - y_min_) / resolution_));
    costs_.assign(static_cast<std::size_t>(width_) * height_, -1);
    free_clear_counts_.assign(costs_.size(), 0);

    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/global_costmap_node", 10);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { onCloud(std::move(msg)); });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publishCostmap(); });

    RCLCPP_INFO(
      get_logger(),
      "Global costmap listening cloud=%s publishing %s frame=%s size=%ux%u res=%.3f",
      cloud_topic_.c_str(),
      costmap_topic_.c_str(),
      frame_id_.c_str(),
      width_,
      height_,
      resolution_);
  }

private:
  void onCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_stamp_ = msg->header.stamp;
    if (!msg->header.frame_id.empty()) {
      frame_id_ = msg->header.frame_id;
    }

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x_it(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_it(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_it(*msg, "z");
      const auto count = static_cast<std::size_t>(msg->width) * msg->height;
      const auto origin = sensorOrigin(msg->header.stamp);
      std::vector<ObservedPoint> points;
      points.reserve(count);
      double observed_x_min = 0.0;
      double observed_x_max = 0.0;
      double observed_y_min = 0.0;
      double observed_y_max = 0.0;
      bool has_observation = false;
      for (std::size_t i = 0; i < count; ++i, ++x_it, ++y_it, ++z_it) {
        if (!isRelevantPoint(*x_it, *y_it, *z_it)) {
          continue;
        }
        points.push_back({*x_it, *y_it, *z_it});
        if (!has_observation) {
          observed_x_min = *x_it;
          observed_x_max = *x_it;
          observed_y_min = *y_it;
          observed_y_max = *y_it;
        } else {
          observed_x_min = std::min(observed_x_min, static_cast<double>(*x_it));
          observed_x_max = std::max(observed_x_max, static_cast<double>(*x_it));
          observed_y_min = std::min(observed_y_min, static_cast<double>(*y_it));
          observed_y_max = std::max(observed_y_max, static_cast<double>(*y_it));
        }
        has_observation = true;
      }
      if (auto_expand_ && has_observation) {
        expandToInclude(
          std::min(observed_x_min, origin.valid ? origin.x : observed_x_min) - expand_margin_,
          std::max(observed_x_max, origin.valid ? origin.x : observed_x_max) + expand_margin_,
          std::min(observed_y_min, origin.valid ? origin.y : observed_y_min) - expand_margin_,
          std::max(observed_y_max, origin.valid ? origin.y : observed_y_max) + expand_margin_);
      }
      std::vector<bool> free_observed(costs_.size(), false);
      std::vector<bool> obstacle_observed(costs_.size(), false);
      for (const auto & point : points) {
        addPoint(
          origin.x,
          origin.y,
          origin.valid,
          point.x,
          point.y,
          point.z,
          free_observed,
          obstacle_observed);
      }
      if (mark_free_space_) {
        clearFreeCells(free_observed, obstacle_observed);
      }
      received_clouds_++;
    } catch (const std::runtime_error & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Global costmap input cloud missing xyz fields: %s",
        ex.what());
    }
  }

  struct Point2
  {
    double x{0.0};
    double y{0.0};
    bool valid{false};
  };

  bool isRelevantPoint(const float x, const float y, const float z) const
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return false;
    }
    return (mark_free_space_ && z < max_free_z_) || (z >= min_obstacle_z_ && z <= max_obstacle_z_);
  }

  Point2 sensorOrigin(const builtin_interfaces::msg::Time & stamp)
  {
    const std::string source_frame = !sensor_frame_id_.empty() ? sensor_frame_id_ : footprint_frame_id_;
    if (frame_id_.empty() || source_frame.empty() || frame_id_ == source_frame) {
      return {0.0, 0.0, false};
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(frame_id_, source_frame, stamp);
      return {transform.transform.translation.x, transform.transform.translation.y, true};
    } catch (const tf2::TransformException &) {
      try {
        const auto transform = tf_buffer_.lookupTransform(frame_id_, source_frame, tf2::TimePointZero);
        return {transform.transform.translation.x, transform.transform.translation.y, true};
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Using global costmap sensor origin fallback; TF %s -> %s unavailable: %s",
          source_frame.c_str(),
          frame_id_.c_str(),
          ex.what());
        return {0.0, 0.0, false};
      }
    }
  }

  void addPoint(
    const double origin_x,
    const double origin_y,
    const bool origin_valid,
    const float point_x,
    const float point_y,
    const float z,
    std::vector<bool> & free_observed,
    std::vector<bool> & obstacle_observed)
  {
    if (point_x < x_min_ || point_x >= x_max_ || point_y < y_min_ || point_y >= y_max_) {
      return;
    }

    const auto index = cellIndex(point_x, point_y);
    if (index >= costs_.size()) {
      return;
    }

    const bool obstacle = z >= min_obstacle_z_ && z <= max_obstacle_z_;
    if (mark_free_space_ && origin_valid && std::isfinite(origin_x) && std::isfinite(origin_y)) {
      raytraceFree(origin_x, origin_y, point_x, point_y, !obstacle, free_observed);
    }

    if (!obstacle) {
      free_observed[index] = true;
      return;
    }

    obstacle_observed[index] = true;
    if (index < free_clear_counts_.size()) {
      free_clear_counts_[index] = 0;
    }
    const int current = costs_[index] < 0 ? 0 : costs_[index];
    costs_[index] = static_cast<std::int8_t>(std::min(100, current + hit_increment_));
  }

  std::size_t cellIndex(const double x, const double y) const
  {
    const auto col = static_cast<std::uint32_t>((x - x_min_) / resolution_);
    const auto row = static_cast<std::uint32_t>((y - y_min_) / resolution_);
    return static_cast<std::size_t>(row) * width_ + col;
  }

  void raytraceFree(
    const double origin_x,
    const double origin_y,
    const double end_x,
    const double end_y,
    const bool include_endpoint,
    std::vector<bool> & free_observed)
  {
    if (origin_x < x_min_ || origin_x >= x_max_ || origin_y < y_min_ || origin_y >= y_max_) {
      return;
    }
    if (end_x < x_min_ || end_x >= x_max_ || end_y < y_min_ || end_y >= y_max_) {
      return;
    }

    int x0 = static_cast<int>((origin_x - x_min_) / resolution_);
    int y0 = static_cast<int>((origin_y - y_min_) / resolution_);
    const int x1 = static_cast<int>((end_x - x_min_) / resolution_);
    const int y1 = static_cast<int>((end_y - y_min_) / resolution_);
    const int dx = std::abs(x1 - x0);
    const int sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0);
    const int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
      const bool endpoint = x0 == x1 && y0 == y1;
      if (include_endpoint || !endpoint) {
        const auto index = static_cast<std::size_t>(y0) * width_ + static_cast<std::size_t>(x0);
        if (index < free_observed.size()) {
          free_observed[index] = true;
        }
      }
      if (endpoint) {
        break;
      }
      const int e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        x0 += sx;
      }
      if (e2 <= dx) {
        err += dx;
        y0 += sy;
      }
    }
  }

  void expandToInclude(
    const double requested_x_min,
    const double requested_x_max,
    const double requested_y_min,
    const double requested_y_max)
  {
    const double new_x_min = std::min(x_min_, std::floor(requested_x_min / resolution_) * resolution_);
    const double new_x_max = std::max(x_max_, std::ceil(requested_x_max / resolution_) * resolution_);
    const double new_y_min = std::min(y_min_, std::floor(requested_y_min / resolution_) * resolution_);
    const double new_y_max = std::max(y_max_, std::ceil(requested_y_max / resolution_) * resolution_);
    if (new_x_min == x_min_ && new_x_max == x_max_ && new_y_min == y_min_ && new_y_max == y_max_) {
      return;
    }

    const auto new_width = static_cast<std::uint32_t>(std::ceil((new_x_max - new_x_min) / resolution_));
    const auto new_height = static_cast<std::uint32_t>(std::ceil((new_y_max - new_y_min) / resolution_));
    const auto new_cell_count = static_cast<std::size_t>(new_width) * new_height;
    if (new_cell_count > max_cells_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Skipping global costmap expansion to %ux%u cells; max_cells=%zu",
        new_width,
        new_height,
        max_cells_);
      return;
    }

    std::vector<std::int8_t> expanded(new_cell_count, -1);
    std::vector<std::uint8_t> expanded_clear_counts(new_cell_count, 0);
    const int col_offset = static_cast<int>(std::lround((x_min_ - new_x_min) / resolution_));
    const int row_offset = static_cast<int>(std::lround((y_min_ - new_y_min) / resolution_));
    for (std::uint32_t row = 0; row < height_; ++row) {
      for (std::uint32_t col = 0; col < width_; ++col) {
        const auto new_col = static_cast<std::uint32_t>(static_cast<int>(col) + col_offset);
        const auto new_row = static_cast<std::uint32_t>(static_cast<int>(row) + row_offset);
        expanded[static_cast<std::size_t>(new_row) * new_width + new_col] =
          costs_[static_cast<std::size_t>(row) * width_ + col];
        expanded_clear_counts[static_cast<std::size_t>(new_row) * new_width + new_col] =
          free_clear_counts_[static_cast<std::size_t>(row) * width_ + col];
      }
    }

    x_min_ = new_x_min;
    x_max_ = new_x_max;
    y_min_ = new_y_min;
    y_max_ = new_y_max;
    width_ = new_width;
    height_ = new_height;
    costs_ = std::move(expanded);
    free_clear_counts_ = std::move(expanded_clear_counts);
  }

  void clearFreeCells(
    const std::vector<bool> & free_observed,
    const std::vector<bool> & obstacle_observed)
  {
    const auto count = std::min(costs_.size(), free_observed.size());
    for (std::size_t i = 0; i < count; ++i) {
      if (i < obstacle_observed.size() && obstacle_observed[i]) {
        if (i < free_clear_counts_.size()) {
          free_clear_counts_[i] = 0;
        }
        continue;
      }
      if (!free_observed[i]) {
        continue;
      }
      if (costs_[i] <= 0) {
        costs_[i] = 0;
        if (i < free_clear_counts_.size()) {
          free_clear_counts_[i] = 0;
        }
        continue;
      }
      if (i >= free_clear_counts_.size()) {
        continue;
      }
      free_clear_counts_[i] = std::min<std::uint8_t>(
        free_clear_observations_,
        static_cast<std::uint8_t>(free_clear_counts_[i] + 1));
      if (free_clear_counts_[i] >= free_clear_observations_) {
        costs_[i] = 0;
        free_clear_counts_[i] = 0;
      }
    }
  }

  void publishCostmap()
  {
    nav_msgs::msg::OccupancyGrid msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (decay_per_publish_ > 0) {
        for (auto & cost : costs_) {
          if (cost > 0) {
            cost = static_cast<std::int8_t>(std::max(0, static_cast<int>(cost) - decay_per_publish_));
          }
        }
      }
      if (last_stamp_.sec == 0 && last_stamp_.nanosec == 0) {
        msg.header.stamp = get_clock()->now();
      } else {
        msg.header.stamp = last_stamp_;
      }
      msg.header.frame_id = frame_id_;
      msg.info.resolution = static_cast<float>(resolution_);
      msg.info.width = width_;
      msg.info.height = height_;
      msg.info.origin.position.x = x_min_;
      msg.info.origin.position.y = y_min_;
      msg.info.origin.position.z = footprintZ(0.0);
      msg.info.origin.orientation.w = 1.0;
      msg.data = inflatedCosts();
    }
    costmap_pub_->publish(msg);

    std_msgs::msg::String heartbeat;
    heartbeat.data = received_clouds_ == 0 ? "waiting_for_cloud" :
      "ready:clouds=" + std::to_string(received_clouds_);
    heartbeat_pub_->publish(heartbeat);
  }

  double footprintZ(const double fallback_z)
  {
    if (frame_id_.empty() || footprint_frame_id_.empty()) {
      return fallback_z;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(frame_id_, footprint_frame_id_, tf2::TimePointZero);
      return transform.transform.translation.z;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Using costmap z fallback; TF %s -> %s unavailable: %s",
        footprint_frame_id_.c_str(),
        frame_id_.c_str(),
        ex.what());
      return fallback_z;
    }
  }

  std::vector<std::int8_t> inflatedCosts() const
  {
    auto inflated = costs_;
    if (inflation_radius_ <= 0.0 || costs_.empty()) {
      return inflated;
    }

    const int radius_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
    for (std::uint32_t row = 0; row < height_; ++row) {
      for (std::uint32_t col = 0; col < width_; ++col) {
        const auto obstacle_index = static_cast<std::size_t>(row) * width_ + col;
        if (costs_[obstacle_index] < 100) {
          continue;
        }

        const int row_min = std::max<int>(0, static_cast<int>(row) - radius_cells);
        const int row_max = std::min<int>(static_cast<int>(height_) - 1, static_cast<int>(row) + radius_cells);
        const int col_min = std::max<int>(0, static_cast<int>(col) - radius_cells);
        const int col_max = std::min<int>(static_cast<int>(width_) - 1, static_cast<int>(col) + radius_cells);
        for (int near_row = row_min; near_row <= row_max; ++near_row) {
          for (int near_col = col_min; near_col <= col_max; ++near_col) {
            const double dx = static_cast<double>(near_col - static_cast<int>(col)) * resolution_;
            const double dy = static_cast<double>(near_row - static_cast<int>(row)) * resolution_;
            const double distance = std::hypot(dx, dy);
            if (distance > inflation_radius_) {
              continue;
            }
            const auto index = static_cast<std::size_t>(near_row) * width_ + static_cast<std::size_t>(near_col);
            const int cost = inflationCost(distance);
            if (inflated[index] < 0) {
              inflated[index] = static_cast<std::int8_t>(cost);
            } else {
              inflated[index] = static_cast<std::int8_t>(std::max<int>(inflated[index], cost));
            }
          }
        }
      }
    }
    return inflated;
  }

  int inflationCost(const double distance) const
  {
    if (distance <= 0.0) {
      return 100;
    }
    if (distance <= inscribed_radius_) {
      return 99;
    }
    const double span = std::max(inflation_radius_ - inscribed_radius_, 1.0e-3);
    const double normalized_distance = (distance - inscribed_radius_) / span;
    const double decay = std::exp(-inflation_cost_scaling_ * normalized_distance);
    return std::clamp(
      static_cast<int>(std::lround(min_inflation_cost_ + (98 - min_inflation_cost_) * decay)),
      min_inflation_cost_,
      98);
  }

  std::string cloud_topic_;
  std::string costmap_topic_;
  std::string frame_id_;
  std::string footprint_frame_id_;
  std::string sensor_frame_id_;
  double resolution_{0.10};
  double x_min_{-20.0};
  double x_max_{20.0};
  double y_min_{-20.0};
  double y_max_{20.0};
  double min_obstacle_z_{0.05};
  double max_obstacle_z_{1.50};
  bool mark_free_space_{true};
  double max_free_z_{0.05};
  bool auto_expand_{true};
  double expand_margin_{1.0};
  std::size_t max_cells_{4000000};
  std::uint8_t free_clear_observations_{4};
  double inflation_radius_{0.6};
  double inscribed_radius_{0.25};
  double inflation_cost_scaling_{4.0};
  int min_inflation_cost_{1};
  int hit_increment_{8};
  int decay_per_publish_{0};
  double publish_rate_hz_{2.0};
  std::uint32_t width_{0};
  std::uint32_t height_{0};
  std::vector<std::int8_t> costs_;
  std::vector<std::uint8_t> free_clear_counts_;
  builtin_interfaces::msg::Time last_stamp_;
  std::uint64_t received_clouds_{0};
  std::mutex mutex_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace
}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::GlobalCostmapNode>());
  rclcpp::shutdown();
  return 0;
}
