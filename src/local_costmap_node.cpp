#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "autonomy/msg/masked_height_scan.hpp"

namespace autonomy
{
namespace
{

class LocalCostmapNode final : public rclcpp::Node
{
public:
  LocalCostmapNode()
  : Node("local_costmap_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    scan_topic_ = declare_parameter<std::string>(
      "height_scan_topic", "/elevation_mapping_node/local_terrain_map");
    costmap_topic_ = declare_parameter<std::string>("costmap_topic", "~/local_costmap");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    footprint_frame_id_ = declare_parameter<std::string>("footprint_frame_id", "");
    lethal_height_ = std::max(1.0e-3, declare_parameter<double>("lethal_height", 0.18));
    inscribed_height_ = std::clamp(
      declare_parameter<double>("inscribed_height", 0.08), 0.0, lethal_height_);
    inflation_radius_ = std::max(0.0, declare_parameter<double>("inflation_radius", 0.45));
    inscribed_radius_ = std::max(0.0, declare_parameter<double>("inscribed_radius", 0.25));
    inflation_cost_scaling_ = std::max(1.0e-3, declare_parameter<double>("inflation_cost_scaling", 4.0));
    min_inflation_cost_ = static_cast<int>(
      std::clamp<long>(declare_parameter<int>("min_inflation_cost", 1), 0L, 99L));
    unknown_is_free_ = declare_parameter<bool>("unknown_is_free", true);
    obstacle_hold_scans_ = static_cast<std::uint8_t>(
      std::clamp<int>(declare_parameter<int>("obstacle_hold_scans", 3), 0, 100));

    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile());
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/local_costmap_node", 10);
    scan_sub_ = create_subscription<autonomy::msg::MaskedHeightScan>(
      scan_topic_,
      rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile(),
      [this](autonomy::msg::MaskedHeightScan::SharedPtr msg) { onScan(std::move(msg)); });
    heartbeat_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() { publishHeartbeat(); });

    RCLCPP_INFO(
      get_logger(),
      "Local costmap listening scan=%s publishing %s frame=%s",
      scan_topic_.c_str(),
      costmap_topic_.c_str(),
      frame_id_.c_str());
  }

private:
  void onScan(autonomy::msg::MaskedHeightScan::SharedPtr scan)
  {
    auto costmap = toCostmap(*scan);
    costmap_pub_->publish(costmap);
    ++received_scans_;
    last_frame_id_ = scan->header.frame_id;
  }

  nav_msgs::msg::OccupancyGrid toCostmap(const autonomy::msg::MaskedHeightScan & scan)
  {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header = scan.header;
    msg.info.resolution = scan.resolution;
    msg.info.width = scan.width;
    msg.info.height = scan.height;
    msg.info.origin.position.x = scan.x_min;
    msg.info.origin.position.y = scan.y_min;
    msg.info.origin.orientation.w = 1.0;
    msg.data.assign(static_cast<std::size_t>(msg.info.width) * msg.info.height, -1);

    projectOriginToMapFrame(scan, msg);
    fillCosts(scan, msg.data);
    applyUnknownPolicy(msg.data);
    inflateCosts(msg.info.width, msg.info.height, msg.info.resolution, msg.data);
    return msg;
  }

  void projectOriginToMapFrame(
    const autonomy::msg::MaskedHeightScan & scan,
    nav_msgs::msg::OccupancyGrid & msg)
  {
    if (frame_id_.empty() || scan.header.frame_id.empty() || frame_id_ == scan.header.frame_id) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform_msg;
    try {
      transform_msg = tf_buffer_.lookupTransform(frame_id_, scan.header.frame_id, scan.header.stamp);
    } catch (const tf2::TransformException &) {
      try {
        transform_msg = tf_buffer_.lookupTransform(frame_id_, scan.header.frame_id, tf2::TimePointZero);
      } catch (const tf2::TransformException & fallback_ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Publishing local costmap in %s; TF %s -> %s unavailable: %s",
          scan.header.frame_id.c_str(),
          scan.header.frame_id.c_str(),
          frame_id_.c_str(),
          fallback_ex.what());
        return;
      }
    }

    tf2::Transform transform;
    tf2::fromMsg(transform_msg.transform, transform);
    const auto map_origin = transform * tf2::Vector3(scan.x_min, scan.y_min, 0.0);
    msg.header.frame_id = frame_id_;
    msg.info.origin.position.x = map_origin.x();
    msg.info.origin.position.y = map_origin.y();
    msg.info.origin.position.z = footprintZ(map_origin.z());
    msg.info.origin.orientation = yawOnly(transform.getRotation());
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

  geometry_msgs::msg::Quaternion yawOnly(const tf2::Quaternion & quaternion) const
  {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    tf2::Quaternion yaw_quaternion;
    yaw_quaternion.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(yaw_quaternion);
  }

  void fillCosts(
    const autonomy::msg::MaskedHeightScan & scan,
    std::vector<std::int8_t> & costs) const
  {
    for (std::size_t i = 0; i < costs.size() && i < scan.data.size(); ++i) {
      const bool valid = i < scan.valid_mask.size() && scan.valid_mask[i] != 0U;
      if (!valid) {
        costs[i] = -1;
        continue;
      }

      const double terrain_z = -static_cast<double>(scan.data[i]);
      if (terrain_z <= 0.0) {
        costs[i] = 0;
      } else if (terrain_z >= lethal_height_) {
        costs[i] = 100;
      } else if (terrain_z <= inscribed_height_) {
        costs[i] = static_cast<std::int8_t>(
          std::lround(40.0 * terrain_z / std::max(inscribed_height_, 1.0e-3)));
      } else {
        const double ratio =
          (terrain_z - inscribed_height_) / std::max(lethal_height_ - inscribed_height_, 1.0e-3);
        costs[i] = static_cast<std::int8_t>(
          std::lround(40.0 + 60.0 * std::clamp(ratio, 0.0, 1.0)));
      }
    }
  }

  void applyUnknownPolicy(std::vector<std::int8_t> & costs)
  {
    if (obstacle_hold_counts_.size() != costs.size()) {
      obstacle_hold_counts_.assign(costs.size(), 0);
    }

    for (std::size_t i = 0; i < costs.size(); ++i) {
      if (costs[i] >= 100) {
        obstacle_hold_counts_[i] = obstacle_hold_scans_;
        continue;
      }

      if (costs[i] < 0) {
        if (obstacle_hold_counts_[i] > 0U) {
          costs[i] = 100;
          --obstacle_hold_counts_[i];
        } else if (unknown_is_free_) {
          costs[i] = 0;
        }
      } else {
        obstacle_hold_counts_[i] = 0;
      }
    }
  }

  void inflateCosts(
    const std::uint32_t width,
    const std::uint32_t height,
    const double resolution,
    std::vector<std::int8_t> & costs) const
  {
    if (inflation_radius_ <= 0.0 || costs.empty() || width == 0U || height == 0U) {
      return;
    }

    auto inflated = costs;
    const int radius_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution));
    for (std::uint32_t row = 0; row < height; ++row) {
      for (std::uint32_t col = 0; col < width; ++col) {
        const auto obstacle_index = static_cast<std::size_t>(row) * width + col;
        if (costs[obstacle_index] < 100) {
          continue;
        }

        const int row_min = std::max<int>(0, static_cast<int>(row) - radius_cells);
        const int row_max = std::min<int>(static_cast<int>(height) - 1, static_cast<int>(row) + radius_cells);
        const int col_min = std::max<int>(0, static_cast<int>(col) - radius_cells);
        const int col_max = std::min<int>(static_cast<int>(width) - 1, static_cast<int>(col) + radius_cells);
        for (int near_row = row_min; near_row <= row_max; ++near_row) {
          for (int near_col = col_min; near_col <= col_max; ++near_col) {
            const double dx = static_cast<double>(near_col - static_cast<int>(col)) * resolution;
            const double dy = static_cast<double>(near_row - static_cast<int>(row)) * resolution;
            const double distance = std::hypot(dx, dy);
            if (distance > inflation_radius_) {
              continue;
            }
            const auto index = static_cast<std::size_t>(near_row) * width + static_cast<std::size_t>(near_col);
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
    costs = std::move(inflated);
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

  void publishHeartbeat()
  {
    std_msgs::msg::String msg;
    msg.data = received_scans_ == 0 ? "waiting_for_scan" :
      "ready:scans=" + std::to_string(received_scans_) + ":frame=" + last_frame_id_;
    heartbeat_pub_->publish(msg);
  }

  std::string scan_topic_;
  std::string costmap_topic_;
  std::string frame_id_;
  std::string footprint_frame_id_;
  std::string last_frame_id_;
  double lethal_height_{0.18};
  double inscribed_height_{0.08};
  double inflation_radius_{0.45};
  double inscribed_radius_{0.25};
  double inflation_cost_scaling_{4.0};
  int min_inflation_cost_{1};
  bool unknown_is_free_{true};
  std::uint8_t obstacle_hold_scans_{3};
  std::uint64_t received_scans_{0};
  std::vector<std::uint8_t> obstacle_hold_counts_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<autonomy::msg::MaskedHeightScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace
}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::LocalCostmapNode>());
  rclcpp::shutdown();
  return 0;
}
