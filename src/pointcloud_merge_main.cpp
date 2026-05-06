#include <rclcpp/rclcpp.hpp>

#include "pointcloud_merger_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<height_map_ros2::PointCloudMergerNode>());
  rclcpp::shutdown();
  return 0;
}
