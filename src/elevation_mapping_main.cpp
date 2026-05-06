#include <rclcpp/rclcpp.hpp>

#include "elevation_mapping_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<height_map_ros2::ElevationMappingNode>());
  rclcpp::shutdown();
  return 0;
}
