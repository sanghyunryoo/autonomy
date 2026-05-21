#include <rclcpp/rclcpp.hpp>

#include "pointcloud_merger_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::PointCloudMergerNode>());
  rclcpp::shutdown();
  return 0;
}
