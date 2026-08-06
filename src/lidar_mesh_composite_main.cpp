#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "composite_sensing/lidar_mesh_composite_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<composite_sensing::LidarMeshCompositeNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "lidar_mesh_composite failed: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
