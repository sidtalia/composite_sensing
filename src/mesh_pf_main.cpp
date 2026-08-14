#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "composite_sensing/mesh_pf_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<composite_sensing::MeshPfNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "mesh_pf failed: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
