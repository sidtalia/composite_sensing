#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>

#include "composite_sensing/mesh_localizer_backend.hpp"

namespace composite_sensing
{

/** Standalone Embree constant-twist mesh particle filter. */
class MeshPfNode : public rclcpp::Node
{
public:
  explicit MeshPfNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_params();
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publish_pose(const PoseMap & pose, const rclcpp::Time & stamp);

  std::string cloud_topic_{"/livox/cloud"};
  std::string pose_topic_{"/localization/mesh_pose"};
  std::string map_file_;
  std::string map_frame_{"map"};
  std::string base_frame_{"base_link"};
  double t_x_{0}, t_y_{0}, t_z_{0.1};
  double q_x_{0}, q_y_{0}, q_z_{0}, q_w_{1};
  double localize_hz_{10.0};
  int num_particles_{2000};
  int beam_samples_{64};
  bool global_init_on_start_{true};
  std::array<double, 6> init_bb_{-20, -20, 0, 20, 20, 2};

  std::unique_ptr<MeshLocalizerBackend> localizer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_srv_;

  std::mutex rate_mu_;
  rclcpp::Time last_localize_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace composite_sensing
