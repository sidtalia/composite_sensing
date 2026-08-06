#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "composite_sensing/lidar_device.hpp"
#include "composite_sensing/mesh_localizer_backend.hpp"

namespace composite_sensing
{

/**
 * In-process lidar SDK + constant-twist mesh PF.
 * Capture thread owns the vendor SDK; localize ~localize_hz; publishes
 * absolute pose (map GPS-like) and static TF base_link ← lidar.
 */
class LidarMeshCompositeNode : public rclcpp::Node
{
public:
  explicit LidarMeshCompositeNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LidarMeshCompositeNode() override;

private:
  void declare_params();
  void start_pipeline();
  void capture_loop();
  void publish_static_lidar_tf();
  void publish_cloud(const LidarFrame & cloud);
  void publish_pose(const PoseMap & pose, const rclcpp::Time & stamp);

  // Lidar
  std::string lidar_backend_{"unitree"};
  std::string port_{"/dev/ttyUSB0"};
  std::string lidar_frame_{"unilidar_lidar"};
  std::string base_frame_{"base_link"};
  std::string parent_frame_{"base_link"};
  double t_x_{0}, t_y_{0}, t_z_{0.1};
  double q_x_{0}, q_y_{0}, q_z_{0}, q_w_{1};
  float range_min_{0.4f};
  float range_max_{50.f};
  int cloud_scan_num_{18};
  bool publish_cloud_{true};
  std::string cloud_topic_{"/unilidar/cloud"};

  // Mesh PF
  std::string map_file_;
  std::string map_frame_{"map"};
  std::string pose_topic_{"/localization/mesh_pose"};
  double localize_hz_{10.0};
  int num_particles_{2000};
  int beam_samples_{64};
  bool global_init_on_start_{true};
  std::array<double, 6> init_bb_{-20, -20, 0, 20, 20, 2};

  std::unique_ptr<LidarDevice> lidar_;
  std::unique_ptr<MeshLocalizerBackend> localizer_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_srv_;

  std::atomic<bool> running_{false};
  std::thread capture_thread_;
};

}  // namespace composite_sensing
