#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "composite_sensing/cloud_deskew.hpp"
#include "composite_sensing/lidar_device.hpp"

namespace composite_sensing
{

/**
 * In-process lidar SDK + static base→lidar TF + optional in-process deskew.
 * Publishes deskewed PointCloud2 on cloud_topic (canonical /livox/cloud).
 * Mesh particle filter lives in mesh_pf_node.
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
  void publish_cloud(
    const LidarFrame & cloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub);
  void publish_temperatures();

  std::string lidar_backend_{"unitree"};
  std::string port_{"/dev/ttyUSB0"};
  std::string lidar_ip_;
  std::string local_ip_;
  uint16_t lidar_port_{0};
  uint16_t local_port_{0};
  std::string config_path_;
  int frame_time_ms_{100};
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
  bool publish_raw_cloud_{false};
  std::string raw_cloud_topic_{"/livox/cloud_raw"};
  std::string core_temp_topic_{"/livox/core_temp"};
  std::string environment_temp_topic_{"/livox/environment_temp"};
  double temp_query_period_s_{2.0};
  double core_temp_warn_c_{55.0};
  double cloud_stale_warn_s_{5.0};
  double status_log_period_s_{60.0};

  bool deskew_enable_{true};
  std::string deskew_motion_frame_{"odom"};
  DeskewReference deskew_reference_{DeskewReference::End};
  double deskew_tf_timeout_s_{0.0};
  double deskew_warn_ms_{40.0};
  double deskew_tf_buffer_s_{10.0};

  std::unique_ptr<LidarDevice> lidar_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr core_temp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr environment_temp_pub_;

  std::atomic<bool> running_{false};
  std::thread capture_thread_;

  uint64_t clouds_published_{0};
  uint64_t points_published_{0};
  size_t last_cloud_points_{0};
  std::chrono::steady_clock::time_point last_cloud_steady_{};
  std::chrono::steady_clock::time_point last_status_log_{};
  bool cloud_stale_warned_{false};
  bool core_temp_high_warned_{false};
};

}  // namespace composite_sensing
