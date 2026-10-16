#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <random>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "composite_sensing/constant_twist_mesh_pf_backend.hpp"
#include "composite_sensing/mesh_localizer_backend.hpp"
#include "composite_sensing/micp_l_backend.hpp"

namespace composite_sensing
{

/** Standalone constant-twist mesh particle filter (Vulkan RT / Embree). */
class MeshPfNode : public rclcpp::Node
{
public:
  explicit MeshPfNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_params();
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void reset_wait();
  void on_trigger(const std_msgs::msg::Empty::SharedPtr msg);
  void on_hunting(const std_msgs::msg::Bool::SharedPtr msg);
  void on_map_odom_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  bool try_seed(const rclcpp::Time & stamp);
  bool try_seed_from_tf(const rclcpp::Time & stamp);
  bool try_seed_from_pose_msg(const rclcpp::Time & stamp);
  bool seed_from_map_odom(
    const tf2::Transform & T_map_odom, const rclcpp::Time & stamp);
  bool lookup_odom_base(
    const rclcpp::Time & stamp, geometry_msgs::msg::TransformStamped & out);
  OdomDelta lookup_odom_delta(const rclcpp::Time & stamp);
  void step_debug_odom_drift();
  void apply_debug_odom_drift(geometry_msgs::msg::TransformStamped * ts) const;
  void warn_debug_odom_drift() const;
  void publish_pose(const PoseMap & pose, const rclcpp::Time & stamp);
  void publish_map_odom(const PoseMap & pose, const rclcpp::Time & stamp);

  std::string cloud_topic_{"/livox/cloud"};
  std::string pose_topic_{"/localization/mesh_pose"};
  std::string map_file_;
  std::string map_frame_{"map"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string raycast_backend_{"auto"};
  bool publish_tf_{true};
  bool init_from_tf_{false};
  std::string init_tf_parent_{"map"};
  std::string init_tf_child_{"odom"};
  double init_tf_trans_std_m_{1.0};
  double init_tf_rot_std_rad_{0.17453292519943295};
  double init_tf_timeout_s_{10.0};
  bool wait_for_aruco_hunt_{false};
  std::string trigger_topic_{"/aruco_registration/trigger"};
  std::string hunting_topic_{"/aruco_registration/hunting"};
  std::string map_odom_pose_topic_{"/aruco_registration/map_odom"};
  bool tracking_use_odom_tf_{true};
  std::string tracking_motion_{"odom"};
  int tracking_particles_{10000};
  double still_trans_noise_m_{0.05};
  double still_rp_noise_rad_{0.05};
  double still_yaw_noise_rad_{0.05};
  double dist_noise_gain_{0.1};
  double rot_rp_noise_gain_{0.2};
  double rot_yaw_noise_gain_{0.2};
  double tracking_still_trans_noise_m_{0.01};
  double tracking_still_rp_noise_rad_{0.008726646259971648};
  double tracking_still_yaw_noise_rad_{0.008726646259971648};
  double tracking_dist_noise_gain_{0.02};
  double tracking_rot_rp_noise_gain_{0.05};
  double tracking_rot_yaw_noise_gain_{0.05};
  double t_x_{0}, t_y_{0}, t_z_{0.1};
  double q_x_{0}, q_y_{0}, q_z_{0}, q_w_{1};
  double localize_hz_{10.0};
  int num_particles_{100000};
  int beam_samples_{64};
  bool global_init_on_start_{true};
  std::array<double, 6> init_bb_{-20, -20, 0, 20, 20, 2};
  std::array<double, 2> init_roll_{-0.7853981633974483, 0.7853981633974483};
  std::array<double, 2> init_pitch_{-0.7853981633974483, 0.7853981633974483};
  std::array<double, 2> init_yaw_{-3.14159265358979323846, 3.14159265358979323846};

  MeshLocalizerConfig fill_localizer_config() const;
  void ensure_pf();
  void ensure_micp();

  std::string tracking_backend_{"micp"};
  int micp_iterations_{5};
  int micp_samples_{1024};
  double micp_max_dist_m_{0.5};
  double micp_adaptive_max_dist_min_m_{0.05};
  std::unique_ptr<ConstantTwistMeshPfBackend> pf_;
  std::unique_ptr<MicpLBackend> micp_;
  MeshLocalizerBackend * active_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hunting_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr map_odom_pose_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_;

  std::mutex rate_mu_;
  rclcpp::Time last_localize_time_{0, 0, RCL_ROS_TIME};
  bool seeded_{true};
  bool seed_wait_started_{false};
  std::chrono::steady_clock::time_point seed_wait_start_{};
  bool have_last_odom_{false};
  tf2::Transform last_odom_base_{};
  bool waiting_for_hunt_{false};
  bool saw_hunt_{false};
  bool ready_to_seed_{false};
  bool have_map_odom_pose_{false};
  tf2::Transform last_map_odom_pose_{};

  bool debug_odom_drift_{false};
  double debug_odom_drift_trans_std_m_{0.005};
  double debug_odom_drift_z_std_m_{0.001};
  double debug_odom_drift_rot_std_rad_{0.0008726646259971648};
  double debug_odom_drift_tx_{0};
  double debug_odom_drift_ty_{0};
  double debug_odom_drift_tz_{0};
  double debug_odom_drift_yaw_{0};
  std::mt19937 debug_odom_rng_{std::random_device{}()};
};

}  // namespace composite_sensing
