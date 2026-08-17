#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "composite_sensing/camera_device.hpp"
#include "composite_sensing/color_sync_shm.hpp"
#include "composite_sensing/dataset_camera_device.hpp"
#include "composite_sensing/realsense_camera_device.hpp"
#include "composite_sensing/vslam_backend.hpp"

namespace composite_sensing {

/**
 * Thin ROS node wiring CameraDevice + VslamBackend.
 * Same params/topics/workers as legacy StereoCompositeNode; IR never on DDS.
 */
class StereoCompositeModularNode : public rclcpp::Node
{
public:
  explicit StereoCompositeModularNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~StereoCompositeModularNode() override;

private:
  struct LatestSlot {
    std::mutex mu;
    std::condition_variable cv_pose;
    std::condition_variable cv_color;
    std::condition_variable cv_depth;

    bool has_pose{false};
    uint64_t pose_seq{0};
    rclcpp::Time ros_stamp{0, 0, RCL_ROS_TIME};
    PoseOptical pose{};
    bool has_gt{false};
    PoseOptical gt_pose{};

    bool has_color{false};
    uint64_t color_seq{0};
    sensor_msgs::msg::Image color_image;
    rclcpp::Time color_stamp{0, 0, RCL_ROS_TIME};

    bool has_stereo_for_depth{false};
    rclcpp::Time stereo_stamp{0, 0, RCL_ROS_TIME};
  };

  void declare_params();
  void start_pipeline();
  void publish_static_tfs();
  void capture_loop();
  void odom_worker();
  void color_worker();
  void depth_worker();
  void publish_odom_tf(const rclcpp::Time & stamp, const PoseOptical & pose);
  void publish_gt_odom(const rclcpp::Time & stamp, const PoseOptical & pose);
  void publish_color_slot(sensor_msgs::msg::Image img, const rclcpp::Time & stamp);

  // Params (startup-only) — names match legacy realsense_cuvslam_node
  std::string serial_number_;
  std::string camera_name_;
  /** ``realsense`` (default) or ``dataset`` (DatasetCameraDevice). */
  std::string camera_backend_{"realsense"};
  std::string dataset_path_;
  std::string dataset_view_{"front"};
  bool dataset_loop_{true};
  double dataset_rate_hz_{30.0};
  /** When false, skip cuVSLAM and use dataset GT poses if available. */
  bool enable_vslam_{true};
  /**
   * Pose published on odom/TF: ``vslam`` (default) or ``gt`` (dataset).
   * With ``gt``, cuVSLAM still runs when enable_vslam (compute load) but
   * mapping consumers see ground-truth TF.
   */
  std::string odom_source_{"vslam"};
  int infra_width_{640};
  int infra_height_{360};
  int infra_fps_{60};
  bool enable_color_{true};
  int color_width_{640};
  int color_height_{360};
  int color_fps_{30};
  double color_publish_fps_{15.0};
  bool enable_depth_{false};
  bool align_depth_{true};
  int depth_width_{640};
  int depth_height_{360};
  int depth_fps_{30};
  double depth_publish_fps_{15.0};
  int emitter_enabled_{0};
  int visual_preset_{3};
  double clip_distance_{0.0};
  bool initial_reset_{false};
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  /** When false (default), odom is a measurement only — EKF owns odom→base TF. */
  bool publish_odom_tf_{false};
  bool apply_body_mount_{false};
  std::array<double, 3> t_body_sensor_{{0, 0, 0}};
  std::array<double, 9> R_body_sensor_{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
  bool async_sba_{true};
  bool slam_sync_mode_{false};
  int warmup_frames_{60};
  bool log_cuvslam_timing_{false};
  /** Off by default: SDK TIME_OF_ARRIVAL → Track + dequeue→Track handoff. */
  bool profile_{false};
  /** Periodic raw RGB/depth ingress vs publish rates (per camera node). */
  bool log_sensor_rates_{true};
  double sensor_rate_log_every_s_{5.0};
  /**
   * Color multi-cam phase sync via POSIX shm (no ROS):
   * ``master`` strides and writes last publish stamp;
   * ``follower`` finds sensor frame nearest master_stamp+period, then strides on that phase;
   * ``standalone`` independent stride.
   */
  std::string color_sync_role_{"standalone"};
  std::string color_sync_shm_name_;

  std::atomic<uint64_t> color_in_n_{0};
  std::atomic<uint64_t> color_keep_n_{0};
  std::atomic<uint64_t> color_pub_n_{0};
  std::atomic<uint64_t> depth_in_n_{0};
  std::atomic<uint64_t> depth_pub_n_{0};
  std::atomic<uint64_t> ir_in_n_{0};

  ColorSyncShm color_sync_shm_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  /** Dataset GT optical→FLU odom, same stamp as est when available. */
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  sensor_msgs::msg::CameraInfo color_camera_info_;
  sensor_msgs::msg::CameraInfo depth_camera_info_;
  std::string color_optical_frame_;
  std::string depth_optical_frame_;

  std::unique_ptr<CameraDevice> camera_;
  DatasetCameraDevice * dataset_camera_{nullptr};  // non-owning, when backend=dataset
  std::unique_ptr<VslamBackend> vslam_;

  LatestSlot slot_;
  std::atomic<bool> running_{false};
  std::thread capture_thread_;
  std::thread odom_thread_;
  std::thread color_thread_;
  std::thread depth_thread_;

  std::array<double, 3> prev_t_flu_{{0, 0, 0}};
  rclcpp::Time prev_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace composite_sensing
