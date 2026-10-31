#pragma once

#include <deque>
#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include "composite_sensing/lidar_device.hpp"

namespace composite_sensing
{

/** IMU-body pose in the FAST-LIO world frame (upstream "camera_init" ← "body"). */
struct FastLioOdom
{
  int64_t stamp_ns{0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d vel_body{Eigen::Vector3d::Zero()};
  bool valid{false};
};

struct FastLioScan
{
  LidarFrame cloud;
  FastLioOdom odom;
};

/**
 * Mid-360 defaults match FAST_LIO/config/mid360.yaml + mapping_mid360.launch.
 * lidar_T/R is the lidar pose in the IMU frame (inverse of SSoT imu_xyz).
 */
struct FastLioConfig
{
  Eigen::Vector3d lidar_T_wrt_imu{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d lidar_R_wrt_imu{Eigen::Matrix3d::Identity()};
  double gyr_cov{0.1};
  double acc_cov{0.1};
  double b_gyr_cov{0.0001};
  double b_acc_cov{0.0001};
  double filter_size_surf{0.5};
  double filter_size_map{0.5};
  double cube_len{1000.0};
  float det_range{100.f};
  int max_iterations{3};
};

/**
 * In-process FAST-LIO2. Feed raw lidar-frame points (time_offset_s from the
 * scan start) and Livox IMU. Returns the scan deskewed to scan end, in the
 * lidar frame, plus the post-update IMU pose.
 *
 * ponytail: one instance. The measurement callback is a file-scope pointer
 * because IKFoM takes a function pointer. A second FastLioOdometry overwrites it.
 */
class FastLioOdometry
{
public:
  explicit FastLioOdometry(const FastLioConfig & config);
  ~FastLioOdometry();

  FastLioOdometry(const FastLioOdometry &) = delete;
  FastLioOdometry & operator=(const FastLioOdometry &) = delete;

  std::vector<FastLioScan> push_imu(const std::vector<LidarImuSample> & samples);
  std::vector<FastLioScan> push_cloud(const LidarFrame & raw);

  bool initialized() const;
  size_t pending_scans() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace composite_sensing
