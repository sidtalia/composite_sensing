#pragma once

#include <array>
#include <deque>
#include <memory>
#include <random>
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
  /// Pose covariance in the LIO world, row-major [x, y, z, rx, ry, rz].
  /// Rotation error is rotated out of the IMU frame into that world.
  std::array<double, 36> pose_cov{};
  /// Twist covariance in the IMU frame, row-major [vx, vy, vz, wx, wy, wz].
  /// Linear block is the velocity state. Angular rate is not a state; that
  /// 3×3 is large so a zero angular twist is not a perfect measurement.
  std::array<double, 36> twist_cov{};
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
  /// Isotropic Gaussian stddev on each lidar point, metres. 0 leaves points alone.
  float point_noise_std_m{0.f};
  uint32_t point_noise_seed{1};
};

/** Add N(0, sigma_m) to xyz. sigma_m <= 0 is a no-op. */
inline void add_lidar_point_noise(
  float & x, float & y, float & z, std::mt19937 & rng, float sigma_m)
{
  if (!(sigma_m > 0.f)) {
    return;
  }
  std::normal_distribution<float> n(0.f, sigma_m);
  x += n(rng);
  y += n(rng);
  z += n(rng);
}

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

inline Eigen::Isometry3d iso_pose(
  const Eigen::Vector3d & t, const Eigen::Quaterniond & q)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = q.normalized().toRotationMatrix();
  T.translation() = t;
  return T;
}

/** Latch T_odom_lio so this IMU pose lands on the EKF base pose. */
inline Eigen::Isometry3d latch_odom_from_lio(
  const Eigen::Isometry3d & T_odom_base,
  const Eigen::Isometry3d & T_base_imu,
  const Eigen::Isometry3d & T_lio_imu)
{
  return T_odom_base * T_base_imu * T_lio_imu.inverse();
}

inline void write_cov_block(
  std::array<double, 36> & c, int r0, int c0, const Eigen::Matrix3d & B)
{
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      c[static_cast<size_t>((r0 + i) * 6 + (c0 + j))] = B(i, j);
    }
  }
}

/**
 * P is the 23-dof error state: position (world), rotation (IMU), then
 * extrinsics, velocity (world) at index 12. Pose cov matches a header frame
 * equal to the LIO world. Twist linear cov matches the published body velocity.
 */
template <typename Derived>
inline void lio_pose_twist_cov(
  const Eigen::MatrixBase<Derived> & P,
  const Eigen::Matrix3d & R_world_imu,
  std::array<double, 36> & pose,
  std::array<double, 36> & twist)
{
  pose.fill(0.0);
  twist.fill(0.0);
  const Eigen::Matrix3d & R = R_world_imu;
  const Eigen::Matrix3d Ppp = P.template block<3, 3>(0, 0);
  const Eigen::Matrix3d Ppr = P.template block<3, 3>(0, 3);
  const Eigen::Matrix3d Prp = P.template block<3, 3>(3, 0);
  const Eigen::Matrix3d Prr = P.template block<3, 3>(3, 3);
  write_cov_block(pose, 0, 0, Ppp);
  write_cov_block(pose, 0, 3, Ppr * R.transpose());
  write_cov_block(pose, 3, 0, R * Prp);
  write_cov_block(pose, 3, 3, R * Prr * R.transpose());
  const Eigen::Matrix3d Pvv = P.template block<3, 3>(12, 12);
  write_cov_block(twist, 0, 0, R.transpose() * Pvv * R);
  // ponytail: angular rate is not in the state. Large variance, not zero.
  twist[21] = twist[28] = twist[35] = 1e3;
}

/** Rotate a 6×6 covariance by blkdiag(R, R). */
inline void rotate_cov6(std::array<double, 36> & c, const Eigen::Matrix3d & R)
{
  Eigen::Matrix<double, 6, 6> M;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      M(i, j) = c[static_cast<size_t>(i * 6 + j)];
    }
  }
  Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
  A.block<3, 3>(0, 0) = R;
  A.block<3, 3>(3, 3) = R;
  M = A * M * A.transpose();
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      c[static_cast<size_t>(i * 6 + j)] = M(i, j);
    }
  }
}

/** Base pose in odom from the latched world and the current IMU pose. */
inline Eigen::Isometry3d lio_base_in_odom(
  const Eigen::Isometry3d & T_odom_lio,
  const Eigen::Isometry3d & T_lio_imu,
  const Eigen::Isometry3d & T_base_imu)
{
  return T_odom_lio * T_lio_imu * T_base_imu.inverse();
}

}  // namespace composite_sensing
