#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "composite_sensing/lidar_device.hpp"

namespace composite_sensing
{

enum class DeskewReference : uint8_t
{
  Start = 0,
  End
};

struct DeskewResult
{
  bool ok{false};
  double elapsed_ms{0.0};
  std::string error;
};

/** Stamped T_odom_base from an odometry topic (sorted by stamp_ns). */
struct StampedOdomPose
{
  int64_t stamp_ns{0};
  Eigen::Isometry3d T_odom_base{Eigen::Isometry3d::Identity()};
};

/**
 * Motion-compensate points into the lidar frame at scan start or end.
 *
 * Uses a polyline of odom poses (T_odom_base) spanning the scan, composed with
 * a fixed T_base_lidar. Assume base_link → lidar is a rigid mount — do not
 * look that transform up from TF.
 *
 * T_odom_lidar(t) = T_odom_base(t) * T_base_lidar
 * p_ref = T_ref^{-1} * T_odom_lidar(t) * p_lidar
 *
 * Output stays in cloud.frame_id (lidar); stamp becomes the reference time.
 * time_offset_s on points = seconds from the cloud header stamp.
 *
 * odom_poses must be sorted by stamp and cover [scan_start, scan_end]
 * (caller should include one sample on each side). No TF wait.
 */
DeskewResult deskew_lidar_frame(
  LidarFrame & cloud,
  const Eigen::Isometry3d & T_base_lidar,
  const std::vector<StampedOdomPose> & odom_poses,
  DeskewReference reference);

}  // namespace composite_sensing
