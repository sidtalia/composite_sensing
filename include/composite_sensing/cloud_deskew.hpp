#pragma once

#include <cmath>
#include <string>

#include <Eigen/Geometry>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <tf2_ros/buffer.h>

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

/**
 * Motion-compensate points into the lidar frame at scan start or end.
 *
 * Looks up ``motion_frame ← lidar_frame`` over the scan (needs EKF
 * ``odom → base_link`` + static ``base_link → lidar``). Output stays in
 * ``cloud.frame_id`` (lidar); stamp becomes the reference time.
 * ``time_offset_s`` on points = seconds from the cloud header stamp.
 */
DeskewResult deskew_lidar_frame(
  LidarFrame & cloud,
  tf2_ros::Buffer & tf_buffer,
  const std::string & motion_frame,
  DeskewReference reference,
  double tf_lookup_timeout_s,
  const rclcpp::Logger & logger,
  rclcpp::Clock & clock);

}  // namespace composite_sensing
