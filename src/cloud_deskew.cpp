#include "composite_sensing/cloud_deskew.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace composite_sensing
{
namespace
{
constexpr double kEpsilon = 1e-12;
// Discrete pose bins so we slerp once per bin, not once per point.
constexpr int kPoseBins = 32;

Eigen::Isometry3d interpolateSE3(
  const Eigen::Isometry3d & T0,
  const Eigen::Isometry3d & T1,
  double alpha)
{
  alpha = std::clamp(alpha, 0.0, 1.0);
  const Eigen::Quaterniond q0(T0.rotation());
  const Eigen::Quaterniond q1(T1.rotation());
  const Eigen::Quaterniond q = q0.slerp(alpha, q1).normalized();
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = q.toRotationMatrix();
  T.translation() = (1.0 - alpha) * T0.translation() + alpha * T1.translation();
  return T;
}

bool lookupEigenTransform(
  tf2_ros::Buffer & tf_buffer,
  const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Time & time,
  double timeout_s,
  Eigen::Isometry3d & out,
  const rclcpp::Logger & logger,
  rclcpp::Clock & clock)
{
  try {
    // timeout_s <= 0 → fail-fast (no wait for future TF).
    const auto timeout = tf2::durationFromSec(std::max(0.0, timeout_s));
    const geometry_msgs::msg::TransformStamped tf =
      tf_buffer.lookupTransform(target_frame, source_frame, time, timeout);
    out = tf2::transformToEigen(tf.transform);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger, clock, 2000,
      "deskew TF lookup failed (%s <- %s @ %.6f): %s",
      target_frame.c_str(), source_frame.c_str(), time.seconds(), ex.what());
    return false;
  }
}

inline void applyIsometryFloat(
  const Eigen::Matrix3f & R, const Eigen::Vector3f & t, LidarPoint & p)
{
  const Eigen::Vector3f v = R * Eigen::Vector3f(p.x, p.y, p.z) + t;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
}
}  // namespace

DeskewResult deskew_lidar_frame(
  LidarFrame & cloud,
  tf2_ros::Buffer & tf_buffer,
  const std::string & motion_frame,
  DeskewReference reference,
  double tf_lookup_timeout_s,
  const rclcpp::Logger & logger,
  rclcpp::Clock & clock)
{
  DeskewResult result;
  const auto t0 = std::chrono::steady_clock::now();

  if (cloud.points.empty() || cloud.frame_id.empty() || motion_frame.empty()) {
    result.error = "empty cloud or frame";
    result.elapsed_ms = 0.0;
    return result;
  }

  const std::string & lidar_frame = cloud.frame_id;
  if (motion_frame == lidar_frame) {
    result.error =
      "motion_frame equals lidar_frame (no motion); set deskew.motion_frame=odom";
    result.elapsed_ms = 0.0;
    return result;
  }

  double max_offset_sec = 0.0;
  for (const auto & p : cloud.points) {
    if (std::isfinite(p.time_offset_s)) {
      max_offset_sec = std::max(max_offset_sec, static_cast<double>(p.time_offset_s));
    }
  }

  const rclcpp::Time cloud_stamp(cloud.timestamp_ns, RCL_ROS_TIME);
  const rclcpp::Time start_time = cloud_stamp;
  const rclcpp::Time end_time(
    cloud.timestamp_ns + static_cast<int64_t>(std::llround(max_offset_sec * 1e9)),
    RCL_ROS_TIME);
  const rclcpp::Time reference_time =
    (reference == DeskewReference::Start) ? start_time : end_time;

  // T_motion_lidar at scan bounds (needs odom→base + base→lidar in TF).
  Eigen::Isometry3d T_start = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_end = Eigen::Isometry3d::Identity();

  if (!lookupEigenTransform(
        tf_buffer, motion_frame, lidar_frame, start_time, tf_lookup_timeout_s,
        T_start, logger, clock))
  {
    result.error = "TF at scan start";
    result.elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    return result;
  }
  if (max_offset_sec < kEpsilon) {
    T_end = T_start;
  } else if (!lookupEigenTransform(
               tf_buffer, motion_frame, lidar_frame, end_time, tf_lookup_timeout_s,
               T_end, logger, clock))
  {
    result.error = "TF at scan end";
    result.elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    return result;
  }

  const Eigen::Isometry3d T_reference =
    (reference == DeskewReference::Start) ? T_start : T_end;
  const Eigen::Isometry3d T_reference_inv = T_reference.inverse();
  // Warp into lidar frame at reference time: p_ref = (T_ref^{-1} T(t)) p
  const Eigen::Isometry3d T0_rel = T_reference_inv * T_start;
  const Eigen::Isometry3d T1_rel = T_reference_inv * T_end;
  const double time_span = max_offset_sec;

  if (time_span < kEpsilon) {
    const Eigen::Matrix3f R = T0_rel.linear().cast<float>();
    const Eigen::Vector3f t = T0_rel.translation().cast<float>();
    for (auto & p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      applyIsometryFloat(R, t, p);
    }
  } else {
    struct PoseBin
    {
      Eigen::Matrix3f R;
      Eigen::Vector3f t;
    };
    PoseBin bins[kPoseBins];
    for (int i = 0; i < kPoseBins; ++i) {
      const double alpha =
        (kPoseBins == 1) ? 0.0 : static_cast<double>(i) / static_cast<double>(kPoseBins - 1);
      const Eigen::Isometry3d Ti = interpolateSE3(T0_rel, T1_rel, alpha);
      bins[i].R = Ti.linear().cast<float>();
      bins[i].t = Ti.translation().cast<float>();
    }
    const double inv_span = 1.0 / time_span;
    const double bin_scale = static_cast<double>(kPoseBins - 1);

    for (auto & p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      if (!std::isfinite(p.time_offset_s)) {
        continue;
      }
      const double alpha = std::clamp(
        static_cast<double>(p.time_offset_s) * inv_span, 0.0, 1.0);
      const int bi = static_cast<int>(std::lround(alpha * bin_scale));
      applyIsometryFloat(bins[bi].R, bins[bi].t, p);
    }
  }

  // Stay in lidar frame; only the stamp moves to the reference instant.
  cloud.timestamp_ns = reference_time.nanoseconds();

  result.ok = true;
  result.elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();
  return result;
}

}  // namespace composite_sensing
