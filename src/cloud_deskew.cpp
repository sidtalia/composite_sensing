#include "composite_sensing/cloud_deskew.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace composite_sensing
{
namespace
{
constexpr double kEpsilon = 1e-12;

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

inline void applyIsometryFloat(
  const Eigen::Matrix3f & R, const Eigen::Vector3f & t, LidarPoint & p)
{
  const Eigen::Vector3f v = R * Eigen::Vector3f(p.x, p.y, p.z) + t;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
}

/** Interpolate T_odom_base at stamp_ns. poses is sorted and covers stamp_ns. */
Eigen::Isometry3d poseAt(
  const std::vector<StampedOdomPose> & poses, int64_t stamp_ns)
{
  if (poses.size() == 1) {
    return poses.front().T_odom_base;
  }
  auto it = std::lower_bound(
    poses.begin(), poses.end(), stamp_ns,
    [](const StampedOdomPose & p, int64_t t) { return p.stamp_ns < t; });
  if (it == poses.begin()) {
    return poses.front().T_odom_base;
  }
  if (it == poses.end()) {
    return poses.back().T_odom_base;
  }
  const StampedOdomPose & b = *it;
  const StampedOdomPose & a = *std::prev(it);
  if (b.stamp_ns <= a.stamp_ns) {
    return b.T_odom_base;
  }
  const double alpha =
    static_cast<double>(stamp_ns - a.stamp_ns) /
    static_cast<double>(b.stamp_ns - a.stamp_ns);
  return interpolateSE3(a.T_odom_base, b.T_odom_base, alpha);
}

}  // namespace

DeskewResult deskew_lidar_frame(
  LidarFrame & cloud,
  const Eigen::Isometry3d & T_base_lidar,
  const std::vector<StampedOdomPose> & odom_poses,
  DeskewReference reference)
{
  DeskewResult result;
  const auto t0 = std::chrono::steady_clock::now();

  if (cloud.points.empty()) {
    result.error = "empty cloud";
    return result;
  }
  if (odom_poses.empty()) {
    result.error = "no odom poses spanning scan";
    return result;
  }

  double max_offset_sec = 0.0;
  for (const auto & p : cloud.points) {
    if (std::isfinite(p.time_offset_s)) {
      max_offset_sec = std::max(max_offset_sec, static_cast<double>(p.time_offset_s));
    }
  }

  const int64_t start_ns = cloud.timestamp_ns;
  const int64_t end_ns =
    start_ns + static_cast<int64_t>(std::llround(max_offset_sec * 1e9));
  const int64_t ref_ns =
    (reference == DeskewReference::Start) ? start_ns : end_ns;

  if (odom_poses.front().stamp_ns > start_ns ||
      odom_poses.back().stamp_ns < end_ns)
  {
    result.error = "odom poses do not cover scan interval";
    result.elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    return result;
  }

  const Eigen::Isometry3d T_ref =
    poseAt(odom_poses, ref_ns) * T_base_lidar;
  const Eigen::Isometry3d T_ref_inv = T_ref.inverse();

  if (max_offset_sec < kEpsilon || odom_poses.size() == 1) {
    const Eigen::Isometry3d T_rel =
      T_ref_inv * (odom_poses.front().T_odom_base * T_base_lidar);
    const Eigen::Matrix3f R = T_rel.linear().cast<float>();
    const Eigen::Vector3f t = T_rel.translation().cast<float>();
    for (auto & p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      applyIsometryFloat(R, t, p);
    }
  } else {
    for (auto & p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      if (!std::isfinite(p.time_offset_s)) {
        continue;
      }
      const int64_t t_ns =
        start_ns + static_cast<int64_t>(std::llround(
          static_cast<double>(p.time_offset_s) * 1e9));
      const Eigen::Isometry3d T_t =
        poseAt(odom_poses, t_ns) * T_base_lidar;
      const Eigen::Isometry3d T_rel = T_ref_inv * T_t;
      applyIsometryFloat(
        T_rel.linear().cast<float>(), T_rel.translation().cast<float>(), p);
    }
  }

  cloud.timestamp_ns = ref_ns;
  result.ok = true;
  result.elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();
  return result;
}

}  // namespace composite_sensing
