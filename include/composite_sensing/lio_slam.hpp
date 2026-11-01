#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include "composite_sensing/fast_lio_odometry.hpp"
#include "composite_sensing/lidar_device.hpp"

namespace composite_sensing
{

/** One optimized IMU pose in the FAST-LIO world. */
struct SlamPose
{
  int64_t stamp_ns{0};
  Eigen::Isometry3d T_world_imu{Eigen::Isometry3d::Identity()};
};

struct ScanContext
{
  static constexpr int kRings = 20;
  static constexpr int kSectors = 60;
  /// nan = no return in that ring/sector. Row-major, ring * kSectors + sector.
  std::array<float, kRings * kSectors> cell{};
};

ScanContext make_scan_context(const std::vector<std::array<float, 3>> & pts);

/** Column-shift distance. 0 is identical. best_shift is sectors, 6 deg each. */
float scan_context_distance(const ScanContext & a, const ScanContext & b, int & best_shift);

/**
 * Scan-Context loop closure and a GTSAM pose graph, on their own thread.
 * FAST-LIO keeps publishing the uncorrected odometry. This rewrites keyframe
 * poses when a revisit agrees.
 *
 * T_imu_lidar is the lidar pose in the IMU frame (FAST-LIO extrinsic).
 * The callback runs on the slam thread.
 *
 * ponytail: loop candidates are a linear scan of keyframes, and ICP starts
 * from the odometry relative pose with a 2 m correspondence gate. A ring-key
 * tree and a yaw-only Scan Context seed are the upgrade past a few hundred
 * keyframes or when drift exceeds that gate.
 */
class LioSlam
{
public:
  using Callback = std::function<void(const std::vector<SlamPose> &)>;

  LioSlam(const Eigen::Isometry3d & T_imu_lidar, Callback cb);
  ~LioSlam();

  LioSlam(const LioSlam &) = delete;
  LioSlam & operator=(const LioSlam &) = delete;

  /// Copies a keyframe onto the slam thread when the IMU has moved 1 m.
  void push(const LidarFrame & cloud, const FastLioOdom & odom);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace composite_sensing
