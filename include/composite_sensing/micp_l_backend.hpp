#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "composite_sensing/embree_mesh_map.hpp"
#include "composite_sensing/mesh_localizer_backend.hpp"

namespace composite_sensing
{

/**
 * MICP-L-style tracking: ray-casting correspondences + point-to-plane GN.
 * Needs a pose seed (ArUco / TF). Kidnap / bbox still uses the particle filter.
 */
class MicpLBackend : public MeshLocalizerBackend
{
public:
  void configure(const MeshLocalizerConfig & config) override;
  void global_init() override;
  void seed_from_pose(
    const std::array<double, 3> & translation,
    const std::array<double, 4> & rotation_xyzw,
    double trans_std_m,
    double rot_std_rad) override;
  void reset() override;
  std::optional<PoseMap> localize(
    const LidarFrame & cloud, const OdomDelta & odom) override;

  const std::string & raycast_backend_used() const { return raycast_backend_used_; }
  size_t triangle_count() const { return map_ ? map_->triangle_count() : 0; }
  bool tracking() const { return initialized_; }

  struct Timing
  {
    double fill_ms{0};
    double raycast_ms{0};
    double solve_ms{0};
    double localize_ms{0};
    size_t n_rays{0};
    size_t n_inliers{0};
    int iterations{0};
  };
  Timing last_timing() const { return last_timing_; }

private:
  PoseMap pose_from_T() const;
  void apply_odom(const OdomDelta & odom);

  MeshLocalizerConfig cfg_{};
  std::unique_ptr<EmbreeMeshMap> map_;
  std::string raycast_backend_used_{"embree"};
  Eigen::Isometry3d T_map_base_{Eigen::Isometry3d::Identity()};
  Eigen::Matrix<double, 6, 6> cov_{Eigen::Matrix<double, 6, 6>::Identity()};
  bool initialized_{false};
  int64_t last_stamp_ns_{0};

  std::vector<Ray> rays_;
  std::vector<HitPlane> hits_;
  std::vector<size_t> idx_;
  mutable std::mutex mu_;
  std::mt19937 rng_{std::random_device{}()};
  Timing last_timing_{};
};

}  // namespace composite_sensing
