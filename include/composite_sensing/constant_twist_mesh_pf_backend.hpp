#pragma once

#include <memory>
#include <mutex>
#include <random>
#include <vector>

#include "composite_sensing/mesh_localizer_backend.hpp"

namespace composite_sensing
{

struct EmbreeMeshMap;

/**
 * RMCL-style particle filter against a triangle mesh (Embree raycast).
 * Motion model: constant twist estimated by differentiating the filter mean
 * (starts at 0). No external odom — safe to fuse pose into an EKF that already
 * consumes VSLAM/wheels.
 */
class ConstantTwistMeshPfBackend : public MeshLocalizerBackend
{
public:
  ConstantTwistMeshPfBackend();
  ~ConstantTwistMeshPfBackend() override;

  void configure(const MeshLocalizerConfig & config) override;
  void global_init() override;
  std::optional<PoseMap> localize(const LidarFrame & cloud) override;

private:
  struct Particle
  {
    double x{0}, y{0}, z{0};
    double yaw{0};
    double weight{1.0};
  };

  void motion_update(double dt);
  void sensor_update(const LidarFrame & cloud);
  void resample();
  PoseMap mean_pose() const;
  double ray_error(
    double bx, double by, double bz, double byaw,
    float px, float py, float pz) const;

  MeshLocalizerConfig cfg_{};
  std::unique_ptr<EmbreeMeshMap> map_;
  std::vector<Particle> particles_;
  mutable std::mutex mu_;

  bool have_mean_{false};
  PoseMap last_mean_{};
  int64_t last_stamp_ns_{0};
  double vx_{0}, vy_{0}, vz_{0}, wyaw_{0};

  std::mt19937 rng_{std::random_device{}()};
};

}  // namespace composite_sensing
