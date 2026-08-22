#pragma once

#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include "composite_sensing/mesh_localizer_backend.hpp"
#include "composite_sensing/mesh_raycaster.hpp"

namespace composite_sensing
{

/**
 * RMCL-style particle filter against a triangle mesh.
 * Raycast backend is Vulkan RT when available, else Embree.
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

  const std::string & raycast_backend_used() const { return raycast_backend_used_; }
  size_t triangle_count() const { return map_ ? map_->triangle_count() : 0; }

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

  /** Fill ray + measured range for one particle / lidar sample. */
  void fill_ray(
    double bx, double by, double bz, double byaw,
    float px, float py, float pz,
    Ray & ray, float & meas) const;

  MeshLocalizerConfig cfg_{};
  std::unique_ptr<MeshRaycaster> map_;
  std::string raycast_backend_used_{"embree"};
  std::vector<Particle> particles_;
  std::vector<Ray> rays_;
  std::vector<Hit> hits_;
  std::vector<float> meas_;
  mutable std::mutex mu_;

  bool have_mean_{false};
  PoseMap last_mean_{};
  int64_t last_stamp_ns_{0};
  double vx_{0}, vy_{0}, vz_{0}, wyaw_{0};

  std::mt19937 rng_{std::random_device{}()};
};

}  // namespace composite_sensing
