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
 * Mesh particle filter against a triangle mesh (Vulkan RT / Embree).
 * Particles are SE(3): translation + full quaternion.
 * Motion: odom delta when tracking and OdomDelta is valid, otherwise
 * constant-twist from the filter mean (global, or tracking with odom off).
 */
class ConstantTwistMeshPfBackend : public MeshLocalizerBackend
{
public:
  ConstantTwistMeshPfBackend();
  ~ConstantTwistMeshPfBackend() override;

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
  size_t particle_count() const { return particles_.size(); }
  bool tracking() const { return tracking_; }

  /** Last localize() wall times (ms). Raycast is the mesh matcher. */
  struct Timing
  {
    double fill_ms{0};
    double raycast_ms{0};
    double weight_ms{0};
    double localize_ms{0};
    size_t n_rays{0};
  };
  Timing last_timing() const { return last_timing_; }

private:
  struct Particle
  {
    double x{0}, y{0}, z{0};
    double qx{0}, qy{0}, qz{0}, qw{1};
    double weight{1.0};
  };

  /** Lidar sample in base_link: unit direction from lidar origin + range. */
  struct BaseSample
  {
    float dx{0}, dy{0}, dz{1};
    float range{0};
  };

  void scatter_particles();
  void resize_ensemble(int n);
  int tracking_particle_count() const;
  void reset_motion_state();
  void motion_update(double dt, const OdomDelta & odom);
  void sensor_update(const LidarFrame & cloud);
  void resample();
  PoseMap mean_pose() const;

  MeshLocalizerConfig cfg_{};
  std::unique_ptr<MeshRaycaster> map_;
  std::string raycast_backend_used_{"embree"};
  std::vector<Particle> particles_;
  std::vector<Ray> rays_;
  std::vector<Hit> hits_;
  std::vector<float> meas_;
  std::vector<BaseSample> samples_base_;
  mutable std::mutex mu_;

  bool initialized_{false};
  bool tracking_{false};
  bool have_mean_{false};
  PoseMap last_mean_{};
  int64_t last_stamp_ns_{0};
  double vx_{0}, vy_{0}, vz_{0};
  double wx_{0}, wy_{0}, wz_{0};

  std::mt19937 rng_{std::random_device{}()};
  Timing last_timing_{};
};

}  // namespace composite_sensing
