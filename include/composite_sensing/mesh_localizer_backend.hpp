#pragma once

#include <array>
#include <optional>
#include <string>

#include "composite_sensing/lidar_device.hpp"

namespace composite_sensing
{

/** Pose of base_link in map (FLU / ROS REP-103). rotation = xyzw. */
struct PoseMap
{
  std::array<double, 3> translation{0, 0, 0};
  std::array<double, 4> rotation{0, 0, 0, 1};  // xyzw
  /** Diagonal position std-dev hint (m) for EKF R; 0 = unknown. */
  double position_stddev_m{0.5};
  bool valid{false};
};

struct MeshLocalizerConfig
{
  std::string map_file;
  std::string map_frame{"map"};
  std::string base_frame{"base_link"};
  /** Extrinsic: lidar pose in base_link (xyz meters, xyzw). */
  std::array<double, 3> t_base_lidar{0, 0, 0};
  std::array<double, 4> q_base_lidar{0, 0, 0, 1};  // xyzw
  int num_particles{2000};
  int beam_samples{64};
  double localize_hz{10.0};
  /** Uniform init bounding box [xmin,ymin,zmin,xmax,ymax,zmax] meters. */
  std::array<double, 6> init_bb{-10, -10, 0, 10, 10, 2};
  /** Yaw range for uniform init [min, max] rad. */
  std::array<double, 2> init_yaw{-3.14159, 3.14159};
  double still_trans_noise_m{0.05};
  double still_yaw_noise_rad{0.05};
  double dist_noise_gain{0.1};   // σ_trans += gain * |Δp|
  double yaw_noise_gain{0.2};    // σ_yaw += gain * |Δyaw|
  double twist_lpf_alpha{0.3};   // 0..1, higher = snappier
  double dist_sigma_m{0.25};     // sensor model
  double max_ray_range_m{50.0};
  bool global_init_on_start{true};
};

/**
 * Map-relative localizer. Motion prior is internal (constant-twist from own
 * pose history) — do not feed EKF/VSLAM odom here.
 */
class MeshLocalizerBackend
{
public:
  virtual ~MeshLocalizerBackend() = default;

  virtual void configure(const MeshLocalizerConfig & config) = 0;

  /** Spread particles uniformly over init_bb (kidnapped / cold start). */
  virtual void global_init() = 0;

  /**
   * One filter step on a lidar cloud in the sensor frame.
   * Returns nullopt if not yet localized / map missing.
   */
  virtual std::optional<PoseMap> localize(const LidarFrame & cloud) = 0;
};

}  // namespace composite_sensing
