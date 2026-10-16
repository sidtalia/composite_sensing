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
  /**
   * Row-major 6×6 pose covariance (x, y, z, rot_x, rot_y, rot_z).
   * Rotation block is the rotation-vector tangent at the mean (map frame).
   */
  std::array<double, 36> covariance{};
  /** sqrt(max(var_x, var_y)); floor applied. Legacy EKF scalar. */
  double position_stddev_m{0.5};
  bool valid{false};
};

/** Incremental odom: T_base(t−1)←base(t), motion in the previous body frame. */
struct OdomDelta
{
  bool valid{false};
  std::array<double, 3> translation{0, 0, 0};
  std::array<double, 4> rotation_xyzw{0, 0, 0, 1};
};

struct MeshLocalizerConfig
{
  std::string map_file;
  std::string map_frame{"map"};
  std::string base_frame{"base_link"};
  /** Extrinsic: lidar pose in base_link (xyz meters, xyzw). */
  std::array<double, 3> t_base_lidar{0, 0, 0};
  std::array<double, 4> q_base_lidar{0, 0, 0, 1};  // xyzw
  int num_particles{100000};
  /** Ensemble size after a pose/TF seed (RMCL pose-init analogue). 0 = same as num_particles. */
  int tracking_particles{10000};
  int beam_samples{64};
  double localize_hz{10.0};
  /** Uniform init bounding box [xmin,ymin,zmin,xmax,ymax,zmax] meters. */
  std::array<double, 6> init_bb{-10, -10, 0, 10, 10, 2};
  /** Uniform init attitude [min, max] rad (ZYX / tf2 RPY). ±45° roll/pitch. */
  std::array<double, 2> init_roll{-0.7853981633974483, 0.7853981633974483};
  std::array<double, 2> init_pitch{-0.7853981633974483, 0.7853981633974483};
  std::array<double, 2> init_yaw{-3.14159, 3.14159};
  double still_trans_noise_m{0.05};
  double still_rp_noise_rad{0.05};   // roll & pitch
  double still_yaw_noise_rad{0.05};
  double dist_noise_gain{0.1};   // σ_trans += gain * |Δp|
  double rot_rp_noise_gain{0.2};
  double rot_yaw_noise_gain{0.2};
  double tracking_still_trans_noise_m{0.01};
  double tracking_still_rp_noise_rad{0.008726646259971648};
  double tracking_still_yaw_noise_rad{0.008726646259971648};
  double tracking_dist_noise_gain{0.02};
  double tracking_rot_rp_noise_gain{0.05};
  double tracking_rot_yaw_noise_gain{0.05};
  /**
   * Tracking motion prior: "odom" (TF delta), "twist" (mean velocity),
   * "still" (Δ=0, sample around last pose + process noise).
   */
  std::string tracking_motion{"odom"};
  double twist_lpf_alpha{0.3};   // 0..1, higher = snappier
  double dist_sigma_m{0.25};     // sensor model
  double max_ray_range_m{50.0};
  bool global_init_on_start{true};
  /** "auto" (Vulkan RT then Embree), "vulkan", or "embree". */
  std::string raycast_backend{"auto"};
  /** Tracking after pose seed: "pf" (particles) or "micp" (MICP-L P2L). */
  std::string tracking_backend{"micp"};
  int micp_iterations{5};
  int micp_samples{1024};
  double micp_max_dist_m{0.5};
  double micp_adaptive_max_dist_min_m{0.05};
};

/**
 * Map-relative localizer. Tracking motion is odom delta, constant-twist,
 * or still (Δ=0 around last pose), see MeshLocalizerConfig::tracking_motion.
 */
class MeshLocalizerBackend
{
public:
  virtual ~MeshLocalizerBackend() = default;

  virtual void configure(const MeshLocalizerConfig & config) = 0;

  /** Spread particles uniformly over init_bb (kidnapped / cold start). */
  virtual void global_init() = 0;

  /**
   * Gaussian scatter around a map←base pose (xyzw). Used to seed from an
   * existing TF (e.g. map←odom ∘ odom←base) without touching init_bb.
   */
  virtual void seed_from_pose(
    const std::array<double, 3> & translation,
    const std::array<double, 4> & rotation_xyzw,
    double trans_std_m,
    double rot_std_rad) = 0;

  /** Stop filtering until the next seed / global_init (ArUco hunt wait). */
  virtual void reset() = 0;

  /**
   * One filter step on a lidar cloud in the sensor frame.
   * Returns nullopt if not yet localized / map missing.
   */
  virtual std::optional<PoseMap> localize(
    const LidarFrame & cloud, const OdomDelta & odom = {}) = 0;
};


}  // namespace composite_sensing
