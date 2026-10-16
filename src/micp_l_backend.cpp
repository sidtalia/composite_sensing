#include "composite_sensing/micp_l_backend.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace composite_sensing
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinPosVar = 0.02 * 0.02;
constexpr double kMinRotVar = (0.5 * kPi / 180.0) * (0.5 * kPi / 180.0);

Eigen::Quaterniond q_xyzw(double qx, double qy, double qz, double qw)
{
  Eigen::Quaterniond q(qw, qx, qy, qz);
  if (q.norm() < 1e-12) {
    return Eigen::Quaterniond::Identity();
  }
  return q.normalized();
}

Eigen::Isometry3d T_from_xyzw(
  const std::array<double, 3> & t, const std::array<double, 4> & xyzw)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(t[0], t[1], t[2]);
  T.linear() = q_xyzw(xyzw[0], xyzw[1], xyzw[2], xyzw[3]).toRotationMatrix();
  return T;
}

Eigen::Isometry3d exp_se3(const Eigen::Matrix<double, 6, 1> & xi)
{
  Eigen::Isometry3d dT = Eigen::Isometry3d::Identity();
  dT.translation() = xi.head<3>();
  const Eigen::Vector3d w = xi.tail<3>();
  const double a = w.norm();
  if (a < 1e-12) {
    return dT;
  }
  dT.linear() = Eigen::AngleAxisd(a, w / a).toRotationMatrix();
  return dT;
}

}  // namespace

void MicpLBackend::configure(const MeshLocalizerConfig & config)
{
  std::lock_guard<std::mutex> lock(mu_);
  cfg_ = config;
  if (cfg_.map_file.empty()) {
    throw std::runtime_error("MicpLBackend: map_file required");
  }
  map_ = EmbreeMeshMap::load(cfg_.map_file);
  raycast_backend_used_ = map_->backend_name();
  initialized_ = false;
  last_stamp_ns_ = 0;
}

void MicpLBackend::global_init()
{
  std::lock_guard<std::mutex> lock(mu_);
  initialized_ = false;
}

void MicpLBackend::seed_from_pose(
  const std::array<double, 3> & translation,
  const std::array<double, 4> & rotation_xyzw,
  double trans_std_m,
  double rot_std_rad)
{
  (void)trans_std_m;
  (void)rot_std_rad;
  std::lock_guard<std::mutex> lock(mu_);
  T_map_base_ = T_from_xyzw(translation, rotation_xyzw);
  cov_.setIdentity();
  cov_.block<3, 3>(0, 0) *= std::max(1e-4, trans_std_m * trans_std_m);
  cov_.block<3, 3>(3, 3) *= std::max(1e-8, rot_std_rad * rot_std_rad);
  initialized_ = true;
  last_stamp_ns_ = 0;
}

void MicpLBackend::reset()
{
  std::lock_guard<std::mutex> lock(mu_);
  initialized_ = false;
  last_stamp_ns_ = 0;
}

void MicpLBackend::apply_odom(const OdomDelta & odom)
{
  if (!odom.valid) {
    return;
  }
  Eigen::Isometry3d dT = Eigen::Isometry3d::Identity();
  dT.translation() = Eigen::Vector3d(
    odom.translation[0], odom.translation[1], odom.translation[2]);
  dT.linear() = q_xyzw(
    odom.rotation_xyzw[0], odom.rotation_xyzw[1],
    odom.rotation_xyzw[2], odom.rotation_xyzw[3])
    .toRotationMatrix();
  T_map_base_ = T_map_base_ * dT;
}

PoseMap MicpLBackend::pose_from_T() const
{
  PoseMap out;
  const Eigen::Vector3d t = T_map_base_.translation();
  const Eigen::Quaterniond q(T_map_base_.linear());
  out.translation = {t.x(), t.y(), t.z()};
  out.rotation = {q.x(), q.y(), q.z(), q.w()};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      out.covariance[static_cast<size_t>(i) * 6u + static_cast<size_t>(j)] =
        cov_(i, j);
    }
  }
  out.position_stddev_m = std::sqrt(std::max(cov_(0, 0), cov_(1, 1)));
  out.valid = true;
  return out;
}

std::optional<PoseMap> MicpLBackend::localize(
  const LidarFrame & cloud, const OdomDelta & odom)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!map_ || !initialized_ || cloud.points.empty()) {
    return std::nullopt;
  }

  using Clock = std::chrono::steady_clock;
  const auto t0 = Clock::now();
  const auto ms = [](Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
  };

  apply_odom(odom);

  const size_t n_cloud = cloud.points.size();
  const size_t n_samp = std::min(
    n_cloud, static_cast<size_t>(std::max(16, cfg_.micp_samples)));
  idx_.resize(n_samp);
  if (n_samp == n_cloud) {
    for (size_t i = 0; i < n_samp; ++i) {
      idx_[i] = i;
    }
  } else {
    std::uniform_int_distribution<size_t> pick(0, n_cloud - 1);
    for (size_t i = 0; i < n_samp; ++i) {
      idx_[i] = pick(rng_);
    }
  }

  const Eigen::Isometry3d T_base_lidar = T_from_xyzw(
    cfg_.t_base_lidar, cfg_.q_base_lidar);
  const float tmax = static_cast<float>(cfg_.max_ray_range_m);
  const int n_iter = std::max(1, cfg_.micp_iterations);
  const double sig2 = std::max(1e-6, cfg_.dist_sigma_m * cfg_.dist_sigma_m);
  const double huber_k = std::max(0.05, cfg_.micp_adaptive_max_dist_min_m * 2.0);

  if (rays_.size() < n_samp) {
    rays_.resize(n_samp);
    hits_.resize(n_samp);
  }

  double fill_ms = 0.0;
  double raycast_ms = 0.0;
  double solve_ms = 0.0;
  size_t last_inliers = 0;

  for (int it = 0; it < n_iter; ++it) {
    const double progress =
      n_iter > 1 ? static_cast<double>(it) / static_cast<double>(n_iter - 1) : 1.0;
    const double max_dist =
      cfg_.micp_max_dist_m * (1.0 - progress) +
      cfg_.micp_adaptive_max_dist_min_m * progress;

    const Eigen::Isometry3d T_map_lidar = T_map_base_ * T_base_lidar;
    const Eigen::Vector3d o = T_map_lidar.translation();
    const Eigen::Matrix3d R = T_map_lidar.linear();

    const auto t_fill0 = Clock::now();
    for (size_t s = 0; s < n_samp; ++s) {
      const auto & pt = cloud.points[idx_[s]];
      const Eigen::Vector3d p_l(pt.x, pt.y, pt.z);
      const double range = p_l.norm();
      Eigen::Vector3d dir_l = p_l;
      if (range >= 1e-3) {
        dir_l /= range;
      } else {
        dir_l = Eigen::Vector3d(1.0, 0.0, 0.0);
      }
      const Eigen::Vector3d dir_m = R * dir_l;
      Ray & ray = rays_[s];
      ray.ox = static_cast<float>(o.x());
      ray.oy = static_cast<float>(o.y());
      ray.oz = static_cast<float>(o.z());
      ray.tmin = 0.05f;
      ray.dx = static_cast<float>(dir_m.x());
      ray.dy = static_cast<float>(dir_m.y());
      ray.dz = static_cast<float>(dir_m.z());
      ray.tmax = tmax;
    }
    const auto t_cast0 = Clock::now();
    map_->batchIntersectPlane(rays_.data(), hits_.data(), n_samp);
    const auto t_cast1 = Clock::now();
    fill_ms += ms(t_fill0, t_cast0);
    raycast_ms += ms(t_cast0, t_cast1);

    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
    size_t n_in = 0;
    for (size_t s = 0; s < n_samp; ++s) {
      if (!hits_[s].hit) {
        continue;
      }
      const auto & pt = cloud.points[idx_[s]];
      const Eigen::Vector3d p_l(pt.x, pt.y, pt.z);
      const double range = p_l.norm();
      if (!(range > 0.2) || range > cfg_.max_ray_range_m) {
        continue;
      }
      const double t_hit = static_cast<double>(hits_[s].t);
      if (std::abs(t_hit - range) > max_dist) {
        continue;
      }
      const Eigen::Vector3d p_b = T_base_lidar * p_l;
      const Eigen::Vector3d p_m = T_map_base_ * p_b;
      const Eigen::Vector3d n(
        hits_[s].nx, hits_[s].ny, hits_[s].nz);
      const Eigen::Vector3d dir_m(
        rays_[s].dx, rays_[s].dy, rays_[s].dz);
      const Eigen::Vector3d q = o + t_hit * dir_m;
      const double r = n.dot(p_m - q);
      const double ar = std::abs(r);
      const double w = ar <= huber_k ? 1.0 : (huber_k / ar);
      Eigen::Matrix<double, 6, 1> J;
      J.head<3>() = n;
      J.tail<3>() = p_m.cross(n);
      H.noalias() += w * (J * J.transpose());
      b.noalias() += w * J * (-r);
      ++n_in;
    }
    last_inliers = n_in;
    const auto t_sol0 = Clock::now();
    if (n_in >= 12) {
      const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(H);
      if (ldlt.info() == Eigen::Success) {
        Eigen::Matrix<double, 6, 1> xi = ldlt.solve(b);
        const double dt = xi.head<3>().norm();
        const double dr = xi.tail<3>().norm();
        if (dt > 0.5) {
          xi.head<3>() *= 0.5 / dt;
        }
        if (dr > 0.35) {
          xi.tail<3>() *= 0.35 / dr;
        }
        T_map_base_ = exp_se3(xi) * T_map_base_;
        const Eigen::Matrix<double, 6, 6> Hinv = ldlt.solve(
          Eigen::Matrix<double, 6, 6>::Identity());
        cov_ = Hinv * sig2;
      }
    }
    solve_ms += ms(t_sol0, Clock::now());
  }

  cov_(0, 0) = std::max(cov_(0, 0), kMinPosVar);
  cov_(1, 1) = std::max(cov_(1, 1), kMinPosVar);
  cov_(2, 2) = std::max(cov_(2, 2), kMinPosVar);
  cov_(3, 3) = std::max(cov_(3, 3), kMinRotVar);
  cov_(4, 4) = std::max(cov_(4, 4), kMinRotVar);
  cov_(5, 5) = std::max(cov_(5, 5), kMinRotVar);

  last_stamp_ns_ = cloud.timestamp_ns;
  last_timing_.fill_ms = fill_ms;
  last_timing_.raycast_ms = raycast_ms;
  last_timing_.solve_ms = solve_ms;
  last_timing_.localize_ms = ms(t0, Clock::now());
  last_timing_.n_rays = n_samp * static_cast<size_t>(n_iter);
  last_timing_.n_inliers = last_inliers;
  last_timing_.iterations = n_iter;
  return pose_from_T();
}

}  // namespace composite_sensing
