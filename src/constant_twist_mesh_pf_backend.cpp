#include "composite_sensing/constant_twist_mesh_pf_backend.hpp"

#include "composite_sensing/mesh_raycaster.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace composite_sensing
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinPosVar = 0.05 * 0.05;
constexpr double kMinRotVar = (1.0 * kPi / 180.0) * (1.0 * kPi / 180.0);

Eigen::Quaterniond q_xyzw(double qx, double qy, double qz, double qw)
{
  Eigen::Quaterniond q(qw, qx, qy, qz);
  if (q.norm() < 1e-12) {
    return Eigen::Quaterniond::Identity();
  }
  return q.normalized();
}

void store_xyzw(const Eigen::Quaterniond & q, double & qx, double & qy, double & qz, double & qw)
{
  qx = q.x();
  qy = q.y();
  qz = q.z();
  qw = q.w();
}

Eigen::Quaterniond rpy_to_quat(double roll, double pitch, double yaw)
{
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

Eigen::Quaterniond rotvec_to_quat(const Eigen::Vector3d & r)
{
  const double a = r.norm();
  if (a < 1e-12) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(a, r / a));
}

Eigen::Vector3d quat_to_rotvec(Eigen::Quaterniond q)
{
  q.normalize();
  if (q.w() < 0.0) {
    q.coeffs() = -q.coeffs();
  }
  const Eigen::AngleAxisd aa(q);
  return aa.axis() * aa.angle();
}

Eigen::Vector3d rotate(const Eigen::Quaterniond & q, const Eigen::Vector3d & v)
{
  return q * v;
}

}  // namespace

ConstantTwistMeshPfBackend::ConstantTwistMeshPfBackend() = default;

ConstantTwistMeshPfBackend::~ConstantTwistMeshPfBackend() = default;

void ConstantTwistMeshPfBackend::resize_ensemble(int n)
{
  n = std::max(1, n);
  particles_.assign(static_cast<size_t>(n), {});
  const size_t n_rays = particles_.size() *
    static_cast<size_t>(std::max(1, cfg_.beam_samples));
  rays_.assign(n_rays, {});
  hits_.assign(n_rays, {});
  meas_.assign(n_rays, 0.0f);
}

int ConstantTwistMeshPfBackend::tracking_particle_count() const
{
  if (cfg_.tracking_particles > 0) {
    return cfg_.tracking_particles;
  }
  return std::max(1, cfg_.num_particles);
}

void ConstantTwistMeshPfBackend::scatter_particles()
{
  if (particles_.empty()) {
    resize_ensemble(cfg_.num_particles);
  }
  std::uniform_real_distribution<double> ux(cfg_.init_bb[0], cfg_.init_bb[3]);
  std::uniform_real_distribution<double> uy(cfg_.init_bb[1], cfg_.init_bb[4]);
  std::uniform_real_distribution<double> uz(cfg_.init_bb[2], cfg_.init_bb[5]);
  std::uniform_real_distribution<double> uroll(cfg_.init_roll[0], cfg_.init_roll[1]);
  std::uniform_real_distribution<double> upitch(cfg_.init_pitch[0], cfg_.init_pitch[1]);
  std::uniform_real_distribution<double> uyaw(cfg_.init_yaw[0], cfg_.init_yaw[1]);
  const double w = 1.0 / static_cast<double>(particles_.size());
  for (auto & p : particles_) {
    p.x = ux(rng_);
    p.y = uy(rng_);
    p.z = uz(rng_);
    const Eigen::Quaterniond q = rpy_to_quat(uroll(rng_), upitch(rng_), uyaw(rng_));
    store_xyzw(q, p.qx, p.qy, p.qz, p.qw);
    p.weight = w;
  }
  initialized_ = true;
  tracking_ = false;
}

void ConstantTwistMeshPfBackend::reset_motion_state()
{
  have_mean_ = false;
  last_stamp_ns_ = 0;
  vx_ = vy_ = vz_ = 0.0;
  wx_ = wy_ = wz_ = 0.0;
}

void ConstantTwistMeshPfBackend::configure(const MeshLocalizerConfig & config)
{
  std::lock_guard<std::mutex> lock(mu_);
  cfg_ = config;
  if (cfg_.map_file.empty()) {
    throw std::runtime_error("ConstantTwistMeshPfBackend: map_file required");
  }
  map_ = create_mesh_raycaster(cfg_.map_file, cfg_.raycast_backend, &raycast_backend_used_);
  resize_ensemble(cfg_.num_particles);
  initialized_ = false;
  tracking_ = false;
  reset_motion_state();
  if (cfg_.global_init_on_start) {
    scatter_particles();
  }
}

void ConstantTwistMeshPfBackend::global_init()
{
  std::lock_guard<std::mutex> lock(mu_);
  resize_ensemble(cfg_.num_particles);
  scatter_particles();
  reset_motion_state();
}

void ConstantTwistMeshPfBackend::seed_from_pose(
  const std::array<double, 3> & translation,
  const std::array<double, 4> & rotation_xyzw,
  double trans_std_m,
  double rot_std_rad)
{
  std::lock_guard<std::mutex> lock(mu_);
  resize_ensemble(tracking_particle_count());
  const double sig_t = std::max(1e-6, trans_std_m);
  const double sig_r = std::max(1e-6, rot_std_rad);
  std::normal_distribution<double> nt(0.0, sig_t);
  std::normal_distribution<double> nr(0.0, sig_r);
  const Eigen::Quaterniond q0 = q_xyzw(
    rotation_xyzw[0], rotation_xyzw[1], rotation_xyzw[2], rotation_xyzw[3]);
  const double w = 1.0 / static_cast<double>(particles_.size());
  for (auto & p : particles_) {
    p.x = translation[0] + nt(rng_);
    p.y = translation[1] + nt(rng_);
    p.z = translation[2] + nt(rng_);
    const Eigen::Vector3d noise(nr(rng_), nr(rng_), nr(rng_));
    const Eigen::Quaterniond q = (rotvec_to_quat(noise) * q0).normalized();
    store_xyzw(q, p.qx, p.qy, p.qz, p.qw);
    p.weight = w;
  }
  initialized_ = true;
  tracking_ = true;
  reset_motion_state();
}

void ConstantTwistMeshPfBackend::reset()
{
  std::lock_guard<std::mutex> lock(mu_);
  initialized_ = false;
  tracking_ = false;
  reset_motion_state();
}

void ConstantTwistMeshPfBackend::motion_update(double dt, const OdomDelta & odom)
{
  if (dt <= 0.0 || dt > 1.0) {
    dt = 1.0 / std::max(1.0, cfg_.localize_hz);
  }

  const bool tracking = tracking_;
  const double still_t = tracking ? cfg_.tracking_still_trans_noise_m : cfg_.still_trans_noise_m;
  const double still_rp =
    tracking ? cfg_.tracking_still_rp_noise_rad : cfg_.still_rp_noise_rad;
  const double still_yaw =
    tracking ? cfg_.tracking_still_yaw_noise_rad : cfg_.still_yaw_noise_rad;
  const double g_t = tracking ? cfg_.tracking_dist_noise_gain : cfg_.dist_noise_gain;
  const double g_rp = tracking ? cfg_.tracking_rot_rp_noise_gain : cfg_.rot_rp_noise_gain;
  const double g_yaw =
    tracking ? cfg_.tracking_rot_yaw_noise_gain : cfg_.rot_yaw_noise_gain;

  Eigen::Vector3d dp(0.0, 0.0, 0.0);
  Eigen::Quaterniond dq = Eigen::Quaterniond::Identity();
  bool body_frame = false;
  const bool still_prior = tracking && cfg_.tracking_motion == "still";
  if (still_prior) {
    // Δ = 0: particles stay at last pose except process noise.
  } else if (tracking && odom.valid) {
    dp = Eigen::Vector3d(
      odom.translation[0], odom.translation[1], odom.translation[2]);
    dq = q_xyzw(
      odom.rotation_xyzw[0], odom.rotation_xyzw[1],
      odom.rotation_xyzw[2], odom.rotation_xyzw[3]);
    body_frame = true;
  } else {
    dp = Eigen::Vector3d(vx_ * dt, vy_ * dt, vz_ * dt);
    dq = rotvec_to_quat(Eigen::Vector3d(wx_ * dt, wy_ * dt, wz_ * dt));
  }

  const double dist = dp.norm();
  const double dang = quat_to_rotvec(dq).norm();
  const double sig_t = still_t + g_t * dist;
  const double sig_rp = still_rp + g_rp * dang;
  const double sig_yaw = still_yaw + g_yaw * dang;
  std::normal_distribution<double> nt(0.0, std::max(1e-9, sig_t));
  std::normal_distribution<double> nrp(0.0, std::max(1e-9, sig_rp));
  std::normal_distribution<double> nyaw(0.0, std::max(1e-9, sig_yaw));

  for (auto & p : particles_) {
    const Eigen::Quaterniond q = q_xyzw(p.qx, p.qy, p.qz, p.qw);
    const Eigen::Vector3d d = body_frame ? (q * dp) : dp;
    p.x += d.x() + nt(rng_);
    p.y += d.y() + nt(rng_);
    p.z += d.z() + nt(rng_);
    const Eigen::Quaterniond q_prop = body_frame
      ? (q * dq).normalized()
      : (rotvec_to_quat(quat_to_rotvec(dq)) * q).normalized();
    const Eigen::Vector3d noise(nrp(rng_), nrp(rng_), nyaw(rng_));
    const Eigen::Quaterniond qn =
      (q_prop * rotvec_to_quat(noise)).normalized();
    store_xyzw(qn, p.qx, p.qy, p.qz, p.qw);
  }
}

void ConstantTwistMeshPfBackend::sensor_update(const LidarFrame & cloud)
{
  if (!map_ || cloud.points.empty() || particles_.empty()) {
    return;
  }
  using Clock = std::chrono::steady_clock;
  const auto ms = [](Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
  };

  const size_t n_samp = static_cast<size_t>(std::max(1, cfg_.beam_samples));
  std::vector<size_t> idx(n_samp);
  std::uniform_int_distribution<size_t> pick(0, cloud.points.size() - 1);
  for (size_t i = 0; i < n_samp; ++i) {
    idx[i] = pick(rng_);
  }

  const Eigen::Quaterniond q_bl = q_xyzw(
    cfg_.q_base_lidar[0], cfg_.q_base_lidar[1],
    cfg_.q_base_lidar[2], cfg_.q_base_lidar[3]);
  samples_base_.resize(n_samp);
  for (size_t s = 0; s < n_samp; ++s) {
    const auto & pt = cloud.points[idx[s]];
    const Eigen::Vector3d p_lidar(pt.x, pt.y, pt.z);
    const double range = p_lidar.norm();
    Eigen::Vector3d dir_l = p_lidar;
    if (range >= 1e-3) {
      dir_l /= range;
    } else {
      dir_l = Eigen::Vector3d(1.0, 0.0, 0.0);
    }
    const Eigen::Vector3d dir_b = rotate(q_bl, dir_l);
    samples_base_[s].dx = static_cast<float>(dir_b.x());
    samples_base_[s].dy = static_cast<float>(dir_b.y());
    samples_base_[s].dz = static_cast<float>(dir_b.z());
    samples_base_[s].range = static_cast<float>(range);
  }

  const size_t n_part = particles_.size();
  const size_t n_rays = n_part * n_samp;
  if (rays_.size() < n_rays) {
    rays_.resize(n_rays);
    hits_.resize(n_rays);
    meas_.resize(n_rays);
  }

  const auto t_fill0 = Clock::now();
  const Eigen::Vector3d t_bl(
    cfg_.t_base_lidar[0], cfg_.t_base_lidar[1], cfg_.t_base_lidar[2]);
  const float tmax = static_cast<float>(cfg_.max_ray_range_m);
  size_t k = 0;
  for (const auto & p : particles_) {
    const Eigen::Quaterniond q = q_xyzw(p.qx, p.qy, p.qz, p.qw);
    const Eigen::Matrix3d R = q.toRotationMatrix();
    const Eigen::Vector3d o = R * t_bl + Eigen::Vector3d(p.x, p.y, p.z);
    const float ox = static_cast<float>(o.x());
    const float oy = static_cast<float>(o.y());
    const float oz = static_cast<float>(o.z());
    for (size_t s = 0; s < n_samp; ++s, ++k) {
      const Eigen::Vector3d d = R * Eigen::Vector3d(
        samples_base_[s].dx, samples_base_[s].dy, samples_base_[s].dz);
      Ray & ray = rays_[k];
      ray.ox = ox;
      ray.oy = oy;
      ray.oz = oz;
      ray.tmin = 0.0f;
      ray.dx = static_cast<float>(d.x());
      ray.dy = static_cast<float>(d.y());
      ray.dz = static_cast<float>(d.z());
      ray.tmax = tmax;
      meas_[k] = samples_base_[s].range;
    }
  }
  const auto t_cast0 = Clock::now();
  map_->batchIntersect(rays_.data(), hits_.data(), n_rays);
  const auto t_cast1 = Clock::now();

  last_timing_.fill_ms = ms(t_fill0, t_cast0);
  last_timing_.raycast_ms = ms(t_cast0, t_cast1);
  last_timing_.n_rays = n_rays;

  const double miss_pen = cfg_.dist_sigma_m * 8.0;
  const double near_pen = cfg_.dist_sigma_m * 4.0;
  double wsum = 0.0;
  k = 0;
  for (auto & p : particles_) {
    double err_acc = 0.0;
    for (size_t s = 0; s < n_samp; ++s, ++k) {
      if (meas_[k] < 1e-3f) {
        err_acc += near_pen;
      } else if (hits_[k].hit == 0) {
        err_acc += miss_pen;
      } else {
        err_acc += std::abs(static_cast<double>(hits_[k].t) - static_cast<double>(meas_[k]));
      }
    }
    err_acc /= static_cast<double>(n_samp);
    const double like = std::exp(
      -0.5 * (err_acc * err_acc) /
      (cfg_.dist_sigma_m * cfg_.dist_sigma_m));
    p.weight *= std::max(like, 1e-12);
    wsum += p.weight;
  }
  if (wsum <= 0.0) {
    const double u = 1.0 / static_cast<double>(particles_.size());
    for (auto & p : particles_) {
      p.weight = u;
    }
  } else {
    for (auto & p : particles_) {
      p.weight /= wsum;
    }
  }
  last_timing_.weight_ms = ms(t_cast1, Clock::now());
}

void ConstantTwistMeshPfBackend::resample()
{
  const size_t N = particles_.size();
  double neff_inv = 0.0;
  for (const auto & p : particles_) {
    neff_inv += p.weight * p.weight;
  }
  const double neff = (neff_inv > 0.0) ? (1.0 / neff_inv) : 0.0;
  if (neff > 0.5 * static_cast<double>(N)) {
    return;
  }

  std::vector<Particle> next(N);
  std::vector<double> cdf(N);
  cdf[0] = particles_[0].weight;
  for (size_t i = 1; i < N; ++i) {
    cdf[i] = cdf[i - 1] + particles_[i].weight;
  }
  std::uniform_real_distribution<double> u01(0.0, 1.0);
  const double u0 = u01(rng_) / static_cast<double>(N);
  size_t j = 0;
  for (size_t i = 0; i < N; ++i) {
    const double u = u0 + static_cast<double>(i) / static_cast<double>(N);
    while (j + 1 < N && u > cdf[j]) {
      ++j;
    }
    next[i] = particles_[j];
    next[i].weight = 1.0 / static_cast<double>(N);
  }
  particles_.swap(next);
}

PoseMap ConstantTwistMeshPfBackend::mean_pose() const
{
  PoseMap out;
  if (particles_.empty()) {
    return out;
  }

  double x = 0, y = 0, z = 0;
  Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
  Eigen::Vector4d qref(1.0, 0.0, 0.0, 0.0);
  if (have_mean_) {
    qref = Eigen::Vector4d(
      last_mean_.rotation[3], last_mean_.rotation[0],
      last_mean_.rotation[1], last_mean_.rotation[2]);
  } else {
    qref = Eigen::Vector4d(
      particles_[0].qw, particles_[0].qx, particles_[0].qy, particles_[0].qz);
  }
  if (qref.norm() < 1e-12) {
    qref = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
  } else {
    qref.normalize();
  }

  for (const auto & p : particles_) {
    x += p.weight * p.x;
    y += p.weight * p.y;
    z += p.weight * p.z;
    Eigen::Vector4d q(p.qw, p.qx, p.qy, p.qz);
    if (q.norm() < 1e-12) {
      continue;
    }
    q.normalize();
    if (q.dot(qref) < 0.0) {
      q = -q;
    }
    A.noalias() += p.weight * (q * q.transpose());
  }
  out.translation = {x, y, z};

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(A);
  Eigen::Vector4d qm = es.eigenvectors().col(3);
  if (qm(0) < 0.0) {
    qm = -qm;
  }
  qm.normalize();
  out.rotation = {qm(1), qm(2), qm(3), qm(0)};

  const Eigen::Quaterniond qmean(qm(0), qm(1), qm(2), qm(3));
  Eigen::Matrix<double, 6, 6> S = Eigen::Matrix<double, 6, 6>::Zero();
  for (const auto & p : particles_) {
    Eigen::Matrix<double, 6, 1> xi;
    xi(0) = p.x - x;
    xi(1) = p.y - y;
    xi(2) = p.z - z;
    Eigen::Quaterniond qi = q_xyzw(p.qx, p.qy, p.qz, p.qw);
    if (qi.dot(qmean) < 0.0) {
      qi.coeffs() = -qi.coeffs();
    }
    const Eigen::Quaterniond qerr = (qi * qmean.conjugate()).normalized();
    const Eigen::Vector3d rv = quat_to_rotvec(qerr);
    xi(3) = rv.x();
    xi(4) = rv.y();
    xi(5) = rv.z();
    S.noalias() += p.weight * (xi * xi.transpose());
  }
  S(0, 0) = std::max(S(0, 0), kMinPosVar);
  S(1, 1) = std::max(S(1, 1), kMinPosVar);
  S(2, 2) = std::max(S(2, 2), kMinPosVar);
  S(3, 3) = std::max(S(3, 3), kMinRotVar);
  S(4, 4) = std::max(S(4, 4), kMinRotVar);
  S(5, 5) = std::max(S(5, 5), kMinRotVar);
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      out.covariance[static_cast<size_t>(i) * 6u + static_cast<size_t>(j)] = S(i, j);
    }
  }
  out.position_stddev_m = std::sqrt(std::max(S(0, 0), S(1, 1)));
  out.valid = true;
  return out;
}

std::optional<PoseMap> ConstantTwistMeshPfBackend::localize(
  const LidarFrame & cloud, const OdomDelta & odom)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!map_ || !initialized_ || particles_.empty() || cloud.points.empty()) {
    return std::nullopt;
  }

  const auto t0 = std::chrono::steady_clock::now();
  double dt = 1.0 / std::max(1.0, cfg_.localize_hz);
  if (last_stamp_ns_ > 0 && cloud.timestamp_ns > last_stamp_ns_) {
    dt = static_cast<double>(cloud.timestamp_ns - last_stamp_ns_) * 1e-9;
  }

  motion_update(dt, odom);
  sensor_update(cloud);
  PoseMap mean = mean_pose();
  resample();

  if (cfg_.tracking_motion != "still" && !(tracking_ && odom.valid) &&
      have_mean_ && dt > 1e-4 && dt < 1.0) {
    const double a = std::clamp(cfg_.twist_lpf_alpha, 0.0, 1.0);
    const double dvx = (mean.translation[0] - last_mean_.translation[0]) / dt;
    const double dvy = (mean.translation[1] - last_mean_.translation[1]) / dt;
    const double dvz = (mean.translation[2] - last_mean_.translation[2]) / dt;
    const Eigen::Quaterniond qn = q_xyzw(
      mean.rotation[0], mean.rotation[1], mean.rotation[2], mean.rotation[3]);
    const Eigen::Quaterniond qp = q_xyzw(
      last_mean_.rotation[0], last_mean_.rotation[1],
      last_mean_.rotation[2], last_mean_.rotation[3]);
    Eigen::Quaterniond dq = (qn * qp.conjugate()).normalized();
    if (dq.w() < 0.0) {
      dq.coeffs() = -dq.coeffs();
    }
    const Eigen::Vector3d w = quat_to_rotvec(dq) / dt;
    vx_ = (1.0 - a) * vx_ + a * dvx;
    vy_ = (1.0 - a) * vy_ + a * dvy;
    vz_ = (1.0 - a) * vz_ + a * dvz;
    wx_ = (1.0 - a) * wx_ + a * w.x();
    wy_ = (1.0 - a) * wy_ + a * w.y();
    wz_ = (1.0 - a) * wz_ + a * w.z();
  }

  last_mean_ = mean;
  have_mean_ = true;
  last_stamp_ns_ = cloud.timestamp_ns;
  last_timing_.localize_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();
  return mean;
}

}  // namespace composite_sensing
