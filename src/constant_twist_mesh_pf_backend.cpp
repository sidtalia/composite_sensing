#include "composite_sensing/constant_twist_mesh_pf_backend.hpp"

#include "composite_sensing/mesh_raycaster.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace composite_sensing
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

void quat_to_yaw(const std::array<double, 4> & q_xyzw, double & yaw)
{
  const double x = q_xyzw[0], y = q_xyzw[1], z = q_xyzw[2], w = q_xyzw[3];
  yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

void yaw_to_quat(double yaw, std::array<double, 4> & q_xyzw)
{
  q_xyzw[0] = 0.0;
  q_xyzw[1] = 0.0;
  q_xyzw[2] = std::sin(yaw * 0.5);
  q_xyzw[3] = std::cos(yaw * 0.5);
}

void rotate_yaw(double yaw, double x, double y, double & xo, double & yo)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  xo = c * x - s * y;
  yo = s * x + c * y;
}

}  // namespace

ConstantTwistMeshPfBackend::ConstantTwistMeshPfBackend() = default;

ConstantTwistMeshPfBackend::~ConstantTwistMeshPfBackend() = default;

void ConstantTwistMeshPfBackend::configure(const MeshLocalizerConfig & config)
{
  std::lock_guard<std::mutex> lock(mu_);
  cfg_ = config;
  if (cfg_.map_file.empty()) {
    throw std::runtime_error("ConstantTwistMeshPfBackend: map_file required");
  }
  map_ = create_mesh_raycaster(cfg_.map_file, cfg_.raycast_backend, &raycast_backend_used_);
  particles_.assign(static_cast<size_t>(std::max(1, cfg_.num_particles)), {});
  const size_t n_rays = particles_.size() *
    static_cast<size_t>(std::max(1, cfg_.beam_samples));
  rays_.assign(n_rays, {});
  hits_.assign(n_rays, {});
  meas_.assign(n_rays, 0.0f);
  have_mean_ = false;
  last_stamp_ns_ = 0;
  vx_ = vy_ = vz_ = wyaw_ = 0.0;
  if (cfg_.global_init_on_start) {
    std::uniform_real_distribution<double> ux(cfg_.init_bb[0], cfg_.init_bb[3]);
    std::uniform_real_distribution<double> uy(cfg_.init_bb[1], cfg_.init_bb[4]);
    std::uniform_real_distribution<double> uz(cfg_.init_bb[2], cfg_.init_bb[5]);
    std::uniform_real_distribution<double> uyaw(cfg_.init_yaw[0], cfg_.init_yaw[1]);
    const double w = 1.0 / static_cast<double>(particles_.size());
    for (auto & p : particles_) {
      p.x = ux(rng_);
      p.y = uy(rng_);
      p.z = uz(rng_);
      p.yaw = uyaw(rng_);
      p.weight = w;
    }
  }
}

void ConstantTwistMeshPfBackend::global_init()
{
  std::lock_guard<std::mutex> lock(mu_);
  if (particles_.empty()) {
    particles_.assign(static_cast<size_t>(std::max(1, cfg_.num_particles)), {});
  }
  std::uniform_real_distribution<double> ux(cfg_.init_bb[0], cfg_.init_bb[3]);
  std::uniform_real_distribution<double> uy(cfg_.init_bb[1], cfg_.init_bb[4]);
  std::uniform_real_distribution<double> uz(cfg_.init_bb[2], cfg_.init_bb[5]);
  std::uniform_real_distribution<double> uyaw(cfg_.init_yaw[0], cfg_.init_yaw[1]);
  const double w = 1.0 / static_cast<double>(particles_.size());
  for (auto & p : particles_) {
    p.x = ux(rng_);
    p.y = uy(rng_);
    p.z = uz(rng_);
    p.yaw = uyaw(rng_);
    p.weight = w;
  }
  have_mean_ = false;
  vx_ = vy_ = vz_ = wyaw_ = 0.0;
}

void ConstantTwistMeshPfBackend::motion_update(double dt)
{
  if (dt <= 0.0 || dt > 1.0) {
    dt = 1.0 / std::max(1.0, cfg_.localize_hz);
  }
  const double dist = std::hypot(vx_ * dt, vy_ * dt) + std::abs(vz_ * dt);
  const double dyaw = std::abs(wyaw_ * dt);
  const double sig_t = cfg_.still_trans_noise_m + cfg_.dist_noise_gain * dist;
  const double sig_y = cfg_.still_yaw_noise_rad + cfg_.yaw_noise_gain * dyaw;
  std::normal_distribution<double> nt(0.0, sig_t);
  std::normal_distribution<double> ny(0.0, sig_y);

  for (auto & p : particles_) {
    p.x += vx_ * dt + nt(rng_);
    p.y += vy_ * dt + nt(rng_);
    p.z += vz_ * dt + 0.25 * nt(rng_);
    p.yaw += wyaw_ * dt + ny(rng_);
    while (p.yaw > kPi) {p.yaw -= 2.0 * kPi;}
    while (p.yaw < -kPi) {p.yaw += 2.0 * kPi;}
  }
}

void ConstantTwistMeshPfBackend::fill_ray(
  double bx, double by, double bz, double byaw,
  float px, float py, float pz,
  Ray & ray, float & meas) const
{
  const double lx = px, ly = py, lz = pz;
  const double qx = cfg_.q_base_lidar[0];
  const double qy = cfg_.q_base_lidar[1];
  const double qz = cfg_.q_base_lidar[2];
  const double qw = cfg_.q_base_lidar[3];
  const double tx = 2.0 * (qy * lz - qz * ly);
  const double ty = 2.0 * (qz * lx - qx * lz);
  const double tz = 2.0 * (qx * ly - qy * lx);
  const double pbx = lx + qw * tx + (qy * tz - qz * ty) + cfg_.t_base_lidar[0];
  const double pby = ly + qw * ty + (qz * tx - qx * tz) + cfg_.t_base_lidar[1];
  const double pbz = lz + qw * tz + (qx * ty - qy * tx) + cfg_.t_base_lidar[2];

  const double ox_b = cfg_.t_base_lidar[0];
  const double oy_b = cfg_.t_base_lidar[1];
  const double oz_b = cfg_.t_base_lidar[2];

  double ox_m, oy_m, px_m, py_m;
  rotate_yaw(byaw, ox_b, oy_b, ox_m, oy_m);
  rotate_yaw(byaw, pbx, pby, px_m, py_m);
  ox_m += bx;
  oy_m += by;
  const double oz_m = oz_b + bz;
  px_m += bx;
  py_m += by;
  const double pz_m = pbz + bz;

  double dx = px_m - ox_m;
  double dy = py_m - oy_m;
  double dz = pz_m - oz_m;
  const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
  meas = static_cast<float>(range);
  if (range >= 1e-3) {
    dx /= range;
    dy /= range;
    dz /= range;
  }
  ray.ox = static_cast<float>(ox_m);
  ray.oy = static_cast<float>(oy_m);
  ray.oz = static_cast<float>(oz_m);
  ray.tmin = 0.0f;
  ray.dx = static_cast<float>(dx);
  ray.dy = static_cast<float>(dy);
  ray.dz = static_cast<float>(dz);
  ray.tmax = static_cast<float>(cfg_.max_ray_range_m);
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

  const size_t n_part = particles_.size();
  const size_t n_rays = n_part * n_samp;
  if (rays_.size() < n_rays) {
    rays_.resize(n_rays);
    hits_.resize(n_rays);
    meas_.resize(n_rays);
  }

  const auto t_fill0 = Clock::now();
  size_t k = 0;
  for (const auto & p : particles_) {
    for (size_t i : idx) {
      const auto & pt = cloud.points[i];
      fill_ray(p.x, p.y, p.z, p.yaw, pt.x, pt.y, pt.z, rays_[k], meas_[k]);
      ++k;
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
  double x = 0, y = 0, z = 0, cs = 0, sn = 0;
  for (const auto & p : particles_) {
    x += p.weight * p.x;
    y += p.weight * p.y;
    z += p.weight * p.z;
    cs += p.weight * std::cos(p.yaw);
    sn += p.weight * std::sin(p.yaw);
  }
  out.translation = {x, y, z};
  yaw_to_quat(std::atan2(sn, cs), out.rotation);
  // Rough spread → stddev hint for EKF
  double var = 0.0;
  for (const auto & p : particles_) {
    const double dx = p.x - x;
    const double dy = p.y - y;
    var += p.weight * (dx * dx + dy * dy);
  }
  out.position_stddev_m = std::max(0.05, std::sqrt(var));
  out.valid = true;
  return out;
}

std::optional<PoseMap> ConstantTwistMeshPfBackend::localize(const LidarFrame & cloud)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!map_ || particles_.empty() || cloud.points.empty()) {
    return std::nullopt;
  }

  const auto t0 = std::chrono::steady_clock::now();
  double dt = 1.0 / std::max(1.0, cfg_.localize_hz);
  if (last_stamp_ns_ > 0 && cloud.timestamp_ns > last_stamp_ns_) {
    dt = static_cast<double>(cloud.timestamp_ns - last_stamp_ns_) * 1e-9;
  }

  motion_update(dt);
  sensor_update(cloud);
  resample();

  PoseMap mean = mean_pose();
  if (have_mean_ && dt > 1e-4 && dt < 1.0) {
    const double a = std::clamp(cfg_.twist_lpf_alpha, 0.0, 1.0);
    const double dvx = (mean.translation[0] - last_mean_.translation[0]) / dt;
    const double dvy = (mean.translation[1] - last_mean_.translation[1]) / dt;
    const double dvz = (mean.translation[2] - last_mean_.translation[2]) / dt;
    double yaw_now = 0, yaw_prev = 0;
    quat_to_yaw(mean.rotation, yaw_now);
    quat_to_yaw(last_mean_.rotation, yaw_prev);
    double dyaw = yaw_now - yaw_prev;
    while (dyaw > kPi) {dyaw -= 2.0 * kPi;}
    while (dyaw < -kPi) {dyaw += 2.0 * kPi;}
    vx_ = (1.0 - a) * vx_ + a * dvx;
    vy_ = (1.0 - a) * vy_ + a * dvy;
    vz_ = (1.0 - a) * vz_ + a * dvz;
    wyaw_ = (1.0 - a) * wyaw_ + a * (dyaw / dt);
  }

  last_mean_ = mean;
  have_mean_ = true;
  last_stamp_ns_ = cloud.timestamp_ns;
  last_timing_.localize_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();
  return mean;
}

}  // namespace composite_sensing
