#include <array>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include <Eigen/Geometry>

#include "composite_sensing/fast_lio_odometry.hpp"
#include "composite_sensing/lio_slam.hpp"

static int check_align()
{
  Eigen::Isometry3d T_base_imu = Eigen::Isometry3d::Identity();
  T_base_imu.translation() = Eigen::Vector3d(0.1, 0.02, 0.05);
  T_base_imu.linear() =
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Isometry3d T_odom_base = Eigen::Isometry3d::Identity();
  T_odom_base.translation() = Eigen::Vector3d(1.0, 2.0, 0.0);
  T_odom_base.linear() =
    Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Isometry3d T_lio = Eigen::Isometry3d::Identity();
  T_lio.translation() = Eigen::Vector3d(4.0, -1.0, 0.2);
  const Eigen::Isometry3d latch =
    composite_sensing::latch_odom_from_lio(T_odom_base, T_base_imu, T_lio);
  const Eigen::Isometry3d back =
    composite_sensing::lio_base_in_odom(latch, T_lio, T_base_imu);
  if ((back.translation() - T_odom_base.translation()).norm() > 1e-9) {
    std::fprintf(stderr, "fast-lio align: latch does not reproduce ekf pose\n");
    return 1;
  }
  Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
  delta.translation() = Eigen::Vector3d(0.5, 0.0, 0.0);
  const Eigen::Isometry3d T_odom_base2 = T_odom_base * delta;
  const Eigen::Isometry3d T_lio2 = latch.inverse() * T_odom_base2 * T_base_imu;
  const Eigen::Isometry3d got =
    composite_sensing::lio_base_in_odom(latch, T_lio2, T_base_imu);
  if ((got.translation() - T_odom_base2.translation()).norm() > 1e-9 ||
    (got.linear() - T_odom_base2.linear()).norm() > 1e-9)
  {
    std::fprintf(stderr, "fast-lio align: motion is not rigid in odom\n");
    return 1;
  }
  return 0;
}

static int check_cov()
{
  Eigen::Matrix<double, 23, 23> P = Eigen::Matrix<double, 23, 23>::Zero();
  P.block<3, 3>(0, 0).diagonal().setConstant(2.0);
  P.block<3, 3>(3, 3) = Eigen::Vector3d(3.0, 0.0, 0.0).asDiagonal();
  P(0, 3) = 0.5;
  P(3, 0) = 0.5;
  P.block<3, 3>(12, 12) = Eigen::Vector3d(4.0, 0.0, 0.0).asDiagonal();
  const Eigen::Matrix3d R =
    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  std::array<double, 36> pose{};
  std::array<double, 36> twist{};
  composite_sensing::lio_pose_twist_cov(P, R, pose, twist);
  const bool ok =
    std::fabs(pose[0] - 2.0) < 1e-9 &&
    std::fabs(pose[28] - 3.0) < 1e-9 &&
    std::fabs(pose[4] - 0.5) < 1e-9 &&
    std::fabs(twist[7] - 4.0) < 1e-9 &&
    std::fabs(twist[0]) < 1e-9 &&
    std::fabs(twist[35] - 1e3) < 1e-9;
  if (!ok) {
    std::fprintf(stderr, "fast-lio cov: pose/twist blocks are in the wrong frame\n");
    return 1;
  }
  return 0;
}

static int check_point_noise()
{
  std::mt19937 quiet(1);
  float x = 1.f;
  float y = 2.f;
  float z = 3.f;
  composite_sensing::add_lidar_point_noise(x, y, z, quiet, 0.f);
  if (x != 1.f || y != 2.f || z != 3.f) {
    std::fprintf(stderr, "fast-lio noise: sigma 0 changed a point\n");
    return 1;
  }
  std::mt19937 a(7);
  std::mt19937 b(7);
  float x2 = 1.f;
  float y2 = 2.f;
  float z2 = 3.f;
  composite_sensing::add_lidar_point_noise(x, y, z, a, 0.05f);
  composite_sensing::add_lidar_point_noise(x2, y2, z2, b, 0.05f);
  if (x != x2 || y != y2 || z != z2 || (x == 1.f && y == 2.f && z == 3.f)) {
    std::fprintf(stderr, "fast-lio noise: seed did not repeat, or noise was zero\n");
    return 1;
  }
  return 0;
}

static int check_scan_context()
{
  std::vector<std::array<float, 3>> a;
  a.reserve(120);
  for (int s = 0; s < 60; ++s) {
    const float ang = static_cast<float>(
      -M_PI + (static_cast<double>(s) + 0.5) * (2.0 * M_PI / 60.0));
    const float c = std::cos(ang);
    const float sn = std::sin(ang);
    // Two rings, sector-dependent shape. Cosine of a column ignores a
    // single shared scale, so the ratio across rings has to change.
    a.push_back({10.f * c, 10.f * sn, 0.2f * static_cast<float>(s + 1)});
    a.push_back({30.f * c, 30.f * sn, 0.2f * static_cast<float>(60 - s)});
  }
  const float c30 = std::cos(static_cast<float>(M_PI / 6.0));
  const float s30 = std::sin(static_cast<float>(M_PI / 6.0));
  std::vector<std::array<float, 3>> b;
  b.reserve(a.size());
  for (const auto & p : a) {
    b.push_back({c30 * p[0] - s30 * p[1], s30 * p[0] + c30 * p[1], p[2]});
  }
  const auto ca = composite_sensing::make_scan_context(a);
  const auto cb = composite_sensing::make_scan_context(b);
  int shift = -1;
  const float d = composite_sensing::scan_context_distance(ca, cb, shift);
  int shift0 = -1;
  const float d0 = composite_sensing::scan_context_distance(ca, ca, shift0);
  if (d0 > 0.05f || shift0 != 0 || d > 0.05f || std::abs(shift - 5) > 1) {
    std::fprintf(
      stderr, "fast-lio slam: scan context shift=%d dist=%.3f identical dist=%.3f\n",
      shift, d, d0);
    return 1;
  }
  return 0;
}

// Stationary IMU + a floor of points. Fails if the deskew/odom pipe never returns.
int main()
{
  if (const int rc = check_align()) {
    return rc;
  }
  if (const int rc = check_cov()) {
    return rc;
  }
  if (const int rc = check_point_noise()) {
    return rc;
  }
  if (const int rc = check_scan_context()) {
    return rc;
  }
  composite_sensing::FastLioConfig cfg;
  composite_sensing::FastLioOdometry lio(cfg);
  bool got = false;
  for (int scan = 0; scan < 6; ++scan) {
    const int64_t beg = static_cast<int64_t>(scan) * 100000000LL;
    std::vector<composite_sensing::LidarImuSample> imu;
    imu.reserve(20);
    for (int i = 0; i < 20; ++i) {
      composite_sensing::LidarImuSample s;
      s.timestamp_ns = beg + static_cast<int64_t>(i) * 5000000LL;
      s.az = -9.81f;
      imu.push_back(s);
    }
    lio.push_imu(imu);

    composite_sensing::LidarFrame cloud;
    cloud.timestamp_ns = beg;
    cloud.frame_id = "livox_frame";
    for (int x = 0; x <= 8; ++x) {
      for (int y = 0; y <= 8; ++y) {
        composite_sensing::LidarPoint p;
        p.x = 0.4f * static_cast<float>(x);
        p.y = 0.4f * static_cast<float>(y);
        p.time_offset_s = 0.009f * static_cast<float>((x + y) % 10);
        cloud.points.push_back(p);
      }
    }
    for (const auto & out : lio.push_cloud(cloud)) {
      if (!out.odom.valid || out.cloud.points.empty()) {
        continue;
      }
      if (!std::isfinite(out.odom.position.x()) ||
        !std::isfinite(out.cloud.points.front().x))
      {
        std::fprintf(stderr, "fast-lio self-check: non-finite output\n");
        return 1;
      }
      got = true;
    }
  }
  if (!got) {
    std::fprintf(stderr, "fast-lio self-check: no deskewed scan\n");
    return 1;
  }
  return 0;
}
