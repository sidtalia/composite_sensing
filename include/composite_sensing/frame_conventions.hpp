#pragma once

#include <array>
#include <cmath>

#include "cuvslam/cuvslam2.h"

namespace composite_sensing {

// cuVSLAM / OpenCV optical: X right, Y down, Z forward.
// ROS REP-103 FLU (camera_link): X forward, Y left, Z up.
inline constexpr std::array<double, 9> kRFluFromOptical = {
  0.0, 0.0, 1.0,
  -1.0, 0.0, 0.0,
  0.0, -1.0, 0.0,
};

inline constexpr std::array<double, 9> kROpticalFromFlu = {
  0.0, -1.0, 0.0,
  0.0, 0.0, -1.0,
  1.0, 0.0, 0.0,
};

inline void mat_mul_3x3(
  const std::array<double, 9> & a, const std::array<double, 9> & b,
  std::array<double, 9> & out)
{
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out[r * 3 + c] =
        a[r * 3 + 0] * b[0 * 3 + c] +
        a[r * 3 + 1] * b[1 * 3 + c] +
        a[r * 3 + 2] * b[2 * 3 + c];
    }
  }
}

inline void mat_vec_3(
  const std::array<double, 9> & m, const std::array<double, 3> & v,
  std::array<double, 3> & out)
{
  out[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
  out[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
  out[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}

/** Row-major rotation → quaternion xyzw. */
inline void rotation_matrix_to_quat_xyzw(
  const std::array<double, 9> & R, std::array<double, 4> & q)
{
  const double tr = R[0] + R[4] + R[8];
  if (tr > 0.0) {
    const double s = std::sqrt(tr + 1.0) * 2.0;
    q[3] = 0.25 * s;
    q[0] = (R[7] - R[5]) / s;
    q[1] = (R[2] - R[6]) / s;
    q[2] = (R[3] - R[1]) / s;
  } else if (R[0] > R[4] && R[0] > R[8]) {
    const double s = std::sqrt(1.0 + R[0] - R[4] - R[8]) * 2.0;
    q[3] = (R[7] - R[5]) / s;
    q[0] = 0.25 * s;
    q[1] = (R[1] + R[3]) / s;
    q[2] = (R[2] + R[6]) / s;
  } else if (R[4] > R[8]) {
    const double s = std::sqrt(1.0 + R[4] - R[0] - R[8]) * 2.0;
    q[3] = (R[2] - R[6]) / s;
    q[0] = (R[1] + R[3]) / s;
    q[1] = 0.25 * s;
    q[2] = (R[5] + R[7]) / s;
  } else {
    const double s = std::sqrt(1.0 + R[8] - R[0] - R[4]) * 2.0;
    q[3] = (R[3] - R[1]) / s;
    q[0] = (R[2] + R[6]) / s;
    q[1] = (R[5] + R[7]) / s;
    q[2] = 0.25 * s;
  }
}

inline void quat_xyzw_to_rotation_matrix(
  const std::array<double, 4> & q, std::array<double, 9> & R)
{
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  const double xx = x * x, yy = y * y, zz = z * z;
  const double xy = x * y, xz = x * z, yz = y * z;
  const double wx = w * x, wy = w * y, wz = w * z;
  R[0] = 1.0 - 2.0 * (yy + zz);
  R[1] = 2.0 * (xy - wz);
  R[2] = 2.0 * (xz + wy);
  R[3] = 2.0 * (xy + wz);
  R[4] = 1.0 - 2.0 * (xx + zz);
  R[5] = 2.0 * (yz - wx);
  R[6] = 2.0 * (xz - wy);
  R[7] = 2.0 * (yz + wx);
  R[8] = 1.0 - 2.0 * (xx + yy);
}

/**
 * Convert cuVSLAM world←optical pose to ROS odom←camera_link (FLU).
 * Same transform as Python optical_pose_to_ros_flu().
 */
inline void optical_pose_to_ros_flu(
  const cuvslam::Pose & pose,
  std::array<double, 3> & t_flu,
  std::array<double, 4> & q_flu_xyzw)
{
  const std::array<double, 4> q_opt = {
    pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3]};
  std::array<double, 9> R_wo{};
  quat_xyzw_to_rotation_matrix(q_opt, R_wo);

  std::array<double, 9> tmp{}, R_wl{};
  mat_mul_3x3(kRFluFromOptical, R_wo, tmp);
  mat_mul_3x3(tmp, kROpticalFromFlu, R_wl);

  const std::array<double, 3> t_opt = {
    pose.translation[0], pose.translation[1], pose.translation[2]};
  mat_vec_3(kRFluFromOptical, t_opt, t_flu);
  rotation_matrix_to_quat_xyzw(R_wl, q_flu_xyzw);
}

/** Fixed TF rotation: FLU body frame → optical frame (same origin). */
inline void flu_to_optical_quat(std::array<double, 4> & q_xyzw)
{
  // Matches R_optical_from_flu as child←parent for TF (child axes in parent).
  // Optical axes in FLU: x=-Y, y=-Z, z=+X → quat (-0.5, 0.5, -0.5, 0.5).
  q_xyzw = {-0.5, 0.5, -0.5, 0.5};
}

/** Intrinsic XYZ RPY (deg) → row-major R, matching tf2 Quaternion.setRPY. */
inline void rpy_deg_to_R(
  double roll_deg, double pitch_deg, double yaw_deg, std::array<double, 9> & R)
{
  constexpr double kDeg = 0.017453292519943295;
  const double r = roll_deg * kDeg, p = pitch_deg * kDeg, y = yaw_deg * kDeg;
  const double cr = std::cos(r), sr = std::sin(r);
  const double cp = std::cos(p), sp = std::sin(p);
  const double cy = std::cos(y), sy = std::sin(y);
  R[0] = cy * cp;
  R[1] = cy * sp * sr - sy * cr;
  R[2] = cy * sp * cr + sy * sr;
  R[3] = sy * cp;
  R[4] = sy * sp * sr + cy * cr;
  R[5] = sy * sp * cr - cy * sr;
  R[6] = -sp;
  R[7] = cp * sr;
  R[8] = cp * cr;
}

/**
 * T_world_body = T_world_sensor * inv(T_body_sensor).
 * ``R_bs`` / ``t_bs`` are the static mount (body → sensor), same as SSoT xyz/rpy.
 */
inline void world_sensor_to_world_body(
  const std::array<double, 3> & t_ws,
  const std::array<double, 4> & q_ws,
  const std::array<double, 9> & R_bs,
  const std::array<double, 3> & t_bs,
  std::array<double, 3> & t_wb,
  std::array<double, 4> & q_wb)
{
  const std::array<double, 9> R_sb = {
    R_bs[0], R_bs[3], R_bs[6],
    R_bs[1], R_bs[4], R_bs[7],
    R_bs[2], R_bs[5], R_bs[8],
  };
  std::array<double, 3> t_sb{};
  mat_vec_3(R_sb, t_bs, t_sb);
  t_sb[0] = -t_sb[0];
  t_sb[1] = -t_sb[1];
  t_sb[2] = -t_sb[2];

  std::array<double, 9> R_ws{};
  quat_xyzw_to_rotation_matrix(q_ws, R_ws);
  std::array<double, 9> R_wb{};
  mat_mul_3x3(R_ws, R_sb, R_wb);
  std::array<double, 3> delta{};
  mat_vec_3(R_ws, t_sb, delta);
  t_wb[0] = t_ws[0] + delta[0];
  t_wb[1] = t_ws[1] + delta[1];
  t_wb[2] = t_ws[2] + delta[2];
  rotation_matrix_to_quat_xyzw(R_wb, q_wb);
}

/** 6×6 row-major multiply: C = A * B. */
inline void mat_mul_6x6(
  const std::array<double, 36> & a, const std::array<double, 36> & b,
  std::array<double, 36> & c)
{
  for (int r = 0; r < 6; ++r) {
    for (int col = 0; col < 6; ++col) {
      double s = 0.0;
      for (int k = 0; k < 6; ++k) {
        s += a[r * 6 + k] * b[k * 6 + col];
      }
      c[r * 6 + col] = s;
    }
  }
}

/** Adjoint of (R, t) on ROS se(3) [vx,vy,vz, ωx,ωy,ωz]: cov := Ad cov Adᵀ. */
inline void adjoint_cov6_ros(
  const std::array<double, 9> & R,
  const std::array<double, 3> & t,
  std::array<double, 36> & cov)
{
  std::array<double, 9> skewR{};
  // skew(t) * R
  const double tx = t[0], ty = t[1], tz = t[2];
  const std::array<double, 9> skew = {
    0.0, -tz, ty,
    tz, 0.0, -tx,
    -ty, tx, 0.0};
  mat_mul_3x3(skew, R, skewR);

  std::array<double, 36> ad{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      ad[r * 6 + c] = R[r * 3 + c];
      ad[(r + 3) * 6 + (c + 3)] = R[r * 3 + c];
      ad[r * 6 + (c + 3)] = skewR[r * 3 + c];
    }
  }
  std::array<double, 36> ad_t{};
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      ad_t[r * 6 + c] = ad[c * 6 + r];
    }
  }
  std::array<double, 36> tmp{};
  mat_mul_6x6(ad, cov, tmp);
  mat_mul_6x6(tmp, ad_t, cov);
}

/**
 * cuVSLAM pose covariance is row-major 6×6 in
 * [rx, ry, rz, x, y, z] in the optical frame.
 * ROS Odometry is [x, y, z, rx, ry, rz] in the published FLU frame.
 */
inline void cuvslam_cov_to_ros_flu(const float * cov_cuv, std::array<double, 36> & cov_ros)
{
  auto cuv_idx = [](int ros_i) { return ros_i < 3 ? ros_i + 3 : ros_i - 3; };
  std::array<double, 36> reordered{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      reordered[i * 6 + j] = static_cast<double>(
        cov_cuv[cuv_idx(i) * 6 + cuv_idx(j)]);
    }
  }
  const std::array<double, 3> zero{0.0, 0.0, 0.0};
  adjoint_cov6_ros(kRFluFromOptical, zero, reordered);
  cov_ros = reordered;
}

/** VO cov → ROS Odometry pose.covariance in the published FLU (optionally body) frame. */
inline void fill_odom_pose_covariance(
  const float * cov_cuv,
  bool apply_body_mount,
  const std::array<double, 9> & R_body_sensor,
  const std::array<double, 3> & t_body_sensor,
  std::array<double, 36> & out)
{
  cuvslam_cov_to_ros_flu(cov_cuv, out);
  if (apply_body_mount) {
    adjoint_cov6_ros(R_body_sensor, t_body_sensor, out);
  }
}

}  // namespace composite_sensing
