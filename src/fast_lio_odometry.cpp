#include "composite_sensing/fast_lio_odometry.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <random>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <so3_math.h>
#include "use-ikfom.hpp"
#include <ikd_Tree.h>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace composite_sensing
{
namespace
{

constexpr double kG = 9.81;
constexpr double kInitTime = 0.1;
constexpr int kMaxIniCount = 10;
constexpr int kNumMatch = 5;
constexpr float kLaserCov = 0.001f;
constexpr float kMoveThreshold = 1.5f;
constexpr int kPoseDof = 23;

using PointType = pcl::PointXYZINormal;
using PointCloud = pcl::PointCloud<PointType>;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

struct ImuSample
{
  double t{0};
  Eigen::Vector3d gyr{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc{Eigen::Vector3d::Zero()};
};

struct ImuPose
{
  double offset_time{0};
  Eigen::Vector3d acc{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyr{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rot{Eigen::Matrix3d::Identity()};
};

bool time_list(const PointType & a, const PointType & b)
{
  return a.curvature < b.curvature;
}

template<typename T>
Eigen::Vector3d v3(const T & v)
{
  return Eigen::Vector3d(v(0), v(1), v(2));
}

float calc_dist(const PointType & a, const PointType & b)
{
  const float dx = a.x - b.x;
  const float dy = a.y - b.y;
  const float dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

template<typename T>
bool esti_plane(Eigen::Matrix<T, 4, 1> & pca, const PointVector & point, T threshold)
{
  Eigen::Matrix<T, kNumMatch, 3> A;
  Eigen::Matrix<T, kNumMatch, 1> b;
  A.setZero();
  b.setOnes();
  b *= -1.0f;
  for (int j = 0; j < kNumMatch; ++j) {
    A(j, 0) = point[j].x;
    A(j, 1) = point[j].y;
    A(j, 2) = point[j].z;
  }
  Eigen::Matrix<T, 3, 1> n = A.colPivHouseholderQr().solve(b);
  const T nn = n.norm();
  pca(0) = n(0) / nn;
  pca(1) = n(1) / nn;
  pca(2) = n(2) / nn;
  pca(3) = 1.0 / nn;
  for (int j = 0; j < kNumMatch; ++j) {
    if (std::fabs(pca(0) * point[j].x + pca(1) * point[j].y + pca(2) * point[j].z + pca(3)) >
      threshold)
    {
      return false;
    }
  }
  return true;
}

struct PendingScan
{
  double beg{0};
  double end{0};
  LidarFrame meta;
  PointCloud::Ptr cloud;
};

class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ImuProcess(const FastLioConfig & cfg)
  : Lidar_T_wrt_IMU(cfg.lidar_T_wrt_imu),
    Lidar_R_wrt_IMU(cfg.lidar_R_wrt_imu),
    cov_acc_scale(cfg.acc_cov, cfg.acc_cov, cfg.acc_cov),
    cov_gyr_scale(cfg.gyr_cov, cfg.gyr_cov, cfg.gyr_cov),
    cov_bias_gyr(cfg.b_gyr_cov, cfg.b_gyr_cov, cfg.b_gyr_cov),
    cov_bias_acc(cfg.b_acc_cov, cfg.b_acc_cov, cfg.b_acc_cov)
  {
    Q = process_noise_cov();
    cov_acc = Eigen::Vector3d(0.1, 0.1, 0.1);
    cov_gyr = Eigen::Vector3d(0.1, 0.1, 0.1);
  }

  bool initializing() const { return imu_need_init_; }

  // false → scan consumed by init, no undistorted cloud.
  bool process(
    const std::vector<ImuSample> & imu,
    const PointCloud::Ptr & raw,
    double beg, double end,
    esekfom::esekf<state_ikfom, 12, input_ikfom> & kf,
    PointCloud & undistort)
  {
    if (imu.empty() || !raw) {
      return false;
    }
    if (imu_need_init_) {
      imu_init(imu, beg, kf);
      last_imu_ = imu.back();
      if (init_iter_num > kMaxIniCount) {
        cov_acc *= std::pow(kG / mean_acc.norm(), 2);
        imu_need_init_ = false;
        cov_acc = cov_acc_scale;
        cov_gyr = cov_gyr_scale;
      }
      return false;
    }
    undistort_pcl(imu, raw, beg, end, kf, undistort);
    return true;
  }

private:
  void imu_init(
    const std::vector<ImuSample> & imu, double beg,
    esekfom::esekf<state_ikfom, 12, input_ikfom> & kf)
  {
    if (b_first_frame_) {
      init_iter_num = 1;
      b_first_frame_ = false;
      mean_acc = imu.front().acc;
      mean_gyr = imu.front().gyr;
      first_lidar_time = beg;
    }
    for (const auto & s : imu) {
      mean_acc += (s.acc - mean_acc) / init_iter_num;
      mean_gyr += (s.gyr - mean_gyr) / init_iter_num;
      cov_acc = cov_acc * (init_iter_num - 1.0) / init_iter_num +
        (s.acc - mean_acc).cwiseProduct(s.acc - mean_acc) *
        (init_iter_num - 1.0) / (init_iter_num * init_iter_num);
      cov_gyr = cov_gyr * (init_iter_num - 1.0) / init_iter_num +
        (s.gyr - mean_gyr).cwiseProduct(s.gyr - mean_gyr) *
        (init_iter_num - 1.0) / (init_iter_num * init_iter_num);
      ++init_iter_num;
    }
    state_ikfom init_state = kf.get_x();
    init_state.grav = S2(-mean_acc / mean_acc.norm() * kG);
    init_state.bg = mean_gyr;
    init_state.offset_T_L_I = Lidar_T_wrt_IMU;
    init_state.offset_R_L_I = Lidar_R_wrt_IMU;
    kf.change_x(init_state);

    auto init_P = kf.get_P();
    init_P.setIdentity();
    init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
    init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
    init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
    init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
    init_P(21, 21) = init_P(22, 22) = 0.00001;
    kf.change_P(init_P);
  }

  void undistort_pcl(
    const std::vector<ImuSample> & imu_in,
    const PointCloud::Ptr & raw,
    double beg, double end,
    esekfom::esekf<state_ikfom, 12, input_ikfom> & kf,
    PointCloud & pcl_out)
  {
    std::vector<ImuSample> imu = imu_in;
    imu.insert(imu.begin(), last_imu_);
    pcl_out = *raw;
    std::sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);

    state_ikfom imu_state = kf.get_x();
    std::vector<ImuPose> poses;
    ImuPose pose0;
    pose0.offset_time = 0.0;
    pose0.acc = acc_s_last;
    pose0.gyr = angvel_last;
    pose0.vel = v3(imu_state.vel);
    pose0.pos = v3(imu_state.pos);
    pose0.rot = imu_state.rot.toRotationMatrix();
    poses.push_back(pose0);

    Eigen::Matrix<double, 12, 12> Qn = Q;
    Qn.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Qn.block<3, 3>(3, 3).diagonal() = cov_acc;
    Qn.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Qn.block<3, 3>(9, 9).diagonal() = cov_bias_acc;

    input_ikfom in;
    for (size_t i = 0; i + 1 < imu.size(); ++i) {
      const ImuSample & head = imu[i];
      const ImuSample & tail = imu[i + 1];
      if (tail.t < last_lidar_end_time_) {
        continue;
      }
      Eigen::Vector3d angvel_avr = 0.5 * (head.gyr + tail.gyr);
      Eigen::Vector3d acc_avr = 0.5 * (head.acc + tail.acc);
      acc_avr *= kG / mean_acc.norm();
      double dt = (head.t < last_lidar_end_time_)
        ? (tail.t - last_lidar_end_time_)
        : (tail.t - head.t);
      in.acc = acc_avr;
      in.gyro = angvel_avr;
      kf.predict(dt, Qn, in);
      imu_state = kf.get_x();
      angvel_last = angvel_avr - v3(imu_state.bg);
      acc_s_last = imu_state.rot.toRotationMatrix() * (acc_avr - v3(imu_state.ba));
      for (int k = 0; k < 3; ++k) {
        acc_s_last[k] += imu_state.grav[k];
      }
      ImuPose pose;
      pose.offset_time = tail.t - beg;
      pose.acc = acc_s_last;
      pose.gyr = angvel_last;
      pose.vel = v3(imu_state.vel);
      pose.pos = v3(imu_state.pos);
      pose.rot = imu_state.rot.toRotationMatrix();
      poses.push_back(pose);
    }

    const double note = end > imu.back().t ? 1.0 : -1.0;
    double dt_end = note * (end - imu.back().t);
    kf.predict(dt_end, Qn, in);
    imu_state = kf.get_x();
    last_imu_ = imu_in.back();
    last_lidar_end_time_ = end;

    if (pcl_out.points.empty() || poses.size() < 2) {
      return;
    }
    auto it_pcl = pcl_out.points.end() - 1;
    for (auto it_kp = poses.end() - 1; it_kp != poses.begin(); --it_kp) {
      const ImuPose & head = *(it_kp - 1);
      const ImuPose & tail = *it_kp;
      const Eigen::Matrix3d & R_imu = head.rot;
      const Eigen::Vector3d & vel_imu = head.vel;
      const Eigen::Vector3d & pos_imu = head.pos;
      const Eigen::Vector3d & acc_imu = tail.acc;
      const Eigen::Vector3d & angvel_avr = tail.gyr;
      for (; it_pcl->curvature / 1000.0 > head.offset_time; --it_pcl) {
        const double dt = it_pcl->curvature / 1000.0 - head.offset_time;
        const Eigen::Matrix3d R_i = R_imu * Exp(angvel_avr, dt);
        const Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        const Eigen::Vector3d T_ei(
          pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
        const Eigen::Vector3d P_compensate =
          imu_state.offset_R_L_I.conjugate() * (
            imu_state.rot.conjugate() * (
              R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) -
            imu_state.offset_T_L_I);
        it_pcl->x = static_cast<float>(P_compensate(0));
        it_pcl->y = static_cast<float>(P_compensate(1));
        it_pcl->z = static_cast<float>(P_compensate(2));
        if (it_pcl == pcl_out.points.begin()) {
          break;
        }
      }
    }
  }

  Eigen::Matrix<double, 12, 12> Q;
  Eigen::Vector3d Lidar_T_wrt_IMU;
  Eigen::Matrix3d Lidar_R_wrt_IMU;
  Eigen::Vector3d mean_acc{0, 0, -1.0};
  Eigen::Vector3d mean_gyr{Eigen::Vector3d::Zero()};
  Eigen::Vector3d cov_acc;
  Eigen::Vector3d cov_gyr;
  Eigen::Vector3d cov_acc_scale;
  Eigen::Vector3d cov_gyr_scale;
  Eigen::Vector3d cov_bias_gyr;
  Eigen::Vector3d cov_bias_acc;
  Eigen::Vector3d angvel_last{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc_s_last{Eigen::Vector3d::Zero()};
  ImuSample last_imu_{};
  double first_lidar_time{0};
  double last_lidar_end_time_{0};
  int init_iter_num{1};
  bool b_first_frame_{true};
  bool imu_need_init_{true};
};

struct LioCore;

LioCore * g_impl = nullptr;

struct LioCore
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit LioCore(const FastLioConfig & cfg)
  : cfg(cfg), imu(cfg), point_rng(cfg.point_noise_seed)
  {
    g_impl = this;
    down_surf.setLeafSize(cfg.filter_size_surf, cfg.filter_size_surf, cfg.filter_size_surf);
    feats_undistort.reset(new PointCloud());
    feats_down_body.reset(new PointCloud());
    feats_down_world.reset(new PointCloud());
    normvec.reset(new PointCloud(100000, 1));
    laser_cloud_ori.reset(new PointCloud(100000, 1));
    corr_norm.reset(new PointCloud(100000, 1));
    double epsi[kPoseDof];
    std::fill(epsi, epsi + kPoseDof, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, &LioCore::h_share, cfg.max_iterations, epsi);
  }

  ~LioCore()
  {
    if (g_impl == this) {
      g_impl = nullptr;
    }
  }

  static void h_share(state_ikfom & s, esekfom::dyn_share_datastruct<double> & ekfom_data)
  {
    if (g_impl) {
      g_impl->h_share_body(s, ekfom_data);
    }
  }

  void h_share_body(state_ikfom & s, esekfom::dyn_share_datastruct<double> & ekfom_data)
  {
    laser_cloud_ori->resize(static_cast<size_t>(feats_down_size));
    corr_norm->resize(static_cast<size_t>(feats_down_size));
    double total_residual = 0.0;

#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; ++i) {
      PointType & point_body = feats_down_body->points[i];
      PointType & point_world = feats_down_world->points[i];
      const Eigen::Vector3d p_body(point_body.x, point_body.y, point_body.z);
      const Eigen::Vector3d p_global(
        s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
      point_world.x = static_cast<float>(p_global(0));
      point_world.y = static_cast<float>(p_global(1));
      point_world.z = static_cast<float>(p_global(2));
      point_world.intensity = point_body.intensity;

      auto & points_near = nearest[static_cast<size_t>(i)];
      if (ekfom_data.converge) {
        std::vector<float> sq_dis(kNumMatch);
        ikdtree.Nearest_Search(point_world, kNumMatch, points_near, sq_dis);
        point_selected[static_cast<size_t>(i)] = !(
          points_near.size() < kNumMatch || sq_dis[kNumMatch - 1] > 5);
      }
      if (!point_selected[static_cast<size_t>(i)]) {
        continue;
      }
      Eigen::Matrix<float, 4, 1> pabcd;
      point_selected[static_cast<size_t>(i)] = false;
      if (esti_plane(pabcd, points_near, 0.1f)) {
        const float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y +
          pabcd(2) * point_world.z + pabcd(3);
        const float sc = 1.f - 0.9f * std::fabs(pd2) / std::sqrt(p_body.norm());
        if (sc > 0.9f) {
          point_selected[static_cast<size_t>(i)] = true;
          normvec->points[i].x = pabcd(0);
          normvec->points[i].y = pabcd(1);
          normvec->points[i].z = pabcd(2);
          normvec->points[i].intensity = pd2;
          res_last[static_cast<size_t>(i)] = std::fabs(pd2);
        }
      }
    }

    effct_feat_num = 0;
    for (int i = 0; i < feats_down_size; ++i) {
      if (!point_selected[static_cast<size_t>(i)]) {
        continue;
      }
      laser_cloud_ori->points[effct_feat_num] = feats_down_body->points[i];
      corr_norm->points[effct_feat_num] = normvec->points[i];
      total_residual += res_last[static_cast<size_t>(i)];
      ++effct_feat_num;
    }
    if (effct_feat_num < 1) {
      ekfom_data.valid = false;
      return;
    }
    ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num, 12);
    ekfom_data.h.resize(effct_feat_num);
    for (int i = 0; i < effct_feat_num; ++i) {
      const PointType & laser_p = laser_cloud_ori->points[i];
      const Eigen::Vector3d point_this_be(laser_p.x, laser_p.y, laser_p.z);
      Eigen::Matrix3d point_be_cross;
      point_be_cross << SKEW_SYM_MATRX(point_this_be);
      const Eigen::Vector3d point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
      Eigen::Matrix3d point_cross;
      point_cross << SKEW_SYM_MATRX(point_this);
      const PointType & norm_p = corr_norm->points[i];
      const Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);
      const Eigen::Vector3d C(s.rot.conjugate() * norm_vec);
      const Eigen::Vector3d A(point_cross * C);
      if (extrinsic_est) {
        const Eigen::Vector3d B(point_be_cross * s.offset_R_L_I.conjugate() * C);
        ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
          A(0), A(1), A(2), B(0), B(1), B(2), C(0), C(1), C(2);
      } else {
        ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
          A(0), A(1), A(2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      }
      ekfom_data.h(i) = -norm_p.intensity;
    }
  }

  void point_body_to_world(const PointType & pi, PointType & po) const
  {
    const Eigen::Vector3d p_body(pi.x, pi.y, pi.z);
    const Eigen::Vector3d p_global(
      state.rot * (state.offset_R_L_I * p_body + state.offset_T_L_I) + state.pos);
    po.x = static_cast<float>(p_global(0));
    po.y = static_cast<float>(p_global(1));
    po.z = static_cast<float>(p_global(2));
    po.intensity = pi.intensity;
  }

  Eigen::Vector3d pos_lid() const
  {
    return Eigen::Vector3d(state.pos) + state.rot * Eigen::Vector3d(state.offset_T_L_I);
  }

  void fov_segment()
  {
    const Eigen::Vector3d pos = pos_lid();
    if (!localmap_inited) {
      for (int i = 0; i < 3; ++i) {
        local_map.vertex_min[i] = static_cast<float>(pos(i) - cfg.cube_len / 2.0);
        local_map.vertex_max[i] = static_cast<float>(pos(i) + cfg.cube_len / 2.0);
      }
      localmap_inited = true;
      return;
    }
    float dist_to_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; ++i) {
      dist_to_edge[i][0] = std::fabs(static_cast<float>(pos(i)) - local_map.vertex_min[i]);
      dist_to_edge[i][1] = std::fabs(static_cast<float>(pos(i)) - local_map.vertex_max[i]);
      if (dist_to_edge[i][0] <= kMoveThreshold * cfg.det_range ||
        dist_to_edge[i][1] <= kMoveThreshold * cfg.det_range)
      {
        need_move = true;
      }
    }
    if (!need_move) {
      return;
    }
    BoxPointType updated = local_map;
    std::vector<BoxPointType> need_rm;
    const float mov_dist = std::max(
      static_cast<float>((cfg.cube_len - 2.0 * kMoveThreshold * cfg.det_range) * 0.5 * 0.9),
      static_cast<float>(cfg.det_range * (kMoveThreshold - 1)));
    for (int i = 0; i < 3; ++i) {
      BoxPointType tmp = local_map;
      if (dist_to_edge[i][0] <= kMoveThreshold * cfg.det_range) {
        updated.vertex_max[i] -= mov_dist;
        updated.vertex_min[i] -= mov_dist;
        tmp.vertex_min[i] = local_map.vertex_max[i] - mov_dist;
        need_rm.push_back(tmp);
      } else if (dist_to_edge[i][1] <= kMoveThreshold * cfg.det_range) {
        updated.vertex_max[i] += mov_dist;
        updated.vertex_min[i] += mov_dist;
        tmp.vertex_max[i] = local_map.vertex_min[i] + mov_dist;
        need_rm.push_back(tmp);
      }
    }
    local_map = updated;
    if (!need_rm.empty()) {
      ikdtree.Delete_Point_Boxes(need_rm);
    }
  }

  void map_incremental()
  {
    const bool ekf_inited = (lidar_beg - first_lidar_time) >= kInitTime;
    PointVector to_add;
    PointVector no_down;
    to_add.reserve(static_cast<size_t>(feats_down_size));
    no_down.reserve(static_cast<size_t>(feats_down_size));
    for (int i = 0; i < feats_down_size; ++i) {
      point_body_to_world(feats_down_body->points[i], feats_down_world->points[i]);
      if (!nearest[static_cast<size_t>(i)].empty() && ekf_inited) {
        const PointVector & points_near = nearest[static_cast<size_t>(i)];
        bool need_add = true;
        PointType mid;
        mid.x = std::floor(feats_down_world->points[i].x / cfg.filter_size_map) *
          cfg.filter_size_map + 0.5 * cfg.filter_size_map;
        mid.y = std::floor(feats_down_world->points[i].y / cfg.filter_size_map) *
          cfg.filter_size_map + 0.5 * cfg.filter_size_map;
        mid.z = std::floor(feats_down_world->points[i].z / cfg.filter_size_map) *
          cfg.filter_size_map + 0.5 * cfg.filter_size_map;
        const float dist = calc_dist(feats_down_world->points[i], mid);
        if (std::fabs(points_near[0].x - mid.x) > 0.5 * cfg.filter_size_map &&
          std::fabs(points_near[0].y - mid.y) > 0.5 * cfg.filter_size_map &&
          std::fabs(points_near[0].z - mid.z) > 0.5 * cfg.filter_size_map)
        {
          no_down.push_back(feats_down_world->points[i]);
          continue;
        }
        for (int j = 0; j < kNumMatch; ++j) {
          if (static_cast<int>(points_near.size()) < kNumMatch) {
            break;
          }
          if (calc_dist(points_near[j], mid) < dist) {
            need_add = false;
            break;
          }
        }
        if (need_add) {
          to_add.push_back(feats_down_world->points[i]);
        }
      } else {
        to_add.push_back(feats_down_world->points[i]);
      }
    }
    ikdtree.Add_Points(to_add, true);
    ikdtree.Add_Points(no_down, false);
  }

  FastLioOdom make_odom(double end) const
  {
    FastLioOdom odom;
    odom.stamp_ns = static_cast<int64_t>(std::llround(end * 1e9));
    odom.position = v3(state.pos);
    odom.orientation = Eigen::Quaterniond(state.rot.toRotationMatrix()).normalized();
    odom.vel_body = odom.orientation.conjugate() * v3(state.vel);
    lio_pose_twist_cov(
      kf.get_P(), odom.orientation.toRotationMatrix(),
      odom.pose_cov, odom.twist_cov);
    odom.valid = true;
    return odom;
  }

  LidarFrame make_cloud(const PendingScan & scan) const
  {
    LidarFrame out = scan.meta;
    out.points.clear();
    out.points.reserve(feats_undistort->size());
    const double span = scan.end - scan.beg;
    out.timestamp_ns = static_cast<int64_t>(std::llround(scan.end * 1e9));
    for (const auto & pt : feats_undistort->points) {
      LidarPoint p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      p.intensity = pt.intensity;
      p.time_offset_s = pt.curvature / 1000.f - static_cast<float>(span);
      out.points.push_back(p);
    }
    return out;
  }

  bool take_imu(double end, std::vector<ImuSample> & out)
  {
    if (imu_buf.empty() || imu_buf.back().t < end) {
      return false;
    }
    if (imu_buf.front().t > end) {
      return false;
    }
    double imu_time = imu_buf.front().t;
    out.clear();
    while (!imu_buf.empty() && imu_time < end) {
      imu_time = imu_buf.front().t;
      if (imu_time > end) {
        break;
      }
      out.push_back(imu_buf.front());
      imu_buf.pop_front();
    }
    return !out.empty();
  }

  // One finished scan, or nothing if the front scan is still waiting / initializing.
  bool process_front(FastLioScan & finished, bool & produced)
  {
    produced = false;
    if (pending.empty()) {
      return false;
    }
    std::vector<ImuSample> meas_imu;
    if (!take_imu(pending.front().end, meas_imu)) {
      if (!imu_buf.empty() && imu_buf.back().t >= pending.front().end &&
        imu_buf.front().t > pending.front().end)
      {
        pending.pop_front();
        return true;
      }
      return false;
    }
    PendingScan scan = std::move(pending.front());
    pending.pop_front();
    lidar_beg = scan.beg;

    if (first_scan) {
      first_scan = false;
      first_lidar_time = scan.beg;
      return true;
    }

    const bool have_cloud = imu.process(
      meas_imu, scan.cloud, scan.beg, scan.end, kf, *feats_undistort);
    state = kf.get_x();
    if (!have_cloud || feats_undistort->empty()) {
      return true;
    }

    fov_segment();
    down_surf.setInputCloud(feats_undistort);
    down_surf.filter(*feats_down_body);
    feats_down_size = static_cast<int>(feats_down_body->size());

    if (ikdtree.Root_Node == nullptr) {
      if (feats_down_size > 5) {
        ikdtree.set_downsample_param(cfg.filter_size_map);
        feats_down_world->resize(feats_down_size);
        for (int i = 0; i < feats_down_size; ++i) {
          point_body_to_world(feats_down_body->points[i], feats_down_world->points[i]);
        }
        ikdtree.Build(feats_down_world->points);
      }
      finished.cloud = make_cloud(scan);
      finished.odom = make_odom(scan.end);
      produced = true;
      return true;
    }
    if (feats_down_size < 5) {
      finished.cloud = make_cloud(scan);
      finished.odom = make_odom(scan.end);
      produced = true;
      return true;
    }

    normvec->resize(feats_down_size);
    feats_down_world->resize(feats_down_size);
    nearest.resize(static_cast<size_t>(feats_down_size));
    double solve_H_time = 0;
    kf.update_iterated_dyn_share_modified(kLaserCov, solve_H_time);
    state = kf.get_x();
    map_incremental();

    finished.cloud = make_cloud(scan);
    finished.odom = make_odom(scan.end);
    produced = true;
    return true;
  }

  std::vector<FastLioScan> drain()
  {
    std::vector<FastLioScan> out;
    while (true) {
      FastLioScan scan;
      bool produced = false;
      if (!process_front(scan, produced)) {
        break;
      }
      if (produced) {
        out.push_back(std::move(scan));
      }
    }
    return out;
  }

  FastLioConfig cfg;
  ImuProcess imu;
  std::mt19937 point_rng;
  esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
  state_ikfom state;
  KD_TREE<PointType> ikdtree;
  pcl::VoxelGrid<PointType> down_surf;
  PointCloud::Ptr feats_undistort;
  PointCloud::Ptr feats_down_body;
  PointCloud::Ptr feats_down_world;
  PointCloud::Ptr normvec;
  PointCloud::Ptr laser_cloud_ori;
  PointCloud::Ptr corr_norm;
  std::vector<PointVector> nearest;
  std::vector<char> point_selected = std::vector<char>(100000, 1);
  std::vector<float> res_last = std::vector<float>(100000, -1000.f);
  std::deque<ImuSample> imu_buf;
  std::deque<PendingScan> pending;
  BoxPointType local_map{};
  bool localmap_inited{false};
  bool first_scan{true};
  bool extrinsic_est{false};
  int feats_down_size{0};
  int effct_feat_num{0};
  double first_lidar_time{0};
  double lidar_beg{0};
};

}  // namespace

struct FastLioOdometry::Impl : LioCore
{
  using LioCore::LioCore;
};

FastLioOdometry::FastLioOdometry(const FastLioConfig & config)
: impl_(std::make_unique<Impl>(config))
{
}

FastLioOdometry::~FastLioOdometry() = default;

std::vector<FastLioScan> FastLioOdometry::push_imu(const std::vector<LidarImuSample> & samples)
{
  for (const auto & s : samples) {
    if (s.timestamp_ns <= 0) {
      continue;
    }
    ImuSample sample;
    sample.t = static_cast<double>(s.timestamp_ns) * 1e-9;
    sample.gyr = Eigen::Vector3d(s.gx, s.gy, s.gz);
    sample.acc = Eigen::Vector3d(s.ax, s.ay, s.az);
    if (!impl_->imu_buf.empty() && sample.t < impl_->imu_buf.back().t) {
      continue;
    }
    impl_->imu_buf.push_back(sample);
  }
  return impl_->drain();
}

std::vector<FastLioScan> FastLioOdometry::push_cloud(const LidarFrame & raw)
{
  if (raw.points.empty() || raw.timestamp_ns <= 0) {
    return {};
  }
  PendingScan scan;
  scan.beg = static_cast<double>(raw.timestamp_ns) * 1e-9;
  scan.meta = raw;
  scan.meta.points.clear();
  scan.cloud.reset(new PointCloud());
  scan.cloud->reserve(raw.points.size());
  double max_off = 0.0;
  for (const auto & p : raw.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const double off = std::isfinite(p.time_offset_s) ? p.time_offset_s : 0.0;
    max_off = std::max(max_off, off);
    PointType pt;
    pt.x = p.x;
    pt.y = p.y;
    pt.z = p.z;
    add_lidar_point_noise(
      pt.x, pt.y, pt.z, impl_->point_rng, impl_->cfg.point_noise_std_m);
    pt.intensity = p.intensity;
    pt.curvature = static_cast<float>(off * 1000.0);
    scan.cloud->push_back(pt);
  }
  if (scan.cloud->empty()) {
    return {};
  }
  scan.end = scan.beg + max_off;
  scan.meta.frame_id = raw.frame_id;
  impl_->pending.push_back(std::move(scan));
  return impl_->drain();
}

bool FastLioOdometry::initialized() const
{
  return !impl_->imu.initializing();
}

size_t FastLioOdometry::pending_scans() const
{
  return impl_->pending.size();
}

}  // namespace composite_sensing
