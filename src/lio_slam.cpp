#include "composite_sensing/lio_slam.hpp"

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

namespace composite_sensing
{
namespace
{

constexpr int kRings = ScanContext::kRings;
constexpr int kSectors = ScanContext::kSectors;
constexpr float kMaxRange = 80.f;
constexpr double kKeyframeGapM = 1.0;
constexpr int kExcludeRecent = 30;
constexpr float kLoopDist = 0.2f;
constexpr double kIcpFitness = 0.3;
constexpr int kQueueCap = 32;

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

gtsam::Pose3 to_pose3(const Eigen::Isometry3d & T)
{
  return gtsam::Pose3(
    gtsam::Rot3(T.linear()),
    gtsam::Point3(T.translation()));
}

Eigen::Isometry3d from_pose3(const gtsam::Pose3 & p)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = p.rotation().matrix();
  T.translation() = Eigen::Vector3d(p.x(), p.y(), p.z());
  return T;
}

gtsam::noiseModel::Diagonal::shared_ptr sigmas(double rx, double ry, double rz,
  double x, double y, double z)
{
  return gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << rx, ry, rz, x, y, z).finished());
}

Cloud::Ptr voxel(const LidarFrame & cloud)
{
  Cloud::Ptr raw(new Cloud());
  raw->reserve(cloud.points.size());
  for (const auto & p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    raw->push_back(pcl::PointXYZ(p.x, p.y, p.z));
  }
  Cloud::Ptr down(new Cloud());
  if (raw->empty()) {
    return down;
  }
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(0.5f, 0.5f, 0.5f);
  vg.setInputCloud(raw);
  vg.filter(*down);
  return down;
}

float column_cosine(const ScanContext & a, const ScanContext & b, int sa, int sb)
{
  double dot = 0.0;
  double na = 0.0;
  double nb = 0.0;
  int n = 0;
  for (int r = 0; r < kRings; ++r) {
    const float va = a.cell[static_cast<size_t>(r * kSectors + sa)];
    const float vb = b.cell[static_cast<size_t>(r * kSectors + sb)];
    if (!std::isfinite(va) || !std::isfinite(vb)) {
      continue;
    }
    dot += static_cast<double>(va) * vb;
    na += static_cast<double>(va) * va;
    nb += static_cast<double>(vb) * vb;
    ++n;
  }
  if (n == 0 || na < 1e-6 || nb < 1e-6) {
    return -1.f;
  }
  return static_cast<float>(dot / std::sqrt(na * nb));
}

bool icp_relative(
  const Cloud::Ptr & src, const Cloud::Ptr & tgt,
  const Eigen::Isometry3d & guess, Eigen::Isometry3d & out)
{
  if (!src || !tgt || src->size() < 50 || tgt->size() < 50) {
    return false;
  }
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(src);
  icp.setInputTarget(tgt);
  icp.setMaxCorrespondenceDistance(2.0);
  icp.setMaximumIterations(30);
  icp.setTransformationEpsilon(1e-4);
  Cloud aligned;
  icp.align(aligned, guess.matrix().cast<float>());
  if (!icp.hasConverged() || icp.getFitnessScore() > kIcpFitness) {
    return false;
  }
  out = Eigen::Isometry3d::Identity();
  out.matrix() = icp.getFinalTransformation().cast<double>();
  return true;
}

struct Keyframe
{
  int64_t stamp_ns{0};
  Eigen::Isometry3d T_lio{Eigen::Isometry3d::Identity()};
  Cloud::Ptr cloud;
  ScanContext sc{};
  bool has_sc{false};
};

struct Job
{
  LidarFrame cloud;
  FastLioOdom odom;
};

}  // namespace

ScanContext make_scan_context(const std::vector<std::array<float, 3>> & pts)
{
  ScanContext sc;
  sc.cell.fill(std::numeric_limits<float>::quiet_NaN());
  for (const auto & p : pts) {
    const float r = std::hypot(p[0], p[1]);
    if (!(r > 0.5f && r < kMaxRange) || !std::isfinite(p[2])) {
      continue;
    }
    const int ring = std::min(
      kRings - 1, static_cast<int>(r / kMaxRange * static_cast<float>(kRings)));
    const double ang = std::atan2(static_cast<double>(p[1]), static_cast<double>(p[0]));
    int sector = static_cast<int>(
      (ang + M_PI) / (2.0 * M_PI) * static_cast<double>(kSectors));
    if (sector < 0) {
      sector = 0;
    }
    if (sector >= kSectors) {
      sector = kSectors - 1;
    }
    float & cell = sc.cell[static_cast<size_t>(ring * kSectors + sector)];
    if (!std::isfinite(cell) || p[2] > cell) {
      cell = p[2];
    }
  }
  return sc;
}

float scan_context_distance(const ScanContext & a, const ScanContext & b, int & best_shift)
{
  float best = 2.f;
  best_shift = 0;
  for (int shift = 0; shift < kSectors; ++shift) {
    double sum = 0.0;
    int n = 0;
    for (int s = 0; s < kSectors; ++s) {
      const float c = column_cosine(a, b, s, (s + shift) % kSectors);
      if (c < 0.f) {
        continue;
      }
      sum += 1.0 - static_cast<double>(c);
      ++n;
    }
    if (n < 4) {
      continue;
    }
    const float dist = static_cast<float>(sum / static_cast<double>(n));
    if (dist < best) {
      best = dist;
      best_shift = shift;
    }
  }
  return best;
}

struct LioSlam::Impl
{
  Impl(const Eigen::Isometry3d & T_imu_lidar, Callback cb)
  : T_imu_lidar(T_imu_lidar), cb(std::move(cb))
  {
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.1;
    params.relinearizeSkip = 1;
    isam = gtsam::ISAM2(params);
    odom_noise = sigmas(0.01, 0.01, 0.02, 0.05, 0.05, 0.05);
    loop_noise = sigmas(0.05, 0.05, 0.05, 0.2, 0.2, 0.2);
    prior_noise = sigmas(1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3);
    thread = std::thread([this] { run(); });
  }

  ~Impl()
  {
    {
      std::lock_guard<std::mutex> lk(mu);
      stop = true;
    }
    cv.notify_one();
    if (thread.joinable()) {
      thread.join();
    }
  }

  void push(const LidarFrame & cloud, const FastLioOdom & odom)
  {
    if (!odom.valid || cloud.points.empty()) {
      return;
    }
    std::lock_guard<std::mutex> lk(mu);
    if (have_last && (odom.position - last_p).norm() < kKeyframeGapM) {
      return;
    }
    if (static_cast<int>(queue.size()) >= kQueueCap) {
      return;
    }
    last_p = odom.position;
    have_last = true;
    queue.push_back(Job{cloud, odom});
    cv.notify_one();
  }

  void run()
  {
    while (true) {
      Job job;
      {
        std::unique_lock<std::mutex> lk(mu);
        cv.wait(lk, [this] { return stop || !queue.empty(); });
        if (stop && queue.empty()) {
          return;
        }
        job = std::move(queue.front());
        queue.pop_front();
      }
      try {
        add_keyframe(std::move(job));
      } catch (const std::exception &) {
      }
    }
  }

  void add_keyframe(Job job)
  {
    Keyframe kf;
    kf.stamp_ns = job.odom.stamp_ns;
    kf.T_lio = iso_pose(job.odom.position, job.odom.orientation);
    kf.cloud = voxel(job.cloud);
    if (kf.cloud && kf.cloud->size() >= 50) {
      std::vector<std::array<float, 3>> pts;
      pts.reserve(kf.cloud->size());
      for (const auto & p : kf.cloud->points) {
        pts.push_back({p.x, p.y, p.z});
      }
      kf.sc = make_scan_context(pts);
      kf.has_sc = true;
    }

    const int j = static_cast<int>(kfs.size());
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values init;
    const gtsam::Pose3 pose_j = to_pose3(kf.T_lio);
    init.insert(gtsam::Symbol('x', j), pose_j);
    if (j == 0) {
      graph.addPrior(gtsam::Symbol('x', 0), pose_j, prior_noise);
    } else {
      const gtsam::Pose3 meas = to_pose3(kfs.back().T_lio.inverse() * kf.T_lio);
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        gtsam::Symbol('x', j - 1), gtsam::Symbol('x', j), meas, odom_noise));
    }
    kfs.push_back(std::move(kf));

    if (kfs.back().has_sc && j > kExcludeRecent) {
      int best_i = -1;
      float best_d = kLoopDist;
      for (int i = 0; i < j - kExcludeRecent; ++i) {
        if (!kfs[static_cast<size_t>(i)].has_sc) {
          continue;
        }
        int shift = 0;
        const float d = scan_context_distance(
          kfs[static_cast<size_t>(i)].sc, kfs.back().sc, shift);
        (void)shift;
        if (d < best_d) {
          best_d = d;
          best_i = i;
        }
      }
      if (best_i >= 0) {
        const Eigen::Isometry3d T_l_i = kfs[static_cast<size_t>(best_i)].T_lio * T_imu_lidar;
        const Eigen::Isometry3d T_l_j = kfs.back().T_lio * T_imu_lidar;
        const Eigen::Isometry3d guess = T_l_i.inverse() * T_l_j;
        Eigen::Isometry3d T_icp;
        if (icp_relative(
            kfs.back().cloud, kfs[static_cast<size_t>(best_i)].cloud, guess, T_icp))
        {
          const Eigen::Isometry3d T_rel =
            T_imu_lidar * T_icp * T_imu_lidar.inverse();
          graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            gtsam::Symbol('x', best_i), gtsam::Symbol('x', j),
            to_pose3(T_rel), loop_noise));
        }
      }
    }

    isam.update(graph, init);
    const gtsam::Values est = isam.calculateEstimate();
    std::vector<SlamPose> poses;
    poses.reserve(kfs.size());
    for (int i = 0; i <= j; ++i) {
      SlamPose sp;
      sp.stamp_ns = kfs[static_cast<size_t>(i)].stamp_ns;
      if (est.exists(gtsam::Symbol('x', i))) {
        sp.T_world_imu = from_pose3(est.at<gtsam::Pose3>(gtsam::Symbol('x', i)));
      } else {
        sp.T_world_imu = kfs[static_cast<size_t>(i)].T_lio;
      }
      poses.push_back(sp);
    }
    if (cb) {
      cb(poses);
    }
  }

  Eigen::Isometry3d T_imu_lidar{Eigen::Isometry3d::Identity()};
  Callback cb;
  std::mutex mu;
  std::condition_variable cv;
  std::deque<Job> queue;
  bool stop{false};
  bool have_last{false};
  Eigen::Vector3d last_p{Eigen::Vector3d::Zero()};
  std::thread thread;
  std::vector<Keyframe> kfs;
  gtsam::ISAM2 isam;
  gtsam::noiseModel::Diagonal::shared_ptr odom_noise;
  gtsam::noiseModel::Diagonal::shared_ptr loop_noise;
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise;
};

LioSlam::LioSlam(const Eigen::Isometry3d & T_imu_lidar, Callback cb)
: impl_(std::make_unique<Impl>(T_imu_lidar, std::move(cb)))
{
}

LioSlam::~LioSlam() = default;

void LioSlam::push(const LidarFrame & cloud, const FastLioOdom & odom)
{
  impl_->push(cloud, odom);
}

}  // namespace composite_sensing
