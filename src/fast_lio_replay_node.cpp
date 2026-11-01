// Standalone FAST-LIO over a rosbag. Does not open the lidar.
//
//   ros2 run composite_sensing fast_lio_replay --ros-args -p use_sim_time:=true
//   ros2 bag play <bag> --clock
//
// Publishes nav_msgs/Odometry on /lidar_inertial/odom:
//   header.frame_id = odom, child_frame_id = lio_odom
// Pose is base_link in odom, latched to /ekf/odometry on the first LIO pose.

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "composite_sensing/cloud_deskew.hpp"
#include "composite_sensing/fast_lio_odometry.hpp"
#include "composite_sensing/lio_slam.hpp"

namespace
{

using composite_sensing::FastLioConfig;
using composite_sensing::FastLioOdom;
using composite_sensing::FastLioOdometry;
using composite_sensing::FastLioScan;
using composite_sensing::LidarFrame;
using composite_sensing::LidarImuSample;
using composite_sensing::StampedOdomPose;

Eigen::Isometry3d xyz_rpy_deg(
  const std::vector<double> & xyz, const std::vector<double> & rpy_deg)
{
  const double d2r = M_PI / 180.0;
  const Eigen::Quaterniond q =
    Eigen::AngleAxisd(rpy_deg[2] * d2r, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy_deg[1] * d2r, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy_deg[0] * d2r, Eigen::Vector3d::UnitX());
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = q.toRotationMatrix();
  T.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
  return T;
}

bool has_field(const sensor_msgs::msg::PointCloud2 & msg, const char * name)
{
  for (const auto & f : msg.fields) {
    if (f.name == name) {
      return true;
    }
  }
  return false;
}

bool pose_at(
  const std::deque<StampedOdomPose> & poses, int64_t stamp_ns,
  Eigen::Isometry3d & out)
{
  if (poses.empty() || stamp_ns < poses.front().stamp_ns ||
    stamp_ns > poses.back().stamp_ns)
  {
    return false;
  }
  auto it = std::lower_bound(
    poses.begin(), poses.end(), stamp_ns,
    [](const StampedOdomPose & p, int64_t t) { return p.stamp_ns < t; });
  if (it == poses.begin() || it->stamp_ns == stamp_ns) {
    out = it->T_odom_base;
    return true;
  }
  const StampedOdomPose & b = *it;
  const StampedOdomPose & a = *std::prev(it);
  if (b.stamp_ns <= a.stamp_ns) {
    out = b.T_odom_base;
    return true;
  }
  const double alpha =
    static_cast<double>(stamp_ns - a.stamp_ns) /
    static_cast<double>(b.stamp_ns - a.stamp_ns);
  const Eigen::Quaterniond q0(a.T_odom_base.rotation());
  const Eigen::Quaterniond q1(b.T_odom_base.rotation());
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = q0.slerp(alpha, q1).normalized().toRotationMatrix();
  T.translation() =
    (1.0 - alpha) * a.T_odom_base.translation() + alpha * b.T_odom_base.translation();
  out = T;
  return true;
}

class FastLioReplay : public rclcpp::Node
{
public:
  FastLioReplay()
  : Node("fast_lio_replay")
  {
    const auto cloud_topic = declare_parameter<std::string>("cloud_topic", "/livox/cloud");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/livox/imu");
    const auto ekf_topic = declare_parameter<std::string>("ekf_topic", "/ekf/odometry");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/lidar_inertial/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    child_frame_ = declare_parameter<std::string>("child_frame", "lio_odom");
    // Published /livox/cloud moves the header to scan end and leaves `time`
    // as the offset from scan start. "start" means header is already scan start.
    cloud_stamp_ = declare_parameter<std::string>("cloud_stamp", "end");
    const auto base_xyz = declare_parameter<std::vector<double>>(
      "base_xyz", {0.105645, 0.022548, 0.105696});
    const auto base_rpy = declare_parameter<std::vector<double>>(
      "base_rpy_deg", {179.958, -13.248, 0.944});
    const auto imu_xyz = declare_parameter<std::vector<double>>(
      "imu_xyz", {0.011, 0.02329, -0.04412});
    const auto imu_rpy = declare_parameter<std::vector<double>>(
      "imu_rpy_deg", {0.0, 0.0, 0.0});
    if (base_xyz.size() != 3 || base_rpy.size() != 3 ||
      imu_xyz.size() != 3 || imu_rpy.size() != 3)
    {
      throw std::runtime_error("base/imu xyz and rpy need 3 values");
    }

    const Eigen::Isometry3d T_base_lidar = xyz_rpy_deg(base_xyz, base_rpy);
    const Eigen::Isometry3d T_lidar_imu = xyz_rpy_deg(imu_xyz, imu_rpy);
    T_base_imu_ = T_base_lidar * T_lidar_imu;
    const Eigen::Isometry3d T_imu_lidar = T_lidar_imu.inverse();

    FastLioConfig cfg;
    cfg.lidar_T_wrt_imu = T_imu_lidar.translation();
    cfg.lidar_R_wrt_imu = T_imu_lidar.linear();
    cfg.point_noise_std_m = static_cast<float>(
      declare_parameter<double>("point_noise_std_m", 0.0));
    cfg.point_noise_seed = static_cast<uint32_t>(
      declare_parameter<int>("point_noise_seed", 1));
    lio_ = std::make_unique<FastLioOdometry>(cfg);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      declare_parameter<std::string>("slam_path_topic", "/lidar_inertial/slam_path"),
      10);
    slam_ = std::make_unique<composite_sensing::LioSlam>(
      T_imu_lidar,
      [this](const std::vector<composite_sensing::SlamPose> & poses) {
        std::lock_guard<std::mutex> lk(slam_mu_);
        slam_poses_ = poses;
      });
    if (cfg.point_noise_std_m > 0.f) {
      RCLCPP_WARN(
        get_logger(), "lidar point noise on: N(0, %.4f m), seed %u",
        cfg.point_noise_std_m, cfg.point_noise_seed);
    }

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::QoS(rclcpp::KeepLast(20)).best_effort(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { on_cloud(msg); });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::QoS(rclcpp::KeepLast(2000)).best_effort(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { on_imu(msg); });
    ekf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      ekf_topic, rclcpp::QoS(rclcpp::KeepLast(4000)).best_effort(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_ekf(msg); });
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    RCLCPP_INFO(
      get_logger(),
      "fast-lio replay: cloud=%s imu=%s ekf=%s → %s (%s → %s)",
      cloud_topic.c_str(), imu_topic.c_str(), ekf_topic.c_str(),
      odom_topic_.c_str(), odom_frame_.c_str(), child_frame_.c_str());
  }

private:
  void on_ekf(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
  {
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    StampedOdomPose s;
    s.stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    s.T_odom_base = composite_sensing::iso_pose(
      Eigen::Vector3d(p.x, p.y, p.z),
      Eigen::Quaterniond(q.w, q.x, q.y, q.z));
    ekf_.push_back(s);
    const int64_t keep_ns = 10000000000LL;
    while (ekf_.size() > 2 && ekf_.back().stamp_ns - ekf_.front().stamp_ns > keep_ns) {
      ekf_.pop_front();
    }
    flush();
  }

  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
  {
    LidarImuSample s;
    s.timestamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    s.gx = static_cast<float>(msg->angular_velocity.x);
    s.gy = static_cast<float>(msg->angular_velocity.y);
    s.gz = static_cast<float>(msg->angular_velocity.z);
    s.ax = static_cast<float>(msg->linear_acceleration.x);
    s.ay = static_cast<float>(msg->linear_acceleration.y);
    s.az = static_cast<float>(msg->linear_acceleration.z);
    take(lio_->push_imu({s}));
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    const bool time_ok = has_field(*msg, "time");
    if (!time_ok && !time_warned_) {
      RCLCPP_WARN(
        get_logger(),
        "%s has no time field; FAST-LIO will treat the scan as instantaneous",
        msg->header.frame_id.c_str());
      time_warned_ = true;
    }
    if (!has_field(*msg, "x")) {
      return;
    }
    LidarFrame cloud;
    cloud.timestamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    cloud.frame_id = msg->header.frame_id;
    cloud.points.reserve(static_cast<size_t>(msg->width) * msg->height);
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");
    const bool intensity = has_field(*msg, "intensity");
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> ii;
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> it;
    if (intensity) {
      ii = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*msg, "intensity");
    }
    if (time_ok) {
      it = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*msg, "time");
    }
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      composite_sensing::LidarPoint p;
      p.x = *ix;
      p.y = *iy;
      p.z = *iz;
      if (ii) {
        p.intensity = **ii;
        ++(*ii);
      }
      if (it) {
        p.time_offset_s = **it;
        ++(*it);
      }
      cloud.points.push_back(p);
    }
    if (cloud_stamp_ == "end") {
      double max_off = 0.0;
      for (const auto & p : cloud.points) {
        if (std::isfinite(p.time_offset_s)) {
          max_off = std::max(max_off, static_cast<double>(p.time_offset_s));
        }
      }
      const int64_t shift = static_cast<int64_t>(std::llround(max_off * 1e9));
      if (shift > 0 && cloud.timestamp_ns > shift) {
        cloud.timestamp_ns -= shift;
      }
    }
    take(lio_->push_cloud(cloud));
  }

  void take(const std::vector<FastLioScan> & scans)
  {
    for (const auto & scan : scans) {
      if (!scan.odom.valid) {
        continue;
      }
      if (slam_) {
        slam_->push(scan.cloud, scan.odom);
      }
      held_.push_back(scan.odom);
    }
    if (!latched_ && held_.size() > 500) {
      held_.erase(held_.begin(), held_.begin() + 250);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "no /ekf/odometry covering the first LIO pose yet");
    }
    flush();
  }

  void flush()
  {
    if (held_.empty()) {
      return;
    }
    if (!latched_) {
      Eigen::Isometry3d T_ekf;
      if (!pose_at(ekf_, held_.front().stamp_ns, T_ekf)) {
        return;
      }
      const Eigen::Isometry3d T_lio = composite_sensing::iso_pose(
        held_.front().position, held_.front().orientation);
      T_odom_lio_ = composite_sensing::latch_odom_from_lio(
        T_ekf, T_base_imu_, T_lio);
      latched_ = true;
      RCLCPP_INFO(
        get_logger(), "latched LIO into %s, publishing %s → %s",
        odom_frame_.c_str(), odom_frame_.c_str(), child_frame_.c_str());
    }
    for (const auto & odom : held_) {
      publish(odom);
    }
    held_.clear();
  }

  void publish(const FastLioOdom & odom)
  {
    const Eigen::Isometry3d T_lio =
      composite_sensing::iso_pose(odom.position, odom.orientation);
    const Eigen::Isometry3d T =
      composite_sensing::lio_base_in_odom(T_odom_lio_, T_lio, T_base_imu_);
    const Eigen::Quaterniond q(T.rotation());
    const Eigen::Vector3d v_base = T_base_imu_.linear() * odom.vel_body;
    // ponytail: ignores the IMU-to-base lever arm in the pose covariance.
    // Upgrade is the adjoint of T_base_imu.inverse().
    auto pose_cov = odom.pose_cov;
    auto twist_cov = odom.twist_cov;
    composite_sensing::rotate_cov6(pose_cov, T_odom_lio_.linear());
    composite_sensing::rotate_cov6(twist_cov, T_base_imu_.linear());

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = rclcpp::Time(odom.stamp_ns);
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = child_frame_;
    msg.pose.pose.position.x = T.translation().x();
    msg.pose.pose.position.y = T.translation().y();
    msg.pose.pose.position.z = T.translation().z();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();
    msg.twist.twist.linear.x = v_base.x();
    msg.twist.twist.linear.y = v_base.y();
    msg.twist.twist.linear.z = v_base.z();
    msg.pose.covariance = pose_cov;
    msg.twist.covariance = twist_cov;
    odom_pub_->publish(msg);
    publish_slam();
  }

  void publish_slam()
  {
    std::vector<composite_sensing::SlamPose> poses;
    {
      std::lock_guard<std::mutex> lk(slam_mu_);
      poses = slam_poses_;
    }
    if (!latched_ || poses.empty() || !path_pub_) {
      return;
    }
    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Time(poses.back().stamp_ns);
    path.header.frame_id = odom_frame_;
    path.poses.reserve(poses.size());
    for (const auto & sp : poses) {
      const Eigen::Isometry3d T = composite_sensing::lio_base_in_odom(
        T_odom_lio_, sp.T_world_imu, T_base_imu_);
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      const Eigen::Quaterniond q(T.rotation());
      ps.pose.position.x = T.translation().x();
      ps.pose.position.y = T.translation().y();
      ps.pose.position.z = T.translation().z();
      ps.pose.orientation.x = q.x();
      ps.pose.orientation.y = q.y();
      ps.pose.orientation.z = q.z();
      ps.pose.orientation.w = q.w();
      path.poses.push_back(ps);
    }
    path_pub_->publish(path);
    if (!slam_announced_) {
      RCLCPP_INFO(
        get_logger(), "fast-lio slam path on %s (%zu keyframes, frame %s)",
        path_pub_->get_topic_name(), poses.size(), odom_frame_.c_str());
      slam_announced_ = true;
    }
  }

  std::string odom_topic_;
  std::string odom_frame_;
  std::string child_frame_;
  std::string cloud_stamp_;
  Eigen::Isometry3d T_base_imu_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d T_odom_lio_{Eigen::Isometry3d::Identity()};
  bool latched_{false};
  bool time_warned_{false};
  std::deque<StampedOdomPose> ekf_;
  std::vector<FastLioOdom> held_;
  std::unique_ptr<FastLioOdometry> lio_;
  std::mutex slam_mu_;
  std::vector<composite_sensing::SlamPose> slam_poses_;
  bool slam_announced_{false};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::unique_ptr<composite_sensing::LioSlam> slam_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastLioReplay>());
  rclcpp::shutdown();
  return 0;
}
