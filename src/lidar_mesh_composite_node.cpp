#include "composite_sensing/lidar_mesh_composite_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "composite_sensing/unilidar_lidar_device.hpp"
#if defined(HOUND_HAVE_LIVOX_SDK) && HOUND_HAVE_LIVOX_SDK
#include "composite_sensing/livox_lidar_device.hpp"
#endif

namespace composite_sensing
{

namespace
{

void rpy_deg_to_quat(
  double roll_deg, double pitch_deg, double yaw_deg,
  double & qx, double & qy, double & qz, double & qw)
{
  tf2::Quaternion q;
  q.setRPY(
    roll_deg * M_PI / 180.0,
    pitch_deg * M_PI / 180.0,
    yaw_deg * M_PI / 180.0);
  qx = q.x();
  qy = q.y();
  qz = q.z();
  qw = q.w();
}

DeskewReference parse_deskew_reference(const std::string & s)
{
  if (s == "start" || s == "Start" || s == "START") {
    return DeskewReference::Start;
  }
  return DeskewReference::End;
}

}  // namespace

LidarMeshCompositeNode::LidarMeshCompositeNode(const rclcpp::NodeOptions & options)
: Node("lidar_mesh_composite", options)
{
  declare_params();
  start_pipeline();
}

LidarMeshCompositeNode::~LidarMeshCompositeNode()
{
  running_.store(false);
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  if (lidar_) {
    lidar_->stop();
  }
}

void LidarMeshCompositeNode::declare_params()
{
  lidar_backend_ = declare_parameter<std::string>("lidar_backend", "unitree");
  port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
  lidar_ip_ = declare_parameter<std::string>("lidar_ip", "");
  local_ip_ = declare_parameter<std::string>("local_ip", "");
  lidar_port_ = static_cast<uint16_t>(declare_parameter<int>("lidar_port", 0));
  local_port_ = static_cast<uint16_t>(declare_parameter<int>("local_port", 0));
  config_path_ = declare_parameter<std::string>("config_path", "");
  frame_time_ms_ = declare_parameter<int>("frame_time_ms", 100);
  lidar_frame_ = declare_parameter<std::string>("lidar_frame", "unilidar_lidar");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  parent_frame_ = declare_parameter<std::string>("parent_frame", "base_link");
  t_x_ = declare_parameter<double>("xyz.x", 0.0);
  t_y_ = declare_parameter<double>("xyz.y", 0.0);
  t_z_ = declare_parameter<double>("xyz.z", 0.1);
  {
    const double r = declare_parameter<double>("rpy.roll", 180.0);
    const double p = declare_parameter<double>("rpy.pitch", -15.0);
    const double y = declare_parameter<double>("rpy.yaw", 0.0);
    rpy_deg_to_quat(r, p, y, q_x_, q_y_, q_z_, q_w_);
  }
  if (declare_parameter<bool>("use_quat_extrinsic", false)) {
    q_x_ = declare_parameter<double>("quat.x", 0.0);
    q_y_ = declare_parameter<double>("quat.y", 0.0);
    q_z_ = declare_parameter<double>("quat.z", 0.0);
    q_w_ = declare_parameter<double>("quat.w", 1.0);
  }
  range_min_ = static_cast<float>(declare_parameter<double>("range_min", 0.4));
  range_max_ = static_cast<float>(declare_parameter<double>("range_max", 50.0));
  cloud_scan_num_ = declare_parameter<int>("cloud_scan_num", 18);
  publish_cloud_ = declare_parameter<bool>("publish_cloud", true);
  cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/unilidar/cloud");
  enable_imu_ = declare_parameter<bool>("enable_imu", false);
  imu_frame_ = declare_parameter<std::string>("imu_frame", "livox_imu");
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/livox/imu");
  imu_x_ = declare_parameter<double>("imu_xyz.x", 0.011);
  imu_y_ = declare_parameter<double>("imu_xyz.y", 0.02329);
  imu_z_ = declare_parameter<double>("imu_xyz.z", -0.04412);
  {
    const double r = declare_parameter<double>("imu_rpy.roll", 0.0);
    const double p = declare_parameter<double>("imu_rpy.pitch", 0.0);
    const double y = declare_parameter<double>("imu_rpy.yaw", 0.0);
    rpy_deg_to_quat(r, p, y, imu_qx_, imu_qy_, imu_qz_, imu_qw_);
  }
  publish_raw_cloud_ = declare_parameter<bool>("publish_raw_cloud", false);
  raw_cloud_topic_ = declare_parameter<std::string>("raw_cloud_topic", "/livox/cloud_raw");
  core_temp_topic_ = declare_parameter<std::string>(
    "core_temp_topic", "/livox/core_temp");
  environment_temp_topic_ = declare_parameter<std::string>(
    "environment_temp_topic", "/livox/environment_temp");
  temp_query_period_s_ = declare_parameter<double>("temp_query_period_s", 2.0);
  core_temp_warn_c_ = declare_parameter<double>("core_temp_warn_c", 55.0);
  cloud_stale_warn_s_ = declare_parameter<double>("cloud_stale_warn_s", 5.0);
  status_log_period_s_ = declare_parameter<double>("status_log_period_s", 60.0);

  deskew_enable_ = declare_parameter<bool>("deskew.enable", true);
  deskew_motion_frame_ = declare_parameter<std::string>("deskew.motion_frame", "odom");
  // Legacy alias: target_frame used to mean motion parent (and wrongly output frame).
  {
    const std::string legacy = declare_parameter<std::string>(
      "deskew.target_frame", "");
    if (!legacy.empty() && legacy != lidar_frame_) {
      deskew_motion_frame_ = legacy;
      RCLCPP_WARN(
        get_logger(),
        "deskew.target_frame is deprecated; use deskew.motion_frame (got %s)",
        legacy.c_str());
    }
  }
  deskew_reference_ = parse_deskew_reference(
    declare_parameter<std::string>("deskew.reference", "end"));
  deskew_odom_topic_ = declare_parameter<std::string>(
    "deskew.odom_topic", "/ekf/odometry");
  deskew_warn_ms_ = declare_parameter<double>("deskew.warn_ms", 40.0);
  deskew_pose_buffer_s_ = declare_parameter<double>("deskew.pose_buffer_s", 1.0);
  {
    const double wire_ms = std::max(
      0.0, declare_parameter<double>("deskew.wire_latency_ms", 20.0));
    deskew_wire_latency_ns_ =
      static_cast<int64_t>(std::llround(wire_ms * 1e6));
  }
  deskew_backend_ = declare_parameter<std::string>("deskew.backend", "odom");
  fast_lio_odom_topic_ = declare_parameter<std::string>(
    "fast_lio.odom_topic", "/lidar_inertial/odom");
  fast_lio_odom_frame_ = declare_parameter<std::string>(
    "fast_lio.odom_frame", "lidar_inertial");

  // Assume base_link → lidar is fixed (rigid mount). T_base_lidar comes from
  // SSoT lidar xyz/rpy, not from TF. Do not look this up per scan.
  T_base_lidar_ = Eigen::Isometry3d::Identity();
  T_base_lidar_.translation() = Eigen::Vector3d(t_x_, t_y_, t_z_);
  T_base_lidar_.linear() =
    Eigen::Quaterniond(q_w_, q_x_, q_y_, q_z_).normalized().toRotationMatrix();
}

void LidarMeshCompositeNode::start_pipeline()
{
  if (lidar_backend_ == "unitree") {
    lidar_ = std::make_unique<UnilidarLidarDevice>();
  } else if (lidar_backend_ == "livox") {
#if defined(HOUND_HAVE_LIVOX_SDK) && HOUND_HAVE_LIVOX_SDK
    auto livox = std::make_unique<LivoxLidarDevice>();
    if (!config_path_.empty()) {
      livox->set_config_path(config_path_);
    }
    livox->set_frame_time_ms(frame_time_ms_);
    lidar_ = std::move(livox);
#else
    throw std::runtime_error(
      "lidar_backend=livox but composite_sensing was built without Livox-SDK2");
#endif
  } else if (lidar_backend_ == "ouster") {
    throw std::runtime_error(
      "lidar_backend=ouster adapter not implemented yet");
  } else {
    throw std::runtime_error("Unknown lidar_backend: " + lidar_backend_);
  }

  LidarOpenConfig lcfg;
  lcfg.backend = lidar_backend_;
  lcfg.port = port_;
  lcfg.cloud_scan_num = static_cast<uint16_t>(cloud_scan_num_);
  lcfg.range_min = range_min_;
  lcfg.range_max = range_max_;
  lcfg.frame_id = lidar_frame_;
  lcfg.lidar_ip = lidar_ip_;
  lcfg.local_ip = local_ip_;
  lcfg.lidar_port = lidar_port_;
  lcfg.local_port = local_port_;
  lcfg.config_path = config_path_;
  lcfg.frame_time_ms = frame_time_ms_;
  lcfg.temp_query_period_s = temp_query_period_s_;
  lcfg.enable_imu = enable_imu_;
  lidar_->open(lcfg);

  static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
  publish_static_lidar_tf();

  if (deskew_enable_ && deskew_backend_ != "fast_lio") {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      deskew_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarMeshCompositeNode::on_odom, this, std::placeholders::_1));
  }
  if (deskew_backend_ == "fast_lio") {
    if (!enable_imu_) {
      throw std::runtime_error(
        "deskew.backend=fast_lio requires enable_imu (Livox IMU)");
    }
    FastLioConfig lio_cfg;
    Eigen::Isometry3d T_lidar_imu = Eigen::Isometry3d::Identity();
    T_lidar_imu.translation() = Eigen::Vector3d(imu_x_, imu_y_, imu_z_);
    T_lidar_imu.linear() =
      Eigen::Quaterniond(imu_qw_, imu_qx_, imu_qy_, imu_qz_)
      .normalized()
      .toRotationMatrix();
    const Eigen::Isometry3d T_imu_lidar = T_lidar_imu.inverse();
    lio_cfg.lidar_T_wrt_imu = T_imu_lidar.translation();
    lio_cfg.lidar_R_wrt_imu = T_imu_lidar.linear();
    fast_lio_ = std::make_unique<FastLioOdometry>(lio_cfg);
    lio_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      fast_lio_odom_topic_, 10);
  }

  if (publish_cloud_) {
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 2);
  }
  if (publish_raw_cloud_) {
    raw_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      raw_cloud_topic_, 2);
  }
  if (enable_imu_ && !imu_topic_.empty()) {
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS());
  }
  if (!core_temp_topic_.empty()) {
    core_temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>(
      core_temp_topic_, 10);
  }
  if (!environment_temp_topic_.empty()) {
    environment_temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>(
      environment_temp_topic_, 10);
  }

  running_.store(true);
  capture_thread_ = std::thread([this]() { capture_loop(); });

  RCLCPP_INFO(
    get_logger(),
    "lidar_mesh_composite up: backend=%s lidar_ip=%s cloud=%s imu=%s deskew=%s "
    "odom=%s motion_frame=%s wire_latency_ms=%.1f (output=%s)",
    lidar_backend_.c_str(),
    lidar_ip_.empty() ? "(n/a)" : lidar_ip_.c_str(),
    cloud_topic_.c_str(),
    enable_imu_ ? imu_topic_.c_str() : "off",
    fast_lio_ ? "fast_lio" : (deskew_enable_ ? "odom" : "off"),
    fast_lio_ ? fast_lio_odom_topic_.c_str() : deskew_odom_topic_.c_str(),
    fast_lio_ ? fast_lio_odom_frame_.c_str() : deskew_motion_frame_.c_str(),
    static_cast<double>(deskew_wire_latency_ns_) * 1e-6,
    lidar_frame_.c_str());
}

void LidarMeshCompositeNode::publish_static_lidar_tf()
{
  std::vector<geometry_msgs::msg::TransformStamped> tfs;
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = now();
  tf.header.frame_id = parent_frame_;
  tf.child_frame_id = lidar_frame_;
  tf.transform.translation.x = t_x_;
  tf.transform.translation.y = t_y_;
  tf.transform.translation.z = t_z_;
  tf.transform.rotation.x = q_x_;
  tf.transform.rotation.y = q_y_;
  tf.transform.rotation.z = q_z_;
  tf.transform.rotation.w = q_w_;
  tfs.push_back(tf);

  if (enable_imu_ && imu_frame_ != lidar_frame_) {
    geometry_msgs::msg::TransformStamped imu_tf;
    imu_tf.header.stamp = tf.header.stamp;
    imu_tf.header.frame_id = lidar_frame_;
    imu_tf.child_frame_id = imu_frame_;
    imu_tf.transform.translation.x = imu_x_;
    imu_tf.transform.translation.y = imu_y_;
    imu_tf.transform.translation.z = imu_z_;
    imu_tf.transform.rotation.x = imu_qx_;
    imu_tf.transform.rotation.y = imu_qy_;
    imu_tf.transform.rotation.z = imu_qz_;
    imu_tf.transform.rotation.w = imu_qw_;
    tfs.push_back(imu_tf);
  }
  static_tf_->sendTransform(tfs);
}

void LidarMeshCompositeNode::publish_imu()
{
  if (!lidar_ || !imu_pub_) {
    return;
  }
  std::vector<LidarImuSample> samples;
  lidar_->drain_imu(samples);
  for (const auto & s : samples) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = rclcpp::Time(s.timestamp_ns);
    msg.header.frame_id = imu_frame_;
    msg.orientation.w = 1.0;
    msg.orientation_covariance[0] = -1.0;
    msg.angular_velocity.x = s.gx;
    msg.angular_velocity.y = s.gy;
    msg.angular_velocity.z = s.gz;
    msg.linear_acceleration.x = s.ax;
    msg.linear_acceleration.y = s.ay;
    msg.linear_acceleration.z = s.az;
    imu_pub_->publish(msg);
  }
  if (fast_lio_) {
    emit_lio(fast_lio_->push_imu(samples));
  }
}

void LidarMeshCompositeNode::publish_cloud(
  const LidarFrame & cloud,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub)
{
  if (!pub) {
    return;
  }
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = rclcpp::Time(cloud.timestamp_ns);
  msg.header.frame_id = cloud.frame_id.empty() ? lidar_frame_ : cloud.frame_id;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(cloud.points.size());
  msg.is_dense = false;
  msg.is_bigendian = false;
  sensor_msgs::PointCloud2Modifier mod(msg);
  // "time" = seconds relative to header stamp (scan start). Kept for debug /
  // legacy consumers; with in-process deskew, mapper MC should be off.
  mod.setPointCloud2Fields(
    5,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
    "time", 1, sensor_msgs::msg::PointField::FLOAT32);
  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_i(msg, "intensity");
  sensor_msgs::PointCloud2Iterator<float> iter_t(msg, "time");
  for (const auto & p : cloud.points) {
    *iter_x = p.x;
    *iter_y = p.y;
    *iter_z = p.z;
    *iter_i = p.intensity;
    *iter_t = p.time_offset_s;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_i;
    ++iter_t;
  }
  pub->publish(msg);
}

void LidarMeshCompositeNode::publish_temperatures()
{
  if (!lidar_) {
    return;
  }
  lidar_->request_status_update();
  const LidarDeviceStatus st = lidar_->get_status();
  const auto stamp = now();

  if (st.core_temp_valid && core_temp_pub_) {
    sensor_msgs::msg::Temperature msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = lidar_frame_;
    msg.temperature = static_cast<double>(st.core_temp_c);
    msg.variance = 0.0;
    core_temp_pub_->publish(msg);

    if (st.core_temp_c >= static_cast<float>(core_temp_warn_c_)) {
      if (!core_temp_high_warned_) {
        RCLCPP_WARN(
          get_logger(),
          "Livox core_temp high: %.1f C (warn >= %.1f C)",
          st.core_temp_c, core_temp_warn_c_);
        core_temp_high_warned_ = true;
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 30000,
          "Livox core_temp high: %.1f C (warn >= %.1f C)",
          st.core_temp_c, core_temp_warn_c_);
      }
    } else if (core_temp_high_warned_) {
      RCLCPP_INFO(
        get_logger(),
        "Livox core_temp recovered: %.1f C", st.core_temp_c);
      core_temp_high_warned_ = false;
    }
  }
  if (st.environment_temp_valid && environment_temp_pub_) {
    sensor_msgs::msg::Temperature msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = lidar_frame_;
    msg.temperature = static_cast<double>(st.environment_temp_c);
    msg.variance = 0.0;
    environment_temp_pub_->publish(msg);
  }
}

void LidarMeshCompositeNode::capture_loop()
{
  last_status_log_ = std::chrono::steady_clock::now();
  last_cloud_steady_ = last_status_log_;
  bool saw_first_cloud = false;

  while (running_.load(std::memory_order_relaxed)) {
    const auto now_tp = std::chrono::steady_clock::now();

    LidarFrame cloud;
    publish_imu();
    const int wait_ms = enable_imu_ ? 20 : 200;
    if (!lidar_->wait_cloud(cloud, wait_ms)) {
      publish_imu();
      publish_temperatures();
      if (cloud_stale_warn_s_ > 0.0 &&
          (now_tp - last_cloud_steady_) >=
            std::chrono::duration<double>(cloud_stale_warn_s_))
      {
        if (!cloud_stale_warned_) {
          RCLCPP_WARN(
            get_logger(),
            "no lidar cloud for >%.1f s (topic %s)%s",
            cloud_stale_warn_s_, cloud_topic_.c_str(),
            saw_first_cloud ? "" : " — never received first cloud");
          cloud_stale_warned_ = true;
        }
      }
    } else {
      if (cloud_stale_warned_) {
        RCLCPP_INFO(get_logger(), "lidar cloud resumed (%zu points)", cloud.points.size());
        cloud_stale_warned_ = false;
      } else if (!saw_first_cloud) {
        RCLCPP_INFO(
          get_logger(), "lidar cloud online: %zu points → %s",
          cloud.points.size(), cloud_topic_.c_str());
      }
      saw_first_cloud = true;
      last_cloud_steady_ = now_tp;
      last_cloud_points_ = cloud.points.size();
      publish_imu();
      publish_temperatures();

      if (fast_lio_) {
        if (publish_raw_cloud_ && raw_cloud_pub_) {
          publish_cloud(cloud, raw_cloud_pub_);
        }
        const auto scans = fast_lio_->push_cloud(cloud);
        if (scans.empty() && !fast_lio_->initialized()) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000, "fast-lio initializing");
        } else if (scans.empty() && fast_lio_->pending_scans() > 0) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "fast-lio waiting on imu (%zu scans)", fast_lio_->pending_scans());
        }
        emit_lio(scans);
      } else {
        // Header is host receive (flush ≈ scan end, minus intra-scan span).
        // Subtract wire/SDK latency so stamp is photon time; deskew window
        // then sits in the past of EKF odom instead of at host now.
        if (deskew_wire_latency_ns_ > 0 &&
          cloud.timestamp_ns > deskew_wire_latency_ns_)
        {
          cloud.timestamp_ns -= deskew_wire_latency_ns_;
        }
        if (publish_raw_cloud_ && raw_cloud_pub_) {
          publish_cloud(cloud, raw_cloud_pub_);
        }
        LidarFrame out = cloud;
        if (deskew_enable_) {
          double max_off = 0.0;
          for (const auto & p : cloud.points) {
            if (std::isfinite(p.time_offset_s)) {
              max_off = std::max(max_off, static_cast<double>(p.time_offset_s));
            }
          }
          const int64_t end_ns =
            cloud.timestamp_ns +
            static_cast<int64_t>(std::llround(max_off * 1e9));
          const auto poses = snapshot_odom_poses(cloud.timestamp_ns, end_ns);
          const DeskewResult dr = deskew_lidar_frame(
            out, T_base_lidar_, poses, deskew_reference_);
          if (dr.ok) {
            if (dr.elapsed_ms >= deskew_warn_ms_) {
              RCLCPP_WARN(
                get_logger(), "deskew took %.2f ms (warn threshold %.1f)",
                dr.elapsed_ms, deskew_warn_ms_);
            }
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "deskew wall time: %.2f ms poses=%zu",
              dr.elapsed_ms, poses.size());
          } else {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "deskew skipped (%.2f ms): %s — publishing raw",
              dr.elapsed_ms, dr.error.c_str());
            out = cloud;
          }
        }
        if (publish_cloud_) {
          publish_cloud(out, cloud_pub_);
          ++clouds_published_;
          points_published_ += out.points.size();
        }
      }
    }

    if (status_log_period_s_ > 0.0 &&
        (now_tp - last_status_log_) >=
          std::chrono::duration<double>(status_log_period_s_))
    {
      const double dt = std::chrono::duration<double>(
        now_tp - last_status_log_).count();
      const LidarDeviceStatus st = lidar_ ? lidar_->get_status() : LidarDeviceStatus{};
      if (st.core_temp_valid) {
        RCLCPP_INFO(
          get_logger(),
          "lidar ok: %.1f Hz cloud, last %zu pts, core_temp=%.1f C",
          clouds_published_ / std::max(dt, 1e-3),
          last_cloud_points_, st.core_temp_c);
      } else {
        RCLCPP_INFO(
          get_logger(),
          "lidar ok: %.1f Hz cloud, last %zu pts (no core_temp yet)",
          clouds_published_ / std::max(dt, 1e-3),
          last_cloud_points_);
      }
      clouds_published_ = 0;
      points_published_ = 0;
      last_status_log_ = now_tp;
    }
  }
}

void LidarMeshCompositeNode::emit_lio(const std::vector<FastLioScan> & scans)
{
  for (const auto & scan : scans) {
    if (publish_cloud_) {
      publish_cloud(scan.cloud, cloud_pub_);
      ++clouds_published_;
      points_published_ += scan.cloud.points.size();
      last_cloud_points_ = scan.cloud.points.size();
    }
    if (!scan.odom.valid || !lio_odom_pub_) {
      continue;
    }
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = rclcpp::Time(scan.odom.stamp_ns);
    msg.header.frame_id = fast_lio_odom_frame_;
    msg.child_frame_id = imu_frame_;
    msg.pose.pose.position.x = scan.odom.position.x();
    msg.pose.pose.position.y = scan.odom.position.y();
    msg.pose.pose.position.z = scan.odom.position.z();
    msg.pose.pose.orientation.x = scan.odom.orientation.x();
    msg.pose.pose.orientation.y = scan.odom.orientation.y();
    msg.pose.pose.orientation.z = scan.odom.orientation.z();
    msg.pose.pose.orientation.w = scan.odom.orientation.w();
    msg.twist.twist.linear.x = scan.odom.vel_body.x();
    msg.twist.twist.linear.y = scan.odom.vel_body.y();
    msg.twist.twist.linear.z = scan.odom.vel_body.z();
    lio_odom_pub_->publish(msg);
    if (!lio_announced_) {
      RCLCPP_INFO(
        get_logger(), "fast-lio odom on %s (%s → %s)",
        fast_lio_odom_topic_.c_str(), fast_lio_odom_frame_.c_str(),
        imu_frame_.c_str());
      lio_announced_ = true;
    }
  }
}

void LidarMeshCompositeNode::on_odom(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  const std::string & parent = msg->header.frame_id;
  const std::string & child = msg->child_frame_id;
  if (parent != deskew_motion_frame_ || child != base_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "deskew odom frames %s→%s (expected %s→%s); using pose anyway",
      parent.c_str(), child.c_str(),
      deskew_motion_frame_.c_str(), base_frame_.c_str());
  }

  StampedOdomPose sample;
  sample.stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  if (sample.stamp_ns <= 0) {
    return;
  }
  const auto & p = msg->pose.pose;
  sample.T_odom_base = Eigen::Isometry3d::Identity();
  sample.T_odom_base.translation() =
    Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  sample.T_odom_base.linear() =
    Eigen::Quaterniond(
      p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)
      .normalized()
      .toRotationMatrix();

  const int64_t keep_ns =
    sample.stamp_ns -
    static_cast<int64_t>(std::llround(deskew_pose_buffer_s_ * 1e9));
  std::lock_guard<std::mutex> lock(odom_mu_);
  if (!odom_poses_.empty() && sample.stamp_ns < odom_poses_.back().stamp_ns) {
    return;
  }
  odom_poses_.push_back(std::move(sample));
  while (!odom_poses_.empty() && odom_poses_.front().stamp_ns < keep_ns) {
    odom_poses_.pop_front();
  }
}

std::vector<StampedOdomPose> LidarMeshCompositeNode::snapshot_odom_poses(
  int64_t start_ns, int64_t end_ns)
{
  std::lock_guard<std::mutex> lock(odom_mu_);
  if (odom_poses_.empty()) {
    return {};
  }
  // One sample on each side of [start_ns, end_ns] when available.
  auto lo = std::lower_bound(
    odom_poses_.begin(), odom_poses_.end(), start_ns,
    [](const StampedOdomPose & p, int64_t t) { return p.stamp_ns < t; });
  auto hi = std::upper_bound(
    odom_poses_.begin(), odom_poses_.end(), end_ns,
    [](int64_t t, const StampedOdomPose & p) { return t < p.stamp_ns; });
  if (lo != odom_poses_.begin()) {
    --lo;
  }
  if (hi != odom_poses_.end()) {
    ++hi;  // include one sample after scan end
  }
  std::vector<StampedOdomPose> out(lo, hi);
  return out;
}

}  // namespace composite_sensing
