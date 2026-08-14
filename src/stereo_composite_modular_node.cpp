#include "composite_sensing/stereo_composite_modular_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <optional>

#include <rclcpp/qos.hpp>

#include "composite_sensing/cuvslam_backend.hpp"
#include "composite_sensing/dataset_camera_device.hpp"
#include "composite_sensing/frame_conventions.hpp"
#include "composite_sensing/realsense_camera_device.hpp"

namespace composite_sensing {
namespace {

rclcpp::QoS sensor_qos(size_t depth = 5)
{
  return rclcpp::SensorDataQoS().keep_last(depth);
}

cuvslam::Pose pose_optical_to_cuvslam(const PoseOptical & p)
{
  cuvslam::Pose pose;
  pose.translation = {p.translation[0], p.translation[1], p.translation[2]};
  pose.rotation = {p.rotation[0], p.rotation[1], p.rotation[2], p.rotation[3]};
  return pose;
}

}  // namespace

StereoCompositeModularNode::StereoCompositeModularNode(
  const rclcpp::NodeOptions & options)
: Node("stereo_composite_node", options)
{
  declare_params();

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  rclcpp::QoS odom_qos(10);
  odom_qos.reliable();
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, odom_qos);
  if (camera_backend_ == "dataset") {
    gt_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/dataset/gt_odometry", odom_qos);
  }

  color_optical_frame_ = camera_name_ + "_color_optical_frame";
  depth_optical_frame_ = camera_name_ + "_depth_optical_frame";

  if (enable_color_) {
    const std::string base = "/" + camera_name_ + "/color";
    color_pub_ = create_publisher<sensor_msgs::msg::Image>(base + "/image_raw", sensor_qos());
    color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      base + "/camera_info", sensor_qos());
  }
  if (enable_depth_) {
    if (align_depth_ && !enable_color_) {
      RCLCPP_WARN(
        get_logger(),
        "align_depth=true requires color; enabling color streams");
      enable_color_ = true;
      const std::string cbase = "/" + camera_name_ + "/color";
      color_pub_ = create_publisher<sensor_msgs::msg::Image>(
        cbase + "/image_raw", sensor_qos());
      color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        cbase + "/camera_info", sensor_qos());
    }
    const std::string base = align_depth_
      ? ("/" + camera_name_ + "/aligned_depth_to_color")
      : ("/" + camera_name_ + "/depth");
    const std::string image_name = align_depth_ ? "/image_raw" : "/image_rect_raw";
    if (align_depth_) {
      depth_optical_frame_ = color_optical_frame_;
    }
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(base + image_name, sensor_qos());
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      base + "/camera_info", sensor_qos());
  }

  start_pipeline();
  publish_static_tfs();

  running_ = true;
  capture_thread_ = std::thread([this] { capture_loop(); });
  odom_thread_ = std::thread([this] { odom_worker(); });
  if (enable_color_) {
    color_thread_ = std::thread([this] { color_worker(); });
  }
  if (enable_depth_) {
    depth_thread_ = std::thread([this] { depth_worker(); });
  }

  RCLCPP_INFO(
    get_logger(),
    "stereo_composite (modular): backend=%s serial=%s IR %dx%d@%d color=%s@%dHz(pub %.1f) "
    "depth=%s%s vslam=%s odom_source=%s odom=%s child=%s publish_tf=%s mount=%s",
    camera_backend_.c_str(), serial_number_.c_str(),
    infra_width_, infra_height_, infra_fps_,
    enable_color_ ? "on" : "off", color_fps_, color_publish_fps_,
    enable_depth_ ? "on" : "off",
    enable_depth_ ? (align_depth_ ? "(aligned_to_color)" : "(native)") : "",
    enable_vslam_ ? "on" : "off",
    odom_source_.c_str(),
    odom_topic_.c_str(),
    base_frame_.c_str(),
    publish_odom_tf_ ? "true" : "false",
    apply_body_mount_ ? "body←cam" : "identity");
}

StereoCompositeModularNode::~StereoCompositeModularNode()
{
  running_ = false;
  slot_.cv_pose.notify_all();
  slot_.cv_color.notify_all();
  slot_.cv_depth.notify_all();
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  if (odom_thread_.joinable()) {
    odom_thread_.join();
  }
  if (color_thread_.joinable()) {
    color_thread_.join();
  }
  if (depth_thread_.joinable()) {
    depth_thread_.join();
  }
  if (camera_) {
    camera_->stop();
  }
}

void StereoCompositeModularNode::declare_params()
{
  serial_number_ = declare_parameter<std::string>("serial_number", "030422250301");
  camera_name_ = declare_parameter<std::string>("camera_name", "camera");
  camera_backend_ = declare_parameter<std::string>("camera_backend", "realsense");
  dataset_path_ = declare_parameter<std::string>("dataset_path", "");
  dataset_view_ = declare_parameter<std::string>("dataset_view", "front");
  dataset_loop_ = declare_parameter<bool>("dataset_loop", true);
  dataset_rate_hz_ = declare_parameter<double>("dataset_rate_hz", 30.0);
  enable_vslam_ = declare_parameter<bool>("enable_vslam", true);
  odom_source_ = declare_parameter<std::string>("odom_source", "vslam");
  infra_width_ = declare_parameter<int>("infra_width", 640);
  infra_height_ = declare_parameter<int>("infra_height", 360);
  infra_fps_ = declare_parameter<int>("infra_fps", 60);
  enable_color_ = declare_parameter<bool>("enable_color", true);
  color_width_ = declare_parameter<int>("color_width", 640);
  color_height_ = declare_parameter<int>("color_height", 360);
  color_fps_ = declare_parameter<int>("color_fps", 30);
  color_publish_fps_ = declare_parameter<double>("color_publish_fps", 15.0);
  enable_depth_ = declare_parameter<bool>("enable_depth", false);
  align_depth_ = declare_parameter<bool>("align_depth", true);
  depth_width_ = declare_parameter<int>("depth_width", 640);
  depth_height_ = declare_parameter<int>("depth_height", 360);
  depth_fps_ = declare_parameter<int>("depth_fps", 30);
  depth_publish_fps_ = declare_parameter<double>("depth_publish_fps", 15.0);
  emitter_enabled_ = declare_parameter<int>("emitter_enabled", 0);
  visual_preset_ = declare_parameter<int>("visual_preset", 3);
  clip_distance_ = declare_parameter<double>("clip_distance", 0.0);
  initial_reset_ = declare_parameter<bool>("initial_reset", false);
  odom_topic_ = declare_parameter<std::string>(
    "odom_topic", "/visual_slam/tracking/odometry");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", false);
  {
    t_body_sensor_[0] = declare_parameter<double>("xyz.x", 0.0);
    t_body_sensor_[1] = declare_parameter<double>("xyz.y", 0.0);
    t_body_sensor_[2] = declare_parameter<double>("xyz.z", 0.0);
    const double roll = declare_parameter<double>("rpy.roll", 0.0);
    const double pitch = declare_parameter<double>("rpy.pitch", 0.0);
    const double yaw = declare_parameter<double>("rpy.yaw", 0.0);
    rpy_deg_to_R(roll, pitch, yaw, R_body_sensor_);
    const double tnorm =
      std::abs(t_body_sensor_[0]) + std::abs(t_body_sensor_[1]) +
      std::abs(t_body_sensor_[2]);
    apply_body_mount_ = tnorm > 1e-9 ||
      std::abs(roll) > 1e-9 || std::abs(pitch) > 1e-9 || std::abs(yaw) > 1e-9;
  }
  async_sba_ = declare_parameter<bool>("async_sba", true);
  slam_sync_mode_ = declare_parameter<bool>("slam_sync_mode", false);
  warmup_frames_ = declare_parameter<int>("warmup_frames", 60);
  log_cuvslam_timing_ = declare_parameter<bool>("log_cuvslam_timing", false);
  profile_ = declare_parameter<bool>("profile", false);

  if (enable_depth_ && !enable_color_ && align_depth_) {
    RCLCPP_WARN(
      get_logger(),
      "enable_depth+align_depth requires color; enabling color streams");
    enable_color_ = true;
  }
  if (enable_depth_) {
    if (depth_width_ != infra_width_ || depth_height_ != infra_height_ ||
      depth_fps_ != infra_fps_)
    {
      RCLCPP_WARN(
        get_logger(),
        "depth %dx%d@%d forced to match infra %dx%d@%d (shared depth module)",
        depth_width_, depth_height_, depth_fps_,
        infra_width_, infra_height_, infra_fps_);
      depth_width_ = infra_width_;
      depth_height_ = infra_height_;
      depth_fps_ = infra_fps_;
    }
  }
}

void StereoCompositeModularNode::start_pipeline()
{
  if (camera_backend_ != "dataset" && initial_reset_) {
    RCLCPP_INFO(
      get_logger(),
      "initial_reset: hardware_reset on serial %s (wait for re-enumerate)",
      serial_number_.c_str());
  }

  CameraOpenConfig cfg;
  cfg.serial = serial_number_;
  cfg.infra_width = infra_width_;
  cfg.infra_height = infra_height_;
  cfg.infra_fps = infra_fps_;
  cfg.enable_color = enable_color_;
  cfg.color_width = color_width_;
  cfg.color_height = color_height_;
  cfg.color_fps = color_fps_;
  cfg.enable_depth = enable_depth_;
  cfg.align_depth = align_depth_;
  cfg.depth_width = depth_width_;
  cfg.depth_height = depth_height_;
  cfg.depth_fps = depth_fps_;
  cfg.visual_preset = visual_preset_;
  cfg.emitter_enabled = emitter_enabled_;
  cfg.clip_distance_m = static_cast<float>(clip_distance_);
  cfg.initial_reset = initial_reset_;
  cfg.dataset_path = dataset_path_;
  cfg.dataset_view = dataset_view_;
  cfg.dataset_loop = dataset_loop_;
  cfg.dataset_rate_hz = dataset_rate_hz_;

  dataset_camera_ = nullptr;
  if (camera_backend_ == "dataset") {
    auto cam = std::make_unique<DatasetCameraDevice>();
    cam->open(cfg);
    if (enable_color_) {
      color_camera_info_ = cam->color_camera_info();
      color_camera_info_.header.frame_id = color_optical_frame_;
    }
    if (enable_depth_) {
      depth_camera_info_ = cam->depth_camera_info();
      depth_camera_info_.header.frame_id = depth_optical_frame_;
    }
    dataset_camera_ = cam.get();
    camera_ = std::move(cam);
    RCLCPP_INFO(
      get_logger(),
      "DatasetCameraDevice open: path=%s view=%s rate=%.1fHz loop=%s",
      dataset_path_.c_str(), dataset_view_.c_str(), dataset_rate_hz_,
      dataset_loop_ ? "true" : "false");
  } else {
    auto cam = std::make_unique<RealsenseCameraDevice>();
    cam->open(cfg);
    if (enable_color_) {
      color_camera_info_ = cam->color_camera_info();
      color_camera_info_.header.frame_id = color_optical_frame_;
    }
    if (enable_depth_) {
      depth_camera_info_ = cam->depth_camera_info();
      depth_camera_info_.header.frame_id = depth_optical_frame_;
    }
    camera_ = std::move(cam);
  }

  if (enable_vslam_) {
    VslamConfig vcfg;
    vcfg.async_sba = async_sba_;
    vcfg.slam_sync_mode = slam_sync_mode_;
    vslam_ = std::make_unique<CuvslamBackend>();
    vslam_->configure(camera_->stereo_calibration(), vcfg);
    RCLCPP_INFO(
      get_logger(),
      "cuVSLAM ready (modular; async_sba=%s slam_sync=%s; odom_source=%s)",
      async_sba_ ? "true" : "false",
      slam_sync_mode_ ? "true" : "false",
      odom_source_.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(),
      "enable_vslam=false — using dataset GT poses when available (odom_source=%s)",
      odom_source_.c_str());
  }
}

void StereoCompositeModularNode::publish_static_tfs()
{
  std::vector<geometry_msgs::msg::TransformStamped> tfs;
  if (dataset_camera_) {
    tfs = dataset_camera_->static_camera_tfs(camera_name_);
  } else if (auto * rs = dynamic_cast<RealsenseCameraDevice *>(camera_.get())) {
    tfs = rs->static_camera_tfs(camera_name_);
  }
  const auto now = get_clock()->now();
  for (auto & tf : tfs) {
    tf.header.stamp = now;
  }
  if (!tfs.empty()) {
    static_tf_broadcaster_->sendTransform(tfs);
  }
  RCLCPP_INFO(get_logger(), "published %zu static camera TFs", tfs.size());
}

void StereoCompositeModularNode::capture_loop()
{
  using clock = std::chrono::steady_clock;
  int frame_id = 0;
  int64_t prev_ts = -1;
  int64_t last_track_ts = -1;
  constexpr int64_t kJitterWarnNs = 75LL * 1000000LL;

  double sum_vo_ms = 0.0;
  double sum_iter_ms = 0.0;
  double max_vo_ms = 0.0;
  double max_iter_ms = 0.0;
  int n_track = 0;
  int n_skip_ts = 0;
  int n_ir_frames = 0;
  int color_raw_count = 0;
  const int color_stride = (enable_color_ && color_publish_fps_ > 0.0)
    ? std::max(
      1,
      static_cast<int>(std::lround(
        static_cast<double>(color_fps_) / color_publish_fps_)))
    : 1;
  auto stats_t0 = clock::now();
  auto profile_t0 = stats_t0;
  const bool log_timing = log_cuvslam_timing_;
  const bool profile = profile_;
  double sum_sdk_to_track_ms = 0.0;
  double sum_deq_to_track_ms = 0.0;
  double max_sdk_to_track_ms = 0.0;
  double max_deq_to_track_ms = 0.0;
  int n_profile = 0;
  int n_arrival_missing = 0;

  if (profile) {
    RCLCPP_INFO(
      get_logger(),
      "profile=true: measuring SDK TIME_OF_ARRIVAL→Track and dequeue→Track");
  }

  if (enable_color_) {
    RCLCPP_INFO(
      get_logger(),
      "color publish: every %d sensor frame(s) (%d Hz raw → ~%.1f Hz)",
      color_stride, color_fps_,
      static_cast<double>(color_fps_) / static_cast<double>(color_stride));
  }

  while (running_ && rclcpp::ok()) {
    const auto iter_t0 = clock::now();

    // Drain color; keep every Nth for publish (does not gate Track).
    if (enable_color_) {
      ColorFrame color_f;
      while (camera_->poll_color(color_f)) {
        ++color_raw_count;
        if ((color_raw_count % color_stride) != 0) {
          continue;
        }
        const auto stamp = this->get_clock()->now();
        sensor_msgs::msg::Image img;
        img.header.stamp = stamp;
        img.header.frame_id = color_optical_frame_;
        img.height = static_cast<uint32_t>(color_f.height);
        img.width = static_cast<uint32_t>(color_f.width);
        img.encoding = "rgb8";
        img.is_bigendian = 0;
        img.step = static_cast<uint32_t>(
          color_f.stride > 0 ? color_f.stride : color_f.width * 3);
        const size_t nbytes = static_cast<size_t>(img.step) * img.height;
        img.data.resize(nbytes);
        std::memcpy(img.data.data(), color_f.pixels, nbytes);
        {
          std::lock_guard<std::mutex> lock(slot_.mu);
          slot_.color_image = std::move(img);
          slot_.color_stamp = stamp;
          slot_.has_color = true;
          ++slot_.color_seq;
        }
        slot_.cv_color.notify_one();
      }
    }

    StereoFrame stereo;
    if (!camera_->wait_stereo(stereo, 1000)) {
      continue;
    }
    const auto deq_t0 = profile ? clock::now() : clock::time_point{};

    ++frame_id;
    if (log_timing) {
      ++n_ir_frames;
    }

    const int64_t timestamp_ns = stereo.timestamp_ns;

    if (prev_ts >= 0 && (timestamp_ns - prev_ts) > kJitterWarnNs) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IR timestamp gap %.1f ms", (timestamp_ns - prev_ts) * 1e-6);
    }
    prev_ts = timestamp_ns;

    if (frame_id <= warmup_frames_) {
      if (enable_depth_) {
        std::lock_guard<std::mutex> lock(slot_.mu);
        slot_.stereo_stamp = this->get_clock()->now();
        slot_.has_stereo_for_depth = true;
        slot_.cv_depth.notify_one();
      }
      continue;
    }

    constexpr int64_t kTsResetBackNs = 1000000000LL;
    constexpr int64_t kTsRejectFwdNs = 10000000000LL;
    if (last_track_ts > 0 && timestamp_ns + kTsResetBackNs < last_track_ts) {
      RCLCPP_WARN(
        get_logger(),
        "IR timestamp jumped backwards (%ld -> %ld); resetting track clock",
        static_cast<long>(last_track_ts), static_cast<long>(timestamp_ns));
      last_track_ts = 0;
    } else if (last_track_ts > 0 && timestamp_ns > last_track_ts + kTsRejectFwdNs) {
      if (log_timing) {
        ++n_skip_ts;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "skipping absurd IR timestamp jump (%ld -> %ld)",
        static_cast<long>(last_track_ts), static_cast<long>(timestamp_ns));
      continue;
    }

    if (timestamp_ns <= last_track_ts) {
      if (log_timing) {
        ++n_skip_ts;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "skipping non-increasing IR timestamp (%ld <= %ld)",
        static_cast<long>(timestamp_ns), static_cast<long>(last_track_ts));
      continue;
    }

    const auto vo_t0 = log_timing ? clock::now() : clock::time_point{};
    double sdk_to_track_ms = 0.0;
    double deq_to_track_ms = 0.0;
    if (profile) {
      deq_to_track_ms =
        std::chrono::duration<double, std::milli>(clock::now() - deq_t0).count();
      if (stereo.arrival_host_ns > 0) {
        const int64_t now_host_ns =
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
        sdk_to_track_ms =
          static_cast<double>(now_host_ns - stereo.arrival_host_ns) * 1e-6;
      } else {
        ++n_arrival_missing;
      }
    }
    std::optional<PoseOptical> vslam_pose;
    PoseOptical gt_pose{};
    bool have_gt = false;
    if (dataset_camera_) {
      have_gt = dataset_camera_->gt_pose_optical(gt_pose);
    }
    try {
      if (enable_vslam_ && vslam_) {
        // Always Track for compute load even when odom_source=gt.
        vslam_pose = vslam_->track(stereo);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "pose update failed: %s", e.what());
      continue;
    }

    std::optional<PoseOptical> pose_opt;
    if (odom_source_ == "gt") {
      if (have_gt) {
        pose_opt = gt_pose;
      }
    } else if (vslam_pose.has_value()) {
      pose_opt = vslam_pose;
    } else if (!enable_vslam_ && have_gt) {
      pose_opt = gt_pose;
    }

    const double vo_ms = log_timing
      ? std::chrono::duration<double, std::milli>(clock::now() - vo_t0).count()
      : 0.0;
    last_track_ts = timestamp_ns;

    if (!pose_opt.has_value()) {
      if (enable_depth_) {
        std::lock_guard<std::mutex> lock(slot_.mu);
        slot_.stereo_stamp = this->get_clock()->now();
        slot_.has_stereo_for_depth = true;
        slot_.cv_depth.notify_one();
      }
      if (enable_vslam_ && odom_source_ != "gt") {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "pose tracking not valid");
      } else if (odom_source_ == "gt") {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "odom_source=gt but no GT pose");
      }
      continue;
    }

    if (profile) {
      sum_sdk_to_track_ms += sdk_to_track_ms;
      sum_deq_to_track_ms += deq_to_track_ms;
      max_sdk_to_track_ms = std::max(max_sdk_to_track_ms, sdk_to_track_ms);
      max_deq_to_track_ms = std::max(max_deq_to_track_ms, deq_to_track_ms);
      ++n_profile;
    }

    if (log_timing) {
      const double iter_ms =
        std::chrono::duration<double, std::milli>(clock::now() - iter_t0).count();
      sum_vo_ms += vo_ms;
      sum_iter_ms += iter_ms;
      max_vo_ms = std::max(max_vo_ms, vo_ms);
      max_iter_ms = std::max(max_iter_ms, iter_ms);
      ++n_track;

      const auto stats_now = clock::now();
      if (stats_now - stats_t0 >= std::chrono::seconds(2)) {
        const double dt =
          std::chrono::duration<double>(stats_now - stats_t0).count();
        const double inv_n = n_track > 0 ? 1.0 / static_cast<double>(n_track) : 0.0;
        RCLCPP_INFO(
          get_logger(),
          "cuVSLAM timing (%.1fs): track mean/max=%.2f/%.2f ms  "
          "iter mean/max=%.2f/%.2f ms  "
          "track=%.1f Hz  ir_frames=%.1f Hz  skip_ts=%d/%d",
          dt,
          sum_vo_ms * inv_n, max_vo_ms,
          sum_iter_ms * inv_n, max_iter_ms,
          n_track / dt, n_ir_frames / dt, n_skip_ts, n_ir_frames);
        sum_vo_ms = sum_iter_ms = 0.0;
        max_vo_ms = max_iter_ms = 0.0;
        n_track = n_skip_ts = n_ir_frames = 0;
        stats_t0 = stats_now;
      }
    }

    if (profile) {
      const auto stats_now = clock::now();
      if (stats_now - profile_t0 >= std::chrono::seconds(2)) {
        const double dt =
          std::chrono::duration<double>(stats_now - profile_t0).count();
        const double inv_n =
          n_profile > 0 ? 1.0 / static_cast<double>(n_profile) : 0.0;
        RCLCPP_INFO(
          get_logger(),
          "profile (%.1fs): sdk_arrival→track mean/max=%.2f/%.2f ms  "
          "dequeue→track mean/max=%.3f/%.3f ms  "
          "n=%d missing_arrival=%d  (%.1f Hz)",
          dt,
          sum_sdk_to_track_ms * inv_n, max_sdk_to_track_ms,
          sum_deq_to_track_ms * inv_n, max_deq_to_track_ms,
          n_profile, n_arrival_missing, n_profile / dt);
        sum_sdk_to_track_ms = sum_deq_to_track_ms = 0.0;
        max_sdk_to_track_ms = max_deq_to_track_ms = 0.0;
        n_profile = n_arrival_missing = 0;
        profile_t0 = stats_now;
      }
    }

    {
      std::lock_guard<std::mutex> lock(slot_.mu);
      slot_.pose = *pose_opt;
      // Dataset frames carry synthetic timestamps (frame_id / rate). Use them
      // so depth/odom/LocalMap stamps align 1:1 with dataset indices. Live
      // cameras keep host ROS time.
      if (camera_backend_ == "dataset" && timestamp_ns > 0) {
        slot_.ros_stamp = rclcpp::Time(timestamp_ns, RCL_ROS_TIME);
      } else {
        slot_.ros_stamp = this->get_clock()->now();
      }
      slot_.has_pose = true;
      ++slot_.pose_seq;
      slot_.has_gt = false;
      if (have_gt) {
        slot_.gt_pose = gt_pose;
        slot_.has_gt = true;
      }
      if (enable_depth_) {
        slot_.stereo_stamp = slot_.ros_stamp;
        slot_.has_stereo_for_depth = true;
      }
    }
    slot_.cv_pose.notify_one();
    if (enable_depth_) {
      slot_.cv_depth.notify_one();
    }
  }
}

void StereoCompositeModularNode::publish_odom_tf(
  const rclcpp::Time & stamp, const PoseOptical & pose_opt)
{
  std::array<double, 3> t{};
  std::array<double, 4> q{};
  // Dataset GT is world←optical (same as smoke_rail_mapper T_wo). Publish it
  // directly as odom←base so TF(odom←*_optical)=T_wo with identity link→optical.
  // RealSense / cuVSLAM path still remaps optical → FLU camera_link.
  if (odom_source_ == "gt") {
    t = {
      static_cast<double>(pose_opt.translation[0]),
      static_cast<double>(pose_opt.translation[1]),
      static_cast<double>(pose_opt.translation[2])};
    q = {
      static_cast<double>(pose_opt.rotation[0]),
      static_cast<double>(pose_opt.rotation[1]),
      static_cast<double>(pose_opt.rotation[2]),
      static_cast<double>(pose_opt.rotation[3])};
  } else {
    const cuvslam::Pose pose = pose_optical_to_cuvslam(pose_opt);
    optical_pose_to_ros_flu(pose, t, q);
    if (apply_body_mount_) {
      std::array<double, 3> t_body{};
      std::array<double, 4> q_body{};
      world_sensor_to_world_body(
        t, q, R_body_sensor_, t_body_sensor_, t_body, q_body);
      t = t_body;
      q = q_body;
    }
  }

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = t[0];
  odom.pose.pose.position.y = t[1];
  odom.pose.pose.position.z = t[2];
  odom.pose.pose.orientation.x = q[0];
  odom.pose.pose.orientation.y = q[1];
  odom.pose.pose.orientation.z = q[2];
  odom.pose.pose.orientation.w = q[3];

  if (prev_stamp_.nanoseconds() > 0) {
    const double dt = (stamp - prev_stamp_).seconds();
    if (dt > 1e-6) {
      odom.twist.twist.linear.x = (t[0] - prev_t_flu_[0]) / dt;
      odom.twist.twist.linear.y = (t[1] - prev_t_flu_[1]) / dt;
      odom.twist.twist.linear.z = (t[2] - prev_t_flu_[2]) / dt;
    }
  }
  prev_t_flu_ = t;
  prev_stamp_ = stamp;
  odom_pub_->publish(odom);

  if (!publish_odom_tf_) {
    return;
  }
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = odom_frame_;
  tf.child_frame_id = base_frame_;
  tf.transform.translation.x = t[0];
  tf.transform.translation.y = t[1];
  tf.transform.translation.z = t[2];
  tf.transform.rotation.x = q[0];
  tf.transform.rotation.y = q[1];
  tf.transform.rotation.z = q[2];
  tf.transform.rotation.w = q[3];
  tf_broadcaster_->sendTransform(tf);
}

void StereoCompositeModularNode::publish_gt_odom(
  const rclcpp::Time & stamp, const PoseOptical & pose_opt)
{
  if (!gt_odom_pub_) {
    return;
  }
  // Same convention as publish_odom_tf when odom_source=gt (raw T_wo).
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = pose_opt.translation[0];
  odom.pose.pose.position.y = pose_opt.translation[1];
  odom.pose.pose.position.z = pose_opt.translation[2];
  odom.pose.pose.orientation.x = pose_opt.rotation[0];
  odom.pose.pose.orientation.y = pose_opt.rotation[1];
  odom.pose.pose.orientation.z = pose_opt.rotation[2];
  odom.pose.pose.orientation.w = pose_opt.rotation[3];
  gt_odom_pub_->publish(odom);
}

void StereoCompositeModularNode::odom_worker()
{
  uint64_t last_seq = 0;
  while (running_ && rclcpp::ok()) {
    PoseOptical pose;
    PoseOptical gt_pose;
    bool has_gt = false;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    {
      std::unique_lock<std::mutex> lock(slot_.mu);
      slot_.cv_pose.wait_for(lock, std::chrono::milliseconds(100), [&] {
        return !running_ || (slot_.has_pose && slot_.pose_seq != last_seq);
      });
      if (!running_) {
        break;
      }
      if (!slot_.has_pose || slot_.pose_seq == last_seq) {
        continue;
      }
      last_seq = slot_.pose_seq;
      pose = slot_.pose;
      stamp = slot_.ros_stamp;
      has_gt = slot_.has_gt;
      if (has_gt) {
        gt_pose = slot_.gt_pose;
      }
    }
    publish_odom_tf(stamp, pose);
    if (has_gt) {
      publish_gt_odom(stamp, gt_pose);
    }
  }
}

void StereoCompositeModularNode::color_worker()
{
  if (color_publish_fps_ <= 0.0) {
    return;
  }
  uint64_t last_seq = 0;

  while (running_ && rclcpp::ok()) {
    sensor_msgs::msg::Image img;
    {
      std::unique_lock<std::mutex> lock(slot_.mu);
      slot_.cv_color.wait_for(lock, std::chrono::milliseconds(100), [&] {
        return !running_ || (slot_.has_color && slot_.color_seq != last_seq);
      });
      if (!running_) {
        break;
      }
      if (!slot_.has_color || slot_.color_seq == last_seq) {
        continue;
      }
      last_seq = slot_.color_seq;
      img = slot_.color_image;
    }

    auto info = color_camera_info_;
    info.header = img.header;
    color_pub_->publish(img);
    color_info_pub_->publish(info);
  }
}

void StereoCompositeModularNode::depth_worker()
{
  using clock = std::chrono::steady_clock;
  if (depth_publish_fps_ <= 0.0) {
    return;
  }
  const auto period = std::chrono::duration_cast<clock::duration>(
    std::chrono::duration<double>(1.0 / depth_publish_fps_));
  auto next = clock::now();

  while (running_ && rclcpp::ok()) {
    {
      std::unique_lock<std::mutex> lock(slot_.mu);
      slot_.cv_depth.wait_until(lock, next, [&] {
        return !running_ || (slot_.has_stereo_for_depth && clock::now() >= next);
      });
      if (!running_) {
        break;
      }
    }

    auto now = clock::now();
    if (now < next) {
      continue;
    }

    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    {
      std::lock_guard<std::mutex> lock(slot_.mu);
      if (!slot_.has_stereo_for_depth) {
        continue;
      }
      stamp = slot_.stereo_stamp;
    }

    DepthFrame depth_f;
    if (!camera_->poll_depth(depth_f)) {
      do {
        next += period;
      } while (next <= clock::now());
      continue;
    }

    sensor_msgs::msg::Image img;
    img.header.stamp = stamp;
    img.header.frame_id = depth_optical_frame_;
    img.height = static_cast<uint32_t>(depth_f.height);
    img.width = static_cast<uint32_t>(depth_f.width);
    img.encoding = "16UC1";
    img.is_bigendian = 0;
    img.step = static_cast<uint32_t>(
      depth_f.stride > 0 ? depth_f.stride : depth_f.width * 2);
    const size_t nbytes = static_cast<size_t>(img.step) * img.height;
    img.data.resize(nbytes);
    std::memcpy(img.data.data(), depth_f.pixels, nbytes);

    auto info = depth_camera_info_;
    info.header = img.header;
    depth_pub_->publish(img);
    depth_info_pub_->publish(info);

    now = clock::now();
    do {
      next += period;
    } while (next <= now);
  }
}

}  // namespace composite_sensing
