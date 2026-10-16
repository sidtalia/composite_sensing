#include "composite_sensing/mesh_pf_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>

#include "composite_sensing/constant_twist_mesh_pf_backend.hpp"
#include "composite_sensing/lidar_device.hpp"
#include "composite_sensing/micp_l_backend.hpp"

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

bool pointcloud_to_lidar_frame(
  const sensor_msgs::msg::PointCloud2 & msg,
  LidarFrame & out)
{
  out.points.clear();
  out.frame_id = msg.header.frame_id;
  out.timestamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
  if (msg.width == 0 || msg.height == 0) {
    return false;
  }
  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
    const bool has_i = msg.fields.end() !=
      std::find_if(msg.fields.begin(), msg.fields.end(),
        [](const auto & f) { return f.name == "intensity"; });
    const bool has_t = msg.fields.end() !=
      std::find_if(msg.fields.begin(), msg.fields.end(),
        [](const auto & f) { return f.name == "time"; });

    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> iter_i;
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> iter_t;
    if (has_i) {
      iter_i = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(
        msg, "intensity");
    }
    if (has_t) {
      iter_t = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(
        msg, "time");
    }

    const size_t n = static_cast<size_t>(msg.width) * msg.height;
    out.points.reserve(n);
    for (size_t i = 0; i < n; ++i, ++iter_x, ++iter_y, ++iter_z) {
      LidarPoint p;
      p.x = *iter_x;
      p.y = *iter_y;
      p.z = *iter_z;
      if (iter_i) {
        p.intensity = **iter_i;
        ++(*iter_i);
      }
      if (iter_t) {
        p.time_offset_s = **iter_t;
        ++(*iter_t);
      }
      out.points.push_back(p);
    }
  } catch (const std::exception &) {
    return false;
  }
  return !out.points.empty();
}

tf2::Transform stamped_to_tf(const geometry_msgs::msg::TransformStamped & ts)
{
  const auto & t = ts.transform;
  return tf2::Transform(
    tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
    tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
}

bool lookup_tf(
  tf2_ros::Buffer & buf,
  const std::string & parent,
  const std::string & child,
  const rclcpp::Time & stamp,
  geometry_msgs::msg::TransformStamped & out)
{
  try {
    out = buf.lookupTransform(
      parent, child, stamp, rclcpp::Duration::from_seconds(0.05));
    return true;
  } catch (const tf2::TransformException &) {
    try {
      out = buf.lookupTransform(parent, child, tf2::TimePointZero);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }
}
}  // namespace

MeshLocalizerConfig MeshPfNode::fill_localizer_config() const
{
  MeshLocalizerConfig mcfg;
  mcfg.map_file = map_file_;
  mcfg.map_frame = map_frame_;
  mcfg.base_frame = base_frame_;
  mcfg.t_base_lidar = {t_x_, t_y_, t_z_};
  mcfg.q_base_lidar = {q_x_, q_y_, q_z_, q_w_};
  mcfg.num_particles = num_particles_;
  mcfg.tracking_particles = tracking_particles_;
  mcfg.beam_samples = beam_samples_;
  mcfg.localize_hz = localize_hz_;
  mcfg.global_init_on_start = false;
  mcfg.init_bb = init_bb_;
  mcfg.init_roll = init_roll_;
  mcfg.init_pitch = init_pitch_;
  mcfg.init_yaw = init_yaw_;
  mcfg.still_trans_noise_m = still_trans_noise_m_;
  mcfg.still_rp_noise_rad = still_rp_noise_rad_;
  mcfg.still_yaw_noise_rad = still_yaw_noise_rad_;
  mcfg.dist_noise_gain = dist_noise_gain_;
  mcfg.rot_rp_noise_gain = rot_rp_noise_gain_;
  mcfg.rot_yaw_noise_gain = rot_yaw_noise_gain_;
  mcfg.tracking_still_trans_noise_m = tracking_still_trans_noise_m_;
  mcfg.tracking_still_rp_noise_rad = tracking_still_rp_noise_rad_;
  mcfg.tracking_still_yaw_noise_rad = tracking_still_yaw_noise_rad_;
  mcfg.tracking_dist_noise_gain = tracking_dist_noise_gain_;
  mcfg.tracking_rot_rp_noise_gain = tracking_rot_rp_noise_gain_;
  mcfg.tracking_rot_yaw_noise_gain = tracking_rot_yaw_noise_gain_;
  mcfg.tracking_motion = tracking_motion_;
  mcfg.raycast_backend = raycast_backend_;
  mcfg.tracking_backend = tracking_backend_;
  mcfg.micp_iterations = micp_iterations_;
  mcfg.micp_samples = micp_samples_;
  mcfg.micp_max_dist_m = micp_max_dist_m_;
  mcfg.micp_adaptive_max_dist_min_m = micp_adaptive_max_dist_min_m_;
  return mcfg;
}

void MeshPfNode::ensure_pf()
{
  if (pf_) {
    return;
  }
  pf_ = std::make_unique<ConstantTwistMeshPfBackend>();
  auto mcfg = fill_localizer_config();
  mcfg.global_init_on_start =
    global_init_on_start_ && !init_from_tf_ && !wait_for_aruco_hunt_;
  pf_->configure(mcfg);
}

void MeshPfNode::ensure_micp()
{
  if (micp_) {
    return;
  }
  micp_ = std::make_unique<MicpLBackend>();
  micp_->configure(fill_localizer_config());
}

MeshPfNode::MeshPfNode(const rclcpp::NodeOptions & options)
: Node("mesh_pf", options)
{
  declare_params();
  if (map_file_.empty()) {
    throw std::runtime_error("mesh_pf: map_file is required");
  }

  if (tracking_backend_ == "micp") {
    ensure_micp();
    active_ = micp_.get();
  } else {
    ensure_pf();
    active_ = pf_.get();
  }
  if (global_init_on_start_ && !init_from_tf_ && !wait_for_aruco_hunt_) {
    ensure_pf();
    pf_->global_init();
    active_ = pf_.get();
    seeded_ = true;
  }

  if (init_from_tf_ && init_tf_child_.empty()) {
    init_tf_child_ = odom_frame_;
  }
  if (wait_for_aruco_hunt_ || init_from_tf_) {
    seeded_ = false;
  }
  if (wait_for_aruco_hunt_) {
    waiting_for_hunt_ = true;
    if (active_) {
      active_->reset();
    }
  }

  if (publish_tf_ || init_from_tf_ || tracking_use_odom_tf_ || wait_for_aruco_hunt_) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  }
  if (publish_tf_) {
    tf_static_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
  }

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    pose_topic_, 10);
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&MeshPfNode::cloud_callback, this, std::placeholders::_1));

  global_srv_ = create_service<std_srvs::srv::Empty>(
    "~/global_localization",
    [this](
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      ensure_pf();
      pf_->global_init();
      active_ = pf_.get();
      seeded_ = true;
      waiting_for_hunt_ = false;
      ready_to_seed_ = false;
      saw_hunt_ = false;
      have_last_odom_ = false;
      RCLCPP_INFO(get_logger(), "mesh_pf: global_init() (particle filter)");
    });
  reset_srv_ = create_service<std_srvs::srv::Empty>(
    "~/reset",
    [this](
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      reset_wait();
    });

  const auto latch = rclcpp::QoS(1).transient_local().reliable();
  if (wait_for_aruco_hunt_) {
    trigger_sub_ = create_subscription<std_msgs::msg::Empty>(
      trigger_topic_, 10,
      std::bind(&MeshPfNode::on_trigger, this, std::placeholders::_1));
    hunting_sub_ = create_subscription<std_msgs::msg::Bool>(
      hunting_topic_, latch,
      std::bind(&MeshPfNode::on_hunting, this, std::placeholders::_1));
    map_odom_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      map_odom_pose_topic_, latch,
      std::bind(&MeshPfNode::on_map_odom_pose, this, std::placeholders::_1));
  }

  const std::string ray_name = micp_
    ? micp_->raycast_backend_used()
    : (pf_ ? pf_->raycast_backend_used() : "?");
  const size_t tris = micp_
    ? micp_->triangle_count()
    : (pf_ ? pf_->triangle_count() : 0ul);
  const std::string seed_note = init_from_tf_
    ? (" seed=" + init_tf_parent_ + "←" + init_tf_child_)
    : "";
  RCLCPP_INFO(
    get_logger(),
    "mesh_pf up: cloud=%s map=%s pose=%s hz=%.1f tracking_backend=%s "
    "particles=%d tracking=%d backend=%s triangles=%zu tf=%s→%s%s%s "
    "odom_tf=%s motion=%s%s",
    cloud_topic_.c_str(), map_file_.c_str(), pose_topic_.c_str(),
    localize_hz_, tracking_backend_.c_str(),
    num_particles_, tracking_particles_,
    ray_name.c_str(), tris,
    map_frame_.c_str(), odom_frame_.c_str(),
    publish_tf_ ? " /tf_static" : " (off)",
    seed_note.c_str(),
    tracking_use_odom_tf_ ? "on" : "off",
    tracking_motion_.c_str(),
    wait_for_aruco_hunt_ ? " wait_aruco" : "");
  if (tracking_motion_ == "still") {
    RCLCPP_WARN(
      get_logger(),
      "mesh_pf: tracking_motion=still — motion prior Δ=0, sample around "
      "last pose (process noise only). odom not used as a prior.");
  }
  if (debug_odom_drift_) {
    warn_debug_odom_drift();
  }
}

void MeshPfNode::declare_params()
{
  cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/livox/cloud");
  pose_topic_ = declare_parameter<std::string>(
    "pose_topic", "/localization/mesh_pose");
  map_file_ = declare_parameter<std::string>("map_file", "");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
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
  localize_hz_ = declare_parameter<double>("localize_hz", 10.0);
  num_particles_ = declare_parameter<int>("num_particles", 100000);
  tracking_particles_ = declare_parameter<int>("tracking_particles", 10000);
  beam_samples_ = declare_parameter<int>("beam_samples", 64);
  global_init_on_start_ = declare_parameter<bool>("global_init_on_start", true);
  raycast_backend_ = declare_parameter<std::string>("raycast_backend", "auto");
  init_bb_ = {
    declare_parameter<double>("init_bb.xmin", -20.0),
    declare_parameter<double>("init_bb.ymin", -20.0),
    declare_parameter<double>("init_bb.zmin", 0.0),
    declare_parameter<double>("init_bb.xmax", 20.0),
    declare_parameter<double>("init_bb.ymax", 20.0),
    declare_parameter<double>("init_bb.zmax", 2.0)};
  init_roll_ = {
    declare_parameter<double>("init_roll.min", -M_PI / 4.0),
    declare_parameter<double>("init_roll.max", M_PI / 4.0)};
  init_pitch_ = {
    declare_parameter<double>("init_pitch.min", -M_PI / 4.0),
    declare_parameter<double>("init_pitch.max", M_PI / 4.0)};
  init_yaw_ = {
    declare_parameter<double>("init_yaw.min", -3.14159265358979323846),
    declare_parameter<double>("init_yaw.max", 3.14159265358979323846)};
  init_from_tf_ = declare_parameter<bool>("init_from_tf", false);
  init_tf_parent_ = declare_parameter<std::string>("init_tf_parent", "map");
  init_tf_child_ = declare_parameter<std::string>("init_tf_child", "odom");
  init_tf_trans_std_m_ = declare_parameter<double>("init_tf_trans_std_m", 1.0);
  {
    const double deg = declare_parameter<double>("init_tf_rot_std_deg", 10.0);
    init_tf_rot_std_rad_ = deg * M_PI / 180.0;
  }
  init_tf_timeout_s_ = declare_parameter<double>("init_tf_timeout_s", 10.0);
  wait_for_aruco_hunt_ = declare_parameter<bool>("wait_for_aruco_hunt", false);
  trigger_topic_ = declare_parameter<std::string>(
    "trigger_topic", "/aruco_registration/trigger");
  hunting_topic_ = declare_parameter<std::string>(
    "hunting_topic", "/aruco_registration/hunting");
  map_odom_pose_topic_ = declare_parameter<std::string>(
    "map_odom_pose_topic", "/aruco_registration/map_odom");
  tracking_use_odom_tf_ = declare_parameter<bool>("tracking_use_odom_tf", true);
  tracking_motion_ = declare_parameter<std::string>("tracking_motion", "");
  for (char & c : tracking_motion_) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  if (tracking_motion_.empty()) {
    tracking_motion_ = tracking_use_odom_tf_ ? "odom" : "twist";
  }
  if (tracking_motion_ != "odom" && tracking_motion_ != "twist" &&
      tracking_motion_ != "still")
  {
    tracking_motion_ = tracking_use_odom_tf_ ? "odom" : "twist";
  }
  still_trans_noise_m_ = declare_parameter<double>("still_trans_noise_m", 0.05);
  dist_noise_gain_ = declare_parameter<double>("dist_noise_gain", 0.1);
  tracking_still_trans_noise_m_ =
    declare_parameter<double>("tracking_still_trans_noise_m", 0.01);
  tracking_dist_noise_gain_ =
    declare_parameter<double>("tracking_dist_noise_gain", 0.02);
  {
    const double kDeg = M_PI / 180.0;
    const double still_rot_deg =
      declare_parameter<double>("still_rot_noise_deg", 3.0);
    const double track_rot_deg =
      declare_parameter<double>("tracking_still_rot_noise_deg", 0.5);
    still_rp_noise_rad_ =
      declare_parameter<double>("still_rp_noise_deg", still_rot_deg) * kDeg;
    still_yaw_noise_rad_ =
      declare_parameter<double>("still_yaw_noise_deg", still_rot_deg) * kDeg;
    tracking_still_rp_noise_rad_ =
      declare_parameter<double>("tracking_still_rp_noise_deg", track_rot_deg) *
      kDeg;
    tracking_still_yaw_noise_rad_ =
      declare_parameter<double>("tracking_still_yaw_noise_deg", track_rot_deg) *
      kDeg;
    const double rot_gain = declare_parameter<double>("rot_noise_gain", 0.2);
    rot_rp_noise_gain_ = declare_parameter<double>("rot_rp_noise_gain", rot_gain);
    rot_yaw_noise_gain_ =
      declare_parameter<double>("rot_yaw_noise_gain", rot_gain);
    const double track_rot_gain =
      declare_parameter<double>("tracking_rot_noise_gain", 0.05);
    tracking_rot_rp_noise_gain_ =
      declare_parameter<double>("tracking_rot_rp_noise_gain", track_rot_gain);
    tracking_rot_yaw_noise_gain_ =
      declare_parameter<double>("tracking_rot_yaw_noise_gain", track_rot_gain);
  }
  tracking_backend_ = declare_parameter<std::string>("tracking_backend", "micp");
  for (char & c : tracking_backend_) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  if (tracking_backend_ != "micp" && tracking_backend_ != "pf") {
    tracking_backend_ = "micp";
  }
  micp_iterations_ = declare_parameter<int>("micp_iterations", 5);
  micp_samples_ = declare_parameter<int>("micp_samples", 1024);
  micp_max_dist_m_ = declare_parameter<double>("micp_max_dist_m", 0.5);
  micp_adaptive_max_dist_min_m_ =
    declare_parameter<double>("micp_adaptive_max_dist_min_m", 0.05);
  debug_odom_drift_ = declare_parameter<bool>("debug_odom_drift", false);
  debug_odom_drift_trans_std_m_ =
    declare_parameter<double>("debug_odom_drift_trans_std_m", 0.005);
  debug_odom_drift_z_std_m_ =
    declare_parameter<double>("debug_odom_drift_z_std_m", 0.001);
  {
    const double deg =
      declare_parameter<double>("debug_odom_drift_rot_std_deg", 0.05);
    debug_odom_drift_rot_std_rad_ = deg * M_PI / 180.0;
  }
}

void MeshPfNode::reset_wait()
{
  seeded_ = false;
  waiting_for_hunt_ = true;
  saw_hunt_ = false;
  ready_to_seed_ = false;
  seed_wait_started_ = false;
  have_last_odom_ = false;
  debug_odom_drift_tx_ = 0.0;
  debug_odom_drift_ty_ = 0.0;
  debug_odom_drift_tz_ = 0.0;
  debug_odom_drift_yaw_ = 0.0;
  if (active_) {
    active_->reset();
  }
  if (tracking_backend_ == "micp") {
    ensure_micp();
    active_ = micp_.get();
  }
  RCLCPP_INFO(
    get_logger(),
    "mesh_pf: reset — waiting for ArUco hunt on %s",
    hunting_topic_.c_str());
}

void MeshPfNode::on_trigger(const std_msgs::msg::Empty::SharedPtr)
{
  reset_wait();
}

void MeshPfNode::on_hunting(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (msg->data) {
    if (!waiting_for_hunt_) {
      reset_wait();
    }
    saw_hunt_ = true;
    RCLCPP_INFO(get_logger(), "mesh_pf: ArUco hunt started — holding");
    return;
  }
  if (!saw_hunt_) {
    return;
  }
  waiting_for_hunt_ = false;
  ready_to_seed_ = true;
  RCLCPP_INFO(get_logger(), "mesh_pf: ArUco hunt done — seeding from map←odom");
  try_seed(now());
}

void MeshPfNode::on_map_odom_pose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  last_map_odom_pose_ = tf2::Transform(
    tf2::Quaternion(
      msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w),
    tf2::Vector3(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
  have_map_odom_pose_ = true;
}

bool MeshPfNode::try_seed(const rclcpp::Time & stamp)
{
  if (try_seed_from_tf(stamp)) {
    return true;
  }
  return try_seed_from_pose_msg(stamp);
}

bool MeshPfNode::seed_from_map_odom(
  const tf2::Transform & T_map_odom, const rclcpp::Time & stamp)
{
  if (!tf_buffer_) {
    return false;
  }
  if (tracking_backend_ == "micp") {
    ensure_micp();
    active_ = micp_.get();
  } else {
    ensure_pf();
    active_ = pf_.get();
  }
  if (!active_) {
    return false;
  }
  geometry_msgs::msg::TransformStamped odom_base;
  if (!lookup_odom_base(stamp, odom_base)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "mesh_pf: waiting for %s←%s to compose seed pose",
      odom_frame_.c_str(), base_frame_.c_str());
    return false;
  }
  const tf2::Transform T_map_base = T_map_odom * stamped_to_tf(odom_base);
  const tf2::Vector3 p = T_map_base.getOrigin();
  const tf2::Quaternion q = T_map_base.getRotation();
  active_->seed_from_pose(
    {p.x(), p.y(), p.z()},
    {q.x(), q.y(), q.z(), q.w()},
    init_tf_trans_std_m_,
    init_tf_rot_std_rad_);
  seeded_ = true;
  waiting_for_hunt_ = false;
  ready_to_seed_ = false;
  have_last_odom_ = false;
  debug_odom_drift_tx_ = 0.0;
  debug_odom_drift_ty_ = 0.0;
  debug_odom_drift_tz_ = 0.0;
  debug_odom_drift_yaw_ = 0.0;
  RCLCPP_INFO(
    get_logger(),
    "mesh_pf: tracking seed (%s) from %s←%s ∘ %s←%s  base=(%.2f, %.2f, %.2f) "
    "σ_t=%.2fm σ_r=%.1fdeg",
    tracking_backend_.c_str(),
    init_tf_parent_.c_str(), init_tf_child_.c_str(),
    odom_frame_.c_str(), base_frame_.c_str(),
    p.x(), p.y(), p.z(),
    init_tf_trans_std_m_,
    init_tf_rot_std_rad_ * 180.0 / M_PI);
  return true;
}

bool MeshPfNode::try_seed_from_tf(const rclcpp::Time & stamp)
{
  if (!tf_buffer_) {
    return false;
  }
  geometry_msgs::msg::TransformStamped parent_child;
  if (!lookup_tf(
        *tf_buffer_, init_tf_parent_, init_tf_child_, stamp, parent_child))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "mesh_pf: waiting for seed TF %s←%s",
      init_tf_parent_.c_str(), init_tf_child_.c_str());
    return false;
  }
  return seed_from_map_odom(stamped_to_tf(parent_child), stamp);
}

bool MeshPfNode::try_seed_from_pose_msg(const rclcpp::Time & stamp)
{
  if (!have_map_odom_pose_) {
    return false;
  }
  return seed_from_map_odom(last_map_odom_pose_, stamp);
}

void MeshPfNode::warn_debug_odom_drift() const
{
  std::fprintf(
      stderr,
      "\033[1;31m[mesh_pf] DEBUG ODOM DRIFT ENABLED — integrating fake "
      "odom←base walk each localize "
      "(σ_xy=%.4f m  σ_z=%.4f m  σ_yaw=%.3f deg/step). "
      "NOT FOR REAL RUNS. accum xyz=(%.3f, %.3f, %.3f) yaw=%.2f deg\033[0m\n",
      debug_odom_drift_trans_std_m_, debug_odom_drift_z_std_m_,
      debug_odom_drift_rot_std_rad_ * 180.0 / M_PI,
      debug_odom_drift_tx_, debug_odom_drift_ty_, debug_odom_drift_tz_,
      debug_odom_drift_yaw_ * 180.0 / M_PI);
  RCLCPP_ERROR(
      get_logger(),
      "DEBUG ODOM DRIFT ENABLED — fake integrating odom←base "
      "(σ_xy=%.4fm σ_z=%.4fm σ_yaw=%.3fdeg/step)  NOT FOR REAL RUNS  "
      "accum xyz=(%.3f, %.3f, %.3f) yaw=%.2fdeg",
      debug_odom_drift_trans_std_m_, debug_odom_drift_z_std_m_,
      debug_odom_drift_rot_std_rad_ * 180.0 / M_PI,
      debug_odom_drift_tx_, debug_odom_drift_ty_, debug_odom_drift_tz_,
      debug_odom_drift_yaw_ * 180.0 / M_PI);
}

void MeshPfNode::step_debug_odom_drift()
{
  if (!debug_odom_drift_) {
    return;
  }
  std::normal_distribution<double> nxy(0.0, debug_odom_drift_trans_std_m_);
  std::normal_distribution<double> nz(0.0, debug_odom_drift_z_std_m_);
  std::normal_distribution<double> nyaw(0.0, debug_odom_drift_rot_std_rad_);
  debug_odom_drift_tx_ += nxy(debug_odom_rng_);
  debug_odom_drift_ty_ += nxy(debug_odom_rng_);
  debug_odom_drift_tz_ += nz(debug_odom_rng_);
  debug_odom_drift_yaw_ += nyaw(debug_odom_rng_);
  RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "\033[1;31mDEBUG ODOM DRIFT ON  accum xyz=(%.3f, %.3f, %.3f) m  "
      "yaw=%.2f deg — NOT FOR REAL RUNS\033[0m",
      debug_odom_drift_tx_, debug_odom_drift_ty_, debug_odom_drift_tz_,
      debug_odom_drift_yaw_ * 180.0 / M_PI);
}

void MeshPfNode::apply_debug_odom_drift(
  geometry_msgs::msg::TransformStamped * ts) const
{
  if (!debug_odom_drift_ || ts == nullptr) {
    return;
  }
  ts->transform.translation.x += debug_odom_drift_tx_;
  ts->transform.translation.y += debug_odom_drift_ty_;
  ts->transform.translation.z += debug_odom_drift_tz_;
  tf2::Quaternion q_yaw;
  q_yaw.setRPY(0.0, 0.0, debug_odom_drift_yaw_);
  const tf2::Quaternion q(
      ts->transform.rotation.x, ts->transform.rotation.y,
      ts->transform.rotation.z, ts->transform.rotation.w);
  const tf2::Quaternion qn = (q_yaw * q).normalized();
  ts->transform.rotation.x = qn.x();
  ts->transform.rotation.y = qn.y();
  ts->transform.rotation.z = qn.z();
  ts->transform.rotation.w = qn.w();
}

bool MeshPfNode::lookup_odom_base(
  const rclcpp::Time & stamp, geometry_msgs::msg::TransformStamped & out)
{
  if (!tf_buffer_) {
    return false;
  }
  const auto t0 = std::chrono::steady_clock::now();
  const bool ok = lookup_tf(*tf_buffer_, odom_frame_, base_frame_, stamp, out);
  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();
  if (ms > 10.0) {
    RCLCPP_WARN(
      get_logger(),
      "mesh_pf: TF %s←%s lookup %.1f ms%s",
      odom_frame_.c_str(), base_frame_.c_str(), ms,
      ok ? "" : " (failed)");
  }
  return ok;
}

OdomDelta MeshPfNode::lookup_odom_delta(const rclcpp::Time & stamp)
{
  OdomDelta out;
  if (tracking_motion_ != "odom" || !tf_buffer_) {
    return out;
  }
  geometry_msgs::msg::TransformStamped odom_base;
  if (!lookup_odom_base(stamp, odom_base)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "mesh_pf: no %s←%s for tracking motion",
      odom_frame_.c_str(), base_frame_.c_str());
    return out;
  }
  apply_debug_odom_drift(&odom_base);
  const tf2::Transform T_curr = stamped_to_tf(odom_base);
  if (have_last_odom_) {
    const tf2::Transform T_delta = last_odom_base_.inverse() * T_curr;
    const tf2::Vector3 p = T_delta.getOrigin();
    const tf2::Quaternion q = T_delta.getRotation();
    out.translation = {p.x(), p.y(), p.z()};
    out.rotation_xyzw = {q.x(), q.y(), q.z(), q.w()};
    out.valid = true;
  }
  last_odom_base_ = T_curr;
  have_last_odom_ = true;
  return out;
}

void MeshPfNode::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!active_ || !msg) {
    return;
  }

  if (waiting_for_hunt_) {
    return;
  }

  if (!seeded_ && (init_from_tf_ || ready_to_seed_)) {
    if (wait_for_aruco_hunt_ && !ready_to_seed_) {
      return;
    }
    if (!seed_wait_started_) {
      seed_wait_start_ = std::chrono::steady_clock::now();
      seed_wait_started_ = true;
    }
    const rclcpp::Time stamp(msg->header.stamp);
    if (!try_seed(stamp)) {
      const bool allow_bbox =
        !wait_for_aruco_hunt_ && init_tf_timeout_s_ > 0.0;
      const double waited = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - seed_wait_start_).count();
      if (allow_bbox && waited >= init_tf_timeout_s_) {
        ensure_pf();
        pf_->global_init();
        active_ = pf_.get();
        seeded_ = true;
        have_last_odom_ = false;
        RCLCPP_WARN(
          get_logger(),
          "mesh_pf: no %s←%s after %.1fs, falling back to init_bb",
          init_tf_parent_.c_str(), init_tf_child_.c_str(), waited);
      } else {
        return;
      }
    }
  }

  const double min_dt = 1.0 / std::max(1.0, localize_hz_);
  {
    std::lock_guard<std::mutex> lock(rate_mu_);
    const auto stamp = rclcpp::Time(msg->header.stamp);
    if (last_localize_time_.nanoseconds() > 0 &&
        (stamp - last_localize_time_).seconds() < min_dt)
    {
      return;
    }
    last_localize_time_ = stamp;
  }

  LidarFrame cloud;
  if (!pointcloud_to_lidar_frame(*msg, cloud)) {
    return;
  }
  step_debug_odom_drift();
  const OdomDelta odom = lookup_odom_delta(rclcpp::Time(msg->header.stamp));
  auto pose = active_->localize(cloud, odom);
  if (const auto * micp = dynamic_cast<MicpLBackend *>(active_)) {
    const auto t = micp->last_timing();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "mesh_pf infer: mode=micp odom_tf=%d inliers=%zu/%zu iter=%d "
      "raycast=%.2fms fill=%.2fms solve=%.2fms localize=%.2fms backend=%s",
      odom.valid ? 1 : 0, t.n_inliers,
      t.iterations > 0 ? t.n_rays / static_cast<size_t>(t.iterations) : t.n_rays,
      t.iterations, t.raycast_ms, t.fill_ms, t.solve_ms, t.localize_ms,
      micp->raycast_backend_used().c_str());
  } else if (const auto * pf_backend =
               dynamic_cast<ConstantTwistMeshPfBackend *>(active_))
  {
    const auto t = pf_backend->last_timing();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "mesh_pf infer: mode=%s particles=%zu odom_tf=%d raycast=%.2fms "
      "fill=%.2fms weight=%.2fms localize=%.2fms rays=%zu backend=%s",
      pf_backend->tracking() ? "tracking" : "global",
      pf_backend->particle_count(),
      odom.valid ? 1 : 0,
      t.raycast_ms, t.fill_ms, t.weight_ms, t.localize_ms, t.n_rays,
      pf_backend->raycast_backend_used().c_str());
  }
  if (pose && pose->valid) {
    const rclcpp::Time stamp(msg->header.stamp);
    publish_pose(*pose, stamp);
    publish_map_odom(*pose, stamp);
  }
}

void MeshPfNode::publish_pose(const PoseMap & pose, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = map_frame_;
  msg.pose.pose.position.x = pose.translation[0];
  msg.pose.pose.position.y = pose.translation[1];
  msg.pose.pose.position.z = pose.translation[2];
  msg.pose.pose.orientation.x = pose.rotation[0];
  msg.pose.pose.orientation.y = pose.rotation[1];
  msg.pose.pose.orientation.z = pose.rotation[2];
  msg.pose.pose.orientation.w = pose.rotation[3];
  for (int i = 0; i < 36; ++i) {
    msg.pose.covariance[i] = pose.covariance[static_cast<size_t>(i)];
  }
  pose_pub_->publish(msg);
}

void MeshPfNode::publish_map_odom(const PoseMap & pose, const rclcpp::Time & stamp)
{
  if (!publish_tf_ || !seeded_ || waiting_for_hunt_ || !tf_buffer_ || !tf_static_) {
    return;
  }
  geometry_msgs::msg::TransformStamped odom_base;
  if (!lookup_odom_base(stamp, odom_base)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "mesh_pf: no %s←%s for map←odom",
      odom_frame_.c_str(), base_frame_.c_str());
    return;
  }
  apply_debug_odom_drift(&odom_base);

  const tf2::Transform T_map_base(
    tf2::Quaternion(
      pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3]),
    tf2::Vector3(
      pose.translation[0], pose.translation[1], pose.translation[2]));
  const auto & t = odom_base.transform;
  const tf2::Transform T_odom_base(
    tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
    tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
  const tf2::Transform T_map_odom = T_map_base * T_odom_base.inverse();

  geometry_msgs::msg::TransformStamped out;
  out.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  out.header.frame_id = map_frame_;
  out.child_frame_id = odom_frame_;
  const tf2::Vector3 p = T_map_odom.getOrigin();
  const tf2::Quaternion q = T_map_odom.getRotation();
  out.transform.translation.x = p.x();
  out.transform.translation.y = p.y();
  out.transform.translation.z = p.z();
  out.transform.rotation.x = q.x();
  out.transform.rotation.y = q.y();
  out.transform.rotation.z = q.z();
  out.transform.rotation.w = q.w();
  tf_static_->sendTransform(out);
}

}  // namespace composite_sensing
