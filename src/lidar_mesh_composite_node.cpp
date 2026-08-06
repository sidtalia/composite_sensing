#include "composite_sensing/lidar_mesh_composite_node.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "composite_sensing/constant_twist_mesh_pf_backend.hpp"
#include "composite_sensing/unilidar_lidar_device.hpp"

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
  // Allow explicit quaternion override
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

  map_file_ = declare_parameter<std::string>("map_file", "");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  pose_topic_ = declare_parameter<std::string>(
    "pose_topic", "/localization/mesh_pose");
  localize_hz_ = declare_parameter<double>("localize_hz", 10.0);
  num_particles_ = declare_parameter<int>("num_particles", 2000);
  beam_samples_ = declare_parameter<int>("beam_samples", 64);
  global_init_on_start_ = declare_parameter<bool>("global_init_on_start", true);
  init_bb_ = {
    declare_parameter<double>("init_bb.xmin", -20.0),
    declare_parameter<double>("init_bb.ymin", -20.0),
    declare_parameter<double>("init_bb.zmin", 0.0),
    declare_parameter<double>("init_bb.xmax", 20.0),
    declare_parameter<double>("init_bb.ymax", 20.0),
    declare_parameter<double>("init_bb.zmax", 2.0)};
}

void LidarMeshCompositeNode::start_pipeline()
{
  if (lidar_backend_ == "unitree") {
    lidar_ = std::make_unique<UnilidarLidarDevice>();
  } else if (lidar_backend_ == "ouster" || lidar_backend_ == "livox") {
    throw std::runtime_error(
      "lidar_backend=" + lidar_backend_ +
      " adapter not implemented yet (unitree works)");
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
  lidar_->open(lcfg);

  auto pf = std::make_unique<ConstantTwistMeshPfBackend>();
  MeshLocalizerConfig mcfg;
  mcfg.map_file = map_file_;
  mcfg.map_frame = map_frame_;
  mcfg.base_frame = base_frame_;
  mcfg.t_base_lidar = {t_x_, t_y_, t_z_};
  mcfg.q_base_lidar = {q_x_, q_y_, q_z_, q_w_};
  mcfg.num_particles = num_particles_;
  mcfg.beam_samples = beam_samples_;
  mcfg.localize_hz = localize_hz_;
  mcfg.global_init_on_start = global_init_on_start_;
  mcfg.init_bb = init_bb_;
  if (!map_file_.empty()) {
    pf->configure(mcfg);
    localizer_ = std::move(pf);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "map_file empty — lidar cloud only (mesh PF disabled)");
  }

  static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
  publish_static_lidar_tf();

  if (publish_cloud_) {
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 2);
  }
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    pose_topic_, 10);

  global_srv_ = create_service<std_srvs::srv::Empty>(
    "~/global_localization",
    [this](
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      if (localizer_) {
        localizer_->global_init();
        RCLCPP_INFO(get_logger(), "mesh PF: global_init()");
      }
    });

  running_.store(true);
  capture_thread_ = std::thread([this]() {capture_loop();});

  RCLCPP_INFO(
    get_logger(),
    "lidar_mesh_composite up: backend=%s port=%s map=%s localize_hz=%.1f particles=%d",
    lidar_backend_.c_str(), port_.c_str(),
    map_file_.empty() ? "(none)" : map_file_.c_str(),
    localize_hz_, num_particles_);
}

void LidarMeshCompositeNode::publish_static_lidar_tf()
{
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
  static_tf_->sendTransform(tf);
}

void LidarMeshCompositeNode::publish_cloud(const LidarFrame & cloud)
{
  if (!cloud_pub_) {
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
  mod.setPointCloud2FieldsByString(2, "xyz", "intensity");
  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_i(msg, "intensity");
  for (const auto & p : cloud.points) {
    *iter_x = p.x;
    *iter_y = p.y;
    *iter_z = p.z;
    *iter_i = p.intensity;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_i;
  }
  cloud_pub_->publish(msg);
}

void LidarMeshCompositeNode::publish_pose(
  const PoseMap & pose, const rclcpp::Time & stamp)
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
  const double v = pose.position_stddev_m * pose.position_stddev_m;
  for (int i = 0; i < 36; ++i) {
    msg.pose.covariance[i] = 0.0;
  }
  msg.pose.covariance[0] = v;
  msg.pose.covariance[7] = v;
  msg.pose.covariance[14] = v;
  msg.pose.covariance[35] = 0.25;  // yaw var (rad^2) conservative
  pose_pub_->publish(msg);
}

void LidarMeshCompositeNode::capture_loop()
{
  const auto min_period = std::chrono::duration<double>(
    1.0 / std::max(1.0, localize_hz_));
  while (running_.load(std::memory_order_relaxed)) {
    LidarFrame cloud;
    if (!lidar_->wait_cloud(cloud, 200)) {
      continue;
    }
    if (publish_cloud_) {
      publish_cloud(cloud);
    }
    if (!localizer_) {
      continue;
    }

    const auto now_tp = std::chrono::steady_clock::now();
    static auto last_loc = now_tp - min_period;
    if (now_tp - last_loc < min_period) {
      continue;
    }
    last_loc = now_tp;

    auto pose = localizer_->localize(cloud);
    if (pose && pose->valid) {
      const rclcpp::Time stamp(
        cloud.timestamp_ns > 0 ? cloud.timestamp_ns : now().nanoseconds());
      publish_pose(*pose, stamp);
    }
  }
}

}  // namespace composite_sensing
