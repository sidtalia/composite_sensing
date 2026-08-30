#include "composite_sensing/mesh_pf_node.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>

#include "composite_sensing/constant_twist_mesh_pf_backend.hpp"
#include "composite_sensing/lidar_device.hpp"

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
}  // namespace

MeshPfNode::MeshPfNode(const rclcpp::NodeOptions & options)
: Node("mesh_pf", options)
{
  declare_params();
  if (map_file_.empty()) {
    throw std::runtime_error("mesh_pf: map_file is required");
  }

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
  mcfg.raycast_backend = raycast_backend_;
  pf->configure(mcfg);
  localizer_ = std::move(pf);

  if (publish_tf_) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
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
      if (localizer_) {
        localizer_->global_init();
        RCLCPP_INFO(get_logger(), "mesh_pf: global_init()");
      }
    });

  const auto * pf_backend =
    dynamic_cast<ConstantTwistMeshPfBackend *>(localizer_.get());
  RCLCPP_INFO(
    get_logger(),
    "mesh_pf up: cloud=%s map=%s pose=%s hz=%.1f particles=%d backend=%s "
    "triangles=%zu tf=%s→%s%s",
    cloud_topic_.c_str(), map_file_.c_str(), pose_topic_.c_str(),
    localize_hz_, num_particles_,
    pf_backend ? pf_backend->raycast_backend_used().c_str() : "?",
    pf_backend ? pf_backend->triangle_count() : 0ul,
    map_frame_.c_str(), odom_frame_.c_str(),
    publish_tf_ ? " /tf_static" : " (off)");
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
  num_particles_ = declare_parameter<int>("num_particles", 2000);
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
}

void MeshPfNode::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!localizer_ || !msg) {
    return;
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
  auto pose = localizer_->localize(cloud);
  if (const auto * pf_backend =
        dynamic_cast<ConstantTwistMeshPfBackend *>(localizer_.get()))
  {
    const auto t = pf_backend->last_timing();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "mesh_pf infer: raycast=%.2fms fill=%.2fms weight=%.2fms "
      "localize=%.2fms rays=%zu backend=%s",
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
  const double v = pose.position_stddev_m * pose.position_stddev_m;
  for (int i = 0; i < 36; ++i) {
    msg.pose.covariance[i] = 0.0;
  }
  msg.pose.covariance[0] = v;
  msg.pose.covariance[7] = v;
  msg.pose.covariance[14] = v;
  msg.pose.covariance[35] = 0.25;
  pose_pub_->publish(msg);
}

void MeshPfNode::publish_map_odom(const PoseMap & pose, const rclcpp::Time & stamp)
{
  if (!publish_tf_ || !tf_buffer_ || !tf_static_) {
    return;
  }
  geometry_msgs::msg::TransformStamped odom_base;
  try {
    odom_base = tf_buffer_->lookupTransform(
      odom_frame_, base_frame_, stamp, rclcpp::Duration::from_seconds(0.05));
  } catch (const tf2::TransformException &) {
    try {
      odom_base = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "mesh_pf: no %s←%s for map←odom (%s)",
        odom_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }
  }

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
