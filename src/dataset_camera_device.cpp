#include "composite_sensing/dataset_camera_device.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "composite_sensing/frame_conventions.hpp"

namespace composite_sensing {
namespace {

sensor_msgs::msg::CameraInfo camera_info_from_pinhole(
  const CameraIntrinsics & intr, const std::string & frame_id)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = static_cast<uint32_t>(intr.width);
  info.height = static_cast<uint32_t>(intr.height);
  info.header.frame_id = frame_id;
  info.distortion_model = "plumb_bob";
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  info.k = {
    intr.fx, 0.0, intr.cx,
    0.0, intr.fy, intr.cy,
    0.0, 0.0, 1.0};
  info.r = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  info.p = {
    intr.fx, 0.0, intr.cx, 0.0,
    0.0, intr.fy, intr.cy, 0.0,
    0.0, 0.0, 1.0, 0.0};
  return info;
}

geometry_msgs::msg::TransformStamped make_tf(
  const std::string & parent,
  const std::string & child,
  double qx, double qy, double qz, double qw)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;
  tf.transform.rotation.x = qx;
  tf.transform.rotation.y = qy;
  tf.transform.rotation.z = qz;
  tf.transform.rotation.w = qw;
  return tf;
}

bool read_T_wo_bin(const std::string & path, PoseOptical & out)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    return false;
  }
  double T[16];
  f.read(reinterpret_cast<char *>(T), sizeof(T));
  if (!f) {
    return false;
  }
  // Row-major 4x4 world←optical.
  std::array<double, 9> R = {
    T[0], T[1], T[2],
    T[4], T[5], T[6],
    T[8], T[9], T[10]};
  std::array<double, 4> q{};
  rotation_matrix_to_quat_xyzw(R, q);
  out.translation = {
    static_cast<float>(T[3]),
    static_cast<float>(T[7]),
    static_cast<float>(T[11])};
  out.rotation = {
    static_cast<float>(q[0]),
    static_cast<float>(q[1]),
    static_cast<float>(q[2]),
    static_cast<float>(q[3])};
  return true;
}

bool read_poses_body_bin(const std::string & path, int index, PoseOptical & out)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    return false;
  }
  f.seekg(static_cast<std::streamoff>(index) * 16 * 8);
  double T[16];
  f.read(reinterpret_cast<char *>(T), sizeof(T));
  if (!f) {
    return false;
  }
  std::array<double, 9> R = {
    T[0], T[1], T[2],
    T[4], T[5], T[6],
    T[8], T[9], T[10]};
  std::array<double, 4> q{};
  rotation_matrix_to_quat_xyzw(R, q);
  out.translation = {
    static_cast<float>(T[3]),
    static_cast<float>(T[7]),
    static_cast<float>(T[11])};
  out.rotation = {
    static_cast<float>(q[0]),
    static_cast<float>(q[1]),
    static_cast<float>(q[2]),
    static_cast<float>(q[3])};
  return true;
}

}  // namespace

DatasetCameraDevice::~DatasetCameraDevice()
{
  stop();
}

DatasetCameraDevice::Kv DatasetCameraDevice::load_camera_txt(const std::string & path)
{
  Kv kv;
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error("DatasetCameraDevice: missing camera.txt at " + path);
  }
  std::string line;
  while (std::getline(f, line)) {
    const auto eq = line.find('=');
    if (eq == std::string::npos) {
      continue;
    }
    const std::string key = line.substr(0, eq);
    const std::string val = line.substr(eq + 1);
    if (key == "width") {
      kv.width = std::stoi(val);
    } else if (key == "height") {
      kv.height = std::stoi(val);
    } else if (key == "fx") {
      kv.fx = std::stof(val);
    } else if (key == "fy") {
      kv.fy = std::stof(val);
    } else if (key == "cx") {
      kv.cx = std::stof(val);
    } else if (key == "cy") {
      kv.cy = std::stof(val);
    } else if (key == "n_frames") {
      kv.n_frames = std::stoi(val);
    }
  }
  if (kv.n_frames <= 0) {
    throw std::runtime_error("DatasetCameraDevice: n_frames <= 0 in camera.txt");
  }
  return kv;
}

void DatasetCameraDevice::open(const CameraOpenConfig & config)
{
  stop();
  if (config.dataset_path.empty()) {
    throw std::runtime_error("DatasetCameraDevice: dataset_path is empty");
  }
  config_ = config;
  kv_ = load_camera_txt(config_.dataset_path + "/camera.txt");

  caps_ = {};
  caps_.stereo = true;  // always paces via wait_stereo (real or synthetic IR)
  caps_.color = config_.enable_color;
  caps_.depth = config_.enable_depth;
  caps_.aligned_depth = config_.enable_depth;  // dataset depth is camera optical

  fill_stereo_cal();
  fill_camera_infos();

  index_ = 0;
  stereo_frame_id_ = 0;
  color_pending_ = false;
  depth_pending_ = false;
  has_gt_ = false;
  next_frame_deadline_ = std::chrono::steady_clock::now();
  open_ = true;
}

void DatasetCameraDevice::fill_stereo_cal()
{
  CameraIntrinsics intr;
  intr.width = kv_.width;
  intr.height = kv_.height;
  intr.fx = kv_.fx;
  intr.fy = kv_.fy;
  intr.cx = kv_.cx;
  intr.cy = kv_.cy;
  stereo_cal_.left = intr;
  stereo_cal_.right = intr;
  stereo_cal_.right_rotation_colmajor = {
    1.f, 0.f, 0.f,
    0.f, 1.f, 0.f,
    0.f, 0.f, 1.f};
  stereo_cal_.right_translation = {config_.dataset_baseline_m, 0.f, 0.f};
}

void DatasetCameraDevice::fill_camera_infos()
{
  color_camera_info_ = camera_info_from_pinhole(stereo_cal_.left, "color_optical_frame");
  depth_camera_info_ = camera_info_from_pinhole(stereo_cal_.left, "depth_optical_frame");
}

StreamCaps DatasetCameraDevice::caps() const
{
  return caps_;
}

StereoCalibration DatasetCameraDevice::stereo_calibration() const
{
  return stereo_cal_;
}

const sensor_msgs::msg::CameraInfo & DatasetCameraDevice::color_camera_info() const
{
  return color_camera_info_;
}

const sensor_msgs::msg::CameraInfo & DatasetCameraDevice::depth_camera_info() const
{
  return depth_camera_info_;
}

std::vector<geometry_msgs::msg::TransformStamped>
DatasetCameraDevice::static_camera_tfs(const std::string & camera_name) const
{
  // Smoke-rail parity: dataset depth/color are optical-frame; T_wo is
  // world←optical. Keep link→*_optical identity so TF(odom←optical)=T_wo
  // when odom publishes raw T_wo (no FLU remapping).
  const std::string link = camera_name + "_link";

  std::vector<geometry_msgs::msg::TransformStamped> tfs;
  auto add_stream = [&](const std::string & stream) {
    const std::string frame = camera_name + "_" + stream + "_frame";
    const std::string optical = camera_name + "_" + stream + "_optical_frame";
    tfs.push_back(make_tf(link, frame, 0, 0, 0, 1));
    tfs.push_back(make_tf(frame, optical, 0, 0, 0, 1));
  };

  add_stream("infra1");
  add_stream("infra2");
  if (caps_.color) {
    add_stream("color");
  }
  if (caps_.depth) {
    add_stream("depth");
  }
  return tfs;
}

std::string DatasetCameraDevice::frame_stem(int index) const
{
  std::ostringstream ss;
  ss << config_.dataset_path << "/frames/" << std::setw(6) << std::setfill('0')
     << index;
  if (!config_.dataset_view.empty()) {
    ss << "_" << config_.dataset_view;
  }
  return ss.str();
}

bool DatasetCameraDevice::wait_stereo(StereoFrame & out, int timeout_ms)
{
  if (!open_) {
    return false;
  }

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(
      std::max(0, timeout_ms));
  auto next_deadline = next_frame_deadline_;
  if (next_deadline > std::chrono::steady_clock::now()) {
    const auto sleep_until = std::min(next_deadline, deadline);
    std::this_thread::sleep_until(sleep_until);
    if (std::chrono::steady_clock::now() < next_deadline) {
      return false;
    }
  }

  int try_index = index_;
  for (int attempt = 0; attempt < kv_.n_frames; ++attempt) {
    if (try_index >= kv_.n_frames) {
      if (!config_.dataset_loop) {
        return false;
      }
      try_index = 0;
    }

    // Decode off the poll mutex (I/O), then publish under lock.
    std::vector<uint8_t> color, depth, ir_l, ir_r;
    int w = 0, h = 0;
    bool has_gt = false;
    PoseOptical gt{};
    {
      // Temporary members via load into locals: use a nested scope with lock-free
      // decode into scratch using OpenCV, then assign.
      const std::string stem = frame_stem(try_index);
      cv::Mat depth_m = cv::imread(stem + "_depth_mm.png", cv::IMREAD_UNCHANGED);
      cv::Mat bgr = cv::imread(stem + "_trav.png", cv::IMREAD_COLOR);
      if (depth_m.empty() || bgr.empty() || depth_m.type() != CV_16UC1 ||
        depth_m.cols != bgr.cols || depth_m.rows != bgr.rows)
      {
        ++try_index;
        continue;
      }
      w = depth_m.cols;
      h = depth_m.rows;
      cv::Mat rgb;
      cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
      if (!rgb.isContinuous()) {
        rgb = rgb.clone();
      }
      color.assign(rgb.datastart, rgb.dataend);
      if (!depth_m.isContinuous()) {
        depth_m = depth_m.clone();
      }
      depth.assign(
        depth_m.datastart,
        depth_m.datastart + static_cast<std::ptrdiff_t>(w) * h * 2);

      cv::Mat ir_lm = cv::imread(stem + "_ir_left.png", cv::IMREAD_GRAYSCALE);
      cv::Mat ir_rm = cv::imread(stem + "_ir_right.png", cv::IMREAD_GRAYSCALE);
      if (!ir_lm.empty() && !ir_rm.empty() &&
        ir_lm.cols == w && ir_lm.rows == h &&
        ir_rm.cols == w && ir_rm.rows == h)
      {
        if (!ir_lm.isContinuous()) {
          ir_lm = ir_lm.clone();
        }
        if (!ir_rm.isContinuous()) {
          ir_rm = ir_rm.clone();
        }
        ir_l.assign(ir_lm.datastart, ir_lm.dataend);
        ir_r.assign(ir_rm.datastart, ir_rm.dataend);
      } else {
        ir_l.resize(static_cast<size_t>(w * h));
        ir_r.resize(static_cast<size_t>(w * h));
        for (int i = 0; i < w * h; ++i) {
          const size_t o = static_cast<size_t>(i) * 3;
          const uint8_t y = static_cast<uint8_t>(
            (77 * color[o] + 150 * color[o + 1] + 29 * color[o + 2]) >> 8);
          ir_l[static_cast<size_t>(i)] = y;
          ir_r[static_cast<size_t>(i)] = y;
        }
      }

      const std::string body_bin = config_.dataset_path + "/frames/poses_body.bin";
      const std::string legacy_bin = config_.dataset_path + "/frames/poses_T_wo.bin";
      if (read_poses_body_bin(body_bin, try_index, gt)) {
        has_gt = true;
      } else if (read_poses_body_bin(legacy_bin, try_index, gt)) {
        has_gt = true;
      } else if (read_T_wo_bin(stem + "_T_wo.bin", gt)) {
        has_gt = true;
      }
    }

    const double hz = std::max(0.1, config_.dataset_rate_hz);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / hz));

    {
      std::lock_guard<std::mutex> lock(mu_);
      color_rgb_ = std::move(color);
      depth_z16_ = std::move(depth);
      ir_left_ = std::move(ir_l);
      ir_right_ = std::move(ir_r);
      img_w_ = w;
      img_h_ = h;
      has_gt_ = has_gt;
      gt_pose_ = gt;
      next_frame_deadline_ = std::chrono::steady_clock::now() + period;
      index_ = try_index + 1;
      ++stereo_frame_id_;
      timestamp_ns_ = static_cast<int64_t>(stereo_frame_id_) *
        (1000000000LL / std::max(1, static_cast<int>(std::lround(hz))));
      color_pending_ = caps_.color;
      depth_pending_ = caps_.depth;

      out.left_pixels = ir_left_.data();
      out.right_pixels = ir_right_.data();
      out.width = img_w_;
      out.height = img_h_;
      out.stride = img_w_;
      out.timestamp_ns = timestamp_ns_;
      out.arrival_host_ns = 0;
    }
    return true;
  }
  return false;
}

bool DatasetCameraDevice::poll_color(ColorFrame & out)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!color_pending_ || color_rgb_.empty()) {
    return false;
  }
  color_pending_ = false;
  out.pixels = color_rgb_.data();
  out.width = img_w_;
  out.height = img_h_;
  out.stride = img_w_ * 3;
  out.timestamp_ns = timestamp_ns_;
  return true;
}

bool DatasetCameraDevice::poll_depth(DepthFrame & out)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!depth_pending_ || depth_z16_.empty()) {
    return false;
  }
  depth_pending_ = false;
  out.pixels = depth_z16_.data();
  out.width = img_w_;
  out.height = img_h_;
  out.stride = img_w_ * 2;
  out.timestamp_ns = timestamp_ns_;
  return true;
}

bool DatasetCameraDevice::gt_pose_optical(PoseOptical & out) const
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!has_gt_) {
    return false;
  }
  out = gt_pose_;
  return true;
}

void DatasetCameraDevice::stop()
{
  std::lock_guard<std::mutex> lock(mu_);
  open_ = false;
  color_pending_ = false;
  depth_pending_ = false;
  has_gt_ = false;
  color_rgb_.clear();
  depth_z16_.clear();
  ir_left_.clear();
  ir_right_.clear();
  caps_ = {};
}

}  // namespace composite_sensing
