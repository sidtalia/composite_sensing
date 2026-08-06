#include "composite_sensing/unilidar_lidar_device.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

#include <unitree_lidar_sdk.h>

namespace composite_sensing
{

UnilidarLidarDevice::UnilidarLidarDevice() = default;

UnilidarLidarDevice::~UnilidarLidarDevice()
{
  stop();
}

void UnilidarLidarDevice::open(const LidarOpenConfig & config)
{
  stop();
  cfg_ = config;
  reader_ = unitree_lidar_sdk::createUnitreeLidarReader();
  if (!reader_) {
    throw std::runtime_error("UnilidarLidarDevice: createUnitreeLidarReader failed");
  }

  int rc = 0;
  if (!cfg_.lidar_ip.empty()) {
    rc = reader_->initializeUDP(
      cfg_.cloud_scan_num,
      cfg_.lidar_port ? cfg_.lidar_port : 5001,
      cfg_.lidar_ip,
      cfg_.local_port ? cfg_.local_port : 5000,
      cfg_.local_ip.empty() ? "10.10.10.100" : cfg_.local_ip,
      cfg_.rotate_yaw_bias,
      cfg_.range_scale,
      cfg_.range_bias,
      cfg_.range_max,
      cfg_.range_min);
  } else {
    rc = reader_->initialize(
      cfg_.cloud_scan_num,
      cfg_.port,
      cfg_.baudrate,
      cfg_.rotate_yaw_bias,
      cfg_.range_scale,
      cfg_.range_bias,
      cfg_.range_max,
      cfg_.range_min);
  }
  if (rc != 0) {
    throw std::runtime_error(
      "UnilidarLidarDevice: failed to open " +
      (cfg_.lidar_ip.empty() ? cfg_.port : cfg_.lidar_ip));
  }
  reader_->setLidarWorkingMode(unitree_lidar_sdk::NORMAL);
  opened_ = true;
}

bool UnilidarLidarDevice::wait_cloud(LidarFrame & out, int timeout_ms)
{
  if (!opened_ || !reader_) {
    return false;
  }
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto msg = reader_->runParse();
    if (msg == unitree_lidar_sdk::POINTCLOUD) {
      const auto & cloud = reader_->getCloud();
      out.points.clear();
      out.points.reserve(cloud.points.size());
      for (const auto & p : cloud.points) {
        const float r2 = p.x * p.x + p.y * p.y + p.z * p.z;
        const float rmin = cfg_.range_min;
        const float rmax = cfg_.range_max;
        if (r2 < rmin * rmin || r2 > rmax * rmax) {
          continue;
        }
        LidarPoint q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        q.intensity = p.intensity;
        q.time_offset_s = p.time;
        q.ring = p.ring;
        out.points.push_back(q);
      }
      out.timestamp_ns = static_cast<int64_t>(cloud.stamp * 1e9);
      out.seq = cloud.id;
      out.frame_id = cfg_.frame_id;
      return !out.points.empty();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

void UnilidarLidarDevice::stop()
{
  if (reader_ && opened_ && !cfg_.lidar_ip.empty()) {
    reader_->closeUDP();
  }
  // SDK has no serial close; drop pointer (SDK owns lifetime of create*).
  reader_ = nullptr;
  opened_ = false;
}

}  // namespace composite_sensing
