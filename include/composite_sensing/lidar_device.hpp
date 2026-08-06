#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace composite_sensing
{

/** One lidar return in the sensor frame (FLU-ish / vendor native). */
struct LidarPoint
{
  float x{0}, y{0}, z{0};
  float intensity{0};
  float time_offset_s{0};
  uint32_t ring{0};
};

/**
 * Contiguous point cloud. Points remain valid until the next wait_cloud()/stop()
 * on the owning device.
 */
struct LidarFrame
{
  std::vector<LidarPoint> points;
  int64_t timestamp_ns{0};
  uint32_t seq{0};
  std::string frame_id;
};

struct LidarOpenConfig
{
  std::string backend{"unitree"};  // unitree | ouster | livox (adapters)
  std::string port{"/dev/ttyUSB0"};
  uint32_t baudrate{2000000};
  uint16_t cloud_scan_num{18};
  float rotate_yaw_bias{0.f};
  float range_scale{0.001f};
  float range_bias{0.f};
  float range_max{50.f};
  float range_min{0.4f};
  std::string frame_id{"unilidar_lidar"};
  // UDP / vendor-specific (ignored by serial Unitree)
  std::string lidar_ip;
  std::string local_ip;
  uint16_t lidar_port{0};
  uint16_t local_port{0};
};

/**
 * Transport-agnostic lidar source. Implementations own the vendor SDK;
 * returned LidarFrame points are copied into the frame buffer.
 */
class LidarDevice
{
public:
  virtual ~LidarDevice() = default;

  virtual void open(const LidarOpenConfig & config) = 0;

  /** Block up to timeout_ms for a full cloud. Returns false on timeout. */
  virtual bool wait_cloud(LidarFrame & out, int timeout_ms) = 0;

  virtual void stop() = 0;
};

}  // namespace composite_sensing
