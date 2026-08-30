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
  /// Livox-SDK2 JSON config path (empty → auto-generate Mid-360 template).
  std::string config_path;
  /// Livox frame aggregation wall-clock fallback (ms).
  int frame_time_ms{100};
  /// Livox: QueryLivoxLidarInternalInfo period (s). 0 disables.
  double temp_query_period_s{2.0};
  /// Livox: EnableLivoxLidarImuData + IMU packet queue.
  bool enable_imu{false};
};

/** One 6-DoF IMU sample. Accel is m/s^2, gyro rad/s, in the IMU frame. */
struct LidarImuSample
{
  int64_t timestamp_ns{0};
  float gx{0}, gy{0}, gz{0};
  float ax{0}, ay{0}, az{0};
};

/** Optional vendor telemetry (temps, etc.). Invalid fields have *_valid=false. */
struct LidarDeviceStatus
{
  bool core_temp_valid{false};
  float core_temp_c{0.f};
  bool environment_temp_valid{false};
  float environment_temp_c{0.f};
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

  /**
   * Livox: true → Normal (motor on), false → WakeUp/standby (motor off).
   * Used on shutdown; launch turns the motor on via InfoChange.
   */
  virtual bool set_scanning(bool /*enable*/) { return false; }

  /** Rate-limited vendor status query (no-op if unsupported). */
  virtual void request_status_update() {}

  virtual LidarDeviceStatus get_status() const { return {}; }

  /** Non-blocking drain of queued IMU samples (empty if unsupported / off). */
  virtual void drain_imu(std::vector<LidarImuSample> & /*out*/) {}
};

}  // namespace composite_sensing
