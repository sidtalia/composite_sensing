#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "composite_sensing/lidar_device.hpp"

#include <livox_lidar_def.h>

namespace composite_sensing
{

/**
 * Livox Mid-360 / Mid-360S via Livox-SDK2.
 * Aggregates Cartesian packets into LidarFrame primarily by frame_time_ms
 * wall clock (steady_clock), matching livox_ros_driver2 publish_freq.
 * Periodically queries core_temp (0x8007) via QueryLivoxLidarInternalInfo.
 */
class LivoxLidarDevice : public LidarDevice
{
public:
  LivoxLidarDevice();
  ~LivoxLidarDevice() override;

  void open(const LidarOpenConfig & config) override;
  bool wait_cloud(LidarFrame & out, int timeout_ms) override;
  void drain_imu(std::vector<LidarImuSample> & out) override;
  void stop() override;
  bool set_scanning(bool enable) override;
  void request_status_update() override;
  LidarDeviceStatus get_status() const override;

  /// Optional path to a hand-written Mid-360 SDK2 JSON (skips auto-generate).
  void set_config_path(const std::string & path) { config_path_override_ = path; }
  void set_frame_time_ms(int ms) { frame_time_ms_ = ms; }

private:
  using SteadyClock = std::chrono::steady_clock;

  static void PointCloudCallback(
    uint32_t handle, const uint8_t dev_type,
    LivoxLidarEthernetPacket * data, void * client_data);
  static void ImuDataCallback(
    uint32_t handle, const uint8_t dev_type,
    LivoxLidarEthernetPacket * data, void * client_data);
  static void InfoChangeCallback(
    uint32_t handle, const LivoxLidarInfo * info, void * client_data);
  static void InfoPushCallback(
    uint32_t handle, uint8_t dev_type, const char * info, void * client_data);
  static void InternalInfoCallback(
    livox_status status, uint32_t handle,
    LivoxLidarDiagInternalInfoResponse * response, void * client_data);

  void on_point_cloud(LivoxLidarEthernetPacket * packet);
  void on_imu_data(LivoxLidarEthernetPacket * packet);
  void on_info_change(uint32_t handle, const LivoxLidarInfo * info);
  void on_info_push(uint32_t handle, uint8_t dev_type, const char * info);
  void on_internal_info(
    livox_status status, uint32_t handle,
    LivoxLidarDiagInternalInfoResponse * response);
  void apply_temp_raw(int32_t raw_core_centi_c, bool have_core,
                      int32_t raw_env_centi_c, bool have_env);
  void flush_building_locked();
  std::string write_mid360_config_json() const;

  LidarOpenConfig cfg_{};
  std::string config_path_override_;
  std::string config_path_used_;
  int frame_time_ms_{100};
  double temp_query_period_s_{2.0};
  bool opened_{false};

  std::mutex mu_;
  std::condition_variable cv_;
  std::vector<LidarPoint> building_;
  std::vector<LidarPoint> ready_;
  int64_t ready_stamp_ns_{0};
  int building_frame_cnt_{-1};
  SteadyClock::time_point building_start_steady_{};
  bool ready_valid_{false};
  uint32_t seq_{0};
  uint64_t pkt_count_{0};
  std::atomic<bool> lidar_ready_{false};
  std::atomic<uint32_t> handle_{0};

  std::atomic<bool> core_temp_valid_{false};
  std::atomic<float> core_temp_c_{0.f};
  std::atomic<bool> environment_temp_valid_{false};
  std::atomic<float> environment_temp_c_{0.f};
  std::atomic<int64_t> last_temp_query_ns_{0};
  std::atomic<bool> temp_query_inflight_{false};

  std::mutex imu_mu_;
  std::deque<LidarImuSample> imu_q_;
  std::atomic<bool> imu_enabled_{false};
};

}  // namespace composite_sensing
