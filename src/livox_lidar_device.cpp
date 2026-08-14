#include "composite_sensing/livox_lidar_device.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <livox_lidar_api.h>
#include <livox_lidar_def.h>

namespace composite_sensing
{

LivoxLidarDevice::LivoxLidarDevice() = default;

LivoxLidarDevice::~LivoxLidarDevice()
{
  stop();
}

std::string LivoxLidarDevice::write_mid360_config_json() const
{
  const std::string lidar_ip =
    cfg_.lidar_ip.empty() ? "192.168.1.138" : cfg_.lidar_ip;
  const std::string host_ip =
    cfg_.local_ip.empty() ? "192.168.1.50" : cfg_.local_ip;
  // Official Mid-360 host ports (cmd/push/point/imu/log).
  const uint16_t host_cmd = cfg_.local_port ? cfg_.local_port : 56101;
  const uint16_t host_push = static_cast<uint16_t>(host_cmd + 100);
  const uint16_t host_point = static_cast<uint16_t>(host_cmd + 200);
  const uint16_t host_imu = static_cast<uint16_t>(host_cmd + 300);
  const uint16_t host_log = static_cast<uint16_t>(host_cmd + 400);

  const char * path = "/tmp/hound_livox_mid360.json";
  std::ofstream out(path, std::ios::trunc);
  if (!out) {
    throw std::runtime_error("LivoxLidarDevice: cannot write " + std::string(path));
  }
  // Livox-SDK2 ParseCfgFile (not livox_ros_driver2):
  // - host_net_info must be an array; include lidar_ip[] → custom_lidars path
  //   (binds cmd/push/point sockets at init; enables UpdateLidarCfg → InfoChange).
  // - This unit reports SDK dev_type 35 → key must be "Mid360s" (not "MID360"/9).
  //   Wrong key leaves host PCL IP stuck (was 192.168.1.5) and never streams here.
  out <<
    "{\n"
    "  \"Mid360s\": {\n"
    "    \"lidar_net_info\": {\n"
    "      \"cmd_data_port\": 56100,\n"
    "      \"push_msg_port\": 56200,\n"
    "      \"point_data_port\": 56300,\n"
    "      \"imu_data_port\": 56400,\n"
    "      \"log_data_port\": 56500\n"
    "    },\n"
    "    \"host_net_info\": [\n"
    "      {\n"
    "        \"host_ip\": \"" << host_ip << "\",\n"
    "        \"lidar_ip\": [\"" << lidar_ip << "\"],\n"
    "        \"cmd_data_port\": " << host_cmd << ",\n"
    "        \"push_msg_port\": " << host_push << ",\n"
    "        \"point_data_port\": " << host_point << ",\n"
    "        \"imu_data_port\": " << host_imu << ",\n"
    "        \"log_data_port\": " << host_log << "\n"
    "      }\n"
    "    ]\n"
    "  }\n"
    "}\n";
  out.close();
  return path;
}

void LivoxLidarDevice::PointCloudCallback(
  uint32_t /*handle*/, const uint8_t /*dev_type*/,
  LivoxLidarEthernetPacket * data, void * client_data)
{
  auto * self = static_cast<LivoxLidarDevice *>(client_data);
  if (self && data) {
    self->on_point_cloud(data);
  }
}

void LivoxLidarDevice::InfoChangeCallback(
  uint32_t handle, const LivoxLidarInfo * info, void * client_data)
{
  auto * self = static_cast<LivoxLidarDevice *>(client_data);
  if (self) {
    self->on_info_change(handle, info);
  }
}

void LivoxLidarDevice::InfoPushCallback(
  uint32_t handle, uint8_t dev_type, const char * info, void * client_data)
{
  auto * self = static_cast<LivoxLidarDevice *>(client_data);
  if (self) {
    self->on_info_push(handle, dev_type, info);
  }
}

void LivoxLidarDevice::InternalInfoCallback(
  livox_status status, uint32_t handle,
  LivoxLidarDiagInternalInfoResponse * response, void * client_data)
{
  auto * self = static_cast<LivoxLidarDevice *>(client_data);
  if (self) {
    self->on_internal_info(status, handle, response);
  }
}

void LivoxLidarDevice::apply_temp_raw(
  int32_t raw_core_centi_c, bool have_core,
  int32_t raw_env_centi_c, bool have_env)
{
  // Protocol: core_temp / environment_temp are int32 in units of 0.01 °C.
  if (have_core) {
    const float c = static_cast<float>(raw_core_centi_c) * 0.01f;
    const bool first = !core_temp_valid_.exchange(true);
    core_temp_c_.store(c);
    if (first) {
      std::cerr << "[LivoxLidarDevice] core_temp online: " << c << " C\n";
    }
  }
  if (have_env) {
    const float c = static_cast<float>(raw_env_centi_c) * 0.01f;
    environment_temp_c_.store(c);
    environment_temp_valid_.store(true);
  }
}

void LivoxLidarDevice::on_info_change(
  uint32_t handle, const LivoxLidarInfo * info)
{
  const char * sn = (info && info->sn[0]) ? info->sn : "(unknown)";
  handle_.store(handle);
  std::cerr << "[LivoxLidarDevice] InfoChange handle=" << handle
            << " sn=" << sn << " → motor ON (Normal) + CartesianHigh\n";
  // Launch path: spin up the motor if it is not already running.
  SetLivoxLidarPclDataType(
    handle, kLivoxLidarCartesianCoordinateHighData, nullptr, nullptr);
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
  lidar_ready_.store(true);
  // Kick an immediate temperature query once the handle is known.
  last_temp_query_ns_.store(0);
  request_status_update();
}

void LivoxLidarDevice::on_info_push(
  uint32_t /*handle*/, uint8_t /*dev_type*/, const char * info)
{
  if (!info || !info[0]) {
    return;
  }
  // Push JSON from ParseLidarStateInfo, e.g. {"core_temp":4500,...} in 0.01°C.
  auto find_int = [](const char * json, const char * key, int32_t & out) -> bool {
    const std::string needle = std::string("\"") + key + "\"";
    const char * p = std::strstr(json, needle.c_str());
    if (!p) {
      return false;
    }
    p = std::strchr(p + needle.size(), ':');
    if (!p) {
      return false;
    }
    char * end = nullptr;
    const long v = std::strtol(p + 1, &end, 10);
    if (end == p + 1) {
      return false;
    }
    out = static_cast<int32_t>(v);
    return true;
  };

  int32_t core_raw = 0;
  int32_t env_raw = 0;
  const bool have_core = find_int(info, "core_temp", core_raw);
  const bool have_env = find_int(info, "environment_temp", env_raw);
  apply_temp_raw(core_raw, have_core, env_raw, have_env);
}

void LivoxLidarDevice::on_internal_info(
  livox_status status, uint32_t /*handle*/,
  LivoxLidarDiagInternalInfoResponse * response)
{
  temp_query_inflight_.store(false);
  if (status != kLivoxLidarStatusSuccess || response == nullptr) {
    static int fail_logs = 0;
    if (fail_logs < 2) {
      std::cerr << "[LivoxLidarDevice] QueryInternalInfo failed status="
                << static_cast<int>(status) << "\n";
      ++fail_logs;
    }
    return;
  }
  if (response->ret_code != 0) {
    // 0x20 = key not supported (bulk query). Prefer push-msg path.
    return;
  }

  bool got_core = false;
  bool got_env = false;
  int32_t core_raw = 0;
  int32_t env_raw = 0;

  size_t off = 0;
  for (uint16_t i = 0; i < response->param_num; ++i) {
    auto * kv = reinterpret_cast<LivoxLidarKeyValueParam *>(&response->data[off]);
    const uint16_t key = kv->key;
    const uint16_t len = kv->length;
    const uint8_t * v = kv->value;
    off += sizeof(uint16_t) * 2 + static_cast<size_t>(len);

    if (key == static_cast<uint16_t>(kKeyCoreTemp) && len >= sizeof(int32_t)) {
      std::memcpy(&core_raw, v, sizeof(int32_t));
      got_core = true;
    } else if (
      key == static_cast<uint16_t>(kKeyEnvironmentTemp) &&
      len >= sizeof(int32_t))
    {
      std::memcpy(&env_raw, v, sizeof(int32_t));
      got_env = true;
    }
  }

  apply_temp_raw(core_raw, got_core, env_raw, got_env);
}

void LivoxLidarDevice::request_status_update()
{
  if (!opened_ || temp_query_period_s_ <= 0.0) {
    return;
  }
  const uint32_t handle = handle_.load();
  if (handle == 0) {
    return;
  }
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    SteadyClock::now().time_since_epoch()).count();
  const int64_t last_ns = last_temp_query_ns_.load();
  if (last_ns > 0) {
    const double dt_s = static_cast<double>(now_ns - last_ns) * 1e-9;
    if (dt_s < temp_query_period_s_) {
      return;
    }
  }
  bool expected = false;
  if (!temp_query_inflight_.compare_exchange_strong(expected, true)) {
    return;
  }
  last_temp_query_ns_.store(now_ns);
  const livox_status st =
    QueryLivoxLidarInternalInfo(handle, InternalInfoCallback, this);
  if (st != kLivoxLidarStatusSuccess) {
    temp_query_inflight_.store(false);
    std::cerr << "[LivoxLidarDevice] QueryLivoxLidarInternalInfo enqueue failed "
              << "status=" << static_cast<int>(st) << "\n";
  }
}

LidarDeviceStatus LivoxLidarDevice::get_status() const
{
  LidarDeviceStatus s;
  s.core_temp_valid = core_temp_valid_.load();
  s.core_temp_c = core_temp_c_.load();
  s.environment_temp_valid = environment_temp_valid_.load();
  s.environment_temp_c = environment_temp_c_.load();
  return s;
}

bool LivoxLidarDevice::set_scanning(bool enable)
{
  const uint32_t handle = handle_.load();
  if (!opened_ || handle == 0) {
    return false;
  }
  // Mid-360S: motor-off standby is kLivoxLidarWakeUp (name is misleading).
  // Do not use kLivoxLidarSleep — it does not stop this product's motor.
  const LivoxLidarWorkMode mode =
    enable ? kLivoxLidarNormal : kLivoxLidarWakeUp;
  const livox_status st =
    SetLivoxLidarWorkMode(handle, mode, nullptr, nullptr);
  std::cerr << "[LivoxLidarDevice] motor " << (enable ? "ON (Normal)" : "OFF (WakeUp/standby)")
            << " status=" << static_cast<int>(st) << "\n";
  return st == kLivoxLidarStatusSuccess;
}

void LivoxLidarDevice::flush_building_locked()
{
  if (building_.empty()) {
    return;
  }
  ready_.swap(building_);

  // time_offset_s is host-steady relative to scan start (see on_point_cloud).
  // Stamp frame start in system/ROS time: flush ≈ scan end.
  double max_off = 0.0;
  for (const auto & p : ready_) {
    if (std::isfinite(p.time_offset_s)) {
      max_off = std::max(max_off, static_cast<double>(p.time_offset_s));
    }
  }
  // Belt-and-suspenders: never claim a scan older than ~2× aggregation window.
  const double cap_s = std::max(0.05, 0.002 * static_cast<double>(frame_time_ms_));
  max_off = std::min(max_off, cap_s);

  const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  ready_stamp_ns_ =
    now_ns - static_cast<int64_t>(std::llround(max_off * 1e9));

  ready_valid_ = true;
  building_.clear();
  building_frame_cnt_ = -1;
  building_start_steady_ = SteadyClock::time_point{};
  cv_.notify_one();
}

void LivoxLidarDevice::on_point_cloud(LivoxLidarEthernetPacket * pkt)
{
  if (!pkt || pkt->dot_num == 0) {
    return;
  }

  const int frame_cnt = static_cast<int>(pkt->frame_cnt);
  // Official livox_ros_driver2: point_interval_ns = time_interval * 100 / dot_num
  // (time_interval is in 0.1 µs → *100 = ns for the whole packet).
  const uint32_t point_interval_ns =
    (pkt->dot_num > 0)
      ? (static_cast<uint32_t>(pkt->time_interval) * 100u / pkt->dot_num)
      : 0u;

  const auto now_steady = SteadyClock::now();
  std::lock_guard<std::mutex> lock(mu_);

  // Prefer wall-clock aggregation (official driver publish_freq). frame_cnt
  // alone can split Mid-360 into tiny partial clouds. Do NOT use device
  // packet stamps for ROS time or cross-packet offsets — bases jump (GPS sync).
  const bool frame_boundary =
    (building_frame_cnt_ >= 0 && frame_cnt != building_frame_cnt_);
  const bool time_boundary =
    !building_.empty() &&
    building_start_steady_.time_since_epoch().count() > 0 &&
    frame_time_ms_ > 0 &&
    (now_steady - building_start_steady_) >=
      std::chrono::milliseconds(frame_time_ms_);
  if ((frame_boundary || time_boundary) && !building_.empty()) {
    flush_building_locked();
  }
  if (building_.empty()) {
    building_frame_cnt_ = frame_cnt;
    building_start_steady_ = now_steady;
  }

  // Host-steady offset from scan start + within-packet point index.
  const double pkt_off_s =
    std::chrono::duration<double>(now_steady - building_start_steady_).count();

  const float rmin2 = cfg_.range_min * cfg_.range_min;
  const float rmax2 = cfg_.range_max * cfg_.range_max;

  auto push_xyz = [&](float x, float y, float z, float intensity, uint32_t i) {
    const float r2 = x * x + y * y + z * z;
    if (r2 < rmin2 || r2 > rmax2) {
      return;
    }
    LidarPoint q;
    q.x = x;
    q.y = y;
    q.z = z;
    q.intensity = intensity;
    q.time_offset_s = static_cast<float>(
      pkt_off_s +
      static_cast<double>(i) * static_cast<double>(point_interval_ns) * 1e-9);
    building_.push_back(q);
  };

  if (pkt->data_type == kLivoxLidarCartesianCoordinateHighData) {
    auto * pts = reinterpret_cast<LivoxLidarCartesianHighRawPoint *>(pkt->data);
    for (uint32_t i = 0; i < pkt->dot_num; ++i) {
      push_xyz(
        pts[i].x * 0.001f, pts[i].y * 0.001f, pts[i].z * 0.001f,
        static_cast<float>(pts[i].reflectivity), i);
    }
  } else if (pkt->data_type == kLivoxLidarCartesianCoordinateLowData) {
    auto * pts = reinterpret_cast<LivoxLidarCartesianLowRawPoint *>(pkt->data);
    for (uint32_t i = 0; i < pkt->dot_num; ++i) {
      push_xyz(
        pts[i].x * 0.01f, pts[i].y * 0.01f, pts[i].z * 0.01f,
        static_cast<float>(pts[i].reflectivity), i);
    }
  } else if (pkt->data_type == kLivoxLidarDoubleEchoData) {
    auto * pts = reinterpret_cast<LivoxLidarDoubleEchoRawPoint *>(pkt->data);
    for (uint32_t i = 0; i < pkt->dot_num; ++i) {
      push_xyz(
        pts[i].x1 * 0.001f, pts[i].y1 * 0.001f, pts[i].z1 * 0.001f,
        static_cast<float>(pts[i].reflectivity1), i);
    }
  } else {
    // IMU or unknown — ignore.
    return;
  }

  ++pkt_count_;
}

void LivoxLidarDevice::open(const LidarOpenConfig & config)
{
  stop();
  cfg_ = config;
  if (cfg_.frame_time_ms > 0) {
    frame_time_ms_ = cfg_.frame_time_ms;
  }
  temp_query_period_s_ = cfg_.temp_query_period_s;
  {
    std::lock_guard<std::mutex> lock(mu_);
    building_.clear();
    ready_.clear();
    ready_valid_ = false;
    building_frame_cnt_ = -1;
    building_start_steady_ = SteadyClock::time_point{};
    pkt_count_ = 0;
  }
  lidar_ready_.store(false);
  handle_.store(0);
  core_temp_valid_.store(false);
  environment_temp_valid_.store(false);
  temp_query_inflight_.store(false);
  last_temp_query_ns_.store(0);

  if (!config_path_override_.empty()) {
    config_path_used_ = config_path_override_;
  } else if (!cfg_.config_path.empty()) {
    config_path_used_ = cfg_.config_path;
  } else {
    config_path_used_ = write_mid360_config_json();
  }

  const char * host =
    cfg_.local_ip.empty() ? "" : cfg_.local_ip.c_str();
  std::cerr << "[LivoxLidarDevice] SdkInit config=" << config_path_used_
            << " host=" << (host[0] ? host : "(default)")
            << " frame_time_ms=" << frame_time_ms_
            << " temp_query_period_s=" << temp_query_period_s_ << "\n";
  // Must run before LivoxLidarSdkInit — InitLogger only reads this flag once.
  // Also silences an already-created logger if SDK was previously inited.
  DisableLivoxSdkConsoleLogger();
  if (!LivoxLidarSdkInit(config_path_used_.c_str(), host)) {
    throw std::runtime_error(
      "LivoxLidarDevice: LivoxLidarSdkInit failed (config=" +
      config_path_used_ + ")");
  }
  DisableLivoxSdkConsoleLogger();
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, this);
  SetLivoxLidarInfoChangeCallback(InfoChangeCallback, this);
  // Periodic push (cmd 0x0102) carries core_temp — preferred over GetInternalInfo.
  SetLivoxLidarInfoCallback(InfoPushCallback, this);
  if (!LivoxLidarSdkStart()) {
    LivoxLidarSdkUninit();
    throw std::runtime_error("LivoxLidarDevice: LivoxLidarSdkStart failed");
  }
  opened_ = true;
}

bool LivoxLidarDevice::wait_cloud(LidarFrame & out, int timeout_ms)
{
  if (!opened_) {
    return false;
  }
  std::unique_lock<std::mutex> lock(mu_);
  const bool ok = cv_.wait_for(
    lock, std::chrono::milliseconds(timeout_ms),
    [this]() { return ready_valid_; });

  // Flush a completed aggregation window if the capture thread woke on timeout.
  if (!ok && !building_.empty() && frame_time_ms_ > 0 &&
      building_start_steady_.time_since_epoch().count() > 0)
  {
    const auto age = SteadyClock::now() - building_start_steady_;
    if (age >= std::chrono::milliseconds(frame_time_ms_)) {
      flush_building_locked();
    }
  }

  if (!ready_valid_) {
    return false;
  }
  out.points = std::move(ready_);
  ready_.clear();
  ready_valid_ = false;
  out.timestamp_ns = ready_stamp_ns_;
  out.seq = ++seq_;
  out.frame_id = cfg_.frame_id;
  return !out.points.empty();
}

void LivoxLidarDevice::stop()
{
  if (!opened_) {
    return;
  }
  // Shutdown path: stop the motor (WakeUp/standby) before tearing down the SDK.
  set_scanning(false);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  opened_ = false;
  LivoxLidarSdkUninit();
  {
    std::lock_guard<std::mutex> lock(mu_);
    building_.clear();
    ready_.clear();
    ready_valid_ = false;
  }
  cv_.notify_all();
  lidar_ready_.store(false);
  handle_.store(0);
  core_temp_valid_.store(false);
  environment_temp_valid_.store(false);
  temp_query_inflight_.store(false);
}

}  // namespace composite_sensing
