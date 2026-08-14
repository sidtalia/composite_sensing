#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <librealsense2/rs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "composite_sensing/frame_conventions.hpp"

namespace composite_sensing {

/** Host system-clock ns when librealsense got the frame; 0 if unsupported. */
inline int64_t rs_frame_time_of_arrival_ns(const rs2::frame & f)
{
  if (!f.supports_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL)) {
    return 0;
  }
  // Metadata unit is milliseconds.
  return static_cast<int64_t>(
    f.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL)) * 1000000LL;
}

struct RsStreamProfiles {
  rs2::video_stream_profile infra1;
  rs2::video_stream_profile infra2;
  rs2::video_stream_profile color;   // valid if has_color
  rs2::video_stream_profile depth;   // valid if has_depth
  bool has_color{false};
  bool has_depth{false};
};

/**
 * Decoupled RealSense ownership: stereo module (IR ± depth) and RGB sensor
 * stream independently so Track is not gated on color frameset sync.
 */
struct RsSensorStreams {
  rs2::device device;
  rs2::sensor stereo;
  rs2::sensor color;
  bool has_color_sensor{false};
  bool stereo_started{false};
  bool color_started{false};
  float clip_distance_applied{0.f};

  RsStreamProfiles profiles{};
  // Syncer pairs IR1+IR2 (sensor.start enqueues single frames, not framesets).
  rs2::syncer ir_sync{4};
  rs2::frame_queue color_queue{4};

  // No destructor that calls stop(): open_rs_sensors used to return-by-value and a
  // ~RsSensorStreams(){stop()} would halt streams when the temporary was destroyed.

  void stop()
  {
    try {
      if (color_started) {
        color.stop();
        color.close();
        color_started = false;
      }
    } catch (...) {
    }
    try {
      if (stereo_started) {
        // Only undo options we explicitly applied. Forcing VISUAL_PRESET=0 on
        // every stop has left D455 depth/color wedged until a Viewer/USB reset.
        try {
          if (clip_distance_applied > 0.f &&
            stereo.supports(RS2_OPTION_MAX_DISTANCE))
          {
            stereo.set_option(RS2_OPTION_MAX_DISTANCE, 16.0f);
          }
        } catch (...) {
        }
        stereo.stop();
        stereo.close();
        stereo_started = false;
      }
    } catch (...) {
    }
    clip_distance_applied = 0.f;
    has_color_sensor = false;
    profiles = {};
  }
};

inline rs2::device find_device(const std::string & serial)
{
  rs2::context ctx;
  auto list = ctx.query_devices();
  if (list.size() == 0) {
    throw std::runtime_error("No RealSense devices found");
  }
  if (serial.empty()) {
    return list[0];
  }
  for (auto && dev : list) {
    if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) &&
      serial == dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER))
    {
      return dev;
    }
  }
  throw std::runtime_error("RealSense serial not found: " + serial);
}

/**
 * Like realsense2_camera ``initial_reset``: USB disconnect/reconnect clears
 * sticky firmware/USB state left by unclean shutdowns.
 *
 * The ROS wrapper clears the handle then sleeps ``reconnect_timeout`` (default
 * 6 s) before re-querying. Grabbing the device as soon as it re-lists is too
 * early — XU/UVC controls still fail with Protocol error. We wait for the
 * serial to drop (when possible), reappear, then settle before returning.
 */
inline void hardware_reset_and_wait(
  const std::string & serial,
  double wait_timeout_s = 20.0,
  double settle_s = 3.0)
{
  std::string sn = serial;
  {
    rs2::device dev = find_device(serial);
    if (sn.empty() && dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
      sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    }
    dev.hardware_reset();
  }

  const auto t0 = std::chrono::steady_clock::now();
  const auto deadline =
    t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(wait_timeout_s));
  // Do not touch the device until USB has had time to drop (ROS ~6 s default).
  const auto earliest_ready =
    t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(std::max(4.0, settle_s + 1.0)));

  bool saw_disconnect = false;
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    bool present = false;
    try {
      rs2::device d = find_device(sn);
      // Probe sensors — listing alone is not enough after reset.
      (void)d.query_sensors();
      present = true;
    } catch (const std::exception &) {
      present = false;
    }

    if (!present) {
      saw_disconnect = true;
      continue;
    }
    if (!saw_disconnect && std::chrono::steady_clock::now() < earliest_ready) {
      // Still the pre-reset enumeration (or half-ready). Keep waiting.
      continue;
    }
    std::this_thread::sleep_for(
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(settle_s)));
    try {
      rs2::device d = find_device(sn);
      (void)d.query_sensors();
      return;
    } catch (const std::exception &) {
      // Fell out again during settle; keep looping.
    }
  }
  throw std::runtime_error(
    "RealSense did not become ready after hardware_reset"
    + (sn.empty() ? std::string() : (": " + sn)));
}

inline rs2::video_stream_profile pick_video_profile(
  const rs2::sensor & sensor,
  rs2_stream stream,
  int index,
  int width,
  int height,
  rs2_format format,
  int fps)
{
  for (auto && p : sensor.get_stream_profiles()) {
    if (p.stream_type() != stream || p.format() != format) {
      continue;
    }
    if (stream == RS2_STREAM_INFRARED && p.stream_index() != index) {
      continue;
    }
    auto vp = p.as<rs2::video_stream_profile>();
    if (vp.width() == width && vp.height() == height && vp.fps() == fps) {
      return vp;
    }
  }
  throw std::runtime_error(
    std::string("No matching profile for ") + rs2_stream_to_string(stream) +
    " " + std::to_string(width) + "x" + std::to_string(height) + "@" +
    std::to_string(fps));
}

inline rs2::sensor find_stereo_sensor(const rs2::device & device)
{
  for (auto && s : device.query_sensors()) {
    for (auto && p : s.get_stream_profiles()) {
      if (p.stream_type() == RS2_STREAM_INFRARED) {
        return s;
      }
    }
  }
  throw std::runtime_error("Stereo/IR sensor not found on RealSense device");
}

inline rs2::sensor find_color_sensor(const rs2::device & device)
{
  for (auto && s : device.query_sensors()) {
    for (auto && p : s.get_stream_profiles()) {
      if (p.stream_type() == RS2_STREAM_COLOR) {
        return s;
      }
    }
  }
  throw std::runtime_error("Color sensor not found on RealSense device");
}

inline void apply_visual_preset(rs2::sensor & sensor, int preset)
{
  if (preset < 0) {
    return;
  }
  try {
    if (sensor.supports(RS2_OPTION_VISUAL_PRESET)) {
      sensor.set_option(RS2_OPTION_VISUAL_PRESET, static_cast<float>(preset));
    }
  } catch (const rs2::error &) {
    // XU can flake; streams may still start with firmware default preset.
  }
}

inline void apply_emitter(rs2::sensor & sensor, int emitter_enabled)
{
  try {
    if (!sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
      return;
    }
    sensor.set_option(RS2_OPTION_EMITTER_ENABLED, static_cast<float>(emitter_enabled));
  } catch (const rs2::error &) {
  }
}

inline void apply_clip_distance(rs2::sensor & sensor, float clip_m)
{
  if (clip_m <= 0.f) {
    return;
  }
  try {
    if (sensor.supports(RS2_OPTION_MAX_DISTANCE)) {
      sensor.set_option(RS2_OPTION_MAX_DISTANCE, clip_m);
    }
  } catch (const rs2::error &) {
  }
}

/**
 * Open stereo (IR ± depth) and optional color on separate sensors / queues.
 * IR Track never waits on RGB frameset sync.
 * Writes into ``out`` in place (do not return-by-value after sensor.start).
 */
inline void open_rs_sensors(
  RsSensorStreams & out,
  const std::string & serial,
  int infra_w, int infra_h, int infra_fps,
  bool enable_color, int color_w, int color_h, int color_fps,
  bool enable_depth, int depth_w, int depth_h, int depth_fps,
  int visual_preset, int emitter_enabled, float clip_m,
  bool initial_reset = false)
{
  out.stop();
  out.profiles = {};
  out.has_color_sensor = false;

  if (initial_reset) {
    hardware_reset_and_wait(serial);
  }

  // XU/UVC can still flake right after re-enumerate; retry like Viewer does.
  constexpr int k_open_attempts = 4;
  std::exception_ptr last_err;
  for (int attempt = 1; attempt <= k_open_attempts; ++attempt) {
    try {
      out.stop();
      out.profiles = {};
      out.has_color_sensor = false;

      out.device = find_device(serial);
      out.stereo = find_stereo_sensor(out.device);

      // Options before start (preset can reset emitter).
      apply_visual_preset(out.stereo, visual_preset);
      apply_emitter(out.stereo, emitter_enabled);
      apply_clip_distance(out.stereo, clip_m);
      out.clip_distance_applied = clip_m > 0.f ? clip_m : 0.f;

      out.profiles.infra1 = pick_video_profile(
        out.stereo, RS2_STREAM_INFRARED, 1, infra_w, infra_h, RS2_FORMAT_Y8, infra_fps);
      out.profiles.infra2 = pick_video_profile(
        out.stereo, RS2_STREAM_INFRARED, 2, infra_w, infra_h, RS2_FORMAT_Y8, infra_fps);

      std::vector<rs2::stream_profile> stereo_profiles{
        out.profiles.infra1, out.profiles.infra2};

      if (enable_depth) {
        out.profiles.depth = pick_video_profile(
          out.stereo, RS2_STREAM_DEPTH, 0, depth_w, depth_h, RS2_FORMAT_Z16, depth_fps);
        out.profiles.has_depth = true;
        stereo_profiles.push_back(out.profiles.depth);
      }

      out.stereo.open(stereo_profiles);
      // Feed the syncer so IR1+IR2 become matched framesets for Track.
      // Callback thread → ir_sync; Track thread dequeues (queue latency ≠ 0).
      out.stereo.start(out.ir_sync);
      out.stereo_started = true;

      if (enable_color) {
        out.color = find_color_sensor(out.device);
        out.profiles.color = pick_video_profile(
          out.color, RS2_STREAM_COLOR, 0, color_w, color_h, RS2_FORMAT_RGB8, color_fps);
        out.profiles.has_color = true;
        out.has_color_sensor = true;
        out.color.open(out.profiles.color);
        out.color.start(out.color_queue);
        out.color_started = true;
      }
      return;
    } catch (const rs2::error & e) {
      last_err = std::current_exception();
      out.stop();
      if (attempt == k_open_attempts) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1500 * attempt));
      (void)e;
    } catch (const std::exception & e) {
      last_err = std::current_exception();
      out.stop();
      if (attempt == k_open_attempts) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1500 * attempt));
      (void)e;
    }
  }
  if (last_err) {
    std::rethrow_exception(last_err);
  }
}

/** rs2 optical extrinsics → ROS FLU pose of stream body frame in camera_link. */
inline void optical_extrinsics_to_flu_pose(
  const rs2_extrinsics & stream_from_ref_optical,
  tf2::Vector3 & t_flu,
  tf2::Quaternion & q_flu)
{
  const auto & ex = stream_from_ref_optical;
  tf2::Matrix3x3 R_opt(
    ex.rotation[0], ex.rotation[3], ex.rotation[6],
    ex.rotation[1], ex.rotation[4], ex.rotation[7],
    ex.rotation[2], ex.rotation[5], ex.rotation[8]);
  tf2::Vector3 t_opt(ex.translation[0], ex.translation[1], ex.translation[2]);

  tf2::Matrix3x3 R_fo(
    kRFluFromOptical[0], kRFluFromOptical[1], kRFluFromOptical[2],
    kRFluFromOptical[3], kRFluFromOptical[4], kRFluFromOptical[5],
    kRFluFromOptical[6], kRFluFromOptical[7], kRFluFromOptical[8]);
  tf2::Matrix3x3 R_of = R_fo.transpose();

  tf2::Matrix3x3 R_flu = R_fo * R_opt * R_of;
  t_flu = R_fo * t_opt;
  R_flu.getRotation(q_flu);
}

inline geometry_msgs::msg::TransformStamped make_tf(
  const std::string & parent,
  const std::string & child,
  const tf2::Vector3 & t,
  const tf2::Quaternion & q)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;
  tf.transform.translation.x = t.x();
  tf.transform.translation.y = t.y();
  tf.transform.translation.z = t.z();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  return tf;
}

/**
 * Static TFs matching realsense2_camera naming:
 *   camera_link → camera_<stream>_frame → camera_<stream>_optical_frame
 * camera_link is FLU at infra1 origin.
 */
inline std::vector<geometry_msgs::msg::TransformStamped> build_static_camera_tfs(
  const std::string & camera_name,
  const RsStreamProfiles & profiles)
{
  const std::string link = camera_name + "_link";
  std::vector<geometry_msgs::msg::TransformStamped> tfs;

  auto add_stream = [&](
    const std::string & stream_suffix,
    const rs2::video_stream_profile & stream_prof,
    bool identity_to_link)
  {
    const std::string frame = camera_name + "_" + stream_suffix + "_frame";
    const std::string optical = camera_name + "_" + stream_suffix + "_optical_frame";

    tf2::Vector3 t(0, 0, 0);
    tf2::Quaternion q(0, 0, 0, 1);
    if (!identity_to_link) {
      auto ex = stream_prof.get_extrinsics_to(profiles.infra1);
      optical_extrinsics_to_flu_pose(ex, t, q);
    }
    tfs.push_back(make_tf(link, frame, t, q));

    std::array<double, 4> q_opt{};
    flu_to_optical_quat(q_opt);
    tf2::Quaternion q_optical(q_opt[0], q_opt[1], q_opt[2], q_opt[3]);
    tfs.push_back(make_tf(frame, optical, tf2::Vector3(0, 0, 0), q_optical));
  };

  add_stream("infra1", profiles.infra1, true);
  add_stream("infra2", profiles.infra2, false);
  if (profiles.has_color) {
    add_stream("color", profiles.color, false);
  }
  if (profiles.has_depth) {
    add_stream("depth", profiles.depth, false);
  }
  return tfs;
}

}  // namespace composite_sensing
