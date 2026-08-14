#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "composite_sensing/camera_device.hpp"
#include "composite_sensing/vslam_backend.hpp"

namespace composite_sensing {

/**
 * CameraDevice over a HOUND rail sim dataset (PNG depth/color/IR + camera.txt).
 *
 * One device = one view (``front`` / ``left`` / ``right``, or empty for legacy
 * single-view filenames). ``wait_stereo`` paces at ``dataset_rate_hz`` and
 * advances the frame index; ``poll_color`` / ``poll_depth`` each return the
 * current frame once per advance.
 */
class DatasetCameraDevice : public CameraDevice
{
public:
  DatasetCameraDevice() = default;
  ~DatasetCameraDevice() override;

  void open(const CameraOpenConfig & config) override;
  StreamCaps caps() const override;
  StereoCalibration stereo_calibration() const override;

  bool wait_stereo(StereoFrame & out, int timeout_ms) override;
  bool poll_color(ColorFrame & out) override;
  bool poll_depth(DepthFrame & out) override;
  void stop() override;

  const sensor_msgs::msg::CameraInfo & color_camera_info() const;
  const sensor_msgs::msg::CameraInfo & depth_camera_info() const;

  /** Static TFs matching realsense2_camera naming for ``camera_name``. */
  std::vector<geometry_msgs::msg::TransformStamped> static_camera_tfs(
    const std::string & camera_name) const;

  /**
   * Ground-truth world←optical pose for the frame last produced by
   * ``wait_stereo``. False if no pose file for that index.
   */
  bool gt_pose_optical(PoseOptical & out) const;

private:
  struct Kv {
    int width{640};
    int height{360};
    float fx{400.f};
    float fy{400.f};
    float cx{320.f};
    float cy{180.f};
    int n_frames{0};
  };

  static Kv load_camera_txt(const std::string & path);
  std::string frame_stem(int index) const;
  void fill_camera_infos();
  void fill_stereo_cal();

  CameraOpenConfig config_{};
  Kv kv_{};
  StreamCaps caps_{};
  StereoCalibration stereo_cal_{};
  sensor_msgs::msg::CameraInfo color_camera_info_;
  sensor_msgs::msg::CameraInfo depth_camera_info_;

  mutable std::mutex mu_;
  bool open_{false};
  int index_{0};
  int stereo_frame_id_{0};
  bool color_pending_{false};
  bool depth_pending_{false};
  bool has_gt_{false};
  PoseOptical gt_pose_{};

  std::vector<uint8_t> color_rgb_;
  std::vector<uint8_t> depth_z16_;
  std::vector<uint8_t> ir_left_;
  std::vector<uint8_t> ir_right_;
  int img_w_{0};
  int img_h_{0};
  int64_t timestamp_ns_{0};

  std::chrono::steady_clock::time_point next_frame_deadline_{};
};

}  // namespace composite_sensing
