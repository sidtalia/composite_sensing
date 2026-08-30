#pragma once

#include <cmath>
#include <cstring>
#include <string>

#include <librealsense2/rs.h>
#include <librealsense2/rsutil.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace composite_sensing {

inline rs2_intrinsics rs_intrinsics_from_camera_info(
  const sensor_msgs::msg::CameraInfo & info)
{
  rs2_intrinsics intr{};
  intr.width = static_cast<int>(info.width);
  intr.height = static_cast<int>(info.height);
  intr.fx = static_cast<float>(info.k[0]);
  intr.fy = static_cast<float>(info.k[4]);
  intr.ppx = static_cast<float>(info.k[2]);
  intr.ppy = static_cast<float>(info.k[5]);
  const std::string m = info.distortion_model;
  if (m == "inverse_plumb_bob" || m == "inverse_brown_conrady") {
    intr.model = RS2_DISTORTION_INVERSE_BROWN_CONRADY;
  } else if (m == "kannala_brandt4" || m == "equidistant" || m == "fisheye") {
    intr.model = RS2_DISTORTION_KANNALA_BRANDT4;
  } else if (m == "ftheta") {
    intr.model = RS2_DISTORTION_FTHETA;
  } else if (m == "plumb_bob" || m == "brown_conrady") {
    intr.model = RS2_DISTORTION_BROWN_CONRADY;
  } else {
    intr.model = RS2_DISTORTION_NONE;
  }
  for (int i = 0; i < 5; ++i) {
    intr.coeffs[i] = (i < static_cast<int>(info.d.size()))
      ? static_cast<float>(info.d[static_cast<size_t>(i)])
      : 0.f;
  }
  return intr;
}

inline sensor_msgs::msg::CameraInfo pinhole_camera_info(
  sensor_msgs::msg::CameraInfo info)
{
  info.distortion_model = "plumb_bob";
  info.d.assign(5, 0.0);
  return info;
}

/// Precomputed remap: pinhole output pixel → distorted input sample.
/// Uses librealsense project (handles inverse Brown–Conrady).
class ColorUndistorter
{
public:
  bool init(const sensor_msgs::msg::CameraInfo & info)
  {
    ready_ = false;
    const rs2_intrinsics intr = rs_intrinsics_from_camera_info(info);
    if (intr.width <= 0 || intr.height <= 0 || intr.fx <= 0.f || intr.fy <= 0.f) {
      return false;
    }
    float coeff_norm = 0.f;
    for (float c : intr.coeffs) {
      coeff_norm += std::fabs(c);
    }
    if (intr.model == RS2_DISTORTION_NONE || coeff_norm < 1e-12f) {
      return false;
    }

    map_x_.create(intr.height, intr.width, CV_32FC1);
    map_y_.create(intr.height, intr.width, CV_32FC1);
    for (int v = 0; v < intr.height; ++v) {
      auto * mx = map_x_.ptr<float>(v);
      auto * my = map_y_.ptr<float>(v);
      for (int u = 0; u < intr.width; ++u) {
        const float pt[3] = {
          (static_cast<float>(u) - intr.ppx) / intr.fx,
          (static_cast<float>(v) - intr.ppy) / intr.fy,
          1.f};
        float pix[2] = {0.f, 0.f};
        rs2_project_point_to_pixel(pix, &intr, pt);
        mx[u] = pix[0];
        my[u] = pix[1];
      }
    }
    pinhole_info_ = pinhole_camera_info(info);
    ready_ = true;
    return true;
  }

  bool ready() const { return ready_; }

  const sensor_msgs::msg::CameraInfo & pinhole_info() const { return pinhole_info_; }

  /// Remap ``img`` in place (rgb8 / bgr8). False if skipped.
  bool apply(sensor_msgs::msg::Image * img) const
  {
    if (!ready_ || img == nullptr) {
      return false;
    }
    const int rows = static_cast<int>(img->height);
    const int cols = static_cast<int>(img->width);
    if (rows != map_x_.rows || cols != map_x_.cols || img->data.empty()) {
      return false;
    }
    if (img->encoding != "rgb8" && img->encoding != "bgr8") {
      return false;
    }
    cv::Mat src(
      rows, cols, CV_8UC3, img->data.data(), static_cast<size_t>(img->step));
    cv::Mat dst;
    cv::remap(src, dst, map_x_, map_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    img->step = static_cast<uint32_t>(dst.step);
    img->data.resize(static_cast<size_t>(dst.step) * static_cast<size_t>(rows));
    if (dst.isContinuous()) {
      std::memcpy(img->data.data(), dst.data, img->data.size());
    } else {
      for (int r = 0; r < rows; ++r) {
        std::memcpy(
          img->data.data() + static_cast<size_t>(r) * img->step,
          dst.ptr(r),
          static_cast<size_t>(cols) * 3);
      }
    }
    return true;
  }

private:
  bool ready_{false};
  cv::Mat map_x_;
  cv::Mat map_y_;
  sensor_msgs::msg::CameraInfo pinhole_info_;
};

}  // namespace composite_sensing
