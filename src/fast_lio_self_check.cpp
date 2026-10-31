#include <cmath>
#include <cstdio>
#include <vector>

#include "composite_sensing/fast_lio_odometry.hpp"

// Stationary IMU + a floor of points. Fails if the deskew/odom pipe never returns.
int main()
{
  composite_sensing::FastLioConfig cfg;
  composite_sensing::FastLioOdometry lio(cfg);
  bool got = false;
  for (int scan = 0; scan < 6; ++scan) {
    const int64_t beg = static_cast<int64_t>(scan) * 100000000LL;
    std::vector<composite_sensing::LidarImuSample> imu;
    imu.reserve(20);
    for (int i = 0; i < 20; ++i) {
      composite_sensing::LidarImuSample s;
      s.timestamp_ns = beg + static_cast<int64_t>(i) * 5000000LL;
      s.az = -9.81f;
      imu.push_back(s);
    }
    lio.push_imu(imu);

    composite_sensing::LidarFrame cloud;
    cloud.timestamp_ns = beg;
    cloud.frame_id = "livox_frame";
    for (int x = 0; x <= 8; ++x) {
      for (int y = 0; y <= 8; ++y) {
        composite_sensing::LidarPoint p;
        p.x = 0.4f * static_cast<float>(x);
        p.y = 0.4f * static_cast<float>(y);
        p.time_offset_s = 0.009f * static_cast<float>((x + y) % 10);
        cloud.points.push_back(p);
      }
    }
    for (const auto & out : lio.push_cloud(cloud)) {
      if (!out.odom.valid || out.cloud.points.empty()) {
        continue;
      }
      if (!std::isfinite(out.odom.position.x()) ||
        !std::isfinite(out.cloud.points.front().x))
      {
        std::fprintf(stderr, "fast-lio self-check: non-finite output\n");
        return 1;
      }
      got = true;
    }
  }
  if (!got) {
    std::fprintf(stderr, "fast-lio self-check: no deskewed scan\n");
    return 1;
  }
  return 0;
}
