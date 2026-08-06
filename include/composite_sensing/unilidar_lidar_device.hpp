#pragma once

#include <memory>
#include <string>

#include "composite_sensing/lidar_device.hpp"

namespace unitree_lidar_sdk
{
class UnitreeLidarReader;
}

namespace composite_sensing
{

/** Unitree UniLidar via vendor SDK (serial or UDP). */
class UnilidarLidarDevice : public LidarDevice
{
public:
  UnilidarLidarDevice();
  ~UnilidarLidarDevice() override;

  void open(const LidarOpenConfig & config) override;
  bool wait_cloud(LidarFrame & out, int timeout_ms) override;
  void stop() override;

private:
  LidarOpenConfig cfg_{};
  unitree_lidar_sdk::UnitreeLidarReader * reader_{nullptr};
  bool opened_{false};
};

}  // namespace composite_sensing
