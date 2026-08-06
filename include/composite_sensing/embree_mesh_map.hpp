#pragma once

#include <memory>
#include <string>
#include <vector>

#include <embree4/rtcore.h>

namespace composite_sensing
{

/** Triangle mesh accelerated with Embree 4 (CPU raycast). */
struct EmbreeMeshMap
{
  RTCDevice device{nullptr};
  RTCScene scene{nullptr};
  std::vector<float> vertices;   // xyz xyz ...
  std::vector<uint32_t> indices;  // triangles

  EmbreeMeshMap() = default;
  EmbreeMeshMap(const EmbreeMeshMap &) = delete;
  EmbreeMeshMap & operator=(const EmbreeMeshMap &) = delete;

  ~EmbreeMeshMap()
  {
    if (scene) {
      rtcReleaseScene(scene);
      scene = nullptr;
    }
    if (device) {
      rtcReleaseDevice(device);
      device = nullptr;
    }
  }

  /** Load with Assimp (dae/obj/ply/…). Throws on failure. */
  static std::unique_ptr<EmbreeMeshMap> load(const std::string & path);

  /**
   * Raycast from origin along dir (must be unit). Returns hit distance or
   * negative if miss. max_range clamps tfar.
   */
  float raycast(
    float ox, float oy, float oz,
    float dx, float dy, float dz,
    float max_range) const;
};

}  // namespace composite_sensing
