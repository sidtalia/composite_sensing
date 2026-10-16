#pragma once

#include <memory>
#include <string>
#include <vector>

#include <embree4/rtcore.h>

#include "composite_sensing/assimp_mesh_load.hpp"
#include "composite_sensing/mesh_raycaster.hpp"

namespace composite_sensing
{

/** Closest-hit plus Embree Ng (unit, facing the incoming ray). */
struct HitPlane
{
  float t{-1.f};
  uint32_t hit{0};
  float nx{0.f};
  float ny{0.f};
  float nz{1.f};
};

/** Triangle mesh accelerated with Embree 4 (CPU packet raycast). */
class EmbreeMeshMap : public MeshRaycaster
{
public:
  EmbreeMeshMap() = default;
  EmbreeMeshMap(const EmbreeMeshMap &) = delete;
  EmbreeMeshMap & operator=(const EmbreeMeshMap &) = delete;

  ~EmbreeMeshMap() override;

  static std::unique_ptr<EmbreeMeshMap> create(TriangleMesh mesh);
  static std::unique_ptr<EmbreeMeshMap> load(const std::string & path);

  const char * backend_name() const override { return "embree"; }
  size_t triangle_count() const override { return indices_.size() / 3; }
  void batchIntersect(const Ray * rays, Hit * hits, size_t n) override;
  void batchIntersectPlane(const Ray * rays, HitPlane * hits, size_t n);

private:
  RTCDevice device_{nullptr};
  RTCScene scene_{nullptr};
  std::vector<float> vertices_;
  std::vector<uint32_t> indices_;
};

}  // namespace composite_sensing
