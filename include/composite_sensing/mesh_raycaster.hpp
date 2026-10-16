#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace composite_sensing
{

/** Closest-hit ray. Direction should be unit length. Layout matches the compute shader. */
struct alignas(16) Ray
{
  float ox, oy, oz, tmin;
  float dx, dy, dz, tmax;
};

struct alignas(16) Hit
{
  float t;
  uint32_t hit;  // 1 = closest hit, 0 = miss
  uint32_t pad0{0};
  uint32_t pad1{0};
};

class MeshRaycaster
{
public:
  virtual ~MeshRaycaster() = default;

  virtual const char * backend_name() const = 0;
  virtual size_t triangle_count() const = 0;

  /** Closest-hit intersect. hits[i].hit == 0 on miss (t undefined). */
  virtual void batchIntersect(const Ray * rays, Hit * hits, size_t n) = 0;
};

/**
 * Create a raycaster for map_file.
 * backend: "auto" (Vulkan RT then Embree), "vulkan", or "embree".
 * used: optional, filled with the backend actually constructed.
 */
std::unique_ptr<MeshRaycaster> create_mesh_raycaster(
  const std::string & mesh_path,
  const std::string & backend,
  std::string * used = nullptr);

}  // namespace composite_sensing
