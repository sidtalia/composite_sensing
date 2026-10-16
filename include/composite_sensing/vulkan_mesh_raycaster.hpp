#pragma once

#include <memory>

#include "composite_sensing/assimp_mesh_load.hpp"
#include "composite_sensing/mesh_raycaster.hpp"

namespace composite_sensing
{

/** Headless Vulkan rayQuery closest-hit against a static triangle mesh. */
class VulkanMeshRaycaster : public MeshRaycaster
{
public:
  VulkanMeshRaycaster(const VulkanMeshRaycaster &) = delete;
  VulkanMeshRaycaster & operator=(const VulkanMeshRaycaster &) = delete;
  ~VulkanMeshRaycaster() override;

  static std::unique_ptr<VulkanMeshRaycaster> create(const TriangleMesh & mesh);

  const char * backend_name() const override { return "vulkan"; }
  size_t triangle_count() const override { return triangle_count_; }
  void batchIntersect(const Ray * rays, Hit * hits, size_t n) override;

private:
  VulkanMeshRaycaster() = default;
  void init(const TriangleMesh & mesh);

  size_t triangle_count_{0};

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace composite_sensing
