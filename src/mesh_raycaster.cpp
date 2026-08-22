#include "composite_sensing/mesh_raycaster.hpp"

#include <cctype>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "composite_sensing/assimp_mesh_load.hpp"
#include "composite_sensing/embree_mesh_map.hpp"

#if defined(HOUND_HAVE_VULKAN_RT)
#include "composite_sensing/vulkan_mesh_raycaster.hpp"
#endif

namespace composite_sensing
{

namespace
{
std::string lower_copy(std::string s)
{
  for (char & c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}
}  // namespace

std::unique_ptr<MeshRaycaster> create_mesh_raycaster(
  const std::string & mesh_path,
  const std::string & backend,
  std::string * used)
{
  const std::string req = lower_copy(backend.empty() ? "auto" : backend);
  if (req != "auto" && req != "vulkan" && req != "embree") {
    throw std::runtime_error(
      "raycast_backend must be one of: auto, vulkan, embree (got '" + backend + "')");
  }

  TriangleMesh mesh = TriangleMesh::load(mesh_path);
  const bool force_vk = req == "vulkan";
  const bool force_cpu = req == "embree";
  const bool try_vk = !force_cpu;

#if defined(HOUND_HAVE_VULKAN_RT)
  if (try_vk) {
    try {
      auto vk = VulkanMeshRaycaster::create(mesh);
      if (used) {
        *used = vk->backend_name();
      }
      std::fprintf(
        stderr, "[mesh_pf] raycast backend=%s triangles=%zu\n",
        vk->backend_name(), vk->triangle_count());
      return vk;
    } catch (const std::exception & e) {
      std::fprintf(stderr, "[mesh_pf] Vulkan RT unavailable: %s\n", e.what());
      if (force_vk) {
        throw;
      }
      std::fprintf(stderr, "[mesh_pf] falling back to Embree\n");
    }
  }
#else
  if (force_vk) {
    throw std::runtime_error(
      "Vulkan RT not built into mesh_pf (need libvulkan-dev and glslangValidator)");
  }
  (void)try_vk;
#endif

  auto cpu = EmbreeMeshMap::create(std::move(mesh));
  if (used) {
    *used = cpu->backend_name();
  }
  std::fprintf(
    stderr, "[mesh_pf] raycast backend=%s triangles=%zu\n",
    cpu->backend_name(), cpu->triangle_count());
  return cpu;
}

}  // namespace composite_sensing
