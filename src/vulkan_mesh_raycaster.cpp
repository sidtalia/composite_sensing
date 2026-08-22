#include "composite_sensing/vulkan_mesh_raycaster.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <vulkan/vulkan.h>

#include "mesh_rayquery.spv.h"

namespace composite_sensing
{
namespace
{

const char * vk_err(VkResult r)
{
  switch (r) {
    case VK_SUCCESS: return "VK_SUCCESS";
    case VK_ERROR_OUT_OF_HOST_MEMORY: return "VK_ERROR_OUT_OF_HOST_MEMORY";
    case VK_ERROR_OUT_OF_DEVICE_MEMORY: return "VK_ERROR_OUT_OF_DEVICE_MEMORY";
    case VK_ERROR_INITIALIZATION_FAILED: return "VK_ERROR_INITIALIZATION_FAILED";
    case VK_ERROR_DEVICE_LOST: return "VK_ERROR_DEVICE_LOST";
    case VK_ERROR_EXTENSION_NOT_PRESENT: return "VK_ERROR_EXTENSION_NOT_PRESENT";
    case VK_ERROR_FEATURE_NOT_PRESENT: return "VK_ERROR_FEATURE_NOT_PRESENT";
    default: return "VkResult";
  }
}

void check(VkResult r, const char * what)
{
  if (r != VK_SUCCESS) {
    throw std::runtime_error(std::string("Vulkan RT ") + what + ": " + vk_err(r));
  }
}

bool has_ext(const std::vector<VkExtensionProperties> & props, const char * name)
{
  for (const auto & p : props) {
    if (std::strcmp(p.extensionName, name) == 0) {
      return true;
    }
  }
  return false;
}

uint32_t find_memory_type(
  VkPhysicalDevice phys, uint32_t type_bits, VkMemoryPropertyFlags flags)
{
  VkPhysicalDeviceMemoryProperties mem{};
  vkGetPhysicalDeviceMemoryProperties(phys, &mem);
  for (uint32_t i = 0; i < mem.memoryTypeCount; ++i) {
    if ((type_bits & (1u << i)) &&
        (mem.memoryTypes[i].propertyFlags & flags) == flags)
    {
      return i;
    }
  }
  throw std::runtime_error("Vulkan RT: no matching memory type");
}

struct GpuBuffer
{
  VkBuffer buffer{VK_NULL_HANDLE};
  VkDeviceMemory memory{VK_NULL_HANDLE};
  VkDeviceSize size{0};
  void * mapped{nullptr};
  VkDeviceAddress address{0};
};

}  // namespace

struct VulkanMeshRaycaster::Impl
{
  VkInstance instance{VK_NULL_HANDLE};
  VkPhysicalDevice phys{VK_NULL_HANDLE};
  VkDevice device{VK_NULL_HANDLE};
  uint32_t queue_family{0};
  VkQueue queue{VK_NULL_HANDLE};
  VkCommandPool cmd_pool{VK_NULL_HANDLE};
  VkCommandBuffer cmd{VK_NULL_HANDLE};
  VkFence fence{VK_NULL_HANDLE};

  PFN_vkCreateAccelerationStructureKHR vkCreateAccelerationStructureKHR{nullptr};
  PFN_vkDestroyAccelerationStructureKHR vkDestroyAccelerationStructureKHR{nullptr};
  PFN_vkGetAccelerationStructureBuildSizesKHR vkGetAccelerationStructureBuildSizesKHR{nullptr};
  PFN_vkCmdBuildAccelerationStructuresKHR vkCmdBuildAccelerationStructuresKHR{nullptr};
  PFN_vkGetAccelerationStructureDeviceAddressKHR vkGetAccelerationStructureDeviceAddressKHR{nullptr};

  GpuBuffer vertex;
  GpuBuffer index;
  GpuBuffer blas_store;
  GpuBuffer tlas_store;
  GpuBuffer tlas_instances;
  GpuBuffer scratch;
  GpuBuffer rays;
  GpuBuffer hits;

  VkAccelerationStructureKHR blas{VK_NULL_HANDLE};
  VkAccelerationStructureKHR tlas{VK_NULL_HANDLE};

  VkDescriptorSetLayout dset_layout{VK_NULL_HANDLE};
  VkPipelineLayout pipe_layout{VK_NULL_HANDLE};
  VkPipeline pipeline{VK_NULL_HANDLE};
  VkDescriptorPool dset_pool{VK_NULL_HANDLE};
  VkDescriptorSet dset{VK_NULL_HANDLE};
  VkShaderModule shader{VK_NULL_HANDLE};

  size_t ray_capacity{0};
  uint32_t scratch_align{256};

  ~Impl()
  {
    if (device) {
      vkDeviceWaitIdle(device);
    }
    destroy_buffer(rays);
    destroy_buffer(hits);
    destroy_buffer(scratch);
    destroy_buffer(tlas_instances);
    destroy_buffer(vertex);
    destroy_buffer(index);
    if (device && vkDestroyAccelerationStructureKHR) {
      if (tlas) {
        vkDestroyAccelerationStructureKHR(device, tlas, nullptr);
      }
      if (blas) {
        vkDestroyAccelerationStructureKHR(device, blas, nullptr);
      }
    }
    destroy_buffer(tlas_store);
    destroy_buffer(blas_store);
    if (device) {
      if (pipeline) {
        vkDestroyPipeline(device, pipeline, nullptr);
      }
      if (pipe_layout) {
        vkDestroyPipelineLayout(device, pipe_layout, nullptr);
      }
      if (dset_layout) {
        vkDestroyDescriptorSetLayout(device, dset_layout, nullptr);
      }
      if (dset_pool) {
        vkDestroyDescriptorPool(device, dset_pool, nullptr);
      }
      if (shader) {
        vkDestroyShaderModule(device, shader, nullptr);
      }
      if (fence) {
        vkDestroyFence(device, fence, nullptr);
      }
      if (cmd_pool) {
        vkDestroyCommandPool(device, cmd_pool, nullptr);
      }
      vkDestroyDevice(device, nullptr);
    }
    if (instance) {
      vkDestroyInstance(instance, nullptr);
    }
  }

  void destroy_buffer(GpuBuffer & b)
  {
    if (!device) {
      return;
    }
    if (b.mapped) {
      vkUnmapMemory(device, b.memory);
      b.mapped = nullptr;
    }
    if (b.buffer) {
      vkDestroyBuffer(device, b.buffer, nullptr);
      b.buffer = VK_NULL_HANDLE;
    }
    if (b.memory) {
      vkFreeMemory(device, b.memory, nullptr);
      b.memory = VK_NULL_HANDLE;
    }
    b.size = 0;
    b.address = 0;
  }

  GpuBuffer create_buffer(
    VkDeviceSize size,
    VkBufferUsageFlags usage,
    VkMemoryPropertyFlags mem_flags,
    bool map)
  {
    GpuBuffer b;
    b.size = size;
    VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bi.size = size;
    bi.usage = usage;
    bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    check(vkCreateBuffer(device, &bi, nullptr, &b.buffer), "vkCreateBuffer");

    VkMemoryRequirements req{};
    vkGetBufferMemoryRequirements(device, b.buffer, &req);

    VkMemoryAllocateFlagsInfo flags{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO};
    if (usage & VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT) {
      flags.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT;
    }

    VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    ai.pNext = (flags.flags != 0) ? &flags : nullptr;
    ai.allocationSize = req.size;
    try {
      ai.memoryTypeIndex = find_memory_type(phys, req.memoryTypeBits, mem_flags);
    } catch (const std::exception &) {
      if (map) {
        throw;
      }
      ai.memoryTypeIndex = find_memory_type(
        phys, req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    }
    check(vkAllocateMemory(device, &ai, nullptr, &b.memory), "vkAllocateMemory");
    check(vkBindBufferMemory(device, b.buffer, b.memory, 0), "vkBindBufferMemory");

    if (usage & VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT) {
      VkBufferDeviceAddressInfo addr{VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO};
      addr.buffer = b.buffer;
      b.address = vkGetBufferDeviceAddress(device, &addr);
    }
    if (map) {
      check(vkMapMemory(device, b.memory, 0, size, 0, &b.mapped), "vkMapMemory");
    }
    return b;
  }

  void load_as_fns()
  {
    auto load = [&](const char * name) {
      return vkGetDeviceProcAddr(device, name);
    };
    vkCreateAccelerationStructureKHR = reinterpret_cast<PFN_vkCreateAccelerationStructureKHR>(
      load("vkCreateAccelerationStructureKHR"));
    vkDestroyAccelerationStructureKHR = reinterpret_cast<PFN_vkDestroyAccelerationStructureKHR>(
      load("vkDestroyAccelerationStructureKHR"));
    vkGetAccelerationStructureBuildSizesKHR =
      reinterpret_cast<PFN_vkGetAccelerationStructureBuildSizesKHR>(
        load("vkGetAccelerationStructureBuildSizesKHR"));
    vkCmdBuildAccelerationStructuresKHR =
      reinterpret_cast<PFN_vkCmdBuildAccelerationStructuresKHR>(
        load("vkCmdBuildAccelerationStructuresKHR"));
    vkGetAccelerationStructureDeviceAddressKHR =
      reinterpret_cast<PFN_vkGetAccelerationStructureDeviceAddressKHR>(
        load("vkGetAccelerationStructureDeviceAddressKHR"));
    if (!vkCreateAccelerationStructureKHR || !vkDestroyAccelerationStructureKHR ||
        !vkGetAccelerationStructureBuildSizesKHR || !vkCmdBuildAccelerationStructuresKHR ||
        !vkGetAccelerationStructureDeviceAddressKHR)
    {
      throw std::runtime_error("Vulkan RT: missing acceleration-structure entry points");
    }
  }

  VkCommandBuffer begin_once()
  {
    VkCommandBufferAllocateInfo ai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    ai.commandPool = cmd_pool;
    ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    ai.commandBufferCount = 1;
    VkCommandBuffer c{};
    check(vkAllocateCommandBuffers(device, &ai, &c), "vkAllocateCommandBuffers");
    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    check(vkBeginCommandBuffer(c, &bi), "vkBeginCommandBuffer");
    return c;
  }

  void submit_wait(VkCommandBuffer c)
  {
    check(vkEndCommandBuffer(c), "vkEndCommandBuffer");
    VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    si.commandBufferCount = 1;
    si.pCommandBuffers = &c;
    check(vkResetFences(device, 1, &fence), "vkResetFences");
    check(vkQueueSubmit(queue, 1, &si, fence), "vkQueueSubmit");
    check(vkWaitForFences(device, 1, &fence, VK_TRUE, UINT64_MAX), "vkWaitForFences");
    vkFreeCommandBuffers(device, cmd_pool, 1, &c);
  }

  void update_descriptors()
  {
    VkWriteDescriptorSetAccelerationStructureKHR as_info{
      VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR};
    as_info.accelerationStructureCount = 1;
    as_info.pAccelerationStructures = &tlas;

    VkWriteDescriptorSet w0{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    w0.pNext = &as_info;
    w0.dstSet = dset;
    w0.dstBinding = 0;
    w0.descriptorCount = 1;
    w0.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;

    VkDescriptorBufferInfo ray_info{rays.buffer, 0, VK_WHOLE_SIZE};
    VkWriteDescriptorSet w1{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    w1.dstSet = dset;
    w1.dstBinding = 1;
    w1.descriptorCount = 1;
    w1.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    w1.pBufferInfo = &ray_info;

    VkDescriptorBufferInfo hit_info{hits.buffer, 0, VK_WHOLE_SIZE};
    VkWriteDescriptorSet w2{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    w2.dstSet = dset;
    w2.dstBinding = 2;
    w2.descriptorCount = 1;
    w2.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    w2.pBufferInfo = &hit_info;

    VkWriteDescriptorSet writes[] = {w0, w1, w2};
    vkUpdateDescriptorSets(device, 3, writes, 0, nullptr);
  }

  void ensure_ray_capacity(size_t n)
  {
    if (n <= ray_capacity) {
      return;
    }
    size_t cap = ray_capacity == 0 ? 131072 : ray_capacity;
    while (cap < n) {
      cap *= 2;
    }
    vkDeviceWaitIdle(device);
    destroy_buffer(rays);
    destroy_buffer(hits);
    const VkBufferUsageFlags ssbo =
      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
    const VkMemoryPropertyFlags host =
      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    rays = create_buffer(cap * sizeof(Ray), ssbo, host, true);
    hits = create_buffer(cap * sizeof(Hit), ssbo, host, true);
    ray_capacity = cap;
    update_descriptors();
  }
};

VulkanMeshRaycaster::~VulkanMeshRaycaster() = default;

std::unique_ptr<VulkanMeshRaycaster> VulkanMeshRaycaster::create(const TriangleMesh & mesh)
{
  auto out = std::unique_ptr<VulkanMeshRaycaster>(new VulkanMeshRaycaster());
  out->init(mesh);
  return out;
}

void VulkanMeshRaycaster::init(const TriangleMesh & mesh)
{
  if (mesh.indices.empty()) {
    throw std::runtime_error("Vulkan RT: empty mesh");
  }
  triangle_count_ = mesh.triangle_count();
  impl_ = std::make_unique<Impl>();
  Impl & I = *impl_;

  VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
  app.pApplicationName = "mesh_pf";
  app.apiVersion = VK_API_VERSION_1_2;

  VkInstanceCreateInfo ici{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
  ici.pApplicationInfo = &app;
  check(vkCreateInstance(&ici, nullptr, &I.instance), "vkCreateInstance");

  uint32_t nphys = 0;
  vkEnumeratePhysicalDevices(I.instance, &nphys, nullptr);
  if (nphys == 0) {
    throw std::runtime_error("Vulkan RT: no physical devices");
  }
  std::vector<VkPhysicalDevice> phys(nphys);
  vkEnumeratePhysicalDevices(I.instance, &nphys, phys.data());

  const char * dev_exts[] = {
    VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME,
    VK_KHR_RAY_QUERY_EXTENSION_NAME,
    VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME,
  };

  bool found = false;
  for (VkPhysicalDevice p : phys) {
    uint32_t next = 0;
    vkEnumerateDeviceExtensionProperties(p, nullptr, &next, nullptr);
    std::vector<VkExtensionProperties> exts(next);
    vkEnumerateDeviceExtensionProperties(p, nullptr, &next, exts.data());
    bool ok_ext = true;
    for (const char * e : dev_exts) {
      if (!has_ext(exts, e)) {
        ok_ext = false;
        break;
      }
    }
    if (!ok_ext) {
      continue;
    }

    VkPhysicalDeviceRayQueryFeaturesKHR rq{
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR};
    VkPhysicalDeviceAccelerationStructureFeaturesKHR asf{
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR};
    asf.pNext = &rq;
    VkPhysicalDeviceBufferDeviceAddressFeatures bda{
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_BUFFER_DEVICE_ADDRESS_FEATURES};
    bda.pNext = &asf;
    VkPhysicalDeviceFeatures2 f2{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2};
    f2.pNext = &bda;
    vkGetPhysicalDeviceFeatures2(p, &f2);
    if (!rq.rayQuery || !asf.accelerationStructure || !bda.bufferDeviceAddress) {
      continue;
    }

    uint32_t nq = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(p, &nq, nullptr);
    std::vector<VkQueueFamilyProperties> qf(nq);
    vkGetPhysicalDeviceQueueFamilyProperties(p, &nq, qf.data());
    int qidx = -1;
    for (uint32_t i = 0; i < nq; ++i) {
      if (qf[i].queueFlags & VK_QUEUE_COMPUTE_BIT) {
        qidx = static_cast<int>(i);
        if (!(qf[i].queueFlags & VK_QUEUE_GRAPHICS_BIT)) {
          break;
        }
      }
    }
    if (qidx < 0) {
      continue;
    }

    I.phys = p;
    I.queue_family = static_cast<uint32_t>(qidx);
    found = true;
    break;
  }
  if (!found) {
    throw std::runtime_error(
      "Vulkan RT: no device with rayQuery + accelerationStructure + compute");
  }

  VkPhysicalDeviceAccelerationStructurePropertiesKHR as_props{
    VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_PROPERTIES_KHR};
  VkPhysicalDeviceProperties2 p2{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2};
  p2.pNext = &as_props;
  vkGetPhysicalDeviceProperties2(I.phys, &p2);
  I.scratch_align = std::max(256u, as_props.minAccelerationStructureScratchOffsetAlignment);

  float qpri = 1.0f;
  VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
  qci.queueFamilyIndex = I.queue_family;
  qci.queueCount = 1;
  qci.pQueuePriorities = &qpri;

  VkPhysicalDeviceRayQueryFeaturesKHR rq_en{
    VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR};
  rq_en.rayQuery = VK_TRUE;
  VkPhysicalDeviceAccelerationStructureFeaturesKHR as_en{
    VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR};
  as_en.accelerationStructure = VK_TRUE;
  as_en.pNext = &rq_en;
  VkPhysicalDeviceBufferDeviceAddressFeatures bda_en{
    VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_BUFFER_DEVICE_ADDRESS_FEATURES};
  bda_en.bufferDeviceAddress = VK_TRUE;
  bda_en.pNext = &as_en;

  VkDeviceCreateInfo dci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
  dci.pNext = &bda_en;
  dci.queueCreateInfoCount = 1;
  dci.pQueueCreateInfos = &qci;
  dci.enabledExtensionCount = static_cast<uint32_t>(sizeof(dev_exts) / sizeof(dev_exts[0]));
  dci.ppEnabledExtensionNames = dev_exts;
  check(vkCreateDevice(I.phys, &dci, nullptr, &I.device), "vkCreateDevice");

  vkGetDeviceQueue(I.device, I.queue_family, 0, &I.queue);
  I.load_as_fns();

  VkCommandPoolCreateInfo pci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
  pci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
  pci.queueFamilyIndex = I.queue_family;
  check(vkCreateCommandPool(I.device, &pci, nullptr, &I.cmd_pool), "vkCreateCommandPool");

  VkFenceCreateInfo fi{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
  fi.flags = VK_FENCE_CREATE_SIGNALED_BIT;
  check(vkCreateFence(I.device, &fi, nullptr, &I.fence), "vkCreateFence");

  VkCommandBufferAllocateInfo cai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
  cai.commandPool = I.cmd_pool;
  cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  cai.commandBufferCount = 1;
  check(vkAllocateCommandBuffers(I.device, &cai, &I.cmd), "vkAllocateCommandBuffers");

  const VkMemoryPropertyFlags host =
    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
  const VkBufferUsageFlags as_input =
    VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
    VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
    VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  const VkBufferUsageFlags as_store =
    VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
    VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
  const VkBufferUsageFlags scratch_usage =
    VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;

  const VkDeviceSize vbytes = mesh.vertices.size() * sizeof(float);
  const VkDeviceSize ibytes = mesh.indices.size() * sizeof(uint32_t);
  I.vertex = I.create_buffer(vbytes, as_input, host, true);
  I.index = I.create_buffer(ibytes, as_input, host, true);
  std::memcpy(I.vertex.mapped, mesh.vertices.data(), vbytes);
  std::memcpy(I.index.mapped, mesh.indices.data(), ibytes);

  VkAccelerationStructureGeometryKHR geo{
    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR};
  geo.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
  geo.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
  geo.geometry.triangles.sType =
    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
  geo.geometry.triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
  geo.geometry.triangles.vertexData.deviceAddress = I.vertex.address;
  geo.geometry.triangles.vertexStride = 3 * sizeof(float);
  geo.geometry.triangles.maxVertex = static_cast<uint32_t>(mesh.vertices.size() / 3 - 1);
  geo.geometry.triangles.indexType = VK_INDEX_TYPE_UINT32;
  geo.geometry.triangles.indexData.deviceAddress = I.index.address;

  VkAccelerationStructureBuildGeometryInfoKHR blas_info{
    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR};
  blas_info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
  blas_info.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
  blas_info.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
  blas_info.geometryCount = 1;
  blas_info.pGeometries = &geo;

  const uint32_t prim_count = static_cast<uint32_t>(mesh.indices.size() / 3);
  VkAccelerationStructureBuildSizesInfoKHR blas_size{
    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR};
  I.vkGetAccelerationStructureBuildSizesKHR(
    I.device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
    &blas_info, &prim_count, &blas_size);

  I.blas_store = I.create_buffer(blas_size.accelerationStructureSize, as_store, host, false);
  VkAccelerationStructureCreateInfoKHR blas_ci{
    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR};
  blas_ci.buffer = I.blas_store.buffer;
  blas_ci.size = blas_size.accelerationStructureSize;
  blas_ci.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
  check(
    I.vkCreateAccelerationStructureKHR(I.device, &blas_ci, nullptr, &I.blas),
    "vkCreateAccelerationStructureKHR(BLAS)");

  VkDeviceSize scratch_sz = blas_size.buildScratchSize;
  scratch_sz = (scratch_sz + I.scratch_align - 1) & ~(static_cast<VkDeviceSize>(I.scratch_align) - 1);

  VkAccelerationStructureDeviceAddressInfoKHR blas_addr_i{
    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR};
  blas_addr_i.accelerationStructure = I.blas;
  const uint64_t blas_ref = I.vkGetAccelerationStructureDeviceAddressKHR(I.device, &blas_addr_i);

  VkTransformMatrixKHR ident{};
  ident.matrix[0][0] = 1.0f;
  ident.matrix[1][1] = 1.0f;
  ident.matrix[2][2] = 1.0f;
  VkAccelerationStructureInstanceKHR inst{};
  inst.transform = ident;
  inst.mask = 0xFF;
  inst.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
  inst.accelerationStructureReference = blas_ref;

  I.tlas_instances = I.create_buffer(
    sizeof(inst),
    VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
      VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
    host, true);
  std::memcpy(I.tlas_instances.mapped, &inst, sizeof(inst));

  VkAccelerationStructureGeometryKHR tlas_geo{
    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR};
  tlas_geo.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
  tlas_geo.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
  tlas_geo.geometry.instances.sType =
    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
  tlas_geo.geometry.instances.arrayOfPointers = VK_FALSE;
  tlas_geo.geometry.instances.data.deviceAddress = I.tlas_instances.address;

  VkAccelerationStructureBuildGeometryInfoKHR tlas_info{
    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR};
  tlas_info.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
  tlas_info.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
  tlas_info.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
  tlas_info.geometryCount = 1;
  tlas_info.pGeometries = &tlas_geo;

  const uint32_t inst_count = 1;
  VkAccelerationStructureBuildSizesInfoKHR tlas_size{
    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR};
  I.vkGetAccelerationStructureBuildSizesKHR(
    I.device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
    &tlas_info, &inst_count, &tlas_size);

  I.tlas_store = I.create_buffer(tlas_size.accelerationStructureSize, as_store, host, false);
  VkAccelerationStructureCreateInfoKHR tlas_ci{
    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR};
  tlas_ci.buffer = I.tlas_store.buffer;
  tlas_ci.size = tlas_size.accelerationStructureSize;
  tlas_ci.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
  check(
    I.vkCreateAccelerationStructureKHR(I.device, &tlas_ci, nullptr, &I.tlas),
    "vkCreateAccelerationStructureKHR(TLAS)");

  scratch_sz = std::max(scratch_sz, tlas_size.buildScratchSize);
  scratch_sz = (scratch_sz + I.scratch_align - 1) & ~(static_cast<VkDeviceSize>(I.scratch_align) - 1);
  I.scratch = I.create_buffer(scratch_sz, scratch_usage, host, false);

  blas_info.dstAccelerationStructure = I.blas;
  blas_info.scratchData.deviceAddress = I.scratch.address;
  tlas_info.dstAccelerationStructure = I.tlas;
  tlas_info.scratchData.deviceAddress = I.scratch.address;

  VkAccelerationStructureBuildRangeInfoKHR blas_range{};
  blas_range.primitiveCount = prim_count;
  VkAccelerationStructureBuildRangeInfoKHR tlas_range{};
  tlas_range.primitiveCount = 1;
  const VkAccelerationStructureBuildRangeInfoKHR * blas_ranges[] = {&blas_range};
  const VkAccelerationStructureBuildRangeInfoKHR * tlas_ranges[] = {&tlas_range};

  VkCommandBuffer build = I.begin_once();
  I.vkCmdBuildAccelerationStructuresKHR(build, 1, &blas_info, blas_ranges);
  VkMemoryBarrier bar{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
  bar.srcAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
  bar.dstAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR;
  vkCmdPipelineBarrier(
    build,
    VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR,
    VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR,
    0, 1, &bar, 0, nullptr, 0, nullptr);
  I.vkCmdBuildAccelerationStructuresKHR(build, 1, &tlas_info, tlas_ranges);
  I.submit_wait(build);

  VkShaderModuleCreateInfo sm{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
  sm.codeSize = sizeof(kMeshRayquerySpv);
  sm.pCode = kMeshRayquerySpv;
  check(vkCreateShaderModule(I.device, &sm, nullptr, &I.shader), "vkCreateShaderModule");

  VkDescriptorSetLayoutBinding binds[3]{};
  binds[0].binding = 0;
  binds[0].descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
  binds[0].descriptorCount = 1;
  binds[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  binds[1].binding = 1;
  binds[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  binds[1].descriptorCount = 1;
  binds[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  binds[2].binding = 2;
  binds[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  binds[2].descriptorCount = 1;
  binds[2].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  VkDescriptorSetLayoutCreateInfo dl{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
  dl.bindingCount = 3;
  dl.pBindings = binds;
  check(
    vkCreateDescriptorSetLayout(I.device, &dl, nullptr, &I.dset_layout),
    "vkCreateDescriptorSetLayout");

  VkPushConstantRange pcr{};
  pcr.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  pcr.size = sizeof(uint32_t);

  VkPipelineLayoutCreateInfo pl{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
  pl.setLayoutCount = 1;
  pl.pSetLayouts = &I.dset_layout;
  pl.pushConstantRangeCount = 1;
  pl.pPushConstantRanges = &pcr;
  check(vkCreatePipelineLayout(I.device, &pl, nullptr, &I.pipe_layout), "vkCreatePipelineLayout");

  VkPipelineShaderStageCreateInfo stage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
  stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
  stage.module = I.shader;
  stage.pName = "main";
  VkComputePipelineCreateInfo cp{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
  cp.stage = stage;
  cp.layout = I.pipe_layout;
  check(
    vkCreateComputePipelines(I.device, VK_NULL_HANDLE, 1, &cp, nullptr, &I.pipeline),
    "vkCreateComputePipelines");

  VkDescriptorPoolSize pool_sz[2]{};
  pool_sz[0].type = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
  pool_sz[0].descriptorCount = 1;
  pool_sz[1].type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  pool_sz[1].descriptorCount = 2;
  VkDescriptorPoolCreateInfo dp{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
  dp.maxSets = 1;
  dp.poolSizeCount = 2;
  dp.pPoolSizes = pool_sz;
  check(vkCreateDescriptorPool(I.device, &dp, nullptr, &I.dset_pool), "vkCreateDescriptorPool");

  VkDescriptorSetAllocateInfo dsa{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
  dsa.descriptorPool = I.dset_pool;
  dsa.descriptorSetCount = 1;
  dsa.pSetLayouts = &I.dset_layout;
  check(vkAllocateDescriptorSets(I.device, &dsa, &I.dset), "vkAllocateDescriptorSets");

  I.ensure_ray_capacity(131072);

  std::fprintf(
    stderr,
    "[mesh_pf] Vulkan RT ready: %s triangles=%zu\n",
    p2.properties.deviceName, triangle_count_);
}

void VulkanMeshRaycaster::batchIntersect(const Ray * rays, Hit * hits, size_t n)
{
  if (!impl_ || n == 0) {
    return;
  }
  Impl & I = *impl_;
  I.ensure_ray_capacity(n);
  std::memcpy(I.rays.mapped, rays, n * sizeof(Ray));

  check(vkResetFences(I.device, 1, &I.fence), "vkResetFences");
  check(vkResetCommandBuffer(I.cmd, 0), "vkResetCommandBuffer");
  VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
  bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
  check(vkBeginCommandBuffer(I.cmd, &bi), "vkBeginCommandBuffer");

  VkMemoryBarrier host_to_shader{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
  host_to_shader.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT;
  host_to_shader.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
  vkCmdPipelineBarrier(
    I.cmd,
    VK_PIPELINE_STAGE_HOST_BIT,
    VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
    0, 1, &host_to_shader, 0, nullptr, 0, nullptr);

  vkCmdBindPipeline(I.cmd, VK_PIPELINE_BIND_POINT_COMPUTE, I.pipeline);
  vkCmdBindDescriptorSets(
    I.cmd, VK_PIPELINE_BIND_POINT_COMPUTE, I.pipe_layout, 0, 1, &I.dset, 0, nullptr);
  const uint32_t count = static_cast<uint32_t>(n);
  vkCmdPushConstants(
    I.cmd, I.pipe_layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(uint32_t), &count);
  const uint32_t groups = (count + 63u) / 64u;
  vkCmdDispatch(I.cmd, groups, 1, 1);

  VkMemoryBarrier shader_to_host{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
  shader_to_host.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
  shader_to_host.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
  vkCmdPipelineBarrier(
    I.cmd,
    VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
    VK_PIPELINE_STAGE_HOST_BIT,
    0, 1, &shader_to_host, 0, nullptr, 0, nullptr);

  check(vkEndCommandBuffer(I.cmd), "vkEndCommandBuffer");
  VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
  si.commandBufferCount = 1;
  si.pCommandBuffers = &I.cmd;
  check(vkQueueSubmit(I.queue, 1, &si, I.fence), "vkQueueSubmit");
  check(vkWaitForFences(I.device, 1, &I.fence, VK_TRUE, UINT64_MAX), "vkWaitForFences");

  std::memcpy(hits, I.hits.mapped, n * sizeof(Hit));
}

}  // namespace composite_sensing
