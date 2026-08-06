#include "composite_sensing/embree_mesh_map.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

namespace composite_sensing
{

namespace
{

void embree_error(void *, RTCError, const char * str)
{
  // Logged by caller if needed; keep quiet in hot path.
  (void)str;
}

}  // namespace

std::unique_ptr<EmbreeMeshMap> EmbreeMeshMap::load(const std::string & path)
{
  Assimp::Importer importer;
  const aiScene * scene = importer.ReadFile(
    path,
    aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_PreTransformVertices);
  if (!scene || !scene->mMeshes || scene->mNumMeshes == 0) {
    throw std::runtime_error("EmbreeMeshMap: failed to load mesh: " + path);
  }

  auto map = std::make_unique<EmbreeMeshMap>();
  for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
    const aiMesh * mesh = scene->mMeshes[m];
    const uint32_t vbase = static_cast<uint32_t>(map->vertices.size() / 3);
    for (unsigned i = 0; i < mesh->mNumVertices; ++i) {
      map->vertices.push_back(mesh->mVertices[i].x);
      map->vertices.push_back(mesh->mVertices[i].y);
      map->vertices.push_back(mesh->mVertices[i].z);
    }
    for (unsigned f = 0; f < mesh->mNumFaces; ++f) {
      const aiFace & face = mesh->mFaces[f];
      if (face.mNumIndices != 3) {
        continue;
      }
      map->indices.push_back(vbase + face.mIndices[0]);
      map->indices.push_back(vbase + face.mIndices[1]);
      map->indices.push_back(vbase + face.mIndices[2]);
    }
  }
  if (map->indices.empty()) {
    throw std::runtime_error("EmbreeMeshMap: no triangles in " + path);
  }

  map->device = rtcNewDevice(nullptr);
  if (!map->device) {
    throw std::runtime_error("EmbreeMeshMap: rtcNewDevice failed");
  }
  rtcSetDeviceErrorFunction(map->device, embree_error, nullptr);
  map->scene = rtcNewScene(map->device);
  RTCGeometry geom = rtcNewGeometry(map->device, RTC_GEOMETRY_TYPE_TRIANGLE);

  float * vb = static_cast<float *>(rtcSetNewGeometryBuffer(
      geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
      3 * sizeof(float), map->vertices.size() / 3));
  std::copy(map->vertices.begin(), map->vertices.end(), vb);

  uint32_t * ib = static_cast<uint32_t *>(rtcSetNewGeometryBuffer(
      geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
      3 * sizeof(uint32_t), map->indices.size() / 3));
  std::copy(map->indices.begin(), map->indices.end(), ib);

  rtcCommitGeometry(geom);
  rtcAttachGeometry(map->scene, geom);
  rtcReleaseGeometry(geom);
  rtcCommitScene(map->scene);
  return map;
}

float EmbreeMeshMap::raycast(
  float ox, float oy, float oz,
  float dx, float dy, float dz,
  float max_range) const
{
  RTCRayHit rh{};
  rh.ray.org_x = ox;
  rh.ray.org_y = oy;
  rh.ray.org_z = oz;
  rh.ray.dir_x = dx;
  rh.ray.dir_y = dy;
  rh.ray.dir_z = dz;
  rh.ray.tnear = 0.0f;
  rh.ray.tfar = max_range;
  rh.ray.mask = -1;
  rh.ray.flags = 0;
  rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;

  RTCIntersectArguments args;
  rtcInitIntersectArguments(&args);
  rtcIntersect1(scene, &rh, &args);

  if (rh.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
    return -1.0f;
  }
  return rh.ray.tfar;
}

}  // namespace composite_sensing
