#include "composite_sensing/embree_mesh_map.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace composite_sensing
{

namespace
{

void embree_error(void *, RTCError, const char * str)
{
  (void)str;
}

void intersect1(
  RTCScene scene, const Ray & ray, Hit & hit)
{
  RTCRayHit rh{};
  rh.ray.org_x = ray.ox;
  rh.ray.org_y = ray.oy;
  rh.ray.org_z = ray.oz;
  rh.ray.dir_x = ray.dx;
  rh.ray.dir_y = ray.dy;
  rh.ray.dir_z = ray.dz;
  rh.ray.tnear = ray.tmin;
  rh.ray.tfar = ray.tmax;
  rh.ray.mask = static_cast<unsigned>(-1);
  rh.ray.flags = 0;
  rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;

  RTCIntersectArguments args;
  rtcInitIntersectArguments(&args);
  rtcIntersect1(scene, &rh, &args);

  if (rh.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
    hit.t = -1.0f;
    hit.hit = 0;
  } else {
    hit.t = rh.ray.tfar;
    hit.hit = 1;
  }
}

void store_plane(const Ray & ray, const float t, const float ngx, const float ngy,
  const float ngz, HitPlane & hit)
{
  hit.t = t;
  hit.hit = 1;
  float nx = ngx;
  float ny = ngy;
  float nz = ngz;
  const float n2 = nx * nx + ny * ny + nz * nz;
  if (n2 > 1e-12f) {
    const float inv = 1.0f / std::sqrt(n2);
    nx *= inv;
    ny *= inv;
    nz *= inv;
  } else {
    nx = 0.f;
    ny = 0.f;
    nz = 1.f;
  }
  if (nx * ray.dx + ny * ray.dy + nz * ray.dz > 0.f) {
    nx = -nx;
    ny = -ny;
    nz = -nz;
  }
  hit.nx = nx;
  hit.ny = ny;
  hit.nz = nz;
}

void intersect1_plane(RTCScene scene, const Ray & ray, HitPlane & hit)
{
  RTCRayHit rh{};
  rh.ray.org_x = ray.ox;
  rh.ray.org_y = ray.oy;
  rh.ray.org_z = ray.oz;
  rh.ray.dir_x = ray.dx;
  rh.ray.dir_y = ray.dy;
  rh.ray.dir_z = ray.dz;
  rh.ray.tnear = ray.tmin;
  rh.ray.tfar = ray.tmax;
  rh.ray.mask = static_cast<unsigned>(-1);
  rh.ray.flags = 0;
  rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;

  RTCIntersectArguments args;
  rtcInitIntersectArguments(&args);
  rtcIntersect1(scene, &rh, &args);

  if (rh.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
    hit.t = -1.0f;
    hit.hit = 0;
    hit.nx = 0.f;
    hit.ny = 0.f;
    hit.nz = 1.f;
  } else {
    store_plane(ray, rh.ray.tfar, rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z, hit);
  }
}

}  // namespace

EmbreeMeshMap::~EmbreeMeshMap()
{
  if (scene_) {
    rtcReleaseScene(scene_);
    scene_ = nullptr;
  }
  if (device_) {
    rtcReleaseDevice(device_);
    device_ = nullptr;
  }
}

std::unique_ptr<EmbreeMeshMap> EmbreeMeshMap::create(TriangleMesh mesh)
{
  if (mesh.indices.empty()) {
    throw std::runtime_error("EmbreeMeshMap: no triangles");
  }

  auto map = std::make_unique<EmbreeMeshMap>();
  map->vertices_ = std::move(mesh.vertices);
  map->indices_ = std::move(mesh.indices);

  map->device_ = rtcNewDevice(nullptr);
  if (!map->device_) {
    throw std::runtime_error("EmbreeMeshMap: rtcNewDevice failed");
  }
  rtcSetDeviceErrorFunction(map->device_, embree_error, nullptr);
  map->scene_ = rtcNewScene(map->device_);
  RTCGeometry geom = rtcNewGeometry(map->device_, RTC_GEOMETRY_TYPE_TRIANGLE);

  float * vb = static_cast<float *>(rtcSetNewGeometryBuffer(
      geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
      3 * sizeof(float), map->vertices_.size() / 3));
  std::copy(map->vertices_.begin(), map->vertices_.end(), vb);

  uint32_t * ib = static_cast<uint32_t *>(rtcSetNewGeometryBuffer(
      geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
      3 * sizeof(uint32_t), map->indices_.size() / 3));
  std::copy(map->indices_.begin(), map->indices_.end(), ib);

  rtcCommitGeometry(geom);
  rtcAttachGeometry(map->scene_, geom);
  rtcReleaseGeometry(geom);
  rtcCommitScene(map->scene_);
  return map;
}

std::unique_ptr<EmbreeMeshMap> EmbreeMeshMap::load(const std::string & path)
{
  return create(TriangleMesh::load(path));
}

void EmbreeMeshMap::batchIntersect(const Ray * rays, Hit * hits, size_t n)
{
  if (!scene_ || n == 0) {
    return;
  }

  RTCIntersectArguments args;
  rtcInitIntersectArguments(&args);

  size_t i = 0;
  for (; i + 4 <= n; i += 4) {
    RTCRayHit4 rh{};
    int valid[4] = {-1, -1, -1, -1};
    for (int k = 0; k < 4; ++k) {
      const Ray & r = rays[i + static_cast<size_t>(k)];
      rh.ray.org_x[k] = r.ox;
      rh.ray.org_y[k] = r.oy;
      rh.ray.org_z[k] = r.oz;
      rh.ray.dir_x[k] = r.dx;
      rh.ray.dir_y[k] = r.dy;
      rh.ray.dir_z[k] = r.dz;
      rh.ray.tnear[k] = r.tmin;
      rh.ray.tfar[k] = r.tmax;
      rh.ray.mask[k] = static_cast<unsigned>(-1);
      rh.ray.flags[k] = 0;
      rh.hit.geomID[k] = RTC_INVALID_GEOMETRY_ID;
    }
    rtcIntersect4(valid, scene_, &rh, &args);
    for (int k = 0; k < 4; ++k) {
      Hit & h = hits[i + static_cast<size_t>(k)];
      if (rh.hit.geomID[k] == RTC_INVALID_GEOMETRY_ID) {
        h.t = -1.0f;
        h.hit = 0;
      } else {
        h.t = rh.ray.tfar[k];
        h.hit = 1;
      }
    }
  }
  for (; i < n; ++i) {
    intersect1(scene_, rays[i], hits[i]);
  }
}

void EmbreeMeshMap::batchIntersectPlane(
  const Ray * rays, HitPlane * hits, size_t n)
{
  if (!scene_ || n == 0) {
    return;
  }

  RTCIntersectArguments args;
  rtcInitIntersectArguments(&args);

  size_t i = 0;
  for (; i + 4 <= n; i += 4) {
    RTCRayHit4 rh{};
    int valid[4] = {-1, -1, -1, -1};
    for (int k = 0; k < 4; ++k) {
      const Ray & r = rays[i + static_cast<size_t>(k)];
      rh.ray.org_x[k] = r.ox;
      rh.ray.org_y[k] = r.oy;
      rh.ray.org_z[k] = r.oz;
      rh.ray.dir_x[k] = r.dx;
      rh.ray.dir_y[k] = r.dy;
      rh.ray.dir_z[k] = r.dz;
      rh.ray.tnear[k] = r.tmin;
      rh.ray.tfar[k] = r.tmax;
      rh.ray.mask[k] = static_cast<unsigned>(-1);
      rh.ray.flags[k] = 0;
      rh.hit.geomID[k] = RTC_INVALID_GEOMETRY_ID;
    }
    rtcIntersect4(valid, scene_, &rh, &args);
    for (int k = 0; k < 4; ++k) {
      HitPlane & h = hits[i + static_cast<size_t>(k)];
      const Ray & r = rays[i + static_cast<size_t>(k)];
      if (rh.hit.geomID[k] == RTC_INVALID_GEOMETRY_ID) {
        h.t = -1.0f;
        h.hit = 0;
        h.nx = 0.f;
        h.ny = 0.f;
        h.nz = 1.f;
      } else {
        store_plane(
          r, rh.ray.tfar[k], rh.hit.Ng_x[k], rh.hit.Ng_y[k], rh.hit.Ng_z[k], h);
      }
    }
  }
  for (; i < n; ++i) {
    intersect1_plane(scene_, rays[i], hits[i]);
  }
}

}  // namespace composite_sensing
