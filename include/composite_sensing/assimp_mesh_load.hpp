#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace composite_sensing
{

/** Packed triangle mesh (xyz vertices, uint32 triangle indices). */
struct TriangleMesh
{
  std::vector<float> vertices;
  std::vector<uint32_t> indices;

  size_t triangle_count() const { return indices.size() / 3; }

  /** Load with Assimp (dae/obj/ply/…). Throws on failure. */
  static TriangleMesh load(const std::string & path);
};

}  // namespace composite_sensing
