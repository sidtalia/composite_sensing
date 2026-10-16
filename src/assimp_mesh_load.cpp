#include "composite_sensing/assimp_mesh_load.hpp"

#include <stdexcept>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

namespace composite_sensing
{

TriangleMesh TriangleMesh::load(const std::string & path)
{
  Assimp::Importer importer;
  const aiScene * scene = importer.ReadFile(
    path,
    aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_PreTransformVertices);
  if (!scene || !scene->mMeshes || scene->mNumMeshes == 0) {
    throw std::runtime_error("TriangleMesh: failed to load mesh: " + path);
  }

  TriangleMesh mesh;
  for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
    const aiMesh * aimesh = scene->mMeshes[m];
    const uint32_t vbase = static_cast<uint32_t>(mesh.vertices.size() / 3);
    for (unsigned i = 0; i < aimesh->mNumVertices; ++i) {
      mesh.vertices.push_back(aimesh->mVertices[i].x);
      mesh.vertices.push_back(aimesh->mVertices[i].y);
      mesh.vertices.push_back(aimesh->mVertices[i].z);
    }
    for (unsigned f = 0; f < aimesh->mNumFaces; ++f) {
      const aiFace & face = aimesh->mFaces[f];
      if (face.mNumIndices != 3) {
        continue;
      }
      mesh.indices.push_back(vbase + face.mIndices[0]);
      mesh.indices.push_back(vbase + face.mIndices[1]);
      mesh.indices.push_back(vbase + face.mIndices[2]);
    }
  }
  if (mesh.indices.empty()) {
    throw std::runtime_error("TriangleMesh: no triangles in " + path);
  }
  return mesh;
}

}  // namespace composite_sensing
