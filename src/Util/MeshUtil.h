#ifndef CNOID_UTIL_MESH_UTIL_H
#define CNOID_UTIL_MESH_UTIL_H

#include "SceneDrawables.h"
#include "exportdecl.h"

namespace cnoid {

struct MeshConsistencyInfo
{
    int numVertices = 0;
    int numUniqueVertices = 0;
    int numTriangles = 0;
    int numInvalidTriangles = 0;
    int numDegenerateTriangles = 0;
};

CNOID_EXPORT MeshConsistencyInfo checkMeshConsistency(const SgMesh* mesh);

//! Applies the given transform to all the vertex positions in place.
CNOID_EXPORT void transformVertices(SgVertexArray& vertices, const Affine3f& T);

/**
   Applies the given transform to all the normal vectors in place. Normals are
   transformed by the inverse transpose of the linear part and normalized, which
   differs from the vertex transform when the transform contains a reflection or
   a non-uniform scale.
*/
CNOID_EXPORT void transformNormals(SgNormalArray& normals, const Affine3f& T);

/**
   Checks whether the triangle winding of the mesh is consistent with the directions
   of its vertex normals. The check is based on the sum of the dot products between
   the face normals calculated from the winding and the corresponding vertex normals.
   @return Positive if the winding is consistent with the normals, negative if it is
   inverted, and zero if it cannot be determined (no normals, no valid triangles, etc.)
*/
CNOID_EXPORT int checkTriangleWindingConsistency(const SgMesh* mesh);

/**
   Reverses the winding of all the triangles by swapping the second and third indices
   of each triangle. Since SgMesh has separate index arrays for normals, colors, and
   texture coordinates, those arrays are reordered in the same way when they are used.
*/
CNOID_EXPORT void flipTriangleWinding(SgMesh* mesh);

/**
   Bakes the given transform into the mesh vertices and normals so that the transform
   does not have to be kept in the scene graph. This is mainly used by scene loaders
   to remove a reflection (a transform whose determinant is negative), which cannot be
   correctly handled by the downstream subsystems assuming rotation-only transforms.
   When the transform contains a reflection, the triangle winding is flipped if
   necessary; whether to flip is decided from the transformed normals when they are
   available because some source files already compensate the winding for a reflected
   node. Note that the vertex, normal, and index arrays shared with other meshes must
   be duplicated by the caller beforehand.
*/
CNOID_EXPORT void bakeTransformIntoMesh(SgMesh* mesh, const Affine3f& T);

}

#endif
