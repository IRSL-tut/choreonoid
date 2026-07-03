#ifndef CNOID_UTIL_MESH_UTIL_H
#define CNOID_UTIL_MESH_UTIL_H

#include "exportdecl.h"

namespace cnoid {

class SgMesh;

struct MeshConsistencyInfo
{
    int numVertices = 0;
    int numUniqueVertices = 0;
    int numTriangles = 0;
    int numInvalidTriangles = 0;
    int numDegenerateTriangles = 0;
};

CNOID_EXPORT MeshConsistencyInfo checkMeshConsistency(const SgMesh* mesh);

}

#endif
