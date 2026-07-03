#include "MeshUtil.h"
#include "SceneDrawables.h"
#include <algorithm>
#include <array>
#include <vector>

using namespace std;
using namespace cnoid;


MeshConsistencyInfo cnoid::checkMeshConsistency(const SgMesh* mesh)
{
    MeshConsistencyInfo info;

    if(!mesh){
        return info;
    }

    auto vertices = mesh->vertices();
    if(vertices){
        info.numVertices = static_cast<int>(vertices->size());
        vector<array<float, 3>> vertexKeys;
        vertexKeys.reserve(info.numVertices);
        for(const auto& v : *vertices){
            vertexKeys.push_back({ v.x(), v.y(), v.z() });
        }
        sort(vertexKeys.begin(), vertexKeys.end());
        info.numUniqueVertices =
            static_cast<int>(unique(vertexKeys.begin(), vertexKeys.end()) - vertexKeys.begin());
    }

    if(mesh->hasTriangles()){
        const auto& indices = mesh->triangleVertices();
        info.numTriangles = mesh->numTriangles();
        for(int i = 0; i < info.numTriangles; ++i){
            int i0 = indices[i * 3];
            int i1 = indices[i * 3 + 1];
            int i2 = indices[i * 3 + 2];
            if(i0 < 0 || i0 >= info.numVertices ||
               i1 < 0 || i1 >= info.numVertices ||
               i2 < 0 || i2 >= info.numVertices){
                ++info.numInvalidTriangles;
                continue;
            }
            if(i0 == i1 || i1 == i2 || i2 == i0){
                ++info.numDegenerateTriangles;
                continue;
            }
            if(vertices){
                const Vector3f& v0 = (*vertices)[i0];
                const Vector3f& v1 = (*vertices)[i1];
                const Vector3f& v2 = (*vertices)[i2];
                if((v1 - v0).cross(v2 - v0).squaredNorm() <= 1.0e-20f){
                    ++info.numDegenerateTriangles;
                }
            }
        }
    }

    return info;
}
