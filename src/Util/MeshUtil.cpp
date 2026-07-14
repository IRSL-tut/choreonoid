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


void cnoid::transformVertices(SgVertexArray& vertices, const Affine3f& T)
{
    for(auto& v : vertices){
        v = T * v;
    }
}


void cnoid::transformNormals(SgNormalArray& normals, const Affine3f& T)
{
    const Matrix3f N = T.linear().inverse().transpose();
    for(auto& n : normals){
        n = N * n;
        const float norm = n.norm();
        if(norm > 0.0f){
            n /= norm;
        }
    }
}


int cnoid::checkTriangleWindingConsistency(const SgMesh* mesh)
{
    if(!mesh || !mesh->hasVertices() || !mesh->hasNormals() || !mesh->hasTriangles()){
        return 0;
    }
    const auto& vertices = *mesh->vertices();
    const auto& normals = *mesh->normals();
    const auto& triangleVertices = mesh->triangleVertices();
    const int numVertices = static_cast<int>(vertices.size());
    const int numNormals = static_cast<int>(normals.size());

    const SgIndexArray* normalIndices = nullptr;
    if(mesh->hasNormalIndices()){
        if(mesh->normalIndices().size() != triangleVertices.size()){
            return 0;
        }
        normalIndices = &mesh->normalIndices();
    }

    double dotSum = 0.0;
    int numCheckedFaces = 0;
    const int numTriangles = mesh->numTriangles();
    for(int i = 0; i < numTriangles; ++i){
        const int top = i * 3;
        const int v0 = triangleVertices[top];
        const int v1 = triangleVertices[top + 1];
        const int v2 = triangleVertices[top + 2];
        if(v0 < 0 || v0 >= numVertices ||
           v1 < 0 || v1 >= numVertices ||
           v2 < 0 || v2 >= numVertices){
            continue;
        }
        Vector3f faceNormal = (vertices[v1] - vertices[v0]).cross(vertices[v2] - vertices[v0]);
        if(faceNormal.squaredNorm() == 0.0f){
            continue;
        }
        const int n0 = normalIndices ? (*normalIndices)[top]     : v0;
        const int n1 = normalIndices ? (*normalIndices)[top + 1] : v1;
        const int n2 = normalIndices ? (*normalIndices)[top + 2] : v2;
        if(n0 < 0 || n0 >= numNormals ||
           n1 < 0 || n1 >= numNormals ||
           n2 < 0 || n2 >= numNormals){
            continue;
        }
        const Vector3f normal = normals[n0] + normals[n1] + normals[n2];
        if(normal.squaredNorm() == 0.0f){
            continue;
        }
        dotSum += faceNormal.normalized().dot(normal.normalized());
        ++numCheckedFaces;
    }

    if(numCheckedFaces == 0 || dotSum == 0.0){
        return 0;
    }
    return (dotSum > 0.0) ? 1 : -1;
}


void cnoid::flipTriangleWinding(SgMesh* mesh)
{
    if(!mesh){
        return;
    }
    auto flipIndices = [](SgIndexArray& indices){
        const size_t n = indices.size() / 3 * 3;
        for(size_t i = 0; i < n; i += 3){
            std::swap(indices[i + 1], indices[i + 2]);
        }
    };
    auto& triangleVertices = mesh->triangleVertices();
    const size_t numFaceVertexIndices = triangleVertices.size();
    flipIndices(triangleVertices);

    // The attribute index arrays are given per face vertex, so they must be
    // reordered in the same way. An array whose size does not match the face
    // vertex indices is left as is because its indexing scheme is unknown.
    if(mesh->normalIndices().size() == numFaceVertexIndices){
        flipIndices(mesh->normalIndices());
    }
    if(mesh->colorIndices().size() == numFaceVertexIndices){
        flipIndices(mesh->colorIndices());
    }
    if(mesh->texCoordIndices().size() == numFaceVertexIndices){
        flipIndices(mesh->texCoordIndices());
    }
}


void cnoid::bakeTransformIntoMesh(SgMesh* mesh, const Affine3f& T)
{
    if(!mesh){
        return;
    }
    if(mesh->hasVertices()){
        transformVertices(*mesh->vertices(), T);
    }
    if(mesh->hasNormals()){
        transformNormals(*mesh->normals(), T);
    }
    if(T.linear().determinant() < 0.0f){
        // Some source files already compensate the triangle winding for a
        // reflected node, so whether to flip is decided from the transformed
        // normals when they are available.
        bool doFlipWinding = true;
        if(mesh->hasNormals()){
            const int consistency = checkTriangleWindingConsistency(mesh);
            if(consistency != 0){
                doFlipWinding = (consistency < 0);
            }
        }
        if(doFlipWinding){
            flipTriangleWinding(mesh);
        }
    }
    mesh->invalidateBoundingBox();
}
