#include "MeshDistanceQuery.h"
#include "SceneDrawables.h"
#include <algorithm>

using namespace std;
using namespace cnoid;

namespace {

// The tree is built by splitting the triangle range at the median, so its
// depth is bounded by about log2(numTriangles / MaxNumLeafTriangles) and the
// fixed traversal stack below is far larger than any reachable depth.
constexpr int MaxNumLeafTriangles = 8;
constexpr int TraversalStackSize = 128;

/**
   Closest point to p on triangle (a, b, c). This is the standard closed-form
   solution that classifies the barycentric region the point projects into
   (vertex, edge, or face) and clamps accordingly.
*/
Vector3 closestPointOnTriangle(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c)
{
    const Vector3 ab = b - a;
    const Vector3 ac = c - a;
    const Vector3 ap = p - a;
    const double d1 = ab.dot(ap);
    const double d2 = ac.dot(ap);
    if(d1 <= 0.0 && d2 <= 0.0){
        return a;
    }

    const Vector3 bp = p - b;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);
    if(d3 >= 0.0 && d4 <= d3){
        return b;
    }

    const double vc = d1 * d4 - d3 * d2;
    if(vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0){
        const double v = d1 / (d1 - d3);
        return a + v * ab;
    }

    const Vector3 cp = p - c;
    const double d5 = ab.dot(cp);
    const double d6 = ac.dot(cp);
    if(d6 >= 0.0 && d5 <= d6){
        return c;
    }

    const double vb = d5 * d2 - d1 * d6;
    if(vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0){
        const double w = d2 / (d2 - d6);
        return a + w * ac;
    }

    const double va = d3 * d6 - d5 * d4;
    if(va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0){
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + w * (c - b);
    }

    // The face region. The denominator va + vb + vc equals the squared norm
    // of the triangle's cross product, and reaching here requires all three
    // to be positive, so the division is safe.
    const double denom = 1.0 / (va + vb + vc);
    const double v = vb * denom;
    const double w = vc * denom;
    return a + v * ab + w * ac;
}

}


MeshDistanceQuery::MeshDistanceQuery()
    : rootNodeIndex_(-1)
{

}


MeshDistanceQuery::MeshDistanceQuery(const SgMesh* mesh)
    : rootNodeIndex_(-1)
{
    setMesh(mesh);
}


void MeshDistanceQuery::clear()
{
    triangles_.clear();
    nodes_.clear();
    rootNodeIndex_ = -1;
}


void MeshDistanceQuery::setMesh(const SgMesh* mesh)
{
    clear();

    if(!mesh || !mesh->hasVertices()){
        return;
    }

    const SgVertexArray& vertices = *mesh->vertices();
    const int numVertices = static_cast<int>(vertices.size());
    const int numTriangles = mesh->numTriangles();
    triangles_.reserve(numTriangles);
    for(int i = 0; i < numTriangles; ++i){
        const auto tri = mesh->triangle(i);
        if(tri[0] >= numVertices || tri[1] >= numVertices || tri[2] >= numVertices){
            continue;
        }
        Triangle triangle;
        triangle.a = vertices[tri[0]];
        triangle.b = vertices[tri[1]];
        triangle.c = vertices[tri[2]];
        triangle.sourceIndex = i;
        // A zero-area triangle contributes no surface and its degenerate
        // geometry is of no use as a nearest-point target, so it is dropped.
        if((triangle.b - triangle.a).cross(triangle.c - triangle.a).squaredNorm() == 0.0f){
            continue;
        }
        triangles_.push_back(triangle);
    }

    if(!triangles_.empty()){
        nodes_.reserve(2 * triangles_.size() / MaxNumLeafTriangles + 2);
        rootNodeIndex_ = buildTree(0, static_cast<int>(triangles_.size()));
    }
}


/**
   Builds the subtree over the triangle range [begin, end), reordering the
   triangles of the range in place, and returns the index of its root node.
*/
int MeshDistanceQuery::buildTree(int begin, int end)
{
    Node node;
    node.bboxMin = Vector3f::Constant(numeric_limits<float>::max());
    node.bboxMax = Vector3f::Constant(-numeric_limits<float>::max());
    for(int i = begin; i < end; ++i){
        const Triangle& triangle = triangles_[i];
        node.bboxMin = node.bboxMin.cwiseMin(triangle.a).cwiseMin(triangle.b).cwiseMin(triangle.c);
        node.bboxMax = node.bboxMax.cwiseMax(triangle.a).cwiseMax(triangle.b).cwiseMax(triangle.c);
    }

    if(end - begin <= MaxNumLeafTriangles){
        node.left = -1;
        node.right = -1;
        node.begin = begin;
        node.end = end;
    } else {
        // Split at the centroid median along the axis where the centroids
        // spread the most, which keeps the tree balanced.
        Vector3f centroidMin = Vector3f::Constant(numeric_limits<float>::max());
        Vector3f centroidMax = Vector3f::Constant(-numeric_limits<float>::max());
        for(int i = begin; i < end; ++i){
            const Triangle& triangle = triangles_[i];
            const Vector3f centroid = (triangle.a + triangle.b + triangle.c) / 3.0f;
            centroidMin = centroidMin.cwiseMin(centroid);
            centroidMax = centroidMax.cwiseMax(centroid);
        }
        int axis;
        (centroidMax - centroidMin).maxCoeff(&axis);

        const int mid = begin + (end - begin) / 2;
        std::nth_element(
            triangles_.begin() + begin, triangles_.begin() + mid, triangles_.begin() + end,
            [axis](const Triangle& lhs, const Triangle& rhs){
                return (lhs.a[axis] + lhs.b[axis] + lhs.c[axis]) < (rhs.a[axis] + rhs.b[axis] + rhs.c[axis]);
            });

        node.left = buildTree(begin, mid);
        node.right = buildTree(mid, end);
        node.begin = -1;
        node.end = -1;
    }

    nodes_.push_back(node);
    return static_cast<int>(nodes_.size()) - 1;
}


double MeshDistanceQuery::squaredDistanceToBox(const Node& node, const Vector3& point) const
{
    double d2 = 0.0;
    for(int axis = 0; axis < 3; ++axis){
        const double v = point[axis];
        if(v < node.bboxMin[axis]){
            const double d = node.bboxMin[axis] - v;
            d2 += d * d;
        } else if(v > node.bboxMax[axis]){
            const double d = v - node.bboxMax[axis];
            d2 += d * d;
        }
    }
    return d2;
}


MeshDistanceQuery::NearestPoint MeshDistanceQuery::findNearestPoint
(const Vector3& point, double maxDistance) const
{
    NearestPoint nearest;
    if(rootNodeIndex_ < 0){
        return nearest;
    }

    // maxDistance squared overflows to infinity for the default argument,
    // which correctly disables the bound.
    double bestSquared = maxDistance * maxDistance;

    int stack[TraversalStackSize];
    int stackSize = 0;
    stack[stackSize++] = rootNodeIndex_;

    while(stackSize > 0){
        const Node& node = nodes_[stack[--stackSize]];
        if(squaredDistanceToBox(node, point) >= bestSquared){
            continue;
        }
        if(node.left < 0){
            for(int i = node.begin; i < node.end; ++i){
                const Triangle& triangle = triangles_[i];
                const Vector3 candidate = closestPointOnTriangle(
                    point,
                    triangle.a.cast<double>(), triangle.b.cast<double>(), triangle.c.cast<double>());
                const double d2 = (candidate - point).squaredNorm();
                if(d2 < bestSquared){
                    bestSquared = d2;
                    nearest.point = candidate;
                    nearest.triangleIndex = triangle.sourceIndex;
                }
            }
        } else {
            // Visit the nearer child first so that the bound tightens as
            // early as possible. The farther child is pushed first because
            // the stack is popped in reverse order.
            const double leftDistance = squaredDistanceToBox(nodes_[node.left], point);
            const double rightDistance = squaredDistanceToBox(nodes_[node.right], point);
            if(leftDistance < rightDistance){
                stack[stackSize++] = node.right;
                stack[stackSize++] = node.left;
            } else {
                stack[stackSize++] = node.left;
                stack[stackSize++] = node.right;
            }
        }
    }

    if(nearest.triangleIndex >= 0){
        nearest.distance = std::sqrt(bestSquared);
    }
    return nearest;
}


MeshDistanceQuery::NearestPoint MeshDistanceQuery::findNearestPointByFullScan
(const Vector3& point, double maxDistance) const
{
    NearestPoint nearest;
    double bestSquared = maxDistance * maxDistance;
    for(const Triangle& triangle : triangles_){
        const Vector3 candidate = closestPointOnTriangle(
            point,
            triangle.a.cast<double>(), triangle.b.cast<double>(), triangle.c.cast<double>());
        const double d2 = (candidate - point).squaredNorm();
        if(d2 < bestSquared){
            bestSquared = d2;
            nearest.point = candidate;
            nearest.triangleIndex = triangle.sourceIndex;
        }
    }
    if(nearest.triangleIndex >= 0){
        nearest.distance = std::sqrt(bestSquared);
    }
    return nearest;
}
