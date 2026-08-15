#ifndef CNOID_UTIL_MESH_DISTANCE_QUERY_H
#define CNOID_UTIL_MESH_DISTANCE_QUERY_H

#include "EigenTypes.h"
#include <vector>
#include <limits>
#include "exportdecl.h"

namespace cnoid {

class SgMesh;

/**
   Nearest-point query on a triangle mesh.

   setMesh() copies the triangles of the given mesh into an internal compact
   array and builds an AABB tree over them, so the query object does not
   depend on the lifetime of the mesh and the queries stay efficient no
   matter how the triangles are organized in the source mesh. A query then
   descends the tree in the branch-and-bound manner, skipping every subtree
   whose bounding box is farther than the best distance found so far.

   The queries are read-only and do not modify any mutable state, so a single
   query object can be shared by concurrent readers.
*/
class CNOID_EXPORT MeshDistanceQuery
{
public:
    MeshDistanceQuery();
    explicit MeshDistanceQuery(const SgMesh* mesh);

    //! Copies the triangles of the mesh and builds the AABB tree over them.
    void setMesh(const SgMesh* mesh);

    void clear();

    //! Number of triangles the query object holds.
    int numTriangles() const { return static_cast<int>(triangles_.size()); }

    struct NearestPoint {
        //! Distance to the nearest point, or infinity when none was found
        double distance;
        //! Nearest point on the mesh surface
        Vector3 point;
        //! Triangle index in the source mesh, or a negative value when none was found
        int triangleIndex;

        NearestPoint()
            : distance(std::numeric_limits<double>::infinity()),
              point(Vector3::Zero()),
              triangleIndex(-1) { }

        bool isValid() const { return triangleIndex >= 0; }
    };

    /**
       Finds the point of the mesh nearest to the given point.

       maxDistance is an upper bound hint: subtrees and triangles farther than
       it are never visited, which lets a caller that knows a bound on the
       result prune the search. When no triangle is within maxDistance, the
       returned value is invalid (negative triangleIndex, infinite distance).
    */
    NearestPoint findNearestPoint(
        const Vector3& point,
        double maxDistance = std::numeric_limits<double>::max()) const;

    //! Shortcut that only returns the distance of findNearestPoint().
    double distance(const Vector3& point) const {
        return findNearestPoint(point).distance;
    }

    /**
       Reference implementation of findNearestPoint() that scans all the
       triangles without using the AABB tree. This exists to validate the
       tree traversal against; it produces exactly the same result.
    */
    NearestPoint findNearestPointByFullScan(
        const Vector3& point,
        double maxDistance = std::numeric_limits<double>::max()) const;

private:
    struct Triangle {
        Vector3f a;
        Vector3f b;
        Vector3f c;
        //! Triangle index in the source mesh
        int sourceIndex;
    };

    struct Node {
        Vector3f bboxMin;
        Vector3f bboxMax;
        //! Child node indices for an internal node, or -1 for a leaf
        int left;
        int right;
        //! Triangle range [begin, end) for a leaf
        int begin;
        int end;
    };

    int buildTree(int begin, int end);
    double squaredDistanceToBox(const Node& node, const Vector3& point) const;

    //! Triangles reordered by the tree construction
    std::vector<Triangle> triangles_;
    std::vector<Node> nodes_;
    int rootNodeIndex_;
};

}

#endif
