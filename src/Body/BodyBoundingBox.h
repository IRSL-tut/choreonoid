#ifndef CNOID_BODY_BODY_BOUNDING_BOX_H
#define CNOID_BODY_BODY_BOUNDING_BOX_H

#include <cnoid/BoundingBox>
#include <utility>
#include "exportdecl.h"

namespace cnoid {

class Body;

/**
   Efficient bounding-box computation for a Body.

   On the first call to compute() (or computeBoth()) the class traverses
   each link's shape once, gathers all mesh vertices in link-local
   coordinates, and reduces the vertex sets with the 3D convex hull.
   Subsequent calls simply transform the cached hull vertices with the
   Body's current per-link world poses.

   Two box types are provided:
     - RootLocalAABB : axis-aligned box in the root link's frame; unaffected
                       by a whole-body rotation in the world.
     - WorldAABB     : axis-aligned box in the world frame.

   The convex-hull precision is the QuickHull epsilon expressed as a
   *relative* value: any point whose distance to an existing hull face is
   below (precision * point_cloud_scale) is treated as lying on the face
   and dropped from the hull. Larger values give fewer hull vertices and a
   slightly looser bounding box; smaller values give a tighter bound at
   the cost of more vertices to iterate every compute() call. The default
   is chosen so that the bounding box is accurate to roughly 0.1 percent
   of the model's size, which is well within what a bounding-box
   application needs.
*/
class CNOID_EXPORT BodyBoundingBox
{
public:
    enum Type {
        RootLocalAABB,
        WorldAABB
    };

    /**
       The Body pointer is retained so subsequent compute() calls can read
       its current link poses. The vertex cache is built lazily on the
       first compute() so precision changes made before that first call
       do not force a rebuild.
    */
    explicit BodyBoundingBox(Body* body);

    ~BodyBoundingBox();

    BodyBoundingBox(const BodyBoundingBox&) = delete;
    BodyBoundingBox& operator=(const BodyBoundingBox&) = delete;

    /**
       Set the relative epsilon passed to QuickHull. If the vertex cache
       has already been built, it is discarded so that the next compute()
       call rebuilds it with the new precision.
    */
    void setConvexHullPrecision(float precision);

    float convexHullPrecision() const;

    /**
       Return the bounding box of the given type using the current per-link
       world transforms of the Body passed to the constructor. Builds the
       vertex cache on the first call.
    */
    BoundingBox compute(Type type = RootLocalAABB) const;

    /**
       Compute two bounding boxes of different types in a single pass over
       the cached vertices. This is more efficient than calling compute()
       twice because each cached point is transformed once for each type
       within the same loop, halving the memory-bandwidth cost.
       The returned pair matches the argument order: first = type1,
       second = type2.
    */
    std::pair<BoundingBox, BoundingBox> compute(Type type1, Type type2) const;

private:
    class Impl;
    Impl* impl;
};

}

#endif
