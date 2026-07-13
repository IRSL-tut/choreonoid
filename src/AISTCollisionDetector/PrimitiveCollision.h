#ifndef CNOID_AIST_COLLISION_DETECTOR_PRIMITIVE_COLLISION_H
#define CNOID_AIST_COLLISION_DETECTOR_PRIMITIVE_COLLISION_H

#include <cnoid/EigenTypes>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

/**
   A convex shape description used for the analytic (non-mesh) collision
   detection between primitive shapes. All the shapes with an axis (cylinder,
   cone, capsule) take the local Y-axis as the axis, which follows the
   convention of SgMesh primitives.
*/
class PrimitiveCollisionShape
{
public:
    enum Type { Sphere, Box, Cylinder, Cone, Capsule, Triangle };

    Type type;

    //! World transform of the shape frame. Not used for Triangle.
    Isometry3 T;

    //! Half extents of the box along the local axes
    Vector3 halfExtents;

    //! Radius of the sphere, cylinder, cone (base circle), or capsule
    double radius;

    /**
       Half of the height along the local Y-axis of the cylinder or cone.
       For the capsule, this is the half length of the cylindrical part
       (i.e. the total height is 2 * halfLength + 2 * radius).
       For the cone, the apex is at +halfLength and the base circle is at
       -halfLength on the local Y-axis.
    */
    double halfLength;

    //! Triangle vertices in world coordinates
    Vector3 vertices[3];

    void setSphere(const Isometry3& T_, double radius_){
        type = Sphere; T = T_; radius = radius_;
    }
    void setBox(const Isometry3& T_, const Vector3& halfExtents_){
        type = Box; T = T_; halfExtents = halfExtents_;
    }
    void setCylinder(const Isometry3& T_, double radius_, double height){
        type = Cylinder; T = T_; radius = radius_; halfLength = height / 2.0;
    }
    void setCone(const Isometry3& T_, double radius_, double height){
        type = Cone; T = T_; radius = radius_; halfLength = height / 2.0;
    }
    void setCapsule(const Isometry3& T_, double radius_, double height){
        type = Capsule; T = T_; radius = radius_; halfLength = height / 2.0;
    }
    void setTriangle(const Vector3& v0, const Vector3& v1, const Vector3& v2){
        type = Triangle;
        T.setIdentity();
        vertices[0] = v0; vertices[1] = v1; vertices[2] = v2;
    }
};

struct PrimitiveContactPoint
{
    Vector3 point;  //!< Contact point in world coordinates
    Vector3 normal; //!< Unit normal in world coordinates directing from shape0 toward shape1
    double depth;   //!< Penetration depth (positive when penetrating)
};

/**
   Detect the collision between two convex primitive shapes and generate
   the contact points. The detected points are appended to out_points
   without clearing the existing elements.
   When findFirstContactOnly is true, the function returns as soon as the
   contact is determined and only a single contact point is generated.
   @return true if the shapes are in contact
*/
CNOID_EXPORT bool detectPrimitiveShapeCollision(
    const PrimitiveCollisionShape& shape0, const PrimitiveCollisionShape& shape1,
    std::vector<PrimitiveContactPoint>& out_points, bool findFirstContactOnly = false);

}

#endif
