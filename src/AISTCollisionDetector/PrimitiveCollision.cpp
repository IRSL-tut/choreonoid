/**
   Analytic collision detection between convex primitive shapes.

   The implementation consists of the following components:

   - Closed-form contact functions for the sphere / capsule pairs
   - GJK distance computation on the "core" shapes with margins
     (a sphere is a point with margin and a capsule is a segment with margin)
   - EPA penetration computation for the overlapping core shapes
   - One-shot contact manifold generation which extracts the support
     features of both shapes along the contact normal and clips them
     against each other

   The contact normal convention: the normal directs from shape0 toward
   shape1, and the penetration depth is positive when the shapes overlap.
*/

#include "PrimitiveCollision.h"
#include <cmath>
#include <cfloat>

using namespace std;
using namespace cnoid;

namespace {

constexpr int GJK_MAX_ITERATIONS = 64;
constexpr double GJK_REL_ERROR2 = 1.0e-12;
constexpr double GJK_ABS_ERROR2 = 1.0e-24;
constexpr int EPA_MAX_ITERATIONS = 128;
constexpr int EPA_MAX_FACES = 512;
constexpr double EPA_TOLERANCE = 1.0e-10;

// The buffer sizes must cover the maximum configurable number of the cap
// circle vertices because the number is now a runtime parameter of
// PrimitiveCollisionParameterSet
constexpr int MAX_FEATURE_POINTS = PrimitiveCollisionParameterSet::MaxNumCapCircleVertices;
constexpr int MAX_CLIPPED_POINTS = MAX_FEATURE_POINTS * 2 + 8;

// If the angle between the contact normal and a shape axis (or face normal)
// is within the tolerance corresponding to this cosine-like threshold,
// an extended feature (face or edge) is selected instead of a single vertex.
constexpr double FEATURE_ANGLE_TOLERANCE = 0.1;

inline double margin(const PrimitiveCollisionShape& shape)
{
    return (shape.type == PrimitiveCollisionShape::Sphere ||
            shape.type == PrimitiveCollisionShape::Capsule) ? shape.radius : 0.0;
}

/**
   Support point of the core shape (the shape without the margin)
   in the local frame.
*/
Vector3 supportLocalCore(const PrimitiveCollisionShape& shape, const Vector3& d)
{
    switch(shape.type){

    case PrimitiveCollisionShape::Sphere:
        return Vector3::Zero();

    case PrimitiveCollisionShape::Box:
        return Vector3(
            d.x() >= 0.0 ? shape.halfExtents.x() : -shape.halfExtents.x(),
            d.y() >= 0.0 ? shape.halfExtents.y() : -shape.halfExtents.y(),
            d.z() >= 0.0 ? shape.halfExtents.z() : -shape.halfExtents.z());

    case PrimitiveCollisionShape::Cylinder: {
        const double y = (d.y() >= 0.0) ? shape.halfLength : -shape.halfLength;
        const double l = sqrt(d.x() * d.x() + d.z() * d.z());
        if(l > 1.0e-12){
            const double s = shape.radius / l;
            return Vector3(d.x() * s, y, d.z() * s);
        }
        return Vector3(shape.radius, y, 0.0);
    }

    case PrimitiveCollisionShape::Cone: {
        const Vector3 apex(0.0, shape.halfLength, 0.0);
        Vector3 base;
        const double l = sqrt(d.x() * d.x() + d.z() * d.z());
        if(l > 1.0e-12){
            const double s = shape.radius / l;
            base = Vector3(d.x() * s, -shape.halfLength, d.z() * s);
        } else {
            base = Vector3(shape.radius, -shape.halfLength, 0.0);
        }
        return (d.dot(apex) >= d.dot(base)) ? apex : base;
    }

    case PrimitiveCollisionShape::Capsule:
        return Vector3(0.0, (d.y() >= 0.0) ? shape.halfLength : -shape.halfLength, 0.0);

    case PrimitiveCollisionShape::Triangle:
    default:
        break;
    }
    return Vector3::Zero();
}

/**
   Support point of the core shape in the world frame
*/
inline Vector3 supportCore(const PrimitiveCollisionShape& shape, const Vector3& d)
{
    if(shape.type == PrimitiveCollisionShape::Triangle){
        double v0 = d.dot(shape.vertices[0]);
        double v1 = d.dot(shape.vertices[1]);
        double v2 = d.dot(shape.vertices[2]);
        if(v0 >= v1){
            return (v0 >= v2) ? shape.vertices[0] : shape.vertices[2];
        }
        return (v1 >= v2) ? shape.vertices[1] : shape.vertices[2];
    }
    return shape.T * supportLocalCore(shape, shape.T.linear().transpose() * d);
}

inline Vector3 shapeCenter(const PrimitiveCollisionShape& shape)
{
    if(shape.type == PrimitiveCollisionShape::Triangle){
        return (shape.vertices[0] + shape.vertices[1] + shape.vertices[2]) / 3.0;
    }
    return shape.T.translation();
}

double boundingSphereRadius(const PrimitiveCollisionShape& shape)
{
    switch(shape.type){
    case PrimitiveCollisionShape::Sphere:
        return shape.radius;
    case PrimitiveCollisionShape::Box:
        return shape.halfExtents.norm();
    case PrimitiveCollisionShape::Cylinder:
    case PrimitiveCollisionShape::Cone:
        return sqrt(shape.radius * shape.radius + shape.halfLength * shape.halfLength);
    case PrimitiveCollisionShape::Capsule:
        return shape.halfLength + shape.radius;
    case PrimitiveCollisionShape::Triangle:
    default:
        break;
    }
    return DBL_MAX;
}

struct SupportPoint
{
    Vector3 w;  // pA - pB (Minkowski difference)
    Vector3 pA;
    Vector3 pB;
};

inline SupportPoint supportMinkowski(
    const PrimitiveCollisionShape& shapeA, const PrimitiveCollisionShape& shapeB, const Vector3& d)
{
    SupportPoint sp;
    sp.pA = supportCore(shapeA, d);
    sp.pB = supportCore(shapeB, -d);
    sp.w = sp.pA - sp.pB;
    return sp;
}

struct Simplex
{
    SupportPoint v[4];
    double lambda[4];
    int n;
};

/**
   Compute the closest point to the origin on the current simplex,
   reduce the simplex to the minimal supporting vertex set, and set the
   barycentric coordinates. Returns the closest point.
   When the simplex is a tetrahedron enclosing the origin, containsOrigin
   is set to true.
*/
Vector3 closestPointOnSimplex(Simplex& simplex, bool& containsOrigin)
{
    containsOrigin = false;

    if(simplex.n == 1){
        simplex.lambda[0] = 1.0;
        return simplex.v[0].w;
    }

    if(simplex.n == 2){
        const Vector3& a = simplex.v[0].w;
        const Vector3& b = simplex.v[1].w;
        const Vector3 ab = b - a;
        const double t = -a.dot(ab);
        const double denom = ab.squaredNorm();
        if(t <= 0.0 || denom < 1.0e-30){
            simplex.n = 1;
            simplex.lambda[0] = 1.0;
            return a;
        }
        if(t >= denom){
            simplex.v[0] = simplex.v[1];
            simplex.n = 1;
            simplex.lambda[0] = 1.0;
            return simplex.v[0].w;
        }
        const double s = t / denom;
        simplex.lambda[0] = 1.0 - s;
        simplex.lambda[1] = s;
        return a + s * ab;
    }

    if(simplex.n == 3){
        // Closest point to the origin on triangle abc (Ericson, RTCD 5.1.5)
        const Vector3& a = simplex.v[0].w;
        const Vector3& b = simplex.v[1].w;
        const Vector3& c = simplex.v[2].w;
        const Vector3 ab = b - a;
        const Vector3 ac = c - a;
        const Vector3 ap = -a;
        const double d1 = ab.dot(ap);
        const double d2 = ac.dot(ap);
        if(d1 <= 0.0 && d2 <= 0.0){
            simplex.n = 1;
            simplex.lambda[0] = 1.0;
            return a;
        }
        const Vector3 bp = -b;
        const double d3 = ab.dot(bp);
        const double d4 = ac.dot(bp);
        if(d3 >= 0.0 && d4 <= d3){
            simplex.v[0] = simplex.v[1];
            simplex.n = 1;
            simplex.lambda[0] = 1.0;
            return simplex.v[0].w;
        }
        const double vc = d1 * d4 - d3 * d2;
        if(vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0){
            const double denom = d1 - d3;
            const double s = (denom > 1.0e-30) ? d1 / denom : 0.0;
            simplex.n = 2;
            simplex.lambda[0] = 1.0 - s;
            simplex.lambda[1] = s;
            return a + s * ab;
        }
        const Vector3 cp = -c;
        const double d5 = ab.dot(cp);
        const double d6 = ac.dot(cp);
        if(d6 >= 0.0 && d5 <= d6){
            simplex.v[0] = simplex.v[2];
            simplex.n = 1;
            simplex.lambda[0] = 1.0;
            return simplex.v[0].w;
        }
        const double vb = d5 * d2 - d1 * d6;
        if(vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0){
            const double denom = d2 - d6;
            const double s = (denom > 1.0e-30) ? d2 / denom : 0.0;
            simplex.v[1] = simplex.v[2];
            simplex.n = 2;
            simplex.lambda[0] = 1.0 - s;
            simplex.lambda[1] = s;
            return a + s * ac;
        }
        const double va = d3 * d6 - d5 * d4;
        if(va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0){
            const double denom = (d4 - d3) + (d5 - d6);
            const double s = (denom > 1.0e-30) ? (d4 - d3) / denom : 0.0;
            simplex.v[0] = simplex.v[1];
            simplex.v[1] = simplex.v[2];
            simplex.n = 2;
            simplex.lambda[0] = 1.0 - s;
            simplex.lambda[1] = s;
            return simplex.v[0].w + s * (simplex.v[1].w - simplex.v[0].w);
        }
        const double denom = va + vb + vc;
        if(fabs(denom) < 1.0e-30){
            // Degenerate triangle
            simplex.n = 1;
            simplex.lambda[0] = 1.0;
            return a;
        }
        const double s = vb / denom;
        const double t = vc / denom;
        simplex.lambda[0] = 1.0 - s - t;
        simplex.lambda[1] = s;
        simplex.lambda[2] = t;
        return a + s * ab + t * ac;
    }

    // simplex.n == 4
    // Check the origin against the four faces of the tetrahedron and
    // find the closest one, or detect the origin enclosure.
    static const int faceIndices[4][3] = {
        { 0, 1, 2 }, { 0, 2, 3 }, { 0, 3, 1 }, { 1, 3, 2 }
    };
    const Vector3 points[4] = {
        simplex.v[0].w, simplex.v[1].w, simplex.v[2].w, simplex.v[3].w
    };

    bool inside = true;
    double minDist2 = DBL_MAX;
    Vector3 closest = Vector3::Zero();
    Simplex bestSub;
    bestSub.n = 0;

    for(int i=0; i < 4; ++i){
        const int ia = faceIndices[i][0];
        const int ib = faceIndices[i][1];
        const int ic = faceIndices[i][2];
        const int id = 6 - ia - ib - ic; // the opposite vertex
        const Vector3& a = points[ia];
        const Vector3& b = points[ib];
        const Vector3& c = points[ic];
        Vector3 n = (b - a).cross(c - a);
        const double n2 = n.squaredNorm();
        if(n2 < 1.0e-30){
            continue; // degenerate face
        }
        // Orient the face normal outward (away from the opposite vertex)
        double signOpposite = n.dot(points[id] - a);
        double signOrigin = n.dot(-a);
        if(signOpposite > 0.0){
            n = -n;
            signOpposite = -signOpposite;
            signOrigin = -signOrigin;
        }
        if(signOrigin > 0.0){
            // The origin is outside this face
            inside = false;
            Simplex sub;
            sub.n = 3;
            sub.v[0] = simplex.v[ia];
            sub.v[1] = simplex.v[ib];
            sub.v[2] = simplex.v[ic];
            bool dummy;
            const Vector3 p = closestPointOnSimplex(sub, dummy);
            const double d2 = p.squaredNorm();
            if(d2 < minDist2){
                minDist2 = d2;
                closest = p;
                bestSub = sub;
            }
        }
    }

    if(inside){
        containsOrigin = true;
        simplex.lambda[0] = simplex.lambda[1] = simplex.lambda[2] = simplex.lambda[3] = 0.25;
        return Vector3::Zero();
    }

    if(bestSub.n == 0){
        // All the faces are degenerate; fall back to keeping a single vertex
        simplex.n = 1;
        simplex.lambda[0] = 1.0;
        return simplex.v[0].w;
    }

    simplex = bestSub;
    return closest;
}

struct GjkResult
{
    bool coresIntersecting;
    double distance;   // distance between the core shapes (valid when not intersecting)
    Vector3 pA;        // the closest / witness point on core A
    Vector3 pB;        // the closest / witness point on core B
    Simplex simplex;   // the terminal simplex
};

bool runGjk(const PrimitiveCollisionShape& shapeA, const PrimitiveCollisionShape& shapeB, GjkResult& result)
{
    Vector3 d = shapeCenter(shapeB) - shapeCenter(shapeA);
    if(d.squaredNorm() < 1.0e-20){
        d = Vector3::UnitX();
    }

    Simplex& simplex = result.simplex;
    simplex.n = 1;
    simplex.v[0] = supportMinkowski(shapeA, shapeB, -d);
    simplex.lambda[0] = 1.0;

    Vector3 v = simplex.v[0].w;
    double dist2 = v.squaredNorm();

    for(int iter = 0; iter < GJK_MAX_ITERATIONS; ++iter){

        if(dist2 < GJK_ABS_ERROR2){
            result.coresIntersecting = true;
            return true;
        }

        const SupportPoint w = supportMinkowski(shapeA, shapeB, -v);

        // Convergence check: no more progress toward the origin
        const double vw = v.dot(w.w);
        if(dist2 - vw <= GJK_REL_ERROR2 * dist2){
            break;
        }

        // Duplicate support point check (numerical safety)
        bool duplicate = false;
        for(int i=0; i < simplex.n; ++i){
            if((simplex.v[i].w - w.w).squaredNorm() < 1.0e-24){
                duplicate = true;
                break;
            }
        }
        if(duplicate){
            break;
        }

        simplex.v[simplex.n++] = w;

        bool containsOrigin;
        v = closestPointOnSimplex(simplex, containsOrigin);
        if(containsOrigin){
            result.coresIntersecting = true;
            return true;
        }
        const double newDist2 = v.squaredNorm();
        if(newDist2 >= dist2){
            // No progress (numerical limit)
            dist2 = newDist2;
            break;
        }
        dist2 = newDist2;
    }

    result.coresIntersecting = false;
    result.distance = sqrt(dist2);
    result.pA.setZero();
    result.pB.setZero();
    for(int i=0; i < simplex.n; ++i){
        result.pA += simplex.lambda[i] * simplex.v[i].pA;
        result.pB += simplex.lambda[i] * simplex.v[i].pB;
    }
    return true;
}

//! EPA face
struct EpaFace
{
    int a, b, c;
    Vector3 n;  // unit normal directing away from the origin
    double d;   // distance from the origin to the face plane
    bool alive;
};

bool addEpaFace(vector<EpaFace>& faces, const vector<SupportPoint>& vertices, int a, int b, int c)
{
    if(faces.size() >= EPA_MAX_FACES){
        return false;
    }
    EpaFace f;
    f.a = a; f.b = b; f.c = c;
    const Vector3& pa = vertices[a].w;
    Vector3 n = (vertices[b].w - pa).cross(vertices[c].w - pa);
    const double l = n.norm();
    if(l < 1.0e-15){
        return false;
    }
    n /= l;
    double d = n.dot(pa);
    if(d < 0.0){
        // Flip so that the normal directs away from the origin
        f.b = c; f.c = b;
        n = -n;
        d = -d;
    }
    f.n = n;
    f.d = d;
    f.alive = true;
    faces.push_back(f);
    return true;
}

/**
   EPA penetration depth computation for the intersecting core shapes.
   Returns true on success. The output normal directs from shapeA toward
   shapeB and the depth is positive.
*/
bool runEpa(
    const PrimitiveCollisionShape& shapeA, const PrimitiveCollisionShape& shapeB,
    const Simplex& terminalSimplex,
    Vector3& out_normal, double& out_depth, Vector3& out_pA, Vector3& out_pB)
{
    vector<SupportPoint> vertices;
    vertices.reserve(64);
    for(int i=0; i < terminalSimplex.n; ++i){
        vertices.push_back(terminalSimplex.v[i]);
    }

    // Expand the simplex to a tetrahedron if necessary
    if(vertices.size() == 1){
        static const Vector3 dirs[6] = {
            Vector3::UnitX(), -Vector3::UnitX(),
            Vector3::UnitY(), -Vector3::UnitY(),
            Vector3::UnitZ(), -Vector3::UnitZ()
        };
        for(int i=0; i < 6 && vertices.size() < 4; ++i){
            const SupportPoint sp = supportMinkowski(shapeA, shapeB, dirs[i]);
            bool duplicate = false;
            for(auto& v : vertices){
                if((v.w - sp.w).squaredNorm() < 1.0e-20){
                    duplicate = true;
                    break;
                }
            }
            if(!duplicate){
                vertices.push_back(sp);
            }
        }
    }
    if(vertices.size() == 2){
        const Vector3 axis = (vertices[1].w - vertices[0].w).normalized();
        Vector3 u = axis.cross(Vector3::UnitX());
        if(u.squaredNorm() < 0.01){
            u = axis.cross(Vector3::UnitY());
        }
        u.normalize();
        const Vector3 v2 = axis.cross(u);
        static const double angles[3] = { 0.0, 2.0944, 4.18879 }; // 0, 120, 240 deg
        for(int i=0; i < 3 && vertices.size() < 4; ++i){
            const Vector3 dir = cos(angles[i]) * u + sin(angles[i]) * v2;
            const SupportPoint sp = supportMinkowski(shapeA, shapeB, dir);
            bool duplicate = false;
            for(auto& v : vertices){
                if((v.w - sp.w).squaredNorm() < 1.0e-20){
                    duplicate = true;
                    break;
                }
            }
            if(!duplicate){
                vertices.push_back(sp);
            }
        }
    }
    if(vertices.size() == 3){
        const Vector3 n =
            (vertices[1].w - vertices[0].w).cross(vertices[2].w - vertices[0].w);
        if(n.squaredNorm() > 1.0e-30){
            const Vector3 nn = n.normalized();
            const SupportPoint sp1 = supportMinkowski(shapeA, shapeB, nn);
            const SupportPoint sp2 = supportMinkowski(shapeA, shapeB, -nn);
            const double d1 = fabs(nn.dot(sp1.w - vertices[0].w));
            const double d2 = fabs(nn.dot(sp2.w - vertices[0].w));
            vertices.push_back((d1 >= d2) ? sp1 : sp2);
        }
    }
    if(vertices.size() < 4){
        return false; // The Minkowski difference is (nearly) flat
    }

    // Fix the tetrahedron winding so that all the face normals direct outward
    {
        const Vector3& a = vertices[0].w;
        const Vector3 n = (vertices[1].w - a).cross(vertices[2].w - a);
        if(n.dot(vertices[3].w - a) > 0.0){
            std::swap(vertices[1], vertices[2]);
        }
    }

    vector<EpaFace> faces;
    faces.reserve(128);
    addEpaFace(faces, vertices, 0, 1, 2);
    addEpaFace(faces, vertices, 0, 2, 3);
    addEpaFace(faces, vertices, 0, 3, 1);
    addEpaFace(faces, vertices, 1, 3, 2);
    if(faces.size() < 4){
        return false;
    }

    const EpaFace* bestFace = nullptr;

    for(int iter = 0; iter < EPA_MAX_ITERATIONS; ++iter){

        // Find the face closest to the origin
        int minIndex = -1;
        double minDist = DBL_MAX;
        for(size_t i=0; i < faces.size(); ++i){
            if(faces[i].alive && faces[i].d < minDist){
                minDist = faces[i].d;
                minIndex = i;
            }
        }
        if(minIndex < 0){
            return false;
        }
        bestFace = &faces[minIndex];

        const Vector3 n = bestFace->n;
        const SupportPoint sp = supportMinkowski(shapeA, shapeB, n);
        const double gain = n.dot(sp.w) - bestFace->d;
        if(gain < EPA_TOLERANCE + 1.0e-6 * bestFace->d){
            break; // converged
        }

        // Remove the faces visible from the new vertex and collect the
        // horizon edges
        const int newIndex = vertices.size();
        vertices.push_back(sp);

        vector<std::pair<int,int>> horizon;
        for(auto& f : faces){
            if(!f.alive){
                continue;
            }
            if(f.n.dot(sp.w) > f.d + 1.0e-12){
                f.alive = false;
                const int e[3][2] = { {f.a, f.b}, {f.b, f.c}, {f.c, f.a} };
                for(int k=0; k < 3; ++k){
                    bool cancelled = false;
                    for(auto it = horizon.begin(); it != horizon.end(); ++it){
                        if(it->first == e[k][1] && it->second == e[k][0]){
                            horizon.erase(it);
                            cancelled = true;
                            break;
                        }
                    }
                    if(!cancelled){
                        horizon.emplace_back(e[k][0], e[k][1]);
                    }
                }
            }
        }
        if(horizon.empty()){
            break;
        }
        bool ok = true;
        for(auto& edge : horizon){
            if(!addEpaFace(faces, vertices, edge.first, edge.second, newIndex)){
                ok = false;
                break;
            }
        }
        if(!ok){
            break;
        }
        bestFace = nullptr;
    }

    if(!bestFace){
        // Re-find the closest face
        double minDist = DBL_MAX;
        for(auto& f : faces){
            if(f.alive && f.d < minDist){
                minDist = f.d;
                bestFace = &f;
            }
        }
        if(!bestFace){
            return false;
        }
    }

    // Compute the witness points using the barycentric coordinates of the
    // projection of the origin onto the closest face
    const Vector3& a = vertices[bestFace->a].w;
    const Vector3& b = vertices[bestFace->b].w;
    const Vector3& c = vertices[bestFace->c].w;
    const Vector3 p = bestFace->n * bestFace->d;
    const Vector3 v0 = b - a;
    const Vector3 v1 = c - a;
    const Vector3 v2 = p - a;
    const double d00 = v0.dot(v0);
    const double d01 = v0.dot(v1);
    const double d11 = v1.dot(v1);
    const double d20 = v2.dot(v0);
    const double d21 = v2.dot(v1);
    const double denom = d00 * d11 - d01 * d01;
    double lb = 0.0;
    double lc = 0.0;
    if(fabs(denom) > 1.0e-30){
        lb = (d11 * d20 - d01 * d21) / denom;
        lc = (d00 * d21 - d01 * d20) / denom;
    }
    const double la = 1.0 - lb - lc;

    out_pA = la * vertices[bestFace->a].pA + lb * vertices[bestFace->b].pA + lc * vertices[bestFace->c].pA;
    out_pB = la * vertices[bestFace->a].pB + lb * vertices[bestFace->b].pB + lc * vertices[bestFace->c].pB;
    out_normal = bestFace->n;
    out_depth = bestFace->d;
    return true;
}

struct SupportFeature
{
    Vector3 points[MAX_FEATURE_POINTS];
    int n;
};

/**
   Extract the support feature (vertex, edge, or face polygon) of the shape
   surface along the direction d. The polygon vertices are stored in an
   order which forms a convex loop.
*/
void getSupportFeature(const PrimitiveCollisionShape& shape, const Vector3& d, int numCapCircleVertices, SupportFeature& out)
{
    out.n = 0;

    switch(shape.type){

    case PrimitiveCollisionShape::Sphere:
        out.points[out.n++] = shape.T.translation() + shape.radius * d;
        break;

    case PrimitiveCollisionShape::Box: {
        const Matrix3& R = shape.T.linear();
        const Vector3 dl = R.transpose() * d;
        const Vector3& h = shape.halfExtents;
        bool free[3];
        int numFree = 0;
        for(int i=0; i < 3; ++i){
            free[i] = (fabs(dl[i]) < FEATURE_ANGLE_TOLERANCE);
            if(free[i]){
                ++numFree;
            }
        }
        if(numFree >= 3){
            // Degenerate direction; treat as a vertex
            free[0] = free[1] = free[2] = false;
            numFree = 0;
        }
        Vector3 base;
        for(int i=0; i < 3; ++i){
            base[i] = free[i] ? 0.0 : ((dl[i] >= 0.0) ? h[i] : -h[i]);
        }
        if(numFree == 0){
            out.points[out.n++] = shape.T * base;
        } else if(numFree == 1){
            int k = free[0] ? 0 : (free[1] ? 1 : 2);
            Vector3 p0 = base; p0[k] = -h[k];
            Vector3 p1 = base; p1[k] = h[k];
            out.points[out.n++] = shape.T * p0;
            out.points[out.n++] = shape.T * p1;
        } else {
            // Face: two free axes
            int k0 = -1, k1 = -1;
            for(int i=0; i < 3; ++i){
                if(free[i]){
                    if(k0 < 0) k0 = i; else k1 = i;
                }
            }
            static const double signs[4][2] = { {1,1}, {-1,1}, {-1,-1}, {1,-1} };
            for(int i=0; i < 4; ++i){
                Vector3 p = base;
                p[k0] = signs[i][0] * h[k0];
                p[k1] = signs[i][1] * h[k1];
                out.points[out.n++] = shape.T * p;
            }
        }
        break;
    }

    case PrimitiveCollisionShape::Cylinder: {
        const Vector3 axis = shape.T.linear().col(1);
        const Vector3& c = shape.T.translation();
        const double ca = d.dot(axis);
        if(fabs(ca) < FEATURE_ANGLE_TOLERANCE){
            // Side edge
            Vector3 e = d - ca * axis;
            const double l = e.norm();
            if(l > 1.0e-12){
                e /= l;
                out.points[out.n++] = c - shape.halfLength * axis + shape.radius * e;
                out.points[out.n++] = c + shape.halfLength * axis + shape.radius * e;
                break;
            }
        }
        // Cap circle polygon
        const double sign = (ca >= 0.0) ? 1.0 : -1.0;
        const Vector3 capCenter = c + sign * shape.halfLength * axis;
        const Vector3 u = shape.T.linear().col(0);
        const Vector3 w = shape.T.linear().col(2);
        for(int i=0; i < numCapCircleVertices; ++i){
            const double theta = 2.0 * M_PI * i / numCapCircleVertices;
            out.points[out.n++] = capCenter + shape.radius * (cos(theta) * u + sin(theta) * w);
        }
        break;
    }

    case PrimitiveCollisionShape::Cone: {
        const Vector3 axis = shape.T.linear().col(1);
        const Vector3& c = shape.T.translation();
        const Vector3 apex = c + shape.halfLength * axis;
        const Vector3 baseCenter = c - shape.halfLength * axis;
        const double ca = d.dot(axis);
        Vector3 e = d - ca * axis;
        const double l = e.norm();
        Vector3 rimPoint;
        if(l > 1.0e-12){
            rimPoint = baseCenter + (shape.radius / l) * e;
        } else {
            rimPoint = baseCenter + shape.radius * shape.T.linear().col(0);
        }
        const double apexVal = d.dot(apex);
        const double rimVal = d.dot(rimPoint);
        const double tol = FEATURE_ANGLE_TOLERANCE * (shape.radius + 2.0 * shape.halfLength);
        if(apexVal > rimVal + tol){
            out.points[out.n++] = apex;
        } else if(rimVal > apexVal + tol){
            if(ca < -FEATURE_ANGLE_TOLERANCE){
                // Base circle polygon
                const Vector3 u = shape.T.linear().col(0);
                const Vector3 w = shape.T.linear().col(2);
                for(int i=0; i < numCapCircleVertices; ++i){
                    const double theta = 2.0 * M_PI * i / numCapCircleVertices;
                    out.points[out.n++] = baseCenter + shape.radius * (cos(theta) * u + sin(theta) * w);
                }
            } else {
                out.points[out.n++] = rimPoint;
            }
        } else {
            // Side line segment
            out.points[out.n++] = apex;
            out.points[out.n++] = rimPoint;
        }
        break;
    }

    case PrimitiveCollisionShape::Capsule: {
        const Vector3 axis = shape.T.linear().col(1);
        const Vector3& c = shape.T.translation();
        const double ca = d.dot(axis);
        if(fabs(ca) < FEATURE_ANGLE_TOLERANCE){
            out.points[out.n++] = c - shape.halfLength * axis + shape.radius * d;
            out.points[out.n++] = c + shape.halfLength * axis + shape.radius * d;
        } else {
            const double sign = (ca >= 0.0) ? 1.0 : -1.0;
            out.points[out.n++] = c + sign * shape.halfLength * axis + shape.radius * d;
        }
        break;
    }

    case PrimitiveCollisionShape::Triangle: {
        double vals[3];
        double maxVal = -DBL_MAX;
        double size = 0.0;
        for(int i=0; i < 3; ++i){
            vals[i] = d.dot(shape.vertices[i]);
            maxVal = std::max(maxVal, vals[i]);
        }
        size = std::max((shape.vertices[1] - shape.vertices[0]).norm(),
                        (shape.vertices[2] - shape.vertices[0]).norm());
        const double tol = FEATURE_ANGLE_TOLERANCE * size;
        for(int i=0; i < 3; ++i){
            if(vals[i] >= maxVal - tol){
                out.points[out.n++] = shape.vertices[i];
            }
        }
        break;
    }
    }
}

/**
   Ensure that the polygon is wound counter-clockwise around the axis n
*/
void makeCounterClockwise(SupportFeature& feature, const Vector3& n)
{
    if(feature.n < 3){
        return;
    }
    Vector3 areaVec = Vector3::Zero();
    for(int i=0; i < feature.n; ++i){
        const Vector3& p0 = feature.points[i];
        const Vector3& p1 = feature.points[(i + 1) % feature.n];
        areaVec += p0.cross(p1);
    }
    if(areaVec.dot(n) < 0.0){
        for(int i=0, j=feature.n - 1; i < j; ++i, --j){
            std::swap(feature.points[i], feature.points[j]);
        }
    }
}

/**
   Clip the incident points (a polygon loop or a segment) against the side
   planes of the reference polygon. The reference polygon must be wound
   counter-clockwise around the axis n.
*/
int clipAgainstPolygon(
    const SupportFeature& reference, const Vector3& n,
    const Vector3* points, int numPoints, bool isLoop,
    Vector3* out_points)
{
    Vector3 buf1[MAX_CLIPPED_POINTS];
    Vector3 buf2[MAX_CLIPPED_POINTS];
    Vector3* src = buf1;
    Vector3* dst = buf2;
    int numSrc = 0;

    for(int i=0; i < numPoints && i < MAX_CLIPPED_POINTS; ++i){
        src[numSrc++] = points[i];
    }

    for(int i=0; i < reference.n; ++i){
        const Vector3& r0 = reference.points[i];
        const Vector3& r1 = reference.points[(i + 1) % reference.n];
        Vector3 h = n.cross(r1 - r0); // interior side plane normal
        const double hl = h.norm();
        if(hl < 1.0e-12){
            continue;
        }
        h /= hl;

        int numDst = 0;
        const int numEdges = isLoop ? numSrc : numSrc - 1;
        if(numSrc == 1){
            // A single point is just tested against the plane
            if(h.dot(src[0] - r0) >= 0.0){
                dst[numDst++] = src[0];
            }
        } else {
            for(int j=0; j < numEdges; ++j){
                const Vector3& p0 = src[j];
                const Vector3& p1 = src[(j + 1) % numSrc];
                const double s0 = h.dot(p0 - r0);
                const double s1 = h.dot(p1 - r0);
                if(s0 >= 0.0){
                    if(numDst < MAX_CLIPPED_POINTS){
                        dst[numDst++] = p0;
                    }
                    if(s1 < 0.0){
                        const double t = s0 / (s0 - s1);
                        if(numDst < MAX_CLIPPED_POINTS){
                            dst[numDst++] = p0 + t * (p1 - p0);
                        }
                    }
                } else if(s1 >= 0.0){
                    const double t = s0 / (s0 - s1);
                    if(numDst < MAX_CLIPPED_POINTS){
                        dst[numDst++] = p0 + t * (p1 - p0);
                    }
                }
            }
            if(!isLoop && numSrc >= 2){
                // Keep the last point of an open polyline if it is inside
                const Vector3& last = src[numSrc - 1];
                if(h.dot(last - r0) >= 0.0){
                    if(numDst == 0 || (dst[numDst-1] - last).squaredNorm() > 1.0e-24){
                        if(numDst < MAX_CLIPPED_POINTS){
                            dst[numDst++] = last;
                        }
                    }
                }
            }
        }
        std::swap(src, dst);
        numSrc = numDst;
        if(numSrc == 0){
            return 0;
        }
    }

    for(int i=0; i < numSrc; ++i){
        out_points[i] = src[i];
    }
    return numSrc;
}

inline void addContactPoint(
    std::vector<PrimitiveContactPoint>& out_points,
    const Vector3& point, const Vector3& normal, double depth)
{
    PrimitiveContactPoint cp;
    cp.point = point;
    cp.normal = normal;
    cp.depth = depth;
    out_points.push_back(cp);
}

Vector3 polygonPlaneNormal(const SupportFeature& polygon)
{
    Vector3 areaVec = Vector3::Zero();
    const Vector3& p0 = polygon.points[0];
    for(int i=1; i < polygon.n - 1; ++i){
        areaVec += (polygon.points[i] - p0).cross(polygon.points[i+1] - p0);
    }
    const double l = areaVec.norm();
    if(l < 1.0e-15){
        return Vector3::Zero();
    }
    return areaVec / l;
}

/**
   Generate the contact manifold for the given contact normal.
   @param n unit contact normal from shape0 toward shape1
   @param witnessPoint the deepest contact point (midway between the surfaces)
   @param depth0 the penetration depth at the witness point
*/
void generateContactManifold(
    const PrimitiveCollisionShape& shape0, const PrimitiveCollisionShape& shape1,
    const Vector3& n, const Vector3& witnessPoint, double depth0,
    const PrimitiveCollisionParameterSet& params,
    std::vector<PrimitiveContactPoint>& out_points)
{
    const size_t pointIndexTop = out_points.size();
    SupportFeature f0, f1;
    getSupportFeature(shape0, n, params.numCapCircleVertices, f0);
    getSupportFeature(shape1, -n, params.numCapCircleVertices, f1);

    // A face - vertex pair must proceed to the face clipping below. The
    // witness point is based on arbitrary support vertices and can be far
    // from the actual contact when the reference face is large. Use it only
    // when neither feature provides a reference face.
    if((f0.n == 1 || f1.n == 1) && f0.n < 3 && f1.n < 3){
        addContactPoint(out_points, witnessPoint, n, depth0);
        return;
    }

    if(f0.n == 2 && f1.n == 2){
        // Edge - edge
        const Vector3& p0 = f0.points[0];
        const Vector3& p1 = f0.points[1];
        const Vector3& q0 = f1.points[0];
        const Vector3& q1 = f1.points[1];
        const Vector3 u = p1 - p0;
        const Vector3 v = q1 - q0;
        const Vector3 cr = u.cross(v);
        const double crossNorm2 = cr.squaredNorm();
        const double parallelThreshold = 1.0e-6 * u.squaredNorm() * v.squaredNorm();

        if(crossNorm2 > parallelThreshold){
            // Skewed edges: single contact at the closest point pair
            const double a = u.dot(u);
            const double b = u.dot(v);
            const double c = v.dot(v);
            const Vector3 r = p0 - q0;
            const double d = u.dot(r);
            const double e = v.dot(r);
            const double denom = a * c - b * b;
            double s = 0.0;
            if(fabs(denom) > 1.0e-30){
                s = std::min(1.0, std::max(0.0, (b * e - c * d) / denom));
            }
            const double t = (c > 1.0e-30) ? std::min(1.0, std::max(0.0, (b * s + e) / c)) : 0.0;
            const Vector3 cp0 = p0 + s * u;
            const Vector3 cp1 = q0 + t * v;
            double depth = (cp0 - cp1).dot(n);
            if(depth <= 0.0){
                depth = depth0;
            }
            addContactPoint(out_points, 0.5 * (cp0 + cp1), n, depth);
        } else {
            // Parallel edges: up to two contact points over the overlapping range
            const double ul2 = u.squaredNorm();
            if(ul2 < 1.0e-24){
                addContactPoint(out_points, witnessPoint, n, depth0);
                return;
            }
            double t0 = (q0 - p0).dot(u) / ul2;
            double t1 = (q1 - p0).dot(u) / ul2;
            if(t0 > t1){
                std::swap(t0, t1);
            }
            const double lo = std::max(0.0, t0);
            const double hi = std::min(1.0, t1);
            if(lo > hi){
                addContactPoint(out_points, witnessPoint, n, depth0);
                return;
            }
            int numAdded = 0;
            for(const double t : { lo, hi }){
                const Vector3 cp0 = p0 + t * u;
                // The corresponding point on the other edge
                const double vl2 = v.squaredNorm();
                double tv = (vl2 > 1.0e-24) ? (cp0 - q0).dot(v) / vl2 : 0.0;
                tv = std::min(1.0, std::max(0.0, tv));
                const Vector3 cp1 = q0 + tv * v;
                const double depth = (cp0 - cp1).dot(n);
                if(depth > 0.0){
                    addContactPoint(out_points, 0.5 * (cp0 + cp1), n, depth);
                    ++numAdded;
                }
                if(hi - lo < 1.0e-12){
                    break; // degenerate overlap range
                }
            }
            if(numAdded == 0){
                addContactPoint(out_points, witnessPoint, n, depth0);
            }
        }
        return;
    }

    // Face clipping. Select the reference face.
    bool referenceIs0;
    if(f0.n >= 3 && f1.n >= 3){
        makeCounterClockwise(f0, n);
        makeCounterClockwise(f1, n);
        const double a0 = fabs(polygonPlaneNormal(f0).dot(n));
        const double a1 = fabs(polygonPlaneNormal(f1).dot(n));
        referenceIs0 = (a0 >= a1);
    } else {
        referenceIs0 = (f0.n >= 3);
        makeCounterClockwise(referenceIs0 ? f0 : f1, n);
    }
    const SupportFeature& reference = referenceIs0 ? f0 : f1;
    const SupportFeature& incident = referenceIs0 ? f1 : f0;

    const Vector3 m = polygonPlaneNormal(reference);
    const double nm = n.dot(m);
    if(m.isZero() || fabs(nm) < 0.05){
        addContactPoint(out_points, witnessPoint, n, depth0);
        return;
    }
    const Vector3& c = reference.points[0];

    Vector3 clipped[MAX_CLIPPED_POINTS];
    // Note that the axis passed to clipAgainstPolygon must be the axis
    // around which the reference polygon is wound counter-clockwise,
    // which is always +n here regardless of which shape the reference
    // face belongs to
    const int numClipped = clipAgainstPolygon(
        reference, n, incident.points, incident.n, incident.n >= 3, clipped);

    int numAdded = 0;
    for(int i=0; i < numClipped; ++i){
        const Vector3& x = clipped[i];
        const double s = ((c - x).dot(m)) / nm;
        // The incident points are on shape1 when the reference face belongs
        // to shape0, and vice versa
        double depth = referenceIs0 ? s : -s;
        if(depth > -params.contactPersistenceTolerance){
            if(depth < 0.0){
                depth = 0.0; // Keep a slightly separating point as a zero-depth contact
            }
            const Vector3 point = referenceIs0 ? Vector3(x + 0.5 * depth * n) : Vector3(x - 0.5 * depth * n);
            bool duplicated = false;
            for(size_t j = pointIndexTop; j < out_points.size(); ++j){
                if((out_points[j].point - point).squaredNorm() < 1.0e-20 &&
                   out_points[j].normal.dot(n) > 1.0 - 1.0e-12){
                    duplicated = true;
                    break;
                }
            }
            if(!duplicated){
                addContactPoint(out_points, point, n, depth);
                ++numAdded;
            }
        }
    }
    if(numAdded == 0){
        addContactPoint(out_points, witnessPoint, n, depth0);
    }
}

//
// Closed-form fast paths
//

void generateContactManifold(
    const PrimitiveCollisionShape& shape0, const PrimitiveCollisionShape& shape1,
    const Vector3& n, const Vector3& witnessPoint, double depth0,
    const PrimitiveCollisionParameterSet& params,
    std::vector<PrimitiveContactPoint>& out_points);

/**
   Box - box contact detection based on the separating axis test.
   This specialized implementation is used instead of the GJK / EPA path
   because the EPA convergence becomes slow and the resulting normal
   becomes inaccurate when the penetration depths along multiple axes are
   almost the same (e.g. regularly aligned boxes overlapping at their
   corners), while the SAT always gives the exact minimum penetration axis.
*/
bool detectBoxBox(
    const PrimitiveCollisionShape& s0, const PrimitiveCollisionShape& s1,
    std::vector<PrimitiveContactPoint>& out_points, bool findFirstContactOnly,
    const PrimitiveCollisionParameterSet& params)
{
    const Matrix3& R0 = s0.T.linear();
    const Matrix3& R1 = s1.T.linear();
    const Vector3 p = s1.T.translation() - s0.T.translation();
    const Vector3& a = s0.halfExtents;
    const Vector3& b = s1.halfExtents;

    const Matrix3 R = R0.transpose() * R1;
    const Matrix3 absR = R.cwiseAbs();
    const Vector3 p0 = R0.transpose() * p; // the center offset in the box0 frame
    const Vector3 p1 = R1.transpose() * p; // the center offset in the box1 frame

    double minDepth = DBL_MAX;
    Vector3 bestAxis; // world frame, directing from box0 toward box1

    // Face axes of box0
    for(int i=0; i < 3; ++i){
        const double depth = a[i] + absR.row(i).dot(b) - fabs(p0[i]);
        if(depth <= 0.0){
            return false;
        }
        if(depth < minDepth){
            minDepth = depth;
            bestAxis = (p0[i] >= 0.0) ? Vector3(R0.col(i)) : Vector3(-R0.col(i));
        }
    }
    // Face axes of box1
    for(int j=0; j < 3; ++j){
        const double depth = b[j] + absR.col(j).dot(a) - fabs(p1[j]);
        if(depth <= 0.0){
            return false;
        }
        if(depth < minDepth){
            minDepth = depth;
            bestAxis = (p1[j] >= 0.0) ? Vector3(R1.col(j)) : Vector3(-R1.col(j));
        }
    }
    // Edge - edge axes. A face axis is preferred when the depths are almost
    // the same so that a stable face contact manifold is generated.
    constexpr double edgePreferenceFactor = 0.95;
    for(int i=0; i < 3; ++i){
        for(int j=0; j < 3; ++j){
            Vector3 axis = R0.col(i).cross(R1.col(j));
            const double l = axis.norm();
            if(l < 1.0e-6){
                continue; // The edges are parallel; the face axes cover this direction
            }
            axis /= l;
            double sum = 0.0;
            for(int k=0; k < 3; ++k){
                sum += a[k] * fabs(axis.dot(R0.col(k)));
                sum += b[k] * fabs(axis.dot(R1.col(k)));
            }
            const double depth = sum - fabs(axis.dot(p));
            if(depth <= 0.0){
                return false;
            }
            if(depth < minDepth * edgePreferenceFactor){
                minDepth = depth;
                bestAxis = (axis.dot(p) >= 0.0) ? axis : Vector3(-axis);
            }
        }
    }

    const Vector3& n = bestAxis;
    const Vector3 sup0 = supportCore(s0, n);
    const Vector3 sup1 = supportCore(s1, -n);
    const Vector3 witnessPoint = 0.5 * (sup0 + sup1);

    if(findFirstContactOnly){
        addContactPoint(out_points, witnessPoint, n, minDepth);
    } else {
        generateContactManifold(s0, s1, n, witnessPoint, minDepth, params, out_points);
    }
    return true;
}

/**
   Contact detection for two cylinders with the (almost) parallel axes.
   For this configuration the minimum penetration direction is either the
   radial direction or the axial direction, and the closed form solution
   avoids the slow EPA convergence on the parallel curved surfaces.
   @return true when the case is handled (including the no-contact case);
   false when the axes are not parallel and the caller should fall back to
   the generic path
*/
bool detectParallelCylinderCylinder(
    const PrimitiveCollisionShape& s0, const PrimitiveCollisionShape& s1,
    std::vector<PrimitiveContactPoint>& out_points, bool findFirstContactOnly,
    const PrimitiveCollisionParameterSet& params, bool& out_contactDetected)
{
    out_contactDetected = false;

    const Vector3 a0 = s0.T.linear().col(1);
    const Vector3 a1 = s1.T.linear().col(1);
    // The closed form below assumes exactly parallel axes. Treating merely
    // near-parallel cylinders as parallel can miss an intersection near the
    // ends of long cylinders.
    if(a0.cross(a1).squaredNorm() > 1.0e-20){
        return false;
    }
    const Vector3 p = s1.T.translation() - s0.T.translation();
    const double axialOffset = p.dot(a0);
    const Vector3 radialVec = p - axialOffset * a0;
    const double radialDist = radialVec.norm();

    const double depthRadial = (s0.radius + s1.radius) - radialDist;
    const double depthAxial = (s0.halfLength + s1.halfLength) - fabs(axialOffset);
    if(depthRadial <= 0.0 || depthAxial <= 0.0){
        return true; // no contact
    }

    Vector3 n;
    double depth;
    Vector3 witnessPoint;

    if(depthRadial <= depthAxial){
        // Side - side contact
        if(radialDist > 1.0e-12){
            n = radialVec / radialDist;
        } else {
            // The axes coincide, so every radial direction is an equivalent
            // minimum penetration direction. Select one perpendicular to the
            // common axis instead of incorrectly using the axial direction.
            n = a0.unitOrthogonal();
        }
        depth = depthRadial;
        const double lo = std::max(-s0.halfLength, axialOffset - s1.halfLength);
        const double hi = std::min(s0.halfLength, axialOffset + s1.halfLength);
        witnessPoint = s0.T.translation() + 0.5 * (lo + hi) * a0 + (s0.radius - 0.5 * depth) * n;
    } else {
        // Cap - cap contact
        n = (axialOffset >= 0.0) ? a0 : Vector3(-a0);
        depth = depthAxial;
        const Vector3 cap0 = s0.T.translation() + ((n.dot(a0) >= 0.0) ? s0.halfLength : -s0.halfLength) * a0;
        const Vector3 cap1 = s1.T.translation() + ((n.dot(a1) >= 0.0) ? -s1.halfLength : s1.halfLength) * a1;
        witnessPoint = 0.5 * (cap0 + cap1);
    }

    if(findFirstContactOnly){
        addContactPoint(out_points, witnessPoint, n, depth);
    } else {
        generateContactManifold(s0, s1, n, witnessPoint, depth, params, out_points);
    }
    out_contactDetected = true;
    return true;
}

bool detectSphereSphere(
    const PrimitiveCollisionShape& s0, const PrimitiveCollisionShape& s1,
    std::vector<PrimitiveContactPoint>& out_points)
{
    const Vector3& c0 = s0.T.translation();
    const Vector3& c1 = s1.T.translation();
    Vector3 d = c1 - c0;
    const double dist = d.norm();
    const double depth = s0.radius + s1.radius - dist;
    if(depth <= 0.0){
        return false;
    }
    Vector3 n = (dist > 1.0e-12) ? Vector3(d / dist) : Vector3::UnitZ();
    const Vector3 point = c0 + n * (s0.radius - 0.5 * depth);
    addContactPoint(out_points, point, n, depth);
    return true;
}

inline double closestPointOnSegment(const Vector3& p, const Vector3& a, const Vector3& b, Vector3& out)
{
    const Vector3 ab = b - a;
    const double l2 = ab.squaredNorm();
    double t = 0.0;
    if(l2 > 1.0e-24){
        t = std::min(1.0, std::max(0.0, (p - a).dot(ab) / l2));
    }
    out = a + t * ab;
    return t;
}

bool detectSphereCapsule(
    const PrimitiveCollisionShape& sphere, const PrimitiveCollisionShape& capsule,
    bool sphereIs0, std::vector<PrimitiveContactPoint>& out_points)
{
    const Vector3& c = sphere.T.translation();
    const Vector3 axis = capsule.T.linear().col(1);
    const Vector3& cc = capsule.T.translation();
    const Vector3 a = cc - capsule.halfLength * axis;
    const Vector3 b = cc + capsule.halfLength * axis;
    Vector3 q;
    const double t = closestPointOnSegment(c, a, b, q);
    const Vector3 d = q - c;
    const double dist = d.norm();
    const double depth = sphere.radius + capsule.radius - dist;
    if(depth <= 0.0){
        return false;
    }
    Vector3 n; // sphere -> capsule
    if(dist > 1.0e-12){
        n = d / dist;
    } else if(t <= 1.0e-12){
        n = axis;
    } else if(t >= 1.0 - 1.0e-12){
        n = -axis;
    } else {
        // The sphere center is on the interior of the capsule axis. Any
        // radial direction is a valid minimum penetration direction; use a
        // shape-local axis so the result follows rigid rotations.
        n = capsule.T.linear().col(0);
    }
    const Vector3 point = c + n * (sphere.radius - 0.5 * depth);
    addContactPoint(out_points, point, sphereIs0 ? n : Vector3(-n), depth);
    return true;
}

bool detectSphereBox(
    const PrimitiveCollisionShape& sphere, const PrimitiveCollisionShape& box,
    bool sphereIs0, std::vector<PrimitiveContactPoint>& out_points)
{
    const Vector3 p = box.T.linear().transpose() * (sphere.T.translation() - box.T.translation());
    const Vector3& h = box.halfExtents;
    Vector3 q(
        std::min(h.x(), std::max(-h.x(), p.x())),
        std::min(h.y(), std::max(-h.y(), p.y())),
        std::min(h.z(), std::max(-h.z(), p.z())));
    const Vector3 delta = p - q;
    const double dist2 = delta.squaredNorm();

    Vector3 n; // sphere -> box in the world frame
    double depth;
    Vector3 point;

    if(dist2 > 1.0e-24){
        const double dist = sqrt(dist2);
        depth = sphere.radius - dist;
        if(depth <= 0.0){
            return false;
        }
        const Vector3 nLocal = delta / dist; // box -> sphere
        n = -(box.T.linear() * nLocal);
        const Vector3 qw = box.T * q;
        point = qw - 0.5 * depth * (-n);
    } else {
        // The sphere center is inside the box
        int minAxis = 0;
        double minDist = DBL_MAX;
        for(int i=0; i < 3; ++i){
            const double dd = h[i] - fabs(p[i]);
            if(dd < minDist){
                minDist = dd;
                minAxis = i;
            }
        }
        Vector3 nLocal = Vector3::Zero();
        nLocal[minAxis] = (p[minAxis] >= 0.0) ? 1.0 : -1.0; // box center -> outward
        n = -(box.T.linear() * nLocal);
        depth = sphere.radius + minDist;
        point = sphere.T.translation() - 0.5 * depth * n;
    }
    addContactPoint(out_points, point, sphereIs0 ? n : Vector3(-n), depth);
    return true;
}

bool detectCapsuleCapsule(
    const PrimitiveCollisionShape& s0, const PrimitiveCollisionShape& s1,
    std::vector<PrimitiveContactPoint>& out_points)
{
    const Vector3 axis0 = s0.T.linear().col(1);
    const Vector3 axis1 = s1.T.linear().col(1);
    const Vector3 a0 = s0.T.translation() - s0.halfLength * axis0;
    const Vector3 b0 = s0.T.translation() + s0.halfLength * axis0;
    const Vector3 a1 = s1.T.translation() - s1.halfLength * axis1;
    const Vector3 b1 = s1.T.translation() + s1.halfLength * axis1;

    const Vector3 u = b0 - a0;
    const Vector3 v = b1 - a1;
    const Vector3 r = a0 - a1;
    const double a = u.dot(u);
    const double b = u.dot(v);
    const double c = v.dot(v);
    const double d = u.dot(r);
    const double e = v.dot(r);
    const double denom = a * c - b * b;

    const double rsum = s0.radius + s1.radius;

    if(a < 1.0e-24){
        PrimitiveCollisionShape sphere0 = s0;
        sphere0.type = PrimitiveCollisionShape::Sphere;
        if(c < 1.0e-24){
            PrimitiveCollisionShape sphere1 = s1;
            sphere1.type = PrimitiveCollisionShape::Sphere;
            return detectSphereSphere(sphere0, sphere1, out_points);
        }
        return detectSphereCapsule(sphere0, s1, true, out_points);
    }
    if(c < 1.0e-24){
        PrimitiveCollisionShape sphere1 = s1;
        sphere1.type = PrimitiveCollisionShape::Sphere;
        return detectSphereCapsule(sphere1, s0, false, out_points);
    }

    const bool isParallel = (denom <= 1.0e-9 * a * c);

    if(!isParallel){
        double s = (b * e - c * d) / denom;
        s = std::min(1.0, std::max(0.0, s));
        double t = (c > 1.0e-24) ? (b * s + e) / c : 0.0;
        if(t < 0.0){
            t = 0.0;
            s = (a > 1.0e-24) ? std::min(1.0, std::max(0.0, -d / a)) : 0.0;
        } else if(t > 1.0){
            t = 1.0;
            s = (a > 1.0e-24) ? std::min(1.0, std::max(0.0, (b - d) / a)) : 0.0;
        }
        const Vector3 p = a0 + s * u;
        const Vector3 q = a1 + t * v;
        const Vector3 diff = q - p;
        const double dist = diff.norm();
        const double depth = rsum - dist;
        if(depth <= 0.0){
            return false;
        }
        Vector3 n;
        if(dist > 1.0e-12){
            n = diff / dist;
        } else {
            n = u.cross(v).normalized();
        }
        addContactPoint(out_points, p + n * (s0.radius - 0.5 * depth), n, depth);
        return true;

    } else {
        // Parallel capsules: generate up to two contact points over the
        // overlapping range for the resting stability
        const double ul2 = a;
        const Vector3 zeroDistanceNormal = s0.T.linear().col(0);
        double t0 = (a1 - a0).dot(u) / ul2;
        double t1 = (b1 - a0).dot(u) / ul2;
        if(t0 > t1){
            std::swap(t0, t1);
        }
        const double lo = std::max(0.0, t0);
        const double hi = std::min(1.0, t1);
        if(lo > hi){
            // No overlap in the axis direction: end-to-end contact
            Vector3 p, q;
            const double distA = (a1 - b0).squaredNorm();
            closestPointOnSegment((distA < (b1 - a0).squaredNorm()) ? a1 : b1, a0, b0, p);
            closestPointOnSegment(p, a1, b1, q);
            const Vector3 diff = q - p;
            const double dist = diff.norm();
            const double depth = rsum - dist;
            if(depth <= 0.0){
                return false;
            }
            const Vector3 n = (dist > 1.0e-12) ? Vector3(diff / dist) : zeroDistanceNormal;
            addContactPoint(out_points, p + n * (s0.radius - 0.5 * depth), n, depth);
            return true;
        }
        bool detected = false;
        const int numSamples = (hi - lo > 1.0e-12) ? 2 : 1;
        for(int i=0; i < numSamples; ++i){
            const double t = (i == 0) ? lo : hi;
            const Vector3 p = a0 + t * u;
            Vector3 q;
            closestPointOnSegment(p, a1, b1, q);
            const Vector3 diff = q - p;
            const double dist = diff.norm();
            const double depth = rsum - dist;
            if(depth > 0.0){
                const Vector3 n = (dist > 1.0e-12) ? Vector3(diff / dist) : zeroDistanceNormal;
                addContactPoint(out_points, p + n * (s0.radius - 0.5 * depth), n, depth);
                detected = true;
            }
        }
        return detected;
    }
}

}


bool cnoid::detectPrimitiveShapeCollision
(const PrimitiveCollisionShape& shape0, const PrimitiveCollisionShape& shape1,
 std::vector<PrimitiveContactPoint>& out_points, bool findFirstContactOnly,
 const PrimitiveCollisionParameterSet& parameters)
{
    typedef PrimitiveCollisionShape PCS;

    // Early rejection with the bounding spheres
    if(shape0.type != PCS::Triangle && shape1.type != PCS::Triangle){
        const double rsum = boundingSphereRadius(shape0) + boundingSphereRadius(shape1);
        if((shape1.T.translation() - shape0.T.translation()).squaredNorm() > rsum * rsum){
            return false;
        }
    }

    // Closed-form fast paths
    if(shape0.type == PCS::Sphere){
        if(shape1.type == PCS::Sphere){
            return detectSphereSphere(shape0, shape1, out_points);
        } else if(shape1.type == PCS::Capsule){
            return detectSphereCapsule(shape0, shape1, true, out_points);
        } else if(shape1.type == PCS::Box){
            return detectSphereBox(shape0, shape1, true, out_points);
        }
    } else if(shape1.type == PCS::Sphere){
        if(shape0.type == PCS::Capsule){
            return detectSphereCapsule(shape1, shape0, false, out_points);
        } else if(shape0.type == PCS::Box){
            return detectSphereBox(shape1, shape0, false, out_points);
        }
    } else if(shape0.type == PCS::Capsule && shape1.type == PCS::Capsule){
        return detectCapsuleCapsule(shape0, shape1, out_points);
    } else if(shape0.type == PCS::Box && shape1.type == PCS::Box){
        return detectBoxBox(shape0, shape1, out_points, findFirstContactOnly, parameters);
    } else if(shape0.type == PCS::Cylinder && shape1.type == PCS::Cylinder){
        bool contactDetected;
        if(detectParallelCylinderCylinder(
               shape0, shape1, out_points, findFirstContactOnly, parameters, contactDetected)){
            return contactDetected;
        }
        // Fall through to the generic path for the non-parallel cylinders
    }

    // Generic convex path with GJK / EPA
    const double m0 = margin(shape0);
    const double m1 = margin(shape1);
    const double marginSum = m0 + m1;

    GjkResult gjk;
    runGjk(shape0, shape1, gjk);

    Vector3 n;
    double depth;
    Vector3 pOnSurface0, pOnSurface1;

    if(!gjk.coresIntersecting){
        const double dist = gjk.distance;
        if(dist >= marginSum){
            return false;
        }
        if(dist > 1.0e-9){
            n = (gjk.pB - gjk.pA) / dist;
            depth = marginSum - dist;
            pOnSurface0 = gjk.pA + m0 * n;
            pOnSurface1 = gjk.pB - m1 * n;
        } else {
            // The core shapes are touching; the normal from the witness
            // points is unreliable. Fall back to the EPA on the cores or
            // to the triangle face normal.
            if(shape1.type == PCS::Triangle || shape0.type == PCS::Triangle){
                const PCS& tri = (shape1.type == PCS::Triangle) ? shape1 : shape0;
                Vector3 tn = (tri.vertices[1] - tri.vertices[0]).cross(tri.vertices[2] - tri.vertices[0]);
                const double l = tn.norm();
                if(l < 1.0e-15){
                    return false;
                }
                tn /= l;
                // Direct the normal from shape0 toward shape1
                const Vector3 c0 = shapeCenter(shape0);
                const Vector3 c1 = shapeCenter(shape1);
                if(tn.dot(c1 - c0) < 0.0){
                    tn = -tn;
                }
                n = tn;
                depth = marginSum;
                pOnSurface0 = gjk.pA + m0 * n;
                pOnSurface1 = gjk.pB - m1 * n;
            } else {
                Vector3 pA, pB;
                double coreDepth;
                if(!runEpa(shape0, shape1, gjk.simplex, n, coreDepth, pA, pB)){
                    return false;
                }
                depth = coreDepth + marginSum;
                pOnSurface0 = pA + m0 * n;
                pOnSurface1 = pB - m1 * n;
            }
        }
    } else {
        Vector3 pA, pB;
        double coreDepth;
        if(!runEpa(shape0, shape1, gjk.simplex, n, coreDepth, pA, pB)){
            // The Minkowski difference is nearly flat (e.g. a sphere center
            // just on a triangle); use the triangle normal if available.
            if(shape1.type == PCS::Triangle || shape0.type == PCS::Triangle){
                const PCS& tri = (shape1.type == PCS::Triangle) ? shape1 : shape0;
                Vector3 tn = (tri.vertices[1] - tri.vertices[0]).cross(tri.vertices[2] - tri.vertices[0]);
                const double l = tn.norm();
                if(l < 1.0e-15){
                    return false;
                }
                tn /= l;
                const Vector3 c0 = shapeCenter(shape0);
                const Vector3 c1 = shapeCenter(shape1);
                if(tn.dot(c1 - c0) < 0.0){
                    tn = -tn;
                }
                n = tn;
                depth = marginSum;
                pOnSurface0 = shapeCenter(shape0) + m0 * n;
                pOnSurface1 = pOnSurface0;
            } else {
                return false;
            }
        } else {
            depth = coreDepth + marginSum;
            pOnSurface0 = pA + m0 * n;
            pOnSurface1 = pB - m1 * n;
        }
    }

    if(depth <= 0.0){
        return false;
    }

    const Vector3 witnessPoint = 0.5 * (pOnSurface0 + pOnSurface1);

    if(findFirstContactOnly){
        addContactPoint(out_points, witnessPoint, n, depth);
        return true;
    }

    generateContactManifold(shape0, shape1, n, witnessPoint, depth, parameters, out_points);
    return true;
}
