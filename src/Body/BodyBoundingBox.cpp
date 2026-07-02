#include "BodyBoundingBox.h"
#include "Body.h"
#include "Link.h"
#include <cnoid/SceneDrawables>
#include <cnoid/MeshExtractor>
#include <QuickHull.hpp>
#include <vector>

using namespace std;
using namespace cnoid;

namespace {

// Default is looser than QuickHull's own float default (1e-4). For a
// bounding-box application, a hull that misses features smaller than
// 0.1% of the model's size is more than acceptable and produces a much
// smaller vertex set to iterate every frame.
constexpr float DefaultPrecision = 1.0e-3f;

inline void expandByTransformedPoints(
    BoundingBox& bbox,
    const Isometry3& T,
    const vector<Vector3f>& points)
{
    // Cast the transform to float once so the inner loop stays in
    // single-precision. The BoundingBox itself keeps its double API;
    // the promotion happens on the last expandBy() call.
    const Matrix3f R = T.linear().cast<float>();
    const Vector3f t = T.translation().cast<float>();
    for(const auto& p : points){
        Vector3f q = R * p + t;
        bbox.expandBy(Vector3(q.x(), q.y(), q.z()));
    }
}


// Both boxes are updated in a single pass over `points`, so each cached
// vertex is loaded from memory only once regardless of how many bounding
// boxes are being computed. This is the whole reason compute(Type,Type)
// exists.
inline void expandByTransformedPointsPair(
    BoundingBox& bbox1, const Isometry3& T1,
    BoundingBox& bbox2, const Isometry3& T2,
    const vector<Vector3f>& points)
{
    const Matrix3f R1 = T1.linear().cast<float>();
    const Vector3f t1 = T1.translation().cast<float>();
    const Matrix3f R2 = T2.linear().cast<float>();
    const Vector3f t2 = T2.translation().cast<float>();
    for(const auto& p : points){
        Vector3f q1 = R1 * p + t1;
        Vector3f q2 = R2 * p + t2;
        bbox1.expandBy(Vector3(q1.x(), q1.y(), q1.z()));
        bbox2.expandBy(Vector3(q2.x(), q2.y(), q2.z()));
    }
}

} // namespace


namespace cnoid {

class BodyBoundingBox::Impl
{
public:
    Body* body;
    float precision;

    // Per-link cached point set. Currently this holds the convex-hull
    // vertices of the link's shape mesh in link-local coordinates; if the
    // convex hull cannot be computed (degenerate mesh) the raw mesh
    // vertices are used instead.
    struct LinkCache {
        vector<Vector3f> localPoints;
    };
    vector<LinkCache> linkCaches;
    bool cacheBuilt;

    Impl(Body* body);
    void invalidateCache();
    void buildCache();
    void computeInternal(
        BoundingBox* out1, BodyBoundingBox::Type type1,
        BoundingBox* out2, BodyBoundingBox::Type type2);
};

}


BodyBoundingBox::BodyBoundingBox(Body* body)
{
    impl = new Impl(body);
}


BodyBoundingBox::~BodyBoundingBox()
{
    delete impl;
}


BodyBoundingBox::Impl::Impl(Body* body)
    : body(body),
      precision(DefaultPrecision),
      cacheBuilt(false)
{

}


void BodyBoundingBox::Impl::invalidateCache()
{
    linkCaches.clear();
    cacheBuilt = false;
}


void BodyBoundingBox::setConvexHullPrecision(float precision)
{
    if(precision <= 0.0f){
        return;
    }
    if(precision == impl->precision){
        return;
    }
    impl->precision = precision;
    impl->invalidateCache();
}


float BodyBoundingBox::convexHullPrecision() const
{
    return impl->precision;
}


void BodyBoundingBox::Impl::buildCache()
{
    linkCaches.clear();
    cacheBuilt = true;
    if(!body){
        return;
    }
    const int numLinks = body->numLinks();
    linkCaches.resize(numLinks);

    quickhull::QuickHull<float> qh;
    MeshExtractor extractor;
    vector<quickhull::Vector3<float>> allPoints;

    for(int i = 0; i < numLinks; ++i){
        auto* link = body->link(i);
        auto* shape = link->shape();
        if(!shape){
            continue;
        }

        // Gather all mesh vertices in link-local coordinates. The
        // MeshExtractor's currentTransform() takes care of any SgTransform
        // nodes that sit between the link's shape group and its meshes.
        allPoints.clear();
        extractor.extract(shape, [&](){
            auto* mesh = extractor.currentMesh();
            if(!mesh || !mesh->hasVertices()) return;
            const Affine3& T = extractor.currentTransform();
            const Matrix3f R = T.linear().cast<float>();
            const Vector3f Tt = T.translation().cast<float>();
            auto& verts = *mesh->vertices();
            allPoints.reserve(allPoints.size() + verts.size());
            for(auto& v : verts){
                Vector3f p = R * v + Tt;
                allPoints.push_back({p.x(), p.y(), p.z()});
            }
        });

        if(allPoints.empty()){
            continue;
        }

        auto& localPoints = linkCaches[i].localPoints;

        // Compute the convex hull to reduce the vertex count while still
        // giving a tight enough AABB after transformation. Fall back to
        // the raw point set when the hull cannot be produced.
        try {
            auto hull = qh.getConvexHull(allPoints, true, false, precision);
            const auto& hullVerts = hull.getVertexBuffer();
            if(hullVerts.size() >= 4){
                localPoints.reserve(hullVerts.size());
                for(auto& v : hullVerts){
                    localPoints.emplace_back(v.x, v.y, v.z);
                }
            }
        } catch(...){
            // Ignore and fall through to the fallback below.
        }

        if(localPoints.empty()){
            localPoints.reserve(allPoints.size());
            for(auto& v : allPoints){
                localPoints.emplace_back(v.x, v.y, v.z);
            }
        }
        localPoints.shrink_to_fit();
    }
}


void BodyBoundingBox::Impl::computeInternal(
    BoundingBox* out1, BodyBoundingBox::Type type1,
    BoundingBox* out2, BodyBoundingBox::Type type2)
{
    if(out1) out1->clear();
    if(out2) out2->clear();
    if(!body) return;

    if(!cacheBuilt){
        buildCache();
    }

    // Prepare the root-inverse transform only if at least one output slot
    // asks for a root-local box.
    const bool need_root =
        (out1 && type1 == BodyBoundingBox::RootLocalAABB) ||
        (out2 && type2 == BodyBoundingBox::RootLocalAABB);
    Isometry3 T_root_inv = Isometry3::Identity();
    if(need_root){
        if(auto rootLink = body->rootLink()){
            T_root_inv = rootLink->T().inverse();
        }
    }

    auto transformFor = [&](BodyBoundingBox::Type t, const Isometry3& T_link){
        return t == BodyBoundingBox::WorldAABB ? T_link : T_root_inv * T_link;
    };

    const int numLinks = body->numLinks();
    for(int i = 0; i < numLinks; ++i){
        if(i >= static_cast<int>(linkCaches.size())) break;
        auto& points = linkCaches[i].localPoints;
        if(points.empty()) continue;

        auto* link = body->link(i);
        const Isometry3& T_link = link->T();

        if(out1 && out2){
            // Fuse both updates into a single pass over `points` so the
            // cached vertex data is streamed through the CPU once.
            expandByTransformedPointsPair(
                *out1, transformFor(type1, T_link),
                *out2, transformFor(type2, T_link),
                points);
        } else if(out1){
            expandByTransformedPoints(*out1, transformFor(type1, T_link), points);
        } else if(out2){
            expandByTransformedPoints(*out2, transformFor(type2, T_link), points);
        }
    }
}


BoundingBox BodyBoundingBox::compute(Type type) const
{
    BoundingBox bb;
    impl->computeInternal(&bb, type, nullptr, RootLocalAABB);
    return bb;
}


std::pair<BoundingBox, BoundingBox> BodyBoundingBox::compute(Type type1, Type type2) const
{
    std::pair<BoundingBox, BoundingBox> result;
    impl->computeInternal(&result.first, type1, &result.second, type2);
    return result;
}
