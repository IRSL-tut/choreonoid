/**
   @author Shin'ichiro Nakaoka
   @author Rafael Cisneros
*/

#include "ColdetModelPair.h"
#include "ColdetModelInternalModel.h"
#include "StdCollisionPairInserter.h"
#include "PrimitiveCollision.h"
#include "Opcode/Opcode.h"
#include "SSVTreeCollider.h"
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

bool isConvexPrimitiveType(int type)
{
    return (type == ColdetModel::SP_BOX ||
            type == ColdetModel::SP_CYLINDER ||
            type == ColdetModel::SP_CONE ||
            type == ColdetModel::SP_SPHERE ||
            type == ColdetModel::SP_CAPSULE);
}

void setPrimitiveContactData
(vector<collision_data>& cdata, const vector<PrimitiveContactPoint>& points, bool flipNormal)
{
    for(auto& p : points){
        collision_data cd;
        cd.id1 = 0;
        cd.id2 = 0;
        cd.num_of_i_points = 1;
        cd.i_points[0] = p.point;
        cd.i_point_new[0] = 1;
        cd.i_point_new[1] = cd.i_point_new[2] = cd.i_point_new[3] = 0;
        cd.n_vector = flipNormal ? Vector3(-p.normal) : p.normal;
        cd.depth = p.depth;
        cd.n = cd.n_vector;
        cd.m = -cd.n_vector;
        cd.c_type = 1;
        cdata.push_back(cd);
    }
}

void collectTrianglesInAabb
(const Opcode::AABBCollisionNode* node, const Vector3& center, const Vector3& halfExtents, vector<int>& out_triangles)
{
    const IceMaths::Point& c = node->mAABB.mCenter;
    const IceMaths::Point& e = node->mAABB.mExtents;
    if(fabs(c.x - center.x()) > e.x + halfExtents.x() ||
       fabs(c.y - center.y()) > e.y + halfExtents.y() ||
       fabs(c.z - center.z()) > e.z + halfExtents.z()){
        return;
    }
    if(node->IsLeaf()){
        out_triangles.push_back(node->GetPrimitive());
    } else {
        collectTrianglesInAabb(node->GetPos(), center, halfExtents, out_triangles);
        collectTrianglesInAabb(node->GetNeg(), center, halfExtents, out_triangles);
    }
}
}


ColdetModelPair::ColdetModelPair()
{
    collisionPairInserter = new Opcode::StdCollisionPairInserter;
}


ColdetModelPair::ColdetModelPair(ColdetModel* model0, ColdetModel* model1, double tolerance)
{
    collisionPairInserter = new Opcode::StdCollisionPairInserter;
    set(model0, model1);
    tolerance_ = tolerance;
}


ColdetModelPair::ColdetModelPair(const ColdetModelPair& org)
{
    collisionPairInserter = new Opcode::StdCollisionPairInserter;
    set(org.models[0], org.models[1]);
    tolerance_ = org.tolerance_;
    primitiveCollisionParameterSet_ = org.primitiveCollisionParameterSet_;
}


ColdetModelPair::~ColdetModelPair()
{
    delete collisionPairInserter;
}


void ColdetModelPair::set(ColdetModel* model0, ColdetModel* model1)
{
    models[0] = model0;
    models[1] = model1;
    // inverse order because of historical background
    // this should be fixed.(note that the direction of normal is inversed when the order inversed 
    if(model0 && model1){
        collisionPairInserter->set(model1->internalModel, model0->internalModel);
    }
}


std::vector<collision_data>& ColdetModelPair::detectCollisionsSub(bool detectAllContacts)
{
    collisionPairInserter->clear();

    const bool convex0 = isConvexPrimitiveType(models[0]->getPrimitiveType());
    const bool convex1 = isConvexPrimitiveType(models[1]->getPrimitiveType());
    bool detected;

    if(convex0 && convex1){
        detected = detectPrimitivePairCollisions(detectAllContacts);
    } else if(convex0 || convex1){
        detected = detectPrimitiveMeshCollisions(detectAllContacts);
    } else {
        detected = detectMeshMeshCollisions(detectAllContacts);
    }

    if(!detected){
        collisionPairInserter->clear();
    }

    return collisionPairInserter->collisions();
}


const PrimitiveCollisionParameterSet& ColdetModelPair::primitiveCollisionParameterSet() const
{
    static const PrimitiveCollisionParameterSet defaultParameterSet;
    return primitiveCollisionParameterSet_ ? *primitiveCollisionParameterSet_ : defaultParameterSet;
}


bool ColdetModelPair::makePrimitiveShape(ColdetModel* model, PrimitiveCollisionShape& out_shape)
{
    const auto internalModel = model->internalModel;
    const auto& params = internalModel->pParams;
    const Isometry3 T = model->position_ * model->primitiveLocalPosition_;

    switch(internalModel->pType){

    case ColdetModel::SP_BOX:
        if(params.size() < 3){
            return false;
        }
        out_shape.setBox(T, Vector3(params[0], params[1], params[2]));
        return true;

    case ColdetModel::SP_SPHERE:
        if(params.size() < 1){
            return false;
        }
        out_shape.setSphere(T, params[0]);
        return true;

    case ColdetModel::SP_CYLINDER:
        if(params.size() < 2){
            return false;
        }
        out_shape.setCylinder(T, params[0], params[1]);
        return true;

    case ColdetModel::SP_CONE:
        if(params.size() < 2){
            return false;
        }
        out_shape.setCone(T, params[0], params[1]);
        return true;

    case ColdetModel::SP_CAPSULE:
        if(params.size() < 2){
            return false;
        }
        out_shape.setCapsule(T, params[0], params[1]);
        return true;

    default:
        break;
    }
    return false;
}


bool ColdetModelPair::detectPrimitivePairCollisions(bool detectAllContacts)
{
    PrimitiveCollisionShape shape0, shape1;
    if(!makePrimitiveShape(models[0], shape0) || !makePrimitiveShape(models[1], shape1)){
        // The primitive parameters are not valid; fall back to the mesh-based detection
        return detectMeshMeshCollisions(detectAllContacts);
    }
    vector<PrimitiveContactPoint> points;
    if(!detectPrimitiveShapeCollision(
           shape0, shape1, points, !detectAllContacts, primitiveCollisionParameterSet())){
        return false;
    }
    setPrimitiveContactData(collisionPairInserter->collisions(), points, false);
    return !points.empty();
}


bool ColdetModelPair::detectPrimitiveMeshCollisions(bool detectAllContacts)
{
    ColdetModel* primModel;
    ColdetModel* meshModel;
    bool reversed; // true when models[0] is the mesh side

    if(isConvexPrimitiveType(models[0]->getPrimitiveType())){
        primModel = models[0];
        meshModel = models[1];
        reversed = false;
    } else {
        primModel = models[1];
        meshModel = models[0];
        reversed = true;
    }
    if(!meshModel->isValid()){
        return false;
    }
    PrimitiveCollisionShape primShape;
    if(!makePrimitiveShape(primModel, primShape)){
        return detectMeshMeshCollisions(detectAllContacts);
    }

    // Compute the AABB of the primitive shape in the mesh-local frame
    // to collect the candidate triangles
    const Isometry3& T_mesh = meshModel->position_;
    const Isometry3 T_local = T_mesh.inverse() * primShape.T;

    Vector3 he; // The half extents of the box enclosing the primitive shape
    switch(primShape.type){
    case PrimitiveCollisionShape::Sphere:
        he = Vector3(primShape.radius, primShape.radius, primShape.radius);
        break;
    case PrimitiveCollisionShape::Box:
        he = primShape.halfExtents;
        break;
    case PrimitiveCollisionShape::Cylinder:
    case PrimitiveCollisionShape::Cone:
        he = Vector3(primShape.radius, primShape.halfLength, primShape.radius);
        break;
    case PrimitiveCollisionShape::Capsule:
        he = Vector3(primShape.radius, primShape.halfLength + primShape.radius, primShape.radius);
        break;
    default:
        return false;
    }
    constexpr double aabbMargin = 1.0e-6;
    const Vector3 aabbHalfExtents =
        T_local.linear().cwiseAbs() * he + Vector3::Constant(aabbMargin);
    const Vector3 aabbCenter = T_local.translation();

    const auto tree = static_cast<const Opcode::AABBCollisionTree*>(meshModel->internalModel->model.GetTree());
    if(!tree){
        return false;
    }
    vector<int> candidateTriangles;
    collectTrianglesInAabb(tree->GetNodes(), aabbCenter, aabbHalfExtents, candidateTriangles);
    if(candidateTriangles.empty()){
        return false;
    }

    const auto& vertices = meshModel->internalModel->vertices;
    const auto& triangles = meshModel->internalModel->triangles;

    vector<PrimitiveContactPoint> points;
    vector<char> isFaceAligned; // one flag per contact point
    bool detected = false;
    for(int index : candidateTriangles){
        const udword* tri = triangles[index].mVRef;
        Vector3 v[3];
        for(int k=0; k < 3; ++k){
            const IceMaths::Point& p = vertices[tri[k]];
            v[k] = T_mesh * Vector3(p.x, p.y, p.z);
        }
        PrimitiveCollisionShape triShape;
        triShape.setTriangle(v[0], v[1], v[2]);
        const size_t pointIndexTop = points.size();
        if(detectPrimitiveShapeCollision(
               primShape, triShape, points, !detectAllContacts, primitiveCollisionParameterSet())){
            detected = true;
            Vector3 faceNormal = (v[1] - v[0]).cross(v[2] - v[0]);
            const double l = faceNormal.norm();
            if(l > 1.0e-15){
                faceNormal /= l;
            }
            for(size_t i = pointIndexTop; i < points.size(); ++i){
                isFaceAligned.push_back(fabs(points[i].normal.dot(faceNormal)) > 0.9999);
            }
            if(!detectAllContacts){
                break;
            }
        }
    }
    if(!detected){
        return false;
    }

    /**
       Filter the contact points.
       - The identical points generated on the shared edges and vertices of
         the adjacent triangles are unified.
       - The "internal edge" artifacts are suppressed: a contact whose normal
         is not aligned with the face normal of its triangle (i.e. an edge or
         vertex contact) is removed if a face-aligned contact exists near it,
         because such a contact is usually a spurious one generated on the
         edge shared by the adjacent triangles of a flat or convex region.
    */
    bool anyFaceAligned = false;
    for(char flag : isFaceAligned){
        if(flag){
            anyFaceAligned = true;
            break;
        }
    }
    const double weldRadius = std::min(0.1 * he.minCoeff(), 0.02);
    const double weldRadius2 = weldRadius * weldRadius;

    vector<PrimitiveContactPoint> filteredPoints;
    filteredPoints.reserve(points.size());
    for(size_t i=0; i < points.size(); ++i){
        auto& p = points[i];
        if(!isFaceAligned[i] && anyFaceAligned){
            bool nearFaceContact = false;
            for(size_t j=0; j < points.size(); ++j){
                if(isFaceAligned[j] && (p.point - points[j].point).squaredNorm() < weldRadius2){
                    nearFaceContact = true;
                    break;
                }
            }
            if(nearFaceContact){
                continue;
            }
        }
        bool duplicated = false;
        for(auto& q : filteredPoints){
            if((p.point - q.point).squaredNorm() < 1.0e-12 && p.normal.dot(q.normal) > 0.99){
                duplicated = true;
                break;
            }
        }
        if(!duplicated){
            filteredPoints.push_back(p);
        }
    }
    setPrimitiveContactData(collisionPairInserter->collisions(), filteredPoints, reversed);
    return true;
}

bool ColdetModelPair::detectMeshMeshCollisions(bool detectAllContacts)
{
    bool result = false;
    
    if(models[0]->isValid() && models[1]->isValid()){

        Opcode::BVTCache colCache;

        // inverse order because of historical background
        // this should be fixed.(note that the direction of normal is inversed when the order inversed 
        colCache.Model0 = &models[1]->internalModel->model;
        colCache.Model1 = &models[0]->internalModel->model;
        
        if(colCache.Model0->HasSingleNode() || colCache.Model1->HasSingleNode())
            return result;

        Opcode::AABBTreeCollider collider;
        collider.setCollisionPairInserter(collisionPairInserter);
        
        if(!detectAllContacts){
            collider.SetFirstContact(true);
        }
        
        bool isOk = collider.Collide(colCache, models[1]->transform, models[0]->transform);
		
        if (!isOk)
            std::cerr << "AABBTreeCollider::Collide() failed" << std::endl;
		
        result = collider.GetContactStatus();
        
        boxTestsCount = collider.GetNbBVBVTests();
        triTestsCount = collider.GetNbPrimPrimTests();
    }

    return result;
}


double ColdetModelPair::computeDistance(ColdetModel* model0, ColdetModel* model1, double* point0, double* point1)
{
    if(model0->isValid() && model1->isValid()){

        Opcode::BVTCache colCache;

        colCache.Model0 = &model1->internalModel->model;
        colCache.Model1 = &model0->internalModel->model;
        
        Opcode::SSVTreeCollider collider;
        
        float d;
        Point p0, p1;
        collider.Distance(colCache, d, p0, p1,
                          model1->transform, model0->transform);
        point0[0] = p1.x;
        point0[1] = p1.y;
        point0[2] = p1.z;
        point1[0] = p0.x;
        point1[1] = p0.y;
        point1[2] = p0.z;
        return d;
    }

    return -1.0;
}


double ColdetModelPair::computeDistance(double* point0, double* point1)
{
    return computeDistance(models[0], models[1], point0, point1);
}
    

double ColdetModelPair::computeDistance(int& triangle0, double* point0, int& triangle1, double* point1)
{
    if(models[0]->isValid() && models[1]->isValid()){

        Opcode::BVTCache colCache;

        colCache.Model0 = &models[1]->internalModel->model;
        colCache.Model1 = &models[0]->internalModel->model;
        
        Opcode::SSVTreeCollider collider;
        
        float d;
        Point p0, p1;
        collider.Distance(colCache, d, p0, p1,
                          models[1]->transform, models[0]->transform);
        point0[0] = p1.x;
        point0[1] = p1.y;
        point0[2] = p1.z;
        point1[0] = p0.x;
        point1[1] = p0.y;
        point1[2] = p0.z;
        triangle1 = colCache.id0;
        triangle0 = colCache.id1;
        return d;
    }

    return -1.0;
}


bool ColdetModelPair::detectIntersection()
{
    if(models[0]->isValid() && models[1]->isValid()){

        Opcode::BVTCache colCache;

        colCache.Model0 = &models[1]->internalModel->model;
        colCache.Model1 = &models[0]->internalModel->model;
        
        Opcode::SSVTreeCollider collider;
        
        return collider.Collide(colCache, tolerance_, 
                                models[1]->transform, models[0]->transform);
    }

    return false;
}

void ColdetModelPair::setCollisionPairInserter(Opcode::CollisionPairInserter* inserter)
{
    delete collisionPairInserter;
    collisionPairInserter = inserter;
    // inverse order because of historical background
    // this should be fixed.(note that the direction of normal is inversed when the order inversed 
    collisionPairInserter->set(models[1]->internalModel, models[0]->internalModel);
}
