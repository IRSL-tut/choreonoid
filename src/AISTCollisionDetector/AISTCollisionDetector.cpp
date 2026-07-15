#include "AISTCollisionDetector.h"
#include "ColdetModelPair.h"
#include "PrimitiveCollision.h"
#include <cnoid/IdPair>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshExtractor>
#include <cnoid/ThreadPool>
#include <cnoid/PutPropertyFunction>
#include <cnoid/ValueTree>
#include <algorithm>
#include <memory>
#include <set>
#include <atomic>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

typedef CollisionDetector::GeometryHandle GeometryHandle;

CollisionDetector* factory()
{
    return new AISTCollisionDetector;
}

struct FactoryRegistration
{
    FactoryRegistration(){
        CollisionDetector::registerFactory("AISTCollisionDetector", factory);
    }
} factoryRegistration;

class ColdetModelEx;
typedef ref_ptr<ColdetModelEx> ColdetModelExPtr;

class ColdetModelEx : public ColdetModel
{
public:
    ReferencedPtr object;
    int groupId;
    bool isEnabled;
    bool isStatic;
    ColdetModelExPtr sibling;

    /**
       Bounding sphere of the whole geometry including all the sibling
       models, which is set on the main (top) model of the chain and used
       for rejecting distant geometry pairs before the sub pair iteration.
       The radius is negative when the sphere is not available.
    */
    Vector3 boundingSphereLocalCenter;
    Vector3 boundingSphereCenter;
    double boundingSphereRadius;

    /**
       Bounding sphere of this component model itself, which is used for
       rejecting distant sub pairs in the iteration over the component
       combinations of the composite geometries.
    */
    Vector3 componentSphereLocalCenter;
    Vector3 componentSphereCenter;
    double componentSphereRadius;

    /**
       True for the model which holds the merged mesh of the primitive
       components. The model is used only for the distance and ray casting
       queries and is excluded from the collision pair detection to avoid
       counting the same shapes twice.
    */
    bool isQueryOnlyModel;

    ColdetModelEx() : groupId(0), isEnabled(true), isStatic(false), isQueryOnlyModel(false) {
        boundingSphereLocalCenter.setZero();
        boundingSphereCenter.setZero();
        boundingSphereRadius = -1.0;
        componentSphereLocalCenter.setZero();
        componentSphereCenter.setZero();
        componentSphereRadius = -1.0;
    }
};

//! Check the overlap of the geometry-level bounding spheres of a model pair
inline bool testGeometryBoundingSphereOverlap(ColdetModelEx* model0, ColdetModelEx* model1)
{
    if(model0->boundingSphereRadius < 0.0 || model1->boundingSphereRadius < 0.0){
        return true; // The spheres are not available; the pair cannot be rejected
    }
    const double rsum = model0->boundingSphereRadius + model1->boundingSphereRadius;
    return (model0->boundingSphereCenter - model1->boundingSphereCenter).squaredNorm() <= rsum * rsum;
}

//! Check the overlap of the component-level bounding spheres of a model pair
inline bool testComponentBoundingSphereOverlap(ColdetModelEx* model0, ColdetModelEx* model1)
{
    if(model0->componentSphereRadius < 0.0 || model1->componentSphereRadius < 0.0){
        return true;
    }
    const double rsum = model0->componentSphereRadius + model1->componentSphereRadius;
    return (model0->componentSphereCenter - model1->componentSphereCenter).squaredNorm() <= rsum * rsum;
}

class ColdetModelPairEx;
typedef ref_ptr<ColdetModelPairEx> ColdetModelPairExPtr;

ColdetModelEx* getColdetModel(GeometryHandle handle)
{
    return reinterpret_cast<ColdetModelEx*>(handle);
}

GeometryHandle getHandle(ColdetModelEx* model)
{
    return reinterpret_cast<GeometryHandle>(model);
}

class ColdetModelPairEx : public ColdetModelPair
{
    ColdetModelPairEx(){ }

public:
    ColdetModelPairEx(
        ColdetModelEx* model1, ColdetModelEx* model2,
        std::shared_ptr<PrimitiveCollisionParameterSet> primitiveCollisionParameterSet)
        : ColdetModelPair(model1, model2)
    {
        setPrimitiveCollisionParameterSet(primitiveCollisionParameterSet);

        // Create the sub pairs for all the combinations of the model
        // components when the models are composite ones
        ColdetModelPairEx* last = this;
        for(ColdetModelEx* m1 = model1; m1; m1 = m1->sibling){
            if(m1->isQueryOnlyModel){
                continue;
            }
            for(ColdetModelEx* m2 = model2; m2; m2 = m2->sibling){
                if(m2->isQueryOnlyModel){
                    continue;
                }
                if(m1 == model1 && m2 == model2){
                    continue; // handled by this pair itself
                }
                auto subPair = new ColdetModelPairEx;
                subPair->set(m1, m2);
                subPair->setPrimitiveCollisionParameterSet(primitiveCollisionParameterSet);
                last->sibling = subPair;
                last = subPair;
            }
        }
    }

    ColdetModelEx* model(int which) {
        return static_cast<ColdetModelEx*>(ColdetModelPair::model(which));
    }

    ColdetModelPairExPtr sibling;
};


/**
   Compute the bounding sphere covering all the component models of the
   geometry and store it in the main model. All the component meshes are
   represented in the same geometry-local frame.
*/
void computeGeometryBoundingSphere(ColdetModelEx* mainModel)
{
    Vector3 lower, upper;
    bool initialized = false;
    for(ColdetModelEx* model = mainModel; model; model = model->sibling){
        Vector3 center, halfExtents;
        if(model->getLocalBoundingBox(center, halfExtents)){
            model->componentSphereLocalCenter = center;
            model->componentSphereCenter = center;
            model->componentSphereRadius = halfExtents.norm();
        } else if(model->componentSphereRadius >= 0.0){
            // A mesh-less primitive component model whose bounding sphere
            // has been analytically set at the extraction
            center = model->componentSphereLocalCenter;
            halfExtents = Vector3::Constant(model->componentSphereRadius);
        } else {
            mainModel->boundingSphereRadius = -1.0;
            return;
        }
        if(!initialized){
            lower = center - halfExtents;
            upper = center + halfExtents;
            initialized = true;
        } else {
            lower = lower.cwiseMin(center - halfExtents);
            upper = upper.cwiseMax(center + halfExtents);
        }
    }
    if(!initialized){
        mainModel->boundingSphereRadius = -1.0;
        return;
    }
    mainModel->boundingSphereLocalCenter = 0.5 * (lower + upper);
    mainModel->boundingSphereCenter = mainModel->boundingSphereLocalCenter;
    mainModel->boundingSphereRadius = (0.5 * (upper - lower)).norm();
}


bool copyCollisionPairCollisions(ColdetModelPairEx* srcPair, CollisionPair& destPair, bool doReserve = false)
{
    vector<Collision>& collisions = destPair.collisions();

    if(collisions.empty()){
        for(int i=0; i < 2; ++i){
            auto model = srcPair->model(i);
            destPair.object(i) = model->object;
            destPair.geometry(i) = getHandle(model);
        }
    }

    const std::vector<collision_data>& cdata = srcPair->collisions();
    const int n = cdata.size();

    if(doReserve){
        collisions.reserve(n);
    }

    for(int j=0; j < n; ++j){
        const collision_data& cd = cdata[j];
        for(int k=0; k < cd.num_of_i_points; ++k){
            if(cd.i_point_new[k]){
                Collision& collision = destPair.newCollision();
                collision.point = cd.i_points[k];
                collision.normal = cd.n_vector;
                collision.depth = cd.depth;
                collision.id1 = cd.id1;
                collision.id2 = cd.id2;
            }
        }
    }

    return !collisions.empty();
}

}

namespace cnoid {

class AISTCollisionDetector::Impl
{
public:
    vector<ColdetModelExPtr> models;
    vector<ColdetModelPairExPtr> modelPairs;
    int maxNumThreads;
    set<IdPair<GeometryHandle>> ignoredPairs;
    set<IdPair<int>> ignoredGroupPairs;
    MeshExtractor* meshExtractor;
    bool isReady;
    bool isDynamicGeometryPairChangeEnabled;
    bool isPrimitiveCollisionDetectionEnabled;
    std::shared_ptr<PrimitiveCollisionParameterSet> primitiveCollisionParameterSet;
    CollisionPair collisionPair;

    Impl();
    Impl(const AISTCollisionDetector::Impl& org);
    ~Impl();
    void initialize();
    std::optional<GeometryHandle> addGeometry(SgNode* geometry);
    std::optional<GeometryHandle> addGeometryWithPrimitiveExtraction(SgNode* geometry);
    void addMesh(ColdetModelEx* model);
    void addMeshOrPrimitive(
        ColdetModelEx* meshModel, ColdetModelEx* queryMeshModel,
        vector<ColdetModelExPtr>& primitiveModels);
    void makeReady();
    bool checkIfGroupPairEnabled(int groupId1, int groupId2);
    bool checkIfModelPairEnabled(ColdetModelPairEx* modelPair);
    bool detectCollisions(GeometryHandle geometry, const std::function<bool(const CollisionPair&)>& callback);
    bool detectCollisions(const std::function<bool(const CollisionPair&)>& callback);
    bool detectCollisionsInParallel(const std::function<bool(const CollisionPair&)>& callback);

    // for multithread version
    int numThreads;
    unique_ptr<ThreadPool> threadPool;
    // One result slot per model pair (indexed by the pair index), so that the
    // final collision order is the deterministic pair-index order regardless of
    // which thread processed which pair. This keeps the simulation reproducible
    // and the warm-start indices stable. Empty slots (pairs not in collision)
    // are skipped when dispatching.
    vector<CollisionPair> collisionPairSlots;

    std::atomic<int> nextPairIndex; // shared cursor for dynamic scheduling

    void extractCollisionsOfAssignedPairs();
    bool dispatchCollisionsInCollisionPairArrays(std::function<bool(const CollisionPair&)> callback);
};

}


AISTCollisionDetector::AISTCollisionDetector()
{
    impl = new Impl;
}


AISTCollisionDetector::Impl::Impl()
{
    isDynamicGeometryPairChangeEnabled = false;
    isPrimitiveCollisionDetectionEnabled = true;
    primitiveCollisionParameterSet = std::make_shared<PrimitiveCollisionParameterSet>();
    maxNumThreads = 0;

    initialize();
}


AISTCollisionDetector::AISTCollisionDetector(const AISTCollisionDetector& org)
{
    impl = new Impl(*org.impl);
}


AISTCollisionDetector::Impl::Impl(const AISTCollisionDetector::Impl& org)
{
    isDynamicGeometryPairChangeEnabled = org.isDynamicGeometryPairChangeEnabled;
    isPrimitiveCollisionDetectionEnabled = org.isPrimitiveCollisionDetectionEnabled;
    primitiveCollisionParameterSet =
        std::make_shared<PrimitiveCollisionParameterSet>(*org.primitiveCollisionParameterSet);
    maxNumThreads = org.maxNumThreads;

    initialize();
}


void AISTCollisionDetector::Impl::initialize()
{
    isReady = false;
    numThreads = 0;
    meshExtractor = new MeshExtractor;
}    


AISTCollisionDetector::Impl::~Impl()
{
    delete meshExtractor;

}


AISTCollisionDetector::~AISTCollisionDetector()
{
    delete impl;
}


const char* AISTCollisionDetector::name() const
{
    return "AISTCollisionDetector";
}


CollisionDetector* AISTCollisionDetector::clone() const
{
    // The copy constructor copies the configuration parameters
    // and does not copy the geometries
    return new AISTCollisionDetector(*this);
}


void AISTCollisionDetector::setNumThreads(int n)
{
    impl->maxNumThreads = n;
}


int AISTCollisionDetector::numThreads() const
{
    return impl->maxNumThreads;
}


/**
   Enable the analytic collision detection based on the primitive shape
   information. This function must be called before adding geometries;
   the geometries added while the mode is disabled are always processed
   as triangle meshes.
*/
void AISTCollisionDetector::setPrimitiveCollisionDetectionEnabled(bool on)
{
    impl->isPrimitiveCollisionDetectionEnabled = on;
}


bool AISTCollisionDetector::isPrimitiveCollisionDetectionEnabled() const
{
    return impl->isPrimitiveCollisionDetectionEnabled;
}


void AISTCollisionDetector::setContactPersistenceTolerance(double tolerance)
{
    impl->primitiveCollisionParameterSet->contactPersistenceTolerance = tolerance;
}


double AISTCollisionDetector::contactPersistenceTolerance() const
{
    return impl->primitiveCollisionParameterSet->contactPersistenceTolerance;
}


void AISTCollisionDetector::setNumCapCircleVertices(int n)
{
    impl->primitiveCollisionParameterSet->numCapCircleVertices =
        std::max(PrimitiveCollisionParameterSet::MinNumCapCircleVertices,
                 std::min(PrimitiveCollisionParameterSet::MaxNumCapCircleVertices, n));
}


int AISTCollisionDetector::numCapCircleVertices() const
{
    return impl->primitiveCollisionParameterSet->numCapCircleVertices;
}


void AISTCollisionDetector::putProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Primitive shape collision detection"), impl->isPrimitiveCollisionDetectionEnabled,
                [this](bool on){ setPrimitiveCollisionDetectionEnabled(on); return true; });
    putProperty.reset().min(0.0).decimals(5);
    putProperty(_("Contact persistence tolerance"),
                impl->primitiveCollisionParameterSet->contactPersistenceTolerance,
                [this](double value){ setContactPersistenceTolerance(value); return true; });
    putProperty.reset().range(
        PrimitiveCollisionParameterSet::MinNumCapCircleVertices,
        PrimitiveCollisionParameterSet::MaxNumCapCircleVertices);
    putProperty(_("Cap circle divisions"),
                impl->primitiveCollisionParameterSet->numCapCircleVertices,
                [this](int n){ setNumCapCircleVertices(n); return true; });
    putProperty.reset().min(0);
    putProperty(_("Collision detection threads"), impl->maxNumThreads,
                [this](int n){ setNumThreads(n); return true; });
}


bool AISTCollisionDetector::store(Mapping* archive)
{
    archive->write("primitive_shape_collision_detection", impl->isPrimitiveCollisionDetectionEnabled);
    archive->write("contact_persistence_tolerance",
                   impl->primitiveCollisionParameterSet->contactPersistenceTolerance);
    archive->write("num_cap_circle_vertices",
                   impl->primitiveCollisionParameterSet->numCapCircleVertices);

    // The number of threads is a performance parameter which does not
    // affect the simulation results, so the default value is not stored
    // to keep the project file concise
    if(impl->maxNumThreads > 0){
        archive->write("num_threads", impl->maxNumThreads);
    }
    return true;
}


bool AISTCollisionDetector::restore(const Mapping* archive)
{
    archive->read("primitive_shape_collision_detection", impl->isPrimitiveCollisionDetectionEnabled);

    auto& params = *impl->primitiveCollisionParameterSet;
    archive->read("contact_persistence_tolerance", params.contactPersistenceTolerance);
    int n;
    if(archive->read("num_cap_circle_vertices", n)){
        setNumCapCircleVertices(n);
    }
    if(archive->read("num_threads", n)){
        setNumThreads(n);
    }
    return true;
}



void AISTCollisionDetector::clearGeometries()
{
    impl->models.clear();
    impl->modelPairs.clear();
    impl->ignoredPairs.clear();
    impl->ignoredGroupPairs.clear();
    impl->isReady = false;
}


int AISTCollisionDetector::numGeometries() const
{
    return impl->models.size();
}


std::optional<GeometryHandle> AISTCollisionDetector::addGeometry(SgNode* geometry)
{
    return impl->addGeometry(geometry);
}


std::optional<GeometryHandle> AISTCollisionDetector::Impl::addGeometry(SgNode* geometry)
{
    if(geometry){
        if(isPrimitiveCollisionDetectionEnabled){
            return addGeometryWithPrimitiveExtraction(geometry);
        }
        ColdetModelExPtr model = new ColdetModelEx;
        if(meshExtractor->extract(geometry, [&]() { addMesh(model); })){
            model->setName(geometry->name());
            model->build();
            if(model->isValid()){
                computeGeometryBoundingSphere(model);
                models.push_back(model);
                isReady = false;
                return getHandle(model);
            }
        }
    }
    return std::nullopt;
}


/**
   Add a geometry with extracting the primitive shape information.
   The shape components which are not primitives are merged into a single
   mesh model like the normal addGeometry function, and each primitive
   shape component is registered as an individual model with its primitive
   information. The models are linked by the sibling chain and share the
   same geometry handle.
   Note that every model including a primitive one also holds its own
   triangle mesh so that the detection can always fall back to the
   mesh-based algorithms.
*/
std::optional<GeometryHandle> AISTCollisionDetector::Impl::addGeometryWithPrimitiveExtraction(SgNode* geometry)
{
    ColdetModelExPtr meshModel = new ColdetModelEx;
    ColdetModelExPtr queryMeshModel = new ColdetModelEx;
    vector<ColdetModelExPtr> primitiveModels;

    if(!meshExtractor->extract(
           geometry,
           [&]() { addMeshOrPrimitive(meshModel, queryMeshModel, primitiveModels); })){
        return std::nullopt;
    }

    ColdetModelExPtr mainModel;
    ColdetModelEx* lastModel = nullptr;
    auto appendModel = [&](ColdetModelExPtr& model){
        model->setName(geometry->name());
        if(!mainModel){
            mainModel = model;
        } else {
            lastModel->sibling = model;
        }
        lastModel = model;
    };

    if(meshModel->getNumVertices() > 0){
        meshModel->build();
        if(meshModel->isValid()){
            appendModel(meshModel);
        }
    }

    // The primitive component models do not hold meshes; their triangles
    // are merged into the query-only mesh model instead
    for(auto& model : primitiveModels){
        appendModel(model);
    }
    if(!primitiveModels.empty() && queryMeshModel->getNumVertices() > 0){
        queryMeshModel->build();
        if(queryMeshModel->isValid()){
            queryMeshModel->isQueryOnlyModel = true;
            appendModel(queryMeshModel);
        }
    }

    if(mainModel){
        computeGeometryBoundingSphere(mainModel);
        models.push_back(mainModel);
        isReady = false;
        return getHandle(mainModel);
    }
    return std::nullopt;
}


void AISTCollisionDetector::Impl::addMesh(ColdetModelEx* model)
{
    SgMesh* mesh = meshExtractor->currentMesh();
    const Affine3& T = meshExtractor->currentTransform();
    
    const int vertexIndexTop = model->getNumVertices();
    
    const SgVertexArray& vertices = *mesh->vertices();
    const int numVertices = vertices.size();
    for(int i=0; i < numVertices; ++i){
        const Vector3 v = T * vertices[i].cast<Affine3::Scalar>();
        model->addVertex(v.x(), v.y(), v.z());
    }

    const int numTriangles = mesh->numTriangles();
    for(int i=0; i < numTriangles; ++i){
        SgMesh::TriangleRef tri = mesh->triangle(i);
        const int v0 = vertexIndexTop + tri[0];
        const int v1 = vertexIndexTop + tri[1];
        const int v2 = vertexIndexTop + tri[2];
        model->addTriangle(v0, v1, v2);
    }
}


/**
   Check if the current mesh of the mesh extractor is a supported primitive
   shape, and register it as an individual primitive model if so. Otherwise
   the mesh is merged into the shared mesh model.
   A primitive model holds only the primitive parameters and does not hold
   the mesh; the tessellated triangles are merged into the query-only mesh
   model which covers the distance and ray casting queries.
*/
void AISTCollisionDetector::Impl::addMeshOrPrimitive
(ColdetModelEx* meshModel, ColdetModelEx* queryMeshModel, vector<ColdetModelExPtr>& primitiveModels)
{
    SgMesh* mesh = meshExtractor->currentMesh();

    int primitiveType = ColdetModel::SP_MESH;
    switch(mesh->primitiveType()){
    case SgMesh::BoxType:      primitiveType = ColdetModel::SP_BOX;      break;
    case SgMesh::SphereType:   primitiveType = ColdetModel::SP_SPHERE;   break;
    case SgMesh::CylinderType: primitiveType = ColdetModel::SP_CYLINDER; break;
    case SgMesh::ConeType:     primitiveType = ColdetModel::SP_CONE;     break;
    case SgMesh::CapsuleType:  primitiveType = ColdetModel::SP_CAPSULE;  break;
    default: break;
    }

    bool doAddPrimitive = false;
    Vector3 scale = Vector3::Ones();
    Vector3 translationByScaling = Vector3::Zero();

    if(primitiveType != ColdetModel::SP_MESH){
        if(!meshExtractor->isCurrentScaled()){
            doAddPrimitive = true;
        } else {
            Affine3 S =
                meshExtractor->currentTransformWithoutScaling().inverse() *
                meshExtractor->currentTransform();
            if(S.linear().isDiagonal()){
                translationByScaling = S.translation();
                scale = S.linear().diagonal();
                auto isEqual = [](double a, double b){ return fabs(a - b) <= 1.0e-9 * fabs(a); };
                switch(primitiveType){
                case ColdetModel::SP_BOX:
                    doAddPrimitive = true;
                    break;
                case ColdetModel::SP_SPHERE:
                case ColdetModel::SP_CAPSULE:
                    // The shape with the hemisphere parts must be uniformly scaled
                    doAddPrimitive = isEqual(scale.x(), scale.y()) && isEqual(scale.x(), scale.z());
                    break;
                case ColdetModel::SP_CYLINDER:
                case ColdetModel::SP_CONE:
                    // The circle cross sections must be kept circular
                    doAddPrimitive = isEqual(scale.x(), scale.z());
                    break;
                default:
                    break;
                }
            }
        }
    }

    if(!doAddPrimitive){
        addMesh(meshModel);
        return;
    }

    // The tessellated triangles of the primitive are merged into the
    // query-only mesh model
    addMesh(queryMeshModel);

    ColdetModelExPtr model = new ColdetModelEx;
    model->setPrimitiveType(static_cast<ColdetModel::PrimitiveType>(primitiveType));

    double boundingRadius = 0.0;

    switch(primitiveType){
    case ColdetModel::SP_BOX: {
        const Vector3& s = mesh->primitive<SgMesh::Box>().size;
        const Vector3 halfExtents = 0.5 * Vector3(s.x() * scale.x(), s.y() * scale.y(), s.z() * scale.z());
        model->setNumPrimitiveParams(3);
        model->setPrimitiveParam(0, halfExtents.x());
        model->setPrimitiveParam(1, halfExtents.y());
        model->setPrimitiveParam(2, halfExtents.z());
        boundingRadius = halfExtents.norm();
        break;
    }
    case ColdetModel::SP_SPHERE: {
        const auto& sphere = mesh->primitive<SgMesh::Sphere>();
        const double radius = sphere.radius * scale.x();
        model->setNumPrimitiveParams(1);
        model->setPrimitiveParam(0, radius);
        boundingRadius = radius;
        break;
    }
    case ColdetModel::SP_CYLINDER: {
        const auto& cylinder = mesh->primitive<SgMesh::Cylinder>();
        const double radius = cylinder.radius * scale.x();
        const double height = cylinder.height * scale.y();
        model->setNumPrimitiveParams(2);
        model->setPrimitiveParam(0, radius);
        model->setPrimitiveParam(1, height);
        boundingRadius = sqrt(radius * radius + height * height / 4.0);
        break;
    }
    case ColdetModel::SP_CONE: {
        const auto& cone = mesh->primitive<SgMesh::Cone>();
        const double radius = cone.radius * scale.x();
        const double height = cone.height * scale.y();
        model->setNumPrimitiveParams(2);
        model->setPrimitiveParam(0, radius);
        model->setPrimitiveParam(1, height);
        boundingRadius = sqrt(radius * radius + height * height / 4.0);
        break;
    }
    case ColdetModel::SP_CAPSULE: {
        const auto& capsule = mesh->primitive<SgMesh::Capsule>();
        const double radius = capsule.radius * scale.x();
        const double height = capsule.height * scale.y();
        model->setNumPrimitiveParams(2);
        model->setPrimitiveParam(0, radius);
        model->setPrimitiveParam(1, height);
        boundingRadius = height / 2.0 + radius;
        break;
    }
    default:
        break;
    }

    Isometry3 T = meshExtractor->currentTransformWithoutScaling();
    T.translation() += T.linear() * translationByScaling;
    model->setPrimitiveLocalPosition(T);

    // The bounding sphere of a mesh-less primitive model is analytically
    // determined here instead of being derived from the mesh BVH
    model->componentSphereLocalCenter = T.translation();
    model->componentSphereCenter = T.translation();
    model->componentSphereRadius = boundingRadius;

    primitiveModels.push_back(model);
}


/**
   The following attribute setting functions apply the given value to all the
   component models of a composite geometry so that the sibling models always
   share the same attributes as the main model.
*/
void AISTCollisionDetector::setCustomObject(GeometryHandle geometry, Referenced* object)
{
    for(auto model = getColdetModel(geometry); model; model = model->sibling){
        model->object = object;
    }
}


void AISTCollisionDetector::setGeometryStatic(GeometryHandle geometry, bool isStatic)
{
    for(auto model = getColdetModel(geometry); model; model = model->sibling){
        model->isStatic = isStatic;
    }
    impl->isReady = false;
}


void AISTCollisionDetector::setGroup(GeometryHandle geometry, int groupId)
{
    for(auto model = getColdetModel(geometry); model; model = model->sibling){
        model->groupId = groupId;
    }
}


void AISTCollisionDetector::setGroupPairEnabled(int groupId1, int groupId2, bool on)
{
    if(on){
        impl->ignoredGroupPairs.erase(IdPair<int>(groupId1, groupId2));
    } else {
        impl->ignoredGroupPairs.insert(IdPair<int>(groupId1, groupId2));
    }
}


void AISTCollisionDetector::ignoreGeometryPair(GeometryHandle geometry1, GeometryHandle geometry2, bool ignore)
{
    IdPair<GeometryHandle> idPair(geometry1, geometry2);
    if(ignore){
        auto result = impl->ignoredPairs.insert(idPair);
        if(result.second){
            impl->isReady = false;
        }
    } else {
        auto p = impl->ignoredPairs.find(idPair);
        if(p != impl->ignoredPairs.end()){
            impl->ignoredPairs.erase(p);
            impl->isReady = false;
        }
    }
}


void AISTCollisionDetector::setGeometryEnabled(GeometryHandle geometry, bool isEnabled)
{
    for(auto model = getColdetModel(geometry); model; model = model->sibling){
        model->isEnabled = isEnabled;
    }
}


void AISTCollisionDetector::setDynamicGeometryPairChangeEnabled(bool on)
{
    if(on != impl->isDynamicGeometryPairChangeEnabled){
        impl->isDynamicGeometryPairChangeEnabled = on;
        impl->isReady = false;
    }
}


bool AISTCollisionDetector::isDynamicGeometryPairChangeEnabled() const
{
    return impl->isDynamicGeometryPairChangeEnabled;
}


bool AISTCollisionDetector::removeGeometry(GeometryHandle geometry)
{
    bool removed = false;
    ColdetModelExPtr model = getColdetModel(geometry);
    
    auto mi = impl->models.rbegin();
    while(mi != impl->models.rend()){
        if(*mi == model){
            impl->models.erase(--(mi.base()));
            removed = true;
            break;
        }
        ++mi;
    }

    if(removed){
        auto pi = impl->modelPairs.begin();
        while(pi != impl->modelPairs.end()){
            auto& modelPair = *pi;
            if(modelPair->model(0) == model || modelPair->model(1) == model){
                pi = impl->modelPairs.erase(pi);
            } else {
                ++pi;
            }
        }
        auto ii = impl->ignoredPairs.begin();
        while(ii != impl->ignoredPairs.end()){
            auto& idPair = *ii;
            if(idPair.hasId(geometry)){
                ii = impl->ignoredPairs.erase(ii);
            } else {
                ++ii;
            }
        }
    }

    return removed;
}


bool AISTCollisionDetector::isGeometryRemovalSupported() const
{
    return true;
}


bool AISTCollisionDetector::makeReady()
{
    impl->makeReady();
    return true;
}


void AISTCollisionDetector::Impl::makeReady()
{
    modelPairs.clear();
    const int n = models.size();
    for(int i=0; i < n; ++i){
        ColdetModelEx* model0 = models[i];
        for(int j = i + 1; j < n; ++j){
            ColdetModelEx* model1 = models[j];
            if(!model0->isStatic || !model1->isStatic){
                bool doRegisterPair = isDynamicGeometryPairChangeEnabled;
                if(!doRegisterPair){
                    if(checkIfGroupPairEnabled(model0->groupId, model1->groupId)){
                        IdPair<GeometryHandle> handlePair(getHandle(model0), getHandle(model1));
                        if(ignoredPairs.find(handlePair) == ignoredPairs.end()){
                            doRegisterPair = true;
                        }
                    }
                }
                if(doRegisterPair){
                    modelPairs.push_back(
                        new ColdetModelPairEx(model0, model1, primitiveCollisionParameterSet));
                }
            }
        }
    }

    const int numPairs = modelPairs.size();

    if(maxNumThreads <= 0){
        numThreads = 0;
        threadPool.reset();
        collisionPairSlots.clear();
    } else {
        numThreads = (maxNumThreads > numPairs) ? numPairs : maxNumThreads;
        threadPool.reset(new ThreadPool(numThreads));
        collisionPairSlots.resize(numPairs);
    }

    isReady = true;
}


bool AISTCollisionDetector::Impl::checkIfGroupPairEnabled(int groupId1, int groupId2)
{
    return (ignoredGroupPairs.find(IdPair<>(groupId1, groupId2)) == ignoredGroupPairs.end());
}


/**
   This is only used when the dynamic geometry pair change is enabled.
*/
bool AISTCollisionDetector::Impl::checkIfModelPairEnabled(ColdetModelPairEx* modelPair)
{
    auto model0 = modelPair->model(0);
    auto model1 = modelPair->model(1);
    if(checkIfGroupPairEnabled(model0->groupId, model1->groupId)){
        IdPair<GeometryHandle> handlePair(getHandle(model0), getHandle(model1));
        if(ignoredPairs.find(handlePair) == ignoredPairs.end()){
            return true;
        }
    }
    return false;
}


void AISTCollisionDetector::updatePosition(GeometryHandle geometry, const Isometry3& position)
{
    auto model = getColdetModel(geometry);
    model->boundingSphereCenter = position * model->boundingSphereLocalCenter;
    do {
        model->setPosition(position);
        model->componentSphereCenter = position * model->componentSphereLocalCenter;
        model = model->sibling;
    } while(model);
}


void AISTCollisionDetector::updatePositions
(std::function<void(Referenced* object, Isometry3*& out_Position)> positionQuery)
{
    for(ColdetModelEx* model : impl->models){ // Do not use auto&
        // The position query is done only once per geometry here because all
        // the component models of a composite geometry share the same position
        Isometry3* T;
        positionQuery(model->object, T);
        model->boundingSphereCenter = (*T) * model->boundingSphereLocalCenter;
        do {
            model->setPosition(*T);
            model->componentSphereCenter = (*T) * model->componentSphereLocalCenter;
            model = model->sibling; // Elements in models are overridden here if auto& is used
        } while(model);
    }
}


bool AISTCollisionDetector::detectCollisions(GeometryHandle geometry, std::function<bool(const CollisionPair&)> callback)
{
    if(!impl->isReady){
        impl->makeReady();
    }
    return impl->detectCollisions(geometry, callback);
}


bool AISTCollisionDetector::Impl::detectCollisions
(GeometryHandle geometry, const std::function<bool(const CollisionPair&)>& callback)
{
    auto& collisions = collisionPair.collisions();

    for(ColdetModelPairEx* modelPair : modelPairs){ // Do not use auto&
        if(!testGeometryBoundingSphereOverlap(modelPair->model(0), modelPair->model(1))){
            continue; // Reject the distant pair without iterating the sub pairs
        }
        collisions.clear();
        do {
            auto model0 = modelPair->model(0);
            auto model1 = modelPair->model(1);
            if(getHandle(modelPair->model(0)) == geometry || getHandle(modelPair->model(1)) == geometry){
                if(model0->isEnabled && model1->isEnabled){
                    if(testComponentBoundingSphereOverlap(model0, model1)){
                        if(!isDynamicGeometryPairChangeEnabled || checkIfModelPairEnabled(modelPair)){
                            if(!modelPair->detectCollisions().empty()){
                                copyCollisionPairCollisions(modelPair, collisionPair);
                            }
                        }
                    }
                }
            }
            modelPair = modelPair->sibling;  // Elements in models are overridden here if auto& is used
        } while(modelPair);

        if(!collisions.empty()){
            if(callback(collisionPair)){
                return true; // Early termination requested
            }
        }
    }
    return false; // All pairs checked
}


bool AISTCollisionDetector::detectCollisions(std::function<bool(const CollisionPair&)> callback)
{
    if(!impl->isReady){
        impl->makeReady();
    }
    if(impl->numThreads > 0){
        return impl->detectCollisionsInParallel(callback);
    } else {
        return impl->detectCollisions(callback);
    }
} 


/**
   \todo Remeber which geometry positions are updated after the last collision detection
   and do the actual collision detection only for the updated geometry pairs.
*/
bool AISTCollisionDetector::Impl::detectCollisions(const std::function<bool(const CollisionPair&)>& callback)
{
    auto& collisions = collisionPair.collisions();

    for(ColdetModelPairEx* modelPair : modelPairs){ // Do not use auto&
        if(!testGeometryBoundingSphereOverlap(modelPair->model(0), modelPair->model(1))){
            continue; // Reject the distant pair without iterating the sub pairs
        }
        collisions.clear();
        do {
            if(modelPair->model(0)->isEnabled && modelPair->model(1)->isEnabled){
                if(testComponentBoundingSphereOverlap(modelPair->model(0), modelPair->model(1))){
                    if(!isDynamicGeometryPairChangeEnabled || checkIfModelPairEnabled(modelPair)){
                        if(!modelPair->detectCollisions().empty()){
                            copyCollisionPairCollisions(modelPair, collisionPair);
                        }
                    }
                }
            }
            modelPair = modelPair->sibling;  // Elements in models are overridden here if auto& is used
        } while(modelPair);

        if(!collisions.empty()){
            if(callback(collisionPair)){
                return true; // Early termination requested
            }
        }
    }
    return false; // All pairs checked
}


/**
 * @note In parallel execution mode, early termination only stops result processing,
 *       not collision detection computation itself. All collision pairs are always
 *       computed before callbacks are invoked. This is an experimental feature.
 */
bool AISTCollisionDetector::Impl::detectCollisionsInParallel(const std::function<bool(const CollisionPair&)>& callback)
{
    // Dynamic scheduling: each worker repeatedly pulls the next pair index
    // atomically, so an uneven per-pair cost (some pairs in contact, others far
    // apart) does not stall a single statically-assigned thread. Each pair
    // writes its result to its own slot (indexed by the pair index), so threads
    // never write to the same slot and no locking is needed.
    nextPairIndex.store(0);
    for(int i=0; i < numThreads; ++i){
        threadPool->post([this](){
            extractCollisionsOfAssignedPairs();
        });
    }
    threadPool->wait();

    return dispatchCollisionsInCollisionPairArrays(callback);
}


void AISTCollisionDetector::Impl::extractCollisionsOfAssignedPairs()
{
    const int numPairs = modelPairs.size();

    while(true){
        const int i = nextPairIndex.fetch_add(1);
        if(i >= numPairs){
            break;
        }
        ColdetModelPairEx* modelPair = modelPairs[i];

        CollisionPair& cpair = collisionPairSlots[i];
        cpair.clearCollisions();
        if(!testGeometryBoundingSphereOverlap(modelPair->model(0), modelPair->model(1))){
            continue; // Reject the distant pair without iterating the sub pairs
        }
        do {
            if(modelPair->model(0)->isEnabled && modelPair->model(1)->isEnabled){
                if(testComponentBoundingSphereOverlap(modelPair->model(0), modelPair->model(1))){
                    if(!isDynamicGeometryPairChangeEnabled || checkIfModelPairEnabled(modelPair)){
                        if(!modelPair->detectCollisions().empty()){
                            copyCollisionPairCollisions(modelPair, cpair, true);
                        }
                    }
                }
            }
            modelPair = modelPair->sibling;
        } while(modelPair);
    }
}


bool AISTCollisionDetector::Impl::dispatchCollisionsInCollisionPairArrays
(std::function<bool(const CollisionPair&)> callback)
{
    // Dispatch in pair-index order, skipping pairs that are not in collision.
    const int numPairs = modelPairs.size();
    for(int i=0; i < numPairs; ++i){
        const CollisionPair& cpair = collisionPairSlots[i];
        if(cpair.empty()){
            continue;
        }
        if(callback(cpair)){
            return true; // Early termination requested
        }
    }
    return false; // All pairs checked
}


double AISTCollisionDetector::detectDistance
(GeometryHandle geometry1, GeometryHandle geometry2, Vector3& out_point1, Vector3& out_point2)
{
    ColdetModelEx* model1 = getColdetModel(geometry1);
    ColdetModelEx* model2 = getColdetModel(geometry2);

    if(!model1->sibling && !model2->sibling){
        return ColdetModelPair::computeDistance(
            model1, model2, out_point1.data(), out_point2.data());
    }

    // Take the minimum distance over all the component combinations of
    // the composite geometries
    double minDistance = -1.0;
    for(ColdetModelEx* m1 = model1; m1; m1 = m1->sibling){
        for(ColdetModelEx* m2 = model2; m2; m2 = m2->sibling){
            Vector3 p1, p2;
            const double d = ColdetModelPair::computeDistance(m1, m2, p1.data(), p2.data());
            if(d >= 0.0 && (minDistance < 0.0 || d < minDistance)){
                minDistance = d;
                out_point1 = p1;
                out_point2 = p2;
            }
        }
    }
    return minDistance;
}


std::optional<double> AISTCollisionDetector::detectDistanceToRayIntersection
(GeometryHandle geometry, const Vector3& point, const Vector3& direction)
{
    std::optional<double> minDistance;
    for(ColdetModelEx* model = getColdetModel(geometry); model; model = model->sibling){
        if(!model->isValid()){
            continue; // Skip the mesh-less primitive component models
        }
        auto distance = model->computeDistanceWithRay(point.data(), direction.data());
        if(distance && (!minDistance || *distance < *minDistance)){
            minDistance = distance;
        }
    }
    return minDistance;
}
