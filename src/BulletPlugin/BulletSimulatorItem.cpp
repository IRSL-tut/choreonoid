#include "BulletSimulatorItem.h"
#include "BulletSimulatorItemImpl.h"
#include "BulletContinuousTrackSimulator.h"
#include <cnoid/ItemManager>
#include <cnoid/WorldItem>
#include <cnoid/BodyItem>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/Material>
#include <cnoid/MaterialTable>
#include <cnoid/BasicSensorSimulationHelper>
#include <cnoid/BodyCollisionLinkFilter>
#include <cnoid/ForceSensor>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshExtractor>
#include <cnoid/Selection>
#include <cnoid/IdPair>
#include <cnoid/EigenUtil>
#include <Eigen/Eigenvalues>
#include <cnoid/EigenArchive>
#include <cnoid/CloneMap>
#include <cnoid/Archive>
#include <cnoid/Format>
#include <cnoid/MessageOut>
#include <cnoid/MessageView>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h>
#include <BulletCollision/CollisionDispatch/btManifoldResult.h>
#include <BulletCollision/CollisionDispatch/btInternalEdgeUtility.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointMotor.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointFeedback.h>
#include <BulletDynamics/Featherstone/btMultiBodyPoint2Point.h>
#include <BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodySliderConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyGearConstraint.h>
#include <BulletDynamics/MLCPSolvers/btDantzigSolver.h>
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <memory>
#include <optional>
#include <cmath>
#include <algorithm>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

constexpr double DefaultGravityAcceleration = 9.80665;
constexpr int DefaultNumIterations = 50;
constexpr double DefaultPositionGain = 1.0;
constexpr double DefaultVelocityGain = 1.0;
constexpr double DefaultContactErp = 0.2;
constexpr double DefaultCollisionMargin = 0.001;

// Parameters of the virtual rotor links realizing the equivalent rotor
// inertia. The default gear ratio is used when a link does not provide the
// gear_ratio information; a typical robot reduction ratio is used because the
// physical behavior is almost independent of how the equivalent rotor inertia
// is decomposed into the rotor inertia and the gear ratio (see setupRotorLink).
constexpr double DefaultRotorGearRatio = 100.0;
// Lower bound of the rotor inertia for the numerical safety in single
// precision builds of the Bullet library
constexpr double MinRotorInertia = 1.0e-10;
constexpr double RotorGearConstraintMaxImpulse = 1.0e8;

enum SolverTypeIndex { SequentialImpulseSolver, DantzigMLCPSolver };

/*
   The inertia tensor given to btMultiBody must be diagonal, so the link frame
   used on the Bullet side is the principal axes frame located at the center of
   mass. The transformation between the Choreonoid link frame and this inertial
   frame is kept in each BulletLink.
*/
void calcPrincipalInertia(const Matrix3& I, Vector3& moments, Matrix3& R)
{
    Eigen::SelfAdjointEigenSolver<Matrix3> solver(I);
    moments = solver.eigenvalues();
    R = solver.eigenvectors();
    if(R.determinant() < 0.0){
        R.col(2) = -R.col(2);
    }
    for(int i = 0; i < 3; ++i){
        if(moments[i] < 0.0){
            moments[i] = 0.0;
        }
    }
}

string meshLabel(SgShape* shape)
{
    if(shape){
        auto& name = shape->name();
        if(!name.empty()){
            return formatR(_("Mesh \"{0}\""), name);
        }
        if(auto mesh = shape->mesh()){
            auto& meshName = mesh->name();
            if(!meshName.empty()){
                return formatR(_("Mesh \"{0}\""), meshName);
            }
        }
    }
    return _("A mesh");
}

// Choose the control law from the actuation mode bits. The mode is a bitmask
// (e.g. JointDisplacement | JointVelocity means position control with a velocity
// feedforward), so it must be tested by bit, not by exact equality. Position
// takes precedence over velocity over effort.
ControlType controlTypeFromActuationMode(short mode)
{
    if(mode & Link::JointDisplacement){
        return PositionControl;
    } else if(mode & Link::JointVelocity){
        return VelocityControl;
    } else if(mode & Link::JointEffort){
        return EffortControl;
    }
    return NoControl;
}

}

namespace {

/*
   Broadphase filter realizing the per-link-pair self-collision configuration.
   Pairs of links that belong to the same body are tested against the body's
   BodyCollisionLinkFilter, which reflects the self-collision detection setting
   of the body item and the excluded link pairs specified in the body file.
*/
class SelfCollisionFilterCallback : public btOverlapFilterCallback
{
public:
    virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const override
    {
        bool collides =
            (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0 &&
            (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask) != 0;
        if(!collides){
            return false;
        }
        auto object0 = static_cast<btCollisionObject*>(proxy0->m_clientObject);
        auto object1 = static_cast<btCollisionObject*>(proxy1->m_clientObject);
        auto link0 = static_cast<BulletLink*>(object0->getUserPointer());
        auto link1 = static_cast<BulletLink*>(object1->getUserPointer());
        if(link0 && link1 && link0->bulletBody == link1->bulletBody){
            // The internal links of a continuous track (linear segments and
            // grouser-covered wheels) never collide within the same body
            if(link0->isTrackInternal || link1->isTrackInternal){
                return false;
            }
            return link0->bulletBody->bodyCollisionLinkFilter.checkIfEnabledLinkPair(
                link0->link->index(), link1->link->index());
        }
        return true;
    }
};


/*
   Set the surface velocity of pseudo continuous tracks to a contact point.
   The target relative tangential velocity (object0 relative to object1) that
   makes a track surface move at its commanded speed is
   dq_target * normalize(axisWorld x normal). Since m_normalWorldOnB points
   from object1 to object0, the cross product gives the correctly signed
   direction for the track link in either object slot.

   NOTE: This mechanism does NOT currently work because of a limitation of the
   Bullet library. The target friction velocities (m_contactMotion1/2) are
   only supported by the rigid body contact solver. For the contacts of
   multibody link colliders, btMultiBodyConstraintSolver::
   setupMultiBodyContactConstraint receives the values as its desiredVelocity
   parameter but never uses them, and convertMultiBodyContact unconditionally
   recomputes the lateral friction directions with btPlaneSpace1. This has
   been confirmed in the Bullet versions 3.24 (Ubuntu 24.04), 3.25 (the
   latest release), and the current upstream master. Since all the dynamic
   objects of this plugin are simulated as multibody link colliders, pseudo
   continuous track links do not drive the body at all. The values are still
   set here so that the mechanism will take effect if track links are
   alternatively simulated as rigid bodies attached to the multibody with
   btMultiBodyFixedConstraint, which is a possible future workaround.
*/
void applySurfaceVelocityToContact(btManifoldPoint& cp, BulletLink* link0, BulletLink* link1)
{
    const Vector3 normal = toEigenVector3(cp.m_normalWorldOnB);
    Vector3 targetRel = Vector3::Zero();
    BulletLink* links[2] = { link0, link1 };
    for(int k = 0; k < 2; ++k){
        if(!links[k]->useSurfaceVelocity){
            continue;
        }
        Link* link = links[k]->link;
        Vector3 axis = link->R() * link->a();
        Vector3 dir = axis.cross(normal);
        double n = dir.norm();
        if(n > 1.0e-9){
            targetRel += (link->dq_target() / n) * dir;
        }
    }
    // The multibody contact solver of Bullet unconditionally resets the
    // lateral friction directions to the tangent plane basis computed by
    // btPlaneSpace1, so a custom friction direction cannot be used. Instead,
    // the target velocity is decomposed into its components on the same basis
    // and set as the target friction velocities (m_contactMotion1/2), which
    // the solver respects when SOLVER_ENABLE_FRICTION_DIRECTION_CACHING is
    // enabled and the lateral friction initialized flag is set.
    btVector3 dir1, dir2;
    btPlaneSpace1(cp.m_normalWorldOnB, dir1, dir2);
    dir1.normalize();
    dir2.normalize();
    cp.m_lateralFrictionDir1 = dir1;
    cp.m_lateralFrictionDir2 = dir2;
    cp.m_contactPointFlags |= BT_CONTACT_FLAG_LATERAL_FRICTION_INITIALIZED;
    cp.m_contactMotion1 = static_cast<btScalar>(targetRel.dot(toEigenVector3(dir1)));
    cp.m_contactMotion2 = static_cast<btScalar>(targetRel.dot(toEigenVector3(dir2)));
}


/*
   Contact added callback used for three purposes: adjusting the contact normals
   on the internal edges of static triangle meshes, overriding the contact
   parameters of explicitly defined contact material pairs, and injecting the
   surface velocity of pseudo continuous tracks. The callback is only invoked
   for pairs including an object with the CF_CUSTOM_MATERIAL_CALLBACK flag.
   Note that the callback is invoked only when a contact point is newly added
   to a persistent manifold, so the surface velocity of the existing contact
   points must be kept up to date by updateSurfaceVelocityContacts.
*/
bool bulletContactAddedCallback(
    btManifoldPoint& cp,
    const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
    const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1)
{
    BulletLink* links[2] = {
        static_cast<BulletLink*>(colObj0Wrap->getCollisionObject()->getUserPointer()),
        static_cast<BulletLink*>(colObj1Wrap->getCollisionObject()->getUserPointer())
    };
    if(!links[0] || !links[1]){
        return true;
    }
    auto simImpl = links[0]->bulletBody->simImpl;

    if(simImpl->isInternalEdgeSmoothingEnabled &&
       (links[0]->hasTriangleMeshShape || links[1]->hasTriangleMeshShape)){
        // Each call is a no-op unless its first wrapper is a triangle mesh
        btAdjustInternalEdgeContacts(cp, colObj1Wrap, colObj0Wrap, partId1, index1);
        btAdjustInternalEdgeContacts(cp, colObj0Wrap, colObj1Wrap, partId0, index0);
    }

    // Override the contact parameters with those of an explicitly defined
    // contact material pair. Without an explicit pair the contacts keep the
    // values combined from the per-object parameters, which realize the same
    // formulas as the derived pair parameters (sqrt(r1*r2) etc.) through the
    // multiplicative combining of the sqrt-converted single material values.
    if(!simImpl->contactPairParamMap.empty()){
        auto it = simImpl->contactPairParamMap.find(
            IdPair<int>(links[0]->materialId, links[1]->materialId));
        if(it != simImpl->contactPairParamMap.end()){
            const auto& param = it->second;
            cp.m_combinedFriction = static_cast<btScalar>(param.friction);
            cp.m_combinedRestitution = static_cast<btScalar>(param.restitution);
        }
    }

    if(links[0]->useSurfaceVelocity || links[1]->useSurfaceVelocity){
        applySurfaceVelocityToContact(cp, links[0], links[1]);
    }

    return true;
}

}


void BulletSimulatorItem::initialize(ExtensionManager* ext)
{
    ext->itemManager().registerClass<BulletSimulatorItem, SimulatorItem>(N_("BulletSimulatorItem"));
    ext->itemManager().addCreationPanel<BulletSimulatorItem>();
}


BulletSimulatorItem::BulletSimulatorItem()
{
    impl = new Impl(this);
    setAllLinkPositionOutputMode(false);
}


BulletSimulatorItem::Impl::Impl(BulletSimulatorItem* self)
    : self(self)
{
    initialize();

    gravity << 0.0, 0.0, -DefaultGravityAcceleration;

    solverType.setSymbol(SequentialImpulseSolver, N_("Sequential impulse"));
    solverType.setSymbol(DantzigMLCPSolver, N_("MLCP (Dantzig)"));
    solverType.select(SequentialImpulseSolver);

    numIterations = DefaultNumIterations;
    contactErp = DefaultContactErp;
    positionGain = DefaultPositionGain;
    velocityGain = DefaultVelocityGain;
    linearDamping = 0.0;
    angularDamping = 0.0;
    collisionMargin = DefaultCollisionMargin;
    useGImpactForDynamicMeshes = false;
    isInternalEdgeSmoothingEnabled = true;
    isSelfCollisionEnabledByDefault = false;
    isRotorInertiaEnabled = true;
    defaultRotorGearRatio = DefaultRotorGearRatio;
    isVelocityOutputEnabled = false;
    isAccelerationOutputEnabled = false;
    isDriveEffortOutputEnabled = false;
}


BulletSimulatorItem::BulletSimulatorItem(const BulletSimulatorItem& org)
    : SimulatorItem(org)
{
    impl = new Impl(this, *org.impl);
}


BulletSimulatorItem::Impl::Impl(BulletSimulatorItem* self, const Impl& org)
    : self(self)
{
    initialize();

    gravity = org.gravity;
    solverType = org.solverType;
    numIterations = org.numIterations;
    contactErp = org.contactErp;
    positionGain = org.positionGain;
    velocityGain = org.velocityGain;
    linearDamping = org.linearDamping;
    angularDamping = org.angularDamping;
    collisionMargin = org.collisionMargin;
    useGImpactForDynamicMeshes = org.useGImpactForDynamicMeshes;
    isInternalEdgeSmoothingEnabled = org.isInternalEdgeSmoothingEnabled;
    isSelfCollisionEnabledByDefault = org.isSelfCollisionEnabledByDefault;
    isRotorInertiaEnabled = org.isRotorInertiaEnabled;
    defaultRotorGearRatio = org.defaultRotorGearRatio;
    isVelocityOutputEnabled = org.isVelocityOutputEnabled;
    isAccelerationOutputEnabled = org.isAccelerationOutputEnabled;
    isDriveEffortOutputEnabled = org.isDriveEffortOutputEnabled;
}


void BulletSimulatorItem::Impl::initialize()
{
    materialTable = nullptr;
    mout = MessageOut::master();
}


BulletSimulatorItem::~BulletSimulatorItem()
{
    delete impl;
}


BulletSimulatorItem::Impl::~Impl()
{
    clear();
}


void BulletSimulatorItem::Impl::clear()
{
    trackSimulator.reset();

    if(world){
        for(auto& constraint : multiBodyConstraints){
            world->removeMultiBodyConstraint(constraint.get());
        }
        for(auto& constraint : typedConstraints){
            world->removeConstraint(constraint.get());
        }
        for(auto& object : collisionObjects){
            if(object->getBroadphaseHandle()){
                world->removeCollisionObject(object.get());
            }
        }
        for(auto& multiBody : multiBodies){
            world->removeMultiBody(multiBody.get());
        }
    }
    multiBodyConstraints.clear();
    typedConstraints.clear();
    collisionObjects.clear();
    multiBodies.clear();

    world.reset();
    solver.reset();
    mlcpInnerSolver.reset();
    broadphase.reset();
    dispatcher.reset();
    collisionConfiguration.reset();
    overlapFilter.reset();

    convexShapeMap.clear();
    triangleMeshShapeMap.clear();
    gimpactShapeMap.clear();
    ownedShapes.clear();
    triangleInfoMaps.clear();
    meshInterfaces.clear();
    meshBuffers.clear();

    materialParamsCache.clear();
    contactPairParamMap.clear();
    pairMaterialIds.clear();
    bodyMap.clear();
    materialTable = nullptr;
}


void BulletSimulatorItem::setGravity(const Vector3& gravity)
{
    impl->gravity = gravity;
}


const Vector3& BulletSimulatorItem::gravity() const
{
    return impl->gravity;
}


Vector3 BulletSimulatorItem::getGravity() const
{
    return impl->gravity;
}


Item* BulletSimulatorItem::doDuplicate() const
{
    return new BulletSimulatorItem(*this);
}


void BulletSimulatorItem::clearSimulation()
{
    impl->clear();
}


SimulationBody* BulletSimulatorItem::createSimulationBody(Body* orgBody, CloneMap& cloneMap)
{
    return new BulletBody(cloneMap.getClone(orgBody), impl);
}


bool BulletSimulatorItem::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    return impl->initializeSimulation(simBodies);
}


bool BulletSimulatorItem::Impl::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    clear();
    trackSimulator = createBulletContinuousTrackSimulator();

    timeStep = self->worldTimeStep();

    materialTable = nullptr;
    if(WorldItem* worldItem = self->worldItem()){
        materialTable = worldItem->materialTable();
    }
    setupContactMaterials();

    collisionConfiguration = make_unique<btDefaultCollisionConfiguration>();
    dispatcher = make_unique<btCollisionDispatcher>(collisionConfiguration.get());
    btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher.get());
    broadphase = make_unique<btDbvtBroadphase>();

    if(solverType.selectedIndex() == DantzigMLCPSolver){
        mlcpInnerSolver = make_unique<btDantzigSolver>();
        solver = make_unique<btMultiBodyMLCPConstraintSolver>(mlcpInnerSolver.get());
    } else {
        solver = make_unique<btMultiBodyConstraintSolver>();
    }

    world = make_unique<btMultiBodyDynamicsWorld>(
        dispatcher.get(), broadphase.get(), solver.get(), collisionConfiguration.get());
    world->setGravity(toBtVector3(gravity));

    auto& solverInfo = world->getSolverInfo();
    solverInfo.m_numIterations = numIterations;
    solverInfo.m_erp2 = static_cast<btScalar>(contactErp);
    // SOLVER_ENABLE_FRICTION_DIRECTION_CACHING is required to make the solver
    // respect the target friction velocities (m_contactMotion1/2) set for the
    // surface velocity of pseudo continuous tracks. Note that this only works
    // for rigid body contacts; the multibody contact solver ignores the target
    // velocities regardless of this flag. See applySurfaceVelocityToContact.
    solverInfo.m_solverMode |=
        SOLVER_USE_2_FRICTION_DIRECTIONS | SOLVER_ENABLE_FRICTION_DIRECTION_CACHING;
    // Joint feedback values are read as world-frame wrenches about the joint frame
    solverInfo.m_jointFeedbackInWorldSpace = true;
    solverInfo.m_jointFeedbackInJointFrame = true;

    overlapFilter = make_unique<SelfCollisionFilterCallback>();
    world->getPairCache()->setOverlapFilterCallback(overlapFilter.get());

    gContactAddedCallback = bulletContactAddedCallback;

    bool hasNonRootFreeJoints = false;
    bodyMap.clear();
    for(size_t i = 0; i < simBodies.size(); ++i){
        auto bulletBody = static_cast<BulletBody*>(simBodies[i]);
        bulletBody->bodyIndex = static_cast<int>(i);
        bodyMap[bulletBody->body()] = bulletBody;
    }
    hasSurfaceVelocityLinks = false;
    for(size_t i = 0; i < simBodies.size(); ++i){
        auto bulletBody = static_cast<BulletBody*>(simBodies[i]);
        bulletBody->createBulletObjects();
        if(!hasNonRootFreeJoints){
            for(auto& link : bulletBody->body()->links()){
                if(link->isFreeJoint() && !link->isRoot()){
                    hasNonRootFreeJoints = true;
                    break;
                }
            }
        }
        for(auto& bulletLink : bulletBody->bulletLinks){
            if(bulletLink && bulletLink->useSurfaceVelocity){
                hasSurfaceVelocityLinks = true;
                break;
            }
        }
    }

    if(trackSimulator && trackSimulator->empty()){
        trackSimulator.reset();
    }
    addExtraJoints(simBodies);

    if(trackSimulator){
        trackSimulator->initializeTrackStates();
    }

    if(hasNonRootFreeJoints && !self->isAllLinkPositionOutputMode()){
        bool confirmed = showConfirmDialog(
            _("Confirmation of all link position recording mode"),
            formatR(_("{0}: There is a model that has free-type joints other than the root link "
                      "and all the link positions should be recorded in this case. "
                      "Do you enable the mode to do it?"), self->displayName()));
        if(confirmed){
            self->setAllLinkPositionOutputMode(true);
            self->notifyUpdate();
        }
    }

    return true;
}


void BulletSimulatorItem::Impl::setupContactMaterials()
{
    contactPairParamMap.clear();
    pairMaterialIds.clear();
    materialParamsCache.clear();

    if(!materialTable){
        return;
    }

    // Collect the explicitly defined contact material pairs. They are applied
    // by the contact added callback because the default combining of per-object
    // parameters cannot express pair-specific parameters.
    const int maxId = materialTable->maxMaterialId();
    for(int i = 0; i <= maxId; ++i){
        for(int j = i; j <= maxId; ++j){
            if(ContactMaterial* cm = materialTable->contactMaterial(i, j)){
                ContactPairParam param;
                param.friction = cm->friction();
                param.restitution = cm->restitution();
                contactPairParamMap[IdPair<int>(i, j)] = param;
                pairMaterialIds.insert(i);
                pairMaterialIds.insert(j);
            }
        }
    }
}


const BulletSimulatorItem::Impl::MaterialParams&
BulletSimulatorItem::Impl::getOrCreateMaterialParams(int materialId)
{
    auto it = materialParamsCache.find(materialId);
    if(it != materialParamsCache.end()){
        return it->second;
    }

    Material* material = nullptr;
    if(materialTable){
        material = materialTable->material(materialId);
    }

    // Convert to per-object parameters using sqrt for AGX-compatible combining.
    // friction = sqrt(roughness) -> with multiplicative combining: sqrt(r1*r2).
    // restitution = sqrt(1-viscosity) -> sqrt((1-v1)*(1-v2)).
    double roughness = material ? material->roughness() : 0.5;
    // The fallback viscosity must be 1.0 (restitution 0); a smaller value would
    // make every contact of materialless bodies elastic.
    double viscosity = material ? material->viscosity() : 1.0;

    MaterialParams params;
    params.friction = std::sqrt(roughness);
    params.restitution = std::sqrt(std::max(0.0, 1.0 - viscosity));
    params.stiffness = material ? material->stiffness() : 0.0;
    params.damping = material ? material->damping() : 0.0;

    auto inserted = materialParamsCache.emplace(materialId, params);
    return inserted.first->second;
}


void BulletSimulatorItem::Impl::applyLinkMaterial(btCollisionObject* object, BulletLink* bulletLink)
{
    const MaterialParams& params = getOrCreateMaterialParams(bulletLink->materialId);

    object->setFriction(static_cast<btScalar>(params.friction));
    if(params.stiffness > 0.0){
        object->setContactStiffnessAndDamping(
            static_cast<btScalar>(params.stiffness), static_cast<btScalar>(params.damping));
    } else {
        object->setRestitution(static_cast<btScalar>(params.restitution));
    }

    object->setUserPointer(bulletLink);

    bool needsCallback =
        bulletLink->useSurfaceVelocity ||
        pairMaterialIds.count(bulletLink->materialId) > 0 ||
        (isInternalEdgeSmoothingEnabled && bulletLink->hasTriangleMeshShape);
    if(needsCallback){
        object->setCollisionFlags(
            object->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
    }
}


void BulletSimulatorItem::Impl::addMultiBodyConstraint(btMultiBodyConstraint* constraint)
{
    constraint->finalizeMultiDof();
    world->addMultiBodyConstraint(constraint);
    multiBodyConstraints.emplace_back(constraint);
}


void BulletSimulatorItem::Impl::addTypedConstraint(btTypedConstraint* constraint)
{
    world->addConstraint(constraint);
    typedConstraints.emplace_back(constraint);
}


bool BulletSimulatorItem::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    return impl->stepSimulation(activeSimBodies);
}


bool BulletSimulatorItem::Impl::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    for(auto& simBody : activeSimBodies){
        auto bulletBody = static_cast<BulletBody*>(simBody);
        bulletBody->body()->setVirtualJointForces();
        bulletBody->inputControl();
        bulletBody->addExternalForces();
    }

    if(hasSurfaceVelocityLinks){
        updateSurfaceVelocityContacts();
    }

    if(trackSimulator){
        trackSimulator->updateSimulation();
    }

    world->stepSimulation(static_cast<btScalar>(timeStep), 0);

    for(auto& simBody : activeSimBodies){
        auto bulletBody = static_cast<BulletBody*>(simBody);
        bulletBody->getState();
        if(bulletBody->hasForceSensors){
            bulletBody->updateForceSensors();
        }
        if(bulletBody->sensorHelper.hasGyroOrAccelerationSensors()){
            bulletBody->sensorHelper.updateGyroAndAccelerationSensors();
        }
    }

    // Update the track device states after the track link positions.
    if(trackSimulator){
        trackSimulator->updateTrackStates();
    }

    return true;
}


/*
   Update the surface velocity of the contact points of pseudo continuous
   tracks before every simulation step. This cannot be covered by the contact
   added callback alone because the callback is not invoked for the contact
   points persisting over multiple steps while the commanded track velocities
   change every step.

   NOTE: This does not currently make the tracks work. See the note on
   applySurfaceVelocityToContact.
*/
void BulletSimulatorItem::Impl::updateSurfaceVelocityContacts()
{
    const int numManifolds = dispatcher->getNumManifolds();
    for(int i = 0; i < numManifolds; ++i){
        btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(i);
        auto link0 = static_cast<BulletLink*>(manifold->getBody0()->getUserPointer());
        auto link1 = static_cast<BulletLink*>(manifold->getBody1()->getUserPointer());
        if(!link0 || !link1){
            continue;
        }
        if(!link0->useSurfaceVelocity && !link1->useSurfaceVelocity){
            continue;
        }
        const int numContacts = manifold->getNumContacts();
        for(int j = 0; j < numContacts; ++j){
            applySurfaceVelocityToContact(manifold->getContactPoint(j), link0, link1);
        }
    }
}


//---------------------------------------------------------------------------
// Model building
//---------------------------------------------------------------------------

BulletLink::BulletLink(Link* link, BulletBody* bulletBody)
    : bulletBody(bulletBody),
      link(link)
{
    unit = nullptr;
    mbIndex = -2;
    rigidBody = nullptr;
    collider = nullptr;
    compound = nullptr;
    T_inertial.setIdentity();
    T_inertialInv.setIdentity();
    principalInertia.setZero();
    materialId = link->materialId();
    isTrackInternal = false;
    controlType = NoControl;
    velocityFeedforward = false;
    useSurfaceVelocity = (link->jointType() == Link::PseudoContinuousTrackJoint);
    hasTriangleMeshShape = false;
    hasAccelSensor = false;
    kp = 0.0;
    kv = 0.0;
    motor = nullptr;
    lastJointEffort = 0.0;
    rotorMbIndex = -1;
    rotorGearRatio = 1.0;
    prevV.setZero();
    prevW.setZero();
}


void BulletLink::initInertialFrame()
{
    Matrix3 R;
    calcPrincipalInertia(link->I(), principalInertia, R);
    T_inertial.linear() = R;
    T_inertial.translation() = link->c();
    T_inertialInv = T_inertial.inverse();
}


BulletBody::BulletBody(Body* body, BulletSimulatorItem::Impl* simImpl)
    : SimulationBody(body),
      simImpl(simImpl)
{
    bodyIndex = 0;
    selfCollisionDetectionEnabled = false;
    hasForceSensors = false;
    needsSensorVelocities = false;
}


BulletLink* BulletBody::getOrCreateBulletLink(Link* link)
{
    auto& bulletLink = bulletLinks[link->index()];
    if(!bulletLink){
        bulletLink = new BulletLink(link, this);
    }
    return bulletLink;
}


void BulletBody::createBulletObjects()
{
    auto body_ = body();

    selfCollisionDetectionEnabled = simImpl->isSelfCollisionEnabledByDefault;
    if(bodyItem()){
        selfCollisionDetectionEnabled = bodyItem()->isSelfCollisionDetectionEnabled();
    }
    bodyCollisionLinkFilter.setTargetBody(body_, selfCollisionDetectionEnabled);

    bulletLinks.clear();
    bulletLinks.resize(body_->numLinks());
    units.clear();
    rigidBodyLinks.clear();
    controlLinks.clear();
    rotorDefaultGearRatioJointNames.clear();

    createLinkTree(body_->rootLink());

    auto joinNames = [](const std::vector<string>& names){
        string joined;
        for(auto& name : names){
            if(!joined.empty()){
                joined += ", ";
            }
            joined += name;
        }
        return joined;
    };

    if(!simImpl->isRotorInertiaEnabled){
        for(auto& link : body_->links()){
            if(link->Jm2() != 0.0){
                simImpl->mout->putWarningln(
                    formatR(_("{0}: The equivalent rotor inertia is ignored. Set the rotor "
                              "inertia property of {1} to true to simulate it using virtual "
                              "geared rotor links."),
                            body_->name(), simImpl->self->displayName()));
                break;
            }
        }
    } else {
        std::vector<string> unmodeledNames;
        for(auto& link : body_->links()){
            if(link->Jm2() != 0.0){
                auto& bulletLink = bulletLinks[link->index()];
                if(!bulletLink || bulletLink->rotorMbIndex < 0){
                    unmodeledNames.push_back(link->name());
                }
            }
        }
        if(!unmodeledNames.empty()){
            simImpl->mout->putWarningln(
                formatR(_("{0}: The equivalent rotor inertia of the following links is ignored "
                          "because the links are not simulated as revolute or prismatic joints "
                          "of a multibody: {1}"),
                        body_->name(), joinNames(unmodeledNames)));
        }
        if(!rotorDefaultGearRatioJointNames.empty()){
            simImpl->mout->putln(
                formatR(_("{0}: The following joints do not have the gear_ratio information "
                          "and the default gear ratio {1} is used for their rotor models: {2}"),
                        body_->name(), simImpl->defaultRotorGearRatio,
                        joinNames(rotorDefaultGearRatioJointNames)));
        }
    }

    initializeSensors();
}


void BulletBody::createLinkTree(Link* link)
{
    if(link->isStatic()){
        createStaticLink(link);
        for(Link* child = link->child(); child; child = child->sibling()){
            createLinkTree(child);
        }
    } else {
        createUnit(link);
    }
}


void BulletBody::collectUnitLinks(Link* link, std::vector<Link*>& unitLinks, std::vector<Link*>& subUnitRoots)
{
    unitLinks.push_back(link);
    for(Link* child = link->child(); child; child = child->sibling()){
        if(child->isFreeJoint()){
            subUnitRoots.push_back(child);
        } else {
            collectUnitLinks(child, unitLinks, subUnitRoots);
        }
    }
}


void BulletBody::createUnit(Link* unitRoot)
{
    std::vector<Link*> unitLinks;
    std::vector<Link*> subUnitRoots;
    collectUnitLinks(unitRoot, unitLinks, subUnitRoots);

    /*
       A single free-floating link without any joints is simulated as a plain
       btRigidBody instead of a multibody consisting of only a floating base.
       This reduces the per-body overhead, and the contacts between two such
       objects go through the rigid body contact solver, which supports some
       features missing from the multibody contact path (e.g. the target
       friction velocities of the surface velocity mechanism). Massless links
       are excluded because a zero-mass btRigidBody would be treated as a
       static object.
    */
    if(unitLinks.size() == 1 && unitRoot->isFreeJoint() && unitRoot->m() > 0.0){
        createRigidBodyLink(unitRoot);
        for(Link* subUnitRoot : subUnitRoots){
            createUnit(subUnitRoot);
        }
        return;
    }

    BulletUnitPtr unit = new BulletUnit;
    units.push_back(unit);

    // Continuous track devices whose links are all contained in this unit.
    // Each track adds two internal linear segment links to the multibody.
    // They are appended after the Choreonoid links so that the multibody
    // link indices of the Choreonoid links are not affected.
    std::unique_ptr<BulletContinuousTrackUnitSetup> trackSetup;
    if(simImpl->trackSimulator){
        trackSetup = simImpl->trackSimulator->prepareUnit(this, unitLinks);
    }
    const int numSegmentLinks = trackSetup ? trackSetup->numInternalLinks() : 0;

    Link* parent = unitRoot->parent();
    bool needsDummyBase =
        (parent && parent->isStatic() && !unitRoot->isFreeJoint()) ||
        (!parent && (unitRoot->isRevoluteJoint() || unitRoot->isPrismaticJoint()));

    // Virtual rotor links realizing the equivalent rotor inertia are appended
    // after the track segment links so that the multibody link indices of the
    // Choreonoid links are not affected. See setupRotorLink.
    int numRotorLinks = 0;
    if(simImpl->isRotorInertiaEnabled){
        const size_t firstJointLinkIndex = needsDummyBase ? 0 : 1;
        for(size_t i = firstJointLinkIndex; i < unitLinks.size(); ++i){
            Link* link = unitLinks[i];
            if((link->isRevoluteJoint() || link->isPrismaticJoint()) && link->Jm2() > 0.0){
                ++numRotorLinks;
            }
        }
    }

    int numMbLinks;
    Isometry3 T_base;
    btMultiBody* multiBody;

    if(needsDummyBase){
        // The unit hangs off a static link or the world with an actual joint.
        // A massless fixed base without a collider is used as the multibody base,
        // like the dummy articulation root of the PhysX plugin.
        unit->hasDummyBase = true;
        numMbLinks = static_cast<int>(unitLinks.size());
        if(parent){
            T_base = parent->T();
        } else {
            // Place the fixed base at the joint mount frame (the q = 0 pose)
            T_base = unitRoot->T();
            if(unitRoot->isRevoluteJoint()){
                T_base = T_base * AngleAxis(-unitRoot->q(), unitRoot->jointAxis());
            } else if(unitRoot->isPrismaticJoint()){
                T_base = T_base * Translation3(-unitRoot->q() * unitRoot->jointAxis());
            }
        }
        multiBody = new btMultiBody(
            numMbLinks + numSegmentLinks + numRotorLinks, 0.0, btVector3(0, 0, 0), true, false);
        multiBody->setBaseWorldTransform(toBtTransform(T_base));

    } else {
        // The unit root link itself is the (floating) base of the multibody
        numMbLinks = static_cast<int>(unitLinks.size()) - 1;
        BulletLink* baseLink = getOrCreateBulletLink(unitRoot);
        baseLink->unit = unit;
        baseLink->mbIndex = -1;
        baseLink->initInertialFrame();
        unit->baseLink = baseLink;
        multiBody = new btMultiBody(
            numMbLinks + numSegmentLinks + numRotorLinks,
            static_cast<btScalar>(unitRoot->m()),
            toBtVector3(baseLink->principalInertia),
            false, false);
        multiBody->setBaseWorldTransform(toBtTransform(unitRoot->T() * baseLink->T_inertial));
    }

    unit->multiBody = multiBody;
    multiBody->setHasSelfCollision(true); // per-pair filtering is done in the broadphase filter
    multiBody->setLinearDamping(static_cast<btScalar>(simImpl->linearDamping));
    multiBody->setAngularDamping(static_cast<btScalar>(simImpl->angularDamping));
    simImpl->multiBodies.emplace_back(multiBody);

    // Set up the multibody links
    unit->mbLinks.resize(numMbLinks);
    const int unitLinkOffset = unit->hasDummyBase ? 0 : 1;
    int nextRotorMbIndex = numMbLinks + numSegmentLinks;
    for(int i = 0; i < numMbLinks; ++i){
        Link* link = unitLinks[i + unitLinkOffset];
        BulletLink* bulletLink = getOrCreateBulletLink(link);
        bulletLink->unit = unit;
        bulletLink->mbIndex = i;
        bulletLink->initInertialFrame();
        unit->mbLinks[i] = bulletLink;

        // Resolve the parent frame information
        Matrix3 R_iP;
        Vector3 c_P;
        int parentIndex;
        Isometry3 Tb;
        if(link == unitRoot){
            // Connected to the dummy fixed base
            parentIndex = -1;
            R_iP.setIdentity();
            c_P.setZero();
            if(parent){
                Tb = link->Tb();
            } else {
                Tb.setIdentity();
            }
        } else {
            Link* parentLink = link->parent();
            BulletLink* parentBulletLink = bulletLinks[parentLink->index()];
            parentIndex = parentBulletLink->mbIndex;
            R_iP = parentBulletLink->T_inertial.linear();
            c_P = parentLink->c();
            Tb = link->Tb();
        }

        const Matrix3& R_iL = bulletLink->T_inertial.linear();

        // Rotation that maps the coordinates of a vector in the parent inertial
        // frame to the coordinates in this inertial frame at q = 0
        Matrix3 M = R_iL.transpose() * Tb.linear().transpose() * R_iP;
        btQuaternion rotParentToThis = toBtQuaternion(M);

        btVector3 parentComToThisPivotOffset = toBtVector3(
            R_iP.transpose() * (Tb.translation() - c_P));
        btVector3 thisPivotToThisComOffset = toBtVector3(
            R_iL.transpose() * link->c());

        btScalar mass = static_cast<btScalar>(link->m());
        btVector3 inertia = toBtVector3(bulletLink->principalInertia);

        if(link->isRevoluteJoint()){
            btVector3 jointAxis = toBtVector3(R_iL.transpose() * link->jointAxis());
            multiBody->setupRevolute(
                i, mass, inertia, parentIndex, rotParentToThis, jointAxis,
                parentComToThisPivotOffset, thisPivotToThisComOffset, true);
        } else if(link->isPrismaticJoint()){
            btVector3 jointAxis = toBtVector3(R_iL.transpose() * link->jointAxis());
            multiBody->setupPrismatic(
                i, mass, inertia, parentIndex, rotParentToThis, jointAxis,
                parentComToThisPivotOffset, thisPivotToThisComOffset, true);
        } else {
            // Fixed joints, pseudo continuous tracks, and any other joint types
            // are welded to the parent
            multiBody->setupFixed(
                i, mass, inertia, parentIndex, rotParentToThis,
                parentComToThisPivotOffset, thisPivotToThisComOffset);
        }

        if(simImpl->isRotorInertiaEnabled && link->Jm2() > 0.0 &&
           (link->isRevoluteJoint() || link->isPrismaticJoint())){
            setupRotorLink(bulletLink, nextRotorMbIndex++, parentIndex, R_iP, c_P, Tb);
        }
    }

    /*
       btMultiBody clamps every generalized velocity to its max coordinate
       velocity (100 by default) when the solver impulses are applied. The
       rotor links spin gearRatio times faster than their joints, so with the
       default limit the coupled joints could not exceed 100 / gearRatio.
       Scale the limit by the maximum gear ratio to keep the equivalent
       velocity margin for the joints.
    */
    double maxRotorGearRatio = 0.0;
    for(int i = 0; i < numMbLinks; ++i){
        if(unit->mbLinks[i]->rotorMbIndex >= 0){
            maxRotorGearRatio = std::max(maxRotorGearRatio, unit->mbLinks[i]->rotorGearRatio);
        }
    }
    if(maxRotorGearRatio > 1.0){
        multiBody->setMaxCoordinateVelocity(
            static_cast<btScalar>(multiBody->getMaxCoordinateVelocity() * maxRotorGearRatio));
    }

    // Set up the internal prismatic segment links of the continuous tracks
    if(trackSetup){
        trackSetup->setupInternalLinks(unit.get(), numMbLinks);
    }

    multiBody->finalizeMultiDof();
    simImpl->world->addMultiBody(multiBody);

    // Set the initial joint states
    for(int i = 0; i < numMbLinks; ++i){
        BulletLink* bulletLink = unit->mbLinks[i];
        Link* link = bulletLink->link;
        if(link->isRevoluteJoint() || link->isPrismaticJoint()){
            multiBody->setJointPos(i, static_cast<btScalar>(link->q()));
            multiBody->setJointVel(i, static_cast<btScalar>(link->dq()));
            if(bulletLink->rotorMbIndex >= 0){
                multiBody->setJointVel(
                    bulletLink->rotorMbIndex,
                    static_cast<btScalar>(bulletLink->rotorGearRatio * link->dq()));
            }
        }
    }
    if(unit->baseLink){
        Link* link = unit->baseLink->link;
        Vector3 vCom = link->v() + link->w().cross(Vector3(link->R() * link->c()));
        multiBody->setBaseVel(toBtVector3(vCom));
        multiBody->setBaseOmega(toBtVector3(link->w()));
    }

    // Joint limits and joint control
    for(int i = 0; i < numMbLinks; ++i){
        BulletLink* bulletLink = unit->mbLinks[i];
        Link* link = bulletLink->link;
        if(link->hasActualJoint()){
            if(link->hasJointDisplacementLimits()){
                auto limit = new btMultiBodyJointLimitConstraint(
                    multiBody, i,
                    static_cast<btScalar>(link->q_lower()),
                    static_cast<btScalar>(link->q_upper()));
                simImpl->addMultiBodyConstraint(limit);
            }
            // The wheel joints of the continuous tracks are driven through
            // the gear constraints of the track handlers; the regular joint
            // motor would conflict with them
            if(!simImpl->trackSimulator ||
               !simImpl->trackSimulator->isTrackDrivenJoint(link)){
                setupJointControl(bulletLink);
            }
            if(bulletLink->rotorMbIndex >= 0){
                /*
                   Gear constraint coupling the virtual rotor with the joint so
                   that dq_rotor = gearRatio * dq_joint. The constraint equation
                   of btMultiBodyGearConstraint is velA + ratio * velB = 0, so
                   the ratio is set to the negated gear ratio.
                */
                auto gear = new btMultiBodyGearConstraint(
                    multiBody, bulletLink->rotorMbIndex, multiBody, bulletLink->mbIndex,
                    btVector3(0, 0, 0), btVector3(0, 0, 0),
                    btMatrix3x3::getIdentity(), btMatrix3x3::getIdentity());
                gear->setGearRatio(static_cast<btScalar>(-bulletLink->rotorGearRatio));
                gear->setMaxAppliedImpulse(static_cast<btScalar>(RotorGearConstraintMaxImpulse));
                simImpl->addMultiBodyConstraint(gear);
            }
        }
    }

    // Colliders. Every multibody link gets a collider object because the link
    // poses are read back from the collider transforms, which the multibody
    // updates every step. Colliders without shapes are not added to the world.
    auto createCollider = [this](BulletLink* bulletLink, btMultiBody* multiBody, int mbIndex){
        auto compound = new btCompoundShape;
        bulletLink->compound = compound;
        simImpl->ownedShapes.emplace_back(compound);
        if(bodyCollisionLinkFilter.checkIfEnabledLinkIndex(bulletLink->link->index())){
            if(auto shape = bulletLink->link->collisionShape()){
                simImpl->meshExtractor.extract(
                    shape, [this, bulletLink](){ simImpl->readMeshNode(bulletLink); });
            }
        }
        auto collider = new btMultiBodyLinkCollider(multiBody, mbIndex);
        bulletLink->collider = collider;
        collider->setCollisionShape(compound);
        collider->setWorldTransform(toBtTransform(bulletLink->link->T() * bulletLink->T_inertial));
        collider->setActivationState(DISABLE_DEACTIVATION);
        simImpl->applyLinkMaterial(collider, bulletLink);
        simImpl->collisionObjects.emplace_back(collider);
        if(compound->getNumChildShapes() > 0){
            simImpl->world->addCollisionObject(
                collider, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
        } else {
            /*
               The colliders of the links without collision shapes must also be
               registered in the world. A btMultiBodyLinkCollider that exists on
               a multibody link but is not registered keeps the island tag -1,
               which makes btMultiBodyDynamicsWorld::calculateSimulationIslands
               access the island union-find array out of bounds (heap corruption
               and eventual crashes), and the multibody constraints anchored to
               the link (joint motors, joint limits, and gear constraints) never
               match any simulation island and are silently ignored. A small
               placeholder shape keeps the broadphase AABB valid, and the empty
               collision filter mask prevents any broadphase pair from being
               created for the collider.
            */
            auto placeholder = new btSphereShape(static_cast<btScalar>(0.001));
            simImpl->ownedShapes.emplace_back(placeholder);
            compound->addChildShape(btTransform::getIdentity(), placeholder);
            simImpl->world->addCollisionObject(collider, btBroadphaseProxy::DefaultFilter, 0);
        }
        return collider;
    };

    if(unit->baseLink){
        auto collider = createCollider(unit->baseLink, multiBody, -1);
        multiBody->setBaseCollider(collider);
    }
    for(int i = 0; i < numMbLinks; ++i){
        auto collider = createCollider(unit->mbLinks[i], multiBody, i);
        multiBody->getLink(i).m_collider = collider;
    }

    // Complete the continuous track setup: the grouser and belt shapes on
    // the wheels, the segment colliders, the gear constraints, and the drives
    if(trackSetup){
        trackSetup->completeSetup();
    }

    // Sub units connected by free joints
    for(Link* subUnitRoot : subUnitRoots){
        createUnit(subUnitRoot);
    }
}


/*
   Append a virtual rotor link realizing the equivalent rotor inertia
   (armature) of a joint. The Bullet multibody dynamics has no direct armature
   parameter, so a rotor link with the moment of inertia
   Jr = Jm2 / gearRatio^2 about the joint axis is attached to the parent link
   at the joint pivot and coupled with the joint by a gear constraint
   (dq_rotor = gearRatio * dq_joint), which reflects gearRatio^2 * Jr = Jm2
   onto the joint axis inertia.

   The reflected inertia and the convergence of the iterative constraint
   solver depend only on Jm2 and not on how it is decomposed into Jr and the
   gear ratio; the decomposition only changes the gyroscopic moment of the
   spinning rotor (proportional to Jm2 / gearRatio), so using the default gear
   ratio when the model does not provide one is a safe approximation. The
   gear_ratio value written in a body file is kept in the link info by the
   body loaders and is read from there.

   The rotor mass is set to zero because the motor mass is normally included
   in the link mass parameters and only the rotational inertia matters here.
   The rotor inertia is made spherical so that it is diagonal in any frame;
   this adds a negligible transverse inertia of Jr to the parent link. For a
   prismatic joint the rotor is still a revolute link spinning about the
   slide axis and the gear ratio has the unit of rad/m.
*/
void BulletBody::setupRotorLink(
    BulletLink* bulletLink, int rotorMbIndex, int parentIndex,
    const Matrix3& R_iP, const Vector3& c_P, const Isometry3& Tb)
{
    Link* link = bulletLink->link;

    double gearRatio = 0.0;
    link->info()->read({"gear_ratio", "gearRatio"}, gearRatio);
    if(gearRatio <= 0.0){
        gearRatio = simImpl->defaultRotorGearRatio;
        rotorDefaultGearRatioJointNames.push_back(link->jointName());
    }
    double Jr = link->Jm2() / (gearRatio * gearRatio);
    if(Jr < MinRotorInertia){
        // Keep the reflected inertia by adjusting the gear ratio to the bound
        Jr = MinRotorInertia;
        gearRatio = sqrt(link->Jm2() / MinRotorInertia);
    }
    bulletLink->rotorMbIndex = rotorMbIndex;
    bulletLink->rotorGearRatio = gearRatio;

    // The rotor link frame has the same orientation as the parent inertial
    // frame and its origin, which is also the center of mass, is at the
    // joint pivot
    btVector3 rotorAxis = toBtVector3(R_iP.transpose() * (Tb.linear() * link->jointAxis()));
    btVector3 parentComToPivotOffset = toBtVector3(R_iP.transpose() * (Tb.translation() - c_P));
    const btScalar J = static_cast<btScalar>(Jr);

    bulletLink->unit->multiBody->setupRevolute(
        rotorMbIndex, 0.0, btVector3(J, J, J), parentIndex,
        btQuaternion(0, 0, 0, 1), rotorAxis,
        parentComToPivotOffset, btVector3(0, 0, 0), true);
}


void BulletBody::setupJointControl(BulletLink* bulletLink)
{
    Link* link = bulletLink->link;

    bulletLink->controlType = controlTypeFromActuationMode(link->actuationMode());
    if(bulletLink->controlType == NoControl){
        return;
    }

    double maxImpulse = UnlimitedMotorImpulse;
    if(link->hasJointEffortLimits()){
        maxImpulse = std::max(fabs(link->u_upper()), fabs(link->u_lower())) * simImpl->timeStep;
    }

    if(bulletLink->controlType == PositionControl || bulletLink->controlType == VelocityControl){
        // The motor gains are dimensionless error reduction gains of the
        // implicitly solved motor constraint (1.0 means reaching the target in
        // one step within the force limit), unlike the stiffness/damping gains
        // of the PhysX and MuJoCo plugins. The gains can still be overridden
        // per joint in the body file with the link info keys drive_stiffness
        // and drive_damping, which are shared with the other plugins, but the
        // values are engine specific.
        bulletLink->kp = simImpl->positionGain;
        bulletLink->kv = simImpl->velocityGain;
        link->info()->read("drive_stiffness", bulletLink->kp);
        link->info()->read("drive_damping", bulletLink->kv);

        auto motor = new btMultiBodyJointMotor(
            bulletLink->unit->multiBody, bulletLink->mbIndex, 0.0,
            static_cast<btScalar>(maxImpulse));
        if(bulletLink->controlType == PositionControl){
            motor->setPositionTarget(static_cast<btScalar>(link->q()),
                                     static_cast<btScalar>(bulletLink->kp));
            motor->setVelocityTarget(0.0, static_cast<btScalar>(bulletLink->kv));
            bulletLink->velocityFeedforward = link->actuationMode() & Link::JointVelocity;
        } else {
            motor->setPositionTarget(0.0, 0.0);
            motor->setVelocityTarget(0.0, static_cast<btScalar>(bulletLink->kv));
        }
        bulletLink->motor = motor;
        simImpl->addMultiBodyConstraint(motor);
    }

    controlLinks.push_back(bulletLink);
}


void BulletBody::createStaticLink(Link* link)
{
    BulletLink* bulletLink = getOrCreateBulletLink(link);
    bulletLink->mbIndex = -2;

    if(!bodyCollisionLinkFilter.checkIfEnabledLinkIndex(link->index())){
        return;
    }
    auto shape = link->collisionShape();
    if(!shape){
        return;
    }

    auto compound = new btCompoundShape;
    bulletLink->compound = compound;
    simImpl->ownedShapes.emplace_back(compound);
    simImpl->meshExtractor.extract(
        shape, [this, bulletLink](){ simImpl->readMeshNode(bulletLink); });

    if(compound->getNumChildShapes() == 0){
        return;
    }

    btRigidBody::btRigidBodyConstructionInfo info(0.0, nullptr, compound);
    auto body = new btRigidBody(info);
    bulletLink->rigidBody = body;
    body->setWorldTransform(toBtTransform(link->T()));
    simImpl->applyLinkMaterial(body, bulletLink);
    simImpl->collisionObjects.emplace_back(body);
    simImpl->world->addRigidBody(
        body, btBroadphaseProxy::StaticFilter,
        btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);
}


/*
   Create a dynamic btRigidBody for a single free-floating link without any
   joints. The world transform and the local inertia follow the same principal
   axes frame convention as the multibody links. See the comment in createUnit
   for when this representation is chosen.
*/
void BulletBody::createRigidBodyLink(Link* link)
{
    BulletLink* bulletLink = getOrCreateBulletLink(link);
    bulletLink->mbIndex = -3;
    bulletLink->initInertialFrame();
    rigidBodyLinks.push_back(bulletLink);

    auto compound = new btCompoundShape;
    bulletLink->compound = compound;
    simImpl->ownedShapes.emplace_back(compound);
    if(bodyCollisionLinkFilter.checkIfEnabledLinkIndex(link->index())){
        if(auto shape = link->collisionShape()){
            simImpl->meshExtractor.extract(
                shape, [this, bulletLink](){ simImpl->readMeshNode(bulletLink); });
        }
    }

    btRigidBody::btRigidBodyConstructionInfo info(
        static_cast<btScalar>(link->m()), nullptr, compound,
        toBtVector3(bulletLink->principalInertia));
    auto body = new btRigidBody(info);
    bulletLink->rigidBody = body;
    body->setWorldTransform(toBtTransform(link->T() * bulletLink->T_inertial));

    // The initial linear velocity is the velocity at the center of mass
    Vector3 vCom = link->v() + link->w().cross(Vector3(link->R() * link->c()));
    body->setLinearVelocity(toBtVector3(vCom));
    body->setAngularVelocity(toBtVector3(link->w()));
    body->setDamping(
        static_cast<btScalar>(simImpl->linearDamping),
        static_cast<btScalar>(simImpl->angularDamping));
    body->setActivationState(DISABLE_DEACTIVATION);
    simImpl->applyLinkMaterial(body, bulletLink);
    simImpl->collisionObjects.emplace_back(body);

    if(compound->getNumChildShapes() > 0){
        simImpl->world->addRigidBody(
            body, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
    } else {
        // A dynamic rigid body must be registered in the world to be
        // integrated even when it has no collision shapes. As with the
        // multibody link colliders, a placeholder shape keeps the broadphase
        // AABB valid and the empty collision filter mask prevents any
        // broadphase pair from being created.
        auto placeholder = new btSphereShape(static_cast<btScalar>(0.001));
        simImpl->ownedShapes.emplace_back(placeholder);
        compound->addChildShape(btTransform::getIdentity(), placeholder);
        simImpl->world->addRigidBody(body, btBroadphaseProxy::DefaultFilter, 0);
    }
}


void BulletBody::initializeSensors()
{
    auto body_ = body();
    sensorHelper.initialize(body_, simImpl->timeStep, simImpl->gravity);

    forceSensorInfos.clear();
    hasForceSensors = false;
    for(auto& sensor : sensorHelper.forceSensors()){
        BulletLink* bulletLink = bulletLinks[sensor->link()->index()];
        if(bulletLink && bulletLink->mbIndex >= 0){
            auto& mbLink = bulletLink->unit->multiBody->getLink(bulletLink->mbIndex);
            bulletLink->jointFeedback = make_unique<btMultiBodyJointFeedback>();
            mbLink.m_jointFeedback = bulletLink->jointFeedback.get();
            BulletForceSensorInfo info;
            info.sensor = sensor;
            info.bulletLink = bulletLink;
            forceSensorInfos.push_back(info);
            hasForceSensors = true;
        } else {
            simImpl->mout->putWarningln(
                formatR(_("The force sensor \"{0}\" is ignored because its link is not "
                          "simulated as a movable joint of a multibody."), sensor->name()));
        }
    }

    needsSensorVelocities = sensorHelper.hasGyroOrAccelerationSensors();

    for(auto& sensor : sensorHelper.accelerationSensors()){
        if(auto bulletLink = bulletLinks[sensor->link()->index()]){
            bulletLink->hasAccelSensor = true;
        }
    }
    for(auto& sensor : sensorHelper.imus()){
        if(auto bulletLink = bulletLinks[sensor->link()->index()]){
            bulletLink->hasAccelSensor = true;
        }
    }

    // Initialize the previous velocities for the finite difference acceleration
    for(auto& bulletLink : bulletLinks){
        if(bulletLink){
            bulletLink->prevV = bulletLink->link->v();
            bulletLink->prevW = bulletLink->link->w();
        }
    }
}


//---------------------------------------------------------------------------
// Collision shape creation
//---------------------------------------------------------------------------

void BulletSimulatorItem::Impl::readMeshNode(BulletLink* bulletLink)
{
    SgMesh* mesh = meshExtractor.currentMesh();
    if(!mesh){
        return;
    }

    Vector3 scale = Vector3::Ones();
    std::optional<Vector3> translation;
    if(meshExtractor.isCurrentScaled()){
        Affine3 S = meshExtractor.currentTransformWithoutScaling().inverse()
                  * meshExtractor.currentTransform();
        if(S.linear().isDiagonal()){
            scale = S.linear().diagonal();
            if(!S.translation().isZero()){
                translation = S.translation();
            }
        } else {
            mout->putWarningln(
                formatR(_("{0} has a non-axis-aligned scaling which is not supported "
                          "in the Bullet plugin. The mesh is skipped."),
                        meshLabel(meshExtractor.currentShape())));
            return;
        }
    }

    btCollisionShape* shape = nullptr;
    bool isTriangleMesh = false;

    if(mesh->primitiveType() != SgMesh::MeshType){
        shape = createPrimitiveShape(mesh, scale);
    }
    if(!shape){
        if(bulletLink->isStaticLink()){
            shape = createTriangleMeshShape(mesh, scale);
            isTriangleMesh = true;
        } else if(useGImpactForDynamicMeshes){
            shape = createGImpactMeshShape(mesh, scale);
            isTriangleMesh = true;
        } else {
            shape = createConvexHullShape(mesh, scale);
        }
    }
    if(!shape){
        return;
    }
    if(isTriangleMesh){
        bulletLink->hasTriangleMeshShape = true;
    }

    Isometry3 T = bulletLink->T_inertialInv * meshExtractor.currentTransformWithoutScaling();
    if(translation){
        T = T * Translation3(*translation);
    }
    bulletLink->compound->addChildShape(toBtTransform(T), shape);
}


btCollisionShape* BulletSimulatorItem::Impl::createPrimitiveShape(SgMesh* mesh, const Vector3& scale)
{
    // The Y-axis convention of the Choreonoid cylinder, capsule, and cone
    // primitives matches the Bullet primitive shapes, so no axis rotation is
    // needed. Primitives that cannot express the given scale fall back to the
    // mesh-based shapes.
    btCollisionShape* shape = nullptr;

    switch(mesh->primitiveType()){

    case SgMesh::BoxType: {
        auto box = mesh->primitive<SgMesh::Box>();
        shape = new btBoxShape(
            btVector3(
                static_cast<btScalar>(box.size.x() * scale.x() / 2.0),
                static_cast<btScalar>(box.size.y() * scale.y() / 2.0),
                static_cast<btScalar>(box.size.z() * scale.z() / 2.0)));
        break;
    }
    case SgMesh::SphereType: {
        if(scale.x() == scale.y() && scale.x() == scale.z()){
            auto sphere = mesh->primitive<SgMesh::Sphere>();
            shape = new btSphereShape(static_cast<btScalar>(sphere.radius * scale.x()));
        }
        break;
    }
    case SgMesh::CapsuleType: {
        if(scale.x() == scale.z()){
            auto capsule = mesh->primitive<SgMesh::Capsule>();
            shape = new btCapsuleShape(
                static_cast<btScalar>(capsule.radius * scale.x()),
                static_cast<btScalar>(capsule.height * scale.y()));
        }
        break;
    }
    case SgMesh::CylinderType: {
        if(scale.x() == scale.z()){
            auto cylinder = mesh->primitive<SgMesh::Cylinder>();
            shape = new btCylinderShape(
                btVector3(
                    static_cast<btScalar>(cylinder.radius * scale.x()),
                    static_cast<btScalar>(cylinder.height * scale.y() / 2.0),
                    static_cast<btScalar>(cylinder.radius * scale.x())));
        }
        break;
    }
    case SgMesh::ConeType: {
        if(scale.x() == scale.z()){
            auto cone = mesh->primitive<SgMesh::Cone>();
            shape = new btConeShape(
                static_cast<btScalar>(cone.radius * scale.x()),
                static_cast<btScalar>(cone.height * scale.y()));
        }
        break;
    }
    default:
        break;
    }

    if(shape){
        ownedShapes.emplace_back(shape);
    }

    return shape;
}


btCollisionShape* BulletSimulatorItem::Impl::createConvexHullShape(SgMesh* mesh, const Vector3& scale)
{
    MeshShapeKey key(mesh, { scale.x(), scale.y(), scale.z() });
    auto it = convexShapeMap.find(key);
    if(it != convexShapeMap.end()){
        return it->second;
    }

    auto vertices = mesh->vertices();
    if(!vertices || vertices->empty()){
        mout->putWarningln(
            formatR(_("{0} has no vertices for creating a convex hull."),
                    meshLabel(meshExtractor.currentShape())));
        return nullptr;
    }

    auto hull = new btConvexHullShape;

    if(mesh->hasTriangles()){
        // Collect vertex indices from valid (non-degenerate) triangles only so
        // that stray vertices do not distort the hull
        auto& indices = mesh->triangleVertices();
        std::unordered_set<int> refIndices;
        const int numTriangles = mesh->numTriangles();
        for(int i = 0; i < numTriangles; ++i){
            int i0 = indices[i * 3];
            int i1 = indices[i * 3 + 1];
            int i2 = indices[i * 3 + 2];
            if(i0 != i1 && i1 != i2 && i2 != i0){
                refIndices.insert(i0);
                refIndices.insert(i1);
                refIndices.insert(i2);
            }
        }
        for(int index : refIndices){
            const Vector3f& v = (*vertices)[index];
            hull->addPoint(
                btVector3(
                    static_cast<btScalar>(v.x() * scale.x()),
                    static_cast<btScalar>(v.y() * scale.y()),
                    static_cast<btScalar>(v.z() * scale.z())),
                false);
        }
    } else {
        for(const auto& v : *vertices){
            hull->addPoint(
                btVector3(
                    static_cast<btScalar>(v.x() * scale.x()),
                    static_cast<btScalar>(v.y() * scale.y()),
                    static_cast<btScalar>(v.z() * scale.z())),
                false);
        }
    }

    if(hull->getNumPoints() == 0){
        delete hull;
        mout->putWarningln(
            formatR(_("{0} has no valid vertices for creating a convex hull."),
                    meshLabel(meshExtractor.currentShape())));
        return nullptr;
    }

    hull->recalcLocalAabb();
    hull->optimizeConvexHull();
    hull->setMargin(static_cast<btScalar>(collisionMargin));

    ownedShapes.emplace_back(hull);
    convexShapeMap[key] = hull;
    return hull;
}


btTriangleIndexVertexArray* BulletSimulatorItem::Impl::createMeshInterface(SgMesh* mesh, const Vector3& scale)
{
    auto vertices = mesh->vertices();
    if(!vertices || vertices->empty() || !mesh->hasTriangles()){
        mout->putWarningln(
            formatR(_("{0} does not have valid triangle mesh data."),
                    meshLabel(meshExtractor.currentShape())));
        return nullptr;
    }

    auto buffers = make_unique<MeshBuffers>();
    const int numVertices = static_cast<int>(vertices->size());
    buffers->vertices.resize(numVertices * 3);
    for(int i = 0; i < numVertices; ++i){
        const Vector3f& v = (*vertices)[i];
        buffers->vertices[i * 3]     = static_cast<btScalar>(v.x() * scale.x());
        buffers->vertices[i * 3 + 1] = static_cast<btScalar>(v.y() * scale.y());
        buffers->vertices[i * 3 + 2] = static_cast<btScalar>(v.z() * scale.z());
    }
    const int numTriangles = mesh->numTriangles();
    auto& triangleVertices = mesh->triangleVertices();
    buffers->indices.assign(triangleVertices.begin(), triangleVertices.end());

    auto meshInterface = new btTriangleIndexVertexArray(
        numTriangles, buffers->indices.data(), 3 * sizeof(int),
        numVertices, buffers->vertices.data(), 3 * sizeof(btScalar));

    meshBuffers.push_back(std::move(buffers));
    meshInterfaces.emplace_back(meshInterface);
    return meshInterface;
}


btCollisionShape* BulletSimulatorItem::Impl::createTriangleMeshShape(SgMesh* mesh, const Vector3& scale)
{
    MeshShapeKey key(mesh, { scale.x(), scale.y(), scale.z() });
    auto it = triangleMeshShapeMap.find(key);
    if(it != triangleMeshShapeMap.end()){
        return it->second;
    }

    auto meshInterface = createMeshInterface(mesh, scale);
    if(!meshInterface){
        return nullptr;
    }

    auto shape = new btBvhTriangleMeshShape(meshInterface, true);
    shape->setMargin(static_cast<btScalar>(collisionMargin));

    if(isInternalEdgeSmoothingEnabled){
        auto triangleInfoMap = new btTriangleInfoMap;
        btGenerateInternalEdgeInfo(shape, triangleInfoMap);
        triangleInfoMaps.emplace_back(triangleInfoMap);
    }

    ownedShapes.emplace_back(shape);
    triangleMeshShapeMap[key] = shape;
    return shape;
}


btCollisionShape* BulletSimulatorItem::Impl::createGImpactMeshShape(SgMesh* mesh, const Vector3& scale)
{
    MeshShapeKey key(mesh, { scale.x(), scale.y(), scale.z() });
    auto it = gimpactShapeMap.find(key);
    if(it != gimpactShapeMap.end()){
        return it->second;
    }

    auto meshInterface = createMeshInterface(mesh, scale);
    if(!meshInterface){
        return nullptr;
    }

    auto shape = new btGImpactMeshShape(meshInterface);
    shape->setMargin(static_cast<btScalar>(collisionMargin));
    shape->updateBound();

    ownedShapes.emplace_back(shape);
    gimpactShapeMap[key] = shape;
    return shape;
}


//---------------------------------------------------------------------------
// Extra joints
//---------------------------------------------------------------------------

BulletLink* BulletSimulatorItem::Impl::findBulletLink(Link* link)
{
    if(!link){
        return nullptr;
    }
    auto p = bodyMap.find(link->body());
    if(p == bodyMap.end()){
        return nullptr;
    }
    auto bulletBody = p->second;
    const int linkIndex = link->index();
    if(linkIndex < 0 || linkIndex >= static_cast<int>(bulletBody->bulletLinks.size())){
        return nullptr;
    }
    BulletLink* bulletLink = bulletBody->bulletLinks[linkIndex];
    if(!bulletLink || bulletLink->link != link){
        return nullptr;
    }
    return bulletLink;
}


void BulletSimulatorItem::Impl::addExtraJoints(const std::vector<SimulationBody*>& simBodies)
{
    std::unordered_set<ExtraJoint*> initializedExtraJoints;

    for(auto simBody : simBodies){
        auto bulletBody = static_cast<BulletBody*>(simBody);
        auto body = bulletBody->body();
        int numExtraJoints = body->numExtraJoints();

        for(int i = 0; i < numExtraJoints; ++i){
            ExtraJoint* extraJoint = body->extraJoint(i);
            if(!initializedExtraJoints.insert(extraJoint).second){
                continue;
            }

            Link* links[2] = { extraJoint->link(0), extraJoint->link(1) };
            BulletLink* bulletLinks[2] = {
                findBulletLink(links[0]),
                findBulletLink(links[1])
            };
            if(!links[0] || !links[1]){
                continue;
            }
            if(!bulletLinks[0] || !bulletLinks[1]){
                mout->putWarningln(
                    formatR(_("The extra joint between {0} and {1} is ignored because "
                              "one of the links is not included in the simulation."),
                            links[0]->name(), links[1]->name()));
                continue;
            }

            // Arrange the pair so that the first slot is a multibody link if
            // the pair has one. The Bullet multibody constraints are used when
            // at least one link is a multibody link; the other side can then
            // be a multibody link or a static or dynamic rigid body. When both
            // links are rigid bodies, the ordinary rigid body constraints are
            // used instead because the multibody constraints require at least
            // one multibody.
            int iA = 0;
            int iB = 1;
            if(bulletLinks[iA]->isRigidBodyLink()){
                std::swap(iA, iB);
            }
            BulletLink* linkA = bulletLinks[iA];
            BulletLink* linkB = bulletLinks[iB];
            if(linkA->isStaticLink() && linkB->isStaticLink()){
                continue; // both links are static; nothing to constrain
            }
            btRigidBody* rbB = linkB->rigidBody;
            if((linkA->isRigidBodyLink() && !linkA->rigidBody) ||
               (linkB->isRigidBodyLink() && !rbB)){
                // A static link without collision shapes has no rigid body;
                // creating one on demand is not supported, so skip with a warning.
                mout->putWarningln(
                    formatR(_("The extra joint between {0} and {1} is ignored because "
                              "the static link has no body in the simulation."),
                            links[0]->name(), links[1]->name()));
                continue;
            }

            // Local frames converted to the Bullet link (inertial) frames
            auto localPivot = [&](BulletLink* bl, int slot){
                return toBtVector3(
                    (bl->T_inertialInv * extraJoint->localPosition(slot)).translation());
            };
            auto localFrame = [&](BulletLink* bl, int slot){
                return toBtMatrix3x3(
                    bl->T_inertialInv.linear() * extraJoint->localRotation(slot));
            };

            int jointType = extraJoint->type();

            if(linkA->isRigidBodyLink()){
                // Both links are rigid bodies and at least one of them is
                // dynamic; realize the extra joint with the ordinary rigid
                // body constraints
                btRigidBody* rbA = linkA->rigidBody;

                if(jointType == ExtraJoint::Ball){
                    addTypedConstraint(
                        new btPoint2PointConstraint(
                            *rbA, *rbB, localPivot(linkA, iA), localPivot(linkB, iB)));

                } else if(jointType == ExtraJoint::Hinge){
                    // Pin two points along the hinge axis like the multibody case
                    constexpr double halfSpan = 0.05;
                    for(int s = -1; s <= 1; s += 2){
                        Vector3 pA = extraJoint->localTranslation(iA)
                            + (s * halfSpan) * (extraJoint->localRotation(iA) * extraJoint->axis());
                        Vector3 pB = extraJoint->localTranslation(iB)
                            + (s * halfSpan) * (extraJoint->localRotation(iB) * extraJoint->axis());
                        addTypedConstraint(
                            new btPoint2PointConstraint(
                                *rbA, *rbB,
                                toBtVector3(linkA->T_inertialInv * pA),
                                toBtVector3(linkB->T_inertialInv * pB)));
                    }

                } else if(jointType == ExtraJoint::Fixed){
                    btTransform frameA(localFrame(linkA, iA), localPivot(linkA, iA));
                    btTransform frameB(localFrame(linkB, iB), localPivot(linkB, iB));
                    addTypedConstraint(new btFixedConstraint(*rbA, *rbB, frameA, frameB));

                } else if(jointType == ExtraJoint::Piston){
                    // The slider axis of btSliderConstraint is the x-axis of
                    // the constraint frames
                    auto sliderFrame = [&](BulletLink* bl, int slot){
                        Vector3 axis = bl->T_inertialInv.linear()
                            * (extraJoint->localRotation(slot) * extraJoint->axis());
                        Matrix3 R = Quaternion::FromTwoVectors(
                            Vector3::UnitX(), axis).toRotationMatrix();
                        return btTransform(toBtMatrix3x3(R), localPivot(bl, slot));
                    };
                    auto slider = new btSliderConstraint(
                        *rbA, *rbB, sliderFrame(linkA, iA), sliderFrame(linkB, iB), true);
                    // A piston joint is a cylindrical pair: both the translation
                    // along the axis and the rotation about it are free
                    slider->setLowerLinLimit(1.0);
                    slider->setUpperLinLimit(-1.0);
                    slider->setLowerAngLimit(1.0);
                    slider->setUpperAngLimit(-1.0);
                    addTypedConstraint(slider);
                }
                continue;
            }

            btMultiBody* mbA = linkA->unit->multiBody;
            btMultiBody* mbB = linkB->isRigidBodyLink() ? nullptr : linkB->unit->multiBody;
            std::vector<btMultiBodyConstraint*> constraints;

            if(jointType == ExtraJoint::Ball){
                if(mbB){
                    constraints.push_back(
                        new btMultiBodyPoint2Point(
                            mbA, linkA->mbIndex, mbB, linkB->mbIndex,
                            localPivot(linkA, iA), localPivot(linkB, iB)));
                } else {
                    constraints.push_back(
                        new btMultiBodyPoint2Point(
                            mbA, linkA->mbIndex, rbB,
                            localPivot(linkA, iA), localPivot(linkB, iB)));
                }

            } else if(jointType == ExtraJoint::Hinge){
                // Pin two points along the hinge axis. Connecting both pairs
                // makes the axis lines coincide while leaving rotation about it.
                constexpr double halfSpan = 0.05;
                for(int s = -1; s <= 1; s += 2){
                    Vector3 pA = extraJoint->localTranslation(iA)
                        + (s * halfSpan) * (extraJoint->localRotation(iA) * extraJoint->axis());
                    Vector3 pB = extraJoint->localTranslation(iB)
                        + (s * halfSpan) * (extraJoint->localRotation(iB) * extraJoint->axis());
                    btVector3 pivotA = toBtVector3(linkA->T_inertialInv * pA);
                    btVector3 pivotB = toBtVector3(linkB->T_inertialInv * pB);
                    if(mbB){
                        constraints.push_back(
                            new btMultiBodyPoint2Point(
                                mbA, linkA->mbIndex, mbB, linkB->mbIndex, pivotA, pivotB));
                    } else {
                        constraints.push_back(
                            new btMultiBodyPoint2Point(
                                mbA, linkA->mbIndex, rbB, pivotA, pivotB));
                    }
                }

            } else if(jointType == ExtraJoint::Fixed){
                if(mbB){
                    constraints.push_back(
                        new btMultiBodyFixedConstraint(
                            mbA, linkA->mbIndex, mbB, linkB->mbIndex,
                            localPivot(linkA, iA), localPivot(linkB, iB),
                            localFrame(linkA, iA), localFrame(linkB, iB)));
                } else {
                    constraints.push_back(
                        new btMultiBodyFixedConstraint(
                            mbA, linkA->mbIndex, rbB,
                            localPivot(linkA, iA), localPivot(linkB, iB),
                            localFrame(linkA, iA), localFrame(linkB, iB)));
                }

            } else if(jointType == ExtraJoint::Piston){
                btVector3 jointAxis = toBtVector3(
                    linkA->T_inertialInv.linear() * (extraJoint->localRotation(iA) * extraJoint->axis()));
                if(mbB){
                    constraints.push_back(
                        new btMultiBodySliderConstraint(
                            mbA, linkA->mbIndex, mbB, linkB->mbIndex,
                            localPivot(linkA, iA), localPivot(linkB, iB),
                            localFrame(linkA, iA), localFrame(linkB, iB),
                            jointAxis));
                } else {
                    constraints.push_back(
                        new btMultiBodySliderConstraint(
                            mbA, linkA->mbIndex, rbB,
                            localPivot(linkA, iA), localPivot(linkB, iB),
                            localFrame(linkA, iA), localFrame(linkB, iB),
                            jointAxis));
                }
            }

            for(auto constraint : constraints){
                constraint->setMaxAppliedImpulse(static_cast<btScalar>(ExtraJointMaxImpulse));
                addMultiBodyConstraint(constraint);
            }
        }
    }
}


//---------------------------------------------------------------------------
// Control input and state transfer
//---------------------------------------------------------------------------

void BulletBody::inputControl()
{
    for(auto bulletLink : controlLinks){
        Link* link = bulletLink->link;
        switch(bulletLink->controlType){

        case EffortControl: {
            double u = link->u();
            if(link->hasJointEffortLimits()){
                if(u < link->u_lower()) u = link->u_lower();
                if(u > link->u_upper()) u = link->u_upper();
            }
            bulletLink->lastJointEffort = u;
            bulletLink->unit->multiBody->addJointTorque(
                bulletLink->mbIndex, static_cast<btScalar>(u));
            break;
        }
        case PositionControl:
            bulletLink->motor->setPositionTarget(
                static_cast<btScalar>(link->q_target()),
                static_cast<btScalar>(bulletLink->kp));
            // The velocity target is used only when the JointVelocity bit is
            // also set; otherwise dq_target is not a supplied command and the
            // motor damps the joint velocity toward zero (a plain servo).
            bulletLink->motor->setVelocityTarget(
                static_cast<btScalar>(bulletLink->velocityFeedforward ? link->dq_target() : 0.0),
                static_cast<btScalar>(bulletLink->kv));
            break;

        case VelocityControl:
            bulletLink->motor->setVelocityTarget(
                static_cast<btScalar>(link->dq_target()),
                static_cast<btScalar>(bulletLink->kv));
            break;

        default:
            break;
        }
    }
}


void BulletBody::addExternalForces()
{
    for(auto& bulletLink : bulletLinks){
        if(!bulletLink || bulletLink->isStaticLink()){
            continue;
        }
        Link* link = bulletLink->link;
        const Vector3& f = link->f_ext();
        const Vector3& tau = link->tau_ext();
        if(f.isZero() && tau.isZero()){
            continue;
        }
        // f_ext/tau_ext follow Choreonoid's convention: force applied at the
        // global origin and torque about the global origin. Convert the torque
        // to be about the link center of mass where Bullet applies the wrench.
        Vector3 pCom = link->T() * link->c();
        Vector3 tauAtCom = tau - pCom.cross(f);
        if(bulletLink->isDynamicRigidBodyLink()){
            bulletLink->rigidBody->applyCentralForce(toBtVector3(f));
            bulletLink->rigidBody->applyTorque(toBtVector3(tauAtCom));
            continue;
        }
        btMultiBody* multiBody = bulletLink->unit->multiBody;
        if(bulletLink->mbIndex < 0){
            multiBody->addBaseForce(toBtVector3(f));
            multiBody->addBaseTorque(toBtVector3(tauAtCom));
        } else {
            multiBody->addLinkForce(bulletLink->mbIndex, toBtVector3(f));
            multiBody->addLinkTorque(bulletLink->mbIndex, toBtVector3(tauAtCom));
        }
    }
    body()->clearExternalForces();
}


void BulletBody::getState()
{
    const bool velocityOutput =
        simImpl->isVelocityOutputEnabled || simImpl->isAccelerationOutputEnabled ||
        needsSensorVelocities;
    const bool accelerationOutput = simImpl->isAccelerationOutputEnabled;
    const double dt = simImpl->timeStep;

    for(auto bulletLink : rigidBodyLinks){
        Link* link = bulletLink->link;
        btRigidBody* body = bulletLink->rigidBody;
        Isometry3 T_bullet = toIsometry3(body->getWorldTransform());
        Isometry3 T = T_bullet * bulletLink->T_inertialInv;
        link->p() = T.translation();
        link->R() = T.linear();

        if(velocityOutput){
            Vector3 w = toEigenVector3(body->getAngularVelocity());
            Vector3 vCom = toEigenVector3(body->getLinearVelocity());
            link->w() = w;
            link->v() = vCom - w.cross(Vector3(T_bullet.translation() - link->p()));

            if(accelerationOutput || bulletLink->hasAccelSensor){
                link->dv() = (link->v() - bulletLink->prevV) / dt;
                link->dw() = (link->w() - bulletLink->prevW) / dt;
                bulletLink->prevV = link->v();
                bulletLink->prevW = link->w();
            }
        }
    }

    for(auto& unit : units){
        btMultiBody* multiBody = unit->multiBody;
        const int numMbLinks = static_cast<int>(unit->mbLinks.size());

        if(velocityOutput){
            auto& omegaBuf = simImpl->omegaBuf;
            auto& velBuf = simImpl->velBuf;
            // The buffers must cover all the multibody links, which can be
            // more than mbLinks because of the internal track segment links
            omegaBuf.resize(multiBody->getNumLinks() + 1);
            velBuf.resize(multiBody->getNumLinks() + 1);
            multiBody->compTreeLinkVelocities(omegaBuf.data(), velBuf.data());
        }

        auto updateLinkState = [&](BulletLink* bulletLink, const Isometry3& T_bullet, int velIndex){
            Link* link = bulletLink->link;
            Isometry3 T = T_bullet * bulletLink->T_inertialInv;
            link->p() = T.translation();
            link->R() = T.linear();

            if(velocityOutput){
                Vector3 w, vCom;
                if(velIndex == 0 && unit->baseLink){
                    // Base velocities are directly available in the world frame
                    w = toEigenVector3(multiBody->getBaseOmega());
                    vCom = toEigenVector3(multiBody->getBaseVel());
                } else {
                    // compTreeLinkVelocities gives the velocities in the local
                    // (inertial) frames; the linear velocity is at the frame
                    // origin, which is the link center of mass
                    const Matrix3& R_bullet = T_bullet.linear();
                    w = R_bullet * toEigenVector3(simImpl->omegaBuf[velIndex]);
                    vCom = R_bullet * toEigenVector3(simImpl->velBuf[velIndex]);
                }
                link->w() = w;
                link->v() = vCom - w.cross(Vector3(T_bullet.translation() - link->p()));

                if(accelerationOutput || bulletLink->hasAccelSensor){
                    link->dv() = (link->v() - bulletLink->prevV) / dt;
                    link->dw() = (link->w() - bulletLink->prevW) / dt;
                    bulletLink->prevV = link->v();
                    bulletLink->prevW = link->w();
                }
            }
        };

        if(unit->baseLink){
            updateLinkState(unit->baseLink, toIsometry3(multiBody->getBaseWorldTransform()), 0);
        }

        for(int i = 0; i < numMbLinks; ++i){
            BulletLink* bulletLink = unit->mbLinks[i];
            Link* link = bulletLink->link;
            updateLinkState(bulletLink, toIsometry3(bulletLink->collider->getWorldTransform()), i + 1);

            if(link->hasActualJoint()){
                link->q() = multiBody->getJointPos(i);
                if(velocityOutput){
                    link->dq() = multiBody->getJointVel(i);
                }
                if(simImpl->isDriveEffortOutputEnabled){
                    // Write back the drive effort generated by the actuator or
                    // motor constraint. This is not the net joint effort
                    // including gravity, contacts, or external forces.
                    if(bulletLink->controlType == EffortControl){
                        link->u() = bulletLink->lastJointEffort;
                    } else if(bulletLink->motor){
                        link->u() = bulletLink->motor->getAppliedImpulse(0) / dt;
                    }
                }
            }
        }
    }
}


void BulletBody::updateForceSensors()
{
    const double dt = simImpl->timeStep;

    for(auto& info : forceSensorInfos){
        BulletLink* bulletLink = info.bulletLink;
        Link* link = bulletLink->link;
        ForceSensor* sensor = info.sensor;

        // With m_jointFeedbackInWorldSpace and m_jointFeedbackInJointFrame both
        // enabled, the reaction force is the constraint wrench that the parent
        // applies to this link, expressed in the world frame with the moment
        // about the joint frame origin, which coincides with the link frame
        // origin in the Choreonoid convention.
        const auto& reaction = bulletLink->jointFeedback->m_reactionForces;
        Vector3 f = toEigenVector3(reaction.getLinear());
        Vector3 tau = toEigenVector3(reaction.getAngular());

        // The feedback only contains the constraint forces and does not
        // include the actuation along the joint axis (direct joint efforts
        // and motor constraint impulses), so it must be added separately.
        // Note that axial forces of joint limit constraints and the gear
        // constraints of the virtual rotor links are not covered.
        double driveEffort = 0.0;
        if(bulletLink->controlType == EffortControl){
            driveEffort = bulletLink->lastJointEffort;
        } else if(bulletLink->motor){
            driveEffort = bulletLink->motor->getAppliedImpulse(0) / dt;
        }
        if(driveEffort != 0.0){
            const Vector3 axis = link->R() * link->jointAxis();
            if(link->isRevoluteJoint()){
                tau += driveEffort * axis;
            } else if(link->isPrismaticJoint()){
                f += driveEffort * axis;
            }
        }

        // The Choreonoid convention of the force sensor value is the wrench
        // that the distal part applies to the proximal part
        f = -f;
        tau = -tau;

        const Matrix3 R_sensor = link->R() * sensor->R_local();
        const Vector3 p_sensor = link->p() + link->R() * sensor->p_local();

        // Shift the moment reference point from the joint origin to the sensor
        // position, then express the wrench in the sensor frame
        Vector3 tauAtSensor = tau + (link->p() - p_sensor).cross(f);
        sensor->f() = R_sensor.transpose() * f;
        sensor->tau() = R_sensor.transpose() * tauAtSensor;
        sensor->notifyStateChange();
    }
}


//---------------------------------------------------------------------------
// Properties and serialization
//---------------------------------------------------------------------------

void BulletSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SimulatorItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void BulletSimulatorItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Gravity"), str(gravity), [&](const string& v){ return toVector3(v, gravity); });
    putProperty(_("Solver"), solverType, changeProperty(solverType));
    putProperty.min(1)(_("Solver iterations"), numIterations, changeProperty(numIterations));
    putProperty.decimals(3).min(0.0).max(1.0)
        (_("Contact ERP"), contactErp, changeProperty(contactErp));
    putProperty.decimals(3).min(0.0).max(1.0)
        (_("Position gain"), positionGain, changeProperty(positionGain));
    putProperty.decimals(3).min(0.0).max(1.0)
        (_("Velocity gain"), velocityGain, changeProperty(velocityGain));
    putProperty.decimals(4).min(0.0)(_("Linear damping"), linearDamping, changeProperty(linearDamping));
    putProperty.decimals(4).min(0.0)(_("Angular damping"), angularDamping, changeProperty(angularDamping));
    putProperty.decimals(4).min(0.0)
        (_("Collision margin"), collisionMargin, changeProperty(collisionMargin));
    putProperty(_("GImpact meshes for dynamic objects"), useGImpactForDynamicMeshes,
                changeProperty(useGImpactForDynamicMeshes));
    putProperty(_("Internal edge smoothing"), isInternalEdgeSmoothingEnabled,
                changeProperty(isInternalEdgeSmoothingEnabled));
    putProperty(_("Self collision"), isSelfCollisionEnabledByDefault,
                changeProperty(isSelfCollisionEnabledByDefault));
    putProperty(_("Rotor inertia"), isRotorInertiaEnabled, changeProperty(isRotorInertiaEnabled));
    // The range must be given explicitly; the min/max values specified for
    // the preceding properties remain valid until they are overridden
    putProperty.decimals(1).range(1.0, 1.0e6)
        (_("Default rotor gear ratio"), defaultRotorGearRatio, changeProperty(defaultRotorGearRatio));
    putProperty(_("Velocity output"), isVelocityOutputEnabled, changeProperty(isVelocityOutputEnabled));
    putProperty(_("Acceleration output"), isAccelerationOutputEnabled, changeProperty(isAccelerationOutputEnabled));
    putProperty(_("Drive effort output"), isDriveEffortOutputEnabled, changeProperty(isDriveEffortOutputEnabled));
}


bool BulletSimulatorItem::store(Archive& archive)
{
    SimulatorItem::store(archive);
    impl->store(archive);
    return true;
}


void BulletSimulatorItem::Impl::store(Archive& archive)
{
    write(archive, "gravity", gravity);
    archive.write("solver", solverType.selectedSymbol());
    archive.write("solver_iterations", numIterations);
    archive.write("contact_erp", contactErp);
    archive.write("position_gain", positionGain);
    archive.write("velocity_gain", velocityGain);
    archive.write("linear_damping", linearDamping);
    archive.write("angular_damping", angularDamping);
    archive.write("collision_margin", collisionMargin);
    archive.write("gimpact_meshes_for_dynamic_objects", useGImpactForDynamicMeshes);
    archive.write("internal_edge_smoothing", isInternalEdgeSmoothingEnabled);
    archive.write("self_collision", isSelfCollisionEnabledByDefault);
    archive.write("rotor_inertia", isRotorInertiaEnabled);
    archive.write("default_rotor_gear_ratio", defaultRotorGearRatio);
    archive.write("velocity_output", isVelocityOutputEnabled);
    archive.write("acceleration_output", isAccelerationOutputEnabled);
    archive.write("drive_effort_output", isDriveEffortOutputEnabled);
}


bool BulletSimulatorItem::restore(const Archive& archive)
{
    SimulatorItem::restore(archive);
    impl->restore(archive);
    return true;
}


void BulletSimulatorItem::Impl::restore(const Archive& archive)
{
    read(archive, "gravity", gravity);
    string symbol;
    if(archive.read("solver", symbol)){
        solverType.select(symbol);
    }
    archive.read("solver_iterations", numIterations);
    archive.read("contact_erp", contactErp);
    archive.read("position_gain", positionGain);
    archive.read("velocity_gain", velocityGain);
    archive.read("linear_damping", linearDamping);
    archive.read("angular_damping", angularDamping);
    archive.read("collision_margin", collisionMargin);
    archive.read("gimpact_meshes_for_dynamic_objects", useGImpactForDynamicMeshes);
    archive.read("internal_edge_smoothing", isInternalEdgeSmoothingEnabled);
    archive.read("self_collision", isSelfCollisionEnabledByDefault);
    archive.read("rotor_inertia", isRotorInertiaEnabled);
    archive.read("default_rotor_gear_ratio", defaultRotorGearRatio);
    archive.read("velocity_output", isVelocityOutputEnabled);
    archive.read("acceleration_output", isAccelerationOutputEnabled);
    archive.read("drive_effort_output", isDriveEffortOutputEnabled);
}
