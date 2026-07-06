#ifndef CNOID_BULLET_PLUGIN_BULLET_SIMULATOR_ITEM_IMPL_H
#define CNOID_BULLET_PLUGIN_BULLET_SIMULATOR_ITEM_IMPL_H

#include "BulletSimulatorItem.h"
#include <cnoid/Link>
#include <cnoid/BasicSensorSimulationHelper>
#include <cnoid/BodyCollisionLinkFilter>
#include <cnoid/ForceSensor>
#include <cnoid/MeshExtractor>
#include <cnoid/Selection>
#include <cnoid/IdPair>
#include <cnoid/EigenTypes>
#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointMotor.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointFeedback.h>
#include <BulletDynamics/MLCPSolvers/btMLCPSolverInterface.h>
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <memory>

namespace cnoid {

class BulletLink;
class BulletUnit;
class BulletBody;
class BtContinuousTrackSimulator;
class MessageOut;
class SgMesh;
class MaterialTable;

typedef ref_ptr<BulletLink> BulletLinkPtr;
typedef ref_ptr<BulletUnit> BulletUnitPtr;

// A large but finite impulse bound used when no explicit limit is given.
// btMultiBodyConstraint clamps the applied impulse per step by this value.
constexpr double UnlimitedMotorImpulse = 1.0e8;
constexpr double ExtraJointMaxImpulse = 1.0e8;

// Resolved joint control law, derived from the link's actuation mode bitmask.
enum ControlType { NoControl, EffortControl, PositionControl, VelocityControl };

inline btVector3 toBtVector3(const Vector3& v)
{
    return btVector3(
        static_cast<btScalar>(v.x()),
        static_cast<btScalar>(v.y()),
        static_cast<btScalar>(v.z()));
}

inline Vector3 toEigenVector3(const btVector3& v)
{
    return Vector3(v.x(), v.y(), v.z());
}

inline btQuaternion toBtQuaternion(const Matrix3& R)
{
    Quaternion q(R);
    q.normalize();
    return btQuaternion(
        static_cast<btScalar>(q.x()),
        static_cast<btScalar>(q.y()),
        static_cast<btScalar>(q.z()),
        static_cast<btScalar>(q.w()));
}

inline btMatrix3x3 toBtMatrix3x3(const Matrix3& R)
{
    return btMatrix3x3(
        static_cast<btScalar>(R(0,0)), static_cast<btScalar>(R(0,1)), static_cast<btScalar>(R(0,2)),
        static_cast<btScalar>(R(1,0)), static_cast<btScalar>(R(1,1)), static_cast<btScalar>(R(1,2)),
        static_cast<btScalar>(R(2,0)), static_cast<btScalar>(R(2,1)), static_cast<btScalar>(R(2,2)));
}

inline btTransform toBtTransform(const Isometry3& T)
{
    return btTransform(toBtQuaternion(T.linear()), toBtVector3(T.translation()));
}

inline Isometry3 toIsometry3(const btTransform& T)
{
    const btQuaternion q = T.getRotation();
    Isometry3 T2;
    T2.linear() = Quaternion(q.w(), q.x(), q.y(), q.z()).toRotationMatrix();
    T2.translation() = toEigenVector3(T.getOrigin());
    return T2;
}


struct BulletForceSensorInfo
{
    ForceSensorPtr sensor;
    BulletLink* bulletLink;
};

/*
   Per-link bridge between a Choreonoid Link and the Bullet objects.
   A link is either a static rigid body (static links), the base of a
   btMultiBody, or an internal link of a btMultiBody. The internal linear
   segment links of a continuous track also have a BulletLink whose link
   pointer refers to the track base link and whose isTrackInternal flag
   is set.
*/
class BulletLink : public Referenced
{
public:
    BulletBody* bulletBody;
    Link* link;
    BulletUnit* unit;

    // -1: base of the unit's multibody, >= 0: multibody link index,
    // -2: static rigid body or not part of a multibody,
    // -3: dynamic rigid body (a single free-floating link without joints)
    int mbIndex;

    // The rigid body of a static link or a dynamic single free link
    btRigidBody* rigidBody;
    btMultiBodyLinkCollider* collider;
    btCompoundShape* compound;

    // The principal axes frame at the center of mass (the Bullet link frame)
    Isometry3 T_inertial;
    Isometry3 T_inertialInv;
    Vector3 principalInertia;

    // The material id used for the contact parameters. This is usually the
    // material id of the link, but the internal links of a continuous track
    // override it with the contact material of the track device.
    int materialId;

    // True for the internal links of a continuous track (linear segments).
    // They never collide with the links of the same body.
    bool isTrackInternal;

    ControlType controlType;
    bool velocityFeedforward;
    bool useSurfaceVelocity;
    bool hasTriangleMeshShape;
    bool hasAccelSensor;
    double kp;
    double kv;
    btMultiBodyJointMotor* motor;
    double lastJointEffort;

    // Multibody link index of the virtual rotor link realizing the equivalent
    // rotor inertia of the joint, or -1 when no rotor model is attached
    int rotorMbIndex;
    double rotorGearRatio;
    std::unique_ptr<btMultiBodyJointFeedback> jointFeedback;

    // Previous velocities for the finite difference acceleration output
    Vector3 prevV;
    Vector3 prevW;

    BulletLink(Link* link, BulletBody* bulletBody);
    void initInertialFrame();
    bool isStaticLink() const { return mbIndex == -2; }
    bool isDynamicRigidBodyLink() const { return mbIndex == -3; }
    bool isRigidBodyLink() const { return mbIndex <= -2; }
};


// One btMultiBody instance corresponding to a connected subtree of links
class BulletUnit : public Referenced
{
public:
    btMultiBody* multiBody;
    bool hasDummyBase;
    BulletLink* baseLink; // null if the base is a dummy fixed base
    std::vector<BulletLink*> mbLinks; // indexed by the multibody link index

    BulletUnit() : multiBody(nullptr), hasDummyBase(false), baseLink(nullptr) { }
};


class BulletBody : public SimulationBody
{
public:
    BulletSimulatorItem::Impl* simImpl;
    int bodyIndex;
    std::vector<BulletLinkPtr> bulletLinks; // indexed by link->index()
    std::vector<BulletUnitPtr> units;
    std::vector<BulletLink*> rigidBodyLinks; // dynamic rigid body links
    std::vector<BulletLink*> controlLinks;
    BasicSensorSimulationHelper sensorHelper;
    std::vector<BulletForceSensorInfo> forceSensorInfos;
    BodyCollisionLinkFilter bodyCollisionLinkFilter;
    bool selfCollisionDetectionEnabled;
    bool hasForceSensors;
    bool needsSensorVelocities;

    // Joints whose rotor models use the default gear ratio because the model
    // does not provide the gear_ratio information
    std::vector<std::string> rotorDefaultGearRatioJointNames;

    BulletBody(Body* body, BulletSimulatorItem::Impl* simImpl);
    void createBulletObjects();
    void createLinkTree(Link* link);
    void createStaticLink(Link* link);
    void createRigidBodyLink(Link* link);
    void createUnit(Link* unitRoot);
    void collectUnitLinks(Link* link, std::vector<Link*>& unitLinks, std::vector<Link*>& subUnitRoots);
    void setupRotorLink(
        BulletLink* bulletLink, int rotorMbIndex, int parentIndex,
        const Matrix3& R_iP, const Vector3& c_P, const Isometry3& Tb);
    void setupJointControl(BulletLink* bulletLink);
    void initializeSensors();
    BulletLink* getOrCreateBulletLink(Link* link);
    void inputControl();
    void addExternalForces();
    void getState();
    void updateForceSensors();
};


class BulletSimulatorItem::Impl
{
public:
    BulletSimulatorItem* self;

    std::unique_ptr<btDefaultCollisionConfiguration> collisionConfiguration;
    std::unique_ptr<btCollisionDispatcher> dispatcher;
    std::unique_ptr<btDbvtBroadphase> broadphase;
    std::unique_ptr<btMLCPSolverInterface> mlcpInnerSolver;
    std::unique_ptr<btMultiBodyConstraintSolver> solver;
    std::unique_ptr<btMultiBodyDynamicsWorld> world;
    std::unique_ptr<btOverlapFilterCallback> overlapFilter;

    // All the Bullet objects are owned by the impl so that they can be
    // destroyed in a well-defined order in clear().
    std::vector<std::unique_ptr<btMultiBody>> multiBodies;
    std::vector<std::unique_ptr<btMultiBodyConstraint>> multiBodyConstraints;
    // Ordinary rigid body constraints used for the extra joints between
    // rigid body links
    std::vector<std::unique_ptr<btTypedConstraint>> typedConstraints;
    std::vector<std::unique_ptr<btCollisionObject>> collisionObjects;
    std::vector<std::unique_ptr<btCollisionShape>> ownedShapes;
    std::vector<std::unique_ptr<btTriangleIndexVertexArray>> meshInterfaces;
    std::vector<std::unique_ptr<btTriangleInfoMap>> triangleInfoMaps;

    struct MeshBuffers {
        std::vector<btScalar> vertices;
        std::vector<int> indices;
    };
    std::vector<std::unique_ptr<MeshBuffers>> meshBuffers;

    // Shape deduplication. The same SgMesh instance is often shared by multiple
    // links (e.g. identical wheels). The scale is part of the key because it is
    // baked into the shape data.
    typedef std::pair<SgMesh*, std::array<double, 3>> MeshShapeKey;
    std::map<MeshShapeKey, btCollisionShape*> convexShapeMap;
    std::map<MeshShapeKey, btCollisionShape*> triangleMeshShapeMap;
    std::map<MeshShapeKey, btCollisionShape*> gimpactShapeMap;

    std::unordered_map<Body*, BulletBody*> bodyMap;

    MaterialTable* materialTable;

    struct MaterialParams {
        double friction;
        double restitution;
        double stiffness;
        double damping;
    };
    std::map<int, MaterialParams> materialParamsCache;

    // Explicitly defined contact material pairs. They are applied by overriding
    // the combined contact parameters in the contact added callback because the
    // default per-object combining cannot express pair-specific parameters.
    struct ContactPairParam {
        double friction;
        double restitution;
    };
    std::unordered_map<IdPair<int>, ContactPairParam> contactPairParamMap;
    std::unordered_set<int> pairMaterialIds;

    Vector3 gravity;
    double timeStep;
    Selection solverType;
    int numIterations;
    double contactErp;
    double positionGain;
    double velocityGain;
    double linearDamping;
    double angularDamping;
    double collisionMargin;
    bool useGImpactForDynamicMeshes;
    bool isInternalEdgeSmoothingEnabled;
    bool isSelfCollisionEnabledByDefault;
    bool isRotorInertiaEnabled;
    double defaultRotorGearRatio;
    bool isVelocityOutputEnabled;
    bool isAccelerationOutputEnabled;
    bool isDriveEffortOutputEnabled;
    bool hasSurfaceVelocityLinks;

    MeshExtractor meshExtractor;
    MessageOut* mout;

    // Scratch buffers for reading out the link velocities of a multibody
    std::vector<btVector3> omegaBuf;
    std::vector<btVector3> velBuf;

    std::unique_ptr<BtContinuousTrackSimulator> trackSimulator;

    Impl(BulletSimulatorItem* self);
    Impl(BulletSimulatorItem* self, const Impl& org);
    ~Impl();
    void initialize();
    void clear();
    bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    void updateSurfaceVelocityContacts();
    void setupContactMaterials();
    const MaterialParams& getOrCreateMaterialParams(int materialId);
    void applyLinkMaterial(btCollisionObject* object, BulletLink* bulletLink);
    void addMultiBodyConstraint(btMultiBodyConstraint* constraint);
    void addTypedConstraint(btTypedConstraint* constraint);
    BulletLink* findBulletLink(Link* link);
    void addExtraJoints(const std::vector<SimulationBody*>& simBodies);
    void readMeshNode(BulletLink* bulletLink);
    btCollisionShape* createPrimitiveShape(
        SgMesh* mesh, const Vector3& scale);
    btCollisionShape* createConvexHullShape(
        SgMesh* mesh, const Vector3& scale);
    btCollisionShape* createTriangleMeshShape(
        SgMesh* mesh, const Vector3& scale);
    btCollisionShape* createGImpactMeshShape(
        SgMesh* mesh, const Vector3& scale);
    btTriangleIndexVertexArray* createMeshInterface(
        SgMesh* mesh, const Vector3& scale);
    void doPutProperties(PutPropertyFunction& putProperty);
    void store(Archive& archive);
    void restore(const Archive& archive);
};

}

#endif
