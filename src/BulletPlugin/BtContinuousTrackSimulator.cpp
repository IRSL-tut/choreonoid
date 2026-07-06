#include "BtContinuousTrackSimulator.h"
#include "BulletSimulatorItemImpl.h"
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/Material>
#include <cnoid/SceneDrawables>
#include <cnoid/MessageOut>
#include <cnoid/Format>
#include <BulletDynamics/Featherstone/btMultiBodyGearConstraint.h>
#include <cmath>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

// The gear constraints only realize velocity-level coupling, which is solved
// exactly in every step, so a large impulse bound is sufficient. Note that
// btMultiBodyGearConstraint does not generate any constraint row when the
// max applied impulse is zero.
constexpr double GearConstraintMaxImpulse = 1.0e8;

}


BtContinuousTrackHandler::BtContinuousTrackHandler(BtContinuousTrack* device)
    : device_(device),
      trackLink_(nullptr),
      sprocketLink_(nullptr),
      idlerLink_(nullptr),
      bulletBody_(nullptr),
      unit_(nullptr),
      multiBody_(nullptr),
      trackBulletLink_(nullptr),
      sprocketBulletLink_(nullptr),
      idlerBulletLink_(nullptr),
      trackMaterialId_(0),
      driveMotor_(nullptr),
      driveVelocityGain_(1.0),
      sprocketGearConstraint_(nullptr),
      idlerGearConstraint_(nullptr),
      upperSegmentGearConstraint_(nullptr)
{
    for(LinearSegment* segment : { &upperSegment_, &lowerSegment_ }){
        segment->mbIndex = -1;
        segment->direction = 1.0;
        segment->localOrigin.setZero();
        segment->slideAxis = Vector3::UnitX();
        segment->length = 0.0;
        segment->numGrousers = 0;
        segment->plateChildIndex = -1;
    }
    lowerSegment_.direction = -1.0;
    for(WheelGrouserSet* wheelGrousers : { &sprocketGrousers_, &idlerGrousers_ }){
        wheelGrousers->wheelBulletLink = nullptr;
        wheelGrousers->wheelRadius = 0.0;
        wheelGrousers->firstGrouserChildIndex = -1;
        wheelGrousers->numPhysicsGrousers = 0;
        wheelGrousers->angleStep = 0.0;
    }
}


BtContinuousTrackHandler::~BtContinuousTrackHandler()
{
    // The multibody links, colliders, shapes, and constraints are owned by
    // the simulator item implementation.
}


bool BtContinuousTrackHandler::resolveLinks(BulletBody* bulletBody)
{
    bulletBody_ = bulletBody;

    trackLink_ = device_->link();
    if(!trackLink_){
        return false;
    }
    if(!device_->isGeometryInitialized()){
        return false;
    }

    Body* body = bulletBody->body();
    sprocketLink_ = body->link(device_->sprocketName());
    idlerLink_ = body->link(device_->idlerName());
    if(!sprocketLink_ || !idlerLink_){
        return false;
    }
    if(!sprocketLink_->isRevoluteJoint() || !idlerLink_->isRevoluteJoint()){
        return false;
    }

    trackMaterialId_ = trackLink_->materialId();
    if(!device_->contactMaterialName().empty()){
        trackMaterialId_ = Material::idOfName(device_->contactMaterialName());
    }

    return true;
}


bool BtContinuousTrackHandler::setupSegmentLinks(BulletUnit* unit, int& mbIndex)
{
    unit_ = unit;
    multiBody_ = unit->multiBody;

    trackBulletLink_ = bulletBody_->bulletLinks[trackLink_->index()];
    sprocketBulletLink_ = bulletBody_->bulletLinks[sprocketLink_->index()];
    idlerBulletLink_ = bulletBody_->bulletLinks[idlerLink_->index()];

    if(!trackBulletLink_ || trackBulletLink_->isStaticLink() ||
       !sprocketBulletLink_ || sprocketBulletLink_->mbIndex < 0 ||
       !idlerBulletLink_ || idlerBulletLink_->mbIndex < 0){
        // The reserved link slots must still be initialized to keep the
        // multibody consistent; fill them with massless fixed links.
        for(int i = 0; i < 2; ++i){
            multiBody_->setupFixed(
                mbIndex, 0.0, btVector3(0, 0, 0), trackBulletLink_ ? trackBulletLink_->mbIndex : -1,
                btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0), btVector3(0, 0, 0));
            ++mbIndex;
        }
        multiBody_ = nullptr;
        return false;
    }

    auto& spec = device_->spec();
    double totalSegmentMass = spec.mass;
    if(totalSegmentMass <= 0.0){
        totalSegmentMass = sprocketLink_->mass() + idlerLink_->mass();
    }
    double segmentMass = totalSegmentMass / 2.0;

    upperSegment_.localOrigin = spec.upperSegmentCenter;
    lowerSegment_.localOrigin = spec.lowerSegmentCenter;
    for(LinearSegment* segment : { &upperSegment_, &lowerSegment_ }){
        segment->slideAxis = spec.slideDir;
        segment->length = spec.segmentLength;
        setupSegmentLink(*segment, mbIndex, segmentMass);
        ++mbIndex;
    }

    return true;
}


bool BtContinuousTrackHandler::setupSegmentLink(
    LinearSegment& segment, int mbIndex, double segmentMass)
{
    auto& spec = device_->spec();
    segment.mbIndex = mbIndex;

    // Box inertia. The box length axis is assumed to be roughly aligned with
    // the slide axis, like the PhysX implementation.
    double w = segment.length;
    double h = spec.width;
    double d = spec.thickness;
    double m12 = segmentMass / 12.0;
    Vector3 inertia(m12 * (h * h + d * d), m12 * (w * w + d * d), m12 * (w * w + h * h));

    /*
       The segment link frame has the same orientation as the track link frame
       and its origin, which is also the center of mass and the principal axes
       frame origin, is at localOrigin when the joint position is zero. The
       parent frame on the Bullet side is the principal axes frame of the
       track link, so the frame values given to setupPrismatic are derived
       in the same way as the regular links in BulletBody::createUnit with
       R_iL = I, Tb.linear() = I, and Tb.translation() = localOrigin.
    */
    const Matrix3 R_iP = trackBulletLink_->T_inertial.linear();
    const Vector3& c_P = trackLink_->c();

    btQuaternion rotParentToThis = toBtQuaternion(R_iP);
    btVector3 jointAxis = toBtVector3(segment.slideAxis);
    btVector3 parentComToThisPivotOffset = toBtVector3(
        R_iP.transpose() * (segment.localOrigin - c_P));
    btVector3 thisPivotToThisComOffset(0, 0, 0);

    multiBody_->setupPrismatic(
        mbIndex, static_cast<btScalar>(segmentMass), toBtVector3(inertia),
        trackBulletLink_->mbIndex, rotParentToThis, jointAxis,
        parentComToThisPivotOffset, thisPivotToThisComOffset, true);

    return true;
}


bool BtContinuousTrackHandler::completeSetup()
{
    if(!multiBody_){
        return false;
    }

    auto simImpl = bulletBody_->simImpl;
    auto& spec = device_->spec();

    /*
       Override the contact material of the wheels with that of the track
       device. The wheels carry the grouser and belt shapes and Bullet cannot
       assign contact parameters per shape, so the whole wheel collider gets
       the track material. This is acceptable because the original wheel
       surface is covered by the belt shapes.
    */
    for(BulletLink* wheelBulletLink : { sprocketBulletLink_, idlerBulletLink_ }){
        if(wheelBulletLink->materialId != trackMaterialId_){
            wheelBulletLink->materialId = trackMaterialId_;
            simImpl->applyLinkMaterial(wheelBulletLink->collider, wheelBulletLink);
        }
    }

    sprocketGrousers_.wheelBulletLink = sprocketBulletLink_;
    idlerGrousers_.wheelBulletLink = idlerBulletLink_;
    addWheelShapes(sprocketGrousers_, spec.sprocketRadius, spec.sprocketBeltSegments);
    addWheelShapes(idlerGrousers_, spec.idlerRadius, spec.idlerBeltSegments);

    createSegmentShapesAndCollider(upperSegment_);
    createSegmentShapesAndCollider(lowerSegment_);

    // Velocity drive on the lower segment (the master joint of the track).
    // The drive_damping value of the device is a stiffness-type gain of the
    // PhysX implementation and has a different meaning here; the velocity
    // error reduction gain of the simulator item is used instead.
    double maxImpulse = UnlimitedMotorImpulse;
    if(spec.driveMaxForce > 0.0){
        maxImpulse = spec.driveMaxForce * simImpl->timeStep;
    }
    driveVelocityGain_ = simImpl->velocityGain;
    driveMotor_ = new btMultiBodyJointMotor(
        multiBody_, lowerSegment_.mbIndex, 0.0, static_cast<btScalar>(maxImpulse));
    driveMotor_->setVelocityTarget(0.0, static_cast<btScalar>(driveVelocityGain_));
    simImpl->addMultiBodyConstraint(driveMotor_);

    /*
       Gear constraints synchronizing the sprocket, the idler, and the upper
       segment with the lower segment. The constraint equation of
       btMultiBodyGearConstraint is velA + gearRatio * velB = 0, so with
       A = lower segment and ratio = effective wheel radius, the wheel
       angular velocity becomes -v_lower / radius, which together with the
       lower segment drive velocity dq_target * radius * direction (with
       direction = -1) makes the wheels rotate at dq_target.
    */
    auto createGearConstraint = [&](int slaveMbIndex, double gearRatio){
        auto gear = new btMultiBodyGearConstraint(
            multiBody_, lowerSegment_.mbIndex, multiBody_, slaveMbIndex,
            btVector3(0, 0, 0), btVector3(0, 0, 0),
            btMatrix3x3::getIdentity(), btMatrix3x3::getIdentity());
        gear->setGearRatio(static_cast<btScalar>(gearRatio));
        gear->setMaxAppliedImpulse(static_cast<btScalar>(GearConstraintMaxImpulse));
        simImpl->addMultiBodyConstraint(gear);
        return gear;
    };
    sprocketGearConstraint_ = createGearConstraint(
        sprocketBulletLink_->mbIndex, spec.effectiveSprocketRadius);
    idlerGearConstraint_ = createGearConstraint(
        idlerBulletLink_->mbIndex, spec.effectiveIdlerRadius);
    upperSegmentGearConstraint_ = createGearConstraint(upperSegment_.mbIndex, 1.0);

    return true;
}


void BtContinuousTrackHandler::addWheelShapes(
    WheelGrouserSet& wheelGrousers, double wheelRadius, int numBeltSegments)
{
    auto simImpl = bulletBody_->simImpl;
    auto& spec = device_->spec();
    BulletLink* wheelBulletLink = wheelGrousers.wheelBulletLink;
    btCompoundShape* compound = wheelBulletLink->compound;

    double shoeHeight = spec.grouserHeight;
    double thickness = spec.thickness;
    double width = spec.width;

    // Number of grousers around the wheel (same formula as the PhysX
    // implementation and the shoe positions computed by the device)
    double circumference = 2.0 * M_PI * (wheelRadius + thickness);
    int numGrousers = static_cast<int>(round(circumference / spec.grouserSpacing));
    if(numGrousers < 4) numGrousers = 4;
    double angleStep = 2.0 * M_PI / numGrousers;

    wheelGrousers.wheelRadius = wheelRadius;
    wheelGrousers.firstGrouserChildIndex = compound->getNumChildShapes();
    wheelGrousers.numPhysicsGrousers = numGrousers;
    wheelGrousers.angleStep = angleStep;

    const bool wasEmpty = (compound->getNumChildShapes() == 0);
    const Isometry3& T_inv = wheelBulletLink->T_inertialInv;

    double shoeThickness = device_->effectiveGrouserThickness();
    auto grouserShape = new btBoxShape(
        btVector3(
            static_cast<btScalar>(shoeThickness / 2.0),
            static_cast<btScalar>(width / 2.0),
            static_cast<btScalar>(shoeHeight / 2.0)));
    simImpl->ownedShapes.emplace_back(grouserShape);

    for(int i = 0; i < numGrousers; ++i){
        double angle = M_PI / 2.0 + i * angleStep;
        double r = wheelRadius + thickness + shoeHeight / 2.0;
        Isometry3 localPose;
        localPose.linear() = AngleAxis(M_PI / 2.0 - angle, Vector3::UnitY()).toRotationMatrix();
        localPose.translation() = Vector3(r * cos(angle), 0.0, r * sin(angle));
        compound->addChildShape(toBtTransform(T_inv * localPose), grouserShape);
    }

    // Belt shapes approximating the circular band between the grousers
    double beltAngleStep = 2.0 * M_PI / numBeltSegments;
    double r_belt = wheelRadius + thickness / 2.0;
    double r_outer = wheelRadius + thickness;
    double chordLength = 2.0 * r_outer * sin(beltAngleStep / 2.0);
    auto beltShape = new btBoxShape(
        btVector3(
            static_cast<btScalar>(chordLength / 2.0),
            static_cast<btScalar>(width / 2.0),
            static_cast<btScalar>(thickness / 2.0)));
    simImpl->ownedShapes.emplace_back(beltShape);

    for(int i = 0; i < numBeltSegments; ++i){
        double angle = M_PI / 2.0 + i * beltAngleStep;
        Isometry3 localPose;
        localPose.linear() = AngleAxis(M_PI / 2.0 - angle, Vector3::UnitY()).toRotationMatrix();
        localPose.translation() = Vector3(r_belt * cos(angle), 0.0, r_belt * sin(angle));
        compound->addChildShape(toBtTransform(T_inv * localPose), beltShape);
    }

    // A wheel link without collision shapes has not been added to the world
    if(wasEmpty && !wheelBulletLink->collider->getBroadphaseHandle()){
        simImpl->world->addCollisionObject(
            wheelBulletLink->collider,
            btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
    }
}


void BtContinuousTrackHandler::createSegmentShapesAndCollider(LinearSegment& segment)
{
    auto simImpl = bulletBody_->simImpl;
    auto& spec = device_->spec();

    /*
       The internal BulletLink of the segment. The link pointer refers to the
       track link so that the callbacks depending on it stay safe, but the
       link is marked as a track internal link, which never collides with
       the other links of the same body.
    */
    auto bulletLink = new BulletLink(trackLink_, bulletBody_);
    segment.bulletLink = bulletLink;
    bulletLink->unit = unit_;
    bulletLink->mbIndex = segment.mbIndex;
    bulletLink->isTrackInternal = true;
    bulletLink->useSurfaceVelocity = false;
    bulletLink->materialId = trackMaterialId_;
    // T_inertial stays identity: the segment link frame is the principal
    // axes frame at the center of mass by construction.

    auto compound = new btCompoundShape;
    bulletLink->compound = compound;
    simImpl->ownedShapes.emplace_back(compound);

    double thickness = spec.thickness;
    double width = spec.width;
    double shoeHeight = spec.grouserHeight;

    // Plate shape
    auto plateShape = new btBoxShape(
        btVector3(
            static_cast<btScalar>(segment.length / 2.0),
            static_cast<btScalar>(width / 2.0),
            static_cast<btScalar>(thickness / 2.0)));
    simImpl->ownedShapes.emplace_back(plateShape);
    segment.plateChildIndex = compound->getNumChildShapes();
    compound->addChildShape(
        btTransform(
            btQuaternion::getIdentity(),
            btVector3(0, 0, static_cast<btScalar>(segment.direction * thickness / 2.0))),
        plateShape);

    // Count grousers on this segment (same logic as the device clearState)
    int totalGrousers = 0;
    {
        double x = segment.length / 2.0 - spec.grouserSpacing;
        while(x >= -segment.length / 2.0 - spec.grouserSpacing * 0.01){
            totalGrousers++;
            x -= spec.grouserSpacing;
        }
    }
    segment.numGrousers = totalGrousers;

    // Grouser shapes
    double grouserDir = (segment.direction > 0) ? 1.0 : -1.0;
    double shoeThickness = device_->effectiveGrouserThickness();
    auto grouserShape = new btBoxShape(
        btVector3(
            static_cast<btScalar>(shoeThickness / 2.0),
            static_cast<btScalar>(width / 2.0),
            static_cast<btScalar>(shoeHeight / 2.0)));
    simImpl->ownedShapes.emplace_back(grouserShape);

    for(int i = 0; i < totalGrousers; ++i){
        double x = segment.length / 2.0 - spec.grouserSpacing - i * spec.grouserSpacing;
        // For the upper segment, move the last grouser (idler end) to the
        // front (sprocket end)
        if(segment.direction > 0 && i == totalGrousers - 1){
            x = segment.length / 2.0;
        }
        Vector3 p = segment.slideAxis * x;
        p.z() += grouserDir * (thickness + shoeHeight / 2.0);
        compound->addChildShape(
            btTransform(btQuaternion::getIdentity(), toBtVector3(p)), grouserShape);
    }

    // Collider
    auto collider = new btMultiBodyLinkCollider(multiBody_, segment.mbIndex);
    bulletLink->collider = collider;
    collider->setCollisionShape(compound);
    Isometry3 segmentT;
    segmentT.linear() = trackLink_->T().linear();
    segmentT.translation() = trackLink_->T() * segment.localOrigin;
    collider->setWorldTransform(toBtTransform(segmentT));
    collider->setActivationState(DISABLE_DEACTIVATION);
    simImpl->applyLinkMaterial(collider, bulletLink);
    simImpl->collisionObjects.emplace_back(collider);
    simImpl->world->addCollisionObject(
        collider, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
    multiBody_->getLink(segment.mbIndex).m_collider = collider;
}


bool BtContinuousTrackHandler::resetAllJointPositions()
{
    double pos = multiBody_->getJointPos(lowerSegment_.mbIndex);
    double spacing = device_->spec().grouserSpacing;

    if(pos >= 0.0 && pos < spacing){
        return false;
    }

    auto resetToRange = [](double val, double period) -> double {
        double r = fmod(val, period);
        if(r < 0.0) r += period;
        return r;
    };

    // Reset the lower segment (master)
    double lowerNew = resetToRange(pos, spacing);
    multiBody_->setJointPos(lowerSegment_.mbIndex, static_cast<btScalar>(lowerNew));

    /*
       Reset the follower joints, preserving the current constraint error
       (posA + ratio * posB). The gear constraints are currently velocity
       level only, but the error preservation keeps this valid if a position
       level correction (ERP) is enabled in the future.
    */
    // Upper segment (gear ratio 1)
    {
        double upperPos = multiBody_->getJointPos(upperSegment_.mbIndex);
        double errBefore = pos + upperPos;
        double upperFmod = resetToRange(upperPos, spacing);
        double errAfterFmod = lowerNew + upperFmod;
        double upperNew = upperFmod + (errBefore - errAfterFmod);
        multiBody_->setJointPos(upperSegment_.mbIndex, static_cast<btScalar>(upperNew));
    }

    // Wheels
    auto resetWheel = [&](BulletLink* wheelBulletLink, double gearRatio){
        double wheelPos = multiBody_->getJointPos(wheelBulletLink->mbIndex);
        double anglePeriod = spacing / gearRatio;
        double errBefore = pos + gearRatio * wheelPos;
        double wheelFmod = resetToRange(wheelPos, anglePeriod);
        double errAfterFmod = lowerNew + gearRatio * wheelFmod;
        double wheelNew = wheelFmod + (errBefore - errAfterFmod) / gearRatio;
        multiBody_->setJointPos(wheelBulletLink->mbIndex, static_cast<btScalar>(wheelNew));
    };
    resetWheel(sprocketBulletLink_, device_->spec().effectiveSprocketRadius);
    resetWheel(idlerBulletLink_, device_->spec().effectiveIdlerRadius);

    return true;
}


void BtContinuousTrackHandler::updateSimulation()
{
    if(!multiBody_ || !driveMotor_){
        return;
    }

    auto& spec = device_->spec();

    // dq_target is the commanded sprocket angular velocity (rad/s);
    // convert it to the linear velocity of the lower segment
    double dq = sprocketLink_->dq_target();
    double segmentVelocity = dq * spec.effectiveSprocketRadius * lowerSegment_.direction;
    driveMotor_->setVelocityTarget(
        static_cast<btScalar>(segmentVelocity), static_cast<btScalar>(driveVelocityGain_));

    // Synchronized phase reset of all the coupled joints
    if(resetAllJointPositions()){
        multiBody_->updateCollisionObjectWorldTransforms(scratchQ_, scratchM_);
    }

    /*
       Compensate the plate positions for the segment sliding so that the
       plates appear stationary while only the grousers move. The child
       transform update also refreshes the AABB of the compound shape.
    */
    for(LinearSegment* segment : { &upperSegment_, &lowerSegment_ }){
        double pos = multiBody_->getJointPos(segment->mbIndex);
        Vector3 p = -segment->slideAxis * pos;
        p.z() += segment->direction * spec.thickness / 2.0;
        segment->bulletLink->compound->updateChildTransform(
            segment->plateChildIndex,
            btTransform(btQuaternion::getIdentity(), toBtVector3(p)),
            true);
    }
}


void BtContinuousTrackHandler::addSegmentShoePositions(
    const LinearSegment& segment, const Isometry3& trackT_inv)
{
    Isometry3 T_segment = toIsometry3(segment.bulletLink->collider->getWorldTransform());
    btCompoundShape* compound = segment.bulletLink->compound;
    const int n = compound->getNumChildShapes();

    // Skip the plate shape; add the grouser shapes converted to the track
    // link local frame
    for(int i = segment.plateChildIndex + 1; i < n; ++i){
        Isometry3 localPose = toIsometry3(compound->getChildTransform(i));
        device_->addShoePosition(SE3(trackT_inv * T_segment * localPose));
    }
}


void BtContinuousTrackHandler::addWheelShoePositions(
    const WheelGrouserSet& wheelGrousers, bool isOuterPositive, const Isometry3& trackT_inv)
{
    BulletLink* wheelBulletLink = wheelGrousers.wheelBulletLink;
    Isometry3 wheelT = toIsometry3(wheelBulletLink->collider->getWorldTransform());
    btCompoundShape* compound = wheelBulletLink->compound;

    double phi = multiBody_->getJointPos(wheelBulletLink->mbIndex);

    for(int i = 0; i < wheelGrousers.numPhysicsGrousers; ++i){
        double theta = M_PI / 2.0 + i * wheelGrousers.angleStep - phi;
        double c = cos(theta);
        if(isOuterPositive ? (c >= -0.01) : (c <= 0.01)){
            Isometry3 localPose = toIsometry3(
                compound->getChildTransform(wheelGrousers.firstGrouserChildIndex + i));
            device_->addShoePosition(SE3(trackT_inv * wheelT * localPose));
        }
    }
}


void BtContinuousTrackHandler::updateTrackStates()
{
    if(!multiBody_ || !trackLink_ || device_->numVisibleShoes() <= 0){
        return;
    }

    Isometry3 trackT_inv = trackLink_->T().inverse();

    device_->clearShoePositions();
    device_->shoePositions().reserve(device_->spec().maxNumVisibleShoes);

    if(upperSegment_.bulletLink){
        addSegmentShoePositions(upperSegment_, trackT_inv);
    }
    if(lowerSegment_.bulletLink){
        addSegmentShoePositions(lowerSegment_, trackT_inv);
    }
    addWheelShoePositions(sprocketGrousers_, true, trackT_inv);
    addWheelShoePositions(idlerGrousers_, false, trackT_inv);

    device_->notifyStateChange();
}


BtContinuousTrackSimulator::BtContinuousTrackSimulator()
{

}


BtContinuousTrackSimulator::~BtContinuousTrackSimulator()
{

}


std::vector<BtContinuousTrackHandler*> BtContinuousTrackSimulator::prepareHandlersForUnit(
    BulletBody* bulletBody, const std::vector<Link*>& unitLinks)
{
    std::vector<BtContinuousTrackHandler*> unitHandlers;

    Body* body = bulletBody->body();
    auto devices = body->devices<BtContinuousTrack>();
    if(devices.empty()){
        return unitHandlers;
    }

    std::unordered_set<Link*> unitLinkSet(unitLinks.begin(), unitLinks.end());

    for(auto& device : devices){
        Link* trackLink = device->link();
        if(!trackLink || !unitLinkSet.count(trackLink)){
            continue; // the device belongs to another unit
        }
        auto handler = std::make_unique<BtContinuousTrackHandler>(device);
        if(!handler->resolveLinks(bulletBody) ||
           !unitLinkSet.count(handler->sprocketLink()) ||
           !unitLinkSet.count(handler->idlerLink())){
            MessageOut::master()->putWarningln(
                formatR(_("BtContinuousTrack \"{0}\" of {1} is ignored because its "
                          "links are invalid or not simulated as a single multibody."),
                        device->name(), body->name()));
            continue;
        }
        trackDrivenJoints_.insert(handler->sprocketLink());
        trackDrivenJoints_.insert(handler->idlerLink());
        unitHandlers.push_back(handler.get());
        handlers_.push_back(std::move(handler));
    }

    return unitHandlers;
}


bool BtContinuousTrackSimulator::isTrackDrivenJoint(Link* link) const
{
    return trackDrivenJoints_.count(link) > 0;
}


void BtContinuousTrackSimulator::initializeTrackStates()
{
    for(auto& handler : handlers_){
        handler->updateTrackStates();
    }
}


void BtContinuousTrackSimulator::updateSimulation()
{
    for(auto& handler : handlers_){
        handler->updateSimulation();
    }
}


void BtContinuousTrackSimulator::updateTrackStates()
{
    for(auto& handler : handlers_){
        handler->updateTrackStates();
    }
}
