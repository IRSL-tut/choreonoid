#ifndef CNOID_BULLET_PLUGIN_BULLET_CONTINUOUS_TRACK_SIMULATOR_IMPL_H
#define CNOID_BULLET_PLUGIN_BULLET_CONTINUOUS_TRACK_SIMULATOR_IMPL_H

#include "BulletContinuousTrackSimulator.h"
#include <cnoid/PiecewiseRigidContinuousTrack>
#include <cnoid/EigenTypes>
#include <LinearMath/btAlignedObjectArray.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>
#include <vector>
#include <memory>
#include <unordered_set>

class btMultiBody;
class btMultiBodyGearConstraint;
class btMultiBodyJointMotor;

namespace cnoid {

class Link;
class BulletLink;
class BulletUnit;
class BulletBody;

/*
   Simulates one PiecewiseRigidContinuousTrack device. The track is approximated by
   grouser and belt shapes fixed to the sprocket and idler wheels plus two
   sliding linear segments (upper and lower) that carry plate and grouser
   shapes. The lower segment is velocity-driven from the sprocket joint
   command and the other joints (sprocket, idler, upper segment) follow it
   through gear constraints. The sliding joints are periodically reset by
   one grouser spacing so that the track appears to run continuously.
*/
class BtContinuousTrackHandler
{
public:
    BtContinuousTrackHandler(PiecewiseRigidContinuousTrack* device);
    ~BtContinuousTrackHandler();

    PiecewiseRigidContinuousTrack* device() const { return device_; }
    Link* trackLink() const { return trackLink_; }
    Link* sprocketLink() const { return sprocketLink_; }
    Link* idlerLink() const { return idlerLink_; }

    // Resolve the links of the device. Returns false if the device cannot
    // be simulated.
    bool resolveLinks(BulletBody* bulletBody);

    // Add the prismatic segment links to the unit's multibody. Must be called
    // after the Choreonoid links of the unit have been set up and before
    // finalizeMultiDof. mbIndex is advanced by the number of added links.
    bool setupSegmentLinks(BulletUnit* unit, int& mbIndex);

    // Create everything that can be created after the multibody has been
    // finalized and the colliders of the Choreonoid links have been created:
    // the grouser and belt shapes on the wheels, the segment colliders, the
    // gear constraints, and the drive motor.
    bool completeSetup();

    // Called before every simulation step
    void updateSimulation();

    // Called after every simulation step and at the simulation start
    void updateTrackStates();

private:
    struct LinearSegment {
        int mbIndex;
        ref_ptr<BulletLink> bulletLink;
        // Direction: +1 for upper, -1 for lower
        double direction;
        // Offset from the track link origin to the segment center at q = 0
        // (in the track link local frame)
        Vector3 localOrigin;
        // Sliding axis in the track link local frame
        Vector3 slideAxis;
        double length;
        int numGrousers;
        int plateChildIndex;
    };

    struct WheelGrouserSet {
        BulletLink* wheelBulletLink;
        double wheelRadius;
        int firstGrouserChildIndex; // compound child index of the first grouser
        int numPhysicsGrousers;     // full-circle count (shapes on the wheel)
        double angleStep;           // 2*PI/numPhysicsGrousers
    };

    PiecewiseRigidContinuousTrack* device_;
    Link* trackLink_;
    Link* sprocketLink_;
    Link* idlerLink_;

    BulletBody* bulletBody_;
    BulletUnit* unit_;
    btMultiBody* multiBody_;
    BulletLink* trackBulletLink_;
    BulletLink* sprocketBulletLink_;
    BulletLink* idlerBulletLink_;

    int trackMaterialId_;

    LinearSegment upperSegment_;
    LinearSegment lowerSegment_;
    WheelGrouserSet sprocketGrousers_;
    WheelGrouserSet idlerGrousers_;

    btMultiBodyJointMotor* driveMotor_;
    double driveVelocityGain_;
    btMultiBodyGearConstraint* sprocketGearConstraint_;
    btMultiBodyGearConstraint* idlerGearConstraint_;
    btMultiBodyGearConstraint* upperSegmentGearConstraint_;

    // Scratch buffers for updateCollisionObjectWorldTransforms
    btAlignedObjectArray<btQuaternion> scratchQ_;
    btAlignedObjectArray<btVector3> scratchM_;

    bool setupSegmentLink(LinearSegment& segment, int mbIndex, double segmentMass);
    void createSegmentShapesAndCollider(LinearSegment& segment);
    void addWheelShapes(WheelGrouserSet& wheelGrousers, double wheelRadius, int numBeltSegments);
    bool resetAllJointPositions();
    void addSegmentShoePositions(const LinearSegment& segment, const Isometry3& trackT_inv);
    void addWheelShoePositions(
        const WheelGrouserSet& wheelGrousers, bool isOuterPositive, const Isometry3& trackT_inv);
};


class BulletContinuousTrackSimulatorImpl : public BulletContinuousTrackSimulator
{
public:
    BulletContinuousTrackSimulatorImpl();
    ~BulletContinuousTrackSimulatorImpl() override;

    // Create the handlers for the PiecewiseRigidContinuousTrack devices whose links are
    // all contained in the given unit. Called from BulletBody::createUnit
    // before the multibody is created. The returned pointers are owned by
    // this simulator.
    std::unique_ptr<BulletContinuousTrackUnitSetup> prepareUnit(
        BulletBody* bulletBody, const std::vector<Link*>& unitLinks) override;
    bool empty() const override;

    // True if the joint of the link is driven by a track handler (the wheel
    // joints). The regular joint motor must not be created for such joints.
    bool isTrackDrivenJoint(Link* link) const override;

    // Generate the initial track states for all the handlers
    void initializeTrackStates() override;

    // Update the drives and the periodic joint resets for all the handlers
    void updateSimulation() override;

    // Update the track device states for all the handlers
    void updateTrackStates() override;

private:
    std::vector<std::unique_ptr<BtContinuousTrackHandler>> handlers_;
    std::unordered_set<Link*> trackDrivenJoints_;
};

}

#endif
