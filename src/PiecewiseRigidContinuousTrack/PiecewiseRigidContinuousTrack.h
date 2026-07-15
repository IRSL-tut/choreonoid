#ifndef CNOID_PIECEWISE_RIGID_CONTINUOUS_TRACK_H
#define CNOID_PIECEWISE_RIGID_CONTINUOUS_TRACK_H

#include <cnoid/Device>
#include <cnoid/EigenTypes>
#include <string>
#include <vector>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class SgMaterial;

class CNOID_EXPORT PiecewiseRigidContinuousTrack : public Device
{
public:
    PiecewiseRigidContinuousTrack();
    PiecewiseRigidContinuousTrack(const PiecewiseRigidContinuousTrack& org, bool copyStateOnly = false);
    virtual const char* typeName() const override;
    void copyStateFrom(const PiecewiseRigidContinuousTrack& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState(DeviceState* existingClone) const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual void clearState() override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf, int size) override;
    virtual double* writeState(double* out_buf) const override;

    struct Spec {
        std::string sprocketName;
        double sprocketRadius;
        std::string idlerName;
        double idlerRadius;
        int numShoes;
        double grouserHeight;
        double grouserThickness;
        double thickness;
        double width;
        std::string contactMaterialName;
        ref_ptr<SgMaterial> appearanceMaterial;
        double mass;
        double driveDamping;
        double driveMaxForce;

        // Computed geometry data (set by clearState)

        // The number of visible shoes varies slightly per frame because wheel
        // rotation shifts which grousers pass the half-arc visibility test.
        // This fixed upper-bound is used for state I/O buffer allocation so
        // that stateSize() remains constant across simulation steps.
        int maxNumVisibleShoes;

        // Center-to-center distance between the sprocket and idler, in meters.
        double wheelDistance;

        // Center-to-center distance between adjacent grousers, in meters. It is
        // calculated using the shoe count selected for phase alignment.
        double grouserSpacing;

        // Arithmetic mean of the nominal sprocket and idler radii, in meters.
        double avgWheelRadius;

        // Sprocket radius plus the belt thickness, in meters.
        double effectiveSprocketRadius;

        // Idler radius plus the belt thickness, in meters.
        double effectiveIdlerRadius;

        // Nominal grouser count on one linear segment, calculated as the floor
        // of wheelDistance / grouserSpacing with a minimum of one.
        int numGrousersPerSegment;

        // Upper bound on visible grousers around the outer sprocket half-arc.
        int numGrousersOnSprocket;

        // Upper bound on visible grousers around the outer idler half-arc.
        int numGrousersOnIdler;

        // Center of the upper rigid linear segment in the device-link frame.
        Vector3 upperSegmentCenter;

        // Center of the lower rigid linear segment in the device-link frame.
        Vector3 lowerSegmentCenter;

        // Unit vector from the idler center toward the sprocket center, in the
        // device-link frame. This is the sliding axis of the linear segments.
        Vector3 slideDir;

        // Sliding-segment length in meters; currently equal to wheelDistance.
        double segmentLength;

        // True when clearState() has successfully calculated valid geometry.
        bool geometryInitialized;

        // Number of physics grousers distributed around the full sprocket.
        int sprocketFullGrousers;

        // Number of physics grousers distributed around the full idler.
        int idlerFullGrousers;

        // Angular interval between sprocket grousers, in radians.
        double angleStepSprocket;

        // Angular interval between idler grousers, in radians.
        double angleStepIdler;

        // Number of box segments approximating the full sprocket belt. This is
        // a multiple of sprocketFullGrousers selected to be close to 48.
        int sprocketBeltSegments;

        // Number of box segments approximating the full idler belt. This is a
        // multiple of idlerFullGrousers selected to be close to 48.
        int idlerBeltSegments;

        // Angular interval between sprocket belt segments, in radians.
        double angleStepSprocketBelt;

        // Angular interval between idler belt segments, in radians.
        double angleStepIdlerBelt;

        Spec();
    };

    const Spec& spec() const { return *spec_; }

    // Spec accessors

    //! Returns the name of the revolute link used as the driven sprocket.
    //! The corresponding YAML key is "sprocket", and the default is empty.
    const std::string& sprocketName() const { return spec_->sprocketName; }
    void setSprocketName(const std::string& name) { spec_->sprocketName = name; }

    //! Returns the nominal sprocket radius in meters.
    //! The corresponding YAML key is "sprocket_radius", and the default is 0.0.
    double sprocketRadius() const { return spec_->sprocketRadius; }
    void setSprocketRadius(double r) { spec_->sprocketRadius = r; }

    //! Returns the name of the revolute link used as the idler wheel.
    //! The corresponding YAML key is "idler", and the default is empty.
    const std::string& idlerName() const { return spec_->idlerName; }
    void setIdlerName(const std::string& name) { spec_->idlerName = name; }

    //! Returns the nominal idler radius in meters.
    //! The corresponding YAML key is "idler_radius", and the default is 0.0.
    double idlerRadius() const { return spec_->idlerRadius; }
    void setIdlerRadius(double r) { spec_->idlerRadius = r; }

    //! Returns the requested number of shoes around the complete track.
    //! The corresponding YAML key is "num_shoes", and the default is 20.
    //! clearState() may use a nearby count for phase alignment, but the value
    //! returned by this function remains the requested value.
    int numShoes() const { return spec_->numShoes; }
    void setNumShoes(int n) { spec_->numShoes = n; }

    //! Returns the radial height of each grouser above the belt, in meters.
    //! The corresponding YAML key is "grouser_height", and the default is 0.01.
    double grouserHeight() const { return spec_->grouserHeight; }
    void setGrouserHeight(double h) { spec_->grouserHeight = h; }

    //! Returns the grouser dimension along the track travel direction, in meters.
    //! The corresponding YAML key is "grouser_thickness", and the default is
    //! 0.0. A value not greater than zero makes effectiveGrouserThickness() use
    //! the belt thickness instead.
    double grouserThickness() const { return spec_->grouserThickness; }
    void setGrouserThickness(double t) { spec_->grouserThickness = t; }

    //! Returns the actual grouser thickness after applying the fallback.
    double effectiveGrouserThickness() const {
        return (spec_->grouserThickness > 0.0) ? spec_->grouserThickness : spec_->thickness;
    }

    //! Returns the belt plate and wheel belt thickness in meters.
    //! The corresponding YAML key is "thickness", and the default is 0.005.
    //! This value also offsets the effective sprocket and idler radii.
    double thickness() const { return spec_->thickness; }
    void setThickness(double t) { spec_->thickness = t; }

    //! Returns the lateral width of all belt and grouser shapes, in meters.
    //! The corresponding YAML key is "width", and the default is 0.1.
    double width() const { return spec_->width; }
    void setWidth(double w) { spec_->width = w; }

    //! Returns the contact material name for the generated track shapes.
    //! The corresponding YAML key is "contact_material". An empty name uses
    //! the device-link material. PhysX assigns the material to each added shape.
    //! Bullet also assigns it to the complete sprocket and idler colliders
    //! because Bullet cannot select a contact material per shape.
    const std::string& contactMaterialName() const { return spec_->contactMaterialName; }
    void setContactMaterialName(const std::string& name) { spec_->contactMaterialName = name; }

    //! Returns the optional material used to render the generated track geometry.
    //! The corresponding YAML key is "appearance_material". A null material
    //! uses the built-in dark-gray material.
    SgMaterial* appearanceMaterial() const { return spec_->appearanceMaterial; }
    void setAppearanceMaterial(SgMaterial* material) { spec_->appearanceMaterial = material; }

    //! Returns the total mass of the upper and lower rigid segments, in kg.
    //! The corresponding YAML key is "mass". A value not greater than zero uses
    //! the sum of the sprocket and idler link masses. Each segment receives half
    //! of the resulting total mass.
    double mass() const { return spec_->mass; }
    void setMass(double m) { spec_->mass = m; }

    //! Returns the damping coefficient of the track velocity drive.
    //! The corresponding YAML key is "drive_damping", and the default is 1.0e4.
    //! PhysX passes this value directly to PxArticulationDrive::damping. Bullet
    //! does not use it because its motor has different gain semantics and uses
    //! the velocity gain configured on BulletSimulatorItem instead.
    double driveDamping() const { return spec_->driveDamping; }
    void setDriveDamping(double d) { spec_->driveDamping = d; }

    //! Returns the maximum force of the track velocity drive, in N.
    //! The corresponding YAML key is "drive_max_force". A value not greater
    //! than zero means unlimited. PhysX uses it as the drive force limit, while
    //! Bullet converts it to a per-step impulse by multiplying it by the time step.
    double driveMaxForce() const { return spec_->driveMaxForce; }
    void setDriveMaxForce(double f) { spec_->driveMaxForce = f; }

    bool isGeometryInitialized() const { return spec_->geometryInitialized; }

    // Engine-independent geometry utilities
    int numGrousersOnLinearSegment(double segmentLength) const;
    Vector3 linearGrouserPosition(
        double segmentLength, const Vector3& segmentAxis,
        double direction, int index, int numGrousers) const;
    Isometry3 wheelGrouserPose(double wheelRadius, int index, double angleStep) const;
    double wheelBeltChordLength(double wheelRadius, int numBeltSegments) const;
    Isometry3 wheelBeltPose(double wheelRadius, int index, int numBeltSegments) const;
    static bool isWheelGrouserVisible(
        int index, double angleStep, double wheelPosition, bool isOuterPositive);

    // Engine-independent phase reset utilities
    static double wrapPosition(double position, double period);
    static double resetCoupledPosition(
        double masterPosition, double newMasterPosition,
        double followerPosition, double gearRatio, double followerPeriod);

    // State accessors
    int numVisibleShoes() const { return static_cast<int>(shoePositions_.size()); }

    const std::vector<SE3>& shoePositions() const { return shoePositions_; }
    std::vector<SE3>& shoePositions() { return shoePositions_; }
    void addShoePosition(const SE3& pos);
    void clearShoePositions();

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    // State
    std::vector<SE3> shoePositions_;

    // Spec
    std::unique_ptr<Spec> spec_;
};

typedef ref_ptr<PiecewiseRigidContinuousTrack> PiecewiseRigidContinuousTrackPtr;

}

#endif
