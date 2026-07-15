#ifndef CNOID_PHYSX_PLUGIN_PHYSX_CONTINUOUS_TRACK_SIMULATOR_H
#define CNOID_PHYSX_PLUGIN_PHYSX_CONTINUOUS_TRACK_SIMULATOR_H

#include <memory>

namespace cnoid {

class PhysxBody;

class PhysXContinuousTrackSimulator
{
public:
    virtual ~PhysXContinuousTrackSimulator() = default;

    virtual void setupTrackHandlers(PhysxBody* physxBody) = 0;
    virtual bool empty() const = 0;
    virtual void initializeTrackStates() = 0;
    virtual void updateSimulation() = 0;
    virtual void updateTrackStates() = 0;
};

std::unique_ptr<PhysXContinuousTrackSimulator> createPhysXContinuousTrackSimulator();

}

#endif
