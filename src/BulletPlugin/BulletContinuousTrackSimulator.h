#ifndef CNOID_BULLET_PLUGIN_BULLET_CONTINUOUS_TRACK_SIMULATOR_H
#define CNOID_BULLET_PLUGIN_BULLET_CONTINUOUS_TRACK_SIMULATOR_H

#include <memory>
#include <vector>

namespace cnoid {

class Link;
class BulletBody;
class BulletUnit;

class BulletContinuousTrackUnitSetup
{
public:
    virtual ~BulletContinuousTrackUnitSetup() = default;

    virtual int numInternalLinks() const = 0;
    virtual void setupInternalLinks(BulletUnit* unit, int firstLinkIndex) = 0;
    virtual void completeSetup() = 0;
};

class BulletContinuousTrackSimulator
{
public:
    virtual ~BulletContinuousTrackSimulator() = default;

    virtual std::unique_ptr<BulletContinuousTrackUnitSetup> prepareUnit(
        BulletBody* bulletBody, const std::vector<Link*>& unitLinks) = 0;
    virtual bool empty() const = 0;
    virtual bool isTrackDrivenJoint(Link* link) const = 0;
    virtual void initializeTrackStates() = 0;
    virtual void updateSimulation() = 0;
    virtual void updateTrackStates() = 0;
};

std::unique_ptr<BulletContinuousTrackSimulator> createBulletContinuousTrackSimulator();

}

#endif
