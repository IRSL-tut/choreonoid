#ifndef CNOID_BULLET_PLUGIN_BULLET_SIMULATOR_ITEM_H
#define CNOID_BULLET_PLUGIN_BULLET_SIMULATOR_ITEM_H

#include <cnoid/SimulatorItem>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT BulletSimulatorItem : public SimulatorItem
{
public:
    static void initialize(ExtensionManager* ext);

    BulletSimulatorItem();
    BulletSimulatorItem(const BulletSimulatorItem& org);
    virtual ~BulletSimulatorItem();

    void setGravity(const Vector3& gravity);
    const Vector3& gravity() const;
    virtual Vector3 getGravity() const override;

    class Impl;

protected:
    virtual SimulationBody* createSimulationBody(Body* orgBody, CloneMap& cloneMap) override;
    virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies) override;
    virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies) override;
    virtual void clearSimulation() override;

    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    Impl* impl;
};

typedef ref_ptr<BulletSimulatorItem> BulletSimulatorItemPtr;

}

#endif
