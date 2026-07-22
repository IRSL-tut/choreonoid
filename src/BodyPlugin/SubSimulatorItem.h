/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_SUB_SIMULATOR_ITEM_H
#define CNOID_BODY_PLUGIN_SUB_SIMULATOR_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class SimulatorItem;

class CNOID_EXPORT SubSimulatorItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    SubSimulatorItem();
    SubSimulatorItem(const SubSimulatorItem& org);
    
    virtual bool isEnabled();
    virtual bool setEnabled(bool on);
    virtual bool initializeSimulation(SimulatorItem* simulatorItem);
    virtual void finalizeSimulation();

    /**
       Override this function to return true if the item also functions during
       the playback of recorded simulation results without an active simulation,
       such as an item that visualizes device states restored from a log.
       Items returning true are retained in the projects saved as log playback
       archives by WorldLogFileItem.
    */
    virtual bool isApplicableToLogPlayback() const;

protected:
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
    
private:
    bool isEnabled_;
};

typedef ref_ptr<SubSimulatorItem> SubSimulatorItemPtr;
}

#endif
