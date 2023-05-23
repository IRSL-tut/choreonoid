#include "BodyItemKinematicsKitManager.h"
#include "BodyItemKinematicsKit.h"
#include "BodySelectionManager.h"
#include "KinematicsBar.h"
#include <cnoid/Link>
#include <cnoid/JointPath>
#include <cnoid/CompositeIK>
#include <cnoid/PinDragIK>
#include <cnoid/CoordinateFrameList>
#include <cnoid/CoordinateFrameListItem>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/WorldItem>
#include <cnoid/ItemList>
#include <cnoid/ValueTree>
#include <map>

using namespace std;
using namespace cnoid;

namespace {

enum BaseLinkId {
    PresetBaseLink = -1,
    UnspecifiedBaseLinkForPinDragIK = -2
};

constexpr int MainEndLinkGuessBufSize = 10;

}
    
namespace cnoid {

class BodyItemKinematicsKitManager::Impl
{
public:
    BodyItem* bodyItem;
    Body* body;

    // Key is pair(target link index, base link index);
    // Use an integer index value as a key to keep the number of instances growing
    map<std::pair<int, int>, BodyItemKinematicsKitPtr> linkPairToKinematicsKitMap;

    ScopedConnection bodyItemConnection;
    ScopedConnection frameListConnection;
    CoordinateFrameListPtr baseFrames;
    CoordinateFrameListPtr offsetFrames;
    CoordinateFrameListPtr defaultBaseFrames;
    CoordinateFrameListPtr defaultOffsetFrames;

    BodySelectionManager* bodySelectionManager;

    Impl(BodyItem* bodyItem);
    void onBodyItemPositionChanged();
    Link* guessMainEndLink();
    void guessMainEndLinkSub(Link* link, int dof, Link** linkOfDof);
    BodyItemKinematicsKit* findKinematicsKit(Link* targetLink, bool isPresetOnly);
    std::shared_ptr<InverseKinematics> findPresetIK(Link* targetLink);
    bool updateCoordinateFramesOf(BodyItemKinematicsKit* kit, bool forceUpdate);
    void findFrameListItems();
    void onFrameListAssociation(CoordinateFrameListItem* frameListItem, bool on);
};

}


BodyItemKinematicsKitManager::BodyItemKinematicsKitManager(BodyItem* bodyItem)
{
    impl = new Impl(bodyItem);
}


BodyItemKinematicsKitManager::Impl::Impl(BodyItem* bodyItem)
    : bodyItem(bodyItem),
      body(bodyItem->body())
{
    bodySelectionManager = BodySelectionManager::instance();

    bodyItemConnection =
        bodyItem->sigTreePositionChanged().connect(
            [&](){ onBodyItemPositionChanged(); });

    onBodyItemPositionChanged();
}


BodyItemKinematicsKitManager::~BodyItemKinematicsKitManager()
{
    delete impl;
}


void BodyItemKinematicsKitManager::Impl::onBodyItemPositionChanged()
{
    if(bodyItem->isConnectedToRoot()){
        if(!frameListConnection.connected()){
            frameListConnection =
                CoordinateFrameListItem::sigListAssociationWith(bodyItem).connect(
                    [&](CoordinateFrameListItem* frameListItem, bool on){
                        onFrameListAssociation(frameListItem, on);
                    });
        }
        findFrameListItems();
    } else {
        if(frameListConnection.connected()){
            frameListConnection.disconnect();
            baseFrames.reset();
            offsetFrames.reset();
        }
    }
}


//  Find a maximum DOF link whose DOF is different from any other links.
Link* BodyItemKinematicsKitManager::Impl::guessMainEndLink()
{
    Link* linkOfDof[MainEndLinkGuessBufSize];
    for(int i=0; i < MainEndLinkGuessBufSize; ++i){
        linkOfDof[i] = nullptr;
    }
    guessMainEndLinkSub(body->rootLink(), 0, linkOfDof);

    for(int i = MainEndLinkGuessBufSize - 1; i > 0; --i){
        auto link = linkOfDof[i];
        if(link && link != body->rootLink()){
            return link;
        }
    }
    return nullptr;
}


void BodyItemKinematicsKitManager::Impl::guessMainEndLinkSub(Link* link, int dof, Link** linkOfDof)
{
    if(!linkOfDof[dof]){
        linkOfDof[dof] = link;
    } else {
        linkOfDof[dof] = body->rootLink();
    }
    ++dof;
    if(dof < MainEndLinkGuessBufSize){
        for(Link* child = link->child(); child; child = child->sibling()){
            guessMainEndLinkSub(child, dof, linkOfDof);
        }
    }
}


BodyItemKinematicsKit* BodyItemKinematicsKitManager::getCurrentKinematicsKit(Link* targetLink)
{
    return impl->findKinematicsKit(targetLink, false);
}


BodyItemKinematicsKit* BodyItemKinematicsKitManager::findPresetKinematicsKit(Link* targetLink)
{
    return impl->findKinematicsKit(targetLink, true);
}


BodyItemKinematicsKit* BodyItemKinematicsKitManager::Impl::findKinematicsKit(Link* targetLink, bool isPresetOnly)
{
    if(!targetLink){
        if(!isPresetOnly){
            return nullptr;
        } else {
            targetLink = guessMainEndLink();
            if(!targetLink){
                return nullptr;
            }
        }
    }
    bool isPresetMode = false;
    if(!isPresetOnly && KinematicsBar::instance()->mode() == KinematicsBar::PresetKinematics){
        isPresetMode = true;
    }

    Link* baseLink = nullptr;
    int baseLinkIndex;

    if(isPresetOnly || isPresetMode){
        baseLinkIndex = PresetBaseLink;
    } else {
        auto pinDragIK = bodyItem->checkPinDragIK();
        if(pinDragIK && pinDragIK->numPinnedLinks() > 0){
            baseLinkIndex = UnspecifiedBaseLinkForPinDragIK;
        } else {
            baseLink = bodyItem->currentBaseLink();
            if(!baseLink){
                baseLink = body->rootLink();
            }
            baseLinkIndex = baseLink->index();
        }
    }

    auto key = make_pair(targetLink->index(), baseLinkIndex);

    BodyItemKinematicsKit* kit = nullptr;
    auto iter = linkPairToKinematicsKitMap.find(key);
    if(iter != linkPairToKinematicsKitMap.end()){
        kit = iter->second;
    }

    if(!kit){
        bool needToRegistration = false;
        
        if(isPresetOnly || isPresetMode){
            auto presetIK = findPresetIK(targetLink);
            if(presetIK){
                kit = new BodyItemKinematicsKit(bodyItem);
                kit->setInverseKinematics(targetLink, presetIK);
                needToRegistration = true;
            }
        }
        if(!kit && !isPresetOnly){
            // Special case
            if(baseLinkIndex == PresetBaseLink && targetLink->isFixedToRoot()){
                baseLink = targetLink;
                baseLinkIndex = targetLink->index();
                key.second = baseLinkIndex;
                auto iter = linkPairToKinematicsKitMap.find(key);
                if(iter != linkPairToKinematicsKitMap.end()){
                    kit = iter->second;
                }
            }
        }
        if(!kit && (baseLinkIndex != PresetBaseLink)){
            kit = new BodyItemKinematicsKit(bodyItem);
            if(baseLinkIndex == UnspecifiedBaseLinkForPinDragIK){
                kit->setEndLink(targetLink);
            } else {
                kit->setInverseKinematics(targetLink, JointPath::getCustomPath(baseLink, targetLink));
            }
            needToRegistration = true;
        }
        if(needToRegistration && kit){
            linkPairToKinematicsKitMap[key] = kit;
            updateCoordinateFramesOf(kit, true);
        }
    }

    if(baseLinkIndex == UnspecifiedBaseLinkForPinDragIK){
        auto pinDragIK = bodyItem->checkPinDragIK();
        pinDragIK->setTargetLink(targetLink, true);
        if(pinDragIK->initialize()){
            kit->setInverseKinematics(targetLink, pinDragIK);
        }
    }
    
    return kit;
}


std::shared_ptr<InverseKinematics> BodyItemKinematicsKitManager::Impl::findPresetIK(Link* targetLink)
{
    std::shared_ptr<InverseKinematics> ik;
    const Mapping& setupMap = *body->info()->findMapping("defaultIKsetup");
    if(setupMap.isValid()){
        const Listing& setup = *setupMap.findListing(targetLink->name());
        if(setup.isValid() && !setup.empty()){
            Link* baseLink = body->link(setup[0].toString());
            if(baseLink){
                if(setup.size() == 1){
                    ik = JointPath::getCustomPath(baseLink, targetLink);
                } else {
                    auto compositeIK = make_shared<CompositeIK>(body, targetLink);
                    ik = compositeIK;
                    for(int i=0; i < setup.size(); ++i){
                        Link* baseLink = body->link(setup[i].toString());
                        if(baseLink){
                            if(!compositeIK->addBaseLink(baseLink)){
                                ik.reset();
                                break;
                            }
                        }
                    }
                }
            }
        }
    } else {
        // If the link is an end link that has more than five degrees of freedom,
        // the path between the link and the root link is treated as an IK link
        // when there is no IK link specification
        if(!targetLink->child()){
            int dof = 0;
            auto link = targetLink->parent();
            while(link){
                if(++dof >= 6){
                    ik = JointPath::getCustomPath(body->rootLink(), targetLink);
                    break;
                }
                link = link->parent();
            }
        }
    }
    return ik;
}


bool BodyItemKinematicsKitManager::Impl::updateCoordinateFramesOf(BodyItemKinematicsKit* kit, bool forceUpdate)
{
    bool updated = false;
    
    auto prevBaseFrames = kit->baseFrames();
    if(forceUpdate || prevBaseFrames){
        CoordinateFrameList* newBaseFrames = nullptr;
        if(baseFrames){
            newBaseFrames = baseFrames;
        } else {
            if(!defaultBaseFrames){
                defaultBaseFrames = new CoordinateFrameList;
                defaultBaseFrames->setFrameType(CoordinateFrameList::Base);
            }
            newBaseFrames = defaultBaseFrames;
        }
        if(newBaseFrames != prevBaseFrames){
            kit->setBaseFrames(newBaseFrames);
            updated = true;
        }
    }

    auto prevOffsetFrames = kit->offsetFrames();
    if(forceUpdate || prevOffsetFrames){
        CoordinateFrameList* newOffsetFrames = nullptr;
        if(offsetFrames){
            newOffsetFrames = offsetFrames;
        } else {
            if(!defaultOffsetFrames){
                defaultOffsetFrames = new CoordinateFrameList;
                defaultOffsetFrames->setFrameType(CoordinateFrameList::Offset);
            }
            newOffsetFrames = defaultOffsetFrames;
        }
        if(newOffsetFrames != prevOffsetFrames){
            kit->setOffsetFrames(newOffsetFrames);
            updated = true;
        }
    }

    return updated;
}
        

void BodyItemKinematicsKitManager::Impl::findFrameListItems()
{
    baseFrames.reset();
    offsetFrames.reset();
    auto items = bodyItem->descendantItems<CoordinateFrameListItem>();
    for(auto& item : items){
        auto frameList = item->frameList();
        if(!baseFrames && frameList->isForBaseFrames()){
            baseFrames = frameList;
        }
        if(!offsetFrames && frameList->isForOffsetFrames()){
            offsetFrames = frameList;
        }
        if(baseFrames && offsetFrames){
            break;
        }
    }
}


/**
   \todo Currently only one frame list item can be associated with a body item for each frame type
   at the same time, but multiple frame lists should be associated with the item as a composite list.
*/
void BodyItemKinematicsKitManager::Impl::onFrameListAssociation(CoordinateFrameListItem* frameListItem, bool on)
{
    bool updated = false;
    
    auto frameList = frameListItem->frameList();
    if(frameList->isForBaseFrames()){
        if(on){
            if(!baseFrames){ // Keep the existing base frames by this condition
                baseFrames = frameList;
                updated = true;
            }
        } else {
            if(frameList == baseFrames){
                baseFrames.reset();
                updated = true;
            }
        }
    } else if(frameList->isForOffsetFrames()){
        if(on){
            if(!offsetFrames){ // Keep the existing base frames by this condition
                offsetFrames = frameList;
                updated = true;
            }
        } else {
            if(frameList == offsetFrames){
                offsetFrames.reset();
                updated = true;
            }
        }
    }
    if(updated){
        for(auto& kv : linkPairToKinematicsKitMap){
            auto& kit = kv.second;
            if(updateCoordinateFramesOf(kit, false)){
                kit->notifyFrameSetChange();
            }
        }
    }
}


void BodyItemKinematicsKitManager::clearKinematicsKits()
{
    impl->linkPairToKinematicsKitMap.clear();
}


bool BodyItemKinematicsKitManager::storeState(Mapping& archive) const
{
    archive.setKeyQuoteStyle(DOUBLE_QUOTED);
    for(auto& kv : impl->linkPairToKinematicsKitMap){
        auto& linkIndexPair = kv.first;
        int baseLinkIndex = linkIndexPair.second;

        // This function only stores the states of preset link kinematics objects
        if(baseLinkIndex == PresetBaseLink){
            int linkIndex = linkIndexPair.first;
            if(auto link = impl->body->link(linkIndex)){
                auto& kit = kv.second;
                MappingPtr state = new Mapping;
                if(!kit->storeState(*state)){
                    return false;
                }
                if(!state->empty()){
                    archive.insert(link->name(), state);
                }
            }
        }
    }
    return true;
}


bool BodyItemKinematicsKitManager::restoreState(const Mapping& archive)
{
    for(auto& kv : archive){
        auto& linkName = kv.first;
        if(auto link = impl->body->link(linkName)){
            if(auto kit = findPresetKinematicsKit(link)){
                kit->restoreState(*kv.second->toMapping());
            }
        }
    }
    return true;
}
