#include "BodyMotionControllerItem.h"
#include "BodyMotionItem.h"
#include <cnoid/ItemManager>
#include <cnoid/ItemList>
#include <cnoid/ControllerIO>
#include <cnoid/MessageView>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Format>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class BodyMotionControllerItemImpl
{
public:
    BodyMotionControllerItem* self;
    BodyMotionItemPtr motionItem;
    shared_ptr<MultiValueSeq> qseqRef;
    BodyPtr body;
    int currentFrame;
    int numJoints;

    BodyMotionControllerItemImpl(BodyMotionControllerItem* self);
    bool initialize(ControllerIO* io);
};

}


void BodyMotionControllerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& itemManager = ext->itemManager();
    itemManager.registerClass<BodyMotionControllerItem, ControllerItem>(N_("BodyMotionControllerItem"));
    itemManager.addCreationPanel<BodyMotionControllerItem>();
}


BodyMotionControllerItem::BodyMotionControllerItem()
{
    impl = new BodyMotionControllerItemImpl(this);
    setNoDelayMode(true);
}


BodyMotionControllerItem::BodyMotionControllerItem(const BodyMotionControllerItem& org)
    : ControllerItem(org)
{
    impl = new BodyMotionControllerItemImpl(this);
}


BodyMotionControllerItemImpl::BodyMotionControllerItemImpl(BodyMotionControllerItem* self)
    : self(self)
{

}


BodyMotionControllerItem::~BodyMotionControllerItem()
{
    delete impl;
}


bool BodyMotionControllerItem::initialize(ControllerIO* io)
{
    return impl->initialize(io);
}


bool BodyMotionControllerItemImpl::initialize(ControllerIO* io)
{
    auto mv = MessageView::instance();

    auto motionItems = self->descendantItems<BodyMotionItem>();
    if(motionItems.empty()){
        mv->putln(
            formatR(_("Any body motion item for {} is not found."), self->displayName()),
            MessageView::Error);
        return false;
    }
    motionItem = motionItems.front();
    // find the first checked item
    for(size_t i=0; i < motionItems.size(); ++i){
        if(motionItems[i]->isChecked()){
            motionItem = motionItems[i];
            break;
        }
    }

    auto motion = motionItem->motion();
    motion->updateLinkPosSeqAndJointPosSeqWithBodyStateSeq();

    qseqRef = static_pointer_cast<MultiValueSeq>(motion->jointPosSeq()->cloneSeq());

    if(qseqRef->numFrames() == 0){
        mv->putln(
            formatR(_("{0} for {1} is empty."), motionItem->displayName(), self->displayName()),
            MessageView::Error);
        return false;
    }
    if(fabs(qseqRef->frameRate() - (1.0 / io->timeStep())) > 1.0e-6){
        mv->putln(
            formatR(_("Frame rate {0} of \"{1}\" is different from the simulation frame rate {2}."),
                    qseqRef->frameRate(), motionItem->displayName(), 1.0 / io->timeStep()),
            MessageView::Error);
        return false;
    }

    body = io->body();

    auto lseq = motion->linkPosSeq();
    if(lseq->numParts() > 0 && lseq->numFrames() > 0){
        SE3& p = lseq->at(0, 0);
        Link* rootLink = body->rootLink();
        rootLink->p() = p.translation();
        rootLink->R() = p.rotation().toRotationMatrix();
    }

    numJoints = std::min(body->numJoints(), qseqRef->numParts());

    MultiValueSeq::Frame q = qseqRef->frame(0);
    for(int i=0; i < numJoints; ++i){
        auto joint = body->joint(i);
        joint->setActuationMode(Link::JointDisplacement);
        joint->q() = joint->q_target() = q[i];
    }
    body->calcForwardKinematics();

    /*
       The first frame is consumed by the initial state set above, so the output
       function must continue from the second frame. A joint displacement written
       by the output function is reflected in the body at the end of the frame,
       and the state of a frame is recorded at its beginning. Hence the reference
       value of a frame must be written one frame earlier to make the recorded
       motion coincide with the reference motion. With a kinematic simulation the
       recorded motion is then identical to the reference motion including its
       number of frames. With a dynamic simulation the difference from the
       reference motion is the actual tracking error of the simulated control.
    */
    currentFrame = 1;

    return true;
}


bool BodyMotionControllerItem::start()
{
    control();
    return true;
}
    
    
double BodyMotionControllerItem::timeStep() const
{
    return impl->qseqRef->timeStep();
}
        

void BodyMotionControllerItem::input()
{

}


bool BodyMotionControllerItem::control()
{
    return (impl->currentFrame < impl->qseqRef->numFrames());
}
        

void BodyMotionControllerItem::output()
{
    if(impl->currentFrame < impl->qseqRef->numFrames()){
        MultiValueSeq::Frame q = impl->qseqRef->frame(impl->currentFrame);
        for(int i=0; i < impl->numJoints; ++i){
            impl->body->joint(i)->q_target() = q[i];
        }
        impl->currentFrame++;
    }
}


void BodyMotionControllerItem::stop()
{
    impl->qseqRef.reset();
    impl->motionItem .reset();
    impl->body.reset();
}


void BodyMotionControllerItem::onDisconnectedFromRoot()
{
    stop();
}
    

Item* BodyMotionControllerItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new BodyMotionControllerItem(*this);
}


void BodyMotionControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Control mode"), string("High-gain"));
}


bool BodyMotionControllerItem::store(Archive& archive)
{
    return true;
}
    

bool BodyMotionControllerItem::restore(const Archive& archive)
{
    return true;
}
