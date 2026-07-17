#include "SceneOverlay.h"
#include "SceneGraph.h"
#include "SceneNodeClassRegistry.h"
#include "CloneMap.h"
#include <vector>

using namespace std;
using namespace cnoid;


void cnoid::registerSceneOverlayNodeClasses()
{
    static bool registered = false;
    if(!registered){
        registerSceneGraphNodeClasses();

        SceneNodeClassRegistry::instance()
            .registerClass<SgOverlay, SgGroup>("SgOverlay")
            .registerClass<SgViewportOverlay, SgOverlay>("SgViewportOverlay")
            .registerClass<SgHudOverlay, SgViewportOverlay>("SgHudOverlay")
            .registerClass<SgOverlayPanel, SgSpatialNode>("SgOverlayPanel");
        registered = true;
    }
}


namespace {

struct NodeClassRegistration {
    NodeClassRegistration() {
        registerSceneOverlayNodeClasses();
    }
} registration;

}


SgOverlay::SgOverlay(int classId)
    : SgGroup(classId)
{

}


SgOverlay::SgOverlay()
    : SgGroup(findClassId<SgOverlay>())
{

}


SgOverlay::SgOverlay(const SgOverlay& org, CloneMap* cloneMap)
    : SgGroup(org, cloneMap)
{

}


SgOverlay::~SgOverlay()
{

}


Referenced* SgOverlay::doClone(CloneMap* cloneMap) const
{
    return new SgOverlay(*this, cloneMap);
}


SgViewportOverlay::SgViewportOverlay(int classId)
    : SgOverlay(classId),
      coordinateMode_(PhysicalPixelCoordinates)
{

}


SgViewportOverlay::SgViewportOverlay()
    : SgOverlay(findClassId<SgViewportOverlay>()),
      coordinateMode_(PhysicalPixelCoordinates)
{

}


SgViewportOverlay::SgViewportOverlay(const SgViewportOverlay& org, CloneMap* cloneMap)
    : SgOverlay(org, cloneMap),
      coordinateMode_(org.coordinateMode_)
{

}


SgViewportOverlay::~SgViewportOverlay()
{

}


Referenced* SgViewportOverlay::doClone(CloneMap* cloneMap) const
{
    return new SgViewportOverlay(*this, cloneMap);
}


void SgViewportOverlay::calcViewVolume(double /* viewportWidth */, double /* viewportHeight */, ViewVolume& io_volume)
{
    io_volume.left = -1.0;
    io_volume.right = 1.0;
    io_volume.bottom = -1.0;
    io_volume.top = 1.0;
    io_volume.zNear = 1.0;
    io_volume.zFar = -1.0;
}


SgOverlayPanel::SgOverlayPanel(int classId)
    : SgSpatialNode(classId)
{
    width_ = 0.0;
    height_ = 0.0;
    colorMode_ = FixedColor;
    color_ << 0.0f, 0.0f, 0.0f;
    transparency_ = 0.0f;
}


SgOverlayPanel::SgOverlayPanel()
    : SgOverlayPanel(findClassId<SgOverlayPanel>())
{

}


SgOverlayPanel::SgOverlayPanel(double width, double height)
    : SgOverlayPanel()
{
    setSize(width, height);
}


SgOverlayPanel::SgOverlayPanel(const SgOverlayPanel& org, CloneMap* /* cloneMap */)
    : SgSpatialNode(org)
{
    width_ = org.width_;
    height_ = org.height_;
    colorMode_ = org.colorMode_;
    color_ = org.color_;
    transparency_ = org.transparency_;
}


SgOverlayPanel::~SgOverlayPanel()
{

}


Referenced* SgOverlayPanel::doClone(CloneMap* cloneMap) const
{
    return new SgOverlayPanel(*this, cloneMap);
}


void SgOverlayPanel::setSize(double width, double height)
{
    width_ = width;
    height_ = height;
    invalidateBoundingBox();
}


const BoundingBox& SgOverlayPanel::untransformedBoundingBox() const
{
    if(!hasValidBoundingBoxCache()){
        untransformedBboxCache.clear();
        untransformedBboxCache.expandBy(Vector3(0.0, 0.0, 0.0));
        untransformedBboxCache.expandBy(Vector3(width_, -height_, 0.0));
    }
    return untransformedBboxCache;
}


const BoundingBox& SgOverlayPanel::boundingBox() const
{
    if(hasValidBoundingBoxCache()){
        return bboxCache_;
    }
    bboxCache_ = untransformedBoundingBox();
    if(hasLocalPosition()){
        bboxCache_.transform(position());
    }
    setBoundingBoxCacheReady();
    return bboxCache_;
}


namespace cnoid {

class SgHudOverlay::Impl
{
public:
    struct Item {
        SgPosTransformPtr transform;
        SgNodePtr node;
        Anchor anchor;
        int offsetX;
        int offsetY;
        int width;
    };

    SgHudOverlay* self;
    vector<Item> items;
    double lastWidth;
    double lastHeight;

    Impl(SgHudOverlay* self);
    void layout(double width, double height);
};

}


SgHudOverlay::Impl::Impl(SgHudOverlay* self)
    : self(self),
      lastWidth(0.0),
      lastHeight(0.0)
{

}


void SgHudOverlay::Impl::layout(double width, double height)
{
    for(auto& item : items){
        double x = 0.0;
        double y = 0.0;
        switch(item.anchor){
        case TopLeft:      x = item.offsetX;                          y = height - item.offsetY; break;
        case TopRight:     x = width - item.offsetX - item.width;     y = height - item.offsetY; break;
        case BottomLeft:   x = item.offsetX;                          y = item.offsetY;          break;
        case BottomRight:  x = width - item.offsetX - item.width;     y = item.offsetY;          break;
        case TopCenter:    x = width * 0.5 + item.offsetX - item.width * 0.5; y = height - item.offsetY; break;
        case BottomCenter: x = width * 0.5 + item.offsetX - item.width * 0.5; y = item.offsetY;          break;
        case LeftCenter:   x = item.offsetX;                          y = height * 0.5 - item.offsetY; break;
        case RightCenter:  x = width - item.offsetX - item.width;     y = height * 0.5 - item.offsetY; break;
        case Center:       x = width * 0.5 + item.offsetX - item.width * 0.5; y = height * 0.5 - item.offsetY; break;
        }
        item.transform->setTranslation(Vector3(x, y, 0.0));
        item.transform->notifyUpdate(SgUpdate::Modified);
    }
    lastWidth = width;
    lastHeight = height;
}


SgHudOverlay::SgHudOverlay(int classId)
    : SgViewportOverlay(classId)
{
    setCoordinateMode(LogicalPixelCoordinates);
    impl = new Impl(this);
}


SgHudOverlay::SgHudOverlay()
    : SgViewportOverlay(findClassId<SgHudOverlay>())
{
    setCoordinateMode(LogicalPixelCoordinates);
    impl = new Impl(this);
}


SgHudOverlay::SgHudOverlay(const SgHudOverlay& org, CloneMap* cloneMap)
    : SgViewportOverlay(org, cloneMap)
{
    impl = new Impl(this);
    for(auto& src : org.impl->items){
        Impl::Item item;
        item.anchor = src.anchor;
        item.offsetX = src.offsetX;
        item.offsetY = src.offsetY;
        item.width = src.width;
        item.node = nullptr;
        if(src.node){
            if(cloneMap){
                item.node = static_cast<SgNode*>(src.node->clone(*cloneMap));
            } else {
                item.node = static_cast<SgNode*>(src.node->clone());
            }
        }
        item.transform = new SgPosTransform;
        if(item.node){
            item.transform->addChild(item.node);
        }
        addChild(item.transform);
        impl->items.push_back(item);
    }
}


SgHudOverlay::~SgHudOverlay()
{
    delete impl;
}


Referenced* SgHudOverlay::doClone(CloneMap* cloneMap) const
{
    return new SgHudOverlay(*this, cloneMap);
}


bool SgHudOverlay::addItem(SgNode* node, Anchor anchor, int offsetX, int offsetY, int width)
{
    if(!node){
        return false;
    }
    Impl::Item item;
    item.node = node;
    item.anchor = anchor;
    item.offsetX = offsetX;
    item.offsetY = offsetY;
    item.width = width;
    item.transform = new SgPosTransform;
    item.transform->addChild(node);
    addChild(item.transform);
    impl->items.push_back(item);
    if(impl->lastWidth > 0.0 && impl->lastHeight > 0.0){
        impl->layout(impl->lastWidth, impl->lastHeight);
    }
    return true;
}


SgOverlayPanel* SgHudOverlay::addPanel
(double width, double height, Anchor anchor, int offsetX, int offsetY)
{
    SgOverlayPanelPtr panel = new SgOverlayPanel(width, height);
    if(addItem(panel, anchor, offsetX, offsetY, static_cast<int>(width))){
        return panel.retn();
    }
    return nullptr;
}


bool SgHudOverlay::removeItem(SgNode* node)
{
    for(auto it = impl->items.begin(); it != impl->items.end(); ++it){
        if(it->node == node){
            removeChild(it->transform);
            impl->items.erase(it);
            return true;
        }
    }
    return false;
}


void SgHudOverlay::clearItems()
{
    for(auto& item : impl->items){
        removeChild(item.transform);
    }
    impl->items.clear();
}


void SgHudOverlay::calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume)
{
    io_volume.left = 0.0;
    io_volume.right = viewportWidth;
    io_volume.bottom = 0.0;
    io_volume.top = viewportHeight;
    io_volume.zNear = 1.0;
    io_volume.zFar = -1.0;
    impl->layout(viewportWidth, viewportHeight);
}
