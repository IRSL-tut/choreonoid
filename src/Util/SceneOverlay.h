#ifndef CNOID_UTIL_SCENE_OVERLAY_H
#define CNOID_UTIL_SCENE_OVERLAY_H

#include "SceneGraph.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SgOverlay : public SgGroup
{
public:
    SgOverlay();
    SgOverlay(const SgOverlay& org, CloneMap* cloneMap = nullptr);
    ~SgOverlay();

protected:
    SgOverlay(int classId);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
};


class CNOID_EXPORT SgViewportOverlay : public SgOverlay
{
public:
    SgViewportOverlay();
    SgViewportOverlay(const SgViewportOverlay& org, CloneMap* cloneMap = nullptr);
    ~SgViewportOverlay();

    struct ViewVolume {
        double left;
        double right;
        double bottom;
        double top;
        double zNear;
        double zFar;
    };

    virtual void calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume);

protected:
    SgViewportOverlay(int classId);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
};


/**
   HudOverlay places child nodes at pixel-based offsets from one of the
   anchor positions of the viewport (corners, edge centers, or the center).
   The viewport is mapped to an orthographic projection in pixel units so
   that text and other 2D elements appear as a head-up display fixed to
   the screen.
*/
class CNOID_EXPORT HudOverlay : public SgViewportOverlay
{
public:
    enum Anchor {
        TopLeft,
        TopRight,
        BottomLeft,
        BottomRight,
        TopCenter,
        BottomCenter,
        LeftCenter,
        RightCenter,
        Center
    };

    HudOverlay();
    HudOverlay(const HudOverlay& org, CloneMap* cloneMap = nullptr);
    ~HudOverlay();

    /**
       Add a node to be displayed at the given anchor position with
       a pixel offset. The node is wrapped in an internal SgPosTransform
       whose translation is updated whenever the viewport size changes.
       offsetX is positive toward the right; offsetY is positive toward
       the bottom.

       When anchor is TopRight, BottomRight, or RightCenter, the item's
       local origin (0, 0) is shifted left by width pixels so that a node
       whose extent is [0, width] appears flush against the right edge of
       the anchor. Set width to 0 (default) if the node does not need such
       right-alignment (its origin is placed exactly at the anchor).

       Returns true if the node was added.
    */
    bool addItem(SgNode* node, Anchor anchor, int offsetX, int offsetY, int width = 0);

    bool removeItem(SgNode* node);

    void clearItems();

    virtual void calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume) override;

protected:
    HudOverlay(int classId);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<SgOverlay> SgOverlayPtr;
typedef ref_ptr<SgViewportOverlay> SgViewportOverlayPtr;
typedef ref_ptr<HudOverlay> HudOverlayPtr;

// Register overlay scene node classes to SceneNodeClassRegistry
void registerSceneOverlayNodeClasses();

}

#endif
