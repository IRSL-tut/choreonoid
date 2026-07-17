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

    enum CoordinateMode {
        PhysicalPixelCoordinates,
        LogicalPixelCoordinates
    };

    void setCoordinateMode(CoordinateMode mode) { coordinateMode_ = mode; }
    CoordinateMode coordinateMode() const { return coordinateMode_; }

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

private:
    CoordinateMode coordinateMode_;
};

/**
   A rectangular overlay panel in the local XY plane.

   The local origin is the upper-left corner of the panel. The panel extends
   along +X by width and along -Y by height, with local Z fixed at zero.
   Use the SgSpatialNode local position to place the panel, including its
   depth relative to other overlay primitives.

   This node is intended to grow as a general overlay panel primitive. Future
   extensions may include image fills, borders, gradients, and other panel
   styles commonly used for viewport overlays and HUDs.
*/
class CNOID_EXPORT SgOverlayPanel : public SgSpatialNode
{
public:
    enum ColorMode {
        FixedColor,
        RendererBackgroundColor
    };

    SgOverlayPanel();
    SgOverlayPanel(double width, double height);
    SgOverlayPanel(const SgOverlayPanel& org, CloneMap* cloneMap = nullptr);
    ~SgOverlayPanel();

    double width() const { return width_; }
    double height() const { return height_; }
    void setSize(double width, double height);

    ColorMode colorMode() const { return colorMode_; }
    void setColorMode(ColorMode mode) { colorMode_ = mode; }

    const Vector3f& color() const { return color_; }
    template<typename Derived> void setColor(const Eigen::MatrixBase<Derived>& c) {
        color_ = c.template cast<Vector3f::Scalar>();
    }

    float transparency() const { return transparency_; }
    void setTransparency(float transparency) { transparency_ = transparency; }

    virtual const BoundingBox& boundingBox() const override;
    virtual const BoundingBox& untransformedBoundingBox() const override;

protected:
    SgOverlayPanel(int classId);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    double width_;
    double height_;
    ColorMode colorMode_;
    Vector3f color_;
    float transparency_;
    mutable BoundingBox bboxCache_;
    mutable BoundingBox untransformedBboxCache;
};


/**
   SgHudOverlay places child nodes at logical-pixel offsets from one of the
   anchor positions of the viewport (corners, edge centers, or the center).
   The viewport is mapped to an orthographic projection in logical-pixel
   units so that text and other 2D elements appear as a head-up display
   fixed to the screen.
*/
class CNOID_EXPORT SgHudOverlay : public SgViewportOverlay
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

    SgHudOverlay();
    SgHudOverlay(const SgHudOverlay& org, CloneMap* cloneMap = nullptr);
    ~SgHudOverlay();

    /**
       Add a node to be displayed at the given anchor position with
       a logical-pixel offset. The node is wrapped in an internal SgPosTransform
       whose translation is updated whenever the viewport size changes.
       offsetX is positive toward the right; offsetY is positive toward
       the bottom.

       When anchor is TopRight, BottomRight, or RightCenter, the item's
       local origin (0, 0) is shifted left by width logical pixels so that
       a node whose extent is [0, width] appears flush against the right
       edge of the anchor. Set width to 0 (default) if the node does not
       need such right-alignment (its origin is placed exactly at the anchor).

       Returns true if the node was added.
    */
    bool addItem(SgNode* node, Anchor anchor, int offsetX, int offsetY, int width = 0);

    SgOverlayPanel* addPanel(
        double width, double height, Anchor anchor, int offsetX, int offsetY);

    bool removeItem(SgNode* node);

    void clearItems();

    virtual void calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume) override;

protected:
    SgHudOverlay(int classId);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<SgOverlay> SgOverlayPtr;
typedef ref_ptr<SgViewportOverlay> SgViewportOverlayPtr;
typedef ref_ptr<SgOverlayPanel> SgOverlayPanelPtr;
typedef ref_ptr<SgHudOverlay> SgHudOverlayPtr;

// Register overlay scene node classes to SceneNodeClassRegistry
void registerSceneOverlayNodeClasses();

}

#endif
