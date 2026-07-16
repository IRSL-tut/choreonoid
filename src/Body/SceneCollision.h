#ifndef CNOID_BODY_SCENE_COLLISION_H
#define CNOID_BODY_SCENE_COLLISION_H

#include "CollisionLinkPair.h"
#include <cnoid/SceneDrawables>
#include <cnoid/SceneOverlay>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class SceneRenderer;

class CNOID_EXPORT SceneCollision : public SgGroup
{
public:
    typedef std::shared_ptr<CollisionLinkPair> CollisionLinkPairPtr;

    static constexpr double DefaultLineLengthRatio = 100.0;
    static constexpr float DefaultLineWidth = 1.0f;
    static constexpr double DefaultPointSize = 6.0;

    SceneCollision(std::shared_ptr<std::vector<CollisionLinkPairPtr>> collisionPairs);
    void setDirty() { isDirty = true; }

    /**
       Mark the collision data changed and notify the scene of the update
       so that the collision lines are updated in the next rendering.
    */
    void notifyCollisionUpdate(){
        isDirty = true;
        notifyUpdate(sgUpdate);
    }

    //! The ratio of the collision line length to the penetration depth
    double lineLengthRatio() const { return lineLengthRatio_; }
    void setLineLengthRatio(double ratio);

    //! The width of the collision lines in pixels
    float lineWidth() const;
    void setLineWidth(float width);

    //! Whether the point markers are rendered at the collision points
    bool isPointMarkerEnabled() const { return isPointMarkerEnabled_; }
    void setPointMarkerEnabled(bool on);

    //! The size of the collision point markers in pixels
    double pointSize() const;
    void setPointSize(double size);

    void render(SceneRenderer* renderer);

private:
    SceneCollision(const SceneCollision& org);

    std::shared_ptr<std::vector<CollisionLinkPairPtr>> collisionPairs;
    SgLineSetPtr lineSet;
    SgOverlayPtr pointOverlay;
    SgPointSetPtr pointSet;
    SgVertexArrayPtr lineVertices;
    SgVertexArrayPtr pointVertices;
    SgMaterialPtr material;
    SgUpdate sgUpdate{ SgUpdate::GeometryModified };
    double lineLengthRatio_;
    bool isPointMarkerEnabled_;
    bool isDirty;
};

typedef ref_ptr<SceneCollision> SceneCollisionPtr;

}

#endif
