#ifndef CNOID_BODY_SCENE_COLLISION_H
#define CNOID_BODY_SCENE_COLLISION_H

#include "CollisionLinkPair.h"
#include <cnoid/SceneDrawables>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class SceneRenderer;

class CNOID_EXPORT SceneCollision : public SgLineSet
{
public:
    typedef std::shared_ptr<CollisionLinkPair> CollisionLinkPairPtr;
    
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

    void render(SceneRenderer* renderer);

private:
    SceneCollision(const SceneCollision& org);

    std::shared_ptr<std::vector<CollisionLinkPairPtr>> collisionPairs;
    SgVertexArrayPtr vertices_;
    SgUpdate sgUpdate{ SgUpdate::GeometryModified };
    bool isDirty;
};
    
typedef ref_ptr<SceneCollision> SceneCollisionPtr;

}

#endif
