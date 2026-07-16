#include "SceneCollision.h"
#include "Body.h"
#include <cnoid/SceneNodeClassRegistry>
#include <cnoid/SceneRenderer>

using namespace std;
using namespace cnoid;

namespace {

struct NodeClassRegistration {
    NodeClassRegistration() {
        SceneNodeClassRegistry::instance().registerClass<SceneCollision, SgGroup>();

        SceneRenderer::addExtension(
            [](SceneRenderer* renderer){
                auto functions = renderer->renderingFunctions();
                functions->setFunction<SceneCollision>(
                    [renderer](SgNode* node){
                        static_cast<SceneCollision*>(node)->render(renderer);
                    });
            });
    }
} registration;

}


SceneCollision::SceneCollision(std::shared_ptr<std::vector<CollisionLinkPairPtr>> collisionPairs)
    : SgGroup(findClassId<SceneCollision>()),
      collisionPairs(collisionPairs)
{
    material = new SgMaterial;
    material->setDiffuseColor(Vector3f(0.0f, 1.0f, 0.0f));

    lineSet = new SgLineSet;
    lineVertices = lineSet->setVertices(new SgVertexArray);
    lineSet->setLineWidth(DefaultLineWidth);
    lineSet->setMaterial(material);
    addChild(lineSet);

    pointSet = new SgPointSet;
    pointVertices = pointSet->setVertices(new SgVertexArray);
    pointSet->setPointSize(DefaultPointSize);
    pointSet->setMaterial(material);

    /**
       The point markers are put in an overlay because a collision point is
       exactly on the body surfaces and the rendering of the markers is
       unstably dropped by the depth test with the surfaces without this.
    */
    pointOverlay = new SgOverlay;
    pointOverlay->addChild(pointSet);

    lineLengthRatio_ = DefaultLineLengthRatio;

    isPointMarkerEnabled_ = true;
    addChild(pointOverlay);

    isDirty = true;
}


SceneCollision::SceneCollision(const SceneCollision&)
{

}


void SceneCollision::setLineLengthRatio(double ratio)
{
    if(ratio != lineLengthRatio_){
        lineLengthRatio_ = ratio;
        notifyCollisionUpdate();
    }
}


float SceneCollision::lineWidth() const
{
    return lineSet->lineWidth();
}


void SceneCollision::setLineWidth(float width)
{
    if(width != lineSet->lineWidth()){
        lineSet->setLineWidth(width);
        notifyCollisionUpdate();
    }
}


void SceneCollision::setPointMarkerEnabled(bool on)
{
    if(on != isPointMarkerEnabled_){
        isPointMarkerEnabled_ = on;
        if(on){
            addChild(pointOverlay);
        } else {
            removeChild(pointOverlay);
        }
        notifyCollisionUpdate();
    }
}


double SceneCollision::pointSize() const
{
    return pointSet->pointSize();
}


void SceneCollision::setPointSize(double size)
{
    if(size != pointSet->pointSize()){
        pointSet->setPointSize(size);
        notifyCollisionUpdate();
    }
}


void SceneCollision::render(SceneRenderer* renderer)
{
    static const SceneRenderer::PropertyKey key("collisionLineRatio");
    static const SceneRenderer::PropertyKey colorKey("CollisionLineColor", 3);
    static const Vector3f defaultLineColor(0.0f, 1.0f, 0.0f);

    /**
       The collisionLineRatio property of the renderer is used only as the
       per-view visibility flag (zero or a positive value) here so that the
       existing visibility toggle of the scene bar keeps working. The actual
       line length is determined by the lineLengthRatio member.
    */
    if(renderer->property(key, 0.0) <= 0.0){
        return;
    }

    /**
       Modifying the geometries of the child nodes in the rendering
       traversal is not a usual operation, but the lazy construction here
       is intentional. The construction is skipped while the collision
       visualization is invisible in every view, which cannot be achieved
       by an eager construction, and when multiple views render the scene,
       only the first rendering after a collision update does the
       construction and the other views just reuse the vertices.
    */
    if(isDirty){
        lineVertices->clear();
        lineSet->lineVertexIndices().clear();
        pointVertices->clear();
        for(size_t i=0; i < collisionPairs->size(); ++i){
            const CollisionLinkPair& pair = *(*collisionPairs)[i];
            const auto& cols = pair.collisions();

            // flip the line direction so that the line is always from the staic object to the dynamic one
            double direction = 1.0;
            if(pair.body(1) && pair.body(0)){
                direction = (pair.body(1)->isStaticModel() && !pair.body(0)->isStaticModel()) ? -1.0 : 1.0;
            }

            for(size_t j=0; j < cols.size(); ++j){
                const Collision& c = cols[j];
                const Vector3f point = c.point.cast<float>();
                const int index = lineVertices->size();
                lineSet->addLine(index, index + 1);
                lineVertices->push_back(point);
                lineVertices->push_back((c.point + direction * lineLengthRatio_ * c.depth * c.normal).cast<float>());
                if(isPointMarkerEnabled_){
                    pointVertices->push_back(point);
                }
            }
        }
        lineVertices->notifyUpdate(sgUpdate);
        if(isPointMarkerEnabled_){
            pointVertices->notifyUpdate(sgUpdate);
        }
        isDirty = false;
    }

    Vector3f lineColor = renderer->property(colorKey, defaultLineColor);
    material->setDiffuseColor(lineColor);

    renderer->renderingFunctions()->dispatchAs<SgGroup>(this);
}
