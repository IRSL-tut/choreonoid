/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneUtil.h"
#include "SceneDrawables.h"
#include "MeshUtil.h"
#include "PolymorphicSceneNodeFunctionSet.h"
#include "CloneMap.h"
#include "EigenUtil.h"
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

using namespace std;
using namespace cnoid;


std::pair<SgGroupPtr, SgGroupPtr> cnoid::createTransformNodeSet(const Affine3& T)
{
    if(T.matrix().isIdentity(1.0e-9)){
        SgGroupPtr group = new SgGroup;
        return { group, group };
    }

    const Matrix3 Q = T.linear();

    if(Q.determinant() > 0.0){

        if(Q.isUnitary(1.0e-6)){
            SgGroupPtr transform = new SgPosTransform(Isometry3(T.matrix()));
            return { transform, transform };
        }

        // Polar decomposition using SVD: Q = R * P, where R is a rotation and
        // P is a symmetric positive definite matrix. Since the determinant of Q
        // is positive, R is guaranteed to be a proper rotation.
        Eigen::JacobiSVD<Matrix3> svd(Q, Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Matrix3 R = svd.matrixU() * svd.matrixV().transpose();
        const Matrix3 P = svd.matrixV() * svd.singularValues().asDiagonal() * svd.matrixV().transpose();

        // Decompose P as SR * diag(scale) * SR^T by the eigenvalue decomposition,
        // which maps to SgScaleTransform with the scale orientation SR.
        Eigen::SelfAdjointEigenSolver<Matrix3> eigensolver(P);
        if(eigensolver.info() == Eigen::Success){
            const Vector3& scale = eigensolver.eigenvalues();
            SgScaleTransformPtr scaleTransform;
            if(!scale.isApprox(Vector3::Zero())){
                scaleTransform = new SgScaleTransform;
                scaleTransform->setScale(scale);
                Matrix3 SR = eigensolver.eigenvectors();
                if(SR.determinant() < 0.0){
                    // Flip the sign of one eigenvector to make SR a proper rotation
                    SR.col(0) = -SR.col(0);
                }
                if(!SR.isIdentity()){
                    scaleTransform->setScaleOrientation(SR);
                }
            }
            SgGroupPtr top;
            if(R.isIdentity() && T.translation().isZero()){
                top = scaleTransform;
            } else {
                SgPosTransformPtr posTransform = new SgPosTransform;
                posTransform->setRotation(R);
                posTransform->setTranslation(T.translation());
                if(scaleTransform){
                    posTransform->addChild(scaleTransform);
                }
                top = posTransform;
            }
            if(!top){
                top = new SgGroup;
            }
            SgGroupPtr bottom = scaleTransform ? SgGroupPtr(scaleTransform) : top;
            return { top, bottom };
        }
    }

    // A negative determinant (reflection) or a failed decomposition.
    // Reflections should normally be baked into descendant mesh vertices
    // beforehand so that this fallback is not reached.
    SgGroupPtr affine = new SgAffineTransform(T);
    return { affine, affine };
}


namespace {

SgNodePtr createBakedSceneNode(SgNode* node, const Affine3f& T)
{
    if(auto shape = dynamic_cast<SgShape*>(node)){
        SgShapePtr baked = new SgShape(*shape);
        if(auto mesh = shape->mesh()){
            SgMeshPtr bakedMesh = new SgMesh(*mesh);
            if(bakedMesh->hasVertices()){
                bakedMesh->setVertices(new SgVertexArray(*bakedMesh->vertices()));
            }
            if(bakedMesh->hasNormals()){
                bakedMesh->setNormals(new SgNormalArray(*bakedMesh->normals()));
            }
            bakeTransformIntoMesh(bakedMesh, T);
            baked->setMesh(bakedMesh);
        }
        return baked;
    }

    if(dynamic_cast<SgPlot*>(node)){
        SgPlotPtr baked;
        if(auto lineSet = dynamic_cast<SgLineSet*>(node)){
            baked = new SgLineSet(*lineSet);
        } else if(auto pointSet = dynamic_cast<SgPointSet*>(node)){
            baked = new SgPointSet(*pointSet);
        } else {
            return node; // An unknown plot type is kept as it is
        }
        if(baked->hasVertices()){
            auto bakedVertices = new SgVertexArray(*baked->vertices());
            transformVertices(*bakedVertices, T);
            baked->setVertices(bakedVertices);
        }
        if(baked->hasNormals()){
            auto bakedNormals = new SgNormalArray(*baked->normals());
            transformNormals(*bakedNormals, T);
            baked->setNormals(bakedNormals);
        }
        baked->invalidateBoundingBox();
        return baked;
    }

    if(auto group = node->toGroupNode()){
        Affine3f T_child = T;
        SgGroupPtr baked;
        if(auto transform = dynamic_cast<SgTransform*>(group)){
            Affine3 T_local;
            transform->getTransform(T_local);
            T_child = T * T_local.cast<float>();
            /*
               The transform node itself is copied and reset to the identity so that
               its name, URI, and the other attributes are preserved in the baked scene.
            */
            if(auto pos = dynamic_cast<SgPosTransform*>(transform)){
                auto copy = new SgPosTransform(*pos);
                copy->setPosition(Isometry3::Identity());
                baked = copy;
            } else if(auto scale = dynamic_cast<SgScaleTransform*>(transform)){
                auto copy = new SgScaleTransform(*scale);
                copy->setScale(1.0);
                copy->setScaleOrientation(Matrix3::Identity());
                baked = copy;
            } else if(auto affine = dynamic_cast<SgAffineTransform*>(transform)){
                auto copy = new SgAffineTransform(*affine);
                copy->setTransform(Affine3::Identity());
                baked = copy;
            } else {
                baked = new SgGroup;
                baked->setName(group->name());
            }
        } else {
            // The group node is copied to preserve its name and attributes.
            // A special group subtype is degraded to a plain group here.
            baked = new SgGroup(*group);
        }
        baked->clearChildren();
        for(int i = 0; i < group->numChildren(); ++i){
            if(auto bakedChild = createBakedSceneNode(group->child(i), T_child)){
                baked->addChild(bakedChild);
            }
        }
        return baked;
    }

    // Nodes without geometry (e.g. lights and cameras) are shared as they are
    return node;
}

}


SgNodePtr cnoid::createTransformBakedScene(SgNode* scene, const Affine3f& T)
{
    if(!scene){
        return nullptr;
    }
    return createBakedSceneNode(scene, T);
}

namespace {

template<class NodePathType>
void calcTotalTransform
(typename NodePathType::const_iterator begin, typename NodePathType::const_iterator end,
 const SgNode* targetNode, Affine3& out_T)
{
    out_T = Affine3::Identity();
    for(typename NodePathType::const_iterator p = begin; p != end; ++p){
        SgNode* node = *p;
        SgTransform* transform = dynamic_cast<SgTransform*>(node);
        if(transform){
            Affine3 T;
            transform->getTransform(T);
            out_T = out_T * T;
        }
        if(node == targetNode){
            break;
        }
    }
}

}
    

Affine3 cnoid::calcTotalTransform(const SgNodePath& path)
{
    Affine3 T;
    ::calcTotalTransform<SgNodePath>(path.begin(), path.end(), nullptr, T);
    return T;
}


Affine3 cnoid::calcTotalTransform(const SgNodePath& path, const SgNode* targetNode)
{
    Affine3 T;
    ::calcTotalTransform<SgNodePath>(path.begin(), path.end(), targetNode, T);
    return T;
}


Affine3 cnoid::calcTotalTransform(SgNodePath::const_iterator begin, SgNodePath::const_iterator end)
{
    Affine3 T;
    ::calcTotalTransform<SgNodePath>(begin, end, nullptr, T);
    return T;
}


Isometry3 cnoid::calcRelativePosition(const SgNodePath& path, const SgNode* targetNode)
{
    Affine3 T;
    ::calcTotalTransform<SgNodePath>(path.begin(), path.end(), targetNode, T);
    return convertToIsometryWithOrthonormalization(T);
}


Isometry3 cnoid::calcRelativePosition(SgNodePath::const_iterator begin, SgNodePath::const_iterator end)
{
    Affine3 T;
    ::calcTotalTransform<SgNodePath>(begin, end, nullptr, T);
    return convertToIsometryWithOrthonormalization(T);
}


namespace {

class Transparenter : PolymorphicSceneNodeFunctionSet
{
public:
    CloneMap& cloneMap;
    bool doKeepOrgTransparency;
    float transparency;
    int numModified;
    SgMaterialPtr defaultMaterial;

    Transparenter(CloneMap& cloneMap, bool doKeepOrgTransparency)
        : cloneMap(cloneMap),
          doKeepOrgTransparency(doKeepOrgTransparency)
    {
        setFunction<SgGroup>(
            [&](SgNode* node){ visitGroup(static_cast<SgGroup*>(node)); });
        setFunction<SgShape>(
            [&](SgNode* node){ visitShape(static_cast<SgShape*>(node)); });
        setFunction<SgPlot>(
            [&](SgNode* node){ visitPlot(static_cast<SgPlot*>(node)); });
        updateDispatchTable();
    }

    SgMaterial* getTransparentMaterial(SgMaterial* material)
    {
        SgMaterial* modified = cloneMap.getClone(material);
        if(doKeepOrgTransparency){
            modified->setTransparency(std::max(transparency, material->transparency()));
        } else {
            modified->setTransparency(transparency);
        }
        return modified;
    }
        
    void visitGroup(SgGroup* group)
    {
        for(SgGroup::const_iterator p = group->begin(); p != group->end(); ++p){
            dispatch(*p);
        }
    }
    
    void visitShape(SgShape* shape)
    {
        if(shape->material()){
            shape->setMaterial(getTransparentMaterial(shape->material()));
        } else {
            if(!defaultMaterial){
                defaultMaterial = new SgMaterial;
                defaultMaterial->setTransparency(transparency);
            }
            shape->setMaterial(defaultMaterial);
        }
        ++numModified;
    }
                
    void visitPlot(SgPlot* plot)
    {
        if(plot->material()){
            plot->setMaterial(getTransparentMaterial(plot->material()));
            ++numModified;
        }
    }

    int apply(SgNode* topNode, float transparency)
    {
        this->transparency = transparency;
        numModified = 0;
        dispatch(topNode);
        return numModified;
    }
};

}


int cnoid::makeTransparent(SgNode* topNode, float transparency, CloneMap& cloneMap, bool doKeepOrgTransparency)
{
    if(topNode){
        Transparenter transparenter(cloneMap, doKeepOrgTransparency);
        return transparenter.apply(topNode, transparency);
    }
    return 0;
}
