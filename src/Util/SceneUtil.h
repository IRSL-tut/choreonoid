/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SCENE_UTIL_H
#define CNOID_UTIL_SCENE_UTIL_H

#include "SceneGraph.h"
#include <utility>
#include "exportdecl.h"

namespace cnoid {

/**
   Creates a transform node hierarchy that represents the given affine transform,
   avoiding SgAffineTransform when possible. SgAffineTransform and negative scales
   break downstream subsystems that assume rotation-only transforms (renderer normal
   matrix, back-face culling, MeshExtractor winding, physics-engine collision-shape
   transforms), so scene loaders should use this function instead of creating
   SgAffineTransform directly.

   The transform is mapped to nodes as follows:
   - Identity: a plain SgGroup
   - Rotation and translation only: SgPosTransform
   - Including a scaling with a positive determinant: SgPosTransform + SgScaleTransform
     (with the scale orientation if necessary) obtained by the polar decomposition
   - Negative determinant (reflection) or a failed decomposition: SgAffineTransform
     as a defensive fallback. A reflection should normally be baked into mesh
     vertices beforehand (see bakeTransformIntoMesh in MeshUtil.h).

   @return The pair of the top node of the created hierarchy and the innermost group
   node to which child nodes should be added.
*/
CNOID_EXPORT std::pair<SgGroupPtr, SgGroupPtr> createTransformNodeSet(const Affine3& T);

/**
   Creates a copy of the scene in which the given transform and all the transform
   nodes contained in the scene are baked into the mesh vertices. This is used by
   the body loaders to remove a transform containing a reflection (e.g. a negative
   mesh scale of URDF / SDF) applied on top of a loaded mesh scene, which cannot be
   baked into the leaf meshes without accumulating the internal transforms as well.

   The node structure, names, and attributes are preserved; the transform nodes
   are kept but reset to the identity. The vertex and normal arrays are duplicated
   so that the original scene, which may be shared via a loader cache, is not
   affected. Nodes without geometry (e.g. lights and cameras) are shared with the
   original scene as they are.
*/
CNOID_EXPORT SgNodePtr createTransformBakedScene(SgNode* scene, const Affine3f& T);

CNOID_EXPORT Affine3 calcTotalTransform(const SgNodePath& path);
CNOID_EXPORT Affine3 calcTotalTransform(const SgNodePath& path, const SgNode* targetNode);
CNOID_EXPORT Affine3 calcTotalTransform(SgNodePath::const_iterator begin, SgNodePath::const_iterator end);

CNOID_EXPORT Isometry3 calcRelativePosition(const SgNodePath& path, const SgNode* targetNode);
CNOID_EXPORT Isometry3 calcRelativePosition(SgNodePath::const_iterator begin, SgNodePath::const_iterator end);

CNOID_EXPORT int makeTransparent(SgNode* topNode, float transparency, CloneMap& cloneMap, bool doKeepOrgTransparency = true);

}

#endif
