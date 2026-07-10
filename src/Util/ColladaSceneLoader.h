#ifndef CNOID_UTIL_COLLADA_SCENE_LOADER_H
#define CNOID_UTIL_COLLADA_SCENE_LOADER_H

#include "AbstractSceneLoader.h"
#include "exportdecl.h"

namespace cnoid {

/**
   Scene loader for the COLLADA format (.dae).
   The loader directly parses the XML data using pugixml and does not depend on
   any external COLLADA library. It supports the basic elements used in robot and
   environment models: mesh geometries, node hierarchies, materials of the common
   profile, and diffuse textures. Animations, skinning, and physics are not supported.
*/
class CNOID_EXPORT ColladaSceneLoader : public AbstractSceneLoader
{
public:
    ColladaSceneLoader();
    ~ColladaSceneLoader();
    virtual void setMessageSink(std::ostream& os) override;
    virtual void setDefaultCreaseAngle(double theta) override;
    virtual void addImageSearchDirectory(const std::string& directory) override;
    virtual void clearImageSearchDirectories() override;
    virtual SgNode* load(const std::string& filename) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
