#ifndef CNOID_UTIL_GLTF_SCENE_LOADER_H
#define CNOID_UTIL_GLTF_SCENE_LOADER_H

#include "AbstractSceneLoader.h"
#include "exportdecl.h"

namespace cnoid {

/**
   Scene loader for the glTF 2.0 format (.gltf / .glb).
   The loader directly parses the JSON part using YAMLReader and does not
   depend on any external glTF library.
   The coordinate values are loaded as they are although the glTF specification
   defines the Y-up coordinate system. The conversion of the upper axis and the
   length unit is only applied when it is specified with the loader hints.
*/
class CNOID_EXPORT GLTFSceneLoader : public AbstractSceneLoader
{
public:
    GLTFSceneLoader();
    ~GLTFSceneLoader();
    virtual void setMessageSink(std::ostream& os) override;
    virtual void addImageSearchDirectory(const std::string& directory) override;
    virtual void clearImageSearchDirectories() override;
    virtual SgNode* load(const std::string& filename) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
