#ifndef CNOID_UTIL_GLTF_SCENE_WRITER_H
#define CNOID_UTIL_GLTF_SCENE_WRITER_H

#include "AbstractSceneWriter.h"
#include "exportdecl.h"

namespace cnoid {

/**
   Scene writer for the glTF 2.0 format.
   The output format is determined by the extension of the output filename;
   ".glb" gives a single binary file, and ".gltf" gives a JSON file with
   an accompanying ".bin" file that stores the binary data.
   Texture images are embedded in the binary data in the GLB format, while
   they are output as external image files referenced by URIs in the ".gltf"
   format. In the latter case, an image file is shared by all the glTF files
   which are output into the same directory and use the same original image.
   The coordinate values are output as they are although the glTF specification
   defines the Y-up coordinate system.
*/
class CNOID_EXPORT GLTFSceneWriter : public AbstractSceneWriter
{
public:
    GLTFSceneWriter();
    ~GLTFSceneWriter();

    GLTFSceneWriter(const GLTFSceneWriter&) = delete;
    GLTFSceneWriter(GLTFSceneWriter&&) = delete;
    GLTFSceneWriter& operator=(const GLTFSceneWriter&) = delete;
    GLTFSceneWriter& operator=(GLTFSceneWriter&&) = delete;

    virtual void setMessageSink(std::ostream& os) override;

    /**
       Materials and textures are output when this is enabled (default).
       Disable this to output only the geometries.
    */
    void setMaterialEnabled(bool on);
    bool isMaterialEnabled() const;

    virtual bool writeScene(const std::string& filename, SgNode* node) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
