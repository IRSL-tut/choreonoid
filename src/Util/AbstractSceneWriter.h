#ifndef CNOID_UTIL_ABSTRACT_SCENE_WRITER_H
#define CNOID_UTIL_ABSTRACT_SCENE_WRITER_H

#include <string>
#include <ostream>
#include "exportdecl.h"

namespace cnoid {

class SgNode;
class SgImage;

class CNOID_EXPORT AbstractSceneWriter
{
public:
    AbstractSceneWriter();
    virtual ~AbstractSceneWriter();
    virtual void setMessageSink(std::ostream& os);
    virtual bool writeScene(const std::string& filename, SgNode* node) = 0;

protected:
    void clearImageFileInformation();

    /**
       Makes the image available as an image file which can be referred to from the files
       output by the writer. If the image has the uri and the original image file is found,
       the file is used as it is or copied into the output directory by findOrCopyImageFile.
       Otherwise, the image is newly written into the output directory using the source
       data attached to the image or by encoding the image pixels into the PNG format.
       \param out_imageFile The image file path relative to the output directory, or the
       absolute path of the original image file when it is out of the output directory.
    */
    bool outputImageFile(SgImage* image, const std::string& outputBaseDir, std::string& out_imageFile);

    bool findOrCopyImageFile(SgImage* image, const std::string& outputBaseDir, std::string& out_copiedFile);
    std::ostream& os(){ return *os_; }

private:
    std::ostream* os_;

    class Impl;
    Impl* impl;
};

}

#endif
