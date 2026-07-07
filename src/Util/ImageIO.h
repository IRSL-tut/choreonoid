#ifndef CNOID_UTIL_IMAGE_IO_H
#define CNOID_UTIL_IMAGE_IO_H

#include "Image.h"
#include "NullOut.h"
#include "exportdecl.h"

namespace cnoid {

class MessageOut;

class CNOID_EXPORT ImageIO
{
public:
    ImageIO();

    void setUpsideDown(bool on) { isUpsideDown_ = on; }

    //! \todo implement this mode.
    void allocateAlphaComponent(bool on);

    bool load(Image& image, const std::string& filename, std::ostream& os = nullout());
    bool save(const Image& image, const std::string& filename, std::ostream& os = nullout());

    /**
       Load an image from the image file data on memory. The image file format is
       automatically detected from the data. Note that the error messages output
       by this function do not include the information on the image data source,
       so the caller should supplement the context if necessary.
       \param data The image file data such as the content of a PNG or JPEG file
       \param size The size of the data in bytes
       \param mout The sink of error messages. No messages are output if this is nullptr.
    */
    bool load(Image& image, const void* data, size_t size, MessageOut* mout = nullptr);

private:
    bool isUpsideDown_;
};

}

#endif
